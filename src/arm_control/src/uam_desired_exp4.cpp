#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <boost/bind.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include "uam_message/arm_angle.h"
#include "uav/desired_start.h"
#include "uav/xyz_yaw_d.h"

namespace {

bool start_flag = false;

struct Vec3 {
    double x;
    double y;
    double z;
};

struct BasePoseState {
    bool valid = false;
    Vec3 position{0.0, 0.0, 0.0};
    double yaw_rad = 0.0;
};

struct ArmState {
    bool valid = false;
    double arm1_deg = 0.0;
    double arm2_deg = 0.0;
    double hand_deg = 0.0;
};

struct RingPoseState {
    bool valid = false;
    ros::Time stamp;
    Vec3 position{0.0, 0.0, 0.0};
};

struct RingConfig {
    std::string name;
    Vec3 approach_offset{0.20, 0.0, 0.0};
    Vec3 depart_offset{-0.20, 0.0, 0.0};
};

enum Stage {
    STAGE_SETTLE = 0,
    STAGE_APPROACH = 1,
    STAGE_PASS = 2,
    STAGE_DEPART = 3,
    STAGE_RECOVERY = 4,
    STAGE_LANDING = 5,
};

BasePoseState g_base_pose;
ArmState g_real_arm_state;
std::vector<RingPoseState> g_ring_poses;

struct MavrosStateCache {
    bool valid = false;
    bool connected = false;
    bool armed = false;
    std::string mode;
};

MavrosStateCache g_mavros_state;

double Clamp(double value, double min_value, double max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

double DegToRad(double value_deg) {
    return value_deg * std::acos(-1.0) / 180.0;
}

double RadToDeg(double value_rad) {
    return value_rad * 180.0 / std::acos(-1.0);
}

double WrapToPi(double angle_rad) {
    const double two_pi = 2.0 * std::acos(-1.0);
    while (angle_rad > std::acos(-1.0)) {
        angle_rad -= two_pi;
    }
    while (angle_rad < -std::acos(-1.0)) {
        angle_rad += two_pi;
    }
    return angle_rad;
}

double AdjustNearReference(double angle_rad, double reference_rad) {
    double adjusted = angle_rad;
    const double two_pi = 2.0 * std::acos(-1.0);
    while (adjusted - reference_rad > std::acos(-1.0)) {
        adjusted -= two_pi;
    }
    while (adjusted - reference_rad < -std::acos(-1.0)) {
        adjusted += two_pi;
    }
    return adjusted;
}

double AngularDistance(double angle_a_rad, double angle_b_rad) {
    return std::fabs(WrapToPi(angle_a_rad - angle_b_rad));
}

double QuaternionToYaw(double x, double y, double z, double w) {
    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    return std::atan2(siny_cosp, cosy_cosp);
}

Vec3 AddVec3(const Vec3 &lhs, const Vec3 &rhs) {
    return {lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

Vec3 SubVec3(const Vec3 &lhs, const Vec3 &rhs) {
    return {lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

Vec3 ScaleVec3(const Vec3 &value, double scale) {
    return {value.x * scale, value.y * scale, value.z * scale};
}

Vec3 InterpolateVec3(const Vec3 &start, const Vec3 &end, double ratio) {
    return AddVec3(start, ScaleVec3(SubVec3(end, start), ratio));
}

double NormVec3(const Vec3 &value) {
    return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}

Vec3 RotateYawOnly(const Vec3 &vector_body, double yaw_rad) {
    const double cos_yaw = std::cos(yaw_rad);
    const double sin_yaw = std::sin(yaw_rad);
    return {cos_yaw * vector_body.x - sin_yaw * vector_body.y,
            sin_yaw * vector_body.x + cos_yaw * vector_body.y,
            vector_body.z};
}

Vec3 InverseRotateYawOnly(const Vec3 &vector_world, double yaw_rad) {
    return RotateYawOnly(vector_world, -yaw_rad);
}

Vec3 ForwardKinematicsBody(double arm1_deg, double arm2_deg) {
    const double q1 = DegToRad(arm1_deg);
    const double q2 = DegToRad(arm2_deg);
    return {
        0.104 + 0.21 * std::cos(q1) * std::cos(q2) - 0.007 * std::sin(q1),
        -0.21 * std::sin(q1) * std::cos(q2) - 0.007 * std::cos(q1),
        0.04894 + 0.21 * std::sin(q2)};
}

Vec3 CurrentEndEffectorWorld(double arm1_deg, double arm2_deg) {
    const Vec3 ee_body = ForwardKinematicsBody(arm1_deg, arm2_deg);
    const Vec3 ee_offset_world = RotateYawOnly(ee_body, g_base_pose.yaw_rad);
    return AddVec3(g_base_pose.position, ee_offset_world);
}

bool SolveInverseKinematics(const Vec3 &target_body,
                            double previous_arm1_deg,
                            double previous_arm2_deg,
                            double arm1_min_deg,
                            double arm1_max_deg,
                            double arm2_min_deg,
                            double arm2_max_deg,
                            double *arm1_deg_out,
                            double *arm2_deg_out) {
    const double z_normalized = (target_body.z - 0.04894) / 0.21;
    if (std::fabs(z_normalized) > 1.0) {
        return false;
    }

    const double q1_reference = DegToRad(previous_arm1_deg);
    const double q2_reference = DegToRad(previous_arm2_deg);
    const double asin_q2 = std::asin(Clamp(z_normalized, -1.0, 1.0));
    std::vector<double> q2_candidates = {asin_q2, std::acos(-1.0) - asin_q2};

    double best_cost = std::numeric_limits<double>::infinity();
    bool found = false;
    double best_arm1_deg = previous_arm1_deg;
    double best_arm2_deg = previous_arm2_deg;

    for (double q2_candidate : q2_candidates) {
        q2_candidate = AdjustNearReference(q2_candidate, q2_reference);

        const double a = 0.21 * std::cos(q2_candidate);
        const double q1_candidate =
            AdjustNearReference(std::atan2(-target_body.y, target_body.x - 0.104) -
                                    std::atan2(0.007, a),
                                q1_reference);

        const double arm1_deg = RadToDeg(q1_candidate);
        const double arm2_deg = RadToDeg(q2_candidate);
        if (arm1_deg < arm1_min_deg || arm1_deg > arm1_max_deg) {
            continue;
        }
        if (arm2_deg < arm2_min_deg || arm2_deg > arm2_max_deg) {
            continue;
        }

        const Vec3 reconstructed = ForwardKinematicsBody(arm1_deg, arm2_deg);
        const Vec3 error = SubVec3(reconstructed, target_body);
        const double position_error = NormVec3(error);
        const double continuity_cost =
            AngularDistance(q1_candidate, q1_reference) + AngularDistance(q2_candidate, q2_reference);
        const double total_cost = position_error * 100.0 + continuity_cost;
        if (total_cost < best_cost) {
            best_cost = total_cost;
            best_arm1_deg = arm1_deg;
            best_arm2_deg = arm2_deg;
            found = true;
        }
    }

    if (!found) {
        return false;
    }

    *arm1_deg_out = best_arm1_deg;
    *arm2_deg_out = best_arm2_deg;
    return true;
}

double QuinticBlend(double ratio) {
    const double x = Clamp(ratio, 0.0, 1.0);
    return x * x * x * (10.0 + x * (-15.0 + 6.0 * x));
}

const char *StageName(Stage stage) {
    switch (stage) {
        case STAGE_SETTLE:
            return "settle";
        case STAGE_APPROACH:
            return "approach";
        case STAGE_PASS:
            return "pass";
        case STAGE_DEPART:
            return "depart";
        case STAGE_RECOVERY:
            return "recovery_hover";
        case STAGE_LANDING:
            return "landing";
        default:
            return "unknown";
    }
}

bool IsRingFresh(std::size_t index, const ros::Time &now, double timeout_sec) {
    if (index >= g_ring_poses.size()) {
        return false;
    }
    if (!g_ring_poses[index].valid) {
        return false;
    }
    return (now - g_ring_poses[index].stamp).toSec() <= timeout_sec;
}

bool AreAllRingsFresh(const ros::Time &now, double timeout_sec) {
    for (std::size_t index = 0; index < g_ring_poses.size(); ++index) {
        if (!IsRingFresh(index, now, timeout_sec)) {
            return false;
        }
    }
    return !g_ring_poses.empty();
}

bool LoadStringVectorParam(const ros::NodeHandle &nh,
                           const std::string &key,
                           const std::vector<std::string> &fallback,
                           std::vector<std::string> *output) {
    XmlRpc::XmlRpcValue xml_value;
    if (!nh.getParam(key, xml_value) || xml_value.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        *output = fallback;
        return false;
    }

    output->clear();
    for (int index = 0; index < xml_value.size(); ++index) {
        if (xml_value[index].getType() != XmlRpc::XmlRpcValue::TypeString) {
            *output = fallback;
            return false;
        }
        output->push_back(static_cast<std::string>(xml_value[index]));
    }
    if (output->empty()) {
        *output = fallback;
        return false;
    }
    return true;
}

bool LoadVec3Param(const ros::NodeHandle &nh, const std::string &key, const Vec3 &fallback, Vec3 *output) {
    XmlRpc::XmlRpcValue xml_value;
    if (!nh.getParam(key, xml_value) || xml_value.getType() != XmlRpc::XmlRpcValue::TypeArray ||
        xml_value.size() != 3) {
        *output = fallback;
        return false;
    }

    Vec3 value = fallback;
    for (int index = 0; index < 3; ++index) {
        if (xml_value[index].getType() != XmlRpc::XmlRpcValue::TypeDouble &&
            xml_value[index].getType() != XmlRpc::XmlRpcValue::TypeInt) {
            *output = fallback;
            return false;
        }
        const double parsed = xml_value[index].getType() == XmlRpc::XmlRpcValue::TypeInt
                                  ? static_cast<int>(xml_value[index])
                                  : static_cast<double>(xml_value[index]);
        if (index == 0) {
            value.x = parsed;
        } else if (index == 1) {
            value.y = parsed;
        } else {
            value.z = parsed;
        }
    }

    *output = value;
    return true;
}

Vec3 LimitBaseReferenceStep(const uav::xyz_yaw_d &previous_command,
                            const Vec3 &desired_reference,
                            double max_xy_step,
                            double max_z_step) {
    Vec3 limited = desired_reference;

    if (max_xy_step > 0.0) {
        const double dx = desired_reference.x - previous_command.x_d;
        const double dy = desired_reference.y - previous_command.y_d;
        const double xy_norm = std::sqrt(dx * dx + dy * dy);
        if (xy_norm > max_xy_step && xy_norm > 1e-6) {
            const double scale = max_xy_step / xy_norm;
            limited.x = previous_command.x_d + dx * scale;
            limited.y = previous_command.y_d + dy * scale;
        }
    }

    if (max_z_step > 0.0) {
        const double dz = desired_reference.z - previous_command.z_d;
        limited.z = previous_command.z_d + Clamp(dz, -max_z_step, max_z_step);
    }

    return limited;
}

bool doReq(uav::desired_start::Request &req, uav::desired_start::Response &resp) {
    if (req.desired_start != 10) {
        ROS_ERROR("提交的数据异常!!!");
        return false;
    }
    resp.desired_sent = 3;
    start_flag = true;
    return true;
}

void BasePoseCb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    g_base_pose.valid = true;
    g_base_pose.position = {msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
    g_base_pose.yaw_rad = QuaternionToYaw(msg->pose.orientation.x,
                                          msg->pose.orientation.y,
                                          msg->pose.orientation.z,
                                          msg->pose.orientation.w);
}

void ArmRealCb(const uam_message::arm_angle::ConstPtr &msg) {
    g_real_arm_state.valid = true;
    g_real_arm_state.arm1_deg = msg->arm1_angle;
    g_real_arm_state.arm2_deg = msg->arm2_angle;
    g_real_arm_state.hand_deg = msg->hand_angle;
}

void MavrosStateCb(const mavros_msgs::State::ConstPtr &msg) {
    g_mavros_state.valid = true;
    g_mavros_state.connected = msg->connected;
    g_mavros_state.armed = msg->armed;
    g_mavros_state.mode = msg->mode;
}

void RingPoseCb(const geometry_msgs::PoseStamped::ConstPtr &msg, std::size_t index) {
    if (index >= g_ring_poses.size()) {
        return;
    }
    g_ring_poses[index].valid = true;
    g_ring_poses[index].stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    g_ring_poses[index].position = {msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
}

}  // namespace

int main(int argc, char *argv[]) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "uam_desired_exp4");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ros::Publisher joint_angle_pub =
        nh.advertise<uam_message::arm_angle>("/wjl/arm/guidefly/angle_d", 10);
    ros::Publisher uav_pos_d_pub =
        nh.advertise<uav::xyz_yaw_d>("/wjl/guidefly/pose_d", 10);
    ros::ServiceServer server = nh.advertiseService("/wjl/start/uav_desired", doReq);

    double hover_z = 1.0;
    double hover_yaw_deg = 0.0;
    double settle_time = 3.0;
    double recovery_time = 3.0;
    double landing_z = 0.5;
    double approach_time = 3.0;
    double pass_time = 2.0;
    double depart_time = 3.0;
    double ee_nominal_x = 0.18;
    double ee_nominal_y = 0.0;
    double ee_nominal_z = 0.06;
    double ring_pose_timeout_sec = 0.3;
    int ring_loss_limit = 10;
    int ik_fail_limit = 10;
    double base_step_limit_xy = 0.03;
    double base_step_limit_z = 0.02;
    double hand_hold_deg = 25.0;
    std::string base_pose_topic = "/mavros/local_position/pose";
    Vec3 final_hover_offset{-0.30, 0.0, 0.0};

    std::vector<std::string> ring_names = {"ring1", "ring2", "ring3", "ring4"};
    std::vector<std::string> configured_ring_names;
    LoadStringVectorParam(pnh, "ring_names", ring_names, &configured_ring_names);
    ring_names = configured_ring_names;

    pnh.param("hover_z", hover_z, hover_z);
    pnh.param("hover_yaw_deg", hover_yaw_deg, hover_yaw_deg);
    pnh.param("settle_time", settle_time, settle_time);
    pnh.param("recovery_time", recovery_time, recovery_time);
    pnh.param("landing_z", landing_z, landing_z);
    pnh.param("approach_time", approach_time, approach_time);
    pnh.param("pass_time", pass_time, pass_time);
    pnh.param("depart_time", depart_time, depart_time);
    pnh.param("ee_nominal_x", ee_nominal_x, ee_nominal_x);
    pnh.param("ee_nominal_y", ee_nominal_y, ee_nominal_y);
    pnh.param("ee_nominal_z", ee_nominal_z, ee_nominal_z);
    pnh.param("ring_pose_timeout_sec", ring_pose_timeout_sec, ring_pose_timeout_sec);
    pnh.param("ring_loss_limit", ring_loss_limit, ring_loss_limit);
    pnh.param("ik_fail_limit", ik_fail_limit, ik_fail_limit);
    pnh.param("base_step_limit_xy", base_step_limit_xy, base_step_limit_xy);
    pnh.param("base_step_limit_z", base_step_limit_z, base_step_limit_z);
    pnh.param("hand_hold_deg", hand_hold_deg, hand_hold_deg);
    pnh.param("base_pose_topic", base_pose_topic, base_pose_topic);
    LoadVec3Param(pnh, "final_hover_offset_xyz", final_hover_offset, &final_hover_offset);

    if (settle_time < 0.0) {
        settle_time = 0.0;
    }
    if (recovery_time < 0.0) {
        recovery_time = 0.0;
    }
    if (approach_time <= 0.0) {
        approach_time = 3.0;
    }
    if (pass_time <= 0.0) {
        pass_time = 2.0;
    }
    if (depart_time <= 0.0) {
        depart_time = 3.0;
    }
    if (ring_pose_timeout_sec <= 0.0) {
        ring_pose_timeout_sec = 0.3;
    }
    if (ring_loss_limit < 1) {
        ring_loss_limit = 1;
    }
    if (ik_fail_limit < 1) {
        ik_fail_limit = 1;
    }
    if (ring_names.empty()) {
        ring_names = {"ring1", "ring2", "ring3", "ring4"};
    }

    std::vector<RingConfig> ring_configs(ring_names.size());
    for (std::size_t index = 0; index < ring_names.size(); ++index) {
        ring_configs[index].name = ring_names[index];
        const std::string prefix = "rings/" + ring_names[index] + "/";
        LoadVec3Param(pnh, prefix + "approach_offset_xyz", ring_configs[index].approach_offset,
                      &ring_configs[index].approach_offset);
        LoadVec3Param(pnh, prefix + "depart_offset_xyz", ring_configs[index].depart_offset,
                      &ring_configs[index].depart_offset);
    }

    double arm1_min = -180.0;
    double arm1_max = 180.0;
    double arm2_min = -180.0;
    double arm2_max = 60.0;
    double hand_min = -15.0;
    double hand_max = 25.0;
    nh.param("/arm/arm_joint1/min", arm1_min, arm1_min);
    nh.param("/arm/arm_joint1/max", arm1_max, arm1_max);
    nh.param("/arm/arm_joint2/min", arm2_min, arm2_min);
    nh.param("/arm/arm_joint2/max", arm2_max, arm2_max);
    nh.param("/arm/left_hand_joint/min", hand_min, hand_min);
    nh.param("/arm/left_hand_joint/max", hand_max, hand_max);

    hand_hold_deg = Clamp(hand_hold_deg, hand_min, hand_max);

    ros::Subscriber base_pose_sub =
        nh.subscribe<geometry_msgs::PoseStamped>(base_pose_topic, 10, BasePoseCb);
    ros::Subscriber arm_real_sub =
        nh.subscribe<uam_message::arm_angle>("/wjl/arm/real/angle_r", 10, ArmRealCb);
    ros::Subscriber mavros_state_sub =
        nh.subscribe<mavros_msgs::State>("/mavros/state", 10, MavrosStateCb);

    g_ring_poses.assign(ring_names.size(), RingPoseState());
    std::vector<ros::Subscriber> ring_pose_subs;
    ring_pose_subs.reserve(ring_names.size());
    for (std::size_t index = 0; index < ring_names.size(); ++index) {
        const std::string topic = "/vrpn_client_node/" + ring_names[index] + "/pose";
        ring_pose_subs.push_back(
            nh.subscribe<geometry_msgs::PoseStamped>(topic, 10, boost::bind(&RingPoseCb, _1, index)));
    }

    ROS_INFO("exp4 service ready, waiting for /wjl/start/uav_desired");
    ROS_INFO(
        "exp4 params: hover_z=%.2f, yaw=%.2f deg, settle=%.2f s, recovery=%.2f s, landing_z=%.2f, segment_time=(%.2f, %.2f, %.2f), nominal_ee=(%.3f, %.3f, %.3f), hand_hold=%.2f",
        hover_z, hover_yaw_deg, settle_time, recovery_time, landing_z, approach_time, pass_time,
        depart_time, ee_nominal_x, ee_nominal_y, ee_nominal_z, hand_hold_deg);
    for (std::size_t index = 0; index < ring_configs.size(); ++index) {
        ROS_INFO("exp4 ring[%zu]=%s, approach=(%.3f, %.3f, %.3f), depart=(%.3f, %.3f, %.3f)",
                 index, ring_configs[index].name.c_str(), ring_configs[index].approach_offset.x,
                 ring_configs[index].approach_offset.y, ring_configs[index].approach_offset.z,
                 ring_configs[index].depart_offset.x, ring_configs[index].depart_offset.y,
                 ring_configs[index].depart_offset.z);
    }

    ros::Rate rate(30.0);
    while (ros::ok() && !start_flag) {
        ros::spinOnce();
        rate.sleep();
    }
    if (!ros::ok()) {
        return 0;
    }

    const ros::Duration return_home_publish_time(1.0);
    auto PublishReturnHome = [&](double home_arm1_deg, double home_arm2_deg, double home_hand_deg) {
        if (!ros::ok()) {
            return;
        }
        uam_message::arm_angle home_angle;
        home_angle.arm1_angle = Clamp(home_arm1_deg, arm1_min, arm1_max);
        home_angle.arm2_angle = Clamp(home_arm2_deg, arm2_min, arm2_max);
        home_angle.hand_angle = Clamp(home_hand_deg, hand_min, hand_max);
        ROS_INFO("exp4 return arm to home: arm_d=(%.2f, %.2f, %.2f)",
                 home_angle.arm1_angle, home_angle.arm2_angle, home_angle.hand_angle);
        ros::Rate return_rate(30.0);
        const ros::Time return_start = ros::Time::now();
        while (ros::ok() && (ros::Time::now() - return_start) < return_home_publish_time) {
            joint_angle_pub.publish(home_angle);
            ros::spinOnce();
            return_rate.sleep();
        }
    };

    const double hover_yaw_rad = DegToRad(hover_yaw_deg);
    const Vec3 nominal_ee_body{ee_nominal_x, ee_nominal_y, ee_nominal_z};
    const Vec3 nominal_ee_world = RotateYawOnly(nominal_ee_body, hover_yaw_rad);

    uav::xyz_yaw_d uav_pos_d;
    uav_pos_d.x_d = 0.0;
    uav_pos_d.y_d = 0.0;
    uav_pos_d.z_d = hover_z;
    uav_pos_d.yaw_d = hover_yaw_deg;
    uav_pos_d.land_flag = false;

    uam_message::arm_angle current_angle;
    current_angle.arm1_angle = g_real_arm_state.valid ? g_real_arm_state.arm1_deg : 0.0;
    current_angle.arm2_angle = g_real_arm_state.valid ? g_real_arm_state.arm2_deg : 0.0;
    current_angle.hand_angle = hand_hold_deg;
    current_angle.arm1_angle = Clamp(current_angle.arm1_angle, arm1_min, arm1_max);
    current_angle.arm2_angle = Clamp(current_angle.arm2_angle, arm2_min, arm2_max);

    double last_valid_arm1_deg = current_angle.arm1_angle;
    double last_valid_arm2_deg = current_angle.arm2_angle;

    bool hover_anchor_initialized = false;
    Vec3 hover_anchor{0.0, 0.0, hover_z};
    Vec3 segment_start_ee_world{0.0, 0.0, hover_z};
    bool segment_start_valid = false;
    Vec3 last_desired_ee_world{0.0, 0.0, hover_z};
    bool last_desired_ee_world_valid = false;
    Stage stage = STAGE_SETTLE;
    int active_ring_index = 0;
    int last_logged_ring_index = -1;
    Stage last_logged_stage = static_cast<Stage>(-1);
    double stage_elapsed = 0.0;
    int ring_loss_count = 0;
    int ik_fail_count = 0;
    bool force_recovery = false;
    uav::xyz_yaw_d last_safe_uav_pos_d = uav_pos_d;

    const ros::Time experiment_start = ros::Time::now();
    ros::Time last_loop_time = experiment_start;
    ros::Time last_log_time = experiment_start - ros::Duration(1.0);

    auto capture_segment_start = [&]() {
        if (last_desired_ee_world_valid) {
            segment_start_ee_world = last_desired_ee_world;
            segment_start_valid = true;
            return;
        }
        if (g_base_pose.valid) {
            const double seed_arm1 = g_real_arm_state.valid ? g_real_arm_state.arm1_deg : last_valid_arm1_deg;
            const double seed_arm2 = g_real_arm_state.valid ? g_real_arm_state.arm2_deg : last_valid_arm2_deg;
            segment_start_ee_world = CurrentEndEffectorWorld(seed_arm1, seed_arm2);
            segment_start_valid = true;
            return;
        }
        segment_start_valid = false;
    };

    auto transition_to = [&](Stage next_stage, int next_ring_index) {
        stage = next_stage;
        active_ring_index = next_ring_index;
        stage_elapsed = 0.0;
        if (next_stage == STAGE_APPROACH || next_stage == STAGE_PASS || next_stage == STAGE_DEPART ||
            next_stage == STAGE_RECOVERY) {
            capture_segment_start();
        } else {
            segment_start_valid = false;
        }
    };

    auto stage_duration = [&](Stage query_stage) {
        if (query_stage == STAGE_APPROACH) {
            return approach_time;
        }
        if (query_stage == STAGE_PASS) {
            return pass_time;
        }
        if (query_stage == STAGE_DEPART) {
            return depart_time;
        }
        if (query_stage == STAGE_RECOVERY) {
            return recovery_time;
        }
        if (query_stage == STAGE_LANDING) {
            return 0.5;
        }
        return settle_time;
    };

    while (ros::ok()) {
        ros::spinOnce();

        const ros::Time now = ros::Time::now();
        const double dt = std::max(0.0, (now - last_loop_time).toSec());
        last_loop_time = now;
        const double elapsed = (now - experiment_start).toSec();

        if (!hover_anchor_initialized && g_base_pose.valid) {
            hover_anchor = g_base_pose.position;
            hover_anchor.z = hover_z;
            hover_anchor_initialized = true;
            uav_pos_d.x_d = hover_anchor.x;
            uav_pos_d.y_d = hover_anchor.y;
            uav_pos_d.z_d = hover_anchor.z;
            last_safe_uav_pos_d = uav_pos_d;
        }

        if (force_recovery && stage != STAGE_RECOVERY && stage != STAGE_LANDING) {
            transition_to(STAGE_RECOVERY, static_cast<int>(ring_configs.size()) - 1);
        }

        if (stage != last_logged_stage || active_ring_index != last_logged_ring_index) {
            if (stage == STAGE_APPROACH || stage == STAGE_PASS || stage == STAGE_DEPART) {
                ROS_INFO("exp4 stage -> %s_%s", StageName(stage),
                         ring_configs[static_cast<std::size_t>(active_ring_index)].name.c_str());
            } else {
                ROS_INFO("exp4 stage -> %s", StageName(stage));
            }
            last_logged_stage = stage;
            last_logged_ring_index = active_ring_index;
        }

        uav::xyz_yaw_d next_uav_pos_d = uav_pos_d;
        uam_message::arm_angle next_angle = current_angle;
        next_angle.hand_angle = hand_hold_deg;
        next_uav_pos_d.yaw_d = hover_yaw_deg;
        next_uav_pos_d.land_flag = false;

        bool hold_commands = false;
        if (g_mavros_state.valid && !g_mavros_state.connected &&
            stage != STAGE_SETTLE && stage != STAGE_LANDING) {
            hold_commands = true;
            force_recovery = true;
            ROS_ERROR_THROTTLE(1.0, "exp4 mavros disconnected, switching to recovery");
        }

        if (stage == STAGE_SETTLE) {
            if (hover_anchor_initialized) {
                next_uav_pos_d.x_d = hover_anchor.x;
                next_uav_pos_d.y_d = hover_anchor.y;
                next_uav_pos_d.z_d = hover_anchor.z;
            }
            next_angle.arm1_angle = last_valid_arm1_deg;
            next_angle.arm2_angle = last_valid_arm2_deg;

            if (hover_anchor_initialized && g_base_pose.valid && g_mavros_state.valid &&
                AreAllRingsFresh(now, ring_pose_timeout_sec)) {
                stage_elapsed += dt;
            } else {
                stage_elapsed = 0.0;
            }

            if (stage_elapsed >= stage_duration(STAGE_SETTLE)) {
                transition_to(STAGE_APPROACH, 0);
            }
        } else if (stage == STAGE_APPROACH || stage == STAGE_PASS || stage == STAGE_DEPART) {
            if (!g_base_pose.valid) {
                hold_commands = true;
                ROS_WARN_THROTTLE(1.0, "exp4 waiting for base pose, keeping last valid command");
            }
            if (!AreAllRingsFresh(now, ring_pose_timeout_sec)) {
                hold_commands = true;
                ++ring_loss_count;
                ROS_WARN_THROTTLE(1.0, "exp4 ring pose missing (%d/%d), pausing state machine",
                                  ring_loss_count, ring_loss_limit);
                if (ring_loss_count >= ring_loss_limit) {
                    force_recovery = true;
                    ROS_ERROR("exp4 ring pose lost continuously, switching to recovery");
                }
            } else {
                ring_loss_count = 0;
            }

            if (!hold_commands && !force_recovery) {
                const std::size_t ring_index = static_cast<std::size_t>(active_ring_index);
                Vec3 stage_goal_ee_world = g_ring_poses[ring_index].position;
                if (stage == STAGE_APPROACH) {
                    stage_goal_ee_world = AddVec3(stage_goal_ee_world, ring_configs[ring_index].approach_offset);
                } else if (stage == STAGE_DEPART) {
                    stage_goal_ee_world = AddVec3(stage_goal_ee_world, ring_configs[ring_index].depart_offset);
                }

                if (!segment_start_valid) {
                    capture_segment_start();
                }

                stage_elapsed += dt;
                const double duration = stage_duration(stage);
                const double blend = QuinticBlend(stage_elapsed / duration);
                const Vec3 desired_ee_world =
                    InterpolateVec3(segment_start_ee_world, stage_goal_ee_world, blend);

                const Vec3 desired_base_world = SubVec3(desired_ee_world, nominal_ee_world);
                const Vec3 limited_base_world = LimitBaseReferenceStep(
                    uav_pos_d, desired_base_world, base_step_limit_xy, base_step_limit_z);
                next_uav_pos_d.x_d = limited_base_world.x;
                next_uav_pos_d.y_d = limited_base_world.y;
                next_uav_pos_d.z_d = limited_base_world.z;

                const Vec3 target_offset_world = SubVec3(desired_ee_world, g_base_pose.position);
                const Vec3 target_body = InverseRotateYawOnly(target_offset_world, g_base_pose.yaw_rad);

                double solved_arm1_deg = last_valid_arm1_deg;
                double solved_arm2_deg = last_valid_arm2_deg;
                const bool solved = SolveInverseKinematics(target_body,
                                                           last_valid_arm1_deg,
                                                           last_valid_arm2_deg,
                                                           arm1_min,
                                                           arm1_max,
                                                           arm2_min,
                                                           arm2_max,
                                                           &solved_arm1_deg,
                                                           &solved_arm2_deg);
                if (!solved) {
                    ++ik_fail_count;
                    next_angle.arm1_angle = last_valid_arm1_deg;
                    next_angle.arm2_angle = last_valid_arm2_deg;
                    ROS_WARN_THROTTLE(1.0,
                                      "exp4 IK failed (%d/%d), target_body=(%.3f, %.3f, %.3f)",
                                      ik_fail_count, ik_fail_limit, target_body.x, target_body.y,
                                      target_body.z);
                    if (ik_fail_count >= ik_fail_limit) {
                        force_recovery = true;
                        ROS_ERROR("exp4 IK failed continuously, switching to recovery");
                    }
                } else {
                    ik_fail_count = 0;
                    next_angle.arm1_angle = solved_arm1_deg;
                    next_angle.arm2_angle = solved_arm2_deg;
                    last_valid_arm1_deg = solved_arm1_deg;
                    last_valid_arm2_deg = solved_arm2_deg;
                    last_desired_ee_world = desired_ee_world;
                    last_desired_ee_world_valid = true;
                    last_safe_uav_pos_d = next_uav_pos_d;
                }

                if (!force_recovery && stage_elapsed >= duration) {
                    if (stage == STAGE_APPROACH) {
                        transition_to(STAGE_PASS, active_ring_index);
                    } else if (stage == STAGE_PASS) {
                        transition_to(STAGE_DEPART, active_ring_index);
                    } else if (static_cast<std::size_t>(active_ring_index + 1) < ring_configs.size()) {
                        transition_to(STAGE_APPROACH, active_ring_index + 1);
                    } else {
                        transition_to(STAGE_RECOVERY, active_ring_index);
                    }
                }
            } else {
                next_uav_pos_d = uav_pos_d;
                next_angle.arm1_angle = current_angle.arm1_angle;
                next_angle.arm2_angle = current_angle.arm2_angle;
            }
        } else if (stage == STAGE_RECOVERY) {
            next_angle.arm1_angle = last_valid_arm1_deg;
            next_angle.arm2_angle = last_valid_arm2_deg;

            if (static_cast<std::size_t>(active_ring_index) < ring_configs.size() &&
                IsRingFresh(static_cast<std::size_t>(active_ring_index), now, ring_pose_timeout_sec)) {
                if (!segment_start_valid) {
                    capture_segment_start();
                }
                stage_elapsed += dt;

                const Vec3 recovery_goal_ee_world =
                    AddVec3(g_ring_poses[static_cast<std::size_t>(active_ring_index)].position,
                            final_hover_offset);
                const double blend = QuinticBlend(stage_elapsed / stage_duration(STAGE_RECOVERY));
                const Vec3 desired_ee_world =
                    InterpolateVec3(segment_start_ee_world, recovery_goal_ee_world, blend);
                const Vec3 desired_base_world = SubVec3(desired_ee_world, nominal_ee_world);
                const Vec3 limited_base_world = LimitBaseReferenceStep(
                    uav_pos_d, desired_base_world, base_step_limit_xy, base_step_limit_z);
                next_uav_pos_d.x_d = limited_base_world.x;
                next_uav_pos_d.y_d = limited_base_world.y;
                next_uav_pos_d.z_d = limited_base_world.z;
                last_safe_uav_pos_d = next_uav_pos_d;
                last_desired_ee_world = desired_ee_world;
                last_desired_ee_world_valid = true;
            } else {
                stage_elapsed += dt;
                next_uav_pos_d = last_safe_uav_pos_d;
            }

            if (stage_elapsed >= stage_duration(STAGE_RECOVERY)) {
                transition_to(STAGE_LANDING, active_ring_index);
            }
        } else if (stage == STAGE_LANDING) {
            stage_elapsed += dt;
            next_uav_pos_d = last_safe_uav_pos_d;
            next_uav_pos_d.z_d = landing_z;
            next_uav_pos_d.land_flag = true;
            next_angle.arm1_angle = last_valid_arm1_deg;
            next_angle.arm2_angle = last_valid_arm2_deg;
            if (stage_elapsed >= stage_duration(STAGE_LANDING)) {
                break;
            }
        }

        next_angle.arm1_angle = Clamp(next_angle.arm1_angle, arm1_min, arm1_max);
        next_angle.arm2_angle = Clamp(next_angle.arm2_angle, arm2_min, arm2_max);
        next_angle.hand_angle = Clamp(next_angle.hand_angle, hand_min, hand_max);

        if ((now - last_log_time).toSec() >= 1.0) {
            last_log_time = now;
            std::string ring_name = "n/a";
            if (stage == STAGE_APPROACH || stage == STAGE_PASS || stage == STAGE_DEPART ||
                stage == STAGE_RECOVERY) {
                if (active_ring_index >= 0 &&
                    static_cast<std::size_t>(active_ring_index) < ring_configs.size()) {
                    ring_name = ring_configs[static_cast<std::size_t>(active_ring_index)].name;
                }
            }
            ROS_INFO(
                "[%s] t=%.2f s, ring=%s, pose_d=(%.2f, %.2f, %.2f, %.2f), arm_d=(%.2f, %.2f, %.2f), ring_loss=%d, ik_fail=%d, rings_ready=%s",
                StageName(stage), elapsed, ring_name.c_str(), next_uav_pos_d.x_d, next_uav_pos_d.y_d,
                next_uav_pos_d.z_d, next_uav_pos_d.yaw_d, next_angle.arm1_angle,
                next_angle.arm2_angle, next_angle.hand_angle, ring_loss_count, ik_fail_count,
                AreAllRingsFresh(now, ring_pose_timeout_sec) ? "true" : "false");
        }

        uav_pos_d = next_uav_pos_d;
        current_angle = next_angle;
        uav_pos_d_pub.publish(uav_pos_d);
        joint_angle_pub.publish(current_angle);
        rate.sleep();
    }

    PublishReturnHome(0.0, 0.0, hand_hold_deg);
    std::cout << "exp4 finished" << std::endl;
    return 0;
}
