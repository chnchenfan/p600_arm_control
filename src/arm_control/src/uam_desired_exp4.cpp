#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <locale.h>
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

struct PoseState {
    bool valid = false;
    ros::Time stamp;
    Vec3 position{0.0, 0.0, 0.0};
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double qw = 1.0;
};

struct ArmState {
    bool valid = false;
    double arm1_deg = 0.0;
    double arm2_deg = 0.0;
    double hand_deg = 0.0;
};

// 每个 target 都是完整刚体：
// - 位置用于确定 rigid body 原点
// - 姿态用于把“钩子工作点偏移”从刚体系变换到世界系
struct TargetConfig {
    std::string name;
    Vec3 first_pass_approach_offset{0.0, 0.0, 0.0};
    Vec3 first_pass_work_offset{0.0, 0.0, 0.0};
    Vec3 first_pass_depart_offset{0.0, 0.0, 0.0};
    Vec3 second_pass_approach_offset{0.0, 0.0, 0.0};
    Vec3 second_pass_work_offset{0.0, 0.0, 0.0};
    Vec3 second_pass_depart_offset{0.0, 0.0, 0.0};
};

enum class MotionStage {
    kSettle = 0,
    kAxisMoveY = 1,
    kAxisMoveX = 2,
    kApproach = 3,
    kPass = 4,
    kDepart = 5,
    kReturnHover = 6,
    kFinalHold = 7,
    kLanding = 8,
};

enum class ProtectionState {
    kNormal = 0,
    kSafetyHold = 1,
    kSafetyLanding = 2,
};

struct FreezeWindowState {
    bool initialized = false;
    ros::Time stamp;
    Vec3 position{0.0, 0.0, 0.0};
};

struct MavrosStateCache {
    bool valid = false;
    bool connected = false;
    bool armed = false;
    std::string mode;
};

PoseState g_base_world_pose;
PoseState g_local_pose;
ArmState g_real_arm_state;
std::vector<PoseState> g_target_poses;
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

void NormalizeQuaternion(double *qx, double *qy, double *qz, double *qw) {
    const double norm = std::sqrt((*qx) * (*qx) + (*qy) * (*qy) + (*qz) * (*qz) + (*qw) * (*qw));
    if (norm < 1e-9) {
        *qx = 0.0;
        *qy = 0.0;
        *qz = 0.0;
        *qw = 1.0;
        return;
    }
    *qx /= norm;
    *qy /= norm;
    *qz /= norm;
    *qw /= norm;
}

Vec3 RotateBodyToWorld(const Vec3 &vector_body, const PoseState &pose) {
    double qx = pose.qx;
    double qy = pose.qy;
    double qz = pose.qz;
    double qw = pose.qw;
    NormalizeQuaternion(&qx, &qy, &qz, &qw);

    const double r00 = 1.0 - 2.0 * (qy * qy + qz * qz);
    const double r01 = 2.0 * (qx * qy - qw * qz);
    const double r02 = 2.0 * (qx * qz + qw * qy);
    const double r10 = 2.0 * (qx * qy + qw * qz);
    const double r11 = 1.0 - 2.0 * (qx * qx + qz * qz);
    const double r12 = 2.0 * (qy * qz - qw * qx);
    const double r20 = 2.0 * (qx * qz - qw * qy);
    const double r21 = 2.0 * (qy * qz + qw * qx);
    const double r22 = 1.0 - 2.0 * (qx * qx + qy * qy);

    return {
        r00 * vector_body.x + r01 * vector_body.y + r02 * vector_body.z,
        r10 * vector_body.x + r11 * vector_body.y + r12 * vector_body.z,
        r20 * vector_body.x + r21 * vector_body.y + r22 * vector_body.z,
    };
}

double QuaternionToYaw(double x, double y, double z, double w) {
    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    return std::atan2(siny_cosp, cosy_cosp);
}

double QuinticBlend(double ratio) {
    const double x = Clamp(ratio, 0.0, 1.0);
    return x * x * x * (10.0 + x * (-15.0 + 6.0 * x));
}

bool IsPoseFresh(const PoseState &pose, const ros::Time &now, double timeout_sec) {
    if (!pose.valid) {
        return false;
    }
    if (timeout_sec <= 0.0) {
        return true;
    }
    return (now - pose.stamp).toSec() <= timeout_sec;
}

Vec3 MapWorldToOutputFrame(const Vec3 &world_position,
                           bool mapping_initialized,
                           const Vec3 &world_anchor,
                           const Vec3 &local_anchor) {
    if (!mapping_initialized) {
        return world_position;
    }
    return AddVec3(local_anchor, SubVec3(world_position, world_anchor));
}

Vec3 LimitOutputReferenceStep(const uav::xyz_yaw_d &previous_command,
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

const char *MotionStageName(MotionStage stage) {
    switch (stage) {
        case MotionStage::kSettle:
            return "settle";
        case MotionStage::kAxisMoveY:
            return "axis_move_y";
        case MotionStage::kAxisMoveX:
            return "axis_move_x";
        case MotionStage::kApproach:
            return "approach";
        case MotionStage::kPass:
            return "pass";
        case MotionStage::kDepart:
            return "depart";
        case MotionStage::kReturnHover:
            return "return_hover";
        case MotionStage::kFinalHold:
            return "final_hold";
        case MotionStage::kLanding:
            return "landing";
        default:
            return "unknown";
    }
}

const char *ProtectionStateName(ProtectionState state) {
    switch (state) {
        case ProtectionState::kNormal:
            return "normal";
        case ProtectionState::kSafetyHold:
            return "safety_hold";
        case ProtectionState::kSafetyLanding:
            return "safety_landing";
        default:
            return "unknown";
    }
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

bool LoadIntVectorParam(const ros::NodeHandle &nh,
                        const std::string &key,
                        const std::vector<int> &fallback,
                        std::vector<int> *output) {
    XmlRpc::XmlRpcValue xml_value;
    if (!nh.getParam(key, xml_value) || xml_value.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        *output = fallback;
        return false;
    }

    output->clear();
    for (int index = 0; index < xml_value.size(); ++index) {
        if (xml_value[index].getType() != XmlRpc::XmlRpcValue::TypeInt) {
            *output = fallback;
            return false;
        }
        output->push_back(static_cast<int>(xml_value[index]));
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

void FillPoseState(const geometry_msgs::PoseStamped::ConstPtr &msg, PoseState *state) {
    state->valid = true;
    state->stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    state->position = {msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
    state->qx = msg->pose.orientation.x;
    state->qy = msg->pose.orientation.y;
    state->qz = msg->pose.orientation.z;
    state->qw = msg->pose.orientation.w;
}

void BaseWorldPoseCb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    FillPoseState(msg, &g_base_world_pose);
}

void LocalPoseCb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    FillPoseState(msg, &g_local_pose);
}

void TargetPoseCb(const geometry_msgs::PoseStamped::ConstPtr &msg, std::size_t index) {
    if (index >= g_target_poses.size()) {
        return;
    }
    FillPoseState(msg, &g_target_poses[index]);
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

bool doReq(uav::desired_start::Request &req, uav::desired_start::Response &resp) {
    if (req.desired_start != 10) {
        ROS_ERROR("提交的数据异常!!!");
        return false;
    }
    resp.desired_sent = 3;
    start_flag = true;
    return true;
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
    bool use_targets = true;
    double settle_time = 3.0;
    double approach_time = 3.0;
    double pass_time = 2.0;
    double depart_time = 3.0;
    double return_time = 3.0;
    double landing_z = 0.5;
    double target_pose_timeout_sec = 0.3;
    int target_loss_limit = 10;
    double output_step_limit_xy = 0.03;
    double output_step_limit_z = 0.02;
    double settle_takeoff_step_limit_xy = 0.01;
    double settle_takeoff_step_limit_z = 0.01;
    double fixed_arm1_deg = 0.0;
    double fixed_arm2_deg = 0.0;
    double fixed_hand_deg = 25.0;
    double freeze_window_sec = 1.0;
    double freeze_motion_threshold_m = 0.03;
    double freeze_arrive_threshold_m = 0.15;
    int freeze_trigger_cycles = 2;
    int freeze_escalate_cycles = 6;
    double z_freeze_error_threshold_m = 0.20;
    double z_freeze_motion_threshold_m = 0.03;
    double first_target1_axis_move_time_y = 2.0;
    double first_target1_axis_move_time_x = 2.0;
    double loop_offset_y_m = 0.50;
    double loop_offset_x_m = 0.50;
    std::string base_world_pose_topic = "/vrpn_client_node/arm_base/pose";
    std::string local_pose_topic = "/mavros/local_position/pose";

    std::vector<std::string> target_names = {"target0", "target1", "target2"};
    std::vector<int> visit_sequence = {1, 0, 2};
    LoadStringVectorParam(pnh, "target_names", target_names, &target_names);
    LoadIntVectorParam(pnh, "visit_sequence", visit_sequence, &visit_sequence);

    pnh.param("hover_z", hover_z, hover_z);
    pnh.param("hover_yaw_deg", hover_yaw_deg, hover_yaw_deg);
    pnh.param("use_targets", use_targets, use_targets);
    pnh.param("settle_time", settle_time, settle_time);
    pnh.param("approach_time", approach_time, approach_time);
    pnh.param("pass_time", pass_time, pass_time);
    pnh.param("depart_time", depart_time, depart_time);
    pnh.param("return_time", return_time, return_time);
    pnh.param("landing_z", landing_z, landing_z);
    pnh.param("target_pose_timeout_sec", target_pose_timeout_sec, target_pose_timeout_sec);
    pnh.param("target_loss_limit", target_loss_limit, target_loss_limit);
    pnh.param("base_step_limit_xy", output_step_limit_xy, output_step_limit_xy);
    pnh.param("base_step_limit_z", output_step_limit_z, output_step_limit_z);
    pnh.param("settle_takeoff_step_limit_xy", settle_takeoff_step_limit_xy, settle_takeoff_step_limit_xy);
    pnh.param("settle_takeoff_step_limit_z", settle_takeoff_step_limit_z, settle_takeoff_step_limit_z);
    pnh.param("fixed_arm1_deg", fixed_arm1_deg, fixed_arm1_deg);
    pnh.param("fixed_arm2_deg", fixed_arm2_deg, fixed_arm2_deg);
    pnh.param("fixed_hand_deg", fixed_hand_deg, fixed_hand_deg);
    pnh.param("freeze_window_sec", freeze_window_sec, freeze_window_sec);
    pnh.param("freeze_motion_threshold_m", freeze_motion_threshold_m, freeze_motion_threshold_m);
    pnh.param("freeze_arrive_threshold_m", freeze_arrive_threshold_m, freeze_arrive_threshold_m);
    pnh.param("freeze_trigger_cycles", freeze_trigger_cycles, freeze_trigger_cycles);
    pnh.param("freeze_escalate_cycles", freeze_escalate_cycles, freeze_escalate_cycles);
    pnh.param("z_freeze_error_threshold_m", z_freeze_error_threshold_m, z_freeze_error_threshold_m);
    pnh.param("z_freeze_motion_threshold_m", z_freeze_motion_threshold_m, z_freeze_motion_threshold_m);
    pnh.param("first_target1_axis_move_time_y", first_target1_axis_move_time_y, first_target1_axis_move_time_y);
    pnh.param("first_target1_axis_move_time_x", first_target1_axis_move_time_x, first_target1_axis_move_time_x);
    pnh.param("loop_offset_y_m", loop_offset_y_m, loop_offset_y_m);
    pnh.param("loop_offset_x_m", loop_offset_x_m, loop_offset_x_m);
    pnh.param("base_world_pose_topic", base_world_pose_topic, base_world_pose_topic);
    pnh.param("local_pose_topic", local_pose_topic, local_pose_topic);

    if (target_names.empty()) {
        target_names = {"target0", "target1", "target2"};
    }
    if (visit_sequence.empty()) {
        visit_sequence = {1, 0, 2};
    }
    if (settle_time < 0.0) {
        settle_time = 0.0;
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
    if (return_time <= 0.0) {
        return_time = 3.0;
    }
    if (target_pose_timeout_sec <= 0.0) {
        target_pose_timeout_sec = 0.3;
    }
    if (target_loss_limit < 1) {
        target_loss_limit = 1;
    }
    if (settle_takeoff_step_limit_xy <= 0.0) {
        settle_takeoff_step_limit_xy = 0.01;
    }
    if (settle_takeoff_step_limit_z <= 0.0) {
        settle_takeoff_step_limit_z = 0.01;
    }
    if (freeze_window_sec <= 0.0) {
        freeze_window_sec = 1.0;
    }
    if (freeze_motion_threshold_m <= 0.0) {
        freeze_motion_threshold_m = 0.03;
    }
    if (freeze_arrive_threshold_m <= 0.0) {
        freeze_arrive_threshold_m = 0.15;
    }
    if (freeze_trigger_cycles < 1) {
        freeze_trigger_cycles = 1;
    }
    if (freeze_escalate_cycles < 1) {
        freeze_escalate_cycles = 1;
    }
    if (z_freeze_error_threshold_m <= 0.0) {
        z_freeze_error_threshold_m = 0.20;
    }
    if (z_freeze_motion_threshold_m <= 0.0) {
        z_freeze_motion_threshold_m = 0.03;
    }
    if (first_target1_axis_move_time_y <= 0.0) {
        first_target1_axis_move_time_y = 2.0;
    }
    if (first_target1_axis_move_time_x <= 0.0) {
        first_target1_axis_move_time_x = 2.0;
    }
    if (use_targets) {
        for (std::size_t index = 0; index < visit_sequence.size(); ++index) {
            if (visit_sequence[index] < 0 ||
                static_cast<std::size_t>(visit_sequence[index]) >= target_names.size()) {
                ROS_ERROR("exp4 invalid visit_sequence[%zu]=%d, target_names size=%zu",
                          index, visit_sequence[index], target_names.size());
                return 1;
            }
        }
    }

    std::vector<TargetConfig> target_configs(target_names.size());
    for (std::size_t index = 0; index < target_names.size(); ++index) {
        target_configs[index].name = target_names[index];
        const std::string prefix = "targets/" + target_names[index] + "/";
        LoadVec3Param(pnh, prefix + "first_pass_approach_offset_xyz",
                      target_configs[index].first_pass_approach_offset,
                      &target_configs[index].first_pass_approach_offset);
        LoadVec3Param(pnh, prefix + "first_pass_work_offset_xyz",
                      target_configs[index].first_pass_work_offset,
                      &target_configs[index].first_pass_work_offset);
        LoadVec3Param(pnh, prefix + "first_pass_depart_offset_xyz",
                      target_configs[index].first_pass_depart_offset,
                      &target_configs[index].first_pass_depart_offset);
        LoadVec3Param(pnh, prefix + "second_pass_approach_offset_xyz",
                      target_configs[index].second_pass_approach_offset,
                      &target_configs[index].second_pass_approach_offset);
        LoadVec3Param(pnh, prefix + "second_pass_work_offset_xyz",
                      target_configs[index].second_pass_work_offset,
                      &target_configs[index].second_pass_work_offset);
        LoadVec3Param(pnh, prefix + "second_pass_depart_offset_xyz",
                      target_configs[index].second_pass_depart_offset,
                      &target_configs[index].second_pass_depart_offset);
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
    fixed_arm1_deg = Clamp(fixed_arm1_deg, arm1_min, arm1_max);
    fixed_arm2_deg = Clamp(fixed_arm2_deg, arm2_min, arm2_max);
    fixed_hand_deg = Clamp(fixed_hand_deg, hand_min, hand_max);

    ros::Subscriber base_world_pose_sub =
        nh.subscribe<geometry_msgs::PoseStamped>(base_world_pose_topic, 10, BaseWorldPoseCb);
    ros::Subscriber local_pose_sub =
        nh.subscribe<geometry_msgs::PoseStamped>(local_pose_topic, 10, LocalPoseCb);
    ros::Subscriber arm_real_sub =
        nh.subscribe<uam_message::arm_angle>("/wjl/arm/real/angle_r", 10, ArmRealCb);
    ros::Subscriber mavros_state_sub =
        nh.subscribe<mavros_msgs::State>("/mavros/state", 10, MavrosStateCb);

    g_target_poses.assign(target_names.size(), PoseState());
    std::vector<ros::Subscriber> target_pose_subs;
    target_pose_subs.reserve(target_names.size());
    for (std::size_t index = 0; index < target_names.size(); ++index) {
        const std::string topic = "/vrpn_client_node/" + target_names[index] + "/pose";
        target_pose_subs.push_back(
            nh.subscribe<geometry_msgs::PoseStamped>(topic, 10, boost::bind(&TargetPoseCb, _1, index)));
    }

    ROS_INFO("exp4 service ready, waiting for /wjl/start/uav_desired");
    ROS_INFO(
        "exp4 params: use_targets=%s, hover_z=%.2f, yaw=%.2f deg, settle=%.2f s, segment_time=(%.2f, %.2f, %.2f, return=%.2f), landing_z=%.2f, fixed_arm=(%.2f, %.2f, %.2f), target_timeout=%.2f, settle_takeoff_step=(xy %.3f, z %.3f), freeze_window=%.2f, freeze_motion=%.3f",
        use_targets ? "true" : "false", hover_z, hover_yaw_deg, settle_time, approach_time,
        pass_time, depart_time, return_time, landing_z, fixed_arm1_deg, fixed_arm2_deg,
        fixed_hand_deg, target_pose_timeout_sec, settle_takeoff_step_limit_xy,
        settle_takeoff_step_limit_z, freeze_window_sec, freeze_motion_threshold_m);
    if (use_targets) {
        for (std::size_t index = 0; index < target_configs.size(); ++index) {
            ROS_INFO("exp4 target[%zu]=%s", index, target_configs[index].name.c_str());
        }
    } else {
        ROS_INFO("exp4 loop mode: hover -> (+y %.2f) -> (+x %.2f) -> (-y %.2f) -> return hover -> final_hold",
                 loop_offset_y_m, loop_offset_x_m, loop_offset_y_m);
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
    auto PublishReturnHome = [&](double home_hand_deg) {
        if (!ros::ok()) {
            return;
        }
        uam_message::arm_angle home_angle;
        home_angle.arm1_angle = Clamp(0.0, arm1_min, arm1_max);
        home_angle.arm2_angle = Clamp(0.0, arm2_min, arm2_max);
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

    auto all_targets_fresh = [&](const ros::Time &now) {
        if (g_target_poses.empty()) {
            return false;
        }
        for (std::size_t index = 0; index < g_target_poses.size(); ++index) {
            if (!IsPoseFresh(g_target_poses[index], now, target_pose_timeout_sec)) {
                return false;
            }
        }
        return true;
    };

    auto target_work_offset = [&](std::size_t target_index, std::size_t visit_index, MotionStage stage) {
        std::size_t visit_count_before_current = 0;
        for (std::size_t index = 0; index < visit_index; ++index) {
            if (visit_sequence[index] == static_cast<int>(target_index)) {
                ++visit_count_before_current;
            }
        }
        // 当前 target 第一次出现用 first_pass，第二次及以后统一归到 second_pass。
        // 这样 visit_sequence 改成单轮或非对称序列后，不会再被“前半段/后半段”错误切分。
        const bool second_pass = visit_count_before_current >= 1;
        const TargetConfig &config = target_configs[target_index];
        if (!second_pass) {
            if (stage == MotionStage::kApproach) {
                return config.first_pass_approach_offset;
            }
            if (stage == MotionStage::kPass) {
                return config.first_pass_work_offset;
            }
            return config.first_pass_depart_offset;
        }
        if (stage == MotionStage::kApproach) {
            return config.second_pass_approach_offset;
        }
        if (stage == MotionStage::kPass) {
            return config.second_pass_work_offset;
        }
        return config.second_pass_depart_offset;
    };

    auto target_work_point_world = [&](std::size_t target_index, std::size_t visit_index, MotionStage stage) {
        const Vec3 offset_body = target_work_offset(target_index, visit_index, stage);
        return AddVec3(g_target_poses[target_index].position, RotateBodyToWorld(offset_body, g_target_poses[target_index]));
    };

    const ros::Time experiment_start = ros::Time::now();
    ros::Time last_loop_time = experiment_start;
    ros::Time last_log_time = experiment_start - ros::Duration(1.0);

    uav::xyz_yaw_d uav_pos_d;
    uav_pos_d.x_d = 0.0;
    uav_pos_d.y_d = 0.0;
    uav_pos_d.z_d = 0.0;
    uav_pos_d.yaw_d = hover_yaw_deg;
    uav_pos_d.land_flag = false;

    uam_message::arm_angle current_angle;
    current_angle.arm1_angle = fixed_arm1_deg;
    current_angle.arm2_angle = fixed_arm2_deg;
    current_angle.hand_angle = fixed_hand_deg;

    MotionStage motion_stage = MotionStage::kSettle;
    MotionStage last_logged_stage = static_cast<MotionStage>(-1);
    ProtectionState protection_state = ProtectionState::kNormal;
    ProtectionState last_logged_protection = ProtectionState::kNormal;
    std::string protection_reason = "none";
    ros::Time protection_state_start;

    std::size_t visit_index = 0;
    double stage_elapsed = 0.0;
    Vec3 segment_start_output{0.0, 0.0, hover_z};
    bool segment_start_initialized = false;
    bool hover_anchor_initialized = false;
    Vec3 hover_anchor_world{0.0, 0.0, 0.0};
    Vec3 hover_anchor_output{0.0, 0.0, hover_z};
    bool frame_mapping_initialized = false;
    Vec3 mapping_world_anchor{0.0, 0.0, 0.0};
    Vec3 mapping_output_anchor{0.0, 0.0, 0.0};
    Vec3 last_safe_output_position{0.0, 0.0, hover_z};
    bool last_safe_output_initialized = false;
    Vec3 safe_hover_output{0.0, 0.0, hover_z};
    Vec3 axis_move_goal_output{0.0, 0.0, hover_z};
    std::vector<Vec3> loop_waypoints_output;
    MotionStage axis_move_followup_stage = MotionStage::kPass;
    FreezeWindowState freeze_window_state;
    int target_loss_count = 0;
    int freeze_fault_count = 0;
    int safety_escalate_count = 0;

    auto set_segment_start_from_last_command = [&]() {
        segment_start_output = {uav_pos_d.x_d, uav_pos_d.y_d, uav_pos_d.z_d};
        segment_start_initialized = true;
    };

    auto enter_motion_stage = [&](MotionStage next_stage) {
        motion_stage = next_stage;
        stage_elapsed = 0.0;
        if (next_stage == MotionStage::kApproach || next_stage == MotionStage::kPass ||
            next_stage == MotionStage::kDepart || next_stage == MotionStage::kReturnHover ||
            next_stage == MotionStage::kAxisMoveY || next_stage == MotionStage::kAxisMoveX) {
            set_segment_start_from_last_command();
        } else {
            segment_start_initialized = false;
        }
    };

    auto enter_axis_move = [&](const Vec3 &goal_output, MotionStage followup_stage) {
        axis_move_goal_output = goal_output;
        axis_move_followup_stage = followup_stage;
        enter_motion_stage(MotionStage::kAxisMoveY);
    };

    auto stage_duration = [&](MotionStage stage) {
        if (stage == MotionStage::kAxisMoveY) {
            return first_target1_axis_move_time_y;
        }
        if (stage == MotionStage::kAxisMoveX) {
            return first_target1_axis_move_time_x;
        }
        if (stage == MotionStage::kApproach) {
            return approach_time;
        }
        if (stage == MotionStage::kPass) {
            return pass_time;
        }
        if (stage == MotionStage::kDepart) {
            return depart_time;
        }
        // 当前路径最后一段改成 final_hold，不再走 return_hover；
        // 这里先保留给后续改回“返回悬停点”逻辑时使用。
        if (stage == MotionStage::kReturnHover) {
            return return_time;
        }
        return settle_time;
    };

    auto should_use_axis_first_to_target = [&](int previous_target_index, int next_target_index, bool from_hover) {
        if (from_hover) {
            return next_target_index == 1;
        }
        return previous_target_index == 1 && next_target_index == 0;
    };

    auto should_use_axis_first_return = [&](int previous_target_index) {
        return false;
    };

    auto ensure_loop_waypoints_initialized = [&]() {
        if (!loop_waypoints_output.empty() || !hover_anchor_initialized) {
            return;
        }
        // 无 target 验证模式：围绕初始悬停点飞一个简单闭环。
        // 只验证起飞、限幅和 local pose 保护，不依赖外部 target 刚体。
        loop_waypoints_output.push_back(
            AddVec3(hover_anchor_output, Vec3{0.0, loop_offset_y_m, 0.0}));
        loop_waypoints_output.push_back(
            AddVec3(hover_anchor_output, Vec3{loop_offset_x_m, loop_offset_y_m, 0.0}));
        loop_waypoints_output.push_back(
            AddVec3(hover_anchor_output, Vec3{loop_offset_x_m, 0.0, 0.0}));
        loop_waypoints_output.push_back(hover_anchor_output);
    };

    auto enter_protection_state = [&](ProtectionState next_state, const std::string &reason) {
        if (protection_state == next_state) {
            return;
        }
        protection_state = next_state;
        protection_reason = reason;
        protection_state_start = ros::Time::now();
        safe_hover_output = hover_anchor_initialized ? hover_anchor_output :
                            (last_safe_output_initialized ? last_safe_output_position : Vec3{uav_pos_d.x_d, uav_pos_d.y_d, uav_pos_d.z_d});
    };

    while (ros::ok()) {
        ros::spinOnce();

        const ros::Time now = ros::Time::now();
        const double dt = std::max(0.0, (now - last_loop_time).toSec());
        last_loop_time = now;
        const double elapsed = (now - experiment_start).toSec();

        const bool local_fresh = IsPoseFresh(g_local_pose, now, target_pose_timeout_sec);
        const bool base_world_fresh = IsPoseFresh(g_base_world_pose, now, target_pose_timeout_sec);
        const bool all_target_fresh = use_targets ? all_targets_fresh(now) : true;
        bool current_target_fresh = !use_targets;
        int current_target_index = -1;
        if (use_targets && visit_index < visit_sequence.size()) {
            current_target_index = visit_sequence[visit_index];
            current_target_fresh =
                current_target_index >= 0 &&
                static_cast<std::size_t>(current_target_index) < g_target_poses.size() &&
                IsPoseFresh(g_target_poses[static_cast<std::size_t>(current_target_index)], now, target_pose_timeout_sec);
        }

        if (!hover_anchor_initialized && local_fresh && base_world_fresh) {
            // hover_anchor 统一按 arm_base 语义冻结：
            // - world_anchor 用于把 target 世界点映射到 PX4 输出系
            // - output_anchor 就是当前 /mavros/local_position/pose
            hover_anchor_world = g_base_world_pose.position;
            hover_anchor_world.z = hover_z;
            hover_anchor_output = g_local_pose.position;
            hover_anchor_output.z = hover_z;
            mapping_world_anchor = g_base_world_pose.position;
            mapping_output_anchor = g_local_pose.position;
            frame_mapping_initialized = true;
            hover_anchor_initialized = true;
            ensure_loop_waypoints_initialized();
            safe_hover_output = hover_anchor_output;
            last_safe_output_position = hover_anchor_output;
            last_safe_output_initialized = true;
            uav_pos_d.x_d = hover_anchor_output.x;
            uav_pos_d.y_d = hover_anchor_output.y;
            uav_pos_d.z_d = hover_anchor_output.z;
        }

        if (motion_stage != last_logged_stage) {
            ROS_INFO("exp4 stage -> %s", MotionStageName(motion_stage));
            last_logged_stage = motion_stage;
        }
        if (protection_state != last_logged_protection) {
            ROS_WARN("exp4 protection -> %s, reason=%s",
                     ProtectionStateName(protection_state), protection_reason.c_str());
            last_logged_protection = protection_state;
        }

        if (g_mavros_state.valid && !g_mavros_state.connected &&
            protection_state == ProtectionState::kNormal &&
            motion_stage != MotionStage::kSettle && motion_stage != MotionStage::kLanding) {
            enter_protection_state(ProtectionState::kSafetyHold, "mavros disconnected");
        }

        uav::xyz_yaw_d next_uav_pos_d = uav_pos_d;
        uam_message::arm_angle next_angle = current_angle;
        next_angle.arm1_angle = fixed_arm1_deg;
        next_angle.arm2_angle = fixed_arm2_deg;
        next_angle.hand_angle = fixed_hand_deg;
        next_uav_pos_d.yaw_d = hover_yaw_deg;
        next_uav_pos_d.land_flag = false;

        bool freshness_fault = false;
        bool freeze_fault_active = false;

        if (protection_state == ProtectionState::kNormal) {
            if (motion_stage == MotionStage::kSettle) {
                // 起飞阶段分两步：
                // 1. 还没建立 hover_anchor 前，先把期望锁在当前 local pose；
                // 2. 建立 hover_anchor 后，再用独立的小步长限幅慢速抬升到 hover_z。
                //
                // 这样做是为了减小起飞瞬间的横向冲击：
                // - 不在刚进入 offboard 时就发一个可能与当前位置有偏差的悬停点；
                // - 先让飞机“原地稳住”；
                // - 再慢慢升到实验悬停高度。
                if (!hover_anchor_initialized) {
                    if (local_fresh) {
                        next_uav_pos_d.x_d = g_local_pose.position.x;
                        next_uav_pos_d.y_d = g_local_pose.position.y;
                        next_uav_pos_d.z_d = g_local_pose.position.z;
                    } else {
                        next_uav_pos_d.x_d = uav_pos_d.x_d;
                        next_uav_pos_d.y_d = uav_pos_d.y_d;
                        next_uav_pos_d.z_d = uav_pos_d.z_d;
                    }
                } else {
                    // settle 段单独使用 settle_takeoff_step_limit_*，
                    // 不复用主段路径的 base_step_limit_*，这样起飞上升会明显更保守。
                    const Vec3 settle_goal_output = hover_anchor_output;
                    const Vec3 limited_settle_output =
                        LimitOutputReferenceStep(uav_pos_d, settle_goal_output,
                                                 settle_takeoff_step_limit_xy,
                                                 settle_takeoff_step_limit_z);
                    next_uav_pos_d.x_d = limited_settle_output.x;
                    next_uav_pos_d.y_d = limited_settle_output.y;
                    next_uav_pos_d.z_d = limited_settle_output.z;
                }

                if (hover_anchor_initialized && local_fresh && base_world_fresh && all_target_fresh) {
                    stage_elapsed += dt;
                } else {
                    stage_elapsed = 0.0;
                }

                if (stage_elapsed >= stage_duration(MotionStage::kSettle)) {
                    if (!use_targets && !loop_waypoints_output.empty()) {
                        visit_index = 0;
                        enter_motion_stage(MotionStage::kApproach);
                    } else if (!visit_sequence.empty() &&
                        should_use_axis_first_to_target(-1, visit_sequence.front(), true)) {
                        const std::size_t target_index = static_cast<std::size_t>(visit_sequence.front());
                        const Vec3 target_goal_world =
                            target_work_point_world(target_index, visit_index, MotionStage::kApproach);
                        const Vec3 target_goal_output =
                            MapWorldToOutputFrame(target_goal_world, frame_mapping_initialized,
                                                  mapping_world_anchor, mapping_output_anchor);
                        enter_axis_move(target_goal_output, MotionStage::kPass);
                    } else {
                        enter_motion_stage(MotionStage::kApproach);
                    }
                }
            } else if (motion_stage == MotionStage::kAxisMoveY ||
                       motion_stage == MotionStage::kAxisMoveX ||
                       motion_stage == MotionStage::kApproach ||
                       motion_stage == MotionStage::kPass ||
                       motion_stage == MotionStage::kDepart ||
                       motion_stage == MotionStage::kReturnHover) {
                const bool returning_without_target =
                    (motion_stage == MotionStage::kAxisMoveY || motion_stage == MotionStage::kAxisMoveX) &&
                    axis_move_followup_stage == MotionStage::kLanding;
                freshness_fault = !local_fresh || !frame_mapping_initialized ||
                                 (use_targets &&
                                  !returning_without_target &&
                                  motion_stage != MotionStage::kReturnHover &&
                                  !current_target_fresh);

                if (freshness_fault) {
                    ++target_loss_count;
                    ROS_WARN_THROTTLE(1.0,
                                      "exp4 target/local freshness fault: local_fresh=%s, current_target_fresh=%s, mapping=%s (%d/%d)",
                                      local_fresh ? "true" : "false",
                                      current_target_fresh ? "true" : "false",
                                      frame_mapping_initialized ? "true" : "false",
                                      target_loss_count, target_loss_limit);
                    if (target_loss_count >= target_loss_limit) {
                        enter_protection_state(ProtectionState::kSafetyHold, "target/local pose freshness lost");
                    }
                } else {
                    target_loss_count = 0;
                }

                if (!freshness_fault && protection_state == ProtectionState::kNormal) {
                    Vec3 stage_goal_output = hover_anchor_output;
                    if (motion_stage == MotionStage::kReturnHover) {
                        stage_goal_output = hover_anchor_output;
                    } else if (motion_stage == MotionStage::kAxisMoveY ||
                               motion_stage == MotionStage::kAxisMoveX) {
                        stage_goal_output = axis_move_goal_output;
                        // “先走 y，再走 x” 的特殊路径都保持当前段起点高度不变：
                        // - 起飞悬停后去 target1：保持 hover 高度
                        // - target1 -> target0：保持 depart 段结束时的当前高度
                        if (motion_stage == MotionStage::kAxisMoveY) {
                            stage_goal_output.x = segment_start_output.x;
                            stage_goal_output.z = segment_start_output.z;
                        } else {
                            stage_goal_output.z = segment_start_output.z;
                        }
                    } else if (use_targets) {
                        const std::size_t target_index = static_cast<std::size_t>(current_target_index);
                        const MotionStage target_stage =
                            motion_stage == MotionStage::kApproach ? MotionStage::kApproach : motion_stage;
                        const Vec3 target_goal_world = target_work_point_world(target_index, visit_index, target_stage);
                        stage_goal_output =
                            MapWorldToOutputFrame(target_goal_world, frame_mapping_initialized,
                                                  mapping_world_anchor, mapping_output_anchor);
                    } else if (visit_index < loop_waypoints_output.size()) {
                        stage_goal_output = loop_waypoints_output[visit_index];
                    }

                    if (!segment_start_initialized) {
                        set_segment_start_from_last_command();
                    }

                    stage_elapsed += dt;
                    const double duration = stage_duration(motion_stage);
                    const double blend = QuinticBlend(stage_elapsed / duration);
                    const Vec3 desired_output =
                        InterpolateVec3(segment_start_output, stage_goal_output, blend);
                    const Vec3 limited_output =
                        LimitOutputReferenceStep(uav_pos_d, desired_output, output_step_limit_xy, output_step_limit_z);
                    next_uav_pos_d.x_d = limited_output.x;
                    next_uav_pos_d.y_d = limited_output.y;
                    next_uav_pos_d.z_d = limited_output.z;

                    const bool moving_stage =
                        motion_stage == MotionStage::kAxisMoveY ||
                        motion_stage == MotionStage::kAxisMoveX ||
                        motion_stage == MotionStage::kApproach ||
                        motion_stage == MotionStage::kPass ||
                        motion_stage == MotionStage::kDepart ||
                        motion_stage == MotionStage::kReturnHover;

                    // 冻结保护覆盖“动捕刚体丢失但一直发最后一帧”的故障。
                    // 这种情况下 local pose 可能仍然 fresh，但实际位置长期不动，
                    // 同时目标误差持续存在，于是不能继续推进工作点轨迹。
                    if (local_fresh && moving_stage) {
                        if (!freeze_window_state.initialized) {
                            freeze_window_state.initialized = true;
                            freeze_window_state.stamp = now;
                            freeze_window_state.position = g_local_pose.position;
                        } else if ((now - freeze_window_state.stamp).toSec() >= freeze_window_sec) {
                            const Vec3 motion = SubVec3(g_local_pose.position, freeze_window_state.position);
                            const double motion_norm = NormVec3(motion);
                            const Vec3 error = SubVec3(
                                Vec3{next_uav_pos_d.x_d, next_uav_pos_d.y_d, next_uav_pos_d.z_d},
                                g_local_pose.position);
                            const double pos_error_norm = NormVec3(error);
                            const double z_error = std::fabs(error.z);
                            const double z_motion = std::fabs(g_local_pose.position.z - freeze_window_state.position.z);

                            const bool generic_freeze_fault =
                                pos_error_norm > freeze_arrive_threshold_m &&
                                motion_norm < freeze_motion_threshold_m;
                            const bool z_freeze_fault =
                                z_error > z_freeze_error_threshold_m &&
                                z_motion < z_freeze_motion_threshold_m;
                            freeze_fault_active = generic_freeze_fault || z_freeze_fault;
                            if (freeze_fault_active) {
                                ++freeze_fault_count;
                                ROS_WARN_THROTTLE(
                                    1.0,
                                    "exp4 freeze guard: pos_error=%.3f, motion=%.3f, z_error=%.3f, z_motion=%.3f (%d/%d)",
                                    pos_error_norm, motion_norm, z_error, z_motion,
                                    freeze_fault_count, freeze_trigger_cycles);
                                if (freeze_fault_count >= freeze_trigger_cycles) {
                                    enter_protection_state(ProtectionState::kSafetyHold, "local pose frozen before target reached");
                                }
                            } else {
                                freeze_fault_count = 0;
                            }
                            freeze_window_state.stamp = now;
                            freeze_window_state.position = g_local_pose.position;
                        }
                    } else {
                        freeze_window_state.initialized = false;
                        freeze_fault_count = 0;
                    }

                    if (protection_state == ProtectionState::kNormal && local_fresh) {
                        last_safe_output_position = Vec3{next_uav_pos_d.x_d, next_uav_pos_d.y_d, next_uav_pos_d.z_d};
                        last_safe_output_initialized = true;
                    }

                    if (protection_state == ProtectionState::kNormal && stage_elapsed >= duration) {
                        if (motion_stage == MotionStage::kAxisMoveY) {
                            enter_motion_stage(MotionStage::kAxisMoveX);
                        } else if (motion_stage == MotionStage::kAxisMoveX) {
                            enter_motion_stage(axis_move_followup_stage);
                        } else if (motion_stage == MotionStage::kApproach) {
                            if (!use_targets) {
                                if (visit_index + 1 < loop_waypoints_output.size()) {
                                    ++visit_index;
                                    enter_motion_stage(MotionStage::kApproach);
                                } else {
                                    enter_motion_stage(MotionStage::kFinalHold);
                                }
                            } else {
                                enter_motion_stage(MotionStage::kPass);
                            }
                        } else if (motion_stage == MotionStage::kPass) {
                            enter_motion_stage(MotionStage::kDepart);
                        } else if (motion_stage == MotionStage::kDepart) {
                            if (visit_index + 1 < visit_sequence.size()) {
                                const int previous_target_index = visit_sequence[visit_index];
                                ++visit_index;
                                const int next_target_index = visit_sequence[visit_index];
                                if (should_use_axis_first_to_target(previous_target_index, next_target_index, false)) {
                                    const std::size_t target_index = static_cast<std::size_t>(next_target_index);
                                    const Vec3 target_goal_world =
                                        target_work_point_world(target_index, visit_index, MotionStage::kApproach);
                                    const Vec3 target_goal_output =
                                        MapWorldToOutputFrame(target_goal_world, frame_mapping_initialized,
                                                              mapping_world_anchor, mapping_output_anchor);
                                    enter_axis_move(target_goal_output, MotionStage::kPass);
                                } else {
                                    enter_motion_stage(MotionStage::kApproach);
                                }
                            } else {
                                enter_motion_stage(MotionStage::kFinalHold);
                            }
                        } else if (motion_stage == MotionStage::kReturnHover) {
                            enter_motion_stage(MotionStage::kLanding);
                        }
                    }
                } else {
                    next_uav_pos_d = uav_pos_d;
                }
            } else if (motion_stage == MotionStage::kFinalHold) {
                next_uav_pos_d.x_d = uav_pos_d.x_d;
                next_uav_pos_d.y_d = uav_pos_d.y_d;
                next_uav_pos_d.z_d = uav_pos_d.z_d;
                next_uav_pos_d.land_flag = false;
            } else if (motion_stage == MotionStage::kLanding) {
                stage_elapsed += dt;
                next_uav_pos_d.x_d = hover_anchor_initialized ? hover_anchor_output.x : uav_pos_d.x_d;
                next_uav_pos_d.y_d = hover_anchor_initialized ? hover_anchor_output.y : uav_pos_d.y_d;
                next_uav_pos_d.z_d = landing_z;
                next_uav_pos_d.land_flag = true;
                if (stage_elapsed >= 0.5) {
                    break;
                }
            }
        }

        if (protection_state == ProtectionState::kSafetyHold) {
            next_uav_pos_d.x_d = safe_hover_output.x;
            next_uav_pos_d.y_d = safe_hover_output.y;
            next_uav_pos_d.z_d = safe_hover_output.z;
            next_uav_pos_d.land_flag = false;

            if (freshness_fault || freeze_fault_active || !local_fresh) {
                ++safety_escalate_count;
            } else {
                safety_escalate_count = 0;
            }
            if (safety_escalate_count >= freeze_escalate_cycles) {
                enter_protection_state(ProtectionState::kSafetyLanding, "fault persisted in safety_hold");
            }
        } else if (protection_state == ProtectionState::kSafetyLanding) {
            next_uav_pos_d.x_d = safe_hover_output.x;
            next_uav_pos_d.y_d = safe_hover_output.y;
            next_uav_pos_d.z_d = landing_z;
            next_uav_pos_d.land_flag = true;
            if (!protection_state_start.isZero() &&
                (now - protection_state_start).toSec() >= 0.5) {
                break;
            }
        }

        next_angle.arm1_angle = Clamp(next_angle.arm1_angle, arm1_min, arm1_max);
        next_angle.arm2_angle = Clamp(next_angle.arm2_angle, arm2_min, arm2_max);
        next_angle.hand_angle = Clamp(next_angle.hand_angle, hand_min, hand_max);

        if ((now - last_log_time).toSec() >= 1.0) {
            last_log_time = now;
            std::string target_name = use_targets ? "n/a" : "loop";
            if (use_targets &&
                current_target_index >= 0 &&
                static_cast<std::size_t>(current_target_index) < target_names.size()) {
                target_name = target_names[static_cast<std::size_t>(current_target_index)];
            }
            const std::size_t total_visits = use_targets ? visit_sequence.size() : loop_waypoints_output.size();
            ROS_INFO(
                "[%s|%s] t=%.2f s, visit=%zu/%zu, target=%s, pose_d=(%.2f, %.2f, %.2f, %.2f), arm_d=(%.2f, %.2f, %.2f), local_fresh=%s, base_world_fresh=%s, current_target_fresh=%s, target_loss=%d, freeze_fault=%d, safety_escalate=%d",
                MotionStageName(motion_stage), ProtectionStateName(protection_state), elapsed,
                visit_index + 1, total_visits, target_name.c_str(),
                next_uav_pos_d.x_d, next_uav_pos_d.y_d, next_uav_pos_d.z_d, next_uav_pos_d.yaw_d,
                next_angle.arm1_angle, next_angle.arm2_angle, next_angle.hand_angle,
                local_fresh ? "true" : "false",
                base_world_fresh ? "true" : "false",
                current_target_fresh ? "true" : "false",
                target_loss_count, freeze_fault_count, safety_escalate_count);
        }

        uav_pos_d = next_uav_pos_d;
        current_angle = next_angle;
        uav_pos_d_pub.publish(uav_pos_d);
        joint_angle_pub.publish(current_angle);
        rate.sleep();
    }

    PublishReturnHome(fixed_hand_deg);
    std::cout << "exp4 finished" << std::endl;
    return 0;
}
