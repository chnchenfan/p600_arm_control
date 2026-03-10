#include <ros/ros.h>

#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>

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

BasePoseState g_base_pose;
ArmState g_real_arm_state;

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
        const double dx = reconstructed.x - target_body.x;
        const double dy = reconstructed.y - target_body.y;
        const double dz = reconstructed.z - target_body.z;
        const double position_error = std::sqrt(dx * dx + dy * dy + dz * dz);
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

const char *StageName(int stage) {
    switch (stage) {
        case 0:
            return "settle";
        case 1:
            return "circle_hold";
        case 2:
            return "recovery";
        case 3:
            return "landing";
        default:
            return "unknown";
    }
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

}  // namespace

int main(int argc, char *argv[]) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "uam_desired_exp2");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ros::Publisher joint_angle_pub =
        nh.advertise<uam_message::arm_angle>("/wjl/arm/guidefly/angle_d", 10);
    ros::Publisher uav_pos_d_pub =
        nh.advertise<uav::xyz_yaw_d>("/wjl/guidefly/pose_d", 10);
    ros::ServiceServer server = nh.advertiseService("/wjl/start/uav_desired", doReq);

    double circle_center_x = 0.0;
    double circle_center_y = 0.0;
    double circle_radius = 0.03;
    double circle_period = 10.0;
    double hover_z = 1.0;
    double hover_yaw_deg = 0.0;
    double settle_time = 3.0;
    double experiment_time = 20.0;
    double recovery_time = 2.0;
    bool exp2_use_initial_ee_hold = true;
    double exp2_hold_ee_x = 0.0;
    double exp2_hold_ee_y = 0.0;
    double exp2_hold_ee_z = 0.0;
    int ik_fail_limit = 10;
    std::string base_pose_topic = "/mavros/local_position/pose";

    pnh.param("circle_center_x", circle_center_x, circle_center_x);
    pnh.param("circle_center_y", circle_center_y, circle_center_y);
    pnh.param("circle_radius", circle_radius, circle_radius);
    pnh.param("circle_period", circle_period, circle_period);
    pnh.param("hover_z", hover_z, hover_z);
    pnh.param("hover_yaw_deg", hover_yaw_deg, hover_yaw_deg);
    pnh.param("settle_time", settle_time, settle_time);
    pnh.param("experiment_time", experiment_time, experiment_time);
    pnh.param("recovery_time", recovery_time, recovery_time);
    pnh.param("exp2_use_initial_ee_hold", exp2_use_initial_ee_hold, exp2_use_initial_ee_hold);
    pnh.param("exp2_hold_ee_x", exp2_hold_ee_x, exp2_hold_ee_x);
    pnh.param("exp2_hold_ee_y", exp2_hold_ee_y, exp2_hold_ee_y);
    pnh.param("exp2_hold_ee_z", exp2_hold_ee_z, exp2_hold_ee_z);
    pnh.param("ik_fail_limit", ik_fail_limit, ik_fail_limit);
    pnh.param("base_pose_topic", base_pose_topic, base_pose_topic);

    if (circle_period <= 0.0) {
        circle_period = 10.0;
    }
    if (circle_radius < 0.0) {
        circle_radius = 0.0;
    }
    if (settle_time < 0.0) {
        settle_time = 0.0;
    }
    if (experiment_time < 0.0) {
        experiment_time = 0.0;
    }
    if (recovery_time < 0.0) {
        recovery_time = 0.0;
    }
    if (ik_fail_limit < 1) {
        ik_fail_limit = 1;
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

    ros::Subscriber base_pose_sub =
        nh.subscribe<geometry_msgs::PoseStamped>(base_pose_topic, 10, BasePoseCb);
    ros::Subscriber arm_real_sub =
        nh.subscribe<uam_message::arm_angle>("/wjl/arm/real/angle_r", 10, ArmRealCb);

    ROS_INFO("exp2 service ready, waiting for /wjl/start/uav_desired");
    ROS_INFO(
        "exp2 params: center=(%.2f, %.2f), radius=%.3f, T=%.2f, hover_z=%.2f, yaw=%.2f, settle=%.2f, exp=%.2f, recovery=%.2f, pose_topic=%s",
        circle_center_x, circle_center_y, circle_radius, circle_period, hover_z, hover_yaw_deg,
        settle_time, experiment_time, recovery_time, base_pose_topic.c_str());

    ros::Rate rate(30.0);
    while (ros::ok() && !start_flag) {
        ros::spinOnce();
        rate.sleep();
    }
    if (!ros::ok()) {
        return 0;
    }

    uav::xyz_yaw_d uav_pos_d;
    uam_message::arm_angle current_angle;
    uav_pos_d.x_d = circle_center_x;
    uav_pos_d.y_d = circle_center_y;
    uav_pos_d.z_d = hover_z;
    uav_pos_d.yaw_d = hover_yaw_deg;
    uav_pos_d.land_flag = false;

    current_angle.arm1_angle = g_real_arm_state.valid ? g_real_arm_state.arm1_deg : 0.0;
    current_angle.arm2_angle = g_real_arm_state.valid ? g_real_arm_state.arm2_deg : 0.0;
    current_angle.hand_angle = g_real_arm_state.valid ? g_real_arm_state.hand_deg : 0.0;

    current_angle.arm1_angle = Clamp(current_angle.arm1_angle, arm1_min, arm1_max);
    current_angle.arm2_angle = Clamp(current_angle.arm2_angle, arm2_min, arm2_max);
    current_angle.hand_angle = Clamp(current_angle.hand_angle, hand_min, hand_max);

    const ros::Time experiment_start = ros::Time::now();
    ros::Time last_log_time = experiment_start - ros::Duration(1.0);
    const ros::Duration landing_publish_time(0.5);
    const double two_pi = 2.0 * std::acos(-1.0);

    bool ee_hold_initialized = false;
    Vec3 ee_hold_world{exp2_hold_ee_x, exp2_hold_ee_y, exp2_hold_ee_z};
    int ik_fail_count = 0;
    int last_stage = -1;
    double last_valid_arm1_deg = current_angle.arm1_angle;
    double last_valid_arm2_deg = current_angle.arm2_angle;
    bool force_recovery = false;

    while (ros::ok()) {
        ros::spinOnce();

        const ros::Time now = ros::Time::now();
        const double elapsed = (now - experiment_start).toSec();
        int stage = 0;

        if (elapsed < settle_time) {
            stage = 0;
        } else if (!force_recovery && elapsed < settle_time + experiment_time) {
            stage = 1;
        } else if (elapsed < settle_time + experiment_time + recovery_time) {
            stage = 2;
        } else if (elapsed < settle_time + experiment_time + recovery_time +
                               landing_publish_time.toSec()) {
            stage = 3;
        } else {
            break;
        }

        if (stage != last_stage) {
            ROS_INFO("exp2 stage -> %s", StageName(stage));
            last_stage = stage;
        }

        uav_pos_d.land_flag = false;
        uav_pos_d.x_d = circle_center_x;
        uav_pos_d.y_d = circle_center_y;
        uav_pos_d.z_d = hover_z;
        uav_pos_d.yaw_d = hover_yaw_deg;
        current_angle.hand_angle = Clamp(current_angle.hand_angle, hand_min, hand_max);

        if (stage == 0) {
            current_angle.arm1_angle = last_valid_arm1_deg;
            current_angle.arm2_angle = last_valid_arm2_deg;
        } else if (stage == 1) {
            const double circle_time = elapsed - settle_time;
            const double phase = two_pi * circle_time / circle_period;
            uav_pos_d.x_d = circle_center_x + circle_radius * std::cos(phase);
            uav_pos_d.y_d = circle_center_y + circle_radius * std::sin(phase);

            if (!ee_hold_initialized) {
                if (exp2_use_initial_ee_hold) {
                    if (g_base_pose.valid) {
                        const double seed_arm1 = g_real_arm_state.valid ? g_real_arm_state.arm1_deg : last_valid_arm1_deg;
                        const double seed_arm2 = g_real_arm_state.valid ? g_real_arm_state.arm2_deg : last_valid_arm2_deg;
                        const Vec3 ee_body = ForwardKinematicsBody(seed_arm1, seed_arm2);
                        const Vec3 ee_world_offset = RotateYawOnly(ee_body, g_base_pose.yaw_rad);
                        ee_hold_world = {g_base_pose.position.x + ee_world_offset.x,
                                         g_base_pose.position.y + ee_world_offset.y,
                                         g_base_pose.position.z + ee_world_offset.z};
                        ee_hold_initialized = true;
                        ROS_INFO("exp2 hold point initialized from current EE: (%.3f, %.3f, %.3f)",
                                 ee_hold_world.x, ee_hold_world.y, ee_hold_world.z);
                    } else {
                        ROS_WARN_THROTTLE(1.0, "exp2 waiting for base pose to initialize EE hold point");
                    }
                } else {
                    ee_hold_initialized = true;
                    ROS_INFO("exp2 hold point initialized from params: (%.3f, %.3f, %.3f)",
                             ee_hold_world.x, ee_hold_world.y, ee_hold_world.z);
                }
            }

            if (!g_base_pose.valid || !ee_hold_initialized) {
                current_angle.arm1_angle = last_valid_arm1_deg;
                current_angle.arm2_angle = last_valid_arm2_deg;
                ROS_WARN_THROTTLE(1.0, "exp2 missing base pose or hold target, keeping last valid arm command");
            } else {
                const Vec3 target_offset_world{ee_hold_world.x - g_base_pose.position.x,
                                               ee_hold_world.y - g_base_pose.position.y,
                                               ee_hold_world.z - g_base_pose.position.z};
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
                    current_angle.arm1_angle = last_valid_arm1_deg;
                    current_angle.arm2_angle = last_valid_arm2_deg;
                    ROS_WARN_THROTTLE(1.0, "exp2 IK failed (%d/%d), target_body=(%.3f, %.3f, %.3f)",
                                      ik_fail_count, ik_fail_limit, target_body.x, target_body.y, target_body.z);
                    if (ik_fail_count >= ik_fail_limit) {
                        force_recovery = true;
                        ROS_ERROR("exp2 IK failed continuously, switching to recovery stage");
                    }
                } else {
                    ik_fail_count = 0;
                    current_angle.arm1_angle = solved_arm1_deg;
                    current_angle.arm2_angle = solved_arm2_deg;
                    last_valid_arm1_deg = solved_arm1_deg;
                    last_valid_arm2_deg = solved_arm2_deg;
                }
            }
        } else if (stage == 2) {
            current_angle.arm1_angle = last_valid_arm1_deg;
            current_angle.arm2_angle = last_valid_arm2_deg;
        } else if (stage == 3) {
            uav_pos_d.z_d = 0.5;
            uav_pos_d.land_flag = true;
            current_angle.arm1_angle = last_valid_arm1_deg;
            current_angle.arm2_angle = last_valid_arm2_deg;
        }

        current_angle.arm1_angle = Clamp(current_angle.arm1_angle, arm1_min, arm1_max);
        current_angle.arm2_angle = Clamp(current_angle.arm2_angle, arm2_min, arm2_max);
        current_angle.hand_angle = Clamp(current_angle.hand_angle, hand_min, hand_max);

        if ((now - last_log_time).toSec() >= 1.0) {
            last_log_time = now;
            ROS_INFO("[%s] t=%.2f s, pose_d=(%.2f, %.2f, %.2f, %.2f), arm_d=(%.2f, %.2f, %.2f), base_valid=%s, ik_fail=%d",
                     StageName(stage), elapsed, uav_pos_d.x_d, uav_pos_d.y_d, uav_pos_d.z_d,
                     uav_pos_d.yaw_d, current_angle.arm1_angle, current_angle.arm2_angle,
                     current_angle.hand_angle, g_base_pose.valid ? "true" : "false", ik_fail_count);
        }

        uav_pos_d_pub.publish(uav_pos_d);
        joint_angle_pub.publish(current_angle);
        rate.sleep();
    }

    std::cout << "exp2 finished" << std::endl;
    return 0;
}
