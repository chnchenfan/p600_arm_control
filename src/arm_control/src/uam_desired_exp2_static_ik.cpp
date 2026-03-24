#include <ros/ros.h>

#include <cmath>
#include <string>

#include <geometry_msgs/PoseStamped.h>

#include "uam_message/arm_angle.h"

namespace {

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

PoseState g_base_pose;
PoseState g_ee_pose;
ArmState g_real_arm_state;

inline double Pi() { return std::acos(-1.0); }

double Clamp(double value, double min_value, double max_value) {
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

double DegToRad(double value_deg) { return value_deg * Pi() / 180.0; }
double RadToDeg(double value_rad) { return value_rad * 180.0 / Pi(); }

double WrapToPi(double angle_rad) {
    const double two_pi = 2.0 * Pi();
    while (angle_rad > Pi()) angle_rad -= two_pi;
    while (angle_rad < -Pi()) angle_rad += two_pi;
    return angle_rad;
}

double AdjustNearReference(double angle_rad, double reference_rad) {
    double adjusted = angle_rad;
    const double two_pi = 2.0 * Pi();
    while (adjusted - reference_rad > Pi()) adjusted -= two_pi;
    while (adjusted - reference_rad < -Pi()) adjusted += two_pi;
    return adjusted;
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

double NormVec3(const Vec3 &value) {
    return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}

Vec3 ClampVec3Norm(const Vec3 &value, double max_norm) {
    if (max_norm <= 0.0) return {0.0, 0.0, 0.0};
    const double norm = NormVec3(value);
    if (norm <= max_norm || norm < 1e-9) return value;
    return ScaleVec3(value, max_norm / norm);
}

double RateLimit(double target, double current, double max_delta) {
    if (max_delta <= 0.0) return target;
    return Clamp(target, current - max_delta, current + max_delta);
}

void NormalizeQuaternion(double *qx, double *qy, double *qz, double *qw) {
    const double norm = std::sqrt((*qx) * (*qx) + (*qy) * (*qy) + (*qz) * (*qz) + (*qw) * (*qw));
    if (norm < 1e-9) {
        *qx = 0.0; *qy = 0.0; *qz = 0.0; *qw = 1.0;
        return;
    }
    *qx /= norm; *qy /= norm; *qz /= norm; *qw /= norm;
}

Vec3 RotateBodyToWorld(const Vec3 &vector_body, const PoseState &pose) {
    double qx = pose.qx, qy = pose.qy, qz = pose.qz, qw = pose.qw;
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

Vec3 RotateWorldToBody(const Vec3 &vector_world, const PoseState &pose) {
    double qx = pose.qx, qy = pose.qy, qz = pose.qz, qw = pose.qw;
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
        r00 * vector_world.x + r10 * vector_world.y + r20 * vector_world.z,
        r01 * vector_world.x + r11 * vector_world.y + r21 * vector_world.z,
        r02 * vector_world.x + r12 * vector_world.y + r22 * vector_world.z,
    };
}

Vec3 GetCorrectedBaseWorldPosition(const PoseState &base_pose, const Vec3 &arm_base_offset_body_m) {
    return SubVec3(base_pose.position, RotateBodyToWorld(arm_base_offset_body_m, base_pose));
}

bool IsPoseFresh(const PoseState &pose, const ros::Time &now, double timeout_sec) {
    if (!pose.valid) return false;
    if (timeout_sec <= 0.0) return true;
    return (now - pose.stamp).toSec() <= timeout_sec;
}

bool SolveInverseKinematics(const Vec3 &target_body,
                            double link_length,
                            double previous_arm1_deg,
                            double previous_arm2_deg,
                            double arm1_zero_offset_deg,
                            double arm2_zero_offset_deg,
                            double arm1_min_deg,
                            double arm1_max_deg,
                            double arm2_min_deg,
                            double arm2_max_deg,
                            double eps_r,
                            double eps_xy,
                            double *arm1_deg_out,
                            double *arm2_deg_out,
                            double *reach_error_out) {
    const double radius = NormVec3(target_body);
    const double reach_error = std::fabs(radius - link_length);
    if (reach_error_out != nullptr) *reach_error_out = reach_error;
    if (radius < 1e-9 || reach_error > eps_r) return false;
    const double horizontal_radius = std::sqrt(target_body.x * target_body.x + target_body.y * target_body.y);
    if (horizontal_radius < eps_xy) return false;

    const double q1_reference = DegToRad(previous_arm1_deg);
    const double q2_reference = DegToRad(previous_arm2_deg);
    const double q1_geometry = std::atan2(-target_body.y, target_body.x);
    const double q2_geometry = std::atan2(target_body.z, horizontal_radius);
    double q1_candidate = q1_geometry - DegToRad(arm1_zero_offset_deg);
    double q2_candidate = q2_geometry - DegToRad(arm2_zero_offset_deg);
    q1_candidate = AdjustNearReference(q1_candidate, q1_reference);
    q2_candidate = AdjustNearReference(q2_candidate, q2_reference);

    const double arm1_deg = RadToDeg(q1_candidate);
    const double arm2_deg = RadToDeg(q2_candidate);
    if (arm1_deg < arm1_min_deg || arm1_deg > arm1_max_deg) return false;
    if (arm2_deg < arm2_min_deg || arm2_deg > arm2_max_deg) return false;
    *arm1_deg_out = arm1_deg;
    *arm2_deg_out = arm2_deg;
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

void BasePoseCb(const geometry_msgs::PoseStamped::ConstPtr &msg) { FillPoseState(msg, &g_base_pose); }
void EePoseCb(const geometry_msgs::PoseStamped::ConstPtr &msg) { FillPoseState(msg, &g_ee_pose); }
void ArmRealCb(const uam_message::arm_angle::ConstPtr &msg) {
    g_real_arm_state.valid = true;
    g_real_arm_state.arm1_deg = msg->arm1_angle;
    g_real_arm_state.arm2_deg = msg->arm2_angle;
    g_real_arm_state.hand_deg = msg->hand_angle;
}

}  // namespace

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "uam_desired_exp2_static_ik");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ros::Publisher joint_angle_pub = nh.advertise<uam_message::arm_angle>("/wjl/arm/guidefly/angle_d", 10);
    ros::Subscriber base_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/arm_base/pose", 10, BasePoseCb);
    ros::Subscriber ee_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/arm_target/pose", 10, EePoseCb);
    ros::Subscriber arm_real_sub = nh.subscribe<uam_message::arm_angle>("/wjl/arm/real/angle_r", 10, ArmRealCb);

    double settle_time = 3.0;
    double test_duration_sec = 20.0;
    double l1_m = 0.161;
    double l2_m = 0.262;
    double ee_outer_kp = 0.25;
    double ee_outer_clip_m = 0.03;
    double arm1_zero_offset_deg = 0.0;
    double arm2_zero_offset_deg = 0.0;
    double pose_timeout_sec = 0.3;
    double eps_r_m = 0.01;
    double eps_xy_m = 0.015;
    int ik_fail_limit = 10;
    int pose_loss_limit = 10;
    double arm1_dot_max_deg_s = 90.0;
    double arm2_dot_max_deg_s = 90.0;
    double hold_freeze_samples = 5.0;
    bool use_initial_ee_hold = true;
    double explicit_hold_x = 0.0;
    double explicit_hold_y = 0.0;
    double explicit_hold_z = 0.0;
    double arm_base_offset_x_m = 0.0;
    double arm_base_offset_y_m = 0.0;
    double arm_base_offset_z_m = 0.0;
    std::string base_pose_topic = "/vrpn_client_node/arm_base/pose";
    std::string ee_pose_topic = "/vrpn_client_node/arm_target/pose";

    pnh.param("settle_time", settle_time, settle_time);
    pnh.param("test_duration_sec", test_duration_sec, test_duration_sec);
    pnh.param("L1_m", l1_m, l1_m);
    pnh.param("L2_m", l2_m, l2_m);
    pnh.param("ee_outer_kp", ee_outer_kp, ee_outer_kp);
    pnh.param("ee_outer_clip_m", ee_outer_clip_m, ee_outer_clip_m);
    pnh.param("arm1_zero_offset_deg", arm1_zero_offset_deg, arm1_zero_offset_deg);
    pnh.param("arm2_zero_offset_deg", arm2_zero_offset_deg, arm2_zero_offset_deg);
    pnh.param("pose_timeout_sec", pose_timeout_sec, pose_timeout_sec);
    pnh.param("eps_r_m", eps_r_m, eps_r_m);
    pnh.param("eps_xy_m", eps_xy_m, eps_xy_m);
    pnh.param("ik_fail_limit", ik_fail_limit, ik_fail_limit);
    pnh.param("pose_loss_limit", pose_loss_limit, pose_loss_limit);
    pnh.param("arm1_dot_max_deg_s", arm1_dot_max_deg_s, arm1_dot_max_deg_s);
    pnh.param("arm2_dot_max_deg_s", arm2_dot_max_deg_s, arm2_dot_max_deg_s);
    pnh.param("hold_freeze_samples", hold_freeze_samples, hold_freeze_samples);
    pnh.param("use_initial_ee_hold", use_initial_ee_hold, use_initial_ee_hold);
    pnh.param("exp2_hold_ee_x", explicit_hold_x, explicit_hold_x);
    pnh.param("exp2_hold_ee_y", explicit_hold_y, explicit_hold_y);
    pnh.param("exp2_hold_ee_z", explicit_hold_z, explicit_hold_z);
    pnh.param("arm_base_offset_x_m", arm_base_offset_x_m, arm_base_offset_x_m);
    pnh.param("arm_base_offset_y_m", arm_base_offset_y_m, arm_base_offset_y_m);
    pnh.param("arm_base_offset_z_m", arm_base_offset_z_m, arm_base_offset_z_m);
    pnh.param("base_pose_topic", base_pose_topic, base_pose_topic);
    pnh.param("ee_pose_topic", ee_pose_topic, ee_pose_topic);

    double arm1_min = -180.0, arm1_max = 180.0, arm2_min = -180.0, arm2_max = 60.0, hand_min = -15.0, hand_max = 25.0;
    nh.param("/arm/arm_joint1/min", arm1_min, arm1_min);
    nh.param("/arm/arm_joint1/max", arm1_max, arm1_max);
    nh.param("/arm/arm_joint2/min", arm2_min, arm2_min);
    nh.param("/arm/arm_joint2/max", arm2_max, arm2_max);
    nh.param("/arm/left_hand_joint/min", hand_min, hand_min);
    nh.param("/arm/left_hand_joint/max", hand_max, hand_max);

    // 覆盖订阅以支持 launch 改 topic。
    base_pose_sub.shutdown();
    ee_pose_sub.shutdown();
    base_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(base_pose_topic, 10, BasePoseCb);
    ee_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(ee_pose_topic, 10, EePoseCb);

    ROS_INFO("exp2 static IK test ready: base_topic=%s, ee_topic=%s, L1=%.3f, L2=%.3f, base_offset=[%.3f, %.3f, %.3f], settle=%.2f, test=%.2f",
             base_pose_topic.c_str(), ee_pose_topic.c_str(), l1_m, l2_m,
             arm_base_offset_x_m, arm_base_offset_y_m, arm_base_offset_z_m, settle_time, test_duration_sec);

    uam_message::arm_angle current_angle;
    current_angle.arm1_angle = 0.0;
    current_angle.arm2_angle = 0.0;
    current_angle.hand_angle = 0.0;
    bool hold_initialized = false;
    Vec3 ee_hold_world{explicit_hold_x, explicit_hold_y, explicit_hold_z};
    Vec3 hold_accumulator{0.0, 0.0, 0.0};
    int hold_sample_count = 0;
    int ik_fail_count = 0;
    int pose_loss_count = 0;
    double last_valid_arm1_deg = 0.0;
    double last_valid_arm2_deg = 0.0;
    const Vec3 arm_base_offset_body_m{arm_base_offset_x_m, arm_base_offset_y_m, arm_base_offset_z_m};
    bool test_started = false;
    ros::Time start_time = ros::Time::now();
    ros::Time test_start;
    ros::Time last_log_time = start_time - ros::Duration(1.0);

    ros::Rate rate(30.0);
    while (ros::ok()) {
        ros::spinOnce();
        const ros::Time now = ros::Time::now();

        if (g_real_arm_state.valid) {
            current_angle.hand_angle = Clamp(g_real_arm_state.hand_deg, hand_min, hand_max);
            last_valid_arm1_deg = Clamp(g_real_arm_state.arm1_deg, arm1_min, arm1_max);
            last_valid_arm2_deg = Clamp(g_real_arm_state.arm2_deg, arm2_min, arm2_max);
            if (!test_started) {
                current_angle.arm1_angle = last_valid_arm1_deg;
                current_angle.arm2_angle = last_valid_arm2_deg;
            }
        }

        const bool base_fresh = IsPoseFresh(g_base_pose, now, pose_timeout_sec);
        const bool ee_fresh = IsPoseFresh(g_ee_pose, now, pose_timeout_sec);

        if (!hold_initialized) {
            if (use_initial_ee_hold) {
                if (ee_fresh) {
                    hold_accumulator = AddVec3(hold_accumulator, g_ee_pose.position);
                    ++hold_sample_count;
                    if (hold_sample_count >= static_cast<int>(hold_freeze_samples)) {
                        ee_hold_world = ScaleVec3(hold_accumulator, 1.0 / static_cast<double>(hold_sample_count));
                        hold_initialized = true;
                        ROS_INFO("exp2 static IK hold point frozen from arm_target: (%.3f, %.3f, %.3f)", ee_hold_world.x, ee_hold_world.y, ee_hold_world.z);
                    }
                }
            } else {
                hold_initialized = true;
                ROS_INFO("exp2 static IK hold point set explicitly: (%.3f, %.3f, %.3f)", ee_hold_world.x, ee_hold_world.y, ee_hold_world.z);
            }
        }

        if (!test_started) {
            joint_angle_pub.publish(current_angle);
            if ((now - start_time).toSec() >= settle_time && hold_initialized && base_fresh && ee_fresh && g_real_arm_state.valid) {
                test_started = true;
                test_start = now;
                ROS_INFO("exp2 static IK test started");
            } else {
                ROS_WARN_THROTTLE(1.0, "exp2 static IK waiting: base_fresh=%s, ee_fresh=%s, arm_fresh=%s, hold_ready=%s",
                                  base_fresh ? "true" : "false",
                                  ee_fresh ? "true" : "false",
                                  g_real_arm_state.valid ? "true" : "false",
                                  hold_initialized ? "true" : "false");
            }
            rate.sleep();
            continue;
        }

        double reach_error = -1.0;
        if (!base_fresh || !ee_fresh || !g_real_arm_state.valid) {
            ++pose_loss_count;
            ROS_WARN_THROTTLE(1.0, "exp2 static IK waiting for fresh data, base_fresh=%s, ee_fresh=%s, arm_fresh=%s (%d/%d)",
                              base_fresh ? "true" : "false", ee_fresh ? "true" : "false", g_real_arm_state.valid ? "true" : "false",
                              pose_loss_count, pose_loss_limit);
            if (pose_loss_count >= pose_loss_limit) {
                ROS_ERROR("exp2 static IK lost required feedback continuously, aborting test");
                return 1;
            }
        } else {
            pose_loss_count = 0;
            const Vec3 base_world_position = GetCorrectedBaseWorldPosition(g_base_pose, arm_base_offset_body_m);
            const Vec3 hold_offset_world = SubVec3(ee_hold_world, base_world_position);
            const Vec3 p_hold_body = RotateWorldToBody(hold_offset_world, g_base_pose);
            const Vec3 p_rel = SubVec3(p_hold_body, Vec3{0.0, 0.0, -l1_m});
            const Vec3 ee_error_world = SubVec3(ee_hold_world, g_ee_pose.position);
            const Vec3 correction_world = ClampVec3Norm(ScaleVec3(ee_error_world, ee_outer_kp), ee_outer_clip_m);
            const Vec3 correction_body = RotateWorldToBody(correction_world, g_base_pose);
            const Vec3 p_rel_corrected = AddVec3(p_rel, correction_body);

            double solved_arm1_deg = last_valid_arm1_deg;
            double solved_arm2_deg = last_valid_arm2_deg;
            const bool solved = SolveInverseKinematics(p_rel_corrected,
                                                       l2_m,
                                                       last_valid_arm1_deg,
                                                       last_valid_arm2_deg,
                                                       arm1_zero_offset_deg,
                                                       arm2_zero_offset_deg,
                                                       arm1_min,
                                                       arm1_max,
                                                       arm2_min,
                                                       arm2_max,
                                                       eps_r_m,
                                                       eps_xy_m,
                                                       &solved_arm1_deg,
                                                       &solved_arm2_deg,
                                                       &reach_error);
            if (!solved) {
                ++ik_fail_count;
                ROS_WARN_THROTTLE(1.0, "exp2 static IK failed (%d/%d), target=(%.3f, %.3f, %.3f), reach_error=%.4f",
                                  ik_fail_count, ik_fail_limit, p_rel_corrected.x, p_rel_corrected.y, p_rel_corrected.z, reach_error);
                if (ik_fail_count >= ik_fail_limit) {
                    ROS_ERROR("exp2 static IK failed continuously, aborting test");
                    return 1;
                }
            } else {
                ik_fail_count = 0;
                const double dt = 1.0 / 30.0;
                solved_arm1_deg = RateLimit(solved_arm1_deg, last_valid_arm1_deg, arm1_dot_max_deg_s * dt);
                solved_arm2_deg = RateLimit(solved_arm2_deg, last_valid_arm2_deg, arm2_dot_max_deg_s * dt);
                current_angle.arm1_angle = Clamp(solved_arm1_deg, arm1_min, arm1_max);
                current_angle.arm2_angle = Clamp(solved_arm2_deg, arm2_min, arm2_max);
                last_valid_arm1_deg = current_angle.arm1_angle;
                last_valid_arm2_deg = current_angle.arm2_angle;
            }
        }

        joint_angle_pub.publish(current_angle);

        if ((now - last_log_time).toSec() >= 1.0) {
            last_log_time = now;
            ROS_INFO("[exp2_static_ik] t=%.2f s, arm_d=(%.2f, %.2f, %.2f), base_fresh=%s, ee_fresh=%s, reach_error=%.4f, pose_loss=%d, ik_fail=%d, hold=(%.3f, %.3f, %.3f)",
                     (now - test_start).toSec(), current_angle.arm1_angle, current_angle.arm2_angle, current_angle.hand_angle,
                     base_fresh ? "true" : "false", ee_fresh ? "true" : "false", reach_error, pose_loss_count, ik_fail_count,
                     ee_hold_world.x, ee_hold_world.y, ee_hold_world.z);
        }

        if ((now - test_start).toSec() >= test_duration_sec) {
            ROS_INFO("exp2 static IK test finished successfully after %.2f s", test_duration_sec);
            return 0;
        }

        rate.sleep();
    }
    return 0;
}
