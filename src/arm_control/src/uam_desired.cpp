#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <locale.h>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>

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
};

struct FreezeWindowState {
    bool initialized = false;
    ros::Time stamp;
    Vec3 position{0.0, 0.0, 0.0};
};

struct TdState {
    bool initialized = false;
    Vec3 position{0.0, 0.0, 0.0};
    Vec3 velocity{0.0, 0.0, 0.0};
};

struct MavrosStateCache {
    bool valid = false;
    bool connected = false;
    bool armed = false;
    std::string mode;
};

PoseState g_local_pose;
PoseState g_tracker_pose;
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

double RoundToCentimeter(double value) {
    return std::round(value * 100.0) / 100.0;
}

Vec3 RoundVec3ToCentimeter(const Vec3 &value) {
    return {RoundToCentimeter(value.x), RoundToCentimeter(value.y), RoundToCentimeter(value.z)};
}

double DegToRad(double value_deg) {
    return value_deg * std::acos(-1.0) / 180.0;
}

Vec3 SubVec3(const Vec3 &lhs, const Vec3 &rhs) {
    return {lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

double NormVec3(const Vec3 &value) {
    return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}

void ResetTd(TdState &td, const Vec3 &position) {
    td.initialized = true;
    td.position = position;
    td.velocity = {0.0, 0.0, 0.0};
}

void UpdateTdAxis(double target, double dt, double bandwidth_rad_s, double accel_limit,
                  double vel_limit, double &position, double &velocity) {
    if (dt <= 0.0) {
        return;
    }

    const double error = target - position;
    double acceleration = bandwidth_rad_s * bandwidth_rad_s * error -
                          2.0 * bandwidth_rad_s * velocity;
    acceleration = Clamp(acceleration, -accel_limit, accel_limit);
    velocity = Clamp(velocity + acceleration * dt, -vel_limit, vel_limit);
    position += velocity * dt;
}

void UpdateTd(TdState &td, const Vec3 &target, double dt, double bandwidth_rad_s,
              double accel_limit_xy, double accel_limit_z, double vel_limit_xy,
              double vel_limit_z) {
    if (!td.initialized) {
        ResetTd(td, target);
        return;
    }

    UpdateTdAxis(target.x, dt, bandwidth_rad_s, accel_limit_xy, vel_limit_xy,
                 td.position.x, td.velocity.x);
    UpdateTdAxis(target.y, dt, bandwidth_rad_s, accel_limit_xy, vel_limit_xy,
                 td.position.y, td.velocity.y);
    UpdateTdAxis(target.z, dt, bandwidth_rad_s, accel_limit_z, vel_limit_z,
                 td.position.z, td.velocity.z);
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

const char *StageName(int stage) {
    switch (stage) {
        case 0:
            return "settle";
        case 1:
            return "disturbance";
        case 2:
            return "recovery";
        case 3:
            return "landing";
        case 4:
            return "safety_hold";
        case 5:
            return "safety_landing";
        default:
            return "unknown";
    }
}

bool doReq(uav::desired_start::Request &req, uav::desired_start::Response &resp) {
    const int desired_start = req.desired_start;
    std::cout << "收到期望数据：" << desired_start << std::endl;
    if (desired_start != 10) {
        ROS_ERROR("提交的数据异常!!!");
        return false;
    }

    resp.desired_sent = 3;
    start_flag = true;
    return true;
}

void LocalPoseCb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    g_local_pose.valid = true;
    g_local_pose.stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    g_local_pose.position.x = msg->pose.position.x;
    g_local_pose.position.y = msg->pose.position.y;
    g_local_pose.position.z = msg->pose.position.z;
}

void TrackerPoseCb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    g_tracker_pose.valid = true;
    g_tracker_pose.stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    g_tracker_pose.position.x = msg->pose.position.x;
    g_tracker_pose.position.y = msg->pose.position.y;
    g_tracker_pose.position.z = msg->pose.position.z;
}

void MavrosStateCb(const mavros_msgs::State::ConstPtr &msg) {
    g_mavros_state.valid = true;
    g_mavros_state.connected = msg->connected;
    g_mavros_state.armed = msg->armed;
    g_mavros_state.mode = msg->mode;
}

}  // namespace

int main(int argc, char *argv[]) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "uam_desired");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ros::Publisher joint_angle_pub =
        nh.advertise<uam_message::arm_angle>("/wjl/arm/guidefly/angle_d", 10);
    ros::Publisher uav_pos_d_pub =
        nh.advertise<uav::xyz_yaw_d>("/wjl/guidefly/pose_d", 10);
    ros::ServiceServer server = nh.advertiseService("/wjl/start/uav_desired", doReq);
    ros::Subscriber local_pose_sub =
        nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, LocalPoseCb);
    ros::Subscriber tracker_pose_sub =
        nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/arm_base/pose", 10, TrackerPoseCb);
    ros::Subscriber mavros_state_sub =
        nh.subscribe<mavros_msgs::State>("/mavros/state", 10, MavrosStateCb);

    // exp1 慢起飞模式下，启动后会用当前 /mavros/local_position/pose 的 x/y 作为水平锚点，只追 z 到 hover_z。
    double hover_z = 1.0;
    double hover_yaw_deg = 0.0;
    double settle_time = 3.0;
    double experiment_time = 20.0;
    double recovery_time = 2.0;
    double landing_z = 0.5;
    double arm1_offset_deg = 0.0;
    double arm1_amp_deg = 6.0;
    double arm1_period = 3.5;
    double arm1_phase_deg = 90.0;
    double arm1_sign = 1.0;
    double arm2_offset_deg = 0.0;
    double arm2_amp_deg = 15.0;
    double arm2_period = 3.5;
    double arm2_phase_deg = 0.0;
    double arm2_sign = 1.0;
    double hand_hold_deg = 0.0;
    // ============================================= TD慢参考生成器 =============================================
    // use_td_reference:
    //   true 时，位置期望先经过 TD 平滑；false 时直接发布 raw target。
    //
    // td_bandwidth_hz:
    //   TD 的等效带宽，越小参考越慢越柔；起飞仍晃时优先降到 0.18。
    //
    // td_accel_limit_xy / td_accel_limit_z:
    //   x/y 和 z 方向参考加速度上限，单位 m/s^2；值越小，给内环的指令变化越柔。
    //
    // td_vel_limit_xy / td_vel_limit_z:
    //   x/y 和 z 方向参考速度上限，单位 m/s；起飞太慢但误差小，可只提高 td_vel_limit_z。
    //
    // settle_reach_xy_m / settle_reach_z_m:
    //   settle 完成判据，实际位置与当前 TD 期望的水平/高度误差阈值，单位 m。
    //
    // settle_min_hold_s:
    //   进入误差阈值后至少稳定保持这么久，才允许进入机械臂扰动阶段，单位 s。
    bool use_td_reference = false;
    double td_bandwidth_hz = 0.22;
    double td_accel_limit_xy = 0.10;
    double td_accel_limit_z = 0.08;
    double td_vel_limit_xy = 0.12;
    double td_vel_limit_z = 0.10;
    double settle_reach_xy_m = 0.12;
    double settle_reach_z_m = 0.10;
    double settle_min_hold_s = 2.0;

    // =============================================位置更新的新鲜度保护================================================
    // pose_timeout_sec:
    //   超过这个时间没新鲜数据，就认为 arm_base/local pose 链路不可靠。
    //
    // pose_loss_limit:
    //   连续多少次检测到超时后，才真正进入 safety_hold，避免单帧抖动误触发。
    double pose_timeout_sec = 0.3;
    int pose_loss_limit = 10;

    // ==============================================数据回传丢失保护=================================================
    // ======================================= 三维总数据（模长）回传丢失保护 ===================================
    // freeze_window_sec:
    //   在这么长一段时间内，检查飞机实际位置有没有明显动。
    //   用来抓这种故障：动捕刚体丢了，但还一直发最后一帧，飞机以为自己没动，于是一直追目标。
    //
    // freeze_motion_threshold_m:
    //   在 freeze_window_sec 这个时间窗里，如果实际位置总位移小于这个值，就认为“几乎没动”。
    //
    // freeze_arrive_threshold_m:
    //   只有当“当前还没到目标点”时，冻结保护才有意义；该值定义“还没到”的位置误差下限。
    //
    // freeze_trigger_cycles:
    //   冻结故障连续命中多少次后，进入 safety_hold。
    //
    // freeze_escalate_cycles:
    //   进入 safety_hold 后，如果异常还持续，再连续多少次后升级到 safety_landing。
    double freeze_window_sec = 1.0;
    double freeze_motion_threshold_m = 0.03;
    double freeze_arrive_threshold_m = 0.15;
    int freeze_trigger_cycles = 3;
    int freeze_escalate_cycles = 6;

    // =========================================== 单Z轴数据回传丢失保护 ============================================
    // z_freeze_error_threshold_m:
    //   专门针对高度方向的冻结保护。如果 z 方向目标误差大于这个值，说明高度还差很多。
    //
    // z_freeze_motion_threshold_m:
    //   在检测时间窗里，如果实际 z 位移小于这个值，就认为高度基本没变化。
    double z_freeze_error_threshold_m = 0.20;
    double z_freeze_motion_threshold_m = 0.03;

    // ============================================= 安全姿态参数 ================================================
    // 保护触发后，机械臂回到这组安全角，停止继续给机体叠加扰动。
    double safety_hold_arm1_deg = 0.0;
    double safety_hold_arm2_deg = 0.0;
    double safety_hold_hand_deg = 0.0;

    nh.param("/wjl/uam/hover_z", hover_z, hover_z);
    pnh.param("hover_z", hover_z, hover_z);
    pnh.param("hover_yaw_deg", hover_yaw_deg, hover_yaw_deg);
    pnh.param("settle_time", settle_time, settle_time);
    pnh.param("experiment_time", experiment_time, experiment_time);
    pnh.param("recovery_time", recovery_time, recovery_time);
    pnh.param("landing_z", landing_z, landing_z);
    pnh.param("arm1_offset_deg", arm1_offset_deg, arm1_offset_deg);
    pnh.param("arm1_amp_deg", arm1_amp_deg, arm1_amp_deg);
    pnh.param("arm1_period", arm1_period, arm1_period);
    pnh.param("arm1_phase_deg", arm1_phase_deg, arm1_phase_deg);
    pnh.param("arm1_sign", arm1_sign, arm1_sign);
    pnh.param("arm2_offset_deg", arm2_offset_deg, arm2_offset_deg);
    pnh.param("arm2_amp_deg", arm2_amp_deg, arm2_amp_deg);
    pnh.param("arm2_period", arm2_period, arm2_period);
    pnh.param("arm2_phase_deg", arm2_phase_deg, arm2_phase_deg);
    pnh.param("arm2_sign", arm2_sign, arm2_sign);
    pnh.param("hand_hold_deg", hand_hold_deg, hand_hold_deg);
    pnh.param("use_td_reference", use_td_reference, use_td_reference);
    pnh.param("td_bandwidth_hz", td_bandwidth_hz, td_bandwidth_hz);
    pnh.param("td_accel_limit_xy", td_accel_limit_xy, td_accel_limit_xy);
    pnh.param("td_accel_limit_z", td_accel_limit_z, td_accel_limit_z);
    pnh.param("td_vel_limit_xy", td_vel_limit_xy, td_vel_limit_xy);
    pnh.param("td_vel_limit_z", td_vel_limit_z, td_vel_limit_z);
    pnh.param("settle_reach_xy_m", settle_reach_xy_m, settle_reach_xy_m);
    pnh.param("settle_reach_z_m", settle_reach_z_m, settle_reach_z_m);
    pnh.param("settle_min_hold_s", settle_min_hold_s, settle_min_hold_s);
    pnh.param("pose_timeout_sec", pose_timeout_sec, pose_timeout_sec);
    pnh.param("pose_loss_limit", pose_loss_limit, pose_loss_limit);
    pnh.param("freeze_window_sec", freeze_window_sec, freeze_window_sec);
    pnh.param("freeze_motion_threshold_m", freeze_motion_threshold_m, freeze_motion_threshold_m);
    pnh.param("freeze_arrive_threshold_m", freeze_arrive_threshold_m, freeze_arrive_threshold_m);
    pnh.param("freeze_trigger_cycles", freeze_trigger_cycles, freeze_trigger_cycles);
    pnh.param("freeze_escalate_cycles", freeze_escalate_cycles, freeze_escalate_cycles);
    pnh.param("z_freeze_error_threshold_m", z_freeze_error_threshold_m, z_freeze_error_threshold_m);
    pnh.param("z_freeze_motion_threshold_m", z_freeze_motion_threshold_m, z_freeze_motion_threshold_m);
    pnh.param("safety_hold_arm1_deg", safety_hold_arm1_deg, safety_hold_arm1_deg);
    pnh.param("safety_hold_arm2_deg", safety_hold_arm2_deg, safety_hold_arm2_deg);
    pnh.param("safety_hold_hand_deg", safety_hold_hand_deg, safety_hold_hand_deg);

    if (arm1_period <= 0.0) {
        ROS_WARN("参数 arm1_period<=0，已回退为默认值 3.5 s");
        arm1_period = 3.5;
    }
    if (arm2_period <= 0.0) {
        ROS_WARN("参数 arm2_period<=0，已回退为默认值 3.5 s");
        arm2_period = 3.5;
    }
    if (settle_time < 0.0) {
        settle_time = 0.0;
    }
    if (td_bandwidth_hz <= 0.0) {
        td_bandwidth_hz = 0.22;
    }
    if (td_accel_limit_xy <= 0.0) {
        td_accel_limit_xy = 0.10;
    }
    if (td_accel_limit_z <= 0.0) {
        td_accel_limit_z = 0.08;
    }
    if (td_vel_limit_xy <= 0.0) {
        td_vel_limit_xy = 0.12;
    }
    if (td_vel_limit_z <= 0.0) {
        td_vel_limit_z = 0.10;
    }
    if (settle_reach_xy_m <= 0.0) {
        settle_reach_xy_m = 0.12;
    }
    if (settle_reach_z_m <= 0.0) {
        settle_reach_z_m = 0.10;
    }
    if (settle_min_hold_s < 0.0) {
        settle_min_hold_s = 0.0;
    }
    if (experiment_time < 0.0) {
        experiment_time = 0.0;
    }
    if (recovery_time < 0.0) {
        recovery_time = 0.0;
    }
    if (pose_timeout_sec <= 0.0) {
        pose_timeout_sec = 0.3;
    }
    if (pose_loss_limit < 1) {
        pose_loss_limit = 1;
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

    arm1_offset_deg = Clamp(arm1_offset_deg, arm1_min, arm1_max);
    arm2_offset_deg = Clamp(arm2_offset_deg, arm2_min, arm2_max);
    hand_hold_deg = Clamp(hand_hold_deg, hand_min, hand_max);
    safety_hold_arm1_deg = Clamp(safety_hold_arm1_deg, arm1_min, arm1_max);
    safety_hold_arm2_deg = Clamp(safety_hold_arm2_deg, arm2_min, arm2_max);
    safety_hold_hand_deg = Clamp(safety_hold_hand_deg, hand_min, hand_max);

    ROS_INFO("服务已经启动，等待实验触发....");
    ROS_INFO(
        "uam_desired params: hover_z=%.2f, yaw=%.2f deg, settle=%.2f s, exp=%.2f s, recovery=%.2f s, pose_timeout=%.2f s, pose_loss_limit=%d, freeze_window=%.2f s",
        hover_z, hover_yaw_deg, settle_time, experiment_time, recovery_time,
        pose_timeout_sec, pose_loss_limit, freeze_window_sec);
    ROS_INFO("exp1 slow takeoff: x/y will anchor at current local pose; only z tracks hover_z");
    ROS_INFO(
        "uam_desired TD: enabled=%s, bw=%.2f Hz, acc_xy=%.3f, acc_z=%.3f, vel_xy=%.3f, vel_z=%.3f, settle_reach=(xy %.3f, z %.3f), settle_hold=%.2f s",
        use_td_reference ? "true" : "false", td_bandwidth_hz, td_accel_limit_xy,
        td_accel_limit_z, td_vel_limit_xy, td_vel_limit_z, settle_reach_xy_m,
        settle_reach_z_m, settle_min_hold_s);

    ros::Rate rate(30.0);
    while (ros::ok() && !start_flag) {
        ros::spinOnce();
        rate.sleep();
    }
    if (!ros::ok()) {
        return 0;
    }

    std::cout << "开始发送期望数据：" << std::endl;

    while (ros::ok() && !IsPoseFresh(g_local_pose, ros::Time::now(), pose_timeout_sec)) {
        ROS_WARN_THROTTLE(1.0, "exp1 waiting for fresh local pose before TD initialization");
        ros::spinOnce();
        rate.sleep();
    }
    if (!ros::ok()) {
        return 0;
    }

    const Vec3 takeoff_initial_position = RoundVec3ToCentimeter(g_local_pose.position);
    const Vec3 takeoff_hover_target{takeoff_initial_position.x, takeoff_initial_position.y, hover_z};
    ROS_INFO("exp1 takeoff anchor: current local=(%.3f, %.3f, %.3f), target=(%.3f, %.3f, %.3f)",
             takeoff_initial_position.x, takeoff_initial_position.y, takeoff_initial_position.z,
             takeoff_hover_target.x, takeoff_hover_target.y, takeoff_hover_target.z);
    const double td_bandwidth_rad_s = 2.0 * std::acos(-1.0) * td_bandwidth_hz;
    TdState td;
    ResetTd(td, takeoff_initial_position);

    uav::xyz_yaw_d uav_pos_d;
    uam_message::arm_angle current_angle;
    uav_pos_d.x_d = takeoff_initial_position.x;
    uav_pos_d.y_d = takeoff_initial_position.y;
    uav_pos_d.z_d = takeoff_initial_position.z;
    uav_pos_d.yaw_d = hover_yaw_deg;
    uav_pos_d.land_flag = false;
    current_angle.arm1_angle = arm1_offset_deg;
    current_angle.arm2_angle = arm2_offset_deg;
    current_angle.hand_angle = hand_hold_deg;

    const ros::Duration landing_publish_time(0.5);
    const ros::Duration return_home_publish_time(1.0);
    const double two_pi = 2.0 * std::acos(-1.0);
    const double arm1_phase_rad = DegToRad(arm1_phase_deg);
    const double arm2_phase_rad = DegToRad(arm2_phase_deg);
    const ros::Time experiment_start = ros::Time::now();
    ros::Time last_loop_time = experiment_start;
    ros::Time last_log_time = experiment_start - ros::Duration(1.0);

    bool protection_triggered = false;
    bool safety_landing_triggered = false;
    bool settle_complete = false;
    ros::Time settle_reached_since;
    ros::Time post_settle_start;
    ros::Time protection_state_start;
    std::string protection_reason = "none";
    int pose_loss_count = 0;
    int freeze_fault_count = 0;
    int protection_escalate_count = 0;
    FreezeWindowState freeze_window_state;
    uav::xyz_yaw_d last_safe_uav_pos_d = uav_pos_d;

    auto PublishReturnHome = [&](double home_arm1_deg, double home_arm2_deg, double home_hand_deg) {
        if (!ros::ok()) {
            return;
        }
        uam_message::arm_angle home_angle;
        home_angle.arm1_angle = Clamp(home_arm1_deg, arm1_min, arm1_max);
        home_angle.arm2_angle = Clamp(home_arm2_deg, arm2_min, arm2_max);
        home_angle.hand_angle = Clamp(home_hand_deg, hand_min, hand_max);
        ROS_INFO("uam_desired return arm to home: arm_d=(%.2f, %.2f, %.2f)",
                 home_angle.arm1_angle, home_angle.arm2_angle, home_angle.hand_angle);
        ros::Rate return_rate(30.0);
        const ros::Time return_start = ros::Time::now();
        while (ros::ok() && (ros::Time::now() - return_start) < return_home_publish_time) {
            joint_angle_pub.publish(home_angle);
            ros::spinOnce();
            return_rate.sleep();
        }
    };

    auto enter_protection = [&](const ros::Time &now, const std::string &reason) {
        if (protection_triggered) {
            return;
        }
        protection_triggered = true;
        protection_state_start = now;
        protection_reason = reason;
        protection_escalate_count = 0;
        ROS_ERROR("exp1 protection triggered: %s", protection_reason.c_str());
    };

    while (ros::ok()) {
        ros::spinOnce();

        const ros::Time now = ros::Time::now();
        const double dt = std::max(0.0, (now - last_loop_time).toSec());
        last_loop_time = now;
        const double elapsed = (now - experiment_start).toSec();
        const bool local_fresh = IsPoseFresh(g_local_pose, now, pose_timeout_sec);
        const bool tracker_fresh = IsPoseFresh(g_tracker_pose, now, pose_timeout_sec);
        int stage = 0;

        if (protection_triggered) {
            stage = safety_landing_triggered ? 5 : 4;
        } else if (!settle_complete) {
            stage = 0;
        } else {
            const double post_settle_elapsed = (now - post_settle_start).toSec();
            if (post_settle_elapsed < experiment_time) {
                stage = 1;
            } else if (post_settle_elapsed < experiment_time + recovery_time) {
                stage = 2;
            } else if (post_settle_elapsed < experiment_time + recovery_time +
                                      landing_publish_time.toSec()) {
                stage = 3;
            } else {
                break;
            }
        }

        Vec3 raw_target = takeoff_hover_target;
        uav_pos_d.yaw_d = hover_yaw_deg;
        uav_pos_d.land_flag = false;
        current_angle.arm1_angle = arm1_offset_deg;
        current_angle.arm2_angle = arm2_offset_deg;
        current_angle.hand_angle = hand_hold_deg;

        if (stage == 1) {
            const double disturb_time = settle_complete ? (now - post_settle_start).toSec() : 0.0;
            current_angle.arm1_angle =
                arm1_offset_deg +
                arm1_sign * arm1_amp_deg *
                    std::sin(two_pi * disturb_time / arm1_period + arm1_phase_rad);
            current_angle.arm2_angle =
                arm2_offset_deg +
                arm2_sign * arm2_amp_deg *
                    std::sin(two_pi * disturb_time / arm2_period + arm2_phase_rad);
        } else if (stage == 3) {
            uav_pos_d.z_d = landing_z;
            uav_pos_d.land_flag = true;
        } else if (stage == 4) {
            uav_pos_d = last_safe_uav_pos_d;
            uav_pos_d.land_flag = false;
            current_angle.arm1_angle = safety_hold_arm1_deg;
            current_angle.arm2_angle = safety_hold_arm2_deg;
            current_angle.hand_angle = safety_hold_hand_deg;
        } else if (stage == 5) {
            uav_pos_d = last_safe_uav_pos_d;
            uav_pos_d.z_d = landing_z;
            uav_pos_d.land_flag = true;
            current_angle.arm1_angle = safety_hold_arm1_deg;
            current_angle.arm2_angle = safety_hold_arm2_deg;
            current_angle.hand_angle = safety_hold_hand_deg;
            if (!protection_state_start.isZero() &&
                (now - protection_state_start) > landing_publish_time) {
                break;
            }
        }

        if (stage <= 2 && !protection_triggered) {
            if (use_td_reference) {
                UpdateTd(td, raw_target, dt, td_bandwidth_rad_s, td_accel_limit_xy,
                         td_accel_limit_z, td_vel_limit_xy, td_vel_limit_z);
                uav_pos_d.x_d = td.position.x;
                uav_pos_d.y_d = td.position.y;
                uav_pos_d.z_d = td.position.z;
            } else {
                ResetTd(td, raw_target);
                uav_pos_d.x_d = raw_target.x;
                uav_pos_d.y_d = raw_target.y;
                uav_pos_d.z_d = raw_target.z;
            }
        }

        if (!protection_triggered && stage == 0 && local_fresh && tracker_fresh) {
            const double xy_error = std::sqrt(
                (uav_pos_d.x_d - g_local_pose.position.x) *
                    (uav_pos_d.x_d - g_local_pose.position.x) +
                (uav_pos_d.y_d - g_local_pose.position.y) *
                    (uav_pos_d.y_d - g_local_pose.position.y));
            const double z_error = std::fabs(uav_pos_d.z_d - g_local_pose.position.z);
            const bool reached_settle =
                xy_error <= settle_reach_xy_m && z_error <= settle_reach_z_m;

            if (reached_settle) {
                if (settle_reached_since.isZero()) {
                    settle_reached_since = now;
                }
            } else {
                settle_reached_since = ros::Time();
            }

            if (elapsed >= settle_time && !settle_reached_since.isZero() &&
                (now - settle_reached_since).toSec() >= settle_min_hold_s) {
                settle_complete = true;
                post_settle_start = now;
                ROS_INFO("exp1 settle complete: xy_error=%.3f, z_error=%.3f, elapsed=%.2f s",
                         xy_error, z_error, elapsed);
            }
        } else if (!protection_triggered && stage == 0) {
            settle_reached_since = ros::Time();
        }

        bool freshness_fault = false;
        bool freeze_fault_active = false;
        if (!protection_triggered && stage <= 2) {
            // freshness 故障模型：
            // local pose 或 arm_base 长时间不更新时，不能继续让实验主段推进。
            freshness_fault =
                !local_fresh ||
                !tracker_fresh ||
                (g_mavros_state.valid && !g_mavros_state.connected);
            if (freshness_fault) {
                ++pose_loss_count;
                ROS_WARN_THROTTLE(
                    1.0,
                    "exp1 freshness fault: local_fresh=%s, base_fresh=%s, mavros_connected=%s (%d/%d)",
                    local_fresh ? "true" : "false", tracker_fresh ? "true" : "false",
                    g_mavros_state.connected ? "true" : "false", pose_loss_count, pose_loss_limit);
                if (pose_loss_count >= pose_loss_limit) {
                    enter_protection(now, "arm_base/local pose freshness lost");
                }
            } else {
                pose_loss_count = 0;
                last_safe_uav_pos_d = uav_pos_d;
            }

            // 冻结故障模型：
            // 动捕刚体可能丢失后仍持续发最后一帧，这时 fresh 仍可能为 true，
            // 但飞机位置长时间不动且目标误差持续存在，就要停掉实验主段。
            if (local_fresh) {
                if (!freeze_window_state.initialized) {
                    freeze_window_state.initialized = true;
                    freeze_window_state.stamp = now;
                    freeze_window_state.position = g_local_pose.position;
                } else if ((now - freeze_window_state.stamp).toSec() >= freeze_window_sec) {
                    const Vec3 motion = SubVec3(g_local_pose.position, freeze_window_state.position);
                    const double motion_norm = NormVec3(motion);
                    const Vec3 error = SubVec3(
                        Vec3{uav_pos_d.x_d, uav_pos_d.y_d, uav_pos_d.z_d},
                        g_local_pose.position);
                    const double pos_error_norm = NormVec3(error);
                    const double z_error = std::fabs(error.z);
                    const double z_motion =
                        std::fabs(g_local_pose.position.z - freeze_window_state.position.z);

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
                            "exp1 freeze guard: pos_error=%.3f, motion=%.3f, z_error=%.3f, z_motion=%.3f (%d/%d)",
                            pos_error_norm, motion_norm, z_error, z_motion, freeze_fault_count,
                            freeze_trigger_cycles);
                        if (freeze_fault_count >= freeze_trigger_cycles) {
                            enter_protection(now, "local pose frozen before target reached");
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
        }

        if (protection_triggered && !safety_landing_triggered) {
            current_angle.arm1_angle = safety_hold_arm1_deg;
            current_angle.arm2_angle = safety_hold_arm2_deg;
            current_angle.hand_angle = safety_hold_hand_deg;
            uav_pos_d = last_safe_uav_pos_d;
            uav_pos_d.land_flag = false;

            bool persistent_fault =
                !local_fresh ||
                !tracker_fresh ||
                (g_mavros_state.valid && !g_mavros_state.connected);
            if (local_fresh) {
                if (!freeze_window_state.initialized) {
                    freeze_window_state.initialized = true;
                    freeze_window_state.stamp = now;
                    freeze_window_state.position = g_local_pose.position;
                } else if ((now - freeze_window_state.stamp).toSec() >= freeze_window_sec) {
                    const Vec3 motion = SubVec3(g_local_pose.position, freeze_window_state.position);
                    const double motion_norm = NormVec3(motion);
                    const Vec3 error = SubVec3(
                        Vec3{last_safe_uav_pos_d.x_d, last_safe_uav_pos_d.y_d, last_safe_uav_pos_d.z_d},
                        g_local_pose.position);
                    const double pos_error_norm = NormVec3(error);
                    const double z_error = std::fabs(error.z);
                    const double z_motion =
                        std::fabs(g_local_pose.position.z - freeze_window_state.position.z);
                    persistent_fault =
                        persistent_fault ||
                        (pos_error_norm > freeze_arrive_threshold_m &&
                         motion_norm < freeze_motion_threshold_m) ||
                        (z_error > z_freeze_error_threshold_m &&
                         z_motion < z_freeze_motion_threshold_m);
                    freeze_window_state.stamp = now;
                    freeze_window_state.position = g_local_pose.position;
                }
            } else {
                freeze_window_state.initialized = false;
            }

            if (persistent_fault) {
                ++protection_escalate_count;
                if (protection_escalate_count >= freeze_escalate_cycles) {
                    safety_landing_triggered = true;
                    protection_state_start = now;
                    ROS_ERROR("exp1 protection escalated to safety_landing: %s",
                              protection_reason.c_str());
                }
            } else {
                protection_escalate_count = 0;
            }
        }

        current_angle.arm1_angle = Clamp(current_angle.arm1_angle, arm1_min, arm1_max);
        current_angle.arm2_angle = Clamp(current_angle.arm2_angle, arm2_min, arm2_max);
        current_angle.hand_angle = Clamp(current_angle.hand_angle, hand_min, hand_max);

        if ((now - last_log_time).toSec() >= 1.0) {
            last_log_time = now;
            ROS_INFO(
                "[%s] t=%.2f s, pose_d=(%.2f, %.2f, %.2f, %.2f), arm_d=(%.2f, %.2f, %.2f), local_fresh=%s, tracker_fresh=%s, land=%s",
                StageName(stage), elapsed, uav_pos_d.x_d, uav_pos_d.y_d, uav_pos_d.z_d,
                uav_pos_d.yaw_d, current_angle.arm1_angle, current_angle.arm2_angle,
                current_angle.hand_angle, local_fresh ? "true" : "false",
                tracker_fresh ? "true" : "false", uav_pos_d.land_flag ? "true" : "false");
        }

        uav_pos_d_pub.publish(uav_pos_d);
        joint_angle_pub.publish(current_angle);
        rate.sleep();
    }

    PublishReturnHome(0.0, 0.0, hand_hold_deg);
    std::cout << "结束飞行！！！" << std::endl;
    return 0;
}
