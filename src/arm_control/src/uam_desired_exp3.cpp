#include <ros/ros.h>

#include <cmath>
#include <iostream>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>

#include "uam_message/arm_angle.h"
#include "uav/desired_start.h"
#include "uav/xyz_yaw_d.h"

namespace {

// 与实验一/二保持一致：
// 先由 UAV 侧完成解锁、起飞和到达起始高度，
// 然后通过 /wjl/start/uav_desired 触发实验三的期望发布。
bool start_flag = false;

// 这里只缓存无人机当前“实际基座位置”。
// 实验三不会像实验二那样用它做在线逆解，只用它来冻结正方形的第一个顶点，
// 从而保证飞机不会先冲到全局原点再开始走方形。
struct BasePoseState {
    bool valid = false;
    ros::Time stamp;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

BasePoseState g_base_pose;
BasePoseState g_vrpn_pose;

struct MavrosStateCache {
    bool valid = false;
    bool connected = false;
    bool armed = false;
    std::string mode;
};

MavrosStateCache g_mavros_state;

struct FreezeWindowState {
    bool initialized = false;
    ros::Time stamp;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct TdState {
    bool initialized = false;
    Vec3 position{0.0, 0.0, 0.0};
    Vec3 velocity{0.0, 0.0, 0.0};
};

// 通用限幅函数。
// 一方面用于参数回退后的保护，另一方面用于最终关节命令限幅。
double Clamp(double value, double min_value, double max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

// 机械臂轨迹的外部参数用角度表达，三角函数内部统一转成弧度。
double DegToRad(double value_deg) {
    return value_deg * std::acos(-1.0) / 180.0;
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

bool IsPoseFresh(const BasePoseState &pose, const ros::Time &now, double timeout_sec) {
    if (!pose.valid) {
        return false;
    }
    if (timeout_sec <= 0.0) {
        return true;
    }
    return (now - pose.stamp).toSec() <= timeout_sec;
}

// 状态机阶段名，只用于日志输出。
const char *StageName(int stage) {
    switch (stage) {
        case 0:
            return "settle";
        case 1:
            return "square_motion";
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

// 实验启动服务回调。
// 与现有链路保持兼容：收到 desired_start=10 才开始正式发布实验三期望。
bool doReq(uav::desired_start::Request &req, uav::desired_start::Response &resp) {
    if (req.desired_start != 10) {
        ROS_ERROR("提交的数据异常!!!");
        return false;
    }

    resp.desired_sent = 3;
    start_flag = true;
    return true;
}

// 缓存飞机实际位置。
// 这里的数据源是 /mavros/local_position/pose，实验三用它来冻结正方形首角。
void BasePoseCb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    g_base_pose.valid = true;
    g_base_pose.stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    g_base_pose.x = msg->pose.position.x;
    g_base_pose.y = msg->pose.position.y;
    g_base_pose.z = msg->pose.position.z;
}

// 缓存动捕真值。
// 实验三新增的保护逻辑会比较：
// 1. 动捕真值 vs mavros/local_position
// 2. 动捕真值 vs 当前期望位置
// 一旦偏差持续超阈值，就中止正方形主段，先把机械臂收回。
void VrpnPoseCb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    g_vrpn_pose.valid = true;
    g_vrpn_pose.stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    g_vrpn_pose.x = msg->pose.position.x;
    g_vrpn_pose.y = msg->pose.position.y;
    g_vrpn_pose.z = msg->pose.position.z;
}

// 缓存 MAVROS 当前状态。
// 这里默认只把 connected/armed/mode 记录下来，并将 connected 作为一条保护依据。
void MavrosStateCb(const mavros_msgs::State::ConstPtr &msg) {
    g_mavros_state.valid = true;
    g_mavros_state.connected = msg->connected;
    g_mavros_state.armed = msg->armed;
    g_mavros_state.mode = msg->mode;
}

// 线性插值函数。
// 实验三的基座轨迹是“分段线性正方形”，每条边都由这个函数在起点和终点之间插值生成。
double InterpolateSegment(double start_value, double end_value, double ratio) {
    return start_value + (end_value - start_value) * ratio;
}

}  // namespace

int main(int argc, char *argv[]) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "uam_desired_exp3");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ros::Publisher joint_angle_pub =
        nh.advertise<uam_message::arm_angle>("/wjl/arm/guidefly/angle_d", 10);
    ros::Publisher uav_pos_d_pub =
        nh.advertise<uav::xyz_yaw_d>("/wjl/guidefly/pose_d", 10);
    ros::ServiceServer server = nh.advertiseService("/wjl/start/uav_desired", doReq);
    ros::Subscriber base_pose_sub =
        nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, BasePoseCb);
    ros::Subscriber vrpn_pose_sub =
        nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/arm_base/pose", 10, VrpnPoseCb);
    ros::Subscriber mavros_state_sub =
        nh.subscribe<mavros_msgs::State>("/mavros/state", 10, MavrosStateCb);

    // ----------------------------
    // 实验三参数区
    // ----------------------------
    // 参数分为两组：
    // 1. 基座轨迹参数：决定无人机在空间里怎么飞
    // 2. 机械臂轨迹参数：决定机械臂在飞行期间怎么周期运动
    //
    // 其中实验三的核心思想是：
    // - 基座按正方形飞行
    // - 机械臂同时按周期正弦运动
    // - 两者并行，不做实验二那种末端固定点补偿
    double hover_z = 1.0;
    double hover_yaw_deg = 0.0;
    double square_side_length = 0.7;
    double edge_time = 7.0;
    double settle_time = 3.0;
    double recovery_time = 2.0;
    double arm1_offset_deg = 0.0;
    double arm1_amp_deg = 10.0;
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
    //   TD 的等效带宽，越小参考越慢越柔。
    //
    // td_accel_limit_xy / td_accel_limit_z:
    //   x/y 和 z 方向参考加速度上限，单位 m/s^2；值越小，给内环的指令变化越柔。
    //
    // td_vel_limit_xy / td_vel_limit_z:
    //   x/y 和 z 方向参考速度上限，单位 m/s；方形段也会受这个速度上限约束。
    //
    // settle_reach_xy_m / settle_reach_z_m:
    //   settle 完成判据，实际位置与当前 TD 期望的水平/高度误差阈值，单位 m。
    //
    // settle_min_hold_s:
    //   进入误差阈值后至少稳定保持这么久，才允许进入方形轨迹和机械臂扰动，单位 s。
    bool use_td_reference = true;
    double td_bandwidth_hz = 0.18;
    double td_accel_limit_xy = 0.04;
    double td_accel_limit_z = 0.04;
    double td_vel_limit_xy = 0.05;
    double td_vel_limit_z = 0.05;
    double settle_reach_xy_m = 0.10;
    double settle_reach_z_m = 0.10;
    double settle_min_hold_s = 2.0;
    bool enable_protection = true;
    bool protection_auto_land = false;
    double max_pose_disagreement = 0.6;
    double max_tracking_error = 1.0;
    double max_altitude_drop = 0.3;
    int protection_trigger_cycles = 10;
    double protection_hold_time = 2.0;

    // =============================================位置更新的新鲜度保护================================================
    // 超过这个时间没新鲜数据，就认为 arm_base/local pose 不可靠。
    double pose_timeout_sec = 0.3;
    // 连续多少次检测到位姿超时后，才真正触发 protection/safety_hold。
    int pose_loss_limit = 10;

    // ==============================================数据回传丢失保护=================================================
    // ======================================= 三维总数据（模长）回传丢失保护 ===================================
    // 在这么长一段时间内，检查飞机实际位置有没有明显动。
    double freeze_window_sec = 1.0;
    // 在 freeze_window_sec 这个时间窗里，如果实际位置总位移小于这个值，就认为“几乎没动”。
    double freeze_motion_threshold_m = 0.03;
    // 只有当“当前还没到目标点”时，冻结保护才有意义。
    double freeze_arrive_threshold_m = 0.15;
    // 冻结故障连续命中多少次后，进入 safety_hold。
    int freeze_trigger_cycles = 3;
    // 进入 safety_hold 后，如果异常还持续，再连续多少次后升级到 safety_landing。
    int freeze_escalate_cycles = 6;

    // =========================================== 单Z轴数据回传丢失保护 ============================================
    // 专门针对高度方向的冻结保护。如果 z 方向目标误差大于这个值，说明高度还差很多。
    double z_freeze_error_threshold_m = 0.20;
    // 在检测时间窗里，如果实际 z 位移小于这个值，就认为高度基本没变化。
    double z_freeze_motion_threshold_m = 0.03;

    // ============================================= 安全姿态参数 ================================================
    // 保护触发后，机械臂回到这组安全角，停止继续对机体叠加扰动。
    double safety_hold_arm1_deg = 0.0;
    double safety_hold_arm2_deg = 0.0;
    double safety_hold_hand_deg = 0.0;

    // hover_z:
    //   起飞完成后以及整个实验三主段中的目标高度。
    //
    // square_side_length:
    //   正方形边长。若改成 1.0，就会走边长 1 米的方形。
    //
    // edge_time:
    //   每条边飞行的时间。边长固定时，它直接决定基座飞行速度。
    //   例如当前 0.7 m / 7 s = 0.1 m/s。
    //
    // settle_time:
    //   进入实验后先悬停稳定多久，再开始走方形。
    //
    // recovery_time:
    //   走完一圈后回到首角并保持多久，再进入降落触发。
    //
    // arm*_offset_deg:
    //   对应关节周期运动的中心位置。
    //
    // arm*_amp_deg:
    //   对应关节正弦运动的幅值。幅值越大，关节摆动范围越大。
    //
    // arm*_period:
    //   对应关节正弦运动的周期。周期越小，运动越快。
    //
    // arm*_phase_deg:
    //   两个关节之间的相位差。实验三默认复用实验一的双关节相位配置。
    //
    // enable_protection:
    //   是否启用实验三保护逻辑。
    //
    // max_pose_disagreement:
    //   动捕真值与 mavros/local_position 的最大允许偏差。
    //
    // max_tracking_error:
    //   动捕真值与当前期望位置的最大允许偏差。
    //
    // max_altitude_drop:
    //   相对 hover_z 的最大允许掉高。
    //
    // protection_trigger_cycles:
    //   连续超阈值多少个控制周期后触发保护，避免单帧噪声误触发。
    //
    // protection_auto_land:
    //   默认关闭，保护触发后先保持悬停，让现场人工判断是否继续降落。
    nh.param("/wjl/uam/hover_z", hover_z, hover_z);
    pnh.param("hover_z", hover_z, hover_z);
    pnh.param("hover_yaw_deg", hover_yaw_deg, hover_yaw_deg);
    pnh.param("square_side_length", square_side_length, square_side_length);
    pnh.param("edge_time", edge_time, edge_time);
    pnh.param("settle_time", settle_time, settle_time);
    pnh.param("recovery_time", recovery_time, recovery_time);
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
    pnh.param("enable_protection", enable_protection, enable_protection);
    pnh.param("protection_auto_land", protection_auto_land, protection_auto_land);
    pnh.param("max_pose_disagreement", max_pose_disagreement, max_pose_disagreement);
    pnh.param("max_tracking_error", max_tracking_error, max_tracking_error);
    pnh.param("max_altitude_drop", max_altitude_drop, max_altitude_drop);
    pnh.param("protection_trigger_cycles", protection_trigger_cycles, protection_trigger_cycles);
    pnh.param("protection_hold_time", protection_hold_time, protection_hold_time);
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

    // 参数保护：
    // 实验节点启动时优先做简单回退，避免无效参数直接把轨迹公式搞坏。
    if (square_side_length < 0.0) {
        square_side_length = 0.0;
    }
    if (edge_time <= 0.0) {
        edge_time = 7.0;
    }
    if (settle_time < 0.0) {
        settle_time = 0.0;
    }
    if (recovery_time < 0.0) {
        recovery_time = 0.0;
    }
    if (max_pose_disagreement <= 0.0) {
        max_pose_disagreement = 0.6;
    }
    if (max_tracking_error <= 0.0) {
        max_tracking_error = 1.0;
    }
    if (max_altitude_drop <= 0.0) {
        max_altitude_drop = 0.3;
    }
    if (protection_trigger_cycles < 1) {
        protection_trigger_cycles = 1;
    }
    if (protection_hold_time < 0.0) {
        protection_hold_time = 0.0;
    }
    if (arm1_period <= 0.0) {
        arm1_period = 3.5;
    }
    if (arm2_period <= 0.0) {
        arm2_period = 3.5;
    }
    if (td_bandwidth_hz <= 0.0) {
        td_bandwidth_hz = 0.18;
    }
    if (td_accel_limit_xy <= 0.0) {
        td_accel_limit_xy = 0.04;
    }
    if (td_accel_limit_z <= 0.0) {
        td_accel_limit_z = 0.04;
    }
    if (td_vel_limit_xy <= 0.0) {
        td_vel_limit_xy = 0.05;
    }
    if (td_vel_limit_z <= 0.0) {
        td_vel_limit_z = 0.05;
    }
    if (settle_reach_xy_m <= 0.0) {
        settle_reach_xy_m = 0.10;
    }
    if (settle_reach_z_m <= 0.0) {
        settle_reach_z_m = 0.10;
    }
    if (settle_min_hold_s < 0.0) {
        settle_min_hold_s = 0.0;
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

    // square_motion_time:
    //   实验三主段总时长。固定为 4 条边总时间，不再单独维护 experiment_time。
    //
    // two_pi / arm*_phase_rad:
    //   用于机械臂正弦轨迹计算。
    const double square_motion_time = 4.0 * edge_time;
    const ros::Duration landing_publish_time(0.5);
    const ros::Duration return_home_publish_time(1.0);
    const double two_pi = 2.0 * std::acos(-1.0);
    const double arm1_phase_rad = DegToRad(arm1_phase_deg);
    const double arm2_phase_rad = DegToRad(arm2_phase_deg);

    auto PublishReturnHome = [&](double home_arm1_deg, double home_arm2_deg, double home_hand_deg) {
        if (!ros::ok()) {
            return;
        }
        uam_message::arm_angle home_angle;
        home_angle.arm1_angle = Clamp(home_arm1_deg, arm1_min, arm1_max);
        home_angle.arm2_angle = Clamp(home_arm2_deg, arm2_min, arm2_max);
        home_angle.hand_angle = Clamp(home_hand_deg, hand_min, hand_max);
        ROS_INFO("exp3 return arm to home: arm_d=(%.2f, %.2f, %.2f)",
                 home_angle.arm1_angle, home_angle.arm2_angle, home_angle.hand_angle);
        ros::Rate return_rate(30.0);
        const ros::Time return_start = ros::Time::now();
        while (ros::ok() && (ros::Time::now() - return_start) < return_home_publish_time) {
            joint_angle_pub.publish(home_angle);
            ros::spinOnce();
            return_rate.sleep();
        }
    };

    ROS_INFO("exp3 service ready, waiting for /wjl/start/uav_desired");
    ROS_INFO(
        "exp3 params: hover_z=%.2f, yaw=%.2f deg, side=%.2f m, edge_time=%.2f s, settle=%.2f s, recovery=%.2f s, arm1=(offset %.2f, amp %.2f, T %.2f, phase %.2f), arm2=(offset %.2f, amp %.2f, T %.2f, phase %.2f)",
        hover_z, hover_yaw_deg, square_side_length, edge_time, settle_time, recovery_time,
        arm1_offset_deg, arm1_amp_deg, arm1_period, arm1_phase_deg, arm2_offset_deg,
        arm2_amp_deg, arm2_period, arm2_phase_deg);
    ROS_INFO(
        "exp3 protection: enable=%s, auto_land=%s, pose_gap<=%.2f m, track_err<=%.2f m, max_drop<=%.2f m, trigger_cycles=%d, hold_time=%.2f s, pose_timeout=%.2f s, freeze_window=%.2f s",
        enable_protection ? "true" : "false", protection_auto_land ? "true" : "false",
        max_pose_disagreement, max_tracking_error, max_altitude_drop, protection_trigger_cycles,
        protection_hold_time, pose_timeout_sec, freeze_window_sec);
    ROS_INFO(
        "exp3 TD: enabled=%s, bw=%.2f Hz, acc_xy=%.3f, acc_z=%.3f, vel_xy=%.3f, vel_z=%.3f, settle_reach=(xy %.3f, z %.3f), settle_hold=%.2f s",
        use_td_reference ? "true" : "false", td_bandwidth_hz, td_accel_limit_xy,
        td_accel_limit_z, td_vel_limit_xy, td_vel_limit_z, settle_reach_xy_m,
        settle_reach_z_m, settle_min_hold_s);
    ROS_INFO("exp3 slow takeoff: x/y will anchor at current local pose; z tracks hover_z through TD");

    ros::Rate rate(30.0);
    while (ros::ok() && !start_flag) {
        ros::spinOnce();
        rate.sleep();
    }
    if (!ros::ok()) {
        return 0;
    }

    while (ros::ok() && !IsPoseFresh(g_base_pose, ros::Time::now(), pose_timeout_sec)) {
        ROS_WARN_THROTTLE(1.0, "exp3 waiting for fresh local pose before TD initialization");
        ros::spinOnce();
        rate.sleep();
    }
    if (!ros::ok()) {
        return 0;
    }

    const Vec3 takeoff_initial_position{g_base_pose.x, g_base_pose.y, g_base_pose.z};
    const Vec3 takeoff_hover_target{takeoff_initial_position.x, takeoff_initial_position.y, hover_z};
    const double td_bandwidth_rad_s = 2.0 * std::acos(-1.0) * td_bandwidth_hz;
    TdState td;
    ResetTd(td, takeoff_initial_position);
    ROS_INFO("exp3 takeoff anchor: current local=(%.3f, %.3f, %.3f), target=(%.3f, %.3f, %.3f)",
             takeoff_initial_position.x, takeoff_initial_position.y, takeoff_initial_position.z,
             takeoff_hover_target.x, takeoff_hover_target.y, takeoff_hover_target.z);

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

    // square_origin_x / y:
    //   正方形的第一个顶点。
    //   它不是参数写死的，而是在实验真正开始后，用飞机“当前 local 位置”冻结得到。
    //   起飞和 settle 都保持这个 x/y，后续方形移动是在该当前位置基础上加减 square_side_length。
    //   这样做的目的是避免飞机先飞去某个固定原点。
    bool square_origin_initialized = true;
    double square_origin_x = takeoff_initial_position.x;
    double square_origin_y = takeoff_initial_position.y;
    bool protection_triggered = false;
    bool safety_landing_triggered = false;
    bool settle_complete = false;
    ros::Time settle_reached_since;
    ros::Time post_settle_start;
    std::string protection_reason = "none";
    ros::Time protection_start_time;
    int protection_violation_count = 0;
    int pose_loss_count = 0;
    int freeze_fault_count = 0;
    int protection_escalate_count = 0;
    double last_pose_disagreement = 0.0;
    double last_tracking_error = 0.0;
    double last_altitude_drop = 0.0;
    FreezeWindowState freeze_window_state;
    uav::xyz_yaw_d last_safe_uav_pos_d = uav_pos_d;

    const ros::Time experiment_start = ros::Time::now();
    ros::Time last_loop_time = experiment_start;
    ros::Time last_log_time = experiment_start - ros::Duration(1.0);

    while (ros::ok()) {
        ros::spinOnce();

        const ros::Time now = ros::Time::now();
        const double dt = std::max(0.0, (now - last_loop_time).toSec());
        last_loop_time = now;
        const double elapsed = (now - experiment_start).toSec();
        int stage = 0;

        // ----------------------------
        // 实验三状态机
        // ----------------------------
        // 0. settle:
        //    刚进入实验，先保持悬停和机械臂偏置角，给飞控一个稳定窗口。
        //
        // 1. square_motion:
        //    无人机开始按正方形飞行，机械臂同时进入双关节正弦运动。
        //
        // 2. recovery:
        //    走完整个正方形后，基座回到首角，机械臂回偏置角。
        //
        // 3. landing:
        //    发布 z_d=0.5 和 land_flag=true，交给 UAV 侧已有逻辑收尾。
        //
        // 4. safety_hold:
        //    保护触发后停止继续走方形，保持最后一个安全 setpoint，并把机械臂收回。
        //
        // 5. safety_landing:
        //    仅在 protection_auto_land=true 时启用，保护保持一段时间后触发降落。
        if (protection_triggered) {
            const double protection_elapsed = (now - protection_start_time).toSec();
            if ((safety_landing_triggered ||
                 (protection_auto_land && protection_elapsed >= protection_hold_time)) &&
                protection_elapsed >= protection_hold_time + landing_publish_time.toSec()) {
                break;
            }
            stage = (safety_landing_triggered ||
                     (protection_auto_land && protection_elapsed >= protection_hold_time)) ? 5 : 4;
        } else if (!settle_complete) {
            stage = 0;
        } else {
            const double post_settle_elapsed = (now - post_settle_start).toSec();
            if (post_settle_elapsed < square_motion_time) {
                stage = 1;
            } else if (post_settle_elapsed < square_motion_time + recovery_time) {
                stage = 2;
            } else if (post_settle_elapsed < square_motion_time + recovery_time +
                                          landing_publish_time.toSec()) {
                stage = 3;
            } else {
                break;
            }
        }

        if (!square_origin_initialized && g_base_pose.valid) {
            square_origin_x = g_base_pose.x;
            square_origin_y = g_base_pose.y;
            square_origin_initialized = true;
            ROS_INFO("exp3 square origin initialized from current base pose: (%.3f, %.3f)",
                     square_origin_x, square_origin_y);
        }

        // 每个循环先写入默认输出：
        // - 基座默认停在首角
        // - 高度默认 hover_z
        // - 机械臂默认回 offset
        //
        // 之后再根据 stage 覆盖主段逻辑。
        uav_pos_d.land_flag = false;
        uav_pos_d.x_d = takeoff_hover_target.x;
        uav_pos_d.y_d = takeoff_hover_target.y;
        uav_pos_d.z_d = takeoff_hover_target.z;
        uav_pos_d.yaw_d = hover_yaw_deg;
        current_angle.arm1_angle = arm1_offset_deg;
        current_angle.arm2_angle = arm2_offset_deg;
        current_angle.hand_angle = hand_hold_deg;

        if (stage == 1) {
            // ----------------------------
            // 基座方形轨迹生成逻辑
            // ----------------------------
            // motion_time       : 进入主段后的累计时间
            // edge_index        : 当前处于第几条边（0/1/2/3）
            // edge_elapsed      : 当前边已经飞了多久
            // ratio             : 当前边内的归一化进度 [0,1]
            //
            // 轨迹顺序固定为：
            // 0: P0 -> P1   (+x)
            // 1: P1 -> P2   (+y)
            // 2: P2 -> P3   (-x)
            // 3: P3 -> P0   (-y)
            const double motion_time = settle_complete ? (now - post_settle_start).toSec() : 0.0;
            const int edge_index = std::min(3, static_cast<int>(motion_time / edge_time));
            const double edge_elapsed = motion_time - edge_index * edge_time;
            const double ratio = Clamp(edge_elapsed / edge_time, 0.0, 1.0);

            double start_x = square_origin_x;
            double start_y = square_origin_y;
            double end_x = square_origin_x;
            double end_y = square_origin_y;

            if (edge_index == 0) {
                end_x = square_origin_x + square_side_length;
            } else if (edge_index == 1) {
                start_x = square_origin_x + square_side_length;
                end_x = square_origin_x + square_side_length;
                end_y = square_origin_y + square_side_length;
            } else if (edge_index == 2) {
                start_x = square_origin_x + square_side_length;
                start_y = square_origin_y + square_side_length;
                end_y = square_origin_y + square_side_length;
            } else {
                start_y = square_origin_y + square_side_length;
            }

            // 将“当前边的起点/终点”插值成连续期望点。
            // 这就是实验三的基座核心飞行逻辑。
            uav_pos_d.x_d = InterpolateSegment(start_x, end_x, ratio);
            uav_pos_d.y_d = InterpolateSegment(start_y, end_y, ratio);

            // ----------------------------
            // 机械臂双关节周期运动逻辑
            // ----------------------------
            // 实验三机械臂不做在线补偿，而是直接复用实验一双关节正弦：
            //
            // arm = offset + sign * amp * sin(2*pi*t/T + phase)
            //
            // 参数与实际行为的关系：
            // - offset 决定关节摆动中心
            // - amp    决定摆动幅度
            // - T      决定运动快慢
            // - phase  决定两个关节之间的相位关系
            //
            // 当前 motion_time 与基座主段时间共用，
            // 因此效果就是“无人机边飞，机械臂边动”。
            current_angle.arm1_angle =
                arm1_offset_deg +
                arm1_sign * arm1_amp_deg *
                    std::sin(two_pi * motion_time / arm1_period + arm1_phase_rad);
            current_angle.arm2_angle =
                arm2_offset_deg +
                arm2_sign * arm2_amp_deg *
                    std::sin(two_pi * motion_time / arm2_period + arm2_phase_rad);
        } else if (stage == 2) {
            // recovery 段只做两件事：
            // 1. 基座回到首角
            // 2. 机械臂回 offset
            uav_pos_d.x_d = square_origin_x;
            uav_pos_d.y_d = square_origin_y;
        } else if (stage == 3) {
            // landing 段不再走方形，直接沿用现有起降链路的降落触发方式。
            uav_pos_d.x_d = square_origin_x;
            uav_pos_d.y_d = square_origin_y;
            uav_pos_d.z_d = 0.5;
            uav_pos_d.land_flag = true;
        } else if (stage == 4) {
            // 保护保持段：
            // 基座停在最近一次确认“还算安全”的 setpoint，
            // 机械臂全部回 offset，避免继续给飞机叠加扰动。
            uav_pos_d = last_safe_uav_pos_d;
            current_angle.arm1_angle = safety_hold_arm1_deg;
            current_angle.arm2_angle = safety_hold_arm2_deg;
            current_angle.hand_angle = safety_hold_hand_deg;
        } else if (stage == 5) {
            uav_pos_d = last_safe_uav_pos_d;
            uav_pos_d.z_d = 0.5;
            uav_pos_d.land_flag = true;
            current_angle.arm1_angle = safety_hold_arm1_deg;
            current_angle.arm2_angle = safety_hold_arm2_deg;
            current_angle.hand_angle = safety_hold_hand_deg;
        }

        if (stage <= 2 && !protection_triggered) {
            const Vec3 raw_target{uav_pos_d.x_d, uav_pos_d.y_d, uav_pos_d.z_d};
            if (use_td_reference) {
                UpdateTd(td, raw_target, dt, td_bandwidth_rad_s, td_accel_limit_xy,
                         td_accel_limit_z, td_vel_limit_xy, td_vel_limit_z);
                uav_pos_d.x_d = td.position.x;
                uav_pos_d.y_d = td.position.y;
                uav_pos_d.z_d = td.position.z;
            } else {
                ResetTd(td, raw_target);
            }
        }

        if (!protection_triggered && stage == 0) {
            const bool local_fresh = IsPoseFresh(g_base_pose, now, pose_timeout_sec);
            const bool vrpn_fresh = IsPoseFresh(g_vrpn_pose, now, pose_timeout_sec);
            if (local_fresh && vrpn_fresh) {
                const double xy_error = std::sqrt(
                    (uav_pos_d.x_d - g_base_pose.x) * (uav_pos_d.x_d - g_base_pose.x) +
                    (uav_pos_d.y_d - g_base_pose.y) * (uav_pos_d.y_d - g_base_pose.y));
                const double z_error = std::fabs(uav_pos_d.z_d - g_base_pose.z);
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
                    last_safe_uav_pos_d = uav_pos_d;
                    ROS_INFO("exp3 settle complete: elapsed=%.2f s, xy_error=%.3f m, z_error=%.3f m",
                             elapsed, xy_error, z_error);
                }
            } else {
                settle_reached_since = ros::Time();
            }
        }

        // ----------------------------
        // 实验三保护逻辑
        // ----------------------------
        // 这部分的目标不是“修复飞控”，而是在已明显异常时尽快停掉任务主段。
        // 当前保护默认看 4 类信号：
        // 1. 动捕真值与 MAVROS 估计的偏差
        // 2. 动捕真值与当前期望位置的偏差
        // 3. 动捕真值相对 hover_z 的掉高
        // 4. mavros/state 是否仍然 connected
        if (!protection_triggered && enable_protection && stage <= 2) {
            bool violation = false;
            bool freshness_fault = false;
            bool freeze_fault_active = false;
            const bool local_fresh = IsPoseFresh(g_base_pose, now, pose_timeout_sec);
            const bool vrpn_fresh = IsPoseFresh(g_vrpn_pose, now, pose_timeout_sec);

            if (g_vrpn_pose.valid && g_base_pose.valid) {
                const double dx = g_vrpn_pose.x - g_base_pose.x;
                const double dy = g_vrpn_pose.y - g_base_pose.y;
                const double dz = g_vrpn_pose.z - g_base_pose.z;
                last_pose_disagreement = std::sqrt(dx * dx + dy * dy + dz * dz);
                if (last_pose_disagreement > max_pose_disagreement) {
                    violation = true;
                }
            }

            if (g_vrpn_pose.valid) {
                const double dx = g_vrpn_pose.x - uav_pos_d.x_d;
                const double dy = g_vrpn_pose.y - uav_pos_d.y_d;
                const double dz = g_vrpn_pose.z - uav_pos_d.z_d;
                last_tracking_error = std::sqrt(dx * dx + dy * dy + dz * dz);
                if (last_tracking_error > max_tracking_error) {
                    violation = true;
                }

                // TD 慢起飞的 settle 阶段，高度期望本身还在从当前高度缓慢爬升。
                // 这时不能用 hover_z - 当前高度判断“掉高”，否则会把正常慢起飞误判成保护。
                // settle 完成后再恢复原来的 hover_z 掉高判据。
                last_altitude_drop = (stage == 0 ? uav_pos_d.z_d : hover_z) - g_vrpn_pose.z;
                if (last_altitude_drop > max_altitude_drop) {
                    violation = true;
                }
            }

            if (g_mavros_state.valid && !g_mavros_state.connected) {
                violation = true;
            }

            // 新增 freshness 保护：
            // 实验三保留旧的“真值偏差/掉高”保护，同时再补一层“数据根本不新鲜”保护，
            // 防止 arm_base 或 local pose 超时后仍继续推进方形轨迹。
            freshness_fault =
                !local_fresh ||
                !vrpn_fresh ||
                (g_mavros_state.valid && !g_mavros_state.connected);
            if (freshness_fault) {
                ++pose_loss_count;
                ROS_WARN_THROTTLE(
                    1.0,
                    "exp3 freshness fault: local_fresh=%s, base_fresh=%s, mavros_connected=%s (%d/%d)",
                    local_fresh ? "true" : "false", vrpn_fresh ? "true" : "false",
                    g_mavros_state.connected ? "true" : "false", pose_loss_count, pose_loss_limit);
                if (pose_loss_count >= pose_loss_limit) {
                    protection_triggered = true;
                    protection_start_time = now;
                    protection_reason = "arm_base/local pose freshness lost";
                }
            } else {
                pose_loss_count = 0;
            }

            // 新增冻结保护：
            // 覆盖“动捕刚体丢失但一直发最后一帧”的故障。
            // 这种情况下 local pose 可能仍然 fresh，但如果目标误差持续存在且位置长时间几乎不动，
            // 就不能继续让方形轨迹往前推。
            if (local_fresh) {
                if (!freeze_window_state.initialized) {
                    freeze_window_state.initialized = true;
                    freeze_window_state.stamp = now;
                    freeze_window_state.x = g_base_pose.x;
                    freeze_window_state.y = g_base_pose.y;
                    freeze_window_state.z = g_base_pose.z;
                } else if ((now - freeze_window_state.stamp).toSec() >= freeze_window_sec) {
                    const double dx = g_base_pose.x - freeze_window_state.x;
                    const double dy = g_base_pose.y - freeze_window_state.y;
                    const double dz = g_base_pose.z - freeze_window_state.z;
                    const double motion_norm = std::sqrt(dx * dx + dy * dy + dz * dz);
                    const double ex = uav_pos_d.x_d - g_base_pose.x;
                    const double ey = uav_pos_d.y_d - g_base_pose.y;
                    const double ez = uav_pos_d.z_d - g_base_pose.z;
                    const double pos_error_norm = std::sqrt(ex * ex + ey * ey + ez * ez);
                    const double z_error = std::fabs(ez);
                    const double z_motion = std::fabs(g_base_pose.z - freeze_window_state.z);
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
                            "exp3 freeze guard: pos_error=%.3f m, motion=%.3f m, z_error=%.3f m, z_motion=%.3f m (%d/%d)",
                            pos_error_norm, motion_norm, z_error, z_motion,
                            freeze_fault_count, freeze_trigger_cycles);
                        if (freeze_fault_count >= freeze_trigger_cycles) {
                            protection_triggered = true;
                            protection_start_time = now;
                            protection_reason = "local pose frozen before target reached";
                        }
                    } else {
                        freeze_fault_count = 0;
                    }
                    freeze_window_state.stamp = now;
                    freeze_window_state.x = g_base_pose.x;
                    freeze_window_state.y = g_base_pose.y;
                    freeze_window_state.z = g_base_pose.z;
                }
            } else {
                freeze_window_state.initialized = false;
                freeze_fault_count = 0;
            }

            if (violation) {
                protection_violation_count++;
            } else {
                protection_violation_count = 0;
                if (!freshness_fault) {
                    last_safe_uav_pos_d = uav_pos_d;
                }
            }

            if (protection_violation_count >= protection_trigger_cycles) {
                protection_triggered = true;
                protection_start_time = now;
                protection_reason = "legacy truth-gap protection";
                uav_pos_d = last_safe_uav_pos_d;
                current_angle.arm1_angle = safety_hold_arm1_deg;
                current_angle.arm2_angle = safety_hold_arm2_deg;
                current_angle.hand_angle = safety_hold_hand_deg;
                ROS_ERROR(
                    "exp3 protection triggered: pose_gap=%.3f m, track_err=%.3f m, altitude_drop=%.3f m, mavros_connected=%s, mode=%s",
                    last_pose_disagreement, last_tracking_error, last_altitude_drop,
                    g_mavros_state.connected ? "true" : "false",
                    g_mavros_state.mode.c_str());
            }
        }

        if (protection_triggered && !safety_landing_triggered && stage == 4) {
            const bool local_fresh = IsPoseFresh(g_base_pose, now, pose_timeout_sec);
            const bool vrpn_fresh = IsPoseFresh(g_vrpn_pose, now, pose_timeout_sec);
            bool persistent_fault =
                !local_fresh ||
                !vrpn_fresh ||
                (g_mavros_state.valid && !g_mavros_state.connected);
            if (local_fresh) {
                if (!freeze_window_state.initialized) {
                    freeze_window_state.initialized = true;
                    freeze_window_state.stamp = now;
                    freeze_window_state.x = g_base_pose.x;
                    freeze_window_state.y = g_base_pose.y;
                    freeze_window_state.z = g_base_pose.z;
                } else if ((now - freeze_window_state.stamp).toSec() >= freeze_window_sec) {
                    const double dx = g_base_pose.x - freeze_window_state.x;
                    const double dy = g_base_pose.y - freeze_window_state.y;
                    const double dz = g_base_pose.z - freeze_window_state.z;
                    const double motion_norm = std::sqrt(dx * dx + dy * dy + dz * dz);
                    const double ex = last_safe_uav_pos_d.x_d - g_base_pose.x;
                    const double ey = last_safe_uav_pos_d.y_d - g_base_pose.y;
                    const double ez = last_safe_uav_pos_d.z_d - g_base_pose.z;
                    const double pos_error_norm = std::sqrt(ex * ex + ey * ey + ez * ez);
                    const double z_error = std::fabs(ez);
                    const double z_motion = std::fabs(g_base_pose.z - freeze_window_state.z);
                    persistent_fault =
                        persistent_fault ||
                        (pos_error_norm > freeze_arrive_threshold_m &&
                         motion_norm < freeze_motion_threshold_m) ||
                        (z_error > z_freeze_error_threshold_m &&
                         z_motion < z_freeze_motion_threshold_m);
                    freeze_window_state.stamp = now;
                    freeze_window_state.x = g_base_pose.x;
                    freeze_window_state.y = g_base_pose.y;
                    freeze_window_state.z = g_base_pose.z;
                }
            } else {
                freeze_window_state.initialized = false;
            }
            if (persistent_fault) {
                ++protection_escalate_count;
                if (protection_escalate_count >= freeze_escalate_cycles) {
                    safety_landing_triggered = true;
                    protection_start_time = now;
                    ROS_ERROR("exp3 protection escalated to safety_landing: %s",
                              protection_reason.c_str());
                }
            } else {
                protection_escalate_count = 0;
            }
        }

        // 最终发送前统一按 YAML 限位，避免参数调整过头时把机械臂打到不可接受区域。
        current_angle.arm1_angle = Clamp(current_angle.arm1_angle, arm1_min, arm1_max);
        current_angle.arm2_angle = Clamp(current_angle.arm2_angle, arm2_min, arm2_max);
        current_angle.hand_angle = Clamp(current_angle.hand_angle, hand_min, hand_max);

        // 1 Hz 日志主要用来观察：
        // 1. 当前处于哪个实验阶段
        // 2. 方形首角是否正确冻结
        // 3. 当前发给 UAV 和机械臂的期望值是什么
        if ((now - last_log_time).toSec() >= 1.0) {
            last_log_time = now;
            ROS_INFO(
                "[%s] t=%.2f s, pose_d=(%.2f, %.2f, %.2f, %.2f), arm_d=(%.2f, %.2f, %.2f), square_origin=(%.2f, %.2f), base_valid=%s, land=%s",
                StageName(stage), elapsed, uav_pos_d.x_d, uav_pos_d.y_d, uav_pos_d.z_d,
                uav_pos_d.yaw_d, current_angle.arm1_angle, current_angle.arm2_angle,
                current_angle.hand_angle, square_origin_x, square_origin_y,
                g_base_pose.valid ? "true" : "false", uav_pos_d.land_flag ? "true" : "false");
            if (enable_protection) {
                ROS_INFO(
                    "exp3 protection monitor: pose_gap=%.3f m, track_err=%.3f m, altitude_drop=%.3f m, count=%d, pose_loss=%d, freeze_count=%d, reason=%s, vrpn_valid=%s, mavros_connected=%s, mode=%s",
                    last_pose_disagreement, last_tracking_error, last_altitude_drop,
                    protection_violation_count, pose_loss_count, freeze_fault_count,
                    protection_reason.c_str(), g_vrpn_pose.valid ? "true" : "false",
                    g_mavros_state.connected ? "true" : "false", g_mavros_state.mode.c_str());
            }
        }

        uav_pos_d_pub.publish(uav_pos_d);
        joint_angle_pub.publish(current_angle);
        rate.sleep();
    }

    PublishReturnHome(0.0, 0.0, hand_hold_deg);
    std::cout << "exp3 finished" << std::endl;
    return 0;
}
