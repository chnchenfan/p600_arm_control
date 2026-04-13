#include <ros/ros.h>

#include <cmath>
#include <iostream>
#include <string>

#include <geometry_msgs/PoseStamped.h>

#include "uam_message/arm_angle.h"
#include "uav/desired_start.h"
#include "uav/xyz_yaw_d.h"

// 实验二：
// 1. UAV 在 x-z 平面走圆弧前馈；
// 2. 机械臂在基座系内做 2DoF IK；
// 3. 用末端动捕误差做位置外环修正；
// 4. 状态机按 settle -> arc_hold -> recovery -> landing 运行。
namespace {

// 与实验一、实验三保持一致：
// 先由 UAV 侧起飞到位，再通过服务触发实验二状态机进入主段。
bool start_flag = false;

// 本文件内部统一用简单三维向量表达位置、误差和坐标变换结果，
// 避免为了实验节点引入额外线性代数依赖。
struct Vec3 {
    double x;
    double y;
    double z;
};

// 通用位姿缓存。
// arm_base / arm_target / mavros local pose 都复用同一结构：
// - valid: 是否收到过该话题
// - stamp: 最近一帧时间戳，用于新鲜度保护
// - position/orientation: 最近一帧位姿
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

// 实验二的飞行保护不再只靠“切 recovery”这一条路径。
// 这里额外区分：
// - normal        : 正常执行实验轨迹
// - safety_hold   : 进入安全悬停，停止推进主段参考
// - safety_landing: 安全悬停后异常仍持续，开始任务层降落
enum class ProtectionState {
    kNormal = 0,
    kSafetyHold = 1,
    kSafetyLanding = 2,
};

// “位置冻结/无响应保护”需要一个时间窗口：
// 若窗口内位置几乎没动，但目标误差始终存在，
// 则说明当前不是普通的慢速跟踪，而更像是“动捕卡死在最后一帧”。
struct FreezeWindowState {
    bool initialized = false;
    ros::Time stamp;
    Vec3 position{0.0, 0.0, 0.0};
};

PoseState g_base_pose;
PoseState g_ee_pose;
PoseState g_local_pose;
ArmState g_real_arm_state;

// 统一的数学工具函数。
// 这里尽量保持“小而直接”，方便真机联调时快速核对公式。
inline double Pi() {
    return std::acos(-1.0);
}

// 限幅函数
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
    return value_deg * Pi() / 180.0;
}

double RadToDeg(double value_rad) {
    return value_rad * 180.0 / Pi();
}

double WrapToPi(double angle_rad) {
    const double two_pi = 2.0 * Pi();
    while (angle_rad > Pi()) {
        angle_rad -= two_pi;
    }
    while (angle_rad < -Pi()) {
        angle_rad += two_pi;
    }
    return angle_rad;
}

double AdjustNearReference(double angle_rad, double reference_rad) {
    double adjusted = angle_rad;
    const double two_pi = 2.0 * Pi();
    while (adjusted - reference_rad > Pi()) {
        adjusted -= two_pi;
    }
    while (adjusted - reference_rad < -Pi()) {
        adjusted += two_pi;
    }
    return adjusted;
}

double AngularDistance(double angle_a_rad, double angle_b_rad) {
    return std::fabs(WrapToPi(angle_a_rad - angle_b_rad));
}

Vec3 AddVec3(const Vec3 &lhs, const Vec3 &rhs) {
    return {lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

// 向量减法函数
Vec3 SubVec3(const Vec3 &lhs, const Vec3 &rhs) {
    return {lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

Vec3 ScaleVec3(const Vec3 &value, double scale) {
    return {value.x * scale, value.y * scale, value.z * scale};
}

Vec3 InterpolateVec3(const Vec3 &start, const Vec3 &end, double ratio) {
    return AddVec3(start, ScaleVec3(SubVec3(end, start), ratio));
}

// 求解向量模长
double NormVec3(const Vec3 &value) {
    return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}

Vec3 ClampVec3Norm(const Vec3 &value, double max_norm) {
    if (max_norm <= 0.0) {
        return {0.0, 0.0, 0.0};
    }
    const double norm = NormVec3(value);
    if (norm <= max_norm || norm < 1e-9) {
        return value;
    }
    return ScaleVec3(value, max_norm / norm);
}

double RateLimit(double target, double current, double max_delta) {
    if (max_delta <= 0.0) {
        return target;
    }
    return Clamp(target, current - max_delta, current + max_delta);
}

double QuaternionToYaw(double x, double y, double z, double w) {
    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    return std::atan2(siny_cosp, cosy_cosp);
}

// 归一化四元数，避免动捕偶发的数值误差让旋转矩阵退化。
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

// 机体系 -> 世界系 的完整姿态旋转。
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

// arm_base 动捕刚体原点不一定与真实机械臂基座原点重合。
// 若刚体原点位于真实基座 x 正方向前方 dx，则真实基座世界坐标应为：
//   p_base_true^W = p_rb^W - R_WB * [dx, 0, 0]^T
// 这里的 dx 以“刚体系 x 正方向前方”为正，代码内部统一做减法。
Vec3 GetCorrectedBaseWorldPosition(const PoseState &base_pose, const Vec3 &arm_base_offset_body_m) {
    return SubVec3(base_pose.position, RotateBodyToWorld(arm_base_offset_body_m, base_pose));
}

// 世界系 -> 机体系 的逆旋转。
// 实验二主逻辑要把固定点从动捕世界系变换到机械臂基座系，就是用这一步。
Vec3 RotateWorldToBody(const Vec3 &vector_world, const PoseState &pose) {
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
        r00 * vector_world.x + r10 * vector_world.y + r20 * vector_world.z,
        r01 * vector_world.x + r11 * vector_world.y + r21 * vector_world.z,
        r02 * vector_world.x + r12 * vector_world.y + r22 * vector_world.z,
    };
}

// 动捕/位姿话题的新鲜度判断。
// 超时后不再继续硬解 IK，而是保持上一帧并切 recovery。
bool IsPoseFresh(const PoseState &pose, const ros::Time &now, double timeout_sec) {
    if (!pose.valid) {
        return false;
    }
    if (timeout_sec <= 0.0) {
        return true;
    }
    return (now - pose.stamp).toSec() <= timeout_sec;
}

bool IsJumpAcceptable(const Vec3 &candidate,
                     bool has_reference,
                     const Vec3 &reference,
                     double jump_threshold_m) {
    if (!has_reference || jump_threshold_m <= 0.0) {
        return true;
    }
    return NormVec3(SubVec3(candidate, reference)) <= jump_threshold_m;
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

Vec3 SelectSafeHoverWorld(bool hover_anchor_initialized,
                          const Vec3 &hover_anchor_world,
                          bool last_safe_world_initialized,
                          const Vec3 &last_safe_world_command,
                          const Vec3 &last_uav_world_command) {
    // safe_hover 的优先级：
    // 1. 已冻结的 hover anchor
    // 2. 最后一次通过安全判定的世界系 UAV 参考
    // 3. 最后一帧世界系 UAV 参考
    // 这样做的原因是：一旦动捕异常，不应继续追逐当前圆弧中途点，
    // 也不能回退到 (0,0,0)，而是应尽量回到已经验证过的安全悬停点。
    if (hover_anchor_initialized) {
        return hover_anchor_world;
    }
    if (last_safe_world_initialized) {
        return last_safe_world_command;
    }
    return last_uav_world_command;
}

Vec3 SelectFallbackOutputPosition(bool last_safe_output_initialized,
                                  const Vec3 &last_safe_output_position,
                                  bool local_pose_valid,
                                  const PoseState &local_pose,
                                  const Vec3 &fallback_output_position) {
    // fresh 丢失时不允许发 (0,0,0)。
    // 优先保持最后一个“通过安全判定”的输出位置；
    // 若连这个都没有，则退回当前 local pose；
    // 再不行才使用调用方给出的保底位置。
    if (last_safe_output_initialized) {
        return last_safe_output_position;
    }
    if (local_pose_valid) {
        return local_pose.position;
    }
    return fallback_output_position;
}

// 2DoF 机械臂逆解。
//
// 几何模型：
//   p_rel = [L2*cos(q2)*cos(q1), -L2*cos(q2)*sin(q1), L2*sin(q2)]
//
// 推导思路：
// 1. 先从目标点的 x/y 分量确定 arm1 的水平朝向 q1；实测约定下，arm1 增大对应末端朝 -y。
// 2. 再用水平投影半径 sqrt(x^2+y^2) 和 z 分量确定 arm2 的抬头角 q2；
// 3. 用上一帧关节角做连续支选择，避免角度在 +/-180 度附近跳变。
//
// 保护逻辑：
// - 若目标点长度和 L2 偏差超过 eps_r，则判不可达；
// - 若水平投影半径小于 eps_xy，则判近奇异，不更新新解。
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
    const double radius = NormVec3(target_body); // radius = sqrt(x^2 + y^2 + z^2)
    const double reach_error = std::fabs(radius - link_length); // 计算可达性误差。reach_error = | ||target_body|| - L2 |
    if (reach_error_out != nullptr) {// 如果这个误差很大，说明目标点不在连杆球面上
        *reach_error_out = reach_error;// 把误差写回去，让调用者知道这次失败是不是因为“点离球面太远”
    }
    if (radius < 1e-9 || reach_error > eps_r) {// 基本可达性检查
        return false;
    }
    // 计算水平投影半径，用于后续求解q2
    const double horizontal_radius = std::sqrt(target_body.x * target_body.x + target_body.y * target_body.y);
    if (horizontal_radius < eps_xy) {
        return false;
    }

    const double q1_reference = DegToRad(previous_arm1_deg);
    const double q2_reference = DegToRad(previous_arm2_deg);

    // 在线零位标定后，IK 几何角需要先扣掉本次飞行测得的装配偏置，
    // 再转换成发送给舵机/控制器的命令角。
    const double q1_geometry = std::atan2(-target_body.y, target_body.x);
    const double q2_geometry = std::atan2(target_body.z, horizontal_radius);
    double q1_candidate = q1_geometry - DegToRad(arm1_zero_offset_deg);
    double q2_candidate = q2_geometry - DegToRad(arm2_zero_offset_deg);
    q1_candidate = AdjustNearReference(q1_candidate, q1_reference);
    q2_candidate = AdjustNearReference(q2_candidate, q2_reference);

    const double arm1_deg = RadToDeg(q1_candidate);
    const double arm2_deg = RadToDeg(q2_candidate);
    if (arm1_deg < arm1_min_deg || arm1_deg > arm1_max_deg) {
        return false;
    }
    if (arm2_deg < arm2_min_deg || arm2_deg > arm2_max_deg) {
        return false;
    }

    *arm1_deg_out = arm1_deg;
    *arm2_deg_out = arm2_deg;
    return true;
}

// 仅用于日志，帮助联调时快速识别状态机处于哪一段。
const char *StageName(int stage) {
    switch (stage) {
        case 0:
            return "settle";
        case 1:
            return "arc_hold";
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

// 与其他实验保持一致的服务触发入口。
bool doReq(uav::desired_start::Request &req, uav::desired_start::Response &resp) {
    if (req.desired_start != 10) {
        ROS_ERROR("提交的数据异常!!!");
        return false;
    }
    resp.desired_sent = 3;
    start_flag = true;
    return true;
}

// PoseStamped -> PoseState 的统一缓存函数。
void FillPoseState(const geometry_msgs::PoseStamped::ConstPtr &msg, PoseState *state) {
    state->valid = true;
    state->stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    state->position = {msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
    state->qx = msg->pose.orientation.x;
    state->qy = msg->pose.orientation.y;
    state->qz = msg->pose.orientation.z;
    state->qw = msg->pose.orientation.w;
}

// 三路位姿缓存：
// - arm_base: 机械臂基座在动捕世界系下的位姿，是任务层几何主输入
// - arm_target: 末端工作点在动捕世界系下的位姿，是固定点冻结和外环误差主输入
// - local_pose: PX4/MAVROS 本地估计位姿，仅用于参考映射和诊断
void BasePoseCb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    FillPoseState(msg, &g_base_pose);
}

void EePoseCb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    FillPoseState(msg, &g_ee_pose);
}

void LocalPoseCb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    FillPoseState(msg, &g_local_pose);
}

void ArmRealCb(const uam_message::arm_angle::ConstPtr &msg) {
    g_real_arm_state.valid = true;
    g_real_arm_state.arm1_deg = msg->arm1_angle;
    g_real_arm_state.arm2_deg = msg->arm2_angle;
    g_real_arm_state.hand_deg = msg->hand_angle;
}

// 若 /wjl/guidefly/pose_d 在 PX4 local 系解释，则将“动捕世界系下算出的参考”
// 映射到 local 系。当前实现只保留平移偏置，不额外引入固定旋转偏置。
Vec3 MapWorldToOutputFrame(const Vec3 &world_position,
                           bool use_local_pose_mapping,
                           bool frame_mapping_initialized,
                           const Vec3 &world_anchor,
                           const Vec3 &local_anchor) {
    if (!use_local_pose_mapping || !frame_mapping_initialized) {
        return world_position;
    }
    return AddVec3(local_anchor, SubVec3(world_position, world_anchor));
}

}  // namespace

int main(int argc, char *argv[]) {
    // 主函数流程：
    // 1. 读取参数并建立订阅/发布；
    // 2. 等待 /wjl/start/uav_desired 服务触发；
    // 3. 在 settle 段冻结 hover_anchor 和 P_hold；
    // 4. 在 arc_hold 段执行 x-z 圆弧 + 末端定点补偿；
    // 5. 进入 recovery 和 landing 完成收尾。
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "uam_desired_exp2");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ros::Publisher joint_angle_pub =
        nh.advertise<uam_message::arm_angle>("/wjl/arm/guidefly/angle_d", 10);
    ros::Publisher online_offset_pub =
        nh.advertise<uam_message::arm_angle>("/wjl/arm/guidefly/online_offset", 10);
    ros::Publisher uav_pos_d_pub =
        nh.advertise<uav::xyz_yaw_d>("/wjl/guidefly/pose_d", 10);
    ros::ServiceServer server = nh.advertiseService("/wjl/start/uav_desired", doReq);

    // 实验二参数分四组：
    // 1. 时间参数：稳定段 / 主段 / 恢复段长度
    // 2. 机械臂几何与主轨迹参数：L1、L2、theta 范围与周期
    // 3. 末端外环、在线标定与保护参数：位置外环增益、零位偏置在线估计、可达性和奇异点阈值
    // 4. 话题与映射参数：arm_base、arm_target、local pose 及是否启用 local 映射
    double settle_time = 3.0; // 悬停时间
    double experiment_time = 20.0;// 测试时间
    double recovery_time = 2.0;// 恢复时间
    double hover_yaw_deg = 0.0;// 
    double landing_z = 0.5;
    double l1_m = 0.161;
    double l2_m = 0.262;
    double theta_min_deg = -10.0;
    double theta_max_deg = 45.0;
    double theta_period_sec = 10.0;
    double ee_outer_kp = 0.25; // 补偿增益
    double ee_outer_clip_m = 0.03;
    bool use_online_joint_calibration = true; // 在线标定零位偏置的标识符， true=启动，flase=关闭
    double online_calibration_samples = 20.0;
    double calibration_max_offset_deg = 20.0;
    double arm1_zero_offset_deg = 0.0;
    double arm2_zero_offset_deg = 0.0;
    double pose_timeout_sec = 0.3;
    double eps_r_m = 0.01; // L2长度误差的最大值
    double eps_xy_m = 0.015; // 水平投影的最小值
    int ik_fail_limit = 10;
    int pose_loss_limit = 10;
    double arm1_dot_max_deg_s = 90.0;
    double arm2_dot_max_deg_s = 90.0;
    bool startup_zero_enabled = true;
    bool startup_motion_check_enabled = true;
    double startup_zero_timeout_sec = 20.0;
    double startup_zero_hold_sec = 0.5;
    double startup_zero_tolerance_deg = 1.0;
    double startup_zero_arm1_deg = 0.0;
    double startup_zero_arm2_deg = 0.0;
    double startup_motion_check_arm1_deg = -5.0;
    double startup_motion_check_arm2_deg = 0.0;
    double hold_freeze_samples = 1.0;
    bool use_local_pose_mapping = true;
    bool use_initial_ee_hold = true; // 当前末端位置自动冻结固定点标志位，true为启用，false为不启用
    double explicit_hold_x = 0.0;
    double explicit_hold_y = 0.0;
    double explicit_hold_z = 0.0;
    double arm_base_offset_x_m = 0.0;
    double arm_base_offset_y_m = 0.0;
    double arm_base_offset_z_m = 0.0;
    // 两类保护共用参数：
    // 1. freeze_* / z_freeze_*：位置冻结/无响应保护
    // 2. uav_cmd_jump_threshold_m / uav_safe_z_max_m：安全有效指令筛选
    // 3. safety_arm1_deg / safety_arm2_deg：进入 safety_hold / safety_landing 后的机械臂安全姿态
    double freeze_window_sec = 1.0;          // 冻结检测时间窗长度；窗内位置几乎不动且误差持续存在，则判定为“卡死”
    double freeze_motion_threshold_m = 0.03; // 时间窗内总位移阈值；小于该值认为飞机“几乎没动”
    double freeze_arrive_threshold_m = 0.15; // 只有未到目标点时才触发冻结保护；该值定义“还没到”的位置误差下限
    int freeze_trigger_cycles = 2;           // 冻结故障连续命中的次数阈值；达到后进入 safety_hold
    int freeze_escalate_cycles = 6;          // 进入 safety_hold 后，异常继续持续的次数阈值；达到后升级为 safety_landing
    double z_freeze_error_threshold_m = 0.20;  // 竖直冻结保护：z 方向误差阈值；用于“往天上冲/往地上压但高度不动”的场景
    double z_freeze_motion_threshold_m = 0.03; // 竖直冻结保护：时间窗内 z 位移阈值；小于该值认为高度基本没变化
    double uav_cmd_jump_threshold_m = 0.25;    // “安全有效指令”筛选中的单周期跳变量阈值；过大则不更新 last_safe_*
    double uav_safe_z_max_m = 2.5;             // “安全有效指令”允许缓存的最高 z；超过该高度不再更新安全参考
    double safety_arm1_deg = 0.0;              // 保护触发后 arm1 回到的安全角度
    double safety_arm2_deg = 0.0;              // 保护触发后 arm2 回到的安全角度
    std::string base_pose_topic = "/vrpn_client_node/arm_base/pose";
    std::string ee_pose_topic = "/vrpn_client_node/arm_target/pose";
    std::string local_pose_topic = "/mavros/local_position/pose";

    pnh.param("settle_time", settle_time, settle_time);
    pnh.param("experiment_time", experiment_time, experiment_time);
    pnh.param("recovery_time", recovery_time, recovery_time);
    pnh.param("hover_yaw_deg", hover_yaw_deg, hover_yaw_deg);
    pnh.param("landing_z", landing_z, landing_z);
    pnh.param("L1_m", l1_m, l1_m);
    pnh.param("L2_m", l2_m, l2_m);
    pnh.param("theta_min_deg", theta_min_deg, theta_min_deg);
    pnh.param("theta_max_deg", theta_max_deg, theta_max_deg);
    pnh.param("theta_period_sec", theta_period_sec, theta_period_sec);
    pnh.param("ee_outer_kp", ee_outer_kp, ee_outer_kp);
    pnh.param("ee_outer_clip_m", ee_outer_clip_m, ee_outer_clip_m);
    pnh.param("use_online_joint_calibration", use_online_joint_calibration, use_online_joint_calibration);
    pnh.param("online_calibration_samples", online_calibration_samples, online_calibration_samples);
    pnh.param("calibration_max_offset_deg", calibration_max_offset_deg, calibration_max_offset_deg);
    pnh.param("arm1_zero_offset_deg", arm1_zero_offset_deg, arm1_zero_offset_deg);
    pnh.param("arm2_zero_offset_deg", arm2_zero_offset_deg, arm2_zero_offset_deg);
    pnh.param("pose_timeout_sec", pose_timeout_sec, pose_timeout_sec);
    pnh.param("eps_r_m", eps_r_m, eps_r_m);
    pnh.param("eps_xy_m", eps_xy_m, eps_xy_m);
    pnh.param("ik_fail_limit", ik_fail_limit, ik_fail_limit);
    pnh.param("pose_loss_limit", pose_loss_limit, pose_loss_limit);
    pnh.param("arm1_dot_max_deg_s", arm1_dot_max_deg_s, arm1_dot_max_deg_s);
    pnh.param("arm2_dot_max_deg_s", arm2_dot_max_deg_s, arm2_dot_max_deg_s);
    pnh.param("startup_zero_enabled", startup_zero_enabled, startup_zero_enabled);
    pnh.param("startup_motion_check_enabled", startup_motion_check_enabled, startup_motion_check_enabled);
    pnh.param("startup_zero_timeout_sec", startup_zero_timeout_sec, startup_zero_timeout_sec);
    pnh.param("startup_zero_hold_sec", startup_zero_hold_sec, startup_zero_hold_sec);
    pnh.param("startup_zero_tolerance_deg", startup_zero_tolerance_deg, startup_zero_tolerance_deg);
    pnh.param("startup_zero_arm1_deg", startup_zero_arm1_deg, startup_zero_arm1_deg);
    pnh.param("startup_zero_arm2_deg", startup_zero_arm2_deg, startup_zero_arm2_deg);
    pnh.param("startup_motion_check_arm1_deg", startup_motion_check_arm1_deg, startup_motion_check_arm1_deg);
    pnh.param("startup_motion_check_arm2_deg", startup_motion_check_arm2_deg, startup_motion_check_arm2_deg);
    pnh.param("hold_freeze_samples", hold_freeze_samples, hold_freeze_samples);
    pnh.param("use_local_pose_mapping", use_local_pose_mapping, use_local_pose_mapping);
    pnh.param("use_initial_ee_hold", use_initial_ee_hold, use_initial_ee_hold);
    pnh.param("exp2_hold_ee_x", explicit_hold_x, explicit_hold_x);
    pnh.param("exp2_hold_ee_y", explicit_hold_y, explicit_hold_y);
    pnh.param("exp2_hold_ee_z", explicit_hold_z, explicit_hold_z);
    pnh.param("arm_base_offset_x_m", arm_base_offset_x_m, arm_base_offset_x_m);
    pnh.param("arm_base_offset_y_m", arm_base_offset_y_m, arm_base_offset_y_m);
    pnh.param("arm_base_offset_z_m", arm_base_offset_z_m, arm_base_offset_z_m);
    pnh.param("freeze_window_sec", freeze_window_sec, freeze_window_sec);
    pnh.param("freeze_motion_threshold_m", freeze_motion_threshold_m, freeze_motion_threshold_m);
    pnh.param("freeze_arrive_threshold_m", freeze_arrive_threshold_m, freeze_arrive_threshold_m);
    pnh.param("freeze_trigger_cycles", freeze_trigger_cycles, freeze_trigger_cycles);
    pnh.param("freeze_escalate_cycles", freeze_escalate_cycles, freeze_escalate_cycles);
    pnh.param("z_freeze_error_threshold_m", z_freeze_error_threshold_m, z_freeze_error_threshold_m);
    pnh.param("z_freeze_motion_threshold_m", z_freeze_motion_threshold_m, z_freeze_motion_threshold_m);
    pnh.param("uav_cmd_jump_threshold_m", uav_cmd_jump_threshold_m, uav_cmd_jump_threshold_m);
    pnh.param("uav_safe_z_max_m", uav_safe_z_max_m, uav_safe_z_max_m);
    pnh.param("safety_arm1_deg", safety_arm1_deg, safety_arm1_deg);
    pnh.param("safety_arm2_deg", safety_arm2_deg, safety_arm2_deg);
    pnh.param("base_pose_topic", base_pose_topic, base_pose_topic);
    pnh.param("ee_pose_topic", ee_pose_topic, ee_pose_topic);
    pnh.param("local_pose_topic", local_pose_topic, local_pose_topic);

    // 参数保护性回退，避免 launch/参数服务器里给出明显无效值。
    if (settle_time < 0.0) {
        settle_time = 0.0;
    }
    if (experiment_time < 0.0) {
        experiment_time = 0.0;
    }
    if (recovery_time < 0.0) {
        recovery_time = 0.0;
    }
    if (theta_period_sec <= 0.0) {
        theta_period_sec = 10.0;
    }
    if (pose_timeout_sec <= 0.0) {
        pose_timeout_sec = 0.3;
    }
    if (online_calibration_samples < 1.0) {
        online_calibration_samples = 1.0;
    }
    if (calibration_max_offset_deg <= 0.0) {
        calibration_max_offset_deg = 20.0;
    }
    if (eps_r_m <= 0.0) {
        eps_r_m = 0.01;
    }
    if (eps_xy_m <= 0.0) {
        eps_xy_m = 0.015;
    }
    if (ik_fail_limit < 1) {
        ik_fail_limit = 1;
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
    if (uav_cmd_jump_threshold_m <= 0.0) {
        uav_cmd_jump_threshold_m = 0.25;
    }
    if (uav_safe_z_max_m <= 0.0) {
        uav_safe_z_max_m = 2.5;
    }
    if (hold_freeze_samples < 1.0) {
        hold_freeze_samples = 1.0;
    }
    if (startup_zero_timeout_sec <= 0.0) {
        startup_zero_timeout_sec = 20.0;
    }
    if (startup_zero_hold_sec <= 0.0) {
        startup_zero_hold_sec = 0.5;
    }
    if (startup_zero_tolerance_deg <= 0.0) {
        startup_zero_tolerance_deg = 1.0;
    }

    // 关节限位：
    // arm1 继续从现有参数服务器读取；
    // arm2 还会进一步收缩到本次实验要求的 theta 区间内。
    double arm1_min = -180.0;
    double arm1_max = 180.0;
    double arm2_min = theta_min_deg;
    double arm2_max = theta_max_deg;
    double hand_min = -15.0;
    double hand_max = 25.0;
    nh.param("/arm/arm_joint1/min", arm1_min, arm1_min);
    nh.param("/arm/arm_joint1/max", arm1_max, arm1_max);
    nh.param("/arm/arm_joint2/min", arm2_min, arm2_min);
    nh.param("/arm/arm_joint2/max", arm2_max, arm2_max);
    nh.param("/arm/left_hand_joint/min", hand_min, hand_min);
    nh.param("/arm/left_hand_joint/max", hand_max, hand_max);
    arm2_min = std::max(arm2_min, theta_min_deg);
    arm2_max = std::min(arm2_max, theta_max_deg);
    safety_arm1_deg = Clamp(safety_arm1_deg, arm1_min, arm1_max);
    safety_arm2_deg = Clamp(safety_arm2_deg, arm2_min, arm2_max);

    // 反馈输入：
    // - base_pose_sub: 实验二任务层几何主输入
    // - ee_pose_sub: 固定点冻结 + 末端误差外环
    // - local_pose_sub: 仅用于将 world 参考映射到 PX4 local 输出
    // - arm_real_sub: 关节实测，用于平滑起步和日志对照
    ros::Subscriber base_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(base_pose_topic, 10, BasePoseCb);
    ros::Subscriber ee_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(ee_pose_topic, 10, EePoseCb);
    ros::Subscriber local_pose_sub =
        nh.subscribe<geometry_msgs::PoseStamped>(local_pose_topic, 10, LocalPoseCb);
    ros::Subscriber arm_real_sub =
        nh.subscribe<uam_message::arm_angle>("/wjl/arm/real/angle_r", 10, ArmRealCb);

    ROS_INFO("exp2 service ready, waiting for /wjl/start/uav_desired");
    ROS_INFO(
        "exp2 params: base_topic=%s, ee_topic=%s, local_topic=%s, L1=%.3f, L2=%.3f, base_offset=[%.3f, %.3f, %.3f], theta=[%.1f, %.1f], T=%.2f, settle=%.2f, exp=%.2f, recovery=%.2f, startup_zero=%s, local_mapping=%s, online_calib=%s, freeze_window=%.2f, freeze_motion=%.3f, freeze_error=%.3f, safe_z_max=%.2f",
        base_pose_topic.c_str(), ee_pose_topic.c_str(), local_pose_topic.c_str(), l1_m, l2_m,
        arm_base_offset_x_m, arm_base_offset_y_m, arm_base_offset_z_m,
        theta_min_deg, theta_max_deg, theta_period_sec, settle_time, experiment_time, recovery_time,
        startup_zero_enabled ? "true" : "false",
        use_local_pose_mapping ? "true" : "false", use_online_joint_calibration ? "true" : "false",
        freeze_window_sec, freeze_motion_threshold_m, freeze_arrive_threshold_m, uav_safe_z_max_m);

    const ros::Duration return_home_publish_time(1.0);
    // 这里把“回原位”统一定义为 arm1=0、arm2=0。
    // 无论实验前的 startup_zero 如何设置，结束尾段都回到机械臂零位。
    auto PublishReturnHome = [&](double home_arm1_deg, double home_arm2_deg, double home_hand_deg) {
        if (!ros::ok()) {
            return;
        }
        uam_message::arm_angle home_angle;
        home_angle.arm1_angle = Clamp(home_arm1_deg, arm1_min, arm1_max);
        home_angle.arm2_angle = Clamp(home_arm2_deg, arm2_min, arm2_max);
        home_angle.hand_angle = Clamp(home_hand_deg, hand_min, hand_max);
        ROS_INFO("exp2 return arm to home: arm_d=(%.2f, %.2f, %.2f)",
                 home_angle.arm1_angle, home_angle.arm2_angle, home_angle.hand_angle);
        ros::Rate return_rate(30.0);
        const ros::Time return_start = ros::Time::now();
        while (ros::ok() && (ros::Time::now() - return_start) < return_home_publish_time) {
            joint_angle_pub.publish(home_angle);
            ros::spinOnce();
            return_rate.sleep();
        }
    };

    ros::Rate rate(30.0);
    int startup_phase = startup_zero_enabled ? (startup_motion_check_enabled ? 0 : 1) : 2;
    ros::Time startup_phase_start = ros::Time::now();
    ros::Time startup_zero_hold_start;
    double startup_zero_hand_deg = 0.0;
    bool startup_zero_hand_initialized = false;
    while (ros::ok() && (!start_flag || startup_phase != 2)) {
        ros::spinOnce();

        if (startup_zero_enabled) {
            if (!startup_zero_hand_initialized && g_real_arm_state.valid) {
                startup_zero_hand_deg = Clamp(g_real_arm_state.hand_deg, hand_min, hand_max);
                startup_zero_hand_initialized = true;
            }

            uam_message::arm_angle startup_zero_cmd;
            startup_zero_cmd.arm1_angle = Clamp(startup_phase == 0 ? startup_motion_check_arm1_deg : startup_zero_arm1_deg,
                                                arm1_min, arm1_max);
            startup_zero_cmd.arm2_angle = Clamp(startup_phase == 0 ? startup_motion_check_arm2_deg : startup_zero_arm2_deg,
                                                arm2_min, arm2_max);
            startup_zero_cmd.hand_angle = startup_zero_hand_initialized ? startup_zero_hand_deg : 0.0;
            joint_angle_pub.publish(startup_zero_cmd);

            if (startup_phase != 2) {
                if (!g_real_arm_state.valid) {
                    ROS_WARN_THROTTLE(1.0, "exp2 startup sequence waiting for /wjl/arm/real/angle_r");
                } else {
                    const double arm1_error = startup_zero_cmd.arm1_angle - g_real_arm_state.arm1_deg;
                    const double arm2_error = startup_zero_cmd.arm2_angle - g_real_arm_state.arm2_deg;
                    const bool within_tolerance =
                        std::max(std::fabs(arm1_error), std::fabs(arm2_error)) <= startup_zero_tolerance_deg;
                    if (within_tolerance) {
                        if (startup_zero_hold_start.isZero()) {
                            startup_zero_hold_start = ros::Time::now();
                        }
                        if ((ros::Time::now() - startup_zero_hold_start).toSec() >= startup_zero_hold_sec) {
                            if (startup_phase == 0) {
                                ROS_INFO("exp2 startup motion check finished: target=(%.1f, %.1f), real=(%.2f, %.2f)",
                                         startup_zero_cmd.arm1_angle, startup_zero_cmd.arm2_angle,
                                         g_real_arm_state.arm1_deg, g_real_arm_state.arm2_deg);
                                startup_phase = 1;
                                startup_phase_start = ros::Time::now();
                                startup_zero_hold_start = ros::Time();
                            } else {
                                startup_phase = 2;
                                ROS_INFO("exp2 startup zero finished: target=(%.1f, %.1f), real=(%.2f, %.2f)",
                                         startup_zero_cmd.arm1_angle, startup_zero_cmd.arm2_angle,
                                         g_real_arm_state.arm1_deg, g_real_arm_state.arm2_deg);
                            }
                        }
                    } else {
                        startup_zero_hold_start = ros::Time();
                    }

                    const char *active_label = (startup_phase == 0) ? "startup motion check" : "startup zero";
                    if ((ros::Time::now() - startup_phase_start).toSec() > startup_zero_timeout_sec) {
                        ROS_ERROR("exp2 %s failed within %.2f s: target=(%.1f, %.1f), real=(%.2f, %.2f), error=(%.2f, %.2f)",
                                  active_label, startup_zero_timeout_sec, startup_zero_cmd.arm1_angle, startup_zero_cmd.arm2_angle,
                                  g_real_arm_state.arm1_deg, g_real_arm_state.arm2_deg, arm1_error, arm2_error);
                        PublishReturnHome(0.0, 0.0, startup_zero_cmd.hand_angle);
                        return 1;
                    }
                }
            }
        }

        rate.sleep();
    }
    if (!ros::ok()) {
        return 0;
    }

    uav::xyz_yaw_d uav_pos_d;
    uam_message::arm_angle current_angle;
    uam_message::arm_angle online_offset_msg;
    current_angle.arm1_angle = g_real_arm_state.valid ? g_real_arm_state.arm1_deg : 0.0;
    current_angle.arm2_angle = g_real_arm_state.valid ? g_real_arm_state.arm2_deg : 0.0;
    current_angle.hand_angle = g_real_arm_state.valid ? g_real_arm_state.hand_deg : 0.0;
    current_angle.arm1_angle = Clamp(current_angle.arm1_angle, arm1_min, arm1_max);
    current_angle.arm2_angle = Clamp(current_angle.arm2_angle, arm2_min, arm2_max);
    current_angle.hand_angle = Clamp(current_angle.hand_angle, hand_min, hand_max);

    const ros::Time experiment_start = ros::Time::now();
    ros::Time last_log_time = experiment_start - ros::Duration(1.0);
    const ros::Duration landing_publish_time(0.5);
    const double two_pi = 2.0 * Pi();
    // 主段 theta_d(t) 采用正弦往返：
    // theta_d = theta_mid + theta_amp * sin(...)
    // 其中相位偏置专门选到“t=0 时 q2=0 附近”，避免一进入主段就跳角。
    const double theta_mid_deg = 0.5 * (theta_min_deg + theta_max_deg);
    const double theta_amp_deg = 0.5 * (theta_max_deg - theta_min_deg);
    double theta_phase_offset = 0.0;
    if (theta_amp_deg > 1e-6) {
        theta_phase_offset = std::asin(Clamp((0.0 - theta_mid_deg) / theta_amp_deg, -1.0, 1.0));
    }

    int hold_sample_count = 0;
    Vec3 hold_accumulator{0.0, 0.0, 0.0};
    int calibration_sample_count = 0; // 有效标定样本数
    double arm1_offset_accumulator_deg = 0.0;
    double arm2_offset_accumulator_deg = 0.0;
    bool joint_offset_initialized = !use_online_joint_calibration;// 偏置冻结标识符，true=偏置标定完成，false=偏置标定失败
    double calibrated_arm1_zero_offset_deg = arm1_zero_offset_deg;
    double calibrated_arm2_zero_offset_deg = arm2_zero_offset_deg;
    bool ee_hold_initialized = false; // 末端固定点冻结标志位，true为已冻结，false为未冻结
    Vec3 ee_hold_world{explicit_hold_x, explicit_hold_y, explicit_hold_z};// 世界系固定点 P_hold
    bool hover_anchor_initialized = false; // 圆弧轨迹起点设置标志位，true表示已设置，false表示未设置
    Vec3 hover_anchor_world{0.0, 0.0, 0.0};// 圆弧轨迹起点
    bool last_safe_world_initialized = false;
    Vec3 last_safe_uav_world_command{0.0, 0.0, 0.0};
    bool last_safe_output_initialized = false;
    Vec3 last_safe_output_position{0.0, 0.0, 0.0};
    Vec3 safe_hover_world{0.0, 0.0, 0.0};
    bool safe_hover_initialized = false;
    bool frame_mapping_initialized = false;
    Vec3 local_anchor{0.0, 0.0, 0.0};
    int ik_fail_count = 0;
    int pose_loss_count = 0;
    int freeze_fault_count = 0;
    int safety_escalate_count = 0;
    int last_stage = -1;
    bool force_recovery = false;
    ProtectionState protection_state = ProtectionState::kNormal;
    ProtectionState last_protection_state = ProtectionState::kNormal;
    FreezeWindowState freeze_window_state;
    ros::Time protection_state_start;
    std::string protection_reason = "none";
    double last_valid_arm1_deg = current_angle.arm1_angle;
    double last_valid_arm2_deg = current_angle.arm2_angle;
    const Vec3 arm_base_offset_body_m{arm_base_offset_x_m, arm_base_offset_y_m, arm_base_offset_z_m};

    Vec3 last_uav_world_command = g_base_pose.valid ? GetCorrectedBaseWorldPosition(g_base_pose, arm_base_offset_body_m) : Vec3{0.0, 0.0, 0.0};
    Vec3 recovery_start_world = last_uav_world_command;
    double recovery_start_arm1_deg = last_valid_arm1_deg;
    double recovery_start_arm2_deg = last_valid_arm2_deg;

    // 实验二主循环。
    // 整个流程分四段：
    // 0. settle   : 悬停稳定，冻结起始条件
    // 1. arc_hold : UAV 按 x-z 圆弧前馈运动，机械臂补偿保持末端定点
    // 2. recovery : UAV 回 hover_anchor，机械臂平滑回零
    // 3. landing  : 通知 UAV 侧进入降落收尾
    while (ros::ok()) {
        ros::spinOnce();

        const ros::Time now = ros::Time::now();
        const double elapsed = (now - experiment_start).toSec();// 现在的时间（映射到相较于实验启动时的起点时间）
        const double dt = 1.0 / 30.0;
        int stage = 0;
        if (protection_state == ProtectionState::kSafetyHold) {
            stage = 4;
        } else if (protection_state == ProtectionState::kSafetyLanding) {
            if (!protection_state_start.isZero() && (now - protection_state_start) > landing_publish_time) {
                break;
            }
            stage = 5;
        } else if (elapsed < settle_time) {
            stage = 0;
        } else if (!force_recovery && elapsed < settle_time + experiment_time) {
            stage = 1;
        } else if (elapsed < settle_time + experiment_time + recovery_time) {
            stage = 2;
        } else if (elapsed < settle_time + experiment_time + recovery_time + landing_publish_time.toSec()) {
            stage = 3;
        } else {
            break;
        }

        // 机械臂L2与L1之间的在线零位标定
        if (protection_state == ProtectionState::kNormal && stage >= 1 &&
            use_online_joint_calibration && !joint_offset_initialized) {
            // 若 settle 阶段尚未采满设定帧数，也在进入主段前用当前可用样本做一次收尾估计；
            // 若完全没有有效样本，则退回 launch 参数中的默认零位偏置。
            if (calibration_sample_count > 0) {
                calibrated_arm1_zero_offset_deg = Clamp(
                    arm1_offset_accumulator_deg / static_cast<double>(calibration_sample_count),
                    -calibration_max_offset_deg,
                    calibration_max_offset_deg);
                calibrated_arm2_zero_offset_deg = Clamp(
                    arm2_offset_accumulator_deg / static_cast<double>(calibration_sample_count),
                    -calibration_max_offset_deg,
                    calibration_max_offset_deg);
                ROS_WARN("exp2 online calibration finalized with partial samples (%d/%d): arm1_offset=%.2f deg, arm2_offset=%.2f deg",
                         calibration_sample_count, static_cast<int>(online_calibration_samples),
                         calibrated_arm1_zero_offset_deg, calibrated_arm2_zero_offset_deg);
            } else {
                ROS_WARN("exp2 online calibration got no valid samples, fallback to param offsets: arm1=%.2f deg, arm2=%.2f deg",
                         calibrated_arm1_zero_offset_deg, calibrated_arm2_zero_offset_deg);
            }
            joint_offset_initialized = true;
        }

        if (stage != last_stage) {
            ROS_INFO("exp2 stage -> %s", StageName(stage));
            if (stage == 2) {
                recovery_start_world = last_uav_world_command;
                recovery_start_arm1_deg = last_valid_arm1_deg;
                recovery_start_arm2_deg = last_valid_arm2_deg;
            }
            last_stage = stage;
        }
        if (protection_state != last_protection_state) {
            ROS_WARN("exp2 protection -> %s, reason=%s",
                     ProtectionStateName(protection_state), protection_reason.c_str());
            last_protection_state = protection_state;
        }

        const bool base_fresh = IsPoseFresh(g_base_pose, now, pose_timeout_sec); // true = 数据新鲜，没有超时
        const bool ee_fresh = IsPoseFresh(g_ee_pose, now, pose_timeout_sec);
        const bool local_fresh = IsPoseFresh(g_local_pose, now, pose_timeout_sec);
        const bool local_required = use_local_pose_mapping;
        const Vec3 base_world_position = GetCorrectedBaseWorldPosition(g_base_pose, arm_base_offset_body_m);

        auto enter_safety_state = [&](ProtectionState next_state, const std::string &reason) {
            if (protection_state == next_state) {
                return;
            }
            protection_state = next_state;
            protection_state_start = now;
            protection_reason = reason;
            safe_hover_world = SelectSafeHoverWorld(
                hover_anchor_initialized, hover_anchor_world,
                last_safe_world_initialized, last_safe_uav_world_command,
                last_uav_world_command);
            safe_hover_initialized = true;
            if (next_state == ProtectionState::kSafetyHold) {
                safety_escalate_count = 0;
                force_recovery = false;
            }
        };

        const Vec3 fallback_world = SelectSafeHoverWorld(
            hover_anchor_initialized, hover_anchor_world,
            last_safe_world_initialized, last_safe_uav_world_command,
            last_uav_world_command);
        const Vec3 fallback_output_world = MapWorldToOutputFrame(
            fallback_world, use_local_pose_mapping, frame_mapping_initialized,
            hover_anchor_world, local_anchor);
        const Vec3 fallback_output_position = SelectFallbackOutputPosition(
            last_safe_output_initialized, last_safe_output_position,
            g_local_pose.valid, g_local_pose, fallback_output_world);

        Vec3 current_output_position = fallback_output_position;
        Vec3 current_world_reference = fallback_world;
        bool safety_fault_persisted_this_cycle = false;

        if (stage == 0) {
            // 稳定段：
            // 1. 更新当前基座位置，作为尚未冻结前的初始参考
            // 2. 尽量使用关节实测作为初值，避免刚进入实验二就突跳
            // 3. 即使动捕暂时不新鲜，也不允许发 (0,0,0)，而是保持最后一个安全输出
            if (g_base_pose.valid) {
                last_uav_world_command = base_world_position;
            }
            if (g_real_arm_state.valid) {
                last_valid_arm1_deg = Clamp(g_real_arm_state.arm1_deg, arm1_min, arm1_max);
                last_valid_arm2_deg = Clamp(g_real_arm_state.arm2_deg, arm2_min, arm2_max);
                current_angle.hand_angle = Clamp(g_real_arm_state.hand_deg, hand_min, hand_max);
            }
            current_angle.arm1_angle = last_valid_arm1_deg;
            current_angle.arm2_angle = last_valid_arm2_deg;

            // 每次飞行都可在 settle 段做一次在线零位标定：
            // 1. 用 arm_base/arm_target 求出当前末端在基座系下的真实方向；
            // 2. 与当前关节实测角做差，得到装配零位偏置样本；
            // 3. 对若干帧样本求均值，得到本次飞行的 arm1/arm2 零位补偿。
            if (use_online_joint_calibration && !joint_offset_initialized && base_fresh && ee_fresh && g_real_arm_state.valid) {
                const Vec3 ee_offset_world = SubVec3(g_ee_pose.position, base_world_position);// PE_W - PB_W
                const Vec3 ee_body = RotateWorldToBody(ee_offset_world, g_base_pose);// PE_B = RB_W(PE_W - PB_W)
                const Vec3 p_rel_meas = SubVec3(ee_body, Vec3{0.0, 0.0, -l1_m});// PE_S = PE_B - PS_B
                const double radius = NormVec3(p_rel_meas);// 求解PE_S模长
                // 求得PE_S与l2的误差。用于标定检查，如果差太多，说明这帧数据不适合用于标定，
                // 可能有：动捕误差、姿态跳变、末端标记不稳等问题
                const double reach_error = std::fabs(radius - l2_m);
                // 计算偏置
                const double horizontal_radius =
                    std::sqrt(p_rel_meas.x * p_rel_meas.x + p_rel_meas.y * p_rel_meas.y);// 水平投影半径：r = sqrt(Px^2 + Py^2)
                if (radius >= 1e-9 && reach_error <= eps_r_m && horizontal_radius >= eps_xy_m) {
                    const double q1_true_deg = RadToDeg(std::atan2(-p_rel_meas.y, p_rel_meas.x)); // 按实测约定计算真实的q1：arm1 增大时末端朝 -y
                    const double q2_true_deg = RadToDeg(std::atan2(p_rel_meas.z, horizontal_radius)); // 计算真实的q2
                    const double arm1_offset_sample_deg = RadToDeg(
                        WrapToPi(DegToRad(q1_true_deg - g_real_arm_state.arm1_deg)));// 减去偏置
                    const double arm2_offset_sample_deg = Clamp(
                        q2_true_deg - g_real_arm_state.arm2_deg,
                        -calibration_max_offset_deg,
                        calibration_max_offset_deg);// 减去偏置并限幅
                    // 样本累加并计算数量
                    arm1_offset_accumulator_deg += arm1_offset_sample_deg;
                    arm2_offset_accumulator_deg += arm2_offset_sample_deg;
                    ++calibration_sample_count;
                    // 当采样达到阈值之后，冻结本次偏置，并求均值作为最终偏置
                    if (calibration_sample_count >= static_cast<int>(online_calibration_samples)) {
                        calibrated_arm1_zero_offset_deg = Clamp(
                            arm1_offset_accumulator_deg / static_cast<double>(calibration_sample_count),
                            -calibration_max_offset_deg,
                            calibration_max_offset_deg);
                        calibrated_arm2_zero_offset_deg = Clamp(
                            arm2_offset_accumulator_deg / static_cast<double>(calibration_sample_count),
                            -calibration_max_offset_deg,
                            calibration_max_offset_deg);
                        joint_offset_initialized = true;// 标记在线标定完成
                        ROS_INFO("exp2 online calibration finished: arm1_offset=%.2f deg, arm2_offset=%.2f deg (%d samples)",
                                 calibrated_arm1_zero_offset_deg, calibrated_arm2_zero_offset_deg,// 偏置的大小
                                 calibration_sample_count);//样本数
                    }
                }
            }

            const Vec3 output_position = base_fresh ?
                MapWorldToOutputFrame(base_world_position, use_local_pose_mapping, frame_mapping_initialized,
                                      hover_anchor_world, local_anchor) :
                fallback_output_position;
            current_output_position = output_position;
            current_world_reference = base_fresh ? base_world_position : fallback_world;
            uav_pos_d.land_flag = false;
            uav_pos_d.x_d = output_position.x;
            uav_pos_d.y_d = output_position.y;
            uav_pos_d.z_d = output_position.z;
            uav_pos_d.yaw_d = hover_yaw_deg;

            if ((!base_fresh || (local_required && !local_fresh)) && ++pose_loss_count >= pose_loss_limit) {
                enter_safety_state(ProtectionState::kSafetyHold, "mocap freshness lost in settle");
            } else if (base_fresh && (!local_required || local_fresh)) {
                pose_loss_count = 0;
            }
        } else if (stage == 1) {
            // 主段是实验二核心：
            // 1. 冻结 hover_anchor 和世界系固定点 P_hold
            // 2. 生成 UAV 的 x-z 圆弧参考
            // 3. 把 P_hold 从世界系变到基座系
            // 4. 用末端实测误差做位置外环修正
            // 5. 对修正后的目标点做 2DoF IK，得到 arm1/arm2 命令
            if (!ee_hold_initialized) {
                if (!base_fresh || !ee_fresh) {
                    ROS_WARN_THROTTLE(1.0, "exp2 waiting for fresh arm_base/arm_target before freezing hold point");
                } else {
                    if (!hover_anchor_initialized) {
                        hover_anchor_world = base_world_position;
                        hover_anchor_initialized = true;
                    }
                    if (use_local_pose_mapping && local_fresh && !frame_mapping_initialized) {
                        local_anchor = g_local_pose.position;
                        frame_mapping_initialized = true;
                    }
                    hold_accumulator = AddVec3(hold_accumulator, g_ee_pose.position);
                    ++hold_sample_count;
                    if (hold_sample_count >= static_cast<int>(hold_freeze_samples)) {
                        ee_hold_world = ScaleVec3(hold_accumulator, 1.0 / static_cast<double>(hold_sample_count));
                        ee_hold_initialized = true;
                        ROS_INFO("exp2 hold point frozen from arm_target: (%.3f, %.3f, %.3f)",
                                 ee_hold_world.x, ee_hold_world.y, ee_hold_world.z);
                    }
                }
            }

            if (!hover_anchor_initialized && base_fresh) {
                hover_anchor_world = base_world_position;
                hover_anchor_initialized = true;
            }
            if (use_local_pose_mapping && local_fresh && !frame_mapping_initialized && hover_anchor_initialized) {
                local_anchor = g_local_pose.position;
                frame_mapping_initialized = true;
                ROS_INFO("exp2 local mapping anchor frozen: world=(%.3f, %.3f, %.3f), local=(%.3f, %.3f, %.3f)",
                         hover_anchor_world.x, hover_anchor_world.y, hover_anchor_world.z,
                         local_anchor.x, local_anchor.y, local_anchor.z);
            }

            const bool hold_ready = ee_hold_initialized && hover_anchor_initialized;
            const bool freshness_fault = !base_fresh || !ee_fresh || (local_required && !local_fresh);
            if (freshness_fault) {
                ++pose_loss_count;
            } else {
                pose_loss_count = 0;
            }

            if (!hold_ready || freshness_fault) {
                // 第一类保护：动捕/位姿数据丢失保护。
                // 一旦主段关键输入不新鲜，不再继续推进圆弧轨迹，
                // 否则在“刚体卡在最后一帧”时，飞控会以为自己一直没到点，持续往错误方向推。
                current_output_position = fallback_output_position;
                current_world_reference = fallback_world;
                uav_pos_d.land_flag = false;
                uav_pos_d.x_d = current_output_position.x;
                uav_pos_d.y_d = current_output_position.y;
                uav_pos_d.z_d = current_output_position.z;
                uav_pos_d.yaw_d = hover_yaw_deg;

                const double max_arm1_delta = arm1_dot_max_deg_s * dt;
                const double max_arm2_delta = arm2_dot_max_deg_s * dt;
                current_angle.arm1_angle = RateLimit(safety_arm1_deg, last_valid_arm1_deg, max_arm1_delta);
                current_angle.arm2_angle = RateLimit(safety_arm2_deg, last_valid_arm2_deg, max_arm2_delta);
                last_valid_arm1_deg = current_angle.arm1_angle;
                last_valid_arm2_deg = current_angle.arm2_angle;

                if (freshness_fault) {
                    ROS_WARN_THROTTLE(1.0,
                                      "exp2 freshness fault: base_fresh=%s, ee_fresh=%s, local_fresh=%s, hold_ready=%s (%d/%d)",
                                      base_fresh ? "true" : "false", ee_fresh ? "true" : "false",
                                      local_fresh ? "true" : "false", hold_ready ? "true" : "false",
                                      pose_loss_count, pose_loss_limit);
                    if (pose_loss_count >= pose_loss_limit) {
                        enter_safety_state(ProtectionState::kSafetyHold, "mocap freshness lost in arc_hold");
                    }
                }
            } else {
                const double stage_time = elapsed - settle_time;
                const double theta_deg = theta_mid_deg + theta_amp_deg *
                    std::sin(theta_phase_offset + two_pi * stage_time / theta_period_sec);
                const double theta_rad = DegToRad(theta_deg);
                Vec3 desired_world = hover_anchor_world;
                desired_world.x = desired_world.x + l2_m * (1.0 - std::cos(theta_rad));
                desired_world.y = hover_anchor_world.y;
                desired_world.z = desired_world.z - l2_m * std::sin(theta_rad);
                last_uav_world_command = desired_world;
                current_world_reference = desired_world;
                current_output_position = MapWorldToOutputFrame(
                    desired_world, use_local_pose_mapping, frame_mapping_initialized,
                    hover_anchor_world, local_anchor);
                uav_pos_d.land_flag = false;
                uav_pos_d.x_d = current_output_position.x;
                uav_pos_d.y_d = current_output_position.y;
                uav_pos_d.z_d = current_output_position.z;
                uav_pos_d.yaw_d = hover_yaw_deg;

                const Vec3 hold_offset_world = SubVec3(ee_hold_world, base_world_position);
                Vec3 p_hold_body = RotateWorldToBody(hold_offset_world, g_base_pose);
                Vec3 p_rel = SubVec3(p_hold_body, Vec3{0.0, 0.0, -l1_m});

                const Vec3 ee_error_world = SubVec3(ee_hold_world, g_ee_pose.position);
                const Vec3 ee_error_body = RotateWorldToBody(ee_error_world, g_base_pose);
                const Vec3 correction_body =
                    ClampVec3Norm(ScaleVec3(ee_error_body, ee_outer_kp), ee_outer_clip_m);
                const Vec3 p_rel_corrected = AddVec3(p_rel, correction_body);

                double solved_arm1_deg = last_valid_arm1_deg;
                double solved_arm2_deg = last_valid_arm2_deg;
                double reach_error = 0.0;
                const bool solved = SolveInverseKinematics(
                    p_rel_corrected, l2_m, last_valid_arm1_deg, last_valid_arm2_deg,
                    calibrated_arm1_zero_offset_deg, calibrated_arm2_zero_offset_deg,
                    arm1_min, arm1_max, arm2_min, arm2_max,
                    eps_r_m, eps_xy_m, &solved_arm1_deg, &solved_arm2_deg, &reach_error);
                if (!solved) {
                    ++ik_fail_count;
                    current_angle.arm1_angle = last_valid_arm1_deg;
                    current_angle.arm2_angle = last_valid_arm2_deg;
                    ROS_WARN_THROTTLE(1.0,
                                      "exp2 IK failed (%d/%d), target=(%.3f, %.3f, %.3f), reach_error=%.4f",
                                      ik_fail_count, ik_fail_limit, p_rel_corrected.x, p_rel_corrected.y,
                                      p_rel_corrected.z, reach_error);
                    if (ik_fail_count >= ik_fail_limit) {
                        force_recovery = true;
                        ROS_ERROR("exp2 IK failed continuously, switching to recovery stage");
                    }
                } else {
                    ik_fail_count = 0;
                    const double max_arm1_delta = arm1_dot_max_deg_s * dt;
                    const double max_arm2_delta = arm2_dot_max_deg_s * dt;
                    solved_arm1_deg = RateLimit(solved_arm1_deg, last_valid_arm1_deg, max_arm1_delta);
                    solved_arm2_deg = RateLimit(solved_arm2_deg, last_valid_arm2_deg, max_arm2_delta);
                    current_angle.arm1_angle = solved_arm1_deg;
                    current_angle.arm2_angle = solved_arm2_deg;
                    last_valid_arm1_deg = solved_arm1_deg;
                    last_valid_arm2_deg = solved_arm2_deg;
                }
            }
        } else if (stage == 2) {
            // 恢复段：
            // UAV 从主段结束位置平滑回到 hover_anchor，
            // 机械臂同步回到 0 度附近，便于后续 landing 或下一次重启实验。
            const double ratio = recovery_time <= 1e-6 ? 1.0 :
                Clamp((elapsed - settle_time - experiment_time) / recovery_time, 0.0, 1.0);
            const Vec3 recovery_target_world = hover_anchor_initialized ? hover_anchor_world : recovery_start_world;
            const Vec3 desired_world = InterpolateVec3(recovery_start_world, recovery_target_world, ratio);
            last_uav_world_command = desired_world;
            current_world_reference = desired_world;
            current_output_position = MapWorldToOutputFrame(
                desired_world, use_local_pose_mapping, frame_mapping_initialized,
                hover_anchor_world, local_anchor);
            uav_pos_d.land_flag = false;
            uav_pos_d.x_d = current_output_position.x;
            uav_pos_d.y_d = current_output_position.y;
            uav_pos_d.z_d = current_output_position.z;
            uav_pos_d.yaw_d = hover_yaw_deg;
            current_angle.arm1_angle = InterpolateVec3(
                {recovery_start_arm1_deg, 0.0, 0.0}, {0.0, 0.0, 0.0}, ratio).x;
            current_angle.arm2_angle = InterpolateVec3(
                {recovery_start_arm2_deg, 0.0, 0.0}, {0.0, 0.0, 0.0}, ratio).x;
            last_valid_arm1_deg = current_angle.arm1_angle;
            last_valid_arm2_deg = current_angle.arm2_angle;

            if ((!base_fresh || (local_required && !local_fresh)) && ++pose_loss_count >= pose_loss_limit) {
                enter_safety_state(ProtectionState::kSafetyHold, "mocap freshness lost in recovery");
            } else if (base_fresh && (!local_required || local_fresh)) {
                pose_loss_count = 0;
            }
        } else if (stage == 3) {
            const Vec3 desired_world = hover_anchor_initialized ? hover_anchor_world : last_uav_world_command;
            current_world_reference = desired_world;
            current_output_position = MapWorldToOutputFrame(
                desired_world, use_local_pose_mapping, frame_mapping_initialized,
                hover_anchor_world, local_anchor);
            uav_pos_d.x_d = current_output_position.x;
            uav_pos_d.y_d = current_output_position.y;
            uav_pos_d.z_d = landing_z;
            uav_pos_d.yaw_d = hover_yaw_deg;
            uav_pos_d.land_flag = true;
            current_angle.arm1_angle = last_valid_arm1_deg;
            current_angle.arm2_angle = last_valid_arm2_deg;
        } else {
            // 保护动作语义统一：
            // - safety_hold    : 保持 safe_hover，不再推进任何实验参考
            // - safety_landing : 保持 safe_hover 的 x/y，再用 landing_z + land_flag 收尾
            const Vec3 hold_world = safe_hover_initialized ? safe_hover_world : fallback_world;
            const Vec3 hold_output_world = MapWorldToOutputFrame(
                hold_world, use_local_pose_mapping, frame_mapping_initialized,
                hover_anchor_world, local_anchor);
            current_world_reference = hold_world;
            current_output_position = SelectFallbackOutputPosition(
                last_safe_output_initialized, last_safe_output_position,
                g_local_pose.valid, g_local_pose, hold_output_world);
            uav_pos_d.x_d = current_output_position.x;
            uav_pos_d.y_d = current_output_position.y;
            uav_pos_d.z_d = (stage == 5) ? landing_z : current_output_position.z;
            uav_pos_d.yaw_d = hover_yaw_deg;
            uav_pos_d.land_flag = (stage == 5);

            const double max_arm1_delta = arm1_dot_max_deg_s * dt;
            const double max_arm2_delta = arm2_dot_max_deg_s * dt;
            current_angle.arm1_angle = RateLimit(safety_arm1_deg, last_valid_arm1_deg, max_arm1_delta);
            current_angle.arm2_angle = RateLimit(safety_arm2_deg, last_valid_arm2_deg, max_arm2_delta);
            last_valid_arm1_deg = current_angle.arm1_angle;
            last_valid_arm2_deg = current_angle.arm2_angle;

            if (stage == 4 && (!base_fresh || (local_required && !local_fresh))) {
                safety_fault_persisted_this_cycle = true;
            }
        }

        // “安全有效指令”的定义：
        // 只有当当前参考通过 freshness、跳变和高度包线检查后，
        // 才允许更新 last_safe_* 缓存。保护触发时保持的必须是这种“安全指令”，
        // 不能简单取“上一帧发过的指令”，否则可能把坏数据也冻结下来。
        const bool stage_requires_ee_fresh_for_safe = (stage == 1);
        const bool command_is_safe =
            protection_state == ProtectionState::kNormal &&
            (stage == 0 || stage == 1 || stage == 2) &&
            base_fresh &&
            (!stage_requires_ee_fresh_for_safe || (ee_fresh && ee_hold_initialized && hover_anchor_initialized)) &&
            (!local_required || local_fresh) &&
            current_output_position.z <= uav_safe_z_max_m &&
            IsJumpAcceptable(current_output_position, last_safe_output_initialized,
                             last_safe_output_position, uav_cmd_jump_threshold_m);
        if (command_is_safe) {
            last_safe_world_initialized = true;
            last_safe_uav_world_command = current_world_reference;
            last_safe_output_initialized = true;
            last_safe_output_position = current_output_position;
            safe_hover_world = SelectSafeHoverWorld(
                hover_anchor_initialized, hover_anchor_world,
                last_safe_world_initialized, last_safe_uav_world_command,
                last_uav_world_command);
            safe_hover_initialized = true;
        }

        // 第二类保护：位置冻结/无响应保护。
        // 故障模型是“刚体丢失后，系统持续发布最后一帧位置”。
        // 这种情况下话题可能还在更新，freshness 也可能没超时，
        // 但飞机的‘实际位置’在到点前长时间几乎不变，所以必须单独检测。
        const bool freeze_detection_enabled = local_fresh && (stage == 0 || stage == 1 || stage == 2 || stage == 4);
        if (freeze_detection_enabled) {
            if (!freeze_window_state.initialized) {
                freeze_window_state.initialized = true;
                freeze_window_state.stamp = now;
                freeze_window_state.position = g_local_pose.position;
            } else if ((now - freeze_window_state.stamp).toSec() >= freeze_window_sec) {
                const Vec3 measured_motion = SubVec3(g_local_pose.position, freeze_window_state.position);
                const Vec3 output_error = SubVec3(
                    Vec3{uav_pos_d.x_d, uav_pos_d.y_d, uav_pos_d.z_d},
                    g_local_pose.position);
                const double motion_norm = NormVec3(measured_motion);
                const double pos_error_norm = NormVec3(output_error);
                const double z_motion = std::fabs(g_local_pose.position.z - freeze_window_state.position.z);
                const double z_error = std::fabs(uav_pos_d.z_d - g_local_pose.position.z);
                const bool position_frozen =
                    pos_error_norm > freeze_arrive_threshold_m && motion_norm < freeze_motion_threshold_m;
                const bool vertical_frozen =
                    z_error > z_freeze_error_threshold_m && z_motion < z_freeze_motion_threshold_m;
                if (position_frozen || vertical_frozen) {
                    ++freeze_fault_count;
                    safety_fault_persisted_this_cycle = true;
                    ROS_WARN_THROTTLE(1.0,
                                      "exp2 freeze guard: pos_error=%.3f, motion=%.3f, z_error=%.3f, z_motion=%.3f (%d/%d)",
                                      pos_error_norm, motion_norm, z_error, z_motion,
                                      freeze_fault_count, freeze_trigger_cycles);
                    if (protection_state == ProtectionState::kNormal &&
                        freeze_fault_count >= freeze_trigger_cycles) {
                        enter_safety_state(
                            ProtectionState::kSafetyHold,
                            vertical_frozen ? "vertical motion frozen before arrival" :
                                              "uav position frozen before arrival");
                    }
                } else {
                    freeze_fault_count = 0;
                }
                freeze_window_state.stamp = now;
                freeze_window_state.position = g_local_pose.position;
            }
        } else {
            freeze_window_state.initialized = false;
            if (protection_state == ProtectionState::kNormal) {
                freeze_fault_count = 0;
            }
        }

        if (protection_state == ProtectionState::kSafetyHold) {
            // safety_hold 不是最终动作，而是“先稳住再决定要不要落”。
            // 若 freshness 丢失或位置冻结在安全悬停阶段仍持续存在，
            // 说明故障不是瞬时抖动，就升级到 safety_landing。
            if (safety_fault_persisted_this_cycle) {
                ++safety_escalate_count;
                if (safety_escalate_count >= freeze_escalate_cycles) {
                    enter_safety_state(ProtectionState::kSafetyLanding, "fault persisted in safety_hold");
                }
            } else {
                safety_escalate_count = 0;
            }
        }

        // 若本周期刚进入 safety_landing，则立即覆盖掉前面生成的普通命令，
        // 保证 land_flag 在同一个控制周期就生效。
        if (protection_state == ProtectionState::kSafetyLanding && stage != 5) {
            const Vec3 hold_world = safe_hover_initialized ? safe_hover_world : fallback_world;
            const Vec3 hold_output_world = MapWorldToOutputFrame(
                hold_world, use_local_pose_mapping, frame_mapping_initialized,
                hover_anchor_world, local_anchor);
            current_output_position = SelectFallbackOutputPosition(
                last_safe_output_initialized, last_safe_output_position,
                g_local_pose.valid, g_local_pose, hold_output_world);
            uav_pos_d.x_d = current_output_position.x;
            uav_pos_d.y_d = current_output_position.y;
            uav_pos_d.z_d = landing_z;
            uav_pos_d.yaw_d = hover_yaw_deg;
            uav_pos_d.land_flag = true;
            stage = 5;
        } else if (protection_state == ProtectionState::kSafetyHold && stage != 4) {
            const Vec3 hold_world = safe_hover_initialized ? safe_hover_world : fallback_world;
            const Vec3 hold_output_world = MapWorldToOutputFrame(
                hold_world, use_local_pose_mapping, frame_mapping_initialized,
                hover_anchor_world, local_anchor);
            current_output_position = SelectFallbackOutputPosition(
                last_safe_output_initialized, last_safe_output_position,
                g_local_pose.valid, g_local_pose, hold_output_world);
            uav_pos_d.x_d = current_output_position.x;
            uav_pos_d.y_d = current_output_position.y;
            uav_pos_d.z_d = current_output_position.z;
            uav_pos_d.yaw_d = hover_yaw_deg;
            uav_pos_d.land_flag = false;
            stage = 4;
        }

        current_angle.arm1_angle = Clamp(current_angle.arm1_angle, arm1_min, arm1_max);
        current_angle.arm2_angle = Clamp(current_angle.arm2_angle, arm2_min, arm2_max);
        current_angle.hand_angle = Clamp(current_angle.hand_angle, hand_min, hand_max);

        // 在线标定结果也作为单独话题发布，便于录包和离线分析。
        online_offset_msg.arm1_angle = calibrated_arm1_zero_offset_deg;
        online_offset_msg.arm2_angle = calibrated_arm2_zero_offset_deg;
        online_offset_msg.hand_angle = joint_offset_initialized ? 1.0 : 0.0;

        // 1 Hz 日志：看状态机、末端误差、mocap 新鲜度和 IK 是否稳定。
        if ((now - last_log_time).toSec() >= 1.0) {
            last_log_time = now;
            const double ee_error_norm =
                (ee_hold_initialized && g_ee_pose.valid) ? NormVec3(SubVec3(ee_hold_world, g_ee_pose.position)) : -1.0;
            ROS_INFO(
                "[%s|%s] t=%.2f s, pose_d=(%.2f, %.2f, %.2f, %.2f), arm_d=(%.2f, %.2f, %.2f), base_fresh=%s, ee_fresh=%s, local_fresh=%s, ee_err=%.4f, pose_loss=%d, freeze_fault=%d, safety_escalate=%d, ik_fail=%d, offset=(%.2f, %.2f), base_offset=(%.4f, %.4f, %.4f)",
                StageName(stage), ProtectionStateName(protection_state), elapsed,
                uav_pos_d.x_d, uav_pos_d.y_d, uav_pos_d.z_d, uav_pos_d.yaw_d,
                current_angle.arm1_angle, current_angle.arm2_angle, current_angle.hand_angle,
                base_fresh ? "true" : "false", ee_fresh ? "true" : "false", local_fresh ? "true" : "false",
                ee_error_norm, pose_loss_count, freeze_fault_count, safety_escalate_count, ik_fail_count,
                calibrated_arm1_zero_offset_deg, calibrated_arm2_zero_offset_deg,
                arm_base_offset_x_m, arm_base_offset_y_m, arm_base_offset_z_m);
        }

        uav_pos_d_pub.publish(uav_pos_d);
        joint_angle_pub.publish(current_angle);
        online_offset_pub.publish(online_offset_msg);
        rate.sleep();
    }

    PublishReturnHome(0.0, 0.0, current_angle.hand_angle);
    std::cout << "exp2 finished" << std::endl;
    return 0;
}
