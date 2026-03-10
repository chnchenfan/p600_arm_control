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

// 服务触发标志。
// 与实验一保持同样的启动方式：先由 UAV 侧完成起飞/到达起点，
// 然后通过 /wjl/start/uav_desired 触发机械臂与实验轨迹。
bool start_flag = false;

// 简单三维向量，专门用于本文件内部表达位置。
// 不引入 Eigen，是为了让实验节点保持轻量，减少编译依赖。
struct Vec3 {
    double x;
    double y;
    double z;
};

// 基座实际位姿缓存。
// 实验二里机械臂补偿使用的是“无人机当前实际位置”，而不是期望位置，
// 因此这里持续缓存 /mavros/local_position/pose 的最新值。
struct BasePoseState {
    bool valid = false;
    Vec3 position{0.0, 0.0, 0.0};
    double yaw_rad = 0.0;
};

// 机械臂实测关节角缓存。
// 主要作用有两个：
// 1. 进入实验二时，用当前真实关节角估计当前世界系末端点；
// 2. 逆解失败时，回退到“上一帧有效解/当前真实角”而不是突然跳零。
struct ArmState {
    bool valid = false;
    double arm1_deg = 0.0;
    double arm2_deg = 0.0;
    double hand_deg = 0.0;
};

BasePoseState g_base_pose;
ArmState g_real_arm_state;

// 通用限幅函数。机械臂关节角和部分参数回退都复用这一套逻辑。
double Clamp(double value, double min_value, double max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

// 度弧度转换。整个文件内部数学计算一律用弧度，和 ROS 参数/日志交互时用角度。
double DegToRad(double value_deg) {
    return value_deg * std::acos(-1.0) / 180.0;
}

double RadToDeg(double value_rad) {
    return value_rad * 180.0 / std::acos(-1.0);
}

// 将角度包装到 [-pi, pi]，用于做分支连续性判断。
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

// 将新角度尽量调整到“靠近参考角”的等价角附近。
// 这样可以避免反解得到的角度在 +180/-180 附近突然跳支。
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

// 计算两个角之间的最小环形距离，用于给逆解候选解打分。
double AngularDistance(double angle_a_rad, double angle_b_rad) {
    return std::fabs(WrapToPi(angle_a_rad - angle_b_rad));
}

// 从四元数中提取 yaw。
// 实验二首版只使用基座平面位置 + yaw，不显式补偿 roll/pitch。
double QuaternionToYaw(double x, double y, double z, double w) {
    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    return std::atan2(siny_cosp, cosy_cosp);
}

// 只考虑 yaw 的二维旋转。
// 这是实验二“简化版”的关键假设：在小圆轨迹和近悬停条件下，
// 先忽略 roll/pitch 对末端位置的高阶影响。
Vec3 RotateYawOnly(const Vec3 &vector_body, double yaw_rad) {
    const double cos_yaw = std::cos(yaw_rad);
    const double sin_yaw = std::sin(yaw_rad);
    return {cos_yaw * vector_body.x - sin_yaw * vector_body.y,
            sin_yaw * vector_body.x + cos_yaw * vector_body.y,
            vector_body.z};
}

// 世界系 -> 机体系 的 yaw 逆旋转。
// 先把世界系下“末端固定点相对基座的位置”转回机体系，
// 然后再对 2DoF 机械臂做逆运动学。
Vec3 InverseRotateYawOnly(const Vec3 &vector_world, double yaw_rad) {
    return RotateYawOnly(vector_world, -yaw_rad);
}

// 当前 2DoF 串联机械臂的正运动学。
// 返回值是末端在机体系下的位置，参数与绘图脚本和 arm_B_dh.py 保持一致。
// 这里的几何常数来自现有 DH 模型，不重新建模。
Vec3 ForwardKinematicsBody(double arm1_deg, double arm2_deg) {
    const double q1 = DegToRad(arm1_deg);
    const double q2 = DegToRad(arm2_deg);
    return {
        0.104 + 0.21 * std::cos(q1) * std::cos(q2) - 0.007 * std::sin(q1),
        -0.21 * std::sin(q1) * std::cos(q2) - 0.007 * std::cos(q1),
        0.04894 + 0.21 * std::sin(q2)};
}

// 实验二核心：根据机体系末端目标位置反解 arm1 / arm2。
//
// 设计目标不是做通用求解器，而是满足本实验“小圆轨迹 + 末端固定点”的工程需求：
// 1. 只针对当前 2DoF 几何模型求解；
// 2. 只解位置，不解姿态；
// 3. 通过上一时刻解做连续性选择，避免跳支；
// 4. 若超出工作空间则直接返回 false，由上层做保护。
bool SolveInverseKinematics(const Vec3 &target_body,
                            double previous_arm1_deg,
                            double previous_arm2_deg,
                            double arm1_min_deg,
                            double arm1_max_deg,
                            double arm2_min_deg,
                            double arm2_max_deg,
                            double *arm1_deg_out,
                            double *arm2_deg_out) {
    // 先利用 z 方向反解 q2。若 z 已超出连杆长度可达范围，直接判不可达。
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

    // q2 理论上有两组解，逐个评估：
    // 1. 是否满足关节限位
    // 2. 回代正解后的末端位置误差
    // 3. 与上一帧解的连续性代价
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

        // 位置误差优先，连续性次之。
        // 这样既能尽量准确到达目标点，也能避免解在不同分支之间抖动。
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

// 状态机阶段名称，仅用于日志输出。
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

// 与实验一相同的服务触发逻辑。
bool doReq(uav::desired_start::Request &req, uav::desired_start::Response &resp) {
    if (req.desired_start != 10) {
        ROS_ERROR("提交的数据异常!!!");
        return false;
    }
    resp.desired_sent = 3;
    start_flag = true;
    return true;
}

// 缓存基座实际位姿。
void BasePoseCb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    g_base_pose.valid = true;
    g_base_pose.position = {msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
    g_base_pose.yaw_rad = QuaternionToYaw(msg->pose.orientation.x,
                                          msg->pose.orientation.y,
                                          msg->pose.orientation.z,
                                          msg->pose.orientation.w);
}

// 缓存机械臂实际关节角。
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

    // 实验二参数分三类：
    // 1. 基座圆轨迹参数
    // 2. 实验时序参数
    // 3. 末端固定点与 IK 保护参数
    double circle_center_x = 0.0;
    double circle_center_y = 0.0;
    double circle_radius = 0.10;
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
    bool use_initial_circle_center = true;

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
    pnh.param("use_initial_circle_center", use_initial_circle_center, use_initial_circle_center);

    // 保护性参数回退，避免 launch/参数服务器给出明显无效值。
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

    // 实验二订阅两路反馈：
    // 1. 基座实际位姿：用于实时补偿
    // 2. 机械臂实际角：用于冻结“当前末端世界坐标”以及失败回退
    ros::Subscriber base_pose_sub =
        nh.subscribe<geometry_msgs::PoseStamped>(base_pose_topic, 10, BasePoseCb);
    ros::Subscriber arm_real_sub =
        nh.subscribe<uam_message::arm_angle>("/wjl/arm/real/angle_r", 10, ArmRealCb);

    ROS_INFO("exp2 service ready, waiting for /wjl/start/uav_desired");
    ROS_INFO(
        "exp2 params: center=(%.2f, %.2f), radius=%.3f, T=%.2f, hover_z=%.2f, yaw=%.2f, settle=%.2f, exp=%.2f, recovery=%.2f, pose_topic=%s, use_initial_circle_center=%s",
        circle_center_x, circle_center_y, circle_radius, circle_period, hover_z, hover_yaw_deg,
        settle_time, experiment_time, recovery_time, base_pose_topic.c_str(),
        use_initial_circle_center ? "true" : "false");

    ros::Rate rate(30.0);
    while (ros::ok() && !start_flag) {
        ros::spinOnce();
        rate.sleep();
    }
    if (!ros::ok()) {
        return 0;
    }

    // 初始化默认输出。
    // 实验开始前保持在悬停点上方，并尽量以当前机械臂真实角作为起点，
    // 避免刚进入实验二就发生关节突跳。
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

    // 这些变量在整个实验二循环里持续维护：
    // ee_hold_initialized: 是否已经确定“世界系固定末端点”
    // ee_hold_world:       末端固定点在世界系中的坐标
    // ik_fail_count:       连续 IK 失败次数
    // last_valid_arm*:     上一帧有效关节解，用于保持和连续性选择
    // force_recovery:      若连续失败过多，强制跳到 recovery
    const ros::Time experiment_start = ros::Time::now();
    ros::Time last_log_time = experiment_start - ros::Duration(1.0);
    const ros::Duration landing_publish_time(0.5);
    const double two_pi = 2.0 * std::acos(-1.0);

    bool ee_hold_initialized = false;
    Vec3 ee_hold_world{exp2_hold_ee_x, exp2_hold_ee_y, exp2_hold_ee_z};
    bool circle_center_initialized = !use_initial_circle_center;
    double active_circle_center_x = circle_center_x;
    double active_circle_center_y = circle_center_y;
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

        // 实验二状态机：
        // 0. settle      悬停稳定
        // 1. circle_hold 基座画圆 + 末端固定
        // 2. recovery    恢复悬停
        // 3. landing     触发降落
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

        if (!circle_center_initialized && g_base_pose.valid) {
            active_circle_center_x = g_base_pose.position.x;
            active_circle_center_y = g_base_pose.position.y;
            circle_center_initialized = true;
            ROS_INFO("exp2 circle center initialized from current base pose: (%.3f, %.3f)",
                     active_circle_center_x, active_circle_center_y);
        }

        uav_pos_d.land_flag = false;
        uav_pos_d.x_d = active_circle_center_x;
        uav_pos_d.y_d = active_circle_center_y;
        uav_pos_d.z_d = hover_z;
        uav_pos_d.yaw_d = hover_yaw_deg;
        current_angle.hand_angle = Clamp(current_angle.hand_angle, hand_min, hand_max);

        if (stage == 0) {
            // 稳定段不主动改变机械臂，只保持当前有效关节角，
            // 目的是先让基座稳定，再进入实验二主段。
            current_angle.arm1_angle = last_valid_arm1_deg;
            current_angle.arm2_angle = last_valid_arm2_deg;
        } else if (stage == 1) {
            // 基座圆轨迹参考。
            // 注意：这里发布的是无人机“期望圆轨迹”，真正补偿时仍使用实际基座位姿。
            const double circle_time = elapsed - settle_time;
            const double phase = two_pi * circle_time / circle_period;
            uav_pos_d.x_d = active_circle_center_x + circle_radius * std::cos(phase);
            uav_pos_d.y_d = active_circle_center_y + circle_radius * std::sin(phase);

            if (!ee_hold_initialized) {
                if (exp2_use_initial_ee_hold) {
                    if (g_base_pose.valid) {
                        // 首次进入主段时，将“当前末端世界坐标”冻结为固定目标点。
                        // 这样实验二不需要你手工提前量一个世界坐标目标，更适合真机联调。
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
                    // 也支持显式给定世界系固定点，适合后续做固定工位或对比实验。
                    ee_hold_initialized = true;
                    ROS_INFO("exp2 hold point initialized from params: (%.3f, %.3f, %.3f)",
                             ee_hold_world.x, ee_hold_world.y, ee_hold_world.z);
                }
            }

            if (!g_base_pose.valid || !ee_hold_initialized) {
                // 只要实际基座位姿或末端固定点还没准备好，就保持上一帧有效关节角，
                // 不要发布跳变或无意义命令。
                current_angle.arm1_angle = last_valid_arm1_deg;
                current_angle.arm2_angle = last_valid_arm2_deg;
                ROS_WARN_THROTTLE(1.0, "exp2 missing base pose or hold target, keeping last valid arm command");
            } else {
                // 末端固定点是世界系量，机械臂逆解需要的是“相对基座”的机体系目标。
                // 这里先算世界系相对位移，再按 yaw 逆旋转回机体系。
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
                    // IK 单次失败时不立即退出，而是先保持上一帧有效解。
                    // 这样可以滤掉瞬时噪声、边界点或姿态抖动带来的偶发不可达。
                    ++ik_fail_count;
                    current_angle.arm1_angle = last_valid_arm1_deg;
                    current_angle.arm2_angle = last_valid_arm2_deg;
                    ROS_WARN_THROTTLE(1.0, "exp2 IK failed (%d/%d), target_body=(%.3f, %.3f, %.3f)",
                                      ik_fail_count, ik_fail_limit, target_body.x, target_body.y, target_body.z);
                    if (ik_fail_count >= ik_fail_limit) {
                        // 连续失败超过阈值后，不再硬撑着继续画圆，
                        // 直接切换到 recovery，避免无人机被错误机械臂命令拖坏。
                        force_recovery = true;
                        ROS_ERROR("exp2 IK failed continuously, switching to recovery stage");
                    }
                } else {
                    // 一旦求解成功，就清零失败计数并更新“上一帧有效解”。
                    ik_fail_count = 0;
                    current_angle.arm1_angle = solved_arm1_deg;
                    current_angle.arm2_angle = solved_arm2_deg;
                    last_valid_arm1_deg = solved_arm1_deg;
                    last_valid_arm2_deg = solved_arm2_deg;
                }
            }
        } else if (stage == 2) {
            // 恢复段：基座回中心悬停，机械臂保持最后一帧有效解。
            // 这里首版不强行回零，避免末端突然丢掉固定点引起额外扰动。
            current_angle.arm1_angle = last_valid_arm1_deg;
            current_angle.arm2_angle = last_valid_arm2_deg;
        } else if (stage == 3) {
            // 降落段：复用实验一的策略，把 z_d 设到 0.5 并置 land_flag=true，
            // 由 UAV 侧已有逻辑完成最终收尾。
            uav_pos_d.z_d = 0.5;
            uav_pos_d.land_flag = true;
            current_angle.arm1_angle = last_valid_arm1_deg;
            current_angle.arm2_angle = last_valid_arm2_deg;
        }

        current_angle.arm1_angle = Clamp(current_angle.arm1_angle, arm1_min, arm1_max);
        current_angle.arm2_angle = Clamp(current_angle.arm2_angle, arm2_min, arm2_max);
        current_angle.hand_angle = Clamp(current_angle.hand_angle, hand_min, hand_max);

        // 1 Hz 日志足够看清实验是否进入了圆轨迹阶段、是否收到基座位姿、
        // IK 是否在持续失败，以及当前发给机械臂/无人机的参考值是什么。
        if ((now - last_log_time).toSec() >= 1.0) {
            last_log_time = now;
            ROS_INFO("[%s] t=%.2f s, pose_d=(%.2f, %.2f, %.2f, %.2f), arm_d=(%.2f, %.2f, %.2f), base_valid=%s, ik_fail=%d",
                     StageName(stage), elapsed, uav_pos_d.x_d, uav_pos_d.y_d, uav_pos_d.z_d,
                     uav_pos_d.yaw_d, current_angle.arm1_angle, current_angle.arm2_angle,
                     current_angle.hand_angle, g_base_pose.valid ? "true" : "false", ik_fail_count);
        }

        // 每个循环同时发布基座期望和机械臂期望，保持与实验一相同的链路结构。
        uav_pos_d_pub.publish(uav_pos_d);
        joint_angle_pub.publish(current_angle);
        rate.sleep();
    }

    std::cout << "exp2 finished" << std::endl;
    return 0;
}
