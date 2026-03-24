#include <ros/ros.h>

#include <cmath>
#include <utility>
#include <vector>

#include <std_msgs/Int32.h>

#include "uam_message/arm_angle.h"

namespace {

// 保存最近一次真机机械臂反馈。
// 这里的数据来自 /wjl/arm/real/angle_r，不是动捕，而是串口驱动从步进电机回读后的角度。
struct ArmState {
    bool valid = false;
    double arm1_deg = 0.0;
    double arm2_deg = 0.0;
    double hand_deg = 0.0;
};

ArmState g_real_arm_state;

// 真机反馈回调：持续刷新当前真实关节角。
void ArmRealCb(const uam_message::arm_angle::ConstPtr &msg) {
    g_real_arm_state.valid = true;
    g_real_arm_state.arm1_deg = msg->arm1_angle;
    g_real_arm_state.arm2_deg = msg->arm2_angle;
    g_real_arm_state.hand_deg = msg->hand_angle;
}

// 用于判定两个关节中“最坏的那个误差”是否已经进入容差范围。
double AbsMax(double a, double b) {
    return std::max(std::fabs(a), std::fabs(b));
}

// 默认静态采样姿态序列。
// 这些点覆盖 0/15/30/45/55 度五层 arm2，与 -60 到 60 度的 arm1 扫描。
// 顺序不是按分层简单拼接，而是人工重排成相邻样本变化较小的路径，
// 让真机采样时尽量减少大幅反向跳变与超时。
std::vector<std::pair<double, double>> DefaultSamples() {
    return {
        {-60.0, 0.0},
        {-45.0, 0.0},
        {-30.0, 0.0},
        {-15.0, 0.0},
        {0.0, 0.0},
        {15.0, 0.0},
        {30.0, 0.0},
        {45.0, 0.0},
        {60.0, 0.0},
        {60.0, 15.0},
        {30.0, 15.0},
        {0.0, 15.0},
        {-30.0, 15.0},
        {-60.0, 15.0},
        {-45.0, 30.0},
        {-15.0, 30.0},
        {15.0, 30.0},
        {45.0, 30.0},
        {30.0, 45.0},
        {0.0, 45.0},
        {-30.0, 45.0},
        {-15.0, 55.0},
        {0.0, 55.0},
        {15.0, 55.0},
    };
}

}  // namespace

int main(int argc, char **argv) {
    ros::init(argc, argv, "uam_static_calibration_collect");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // 下面这些参数共同决定“什么时候认为某组姿态到位，什么时候开始记样本”。
    double publish_rate_hz = 30.0;
    double settle_confirm_sec = 0.5;
    double sample_hold_sec = 2.5;
    double sample_timeout_sec = 15.0;
    double angle_tolerance_deg = 1.0;
    double default_hand_angle_deg = 0.0;

    pnh.param("publish_rate_hz", publish_rate_hz, publish_rate_hz);
    pnh.param("settle_confirm_sec", settle_confirm_sec, settle_confirm_sec);
    pnh.param("sample_hold_sec", sample_hold_sec, sample_hold_sec);
    pnh.param("sample_timeout_sec", sample_timeout_sec, sample_timeout_sec);
    pnh.param("angle_tolerance_deg", angle_tolerance_deg, angle_tolerance_deg);
    pnh.param("default_hand_angle_deg", default_hand_angle_deg, default_hand_angle_deg);

    if (publish_rate_hz <= 0.0) {
        publish_rate_hz = 30.0;
    }
    if (settle_confirm_sec <= 0.0) {
        settle_confirm_sec = 0.5;
    }
    if (sample_hold_sec <= 0.0) {
        sample_hold_sec = 2.5;
    }
    if (sample_timeout_sec <= 0.0) {
        sample_timeout_sec = 15.0;
    }
    if (angle_tolerance_deg <= 0.0) {
        angle_tolerance_deg = 1.0;
    }

    const std::vector<std::pair<double, double>> samples = DefaultSamples();

    // 信息流：
    // 1. arm_pub 向 /wjl/arm/guidefly/angle_d 发布当前目标角；
    // 2. motors_simulation 把它桥接到 /wjl/arm/real/angle_d；
    // 3. serial_ 读 /wjl/arm/real/angle_d 后驱动真机，并把反馈发到 /wjl/arm/real/angle_r；
    // 4. 本节点再订阅 /wjl/arm/real/angle_r，用于判断收敛；
    // 5. 本节点额外发布 /wjl/calibration/sample_index，给录包和拟合脚本做“有效样本标签”。
    ros::Publisher arm_pub = nh.advertise<uam_message::arm_angle>("/wjl/arm/guidefly/angle_d", 10);
    ros::Publisher sample_index_pub = nh.advertise<std_msgs::Int32>("/wjl/calibration/sample_index", 10, true);
    ros::Subscriber arm_sub = nh.subscribe<uam_message::arm_angle>("/wjl/arm/real/angle_r", 10, ArmRealCb);

    ROS_INFO("static calibration collector ready: %zu samples, hold=%.2f s, confirm=%.2f s, tol=%.2f deg, timeout=%.2f s",
             samples.size(), sample_hold_sec, settle_confirm_sec, angle_tolerance_deg, sample_timeout_sec);

    // kMove：向目标姿态运动，但 sample_index=-1，不把数据计入拟合；
    // kSample：已确认到位，sample_index=当前样本号，这段数据才用于后处理取均值。
    enum class Stage {
        kMove,
        kSample,
    };

    std_msgs::Int32 sample_index_msg;
    sample_index_msg.data = -1;
    sample_index_pub.publish(sample_index_msg);

    bool hand_initialized = false;
    double hand_angle_deg = default_hand_angle_deg;
    std::size_t sample_idx = 0;
    Stage stage = Stage::kMove;
    ros::Time stage_start = ros::Time::now();
    ros::Time tolerance_hold_start;
    ros::Time sample_start;

    ros::Rate rate(publish_rate_hz);
    while (ros::ok()) {
        ros::spinOnce();
        const ros::Time now = ros::Time::now();

        // 手爪角在整个静态标定过程中保持不动。
        // 如果已经收到真机反馈，就锁定为启动时的真实手爪角；否则用默认值。
        if (!hand_initialized && g_real_arm_state.valid) {
            hand_angle_deg = g_real_arm_state.hand_deg;
            hand_initialized = true;
        }

        if (sample_idx >= samples.size()) {
            sample_index_msg.data = -1;
            sample_index_pub.publish(sample_index_msg);
            ROS_INFO("static calibration collector finished all %zu samples", samples.size());
            break;
        }

        const double target_arm1 = samples[sample_idx].first;
        const double target_arm2 = samples[sample_idx].second;

        // 连续发布目标角，保证即使串口侧或桥接节点短暂丢帧，目标仍会被刷新。
        uam_message::arm_angle cmd;
        cmd.arm1_angle = target_arm1;
        cmd.arm2_angle = target_arm2;
        cmd.hand_angle = hand_angle_deg;
        arm_pub.publish(cmd);

        // sample_index 的意义：
        // -1  表示当前处于移动/收敛阶段；
        // >=0 表示当前处于有效采样窗口，录包和拟合脚本会按这个编号切段。
        sample_index_msg.data = (stage == Stage::kSample) ? static_cast<int>(sample_idx) : -1;
        sample_index_pub.publish(sample_index_msg);

        if (!g_real_arm_state.valid) {
            ROS_WARN_THROTTLE(1.0, "static calibration collector waiting for /wjl/arm/real/angle_r");
            rate.sleep();
            continue;
        }

        const double arm1_error = target_arm1 - g_real_arm_state.arm1_deg;
        const double arm2_error = target_arm2 - g_real_arm_state.arm2_deg;
        const bool within_tolerance = AbsMax(arm1_error, arm2_error) <= angle_tolerance_deg;

        if (stage == Stage::kMove) {
            // 收敛判定不是“瞬间误差进阈值”就算完成，而是必须持续一段 settle_confirm_sec。
            // 这样可以避免机械臂在阈值边缘来回抖动时误触发采样。
            if (within_tolerance) {
                if (tolerance_hold_start.isZero()) {
                    tolerance_hold_start = now;
                }
                if ((now - tolerance_hold_start).toSec() >= settle_confirm_sec) {
                    stage = Stage::kSample;
                    sample_start = now;
                    sample_index_msg.data = static_cast<int>(sample_idx);
                    sample_index_pub.publish(sample_index_msg);
                    ROS_INFO("static calibration sample %zu/%zu entered hold window: target=(%.1f, %.1f)",
                             sample_idx + 1, samples.size(), target_arm1, target_arm2);
                }
            } else {
                tolerance_hold_start = ros::Time();
            }

            // 如果长时间无法到位，直接报错退出，避免录到质量很差的伪静态样本。
            if ((now - stage_start).toSec() > sample_timeout_sec) {
                ROS_ERROR("static calibration sample %zu/%zu failed to converge within %.2f s: target=(%.1f, %.1f), real=(%.2f, %.2f), error=(%.2f, %.2f)",
                          sample_idx + 1, samples.size(), sample_timeout_sec,
                          target_arm1, target_arm2,
                          g_real_arm_state.arm1_deg, g_real_arm_state.arm2_deg,
                          arm1_error, arm2_error);
                return 1;
            }
        } else {
            // 一旦进入有效窗口，就固定保持 sample_hold_sec。
            // 后处理脚本会在这段窗口内对动捕和角度数据做均值。
            if ((now - sample_start).toSec() >= sample_hold_sec) {
                ROS_INFO("static calibration sample %zu/%zu completed: target=(%.1f, %.1f), real=(%.2f, %.2f)",
                         sample_idx + 1, samples.size(), target_arm1, target_arm2,
                         g_real_arm_state.arm1_deg, g_real_arm_state.arm2_deg);
                ++sample_idx;
                stage = Stage::kMove;
                stage_start = now;
                tolerance_hold_start = ros::Time();
                sample_index_msg.data = -1;
                sample_index_pub.publish(sample_index_msg);
                if (sample_idx < samples.size()) {
                    ROS_INFO("static calibration switching to sample %zu/%zu: target=(%.1f, %.1f)",
                             sample_idx + 1, samples.size(),
                             samples[sample_idx].first, samples[sample_idx].second);
                }
            }
        }

        rate.sleep();
    }

    return 0;
}
