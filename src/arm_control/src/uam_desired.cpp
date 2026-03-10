#include <ros/ros.h>

#include <cmath>
#include <iostream>
#include <string>

#include "uam_message/arm_angle.h"
#include "uav/desired_start.h"
#include "uav/xyz_yaw_d.h"

namespace {

bool start_flag = false;

double Clamp(double value, double min_value, double max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
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

    double hover_x = 0.0;
    double hover_y = 0.0;
    double hover_z = 1.0;
    double hover_yaw_deg = 0.0;
    double settle_time = 3.0;
    double experiment_time = 20.0;
    double recovery_time = 2.0;
    double arm1_hold_deg = 0.0;
    double arm2_offset_deg = 0.0;
    double arm2_amp_deg = 15.0;
    double arm2_period = 4.0;
    double arm2_sign = 1.0;
    double hand_hold_deg = 0.0;

    nh.param("/wjl/uam/hover_x", hover_x, hover_x);
    nh.param("/wjl/uam/hover_y", hover_y, hover_y);
    nh.param("/wjl/uam/hover_z", hover_z, hover_z);
    pnh.param("hover_x", hover_x, hover_x);
    pnh.param("hover_y", hover_y, hover_y);
    pnh.param("hover_z", hover_z, hover_z);
    pnh.param("hover_yaw_deg", hover_yaw_deg, hover_yaw_deg);
    pnh.param("settle_time", settle_time, settle_time);
    pnh.param("experiment_time", experiment_time, experiment_time);
    pnh.param("recovery_time", recovery_time, recovery_time);
    pnh.param("arm1_hold_deg", arm1_hold_deg, arm1_hold_deg);
    pnh.param("arm2_offset_deg", arm2_offset_deg, arm2_offset_deg);
    pnh.param("arm2_amp_deg", arm2_amp_deg, arm2_amp_deg);
    pnh.param("arm2_period", arm2_period, arm2_period);
    pnh.param("arm2_sign", arm2_sign, arm2_sign);
    pnh.param("hand_hold_deg", hand_hold_deg, hand_hold_deg);

    if (arm2_period <= 0.0) {
        ROS_WARN("参数 arm2_period<=0，已回退为默认值 4.0 s");
        arm2_period = 4.0;
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

    arm1_hold_deg = Clamp(arm1_hold_deg, arm1_min, arm1_max);
    arm2_offset_deg = Clamp(arm2_offset_deg, arm2_min, arm2_max);
    hand_hold_deg = Clamp(hand_hold_deg, hand_min, hand_max);

    ROS_INFO("服务已经启动，等待实验触发....");
    ROS_INFO(
        "uam_desired params: hover=(%.2f, %.2f, %.2f), yaw=%.2f deg, settle=%.2f s, exp=%.2f s, recovery=%.2f s, arm2_amp=%.2f deg, arm2_period=%.2f s, arm2_sign=%.2f",
        hover_x, hover_y, hover_z, hover_yaw_deg, settle_time, experiment_time, recovery_time,
        arm2_amp_deg, arm2_period, arm2_sign);

    ros::Rate rate(30.0);
    while (ros::ok() && !start_flag) {
        ros::spinOnce();
        rate.sleep();
    }
    if (!ros::ok()) {
        return 0;
    }

    std::cout << "开始发送期望数据：" << std::endl;

    uav::xyz_yaw_d uav_pos_d;
    uam_message::arm_angle current_angle;
    uav_pos_d.x_d = hover_x;
    uav_pos_d.y_d = hover_y;
    uav_pos_d.z_d = hover_z;
    uav_pos_d.yaw_d = hover_yaw_deg;
    uav_pos_d.land_flag = false;
    current_angle.arm1_angle = arm1_hold_deg;
    current_angle.arm2_angle = arm2_offset_deg;
    current_angle.hand_angle = hand_hold_deg;

    const ros::Duration landing_publish_time(0.5);
    const double two_pi = 2.0 * std::acos(-1.0);
    const ros::Time experiment_start = ros::Time::now();
    ros::Time last_log_time = experiment_start - ros::Duration(1.0);

    while (ros::ok()) {
        ros::spinOnce();

        const ros::Time now = ros::Time::now();
        const double elapsed = (now - experiment_start).toSec();
        int stage = 0;

        if (elapsed < settle_time) {
            stage = 0;
            uav_pos_d.land_flag = false;
            current_angle.arm1_angle = arm1_hold_deg;
            current_angle.arm2_angle = arm2_offset_deg;
            current_angle.hand_angle = hand_hold_deg;
        } else if (elapsed < settle_time + experiment_time) {
            stage = 1;
            const double disturb_time = elapsed - settle_time;
            const double arm2_cmd =
                arm2_offset_deg +
                arm2_sign * arm2_amp_deg * std::sin(two_pi * disturb_time / arm2_period);
            uav_pos_d.land_flag = false;
            current_angle.arm1_angle = arm1_hold_deg;
            current_angle.arm2_angle = arm2_cmd;
            current_angle.hand_angle = hand_hold_deg;
        } else if (elapsed < settle_time + experiment_time + recovery_time) {
            stage = 2;
            uav_pos_d.land_flag = false;
            current_angle.arm1_angle = arm1_hold_deg;
            current_angle.arm2_angle = arm2_offset_deg;
            current_angle.hand_angle = hand_hold_deg;
        } else if (elapsed < settle_time + experiment_time + recovery_time +
                               landing_publish_time.toSec()) {
            stage = 3;
            uav_pos_d.z_d = 0.5;
            uav_pos_d.land_flag = true;
            current_angle.arm1_angle = arm1_hold_deg;
            current_angle.arm2_angle = arm2_offset_deg;
            current_angle.hand_angle = hand_hold_deg;
        } else {
            break;
        }

        current_angle.arm1_angle = Clamp(current_angle.arm1_angle, arm1_min, arm1_max);
        current_angle.arm2_angle = Clamp(current_angle.arm2_angle, arm2_min, arm2_max);
        current_angle.hand_angle = Clamp(current_angle.hand_angle, hand_min, hand_max);

        uav_pos_d.x_d = hover_x;
        uav_pos_d.y_d = hover_y;
        uav_pos_d.yaw_d = hover_yaw_deg;
        if (stage != 3) {
            uav_pos_d.z_d = hover_z;
        }

        if ((now - last_log_time).toSec() >= 1.0) {
            last_log_time = now;
            ROS_INFO(
                "[%s] t=%.2f s, pose_d=(%.2f, %.2f, %.2f, %.2f), arm_d=(%.2f, %.2f, %.2f), land=%s",
                StageName(stage), elapsed, uav_pos_d.x_d, uav_pos_d.y_d, uav_pos_d.z_d,
                uav_pos_d.yaw_d, current_angle.arm1_angle, current_angle.arm2_angle,
                current_angle.hand_angle, uav_pos_d.land_flag ? "true" : "false");
        }

        uav_pos_d_pub.publish(uav_pos_d);
        joint_angle_pub.publish(current_angle);
        rate.sleep();
    }

    std::cout << "结束飞行！！！" << std::endl;
    return 0;
}
