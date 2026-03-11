#include <ros/ros.h>

#include <cmath>
#include <iostream>
#include <string>

#include <geometry_msgs/PoseStamped.h>

#include "uam_message/arm_angle.h"
#include "uav/desired_start.h"
#include "uav/xyz_yaw_d.h"

namespace {

bool start_flag = false;

struct BasePoseState {
    bool valid = false;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

BasePoseState g_base_pose;

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
    g_base_pose.x = msg->pose.position.x;
    g_base_pose.y = msg->pose.position.y;
    g_base_pose.z = msg->pose.position.z;
}

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

    double hover_z = 1.0;
    double hover_yaw_deg = 0.0;
    double square_side_length = 0.7;
    double edge_time = 7.0;
    double settle_time = 3.0;
    double recovery_time = 2.0;
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
    if (arm1_period <= 0.0) {
        arm1_period = 3.5;
    }
    if (arm2_period <= 0.0) {
        arm2_period = 3.5;
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

    const double square_motion_time = 4.0 * edge_time;
    const ros::Duration landing_publish_time(0.5);
    const double two_pi = 2.0 * std::acos(-1.0);
    const double arm1_phase_rad = DegToRad(arm1_phase_deg);
    const double arm2_phase_rad = DegToRad(arm2_phase_deg);

    ROS_INFO("exp3 service ready, waiting for /wjl/start/uav_desired");
    ROS_INFO(
        "exp3 params: hover_z=%.2f, yaw=%.2f deg, side=%.2f m, edge_time=%.2f s, settle=%.2f s, recovery=%.2f s, arm1=(offset %.2f, amp %.2f, T %.2f, phase %.2f), arm2=(offset %.2f, amp %.2f, T %.2f, phase %.2f)",
        hover_z, hover_yaw_deg, square_side_length, edge_time, settle_time, recovery_time,
        arm1_offset_deg, arm1_amp_deg, arm1_period, arm1_phase_deg, arm2_offset_deg,
        arm2_amp_deg, arm2_period, arm2_phase_deg);

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
    uav_pos_d.x_d = 0.0;
    uav_pos_d.y_d = 0.0;
    uav_pos_d.z_d = hover_z;
    uav_pos_d.yaw_d = hover_yaw_deg;
    uav_pos_d.land_flag = false;
    current_angle.arm1_angle = arm1_offset_deg;
    current_angle.arm2_angle = arm2_offset_deg;
    current_angle.hand_angle = hand_hold_deg;

    bool square_origin_initialized = false;
    double square_origin_x = 0.0;
    double square_origin_y = 0.0;

    const ros::Time experiment_start = ros::Time::now();
    ros::Time last_log_time = experiment_start - ros::Duration(1.0);

    while (ros::ok()) {
        ros::spinOnce();

        const ros::Time now = ros::Time::now();
        const double elapsed = (now - experiment_start).toSec();
        int stage = 0;

        if (elapsed < settle_time) {
            stage = 0;
        } else if (elapsed < settle_time + square_motion_time) {
            stage = 1;
        } else if (elapsed < settle_time + square_motion_time + recovery_time) {
            stage = 2;
        } else if (elapsed < settle_time + square_motion_time + recovery_time +
                               landing_publish_time.toSec()) {
            stage = 3;
        } else {
            break;
        }

        if (!square_origin_initialized && g_base_pose.valid) {
            square_origin_x = g_base_pose.x;
            square_origin_y = g_base_pose.y;
            square_origin_initialized = true;
            ROS_INFO("exp3 square origin initialized from current base pose: (%.3f, %.3f)",
                     square_origin_x, square_origin_y);
        }

        uav_pos_d.land_flag = false;
        uav_pos_d.z_d = hover_z;
        uav_pos_d.yaw_d = hover_yaw_deg;
        uav_pos_d.x_d = square_origin_x;
        uav_pos_d.y_d = square_origin_y;
        current_angle.arm1_angle = arm1_offset_deg;
        current_angle.arm2_angle = arm2_offset_deg;
        current_angle.hand_angle = hand_hold_deg;

        if (stage == 1) {
            const double motion_time = elapsed - settle_time;
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

            uav_pos_d.x_d = InterpolateSegment(start_x, end_x, ratio);
            uav_pos_d.y_d = InterpolateSegment(start_y, end_y, ratio);

            current_angle.arm1_angle =
                arm1_offset_deg +
                arm1_sign * arm1_amp_deg *
                    std::sin(two_pi * motion_time / arm1_period + arm1_phase_rad);
            current_angle.arm2_angle =
                arm2_offset_deg +
                arm2_sign * arm2_amp_deg *
                    std::sin(two_pi * motion_time / arm2_period + arm2_phase_rad);
        } else if (stage == 2) {
            uav_pos_d.x_d = square_origin_x;
            uav_pos_d.y_d = square_origin_y;
        } else if (stage == 3) {
            uav_pos_d.x_d = square_origin_x;
            uav_pos_d.y_d = square_origin_y;
            uav_pos_d.z_d = 0.5;
            uav_pos_d.land_flag = true;
        }

        current_angle.arm1_angle = Clamp(current_angle.arm1_angle, arm1_min, arm1_max);
        current_angle.arm2_angle = Clamp(current_angle.arm2_angle, arm2_min, arm2_max);
        current_angle.hand_angle = Clamp(current_angle.hand_angle, hand_min, hand_max);

        if ((now - last_log_time).toSec() >= 1.0) {
            last_log_time = now;
            ROS_INFO(
                "[%s] t=%.2f s, pose_d=(%.2f, %.2f, %.2f, %.2f), arm_d=(%.2f, %.2f, %.2f), square_origin=(%.2f, %.2f), base_valid=%s, land=%s",
                StageName(stage), elapsed, uav_pos_d.x_d, uav_pos_d.y_d, uav_pos_d.z_d,
                uav_pos_d.yaw_d, current_angle.arm1_angle, current_angle.arm2_angle,
                current_angle.hand_angle, square_origin_x, square_origin_y,
                g_base_pose.valid ? "true" : "false", uav_pos_d.land_flag ? "true" : "false");
        }

        uav_pos_d_pub.publish(uav_pos_d);
        joint_angle_pub.publish(current_angle);
        rate.sleep();
    }

    std::cout << "exp3 finished" << std::endl;
    return 0;
}
