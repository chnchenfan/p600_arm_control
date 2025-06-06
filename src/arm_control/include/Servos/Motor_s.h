#pragma once
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include "uam_message/arm_angle.h"
class Motor_s{
public:
    double pos_angle_s;//期望角度
    Motor_s(std::string joint_name,ros::NodeHandle nh);
    ~Motor_s();
    void Pos_gazebo_pub();
    void limit_check();//限制检查
private:
    uint8_t addr;//电机地址
    double pos_angle_min;
    double pos_angle_max;
    std::string joint_name;//关节名
    ros::Publisher pos_gazebo_pub;//仿真期望角度发布者
};
//接受来自飞机或者动态调参的电机角度，发布到仿真期望角度并将实际角度发送给server类
class Motors_s{
public:
    Motor_s arm1;
    Motor_s arm2;
    Motor_s left_hand;//作为抓手控制
    Motors_s(ros::NodeHandle nh);
    ~Motors_s();
private:
    ros::Subscriber pos_angle_sub;//订阅电机目标角度
    ros::Publisher pos_real_pub;//实际，发送到server类
    void Pos_target_cb(const boost::shared_ptr<const uam_message::arm_angle>& msg);

};

