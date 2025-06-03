#pragma once
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include "Servos/Struct_.h"
#include "linux_serial/arm_angle.h"
class Server_{
public:
    double pos_angle_r;//实际的电机角度
    State_pkg state_pkg;//电机状态包
    uint8_t send_pack[20];//发送数据包
    int send_len;//发送长度
    Server_(ros::NodeHandle nh,uint8_t addr);
    ~Server_();
    void Pos_Control(); 
    void Read_Sys_Params();
    void State_show(); 
private:
    double pos_angle_s;//期望的电机角度
    double pos_angle_t;//电机目标角度,驱动器中获取
    double vel_r;//电机转速
    double pos_error;//电机位置误差
    double pos_angle_min;//电机最小角度
    double pos_angle_max;//电机最大角度
    ros::Publisher pub1;//发布电机状态
    ros::Subscriber pos_angle_sub;//订阅电机目标角度
    int multiple_reducer;//减速比,1到3号电机力矩放大48倍
    int direction_rotation;//电机旋转方向,1为正转，-1为反转,加了减速器与电机旋转方向实际相反
    void Pos_target_cb(const boost::shared_ptr<const std_msgs::Float64>& msg);
};

class Servers_{
public:
    Server_ server1;
    Server_ server2;
    Server_ server3;//作为抓手控制
    Servers_(ros::NodeHandle nh);
    ~Servers_();
    void Arm_angle_pub();
private:
    ros::Publisher arm_angle_pub;//发布电机角度
    linux_serial::arm_angle arm_angle;
};

