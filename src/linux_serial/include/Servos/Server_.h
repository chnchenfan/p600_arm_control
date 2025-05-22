#pragma once
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include "Servos/Struct_.h"

class Server_{
public:
    State_pkg state_pkg;//电机状态包
    uint8_t send_pack[20];//发送数据包
    int send_len;//发送长度
    Server_(ros::NodeHandle nh,uint8_t addr);
    ~Server_();
    void Pos_Control(); 
    void Read_Sys_Params();
    void State_show(); 
private:
    double pos_angle_r;//实际的电机角度
    double pos_angle_s;//期望的电机角度
    double pos_angle_t;//电机目标角度
    double vel_r;//电机转速
    double pos_error;//电机位置误差
};

class Servers_{
public:
    Server_ server1;
    Server_ server2;
    Server_ server3;
    Server_ server4;//作为抓手控制
    Servers_(ros::NodeHandle nh);
    ~Servers_();
private:
};

