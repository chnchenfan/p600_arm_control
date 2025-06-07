#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <uam_message/uam_cmdConfig.h>
#include "uam_message/arm_angle.h"//多个电机信息
#include "std_msgs/Float64.h"
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <geometry_msgs/Point.h>
#include "uav/xyz_yaw_d.h"
#include "uav/desired_start.h"
bool start_flag=false;
bool doReq(uav::desired_start::Request& req,uav::desired_start::Response& resp){
    int desired_start= req.desired_start;
    std::cout<<"收到期望数据："<<desired_start<<std::endl;
    //逻辑处理
    if (desired_start!=10)
    {
        ROS_ERROR("提交的数据异常!!!");
        return false;
    }
    //如果没有异常，那么相加并将结果赋值给 resp
    resp.desired_sent=3;
    start_flag=true;
    return true;
}

int main(int argc,char* argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"uam_desired");
    ros::NodeHandle nh;
    ros::Publisher uav_pos_d_pub;
    uav::xyz_yaw_d uav_pos_d;
    uam_message::arm_angle current_angle;
    ros::Publisher joint_angle_pub;
    joint_angle_pub=nh.advertise<uam_message::arm_angle>("/wjl/arm/guidefly/angle_d",10);
    uav_pos_d_pub=nh.advertise<uav::xyz_yaw_d>("/wjl/guidefly/pose_d",10);
    ros::ServiceServer server = nh.advertiseService("/wjl/start/uav_desired",doReq);
    ROS_INFO("服务已经启动....");
    ros::Rate rate(30);
    while(!start_flag){
        ros::spinOnce();
        rate.sleep();
    }
    std::cout<<"开始发送期望数据："<<std::endl;
    uav_pos_d.land_flag=false;
    double x=0,y=0,z=0.35,yaw_d=0;
    ros::Time t_old = ros::Time::now();
    current_angle.arm1_angle=0;
    current_angle.arm2_angle=0;
    current_angle.hand_angle=0;
    int num=0;
    int muti=3;
    while(ros::Time::now()-t_old<ros::Duration(20*muti)){
        ros::Time t_new = ros::Time::now();
        double t=(t_new-t_old).toSec();
        if(t>1*muti){
            x=0.2;
        } 
        if(t<6*muti){
            y=0;
        }else if(t>6*muti && t<9*muti){
            y=1*sin(M_PI*(t/muti-6)/6);
        }else{
            y=1;
        }
        if(t<2*muti){
            z=0.35+0.65*sin(M_PI*t/muti/4);
        }else{
            z=1;
        }
        if(t<3*muti){
            yaw_d=0;
        }else if(t<5*muti){
            yaw_d=sin((t/muti-3)*M_PI/4)*90;
        }else{
            yaw_d=90;
        }
        if(t>10*muti && t<13*muti){
            current_angle.arm1_angle=30;
        }else if(t>13*muti && t<20*muti){
            current_angle.arm2_angle=-30;
        }
        rate.sleep();
        ros::spinOnce(); 
        num++;
        if(num>30){
            num=0;
            std::cout<<"时间t:"<<t<<"s,x:"<<x<<" y:"<<y<<" z:"<<z<<",yaw:"<<yaw_d<<std::endl;
            std::cout<<"arm1:"<<current_angle.arm1_angle<<" arm2:"<<current_angle.arm2_angle<<std::endl;
        }
        uav_pos_d.x_d=x;
        uav_pos_d.y_d=y;
        uav_pos_d.z_d=z;
        uav_pos_d.yaw_d=yaw_d;
        uav_pos_d_pub.publish(uav_pos_d);
        joint_angle_pub.publish(current_angle);
    }
    uav_pos_d.z_d=0.5;
    uav_pos_d.land_flag=true;
    current_angle.arm1_angle=0;
    current_angle.arm2_angle=0;
    t_old = ros::Time::now();
    while(ros::Time::now()-t_old<ros::Duration(0.5)){
        uav_pos_d_pub.publish(uav_pos_d);
        joint_angle_pub.publish(current_angle);
        rate.sleep();
    }
    std::cout<<"结束飞行！！！"<<std::endl;
}
