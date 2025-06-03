#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <uam_message/Angle_setConfig.h>
#include "std_msgs/Float64.h"
#include <iostream>
#include <cstdlib>
#include <unistd.h>
//单个电机信息
class Joint_info
{
public:
    ros::Publisher joint_angle_pub;
    std::string joint_name;
    std_msgs::Float64 current_angle;
    Joint_info(ros::NodeHandle nh,std::string joint_name){
        this->joint_name=joint_name;
        this->joint_angle_pub=nh.advertise<std_msgs::Float64>("/wjl/arm/"+joint_name+"/pos_target",10);    
    }
    void Pub(){
        joint_angle_pub.publish(current_angle);
    }
};
class All_joints{
public:
    Joint_info joint1;
    Joint_info joint2;
    Joint_info joint3;
    uam_message::Angle_setConfig config;
    ros::NodeHandle nh;
    All_joints(ros::NodeHandle nh):joint1(nh,"joint1"),joint2(nh,"joint2"),joint3(nh,"joint3")
    {
        this->nh=nh;
    }
    void Pub(){
        joint1.current_angle.data=config.joint1_angle;
        joint2.current_angle.data=config.joint2_angle;
        joint3.current_angle.data=config.joint3_angle;
        //保证话题发布成功
        ros::Rate rate(50);
        int count=0;
        while(ros::ok()){
            joint1.Pub();
            joint2.Pub();
            joint3.Pub();
            rate.sleep();
            ros::spinOnce();
            if(count>50){
                break;
            }
            count++;
        }
    }
};

void callback(uam_message::Angle_setConfig &config, uint32_t level,All_joints& All_joints) {
    All_joints.config=config;
    All_joints.Pub();
}
 
int main(int argc,char* argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"dynamic_angle");
    ros::NodeHandle nh;
    All_joints all_joints(nh);
    dynamic_reconfigure::Server<uam_message::Angle_setConfig> server;
    dynamic_reconfigure::Server<uam_message::Angle_setConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2,boost::ref(all_joints));
    server.setCallback(f);
    ROS_INFO("dynamic_angle is ready");
    ros::spin();
}