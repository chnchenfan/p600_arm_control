#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <uam_message/uam_cmdConfig.h>
#include "uam_message/arm_angle.h"//多个电机信息
#include "std_msgs/Float64.h"
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include "uav/xyz_yaw_d.h"
class All_joints{
public:
    uam_message::uam_cmdConfig config;
    ros::NodeHandle nh;
    uam_message::arm_angle current_angle;
    ros::Publisher joint_angle_pub;
    ros::Publisher uav_pos_d_pub;
    uav::xyz_yaw_d uav_pos_d;
    All_joints(ros::NodeHandle nh)
    {
        this->nh=nh;
        this->joint_angle_pub=nh.advertise<uam_message::arm_angle>("/wjl/arm/pos_target",10);
        this->uav_pos_d_pub=nh.advertise<uav::xyz_yaw_d>("/wjl/guidefly/pose_d",10);
    }
    void Pub(){
        uav_pos_d.x_d=config.uav_x_d;
        uav_pos_d.y_d=config.uav_y_d;
        uav_pos_d.z_d=config.uav_z_d;
        uav_pos_d.yaw_d=config.uav_yaw_d;
        uav_pos_d.land_flag=config.land_flag;
        current_angle.arm1_angle=config.arm_joint1_cmd;
        current_angle.arm2_angle=config.arm_joint2_cmd;
        current_angle.hand_angle=config.left_hand_joint_cmd;
        //保证话题发布成功
        ros::Rate rate(60);
        int count=0;
        while(ros::ok()){
            joint_angle_pub.publish(current_angle);
            uav_pos_d_pub.publish(uav_pos_d);
            rate.sleep();
            ros::spinOnce();
            if(count>20){
                break;
            }
            count++;
        }
    }
};

void callback(uam_message::uam_cmdConfig &config, uint32_t level,All_joints& All_joints) {
    All_joints.config=config;
    All_joints.Pub();
}
 
int main(int argc,char* argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"dynamic_angle");
    ros::NodeHandle nh;
    All_joints all_joints(nh);
    dynamic_reconfigure::Server<uam_message::uam_cmdConfig> server;
    dynamic_reconfigure::Server<uam_message::uam_cmdConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2,boost::ref(all_joints));
    server.setCallback(f);
    ROS_INFO("dynamic_angle is ready");
    ros::spin();
}