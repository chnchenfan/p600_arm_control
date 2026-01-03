#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include "uam_message/arm_angle.h"
//头文件说明
uam_message::arm_angle arm_angle_d;
double temp;
void xbox_cb(const boost::shared_ptr<const sensor_msgs::Joy>& msg)
{
    // 将接收到的消息打印出来
    // std::cout<<"订阅的消息话题为:"<<*msg<<std::endl;
    arm_angle_d.arm1_angle=-msg->axes[0]*180/2;
    temp=msg->axes[1]*180/2;
    if(temp>=20){
        arm_angle_d.arm2_angle=20;
    }else if(temp<=-90){
        arm_angle_d.arm2_angle=-90;
    }else{
        arm_angle_d.arm2_angle=temp;
    }
    double t=msg->axes[3]*180/2;
    if(t<=-15){
        arm_angle_d.hand_angle=-15;
    }else if(t>=25){
        arm_angle_d.hand_angle=25;
    }else{
        arm_angle_d.hand_angle=t;
    }

}
int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");

    //1.初始化 ROS 节点
    ros::init(argc,argv,"xbox_control");
    ros::NodeHandle nh;
    arm_angle_d.arm1_angle=0;
    arm_angle_d.arm2_angle=0;
    arm_angle_d.hand_angle=0;
    ros::Subscriber xbox_sub = nh.subscribe<sensor_msgs::Joy>("/joy",10,xbox_cb);
    ros::Publisher joint_angle_pub = nh.advertise<uam_message::arm_angle>("/wjl/arm/guidefly/angle_d", 10);
    ros::Rate r(30);
    while (ros::ok())
    {
        joint_angle_pub.publish(arm_angle_d);
        r.sleep();
        ros::spinOnce();
        // std::cout<<"arm1_angle:"<<arm_angle_d.arm1_angle<<std::endl;
        // std::cout<<"arm2_angle:"<<arm_angle_d.arm2_angle<<std::endl;
        // std::cout<<"hand_angle:"<<arm_angle_d.hand_angle<<std::endl;
    }
    return 0;
}