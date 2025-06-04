
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <thread>
#include "Servos/Motor_s.h"
//通过这个函数判断yaml是否加载到参数服务器中
void Judge_param_load(ros::NodeHandle nh);
int main(int agrc,char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(agrc,argv,"motors_simulation");
    ros::NodeHandle nh;
    Judge_param_load(nh);
    Motors_s motors_(nh);
     ros::Rate r(30);
    while(ros::ok()){
        r.sleep();
        ros::spinOnce();
    }
}


void Judge_param_load(ros::NodeHandle nh){
    while (!nh.hasParam("/load_param_arm_flag"))
  	{
    	ROS_INFO("等待参数加载...");
    	ros::Duration(1.0).sleep();
  	}
    ROS_INFO("加载成功");
}


