
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <thread>
#include "linux_serial/linux_serial.h"
//通过这个函数判断yaml是否加载到参数服务器中
void Judge_param_load(ros::NodeHandle nh);
int main(int agrc,char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(agrc,argv,"arm_node");
    ros::NodeHandle nh;
    Judge_param_load(nh);
    ROS_INFO("linux_serial start....");
    std::string usb_name;
    nh.getParam("arm_usb", usb_name);
    double serial_loop_hz = 15.0;
    nh.param("arm_serial_loop_hz", serial_loop_hz, serial_loop_hz);
    if (serial_loop_hz <= 0.0) {
        serial_loop_hz = 15.0;
    }
    Linux_serial _serial(nh,usb_name);
    ros::Rate r(serial_loop_hz);
    while(ros::ok()){
        r.sleep();
        _serial.Send_all_data();
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

