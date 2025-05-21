
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "linux_serial/linux_serial.h"
//通过这个函数判断yaml是否加载到参数服务器中
void Judge_param_load(ros::NodeHandle nh){
    std::cout<<nh.hasParam("/load_param_flag")<<std::endl;
    while (!nh.hasParam("/load_param_flag"))
  	{
    	ROS_INFO("等待参数加载...");
    	ros::Duration(1.0).sleep();
  	}
    ROS_INFO("加载成功");
}
//逻辑
//初始化串口以及舵机类
//当接收到话题信息，将数字转为字符，然后通过串口发送
int main(int agrc,char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(agrc,argv,"arm_node");
    ros::NodeHandle nh;
    Judge_param_load(nh);
    ROS_INFO("linux_serial start....");
    std::string usb_name;
    nh.getParam("arm_usb", usb_name);
    Linux_serial _serial(nh,usb_name);
    ros::Rate r(20);
    while(ros::ok()){
        if(_serial.sub_data.updata_flag){
            ROS_INFO("串口发送成功");
            _serial.Send_data();
            _serial.sub_data.updata_flag=false;
        }
        r.sleep();
        ros::spinOnce();
    }
}
 



