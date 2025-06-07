#include "Servos/Motor_s.h"
#include <math.h>
Motor_s::Motor_s(std::string joint_name,ros::NodeHandle nh){
    this->joint_name=joint_name;
    std::string model_name;
    nh.getParam("/model_name0", model_name);
    nh.getParam("/arm/"+joint_name+"/min", pos_angle_min);//获取角度
    nh.getParam("/arm/"+joint_name+"/max", pos_angle_max);
    // std::cout<<"joint_name: "<<joint_name<<"  pos_angle_min: "<<pos_angle_min<<"  pos_angle_max: "<<pos_angle_max<<std::endl;
    this->pos_gazebo_pub=nh.advertise<std_msgs::Float64>("/"+model_name+"/"+joint_name+"/pos_cmd",10);  
}
Motor_s::~Motor_s(){

}
void Motor_s::limit_check(){
    if(pos_angle_s>pos_angle_max) pos_angle_s=pos_angle_max;
    if(pos_angle_s<pos_angle_min) pos_angle_s=pos_angle_min;
}
/**
 * @brief 发布期望角度到gazebo仿真中
 */
void Motor_s::Pos_gazebo_pub(){
    std_msgs::Float64 pos;
    pos.data=pos_angle_s*M_PI/180;
    pos_gazebo_pub.publish(pos);
}


Motors_s::Motors_s(ros::NodeHandle nh):arm1("arm_joint1",nh),arm2("arm_joint2",nh),left_hand("left_hand_joint",nh)
{
    pos_angle_sub=nh.subscribe<uam_message::arm_angle>("/wjl/arm/guidefly/angle_d",10,&Motors_s::Pos_target_cb,this);
    pos_real_pub=nh.advertise<uam_message::arm_angle>("/wjl/arm/real/angle_d",10);
}
Motors_s::~Motors_s(){
    
}

void Motors_s::Pos_target_cb(const boost::shared_ptr<const uam_message::arm_angle>& msg){
    arm1.pos_angle_s=msg->arm1_angle;
    arm2.pos_angle_s=msg->arm2_angle;
    left_hand.pos_angle_s=msg->hand_angle;
    arm1.limit_check();
    arm2.limit_check();
    left_hand.limit_check();
    pos_real_pub.publish(msg);
    arm1.Pos_gazebo_pub();
    arm2.Pos_gazebo_pub();
    left_hand.Pos_gazebo_pub();
}

