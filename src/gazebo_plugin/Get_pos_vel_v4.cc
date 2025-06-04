#ifndef _GET_POS_VEL_HH_
#define _GET_POS_VEL_HH_
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <string>
#include <cmath>
#include "std_msgs/Float64.h" 
#include <geometry_msgs/PoseStamped.h>
#include "uam_message/uam_v4_pose.h"
namespace gazebo {
class Joint_info
{
public:
    std::string joint_name;//关节名
    std::string link_name;//link名
    physics::JointPtr joint;
    Joint_info(){}
    void Init_info(std::string link_name,ros::NodeHandle nh,physics::ModelPtr model){
        this->link_name=link_name;
        // if(link_name=="arm_link1"){
        //     nh.getParam("arm_joint1", this->joint_name);
        // }else if(link_name=="arm_link2"){
        //     nh.getParam("arm_joint2", this->joint_name);
        // }else if(link_name=="left_hand_link"){
        //     nh.getParam("left_hand_joint", this->joint_name);
        // }else if(link_name=="right_hand_link"){
        //     nh.getParam("right_hand_joint", this->joint_name);
        // }
        nh.getParam(this->link_name+"/joint", this->joint_name);
        this->joint=model->GetJoint(this->joint_name);//获取关节

        if (this->joint) {
            ROS_INFO_STREAM("Joint " << this->joint_name << " successfully initialized.");
        } else {
            ROS_ERROR_STREAM("Failed to initialize joint " << this->joint_name << ".");
        }
    }
};
class Uav_arm
{
public:
    Joint_info arm1;
    Joint_info arm2;
    Joint_info left_hand_link;
    Joint_info right_hand_link;
    //机体位置存储
    ignition::math::Pose3d pose;
    ros::Publisher uar_arm_pos_pub;
    Uav_arm(){}
    void Init_info(ros::NodeHandle nh,physics::ModelPtr model){
        uar_arm_pos_pub = nh.advertise<uam_message::uam_v4_pose>("/"+model->GetScopedName()+"/uav_arm_tf",10);
        arm1.Init_info("arm_link1",nh,model);
        arm2.Init_info("arm_link2",nh,model);
        left_hand_link.Init_info("left_hand_link",nh,model);
        right_hand_link.Init_info("right_hand_link",nh,model);
    }
    void uam_v4_pose(physics::ModelPtr model){
        this->pose=model->WorldPose();
        uam_message::uam_v4_pose pose_msg;
        this->pose=model->WorldPose();
        pose_msg.x=pose.Pos().X();
        pose_msg.y=pose.Pos().Y();
        pose_msg.z=pose.Pos().Z();
        pose_msg.qx=pose.Rot().X();
        pose_msg.qy=pose.Rot().Y();
        pose_msg.qz=pose.Rot().Z();
        pose_msg.qw=pose.Rot().W();
        pose_msg.arm1_angle=arm1.joint->Position(0);
        pose_msg.arm2_angle=arm2.joint->Position(0);
        pose_msg.left_hand_angle=left_hand_link.joint->Position(0);
        pose_msg.right_hand_angle=right_hand_link.joint->Position(0);
        uar_arm_pos_pub.publish(pose_msg);
        // 发布机器人的位姿信息
    }
};
class Get_pos_vel : public ModelPlugin {
public:
    Get_pos_vel() {}
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) override {
        // 获取机器人模型
        ros::NodeHandle nh;
        this->model = _parent;
        std::string modelName = this->model->GetName();
        ROS_INFO_STREAM("Model Name: " << modelName);
        //载入pid参数
        while (!nh.hasParam("/load_param_flag"))
  	    {
    	    ROS_INFO("等待参数加载...");
    	    ros::Duration(1.0).sleep();
  	    }

        //this->model->GetScopedName()相当于模型名字，也可以作为launch的组名
        this->uav_arm.Init_info(nh,this->model);
        // 创建一个Gazebo回调函数，用于在每个仿真步骤中获取机器人的位姿信息
        update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
            boost::bind(&Get_pos_vel::OnUpdate, this));
    }  
private:
    void OnUpdate() {
        // this->uav_arm.Pos_pub(this->model
        this->uav_arm.uam_v4_pose(this->model);
    }
private:
  // 机器人模型
    physics::ModelPtr model;
    gazebo::event::ConnectionPtr update_connection_;
    Uav_arm uav_arm;
};
GZ_REGISTER_MODEL_PLUGIN(Get_pos_vel)
} 
#endif