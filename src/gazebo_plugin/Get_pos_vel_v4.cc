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
    Joint_info arm3;
    Joint_info arm4;
    Joint_info arm5;
    Joint_info hand_left_link1;
    Joint_info hand_left_link2;
    Joint_info hand_left_link3;
    Joint_info hand_right_link1;
    Joint_info hand_right_link2;
    Joint_info hand_right_link3;
    //机体位置存储
    ignition::math::Pose3d pose;
    ros::Publisher uar_arm_pos_pub;
    Uav_arm(){}
    void Init_info(ros::NodeHandle nh,physics::ModelPtr model){
        uar_arm_pos_pub = nh.advertise<uam_message::uam_v4_pose>("/"+model->GetScopedName()+"/uav_arm_tf",10);
        arm1.Init_info("arm_link1",nh,model);
        arm2.Init_info("arm_link2",nh,model);
        arm3.Init_info("arm_link3",nh,model);
        arm4.Init_info("arm_link4",nh,model);
        arm5.Init_info("arm_link5",nh,model);
        hand_left_link1.Init_info("hand_left_link1",nh,model);
        hand_left_link2.Init_info("hand_left_link2",nh,model);
        hand_left_link3.Init_info("hand_left_link3",nh,model);
        hand_right_link1.Init_info("hand_right_link1",nh,model);
        hand_right_link2.Init_info("hand_right_link2",nh,model);
        hand_right_link3.Init_info("hand_right_link3",nh,model);
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
        pose_msg.arm3_angle=arm3.joint->Position(0);
        pose_msg.arm4_angle=arm4.joint->Position(0);
        pose_msg.arm5_angle=arm5.joint->Position(0);
        pose_msg.hand_l_joint1_angle=hand_left_link1.joint->Position(0);
        pose_msg.hand_l_joint2_angle=hand_left_link2.joint->Position(0);
        pose_msg.hand_l_joint3_angle=hand_left_link3.joint->Position(0);
        pose_msg.hand_r_joint1_angle=hand_right_link1.joint->Position(0);
        pose_msg.hand_r_joint2_angle=hand_right_link2.joint->Position(0);
        pose_msg.hand_r_joint3_angle=hand_right_link3.joint->Position(0);
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