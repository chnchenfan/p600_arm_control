#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include "uam_message/arm_pidConfig.h"
#include "uam_message/pid.h"
#include <std_msgs/Float64.h>
#include <cstdlib>
#include <unistd.h>
#include "yaml-cpp/yaml.h"
#include <fstream>
#include <iostream>
// 说明
/* 
这是动态调参机械臂的PID的参数调节
在PID_set.yaml文件中，配置了各个关节的PID参数
yaml这部分作为动态参数调节
pid_set:
   arm_joint1_d: 0.01
这部分是uav_info.yaml中启动及机械臂的参数，后面将这部分从PID_set.yaml复制到uav_info.yaml即可
Pos_PID:
  arm_joint1: {P: 10, I: 0, D: 0.01} 
*/
YAML::Node config = YAML::LoadFile("/home/wjl/p600_arm_control/src/uam_message/config/arm_info_init.yaml");
void load_YAML(){
    std::ofstream fout("/home/wjl/p600_arm_control/src/uam_message/config/arm_info_init.yaml");
    fout << config;
    fout.close();
}

//单个电机信息
class Joint_info
{
public:
    ros::Publisher joint_pid_pub;
    std::string joint_name;
    uam_message::pid current_pid;
    Joint_info(ros::NodeHandle nh,std::string joint_name){
        this->joint_name=joint_name;
        float p1=config["Pos_PID"][joint_name]["P"].as<float>();
        float i1=config["Pos_PID"][joint_name]["I"].as<float>();
        float d1=config["Pos_PID"][joint_name]["D"].as<float>();
        current_pid.p=p1;
        current_pid.i=i1;
        current_pid.d=d1;
        
        std::string model_name;
        nh.getParam("/model_name0", model_name);
        this->joint_pid_pub=nh.advertise<uam_message::pid>("/"+model_name+"/"+joint_name+"/pid",10);
    }
    void Pub(){
        joint_pid_pub.publish(current_pid);
    }
    void yaml_save(){
        config["Pos_PID"][joint_name]["P"]=current_pid.p;
        config["Pos_PID"][joint_name]["I"]=current_pid.i;
        config["Pos_PID"][joint_name]["D"]=current_pid.d;
        std::string p=joint_name+"_p";
        std::string i=joint_name+"_i";
        std::string d=joint_name+"_d";
        config["pid_set"][p]=current_pid.p;
        config["pid_set"][i]=current_pid.i;
        config["pid_set"][d]=current_pid.d;
    }
};
class All_joints{
public:
    Joint_info arm_joint1;
    Joint_info arm_joint2;
    Joint_info hand_left_joint;
    uam_message::arm_pidConfig config;
    ros::NodeHandle nh;
    All_joints(ros::NodeHandle nh):arm_joint1(nh,"arm_joint1"),arm_joint2(nh,"arm_joint2"),hand_left_joint(nh,"left_hand_joint")
    {
        this->nh=nh;
    }
    void Pub(){
        
        arm_joint1.current_pid.p=config.arm_joint1_p;
        arm_joint1.current_pid.i=config.arm_joint1_i;
        arm_joint1.current_pid.d=config.arm_joint1_d;
        arm_joint2.current_pid.p=config.arm_joint2_p;
        arm_joint2.current_pid.i=config.arm_joint2_i;
        arm_joint2.current_pid.d=config.arm_joint2_d;
        hand_left_joint.current_pid.p=config.hand_left_joint_p;
        hand_left_joint.current_pid.i=config.hand_left_joint_i;
        hand_left_joint.current_pid.d=config.hand_left_joint_d;
        //保证话题发布成功
        for(int i=0;i<10;i++){
            arm_joint1.Pub();
            arm_joint2.Pub();
            hand_left_joint.Pub();
        }
        // 实现功能当land_pid_param等于0时保存参数到yaml文件中
        // 但是有个bug，这里保存的yaml文件信息是上一次的参数，所以调整好后再修改下land_pid_param值
        if(config.land_pid_param==0){
            arm_joint1.yaml_save();
            arm_joint2.yaml_save();
            hand_left_joint.yaml_save();
            load_YAML();
            std::cout<<"保存成功"<<std::endl;
        }
    }
};

void callback(uam_message::arm_pidConfig &config, uint32_t level,All_joints& All_joints) {
    All_joints.config=config;
    All_joints.Pub();
}
 
int main(int argc,char* argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"pid_set");
    ros::NodeHandle nh;
    All_joints all_joints(nh);
    dynamic_reconfigure::Server<uam_message::arm_pidConfig> server;
    dynamic_reconfigure::Server<uam_message::arm_pidConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2,boost::ref(all_joints));
    server.setCallback(f);
    ROS_INFO("机械臂PID调节开始");
    ros::spin();
}

