#ifndef _POSITION_CONTROLLER_HH_
#define _POSITION_CONTROLLER_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <std_msgs/Float64.h> 
#include "uam_message/pid.h"
namespace gazebo
{

class PositionPlugin : public ModelPlugin
{

public:
    PositionPlugin() {}
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        ros::NodeHandle nh;
        // 存储关节信息
        this->model = _model;
        // 安全检查
        if (_model->GetJointCount() == 0)
        {
            std::cout << "load faided,find jointcout is 0!\n";
            return;
        }else{
            std::cout<<"load successed!\n";
        }
        // 默认值
        double position = 0;
        if (_sdf->HasElement("jointname")){
            joint_name = _sdf->Get<std::string>("jointname");//获取sdf元素并赋值到
            joint_art_name = _model->GetScopedName() +"::" + joint_name;//这里的关节名是指的是，模型名::关节名，后面发送位置控制器中
        }
        if (_sdf->HasElement("Pid_sub")){
            pid_sub_flag = _sdf->Get<bool>("Pid_sub");
        }
        std::string param_name="/Pos_PID/"+joint_name;
        //载入pid参数
        while (!nh.hasParam(param_name))
  	    {
    	    ROS_INFO("param waiting...");
    	    ros::Duration(1.0).sleep();
  	    }
        nh.getParam(param_name+"/P", this->p_gain);
        nh.getParam(param_name+"/I", this->i_gain);
        nh.getParam(param_name+"/D", this->d_gain);
        std::cout<<joint_name<<"pid:"<<p_gain<<" "<<i_gain<<" "<<d_gain<<" "<<std::endl;
        this->pid = common::PID(p_gain, i_gain, d_gain);
        pos_target_pub=nh.advertise<std_msgs::Float64>("/"+_model->GetScopedName()+"/"+joint_name+"/pos_target_controller",10); 


        // 给关节申请控制器
        this->model->GetJointController()->SetPositionPID(joint_art_name, this->pid);

        // 设置期望位置
        this->model->GetJointController()->SetPositionTarget(joint_art_name, position);
        
        timer_=nh.createTimer(ros::Duration(0.005), &PositionPlugin::TimerCallback, this);

        // 如果它尚未被初始化，初始化ROS
        if (!ros::isInitialized())
        {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, joint_name + "_" + "node",ros::init_options::NoSigintHandler);
        }

        // 创建ros节点
        this->rosNode.reset(new ros::NodeHandle(joint_name + "_" + "Handle"));

        // 创建话题并订阅位置信息
        ros::SubscribeOptions so =ros::SubscribeOptions::create<std_msgs::Float64>(
            "/" + this->model->GetName() + "/" + joint_name + "/pos_cmd",1,
            boost::bind(&PositionPlugin::OnRosMsg, this, _1),ros::VoidPtr(), &this->rosQueue);
        this->rosSub = this->rosNode->subscribe(so);

        if(pid_sub_flag){
            //创建对pid话题并订阅
            ros::SubscribeOptions so_pid =ros::SubscribeOptions::create<uam_message::pid>(
                "/" + this->model->GetName() + "/" + joint_name + "/pid",1,
                boost::bind(&PositionPlugin::OnRosMsg_pid, this, _1),ros::VoidPtr(), &this->rosQueue);
            this->ros_pid_sub = this->rosNode->subscribe(so_pid);
        }

        // 启动一个队列辅助线程
        this->rosQueueThread =std::thread(std::bind(&PositionPlugin::QueueThread, this));

    }   

private: 
    // 处理来自ROS的传入消息，_msg 一个用于设置位置的浮点值

    void OnRosMsg(const std_msgs::Float64ConstPtr &msg)
    {
        receive_data=msg->data;
    }


    void OnRosMsg_pid(const boost::shared_ptr<const uam_message::pid> &msg)
    {
        this->pid=common::PID(msg->p, msg->i,msg->d);
        this->model->GetJointController()->SetPositionPID(joint_art_name, this->pid);
    }

    void TimerCallback(const ros::TimerEvent& event)
    {
        // 在定时器触发时执行的操作
        Inc_pub_loc();
    }
    void QueueThread()
    {
        static const double timeout = 0.01;
        while (this->rosNode->ok())
        {
            this->rosQueue.callAvailable(ros::WallDuration(timeout));
        }
    }
    //和定时器组合，200hz,1hz发送或者减少0.001
    void Inc_pub_loc(){
        const double increment = 0.001; 
        // std::cout<<"re:"<<receive_data<<" "<<"curr:"<<current_pos<<std::endl;
        if(std::abs(receive_data - current_pos) < 1e-10){}
        else if(receive_data<current_pos){
            if(current_pos-receive_data>increment){
                current_pos-=increment;
            }else{
                //作为增量小于increment的处理
                current_pos-=(current_pos-receive_data);
            }
            this->model->GetJointController()->SetPositionTarget(joint_art_name,current_pos);
            // std::cout<<"ttt:"<<std::endl;
        }else{
            if(receive_data-current_pos>increment){
                current_pos+=increment;
            }else{
                //作为增量小于increment的处理
                current_pos+=(receive_data-current_pos);
            }
            this->model->GetJointController()->SetPositionTarget(joint_art_name,current_pos);
            // std::cout<<"uuu:"<<std::endl;
        }
        // std::cout<<"joint_name:"<<joint_name<<"  current_pos:"<<current_pos<<std::endl;
        cur_pos_target_.data=current_pos;
        pos_target_pub.publish(cur_pos_target_);
    }

/// \brief Pointer to the model.
private: 
    physics::ModelPtr model;
    common::PID pid;
    std::unique_ptr<ros::NodeHandle> rosNode;
    ros::Subscriber rosSub;//订阅位置
    ros::Subscriber ros_pid_sub;//
    ros::Publisher pos_target_pub;//发布设置控制器的位置信息
    std_msgs::Float64 cur_pos_target_;
    ros::CallbackQueue rosQueue;
    std::thread rosQueueThread;
    double p_gain=1, i_gain=0, d_gain=0;
    double receive_data=0;
    double current_pos=0;
    ros::Timer timer_;//定时器
    std::string joint_art_name;//模型名::关节名
    std::string joint_name;//关节名
    std::string ScopedName="ScopedName";//模型名
    bool pid_sub_flag=false;
};

// 告诉gazebo这个插件,让gazebo加载注册这个插件
GZ_REGISTER_MODEL_PLUGIN(PositionPlugin)
}
#endif