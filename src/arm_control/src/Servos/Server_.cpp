#include "Servos/Server_.h"
Server_::Server_(ros::NodeHandle nh,uint8_t addr){
    state_pkg.addr_ = addr;
    if(state_pkg.addr_==3){
        multiple_reducer=40;
        direction_rotation=1;
    }else{
        multiple_reducer=50;
        direction_rotation=1;
    }
}
Server_::~Server_(){

}

/**
  * @brief    默认读取系统参数
*/
void Server_::Read_Sys_Params(){
    SysParams_t s;
    s=SysParams_t::S_State;//默认读取系统参数
    int i=0;
    // 装载命令
    send_pack[i] = state_pkg.addr_; ++i;                   // 地址
    switch(s)                             // 功能码
    {
        case S_VER  : send_pack[i] = 0x1F; ++i; break;
        case S_RL   : send_pack[i] = 0x20; ++i; break;
        case S_PID  : send_pack[i] = 0x21; ++i; break;
        case S_VBUS : send_pack[i] = 0x24; ++i; break;
        case S_CPHA : send_pack[i] = 0x27; ++i; break;
        case S_ENCL : send_pack[i] = 0x31; ++i; break;
        case S_TPOS : send_pack[i] = 0x33; ++i; break;
        case S_VEL  : send_pack[i] = 0x35; ++i; break;
        case S_CPOS : send_pack[i] = 0x36; ++i; break;
        case S_PERR : send_pack[i] = 0x37; ++i; break;
        case S_FLAG : send_pack[i] = 0x3A; ++i; break;
        case S_ORG  : send_pack[i] = 0x3B; ++i; break;
        case S_Conf : send_pack[i] = 0x42; ++i; send_pack[i] = 0x6C; ++i; break;
        case S_State: send_pack[i] = 0x43; ++i; send_pack[i] = 0x7A; ++i; break;
        default: break;
    }
    send_pack[i] = 0x6B; ++i;                   // 校验字节
    send_len = i;
}
/**
  * @brief    位置模式，发送位置信息
  * @param    addr：电机地址
  * @param    dir ：方向        ，0为CW，其余值为CCW
  * @param    vel ：速度(RPM)   ，范围0 - 5000RPM
  * @param    acc ：加速度      ，范围0 - 255，注意：0是直接启动
  * @param    clk ：脉冲数      ，范围0- (2^32 - 1)个
  * @param    raF ：相位/绝对标志，false为相对运动，true为绝对值运动
  * @param    snF ：多机同步标志 ，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Server_::Pos_Control(){
    uint16_t vel=100;
    uint8_t acc=0;
    uint8_t dir=0;
    uint32_t clk=0;
    double send_pos;
    send_pos=direction_rotation*pos_angle_s;
    // 角度转脉冲数
    if(send_pos>=0){
        dir=0;
        clk=80*send_pos/9*multiple_reducer;//360度对应3200脉冲，用了减速器，电机转动48圈，减去器带动机械臂的转动1圈
    }else{
        dir=1;
        clk=-80*send_pos/9*multiple_reducer;
    }
    // 装载命令
    send_pack[0]  =  state_pkg.addr_;                      // 地址
    send_pack[1]  =  0xFD;                      // 功能码
    send_pack[2]  =  dir;                       // 方向
    send_pack[3]  =  (uint8_t)(vel >> 8);       // 速度(RPM)高8位字节
    send_pack[4]  =  (uint8_t)(vel >> 0);       // 速度(RPM)低8位字节 
    send_pack[5]  =  acc;                       // 加速度，注意：0是直接启动
    send_pack[6]  =  (uint8_t)(clk >> 24);      // 脉冲数(bit24 - bit31)
    send_pack[7]  =  (uint8_t)(clk >> 16);      // 脉冲数(bit16 - bit23)
    send_pack[8]  =  (uint8_t)(clk >> 8);       // 脉冲数(bit8  - bit15)
    send_pack[9]  =  (uint8_t)(clk >> 0);       // 脉冲数(bit0  - bit7 )
    send_pack[10] =  true;                       // 相位/绝对标志，false为相对运动，true为绝对值运动
    send_pack[11] =  false;                       // 多机同步运动标志，false为不启用，true为启用
    send_pack[12] =  0x6B;                      // 校验字节
    send_len= 13;
}

/**
 * @brief 显示电机状态
 *
 * 此函数用于根据电机状态包中的数据计算并显示电机的状态信息。
 *
 * @param 无
 *
 * @return 无
 */
void Server_::State_show(){
    //得到的是步进电机的状态，所以要先将脉冲数转换成角度，然后转成减速器所需要的角度，减少48倍
    if(state_pkg.direction_tp==0){
        pos_angle_t=(double)state_pkg.target_position*360/65536/multiple_reducer*direction_rotation;
    }else{
        pos_angle_t=-(double)state_pkg.target_position*360/65536/multiple_reducer*direction_rotation;
    }
    if(state_pkg.direction_cp==0){
        pos_angle_r=(double)state_pkg.current_position*360/65536/multiple_reducer*direction_rotation;
    }else{
        pos_angle_r=-(double)state_pkg.current_position*360/65536/multiple_reducer*direction_rotation;
    }
    if(state_pkg.direction_pe==0){
        pos_error=(double)state_pkg.position_error*360/65536/multiple_reducer*direction_rotation;
    }else{
        pos_error=-(double)state_pkg.position_error*360/65536/multiple_reducer*direction_rotation;
    }
    int temp=0;
    if(state_pkg.direction_tv==0){
        temp=1;
    }else{
        temp=-1;
    }
    // std::cout<<"第"<<static_cast<int>(state_pkg.addr_)<<"号电机状态："<<std::endl;
    // std::cout<<"目标位置:"<<pos_angle_t<<"度"<<std::endl;
    // std::cout<<"当前位置:"<<pos_angle_r<<"度"<<std::endl;
    // std::cout<<"位置误差:"<<pos_error<<"度"<<std::endl;
    // std::cout<<"总线电压:"<<state_pkg.bus_voltage<<"mV"<<std::endl;
    // std::cout<<"相电流:"<<state_pkg.bus_phase_current<<"mA"<<std::endl;
    // std::cout<<"校准后编报器值:"<<state_pkg.encoder_value<<std::endl;
    // std::cout<<"速度:"<<temp*state_pkg.target_velocity<<"RPM"<<std::endl;
}



Servers_::Servers_(ros::NodeHandle nh):server1(nh,1),server2(nh,2),server3(nh,3)
{
    arm_angle_r_pub = nh.advertise<uam_message::arm_angle>("/wjl/arm/real/angle_r", 10);
    arm_angle_error_pub = nh.advertise<uam_message::arm_angle>("/wjl/arm/real/angle_error", 10);
    pos_angle_sub=nh.subscribe<uam_message::arm_angle>("/wjl/arm/real/angle_d",10,&Servers_::Pos_target_cb,this);
}
Servers_::~Servers_(){
    
}

void Servers_::Arm_angle_pub(){
    uam_message::arm_angle arm_angle_msg;
    arm_angle_msg.arm1_angle=server1.pos_angle_r;
    arm_angle_msg.arm2_angle=server2.pos_angle_r;
    arm_angle_msg.hand_angle=server3.pos_angle_r;
    arm_angle_r_pub.publish(arm_angle_msg);

    uam_message::arm_angle arm_angle_error_msg;
    arm_angle_error_msg.arm1_angle=server1.pos_error;
    arm_angle_error_msg.arm2_angle=server2.pos_error;
    arm_angle_error_msg.hand_angle=server3.pos_error;
    arm_angle_error_pub.publish(arm_angle_error_msg);
}

void Servers_::Pos_target_cb(const boost::shared_ptr<const uam_message::arm_angle>& msg){
    server1.pos_angle_s=msg->arm1_angle;
    server2.pos_angle_s=msg->arm2_angle;
    server3.pos_angle_s=msg->hand_angle;
}