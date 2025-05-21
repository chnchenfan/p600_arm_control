#include "linux_serial/linux_serial.h"
Linux_serial::Linux_serial(ros::NodeHandle &nh,std::string usb_name):
sp(iosev, usb_name),sub_data(nh),send_pack_info{ 0xFF,0,0,0xFE }
{
    //串口初始化
    sp.set_option(serial_port::baud_rate(115200));
    sp.set_option(serial_port::flow_control(serial_port::flow_control::none));//无流控制
    sp.set_option(serial_port::parity(serial_port::parity::none));//不使用奇偶校验
    sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));//设置串口的停止位为一个停止位
    sp.set_option(serial_port::character_size(8)); //串口的数据位为8位
}
Linux_serial::~Linux_serial(){
}


/********************************************************
函数功能：传输数据
入口参数：传入位置信息
出口参数：
********************************************************/
void Linux_serial::Send_data()
{

    //high_byte是0，低位是1
	Int2char(sub_data.pos_value,send_pack_info[1],send_pack_info[2]);
    // 通过串口下发数据
    boost::asio::write(sp, boost::asio::buffer(send_pack_info));
}

void Linux_serial::Read_data()
{
    // 创建一个缓冲区来存储读取的数据
    std::vector<char> receive_data(4);

    // 读取串口数据
    size_t bytes_read = boost::asio::read(sp, boost::asio::buffer(receive_data));
    if (receive_data[0] == 0xFF  && receive_data[3] == 0xFE)
    {
        uint8_t a=receive_data[1];//高位
        uint8_t b=receive_data[2];
        uint16_t c=Char2int(a,b);
        // 数据一致，进行相应的操作
    }
    else
    {
        std::cout<<"数据有问题！！！"<<std::endl;
    }
    //后续处理
}
void Linux_serial::Int2char(int value,unsigned char& high_byte,unsigned char& low_byte){
    high_byte = value >> 8;
    low_byte = value & 0xff;
}

uint16_t Linux_serial::Char2int(uint8_t high_byte, uint8_t low_byte) {
    return (high_byte << 8) | low_byte;
}