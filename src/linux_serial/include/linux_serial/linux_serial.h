#include <ros/ros.h>
#include <boost/asio.hpp>
#include "linux_serial/Sub_data.h"
#define DATA 2
using namespace boost::asio;
//订阅消息和串口处理是分开的
class Linux_serial{
public:
    Sub_data sub_data;
    //串口相关对象
    boost::asio::io_service iosev;
    boost::asio::serial_port sp;
    Linux_serial(ros::NodeHandle &nh,std::string usb_name);
    ~Linux_serial();
    void Send_data();
    void Read_data();
private:
    unsigned char send_pack_info[DATA+2];
    void Int2char(int value,unsigned char& high_byte,unsigned char& low_byte);//传输的int值在uint16_t范围内，对应好
    uint16_t Char2int(uint8_t high_byte, uint8_t low_byte);//注意这里接受整数范围为uint16_t的范围
    //若是传输int32_t的信息，则需要4个字节变量去转换
    //若是传输int16_t的信息，则需要2个字节变量去转换
};
