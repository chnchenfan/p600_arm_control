#include <ros/ros.h>
#include <boost/asio.hpp>
#include "Servos/Server_.h"
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <thread>
#include <deque>
#include <array>
#include <cstddef>
using namespace boost::asio;
class Linux_serial{
public:
    Servers_ servers;//所有电机
    //串口相关对象
    boost::asio::io_service iosev;//用于管理异步操作。
    boost::asio::serial_port sp;//用于实际进行串口通信。
    Linux_serial(ros::NodeHandle &nh,std::string usb_name);
    ~Linux_serial();
    void Send_data(uint8_t *data,int len);
    void Send_all_data();
private:
    double command_gap_sec = 0.005;
    std::deque<uint8_t> rx_buffer;  // 接收数据缓冲区
    uint8_t read_buf[128];          // 临时读取缓冲区
    std::thread io_thread;         // IO服务线程,这里用了串口异步通讯
    // 下面这些时间戳和计数器用于判断“最近是否真的收到了状态回包”。
    // 现场排查时，单看目标角/反馈角不够，需要区分：
    // 1. 完全没有收到任何回包；
    // 2. 只收到位置应答，没有收到完整状态帧；
    // 3. 状态帧正常，但某一轴反馈仍异常。
    ros::Time last_rx_time;
    ros::Time last_state_frame_time;
    ros::Time last_ack_frame_time;
    std::array<ros::Time, 4> last_state_frame_time_by_addr;
    std::array<ros::Time, 4> last_ack_frame_time_by_addr;
    std::size_t parse_error_count = 0;
    void start_async_read();
    void handle_read(const boost::system::error_code& error, size_t bytes_transferred);
    void process_rx_data();
    void handle_valid_frame(const std::vector<uint8_t>& frame);
};
