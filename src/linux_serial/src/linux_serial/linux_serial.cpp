#include "linux_serial/linux_serial.h"
#include "Servos/Server_.h"
Linux_serial::Linux_serial(ros::NodeHandle &nh,std::string usb_name):
sp(iosev, usb_name),servers(nh)
{
    //串口初始化
    sp.set_option(serial_port::baud_rate(115200));
    sp.set_option(serial_port::flow_control(serial_port::flow_control::none));//无流控制
    sp.set_option(serial_port::parity(serial_port::parity::none));//不使用奇偶校验
    sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));//设置串口的停止位为一个停止位
    sp.set_option(serial_port::character_size(8)); //串口的数据位为8位
    // 启动异步读取
    start_async_read();
    
    // 启动IO服务线程
    io_thread = std::thread([this](){
        ROS_INFO("IO thread started");
        iosev.run();
        ROS_INFO("IO thread exited");
    });
}
Linux_serial::~Linux_serial(){
}


/**
  * @brief 发送一次串口信息
*/
void Linux_serial::Send_data(uint8_t *data,int len)
{
    // 使用boost库中的write函数发送数据
    boost::asio::write(sp, boost::asio::buffer(data,len));
}
/**
  * @brief 发送所有串口信息
*/
void Linux_serial::Send_all_data(){
    servers.server1.Read_Sys_Params();// 读取系统参数
    Send_data(servers.server1.send_pack,servers.server1.send_len);// 发送数据
    ros::Duration(0.01).sleep();// 等待0.1秒
    servers.server1.Pos_Control();// 位置控制
    Send_data(servers.server1.send_pack,servers.server1.send_len);// 发送数据

    ros::Duration(0.01).sleep();
    servers.server2.Read_Sys_Params();
    Send_data(servers.server2.send_pack,servers.server2.send_len);
    ros::Duration(0.01).sleep();
    servers.server2.Pos_Control();
    Send_data(servers.server2.send_pack,servers.server2.send_len);

    ros::Duration(0.01).sleep();
    servers.server3.Read_Sys_Params();
    Send_data(servers.server3.send_pack,servers.server3.send_len);
    ros::Duration(0.01).sleep();
    servers.server3.Pos_Control();
    Send_data(servers.server3.send_pack,servers.server3.send_len);

    ros::Duration(0.01).sleep();
    servers.server4.Read_Sys_Params();
    Send_data(servers.server4.send_pack,servers.server4.send_len);
    ros::Duration(0.01).sleep();
    servers.server4.Pos_Control();
    Send_data(servers.server4.send_pack,servers.server4.send_len);

    ros::Duration(0.01).sleep();
    servers.Arm_angle_pub();// 发布角度

}


/**
 * @brief 开始异步读取数据
 *
 * 使用异步读取函数，从串行端口读取数据到read_buf中，数据大小为sizeof(read_buf)。
 * 读取完成后，调用handle_read函数处理读取到的数据。
 */
void Linux_serial::start_async_read()
{
    // 使用异步读取函数，读取数据到read_buf中，大小为sizeof(read_buf)
    sp.async_read_some(boost::asio::buffer(read_buf, sizeof(read_buf)),
        // 当读取完成时，调用handle_read函数
        boost::bind(&Linux_serial::handle_read, this,
        // 传递错误信息
        boost::asio::placeholders::error,
        // 传递读取的字节数
        boost::asio::placeholders::bytes_transferred));
}

/**
 * @brief 处理串口读取完成事件
 *
 * 当串口读取完成时，该函数将被调用。
 *
 * @param error 错误码，表示读取过程中是否发生错误
 * @param bytes_transferred 实际读取的字节数
 */
void Linux_serial::handle_read(const boost::system::error_code& error, size_t bytes_transferred)
{
    // 检查是否发生错误
    if (!error)
    {
        // 将新数据追加到缓冲区
        rx_buffer.insert(rx_buffer.end(), read_buf, read_buf + bytes_transferred);
        
        // 处理协议帧
        process_rx_data();
        
        // 继续下一次读取
        start_async_read();
    }
    else 
    {
        // 如果操作被取消
        if(error == boost::asio::error::operation_aborted) {
            ROS_DEBUG("Read operation cancelled");
        } else {
            // 输出错误信息
            ROS_ERROR_STREAM("Serial read error: " << error.message());
        }
    }
}

/**
 * @brief 处理接收到的数据
 *
 * 该函数负责处理接收缓冲区中的数据，寻找并处理有效的数据帧。
 *
 * @details
 * 1. 首先检查接收缓冲区中是否至少有4个字节（帧头和帧尾），如果没有，则直接返回。
 * 2. 在接收缓冲区中查找可能的帧头（0x01, 0x02, 0x03, 0x04）。
 * 3. 如果没有找到任何有效帧头，则清空接收缓冲区并返回。
 * 4. 移除帧头前的无效数据。
 * 5. 检查剩余数据是否足够（至少包含帧头和帧尾），如果不够，则返回。
 * 6. 在剩余数据中查找帧尾（0x6B）。
 * 7. 如果没有找到帧尾，则等待更多数据，并返回。
 * 8. 计算帧的长度（包括帧头和帧尾）。
 * 9. 提取完整的帧数据。
 * 10. 调用handle_valid_frame函数处理有效帧。
 * 11. 移除已处理的数据。
 * 12. 如果接收缓冲区中的数据超过1024字节，则发出警告并清空接收缓冲区。
 */
void Linux_serial::process_rx_data() {
    while (rx_buffer.size() >= 4) {  // 至少需要帧头+帧尾
        // 查找可能的帧头（0x01, 0x02, 0x03, 0x04）
        auto it = std::find_first_of(
            rx_buffer.begin(), rx_buffer.end(),
            std::begin({0x01, 0x02, 0x03, 0x04}),
            std::end({0x01, 0x02, 0x03, 0x04})
        );

        if (it == rx_buffer.end()) {
            // 没有任何有效帧头，清空缓冲区
            rx_buffer.clear();
            return;
        }

        // 移除帧头前的无效数据
        rx_buffer.erase(rx_buffer.begin(), it);

        // 检查剩余数据是否足够（至少帧头+帧尾）
        if (rx_buffer.size() < 4) return;

        // 查找帧尾 0x6B
        auto end_it = std::find(rx_buffer.begin() + 1, rx_buffer.end(), 0x6B);
        if (end_it == rx_buffer.end()) {
            // 没有找到帧尾，等待更多数据
            return;
        }

        // 计算帧长度（包括帧头和帧尾）
        size_t frame_length = std::distance(rx_buffer.begin(), end_it) + 1;

        // 提取完整帧
        std::vector<uint8_t> frame(rx_buffer.begin(), rx_buffer.begin() + frame_length);

        // 处理有效帧
        handle_valid_frame(frame);

        // 移除已处理的数据
        rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + frame_length);
    }

    // 防止缓冲区无限增长
    if (rx_buffer.size() > 1024) {
        ROS_WARN("Rx buffer overflow, clearing...");
        rx_buffer.clear();
    }
}

/**
 * @brief 处理有效的串行帧,提取有效数据
 *
 * 根据帧的第一个字节决定如何解析和处理帧数据。
 *
 * @param frame 帧数据，类型为 std::vector<uint8_t>
 */
void Linux_serial::handle_valid_frame(const std::vector<uint8_t>& frame)
{
    int a=frame.size();
    switch (frame[0])
    {
    case 0x01:
        if (a==8 && frame[1]==0x36 && frame[7]==0x6B){
            // // 大端序（高位字节在前）,就这样转换
            // uint32_t angle = ((uint32_t)frame[3] << 24) | 
            //      ((uint32_t)frame[4] << 16) | 
            //      ((uint32_t)frame[5] << 8)  | 
            //      ((uint32_t)frame[6]);
            // // 输出结果
            // double c=(double)angle*360/65536;
            // std::cout<<"实际角度:"<<c<< std::endl;
        }else if(a==31 && frame[1]==0x43 && frame[30]==0x6B){
            size_t idx =4;  // 从 frame[5] 开始
            // 2. bus_voltage (2 bytes, 大端序)
            servers.server1.state_pkg.bus_voltage = (frame[idx] << 8) | frame[idx + 1];
            idx += 2;
            // 3. bus_phase_current (2 bytes)
            servers.server1.state_pkg.bus_phase_current = (frame[idx] << 8) | frame[idx + 1];
            idx += 2;
            // 4. encoder_value (2 bytes)
            servers.server1.state_pkg.encoder_value = (frame[idx] << 8) | frame[idx + 1];
            idx += 2;
            // 5. direction_tp (1 byte)
            servers.server1.state_pkg.direction_tp = frame[idx++];
            // 6. target_position (4 bytes, 大端序)
            servers.server1. state_pkg.target_position = (frame[idx] << 24) | (frame[idx + 1] << 16) |
                                (frame[idx + 2] << 8) | frame[idx + 3];
            idx += 4;
            // 7. direction_tv (1 byte)
            servers.server1.state_pkg.direction_tv = frame[idx++];
            // 8. target_velocity (2 bytes)
            servers.server1.state_pkg.target_velocity = (frame[idx] << 8) | frame[idx + 1];
            idx += 2;
            // 9. direction_cp (1 byte)
            servers.server1.state_pkg.direction_cp = frame[idx++];
            // 10. current_position (4 bytes)
            servers.server1.state_pkg.current_position = (frame[idx] << 24) | (frame[idx + 1] << 16) |
                                (frame[idx + 2] << 8) | frame[idx + 3];
            idx += 4;
            // 11. direction_pe (1 byte)
            servers.server1.state_pkg.direction_pe = frame[idx++];
            // 12. position_error (4 bytes)
            servers.server1.state_pkg.position_error = (frame[idx] << 24) | (frame[idx + 1] << 16) |
                                (frame[idx + 2] << 8) | frame[idx + 3];
            idx += 4;
            // 13. ready_status (1 byte)
            servers.server1.state_pkg.ready_status = frame[idx++];
            // 14. motor_status (1 byte)
            servers.server1.state_pkg.motor_status = frame[idx];\
            servers.server1.State_show();
        }
        break;
    case 0x02:
        if (a==8 && frame[1]==0x36 && frame[7]==0x6B){

        }else if(a==31 && frame[1]==0x43 && frame[30]==0x6B){
            size_t idx =4;  // 从 frame[5] 开始
            servers.server2.state_pkg.bus_voltage = (frame[idx] << 8) | frame[idx + 1];
            idx += 2;
            servers.server2.state_pkg.bus_phase_current = (frame[idx] << 8) | frame[idx + 1];
            idx += 2;
            servers.server2.state_pkg.encoder_value = (frame[idx] << 8) | frame[idx + 1];
            idx += 2;
            servers.server2.state_pkg.direction_tp = frame[idx++];
            servers.server2. state_pkg.target_position = (frame[idx] << 24) | (frame[idx + 1] << 16) |
                                (frame[idx + 2] << 8) | frame[idx + 3];
            idx += 4;
            servers.server2.state_pkg.direction_tv = frame[idx++];
            servers.server2.state_pkg.target_velocity = (frame[idx] << 8) | frame[idx + 1];
            idx += 2;
            servers.server2.state_pkg.direction_cp = frame[idx++];
            servers.server2.state_pkg.current_position = (frame[idx] << 24) | (frame[idx + 1] << 16) |
                                (frame[idx + 2] << 8) | frame[idx + 3];
            idx += 4;
            servers.server2.state_pkg.direction_pe = frame[idx++];
            servers.server2.state_pkg.position_error = (frame[idx] << 24) | (frame[idx + 1] << 16) |
                                (frame[idx + 2] << 8) | frame[idx + 3];
            idx += 4;
            servers.server2.state_pkg.ready_status = frame[idx++];
            servers.server2.state_pkg.motor_status = frame[idx];\
            servers.server2.State_show();
        }
        break;
    case 0x03:
        if (a==8 && frame[1]==0x36 && frame[7]==0x6B){

        }else if(a==31 && frame[1]==0x43 && frame[30]==0x6B){
            size_t idx =4;  // 从 frame[5] 开始
            servers.server3.state_pkg.bus_voltage = (frame[idx] << 8) | frame[idx + 1];
            idx += 2;
            servers.server3.state_pkg.bus_phase_current = (frame[idx] << 8) | frame[idx + 1];
            idx += 2;
            servers.server3.state_pkg.encoder_value = (frame[idx] << 8) | frame[idx + 1];
            idx += 2;
            servers.server3.state_pkg.direction_tp = frame[idx++];
            servers.server3. state_pkg.target_position = (frame[idx] << 24) | (frame[idx + 1] << 16) |
                                (frame[idx + 2] << 8) | frame[idx + 3];
            idx += 4;
            servers.server3.state_pkg.direction_tv = frame[idx++];
            servers.server3.state_pkg.target_velocity = (frame[idx] << 8) | frame[idx + 1];
            idx += 2;
            servers.server3.state_pkg.direction_cp = frame[idx++];
            servers.server3.state_pkg.current_position = (frame[idx] << 24) | (frame[idx + 1] << 16) |
                                (frame[idx + 2] << 8) | frame[idx + 3];
            idx += 4;
            servers.server3.state_pkg.direction_pe = frame[idx++];
            servers.server3.state_pkg.position_error = (frame[idx] << 24) | (frame[idx + 1] << 16) |
                                (frame[idx + 2] << 8) | frame[idx + 3];
            idx += 4;
            servers.server3.state_pkg.ready_status = frame[idx++];
            servers.server3.state_pkg.motor_status = frame[idx];\
            servers.server3.State_show();
        }
        break;
    case 0x04:
        if (a==8 && frame[1]==0x36 && frame[7]==0x6B){

        }else if(a==31 && frame[1]==0x43 && frame[30]==0x6B){
            size_t idx =4;  // 从 frame[5] 开始
            servers.server4.state_pkg.bus_voltage = (frame[idx] << 8) | frame[idx + 1];
            idx += 2;
            servers.server4.state_pkg.bus_phase_current = (frame[idx] << 8) | frame[idx + 1];
            idx += 2;
            servers.server4.state_pkg.encoder_value = (frame[idx] << 8) | frame[idx + 1];
            idx += 2;
            servers.server4.state_pkg.direction_tp = frame[idx++];
            servers.server4. state_pkg.target_position = (frame[idx] << 24) | (frame[idx + 1] << 16) |
                                (frame[idx + 2] << 8) | frame[idx + 3];
            idx += 4;
            servers.server4.state_pkg.direction_tv = frame[idx++];
            servers.server4.state_pkg.target_velocity = (frame[idx] << 8) | frame[idx + 1];
            idx += 2;
            servers.server4.state_pkg.direction_cp = frame[idx++];
            servers.server4.state_pkg.current_position = (frame[idx] << 24) | (frame[idx + 1] << 16) |
                                (frame[idx + 2] << 8) | frame[idx + 3];
            idx += 4;
            servers.server4.state_pkg.direction_pe = frame[idx++];
            servers.server4.state_pkg.position_error = (frame[idx] << 24) | (frame[idx + 1] << 16) |
                                (frame[idx + 2] << 8) | frame[idx + 3];
            idx += 4;
            servers.server4.state_pkg.ready_status = frame[idx++];
            servers.server4.state_pkg.motor_status = frame[idx];\
            servers.server4.State_show();
        }
        break;
    }
    
    
}