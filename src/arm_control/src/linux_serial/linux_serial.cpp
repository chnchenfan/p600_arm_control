#include "linux_serial/linux_serial.h"
#include "Servos/Server_.h"
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <vector>

namespace {

constexpr uint8_t kFrameTail = 0x6B;

double HostSideErrorDeg(double desired_deg, double feedback_deg) {
    return desired_deg - feedback_deg;
}

bool IsValidFrameHead(uint8_t value) {
    return value == 0x01 || value == 0x02 || value == 0x03 || value == 0x04;
}

std::size_t ExpectedFrameLength(uint8_t function_code) {
    switch (function_code) {
        case 0x43:
            return 31;
        case 0xFD:
            return 4;
        case 0x36:
            return 8;
        default:
            return 0;
    }
}

std::string BytesToHexPreview(const uint8_t *data, size_t len, size_t max_len = 12) {
    std::ostringstream oss;
    oss << std::hex << std::setfill('0');
    const size_t preview_len = std::min(len, max_len);
    for (size_t i = 0; i < preview_len; ++i) {
        if (i != 0) oss << " ";
        oss << "0x" << std::setw(2) << static_cast<unsigned>(data[i]);
    }
    if (len > preview_len) {
        oss << " ...";
    }
    return oss.str();
}

std::string BytesToHexPreview(const std::vector<uint8_t> &data, size_t max_len = 12) {
    if (data.empty()) {
        return "(empty)";
    }
    return BytesToHexPreview(data.data(), data.size(), max_len);
}

const char *DriverTagFromAddr(uint8_t addr) {
    switch (addr) {
        case 0x01:
            return "1号驱动器";
        case 0x02:
            return "2号驱动器";
        case 0x03:
            return "3号驱动器";
        default:
            return "未知驱动器";
    }
}

Server_ *ServerByAddr(Servers_ &servers, uint8_t addr) {
    switch (addr) {
        case 0x01:
            return &servers.server1;
        case 0x02:
            return &servers.server2;
        case 0x03:
            return &servers.server3;
        default:
            return nullptr;
    }
}

void LogDriverStateThrottle(const char *tag, const Server_ &server) {
    // 这里额外打印驱动板返回的原始状态字段，是为了区分两类常见问题：
    // 1. 主机端命令已经发出，但驱动板没有真正接受/执行位置控制；
    // 2. 驱动板已经执行，但当前位置/目标位置的解析没有正确更新。
    //
    // 当 real_d 已经变化、feedback 仍长期不变时，只看角度日志不够，
    // 需要连同 ready_status / motor_status / target_position / current_position 一起看，
    // 才能判断问题是在“板子不动”还是“状态解析没跟上”。
    ROS_INFO_THROTTLE(1.0,
                      "%s 驱动状态: 就绪=%u, 电机状态=%u, 目标方向=%u, 当前位置方向=%u, 目标脉冲=%u, 当前位置脉冲=%u, 位置误差脉冲=%u, 主机目标角=%.2f, 反馈角=%.2f, 板载误差角=%.2f",
                      tag,
                      static_cast<unsigned>(server.state_pkg.ready_status),
                      static_cast<unsigned>(server.state_pkg.motor_status),
                      static_cast<unsigned>(server.state_pkg.direction_tp),
                      static_cast<unsigned>(server.state_pkg.direction_cp),
                      static_cast<unsigned>(server.state_pkg.target_position),
                      static_cast<unsigned>(server.state_pkg.current_position),
                      static_cast<unsigned>(server.state_pkg.position_error),
                      server.pos_angle_s,
                      server.pos_angle_r,
                      server.pos_error);
}

void DecodeStateFrameToServer(Server_ &server, const std::vector<uint8_t> &frame) {
    size_t idx = 4;
    server.state_pkg.bus_voltage = (frame[idx] << 8) | frame[idx + 1];
    idx += 2;
    server.state_pkg.bus_phase_current = (frame[idx] << 8) | frame[idx + 1];
    idx += 2;
    server.state_pkg.encoder_value = (frame[idx] << 8) | frame[idx + 1];
    idx += 2;
    server.state_pkg.direction_tp = frame[idx++];
    server.state_pkg.target_position = (frame[idx] << 24) | (frame[idx + 1] << 16) |
                                       (frame[idx + 2] << 8) | frame[idx + 3];
    idx += 4;
    server.state_pkg.direction_tv = frame[idx++];
    server.state_pkg.target_velocity = (frame[idx] << 8) | frame[idx + 1];
    idx += 2;
    server.state_pkg.direction_cp = frame[idx++];
    server.state_pkg.current_position = (frame[idx] << 24) | (frame[idx + 1] << 16) |
                                        (frame[idx + 2] << 8) | frame[idx + 3];
    idx += 4;
    server.state_pkg.direction_pe = frame[idx++];
    server.state_pkg.position_error = (frame[idx] << 24) | (frame[idx + 1] << 16) |
                                      (frame[idx + 2] << 8) | frame[idx + 3];
    idx += 4;
    server.state_pkg.ready_status = frame[idx++];
    server.state_pkg.motor_status = frame[idx];
    server.State_show();
}

}  // namespace

Linux_serial::Linux_serial(ros::NodeHandle &nh,std::string usb_name):
sp(iosev, usb_name),servers(nh)
{
    nh.param("arm_serial_command_gap", command_gap_sec, command_gap_sec);
    if (command_gap_sec < 0.0) {
        command_gap_sec = 0.0;
    }
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
    ros::Duration(command_gap_sec).sleep();
    servers.server1.Pos_Control();// 位置控制
    Send_data(servers.server1.send_pack,servers.server1.send_len);// 发送数据

    ros::Duration(command_gap_sec).sleep();
    servers.server2.Read_Sys_Params();
    Send_data(servers.server2.send_pack,servers.server2.send_len);
    ros::Duration(command_gap_sec).sleep();
    servers.server2.Pos_Control();
    Send_data(servers.server2.send_pack,servers.server2.send_len);

    ros::Duration(command_gap_sec).sleep();
    servers.server3.Read_Sys_Params();
    Send_data(servers.server3.send_pack,servers.server3.send_len);
    ros::Duration(command_gap_sec).sleep();
    servers.server3.Pos_Control();
    Send_data(servers.server3.send_pack,servers.server3.send_len);

    ros::Duration(command_gap_sec).sleep();
    servers.Arm_angle_pub();// 发布角度

    const double arm1_host_error = HostSideErrorDeg(servers.server1.pos_angle_s, servers.server1.pos_angle_r);
    const double arm2_host_error = HostSideErrorDeg(servers.server2.pos_angle_s, servers.server2.pos_angle_r);
    const double hand_host_error = HostSideErrorDeg(servers.server3.pos_angle_s, servers.server3.pos_angle_r);
    const ros::Time now = ros::Time::now();
    const bool rx_recent = !last_rx_time.isZero() && (now - last_rx_time).toSec() <= 1.0;
    const bool state_recent = !last_state_frame_time.isZero() && (now - last_state_frame_time).toSec() <= 1.0;
    const bool ack_recent = !last_ack_frame_time.isZero() && (now - last_ack_frame_time).toSec() <= 1.0;

    ROS_INFO_THROTTLE(1.0,
                      // 这里同时打印两种误差：
                      // 1. board_err：驱动板在状态包里返回的位置误差；
                      // 2. host_err：主机端直接用 desired-feedback 算出来的真实角度差。
                      //
                      // 之前遇到的现象是：desired 已经变成 -5 deg，但 feedback 仍然是 0，
                      // 同时 board_err 也还是 0。为避免“板子上报误差为 0”误导判断，
                      // 这里必须把 host_err 也一起打出来。
                      "串口循环: 目标角=(%.2f, %.2f, %.2f), 反馈角=(%.2f, %.2f, %.2f), 板载误差=(%.2f, %.2f, %.2f), 主机误差=(%.2f, %.2f, %.2f)",
                      servers.server1.pos_angle_s, servers.server2.pos_angle_s, servers.server3.pos_angle_s,
                      servers.server1.pos_angle_r, servers.server2.pos_angle_r, servers.server3.pos_angle_r,
                      servers.server1.pos_error, servers.server2.pos_error, servers.server3.pos_error,
                      arm1_host_error, arm2_host_error, hand_host_error);

    // 这里保留一条简短的中文健康提示，用来快速判断接收链路当前处于哪一类状态：
    // 1. 最近完全没有任何回包；
    // 2. 只收到了位置控制应答，没有收到完整状态帧；
    // 3. 状态帧恢复正常，可以继续看高层几何与控制问题。
    if (!rx_recent) {
        ROS_WARN_THROTTLE(1.0,
                          "串口接收健康: 最近1秒未收到任何回包, 解析错误累计=%zu",
                          parse_error_count);
    } else if (!state_recent && ack_recent) {
        ROS_WARN_THROTTLE(1.0,
                          "串口接收健康: 最近1秒只收到位置应答，未收到状态帧, 解析错误累计=%zu",
                          parse_error_count);
    } else if (state_recent) {
        ROS_INFO_THROTTLE(1.0,
                          "串口接收健康: 最近1秒状态帧正常, 解析错误累计=%zu",
                          parse_error_count);
    } else {
        ROS_WARN_THROTTLE(1.0,
                          "串口接收健康: 最近1秒收到回包，但尚未识别到状态帧或位置应答, 解析错误累计=%zu",
                          parse_error_count);
    }

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
        last_rx_time = ros::Time::now();

        // 这里增加一层“原始串口数据预览”，是为了确认 RX 回包是否真的进到了程序。
        // 如果现场现象是“命令已发出，但驱动状态日志完全不出现”，那就需要先判断：
        // 1. 是不是根本没有任何回包进入当前进程；
        // 2. 还是回包已经来了，但格式和当前解析逻辑不匹配。
        ROS_INFO_THROTTLE(1.0,
                          "串口原始接收: 本次字节数=%zu, 预览=%s",
                          bytes_transferred,
                          BytesToHexPreview(read_buf, bytes_transferred).c_str());
        
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
    while (!rx_buffer.empty()) {
        auto it = std::find_if(rx_buffer.begin(), rx_buffer.end(), [](uint8_t value) {
            return IsValidFrameHead(value);
        });

        if (it == rx_buffer.end()) {
            const std::vector<uint8_t> preview_buffer(rx_buffer.begin(), rx_buffer.end());
            ROS_WARN_THROTTLE(1.0,
                              "串口解析: 缓冲区中没有识别到有效帧头，当前缓存长度=%zu, 预览=%s",
                              rx_buffer.size(),
                              BytesToHexPreview(preview_buffer).c_str());
            rx_buffer.clear();
            return;
        }

        if (it != rx_buffer.begin()) {
            const std::vector<uint8_t> dropped(rx_buffer.begin(), it);
            ROS_WARN_THROTTLE(1.0,
                              "串口解析: 丢弃帧头前的无效字节, 长度=%zu, 预览=%s",
                              dropped.size(),
                              BytesToHexPreview(dropped).c_str());
            rx_buffer.erase(rx_buffer.begin(), it);
        }

        if (rx_buffer.size() < 2) {
            return;
        }

        const uint8_t frame_head = rx_buffer[0];
        const uint8_t function_code = rx_buffer[1];
        const std::size_t expected_frame_length = ExpectedFrameLength(function_code);

        // 这里改成“按功能码定长分帧”，是因为 0x43 状态帧内部 payload 里也可能出现 0x6B。
        // 如果还按“找到第一个 0x6B 就截帧”，31 字节状态帧就会被提前截成 14 字节等错误长度，
        // 后面的缓冲区也会整体错位，最终造成 arm2 状态反馈长期异常。
        if (expected_frame_length == 0) {
            ++parse_error_count;
            ROS_WARN_THROTTLE(1.0,
                              "串口解析: 收到未支持的功能码 0x%02x，丢弃一个字节重同步, 原始预览=%s",
                              static_cast<unsigned>(function_code),
                              BytesToHexPreview(std::vector<uint8_t>(rx_buffer.begin(),
                                                                     rx_buffer.begin() + std::min<std::size_t>(rx_buffer.size(), 12))).c_str());
            rx_buffer.pop_front();
            continue;
        }

        if (rx_buffer.size() < expected_frame_length) {
            return;
        }

        if (rx_buffer[expected_frame_length - 1] != kFrameTail) {
            ++parse_error_count;
            const std::vector<uint8_t> preview(rx_buffer.begin(),
                                               rx_buffer.begin() + expected_frame_length);
            ROS_WARN_THROTTLE(1.0,
                              "串口解析: 长度或帧尾非法，丢弃一个字节重同步, 期望长度=%zu, 帧头=0x%02x, 功能码=0x%02x, 预览=%s",
                              expected_frame_length,
                              static_cast<unsigned>(frame_head),
                              static_cast<unsigned>(function_code),
                              BytesToHexPreview(preview).c_str());
            rx_buffer.pop_front();
            continue;
        }

        std::vector<uint8_t> frame(rx_buffer.begin(), rx_buffer.begin() + expected_frame_length);
        ROS_INFO_THROTTLE(1.0,
                          "串口解析: 命中完整帧, 长度=%zu, 帧头=0x%02x, 功能码=0x%02x",
                          expected_frame_length,
                          static_cast<unsigned>(frame_head),
                          static_cast<unsigned>(function_code));
        handle_valid_frame(frame);
        rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + expected_frame_length);
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
    const int frame_size = static_cast<int>(frame.size());
    Server_ *server = ServerByAddr(servers, frame[0]);
    if (server == nullptr) {
        ROS_WARN_THROTTLE(1.0,
                          "串口解析: 收到未识别帧头 0x%02x, 长度=%d, 原始预览=%s",
                          static_cast<unsigned>(frame[0]),
                          frame_size,
                          BytesToHexPreview(frame).c_str());
        return;
    }

    const uint8_t function_code = frame[1];
    const ros::Time now = ros::Time::now();

    if (function_code == 0x43 && frame_size == 31 && frame[30] == kFrameTail) {
        DecodeStateFrameToServer(*server, frame);
        last_state_frame_time = now;
        last_state_frame_time_by_addr[frame[0]] = now;
        LogDriverStateThrottle(DriverTagFromAddr(frame[0]), *server);
        return;
    }

    if (function_code == 0xFD && frame_size == 4 && frame[3] == kFrameTail) {
        last_ack_frame_time = now;
        last_ack_frame_time_by_addr[frame[0]] = now;
        ROS_INFO_THROTTLE(1.0,
                          "%s 位置应答: 应答码=0x%02x, 原始预览=%s",
                          DriverTagFromAddr(frame[0]),
                          static_cast<unsigned>(frame[2]),
                          BytesToHexPreview(frame).c_str());
        return;
    }

    if (function_code == 0x36 && frame_size == 8 && frame[7] == kFrameTail) {
        ROS_INFO_THROTTLE(1.0,
                          "%s 当前位置回读: 原始预览=%s",
                          DriverTagFromAddr(frame[0]),
                          BytesToHexPreview(frame).c_str());
        return;
    }

    ++parse_error_count;
    ROS_WARN_THROTTLE(1.0,
                      "串口解析: 收到未按预期匹配的完整帧, 帧头=0x%02x, 功能码=0x%02x, 长度=%d, 原始预览=%s",
                      static_cast<unsigned>(frame[0]),
                      static_cast<unsigned>(function_code),
                      frame_size,
                      BytesToHexPreview(frame).c_str());
}
