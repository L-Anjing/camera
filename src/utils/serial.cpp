#include <serial/serial.h>
#include <thread>
#include <sstream>
#include <vector>
#include <cstring>   // memcpy
#include <iomanip>   // setw setfill
#include <limits>
#include <exception>
#include <memory>
#include <iostream>

#if defined(BUILD_WITH_ROS1)
#include <ros/ros.h>
#include <std_msgs/String.h>
#elif defined(BUILD_WITH_ROS2)
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#endif

serial::Serial ser;

#if defined(BUILD_WITH_ROS2)
std::shared_ptr<rclcpp::Node> g_node;
#endif

// 包头包尾（binary）·
const uint8_t CAM_HEAD[2] = {0xFF, 0xFE};
const uint8_t CAM_TAIL[2] = {0xAA, 0xDD};

// 解析 bbox 的字符串并生成串口二进制 payload
// 输入示例: "1,12.3,45.6,78.9,0.123"  --> 5 个字段，最后一个为 yaw
// 输出 payload: [ID(1B)] [x(4B float)] [y(4B float)] [z(4B float)] [yaw(4B float)]
bool parseBBoxString(const std::string &msg, std::vector<uint8_t> &payload)
{
    std::stringstream ss(msg);
    std::string token;
    std::vector<std::string> tokens;

    while (std::getline(ss, token, ','))
        tokens.push_back(token);

    // 现在期望 5 个字段： id,x,y,z,yaw
    if (tokens.size() != 5)
        return false;

    int cls = 0;
    float x = 0.f, y = 0.f, z = 0.f, yaw = 0.f;
    try {
        cls = std::stoi(tokens[0]);
        x = std::stof(tokens[1]);
        y = std::stof(tokens[2]);
        z = std::stof(tokens[3]);
        yaw = std::stof(tokens[4]);
    } catch (const std::exception &e) {
#if defined(BUILD_WITH_ROS1)
        ROS_WARN_STREAM("[serial_bridge] parse error: " << e.what() << " input: " << msg);
#elif defined(BUILD_WITH_ROS2)
        if (g_node)
            RCLCPP_WARN(g_node->get_logger(), "[serial_bridge] parse error: %s input: %s", e.what(), msg.c_str());
#endif
        return false;
    }

    // 根据你的规则： cls 允许范围 1..3
    // 1
    if (!(cls >= 1 && cls <= 3))
        return false;
    // 如果 x == 0 则视为无效
    if (x == 0.0f)
        return false;

    // 组装二进制 payload
    payload.clear();
    payload.push_back(static_cast<uint8_t>(cls)); // ID

    auto append_float = [&](float value) {
        uint8_t bytes[4];
        std::memcpy(bytes, &value, sizeof(float)); // 小端序的原生 float bytes
        payload.insert(payload.end(), bytes, bytes + 4);
    };

    append_float(x);
    append_float(y);
    append_float(z);
    append_float(yaw);

    return true;
}

// 回调 1：来自 camera 的发送请求（需要加包头包尾）
#if defined(BUILD_WITH_ROS1)
void CameraCallback(const std_msgs::String::ConstPtr &msg)
#elif defined(BUILD_WITH_ROS2)
void CameraCallback(const std_msgs::msg::String::SharedPtr msg)
#endif
{
    if (!ser.isOpen()) {
#if defined(BUILD_WITH_ROS1)
        ROS_WARN_THROTTLE(5.0, "[serial_bridge] serial port not open");
#elif defined(BUILD_WITH_ROS2)
        if (g_node)
            RCLCPP_WARN(g_node->get_logger(), "[serial_bridge] serial port not open");
#endif
        return;
    }

    std::vector<uint8_t> payload;
    if (!parseBBoxString(msg->data, payload)) {
#if defined(BUILD_WITH_ROS1)
        ROS_WARN_STREAM("[serial_bridge] Invalid k4a bbox: " << msg->data);
#elif defined(BUILD_WITH_ROS2)
        if (g_node)
            RCLCPP_WARN(g_node->get_logger(), "[serial_bridge] Invalid k4a bbox: %s", msg->data.c_str());
#endif
        return;
    }

    // 构建帧： HEAD + payload + TAIL  （不加 checksum）
    std::vector<uint8_t> packet;
    packet.insert(packet.end(), CAM_HEAD, CAM_HEAD + 2);
    packet.insert(packet.end(), payload.begin(), payload.end());
    packet.insert(packet.end(), CAM_TAIL, CAM_TAIL + 2);

    // 发送二进制数据（使用 const uint8_t*）
    try {
        ser.write(packet.data(), packet.size());
    } catch (const std::exception &e) {
#if defined(BUILD_WITH_ROS1)
        ROS_ERROR_STREAM("[serial_bridge] serial write failed: " << e.what());
#elif defined(BUILD_WITH_ROS2)
        if (g_node)
            RCLCPP_ERROR(g_node->get_logger(), "[serial_bridge] serial write failed: %s", e.what());
#endif
        return;
    }

    // 日志：输出 HEX 便于调试
    std::stringstream dbg;
    dbg << std::hex << std::uppercase;
    for (uint8_t b : packet) {
        dbg << std::setw(2) << std::setfill('0') << static_cast<int>(b) << " ";
    }
#if defined(BUILD_WITH_ROS1)
    ROS_INFO_STREAM("[serial_bridge] TX Camera -> Serial HEX: " << dbg.str());
#elif defined(BUILD_WITH_ROS2)
    if (g_node)
        RCLCPP_INFO(g_node->get_logger(), "[serial_bridge] TX Camera -> Serial HEX: %s", dbg.str().c_str());
#endif
}




int main(int argc, char **argv)
{
#if defined(BUILD_WITH_ROS1)
    ros::init(argc, argv, "serial_bridge");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
#elif defined(BUILD_WITH_ROS2)
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("serial_bridge");
    g_node = node;
#endif

    std::string port;
    int baud = 115200;

#if defined(BUILD_WITH_ROS1)
    pnh.param<std::string>("port", port, "/dev/ttyUSB0");
    pnh.param<int>("baudrate", baud, 115200);
#elif defined(BUILD_WITH_ROS2)
    node->declare_parameter<std::string>("port", "/dev/ttyUSB0");
    node->declare_parameter<int>("baudrate", 115200);
    port = node->get_parameter("port").as_string();
    baud = node->get_parameter("baudrate").as_int();
#endif

    try {
        ser.setPort(port);
        ser.setBaudrate(baud);
        serial::Timeout to(serial::Timeout::max(), 1000, 0, 1000, 0);
        ser.setTimeout(to);
        ser.open();
    } catch (const std::exception &e) {
#if defined(BUILD_WITH_ROS1)
        ROS_FATAL_STREAM("Failed to open serial port: " << e.what());
#elif defined(BUILD_WITH_ROS2)
        RCLCPP_FATAL(node->get_logger(), "Failed to open serial port: %s", e.what());
#endif
        return -1;
    }

    if (!ser.isOpen()) {
#if defined(BUILD_WITH_ROS1)
        ROS_FATAL("Serial port not open after open()");
#elif defined(BUILD_WITH_ROS2)
        RCLCPP_FATAL(node->get_logger(), "Serial port not open after open()");
#endif
        return -1;
    }
#if defined(BUILD_WITH_ROS1)
    ROS_INFO("Serial port opened.");
#elif defined(BUILD_WITH_ROS2)
    RCLCPP_INFO(node->get_logger(), "Serial port opened.");
#endif

    // 订阅 camera → 串口
#if defined(BUILD_WITH_ROS1)
    ros::Subscriber sub_camera = nh.subscribe("/k4a/target_info", 10, CameraCallback);
    ros::spin();
#elif defined(BUILD_WITH_ROS2)
    auto sub_camera = node->create_subscription<std_msgs::msg::String>(
        "/k4a/target_info", 10, CameraCallback);
    rclcpp::spin(node);
    rclcpp::shutdown();
#endif

    if (ser.isOpen()) ser.close();
    return 0;
}
