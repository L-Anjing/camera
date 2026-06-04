#include <serial/serial.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <thread>
#include <sstream>
#include <vector>
#include <cstring>
#include <iomanip>
#include <limits>
#include <exception>
#include <memory>
#include <iostream>

serial::Serial ser;
std::shared_ptr<rclcpp::Node> g_node;

// 包头包尾
const uint8_t CAM_HEAD[2] = {0xFF, 0xFE};
const uint8_t CAM_TAIL[2] = {0xAA, 0xDD};

// ======================================================
// 解析 bbox 字符串
// 输入:
//   "1,12.3,45.6,78.9,0.123"
//
// 输出 payload:
//   [ID(1B)]
//   [x(4B float)]
//   [y(4B float)]
//   [z(4B float)]
//   [yaw(4B float)]
//
// 如果数据异常:
//   自动发送全零数据
// ======================================================
bool parseBBoxString(const std::string &msg,
                     std::vector<uint8_t> &payload)
{
    int cls = 0;
    float x = 0.f;
    float y = 0.f;
    float z = 0.f;
    float yaw = 0.f;

    bool valid = true;

    std::stringstream ss(msg);
    std::string token;
    std::vector<std::string> tokens;

    while (std::getline(ss, token, ','))
    {
        tokens.push_back(token);
    }

    // ==================================================
    // 数据格式检查
    // ==================================================
    if (tokens.size() != 5)
    {
        valid = false;
    }
    else
    {
        try
        {
            cls = std::stoi(tokens[0]);
            x = std::stof(tokens[1]);
            y = std::stof(tokens[2]);
            z = std::stof(tokens[3]);
            yaw = std::stof(tokens[4]);
        }
        catch (const std::exception &e)
        {
            valid = false;
        }
    }

    // ==================================================
    // 合法性检查
    // ==================================================
    if (!(cls >= 1 && cls <= 3))
    {
        valid = false;
    }

    // 检查 NaN / Inf
    if (!std::isfinite(x) ||
        !std::isfinite(y) ||
        !std::isfinite(z) ||
        !std::isfinite(yaw))
    {
        valid = false;
    }

    // ==================================================
    // 如果数据异常 -> 自动发送全零
    // ==================================================
    if (!valid)
    {
        cls = 0;
        x = 0.f;
        y = 0.f;
        z = 0.f;
        yaw = 0.f;

        RCLCPP_WARN(
            g_node->get_logger(),
            "[serial_bridge] Invalid bbox, send ZERO packet. input: %s",
            msg.c_str());
    }

    // ==================================================
    // 组装 payload
    // ==================================================
    payload.clear();

    payload.push_back(static_cast<uint8_t>(cls));

    auto append_float = [&](float value)
    {
        uint8_t bytes[4];
        std::memcpy(bytes, &value, sizeof(float));
        payload.insert(payload.end(), bytes, bytes + 4);
    };

    append_float(x);
    append_float(y);
    append_float(z);
    append_float(yaw);

    return true;
}

// ======================================================
// Camera 回调
// ======================================================
void CameraCallback(const std_msgs::msg::String::SharedPtr msg)
{
    if (!ser.isOpen())
    {
        RCLCPP_WARN(
            g_node->get_logger(),
            "[serial_bridge] serial port not open");
        return;
    }

    std::vector<uint8_t> payload;

    parseBBoxString(msg->data, payload);

    // ==================================================
    // 构建完整数据包
    // HEAD + payload + TAIL
    // ==================================================
    std::vector<uint8_t> packet;

    packet.insert(packet.end(),
                  CAM_HEAD,
                  CAM_HEAD + 2);

    packet.insert(packet.end(),
                  payload.begin(),
                  payload.end());

    packet.insert(packet.end(),
                  CAM_TAIL,
                  CAM_TAIL + 2);

    // ==================================================
    // 串口发送
    // ==================================================
    try
    {
        ser.write(packet.data(), packet.size());
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(
            g_node->get_logger(),
            "[serial_bridge] serial write failed: %s",
            e.what());

        return;
    }

    // ==================================================
    // HEX 调试输出
    // ==================================================
    std::stringstream dbg;

    dbg << std::hex << std::uppercase;

    for (uint8_t b : packet)
    {
        dbg << std::setw(2)
            << std::setfill('0')
            << static_cast<int>(b)
            << " ";
    }

    // RCLCPP_INFO(
    //     g_node->get_logger(),
    //     "[serial_bridge] TX HEX: %s",
    //     dbg.str().c_str());
}

// ======================================================
// main
// ======================================================
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node =
        rclcpp::Node::make_shared("serial_bridge");

    g_node = node;

    // ==================================================
    // 参数
    // ==================================================
    node->declare_parameter<std::string>(
        "port",
        "/dev/azurekinect");

    node->declare_parameter<int>(
        "baudrate",
        115200);

    std::string port =
        node->get_parameter("port").as_string();

    int baud =
        node->get_parameter("baudrate").as_int();

    // ==================================================
    // 打开串口
    // ==================================================
    try
    {
        ser.setPort(port);

        ser.setBaudrate(baud);

        serial::Timeout to(
            serial::Timeout::max(),
            1000,
            0,
            1000,
            0);

        ser.setTimeout(to);

        ser.open();
    }
    catch (const std::exception &e)
    {
        RCLCPP_FATAL(
            node->get_logger(),
            "Failed to open serial port: %s",
            e.what());

        return -1;
    }

    if (!ser.isOpen())
    {
        RCLCPP_FATAL(
            node->get_logger(),
            "Serial port not open");

        return -1;
    }

    RCLCPP_INFO(
        node->get_logger(),
        "Serial port opened.");

    // ==================================================
    // ROS2 订阅
    // ==================================================
    auto sub_camera =
        node->create_subscription<std_msgs::msg::String>(
            "/k4a/target_info",
            10,
            CameraCallback);

    rclcpp::spin(node);

    rclcpp::shutdown();

    if (ser.isOpen())
    {
        ser.close();
    }

    return 0;
}