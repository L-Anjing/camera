#include <serial/serial.h>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <cstring>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_rx_bag_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    ros::Publisher pub_hex = nh.advertise<std_msgs::String>("/serial/rx_hex", 100);
    ros::Publisher pub_omega = nh.advertise<std_msgs::Float32MultiArray>("/serial/omega", 100);

    std::string port;
    int baud = 115200;
    int max_read_bytes = 256;
    int idle_sleep_ms = 5;

    pnh.param<std::string>("port", port, "/dev/ttyUSB0");
    pnh.param<int>("baudrate", baud, 115200);
    pnh.param<int>("max_read_bytes", max_read_bytes, 256);
    pnh.param<int>("idle_sleep_ms", idle_sleep_ms, 5);

    if (max_read_bytes <= 0) {
        max_read_bytes = 256;
    }
    if (idle_sleep_ms < 0) {
        idle_sleep_ms = 5;
    }

    constexpr uint8_t FRAME_HEAD_0 = 0xAA;
    constexpr uint8_t FRAME_HEAD_1 = 0xDD;
    constexpr uint8_t FRAME_TAIL_0 = 0xFF;
    constexpr uint8_t FRAME_TAIL_1 = 0xFE;
    constexpr size_t PAYLOAD_SIZE = 8;   // omega1 + omega2 (float + float)
    constexpr size_t FRAME_SIZE = 12;    // AA DD + 8B + FF FE

    std::vector<uint8_t> rx_buffer;
    rx_buffer.reserve(4096);

    serial::Serial ser;
    try {
        ser.setPort(port);
        ser.setBaudrate(static_cast<uint32_t>(baud));
        serial::Timeout to(serial::Timeout::max(), 1000, 0, 1000, 0);
        ser.setTimeout(to);
        ser.open();
    } catch (const std::exception &e) {
        ROS_FATAL_STREAM("[serial_rx_bag_node] Failed to open serial port: " << e.what());
        return -1;
    }

    if (!ser.isOpen()) {
        ROS_FATAL("[serial_rx_bag_node] Serial port not open after open()");
        return -1;
    }

    ROS_INFO_STREAM("[serial_rx_bag_node] Serial opened: " << port << " @ " << baud);
    ros::Rate spin_rate(500);
    while (ros::ok()) {
        size_t available_bytes = 0;
        try {
            available_bytes = ser.available();
        } catch (const std::exception &) {
            available_bytes = 0;
        }

        if (available_bytes > 0) {
            size_t to_read = available_bytes;
            if (to_read > static_cast<size_t>(max_read_bytes)) {
                to_read = static_cast<size_t>(max_read_bytes);
            }

            std::string data;
            try {
                data = ser.read(to_read);
            } catch (const std::exception &) {
                data.clear();
            }

            if (!data.empty()) {
                rx_buffer.insert(rx_buffer.end(), data.begin(), data.end());

                while (rx_buffer.size() >= FRAME_SIZE) {
                    size_t head_pos = 0;
                    bool found_head = false;
                    while (head_pos + 1 < rx_buffer.size()) {
                        if (rx_buffer[head_pos] == FRAME_HEAD_0 && rx_buffer[head_pos + 1] == FRAME_HEAD_1) {
                            found_head = true;
                            break;
                        }
                        ++head_pos;
                    }

                    if (!found_head) {
                        rx_buffer.clear();
                        break;
                    }

                    if (head_pos > 0) {
                        rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + static_cast<long>(head_pos));
                    }

                    if (rx_buffer.size() < FRAME_SIZE) {
                        break;
                    }

                    if (rx_buffer[10] != FRAME_TAIL_0 || rx_buffer[11] != FRAME_TAIL_1) {
                        rx_buffer.erase(rx_buffer.begin());
                        continue;
                    }

                    float omega1 = 0.0f;
                    float omega2 = 0.0f;
                    std::memcpy(&omega1, &rx_buffer[2], sizeof(float));
                    std::memcpy(&omega2, &rx_buffer[6], sizeof(float));

                    std::stringstream hex_ss;
                    hex_ss << std::uppercase << std::hex;
                    for (size_t i = 0; i < FRAME_SIZE; ++i) {
                        hex_ss << std::setw(2) << std::setfill('0') << static_cast<int>(rx_buffer[i]) << ' ';
                    }

                    std_msgs::String hex_out;
                    hex_out.data = hex_ss.str();
                    pub_hex.publish(hex_out);

                    std_msgs::Float32MultiArray omega_out;
                    omega_out.data = {omega1, omega2};
                    pub_omega.publish(omega_out);

                    ROS_INFO_STREAM("[serial_rx_bag_node] omega1=" << omega1 << ", omega2=" << omega2
                                    << " | HEX: " << hex_out.data);

                    rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + static_cast<long>(FRAME_SIZE));
                }
            }
        } else if (idle_sleep_ms > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(idle_sleep_ms));
        }

        ros::spinOnce();
        spin_rate.sleep();
    }

    if (ser.isOpen()) {
        ser.close();
    }

    return 0;
}
