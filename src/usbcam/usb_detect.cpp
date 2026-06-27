#include <iostream>
#include <csignal>
#include <chrono>
#include <iomanip>

#include "usbcam/usbcam.hpp"
#include "utils/block_recognizer.hpp"
#include "utils/kalman_tracker.hpp"
#include "utils/myinfer.hpp"
#include "utils/vision_draw.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

volatile sig_atomic_t stop_flag = 0;
void sigintHandler(int) { stop_flag = 1; }

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("usb_detector");
    signal(SIGINT, sigintHandler);

    // 使用与 K4A 相同的 topic, 以便 serial 节点复用
    auto target_pub =
        node->create_publisher<std_msgs::msg::String>("/target_info", 10);

    // ── 初始化 ──
    UsbCam cam;
    if (!cam.open("config/UsbConfig.yaml"))
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to open USB camera");
        return -1;
    }

    Yolo yolo;
    BlockRecognizer block_recognizer;
    TrackManager track_manager;

    std::string engine_path =
        "/home/pi/workspace/camera_ws/src/camera_bridge/workspace/models/260425.engine";
    yolo.Yolov8_Enable(engine_path);
    yolo.Set_Confidence_Threshold(0.5f, 0.5f);

    yolo::BoxArray detections;
    int frame_id = 0;
    auto prev_time = std::chrono::steady_clock::now();

    RCLCPP_INFO(node->get_logger(), "USB detector started");

    while (!stop_flag && rclcpp::ok())
    {
        auto frame_start = std::chrono::steady_clock::now();

        // ── 取帧 ──
        cv::Mat color_image;
        if (!cam.read(color_image) || color_image.empty())
        {
            std::cerr << "[UsbCam] read failed" << std::endl;
            continue;
        }

        // ── YOLO 推理（letterbox 640） ──
        detections.clear();
        yolo.Single_Inference_Letterbox(color_image, detections, 640);

        // ── Block 融合 ──
        FinalBlockResults blocks = block_recognizer.recognize(detections);

        // ── 可视化 ──
        cv::Mat vis = color_image.clone();
        vision::draw_yolo_detections(vis, detections);

        if (!blocks.empty())
        {
            for (const auto &result : blocks)
            {
                vision::draw_block_result(vis, result);

                // face 中心像素
                float u = (result.best_pattern.left + result.best_pattern.right) / 2.0f;
                float v = (result.best_pattern.top + result.best_pattern.bottom) / 2.0f;

                // 夹角 (弧度)
                double angle_z = cam.pixel_to_angle_z(u, v);

                // 卡尔曼滤波（在角度空间做平滑，传入 (angle, 0, 0) 作为观测）
                auto now = std::chrono::steady_clock::now();
                float dt = std::chrono::duration<float>(now - prev_time).count();
                prev_time = now;
                dt = std::min(dt, 0.5f);

                Eigen::Vector3f raw(angle_z, u, v);
                Eigen::Vector3f kf_out = track_manager.process(
                    raw, dt, static_cast<int>(result.block_class), result.confidence);

                double smooth_angle = kf_out.x();
                double smooth_u = kf_out.y();
                double smooth_v = kf_out.z();

                // 在图上标记角度
                cv::putText(vis,
                    cv::format("theta_z=%.1fdeg", smooth_angle * 180.0 / M_PI),
                    cv::Point(static_cast<int>(u) - 40, static_cast<int>(v) - 20),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);

                // ── 发布 (兼容 serial 节点: cls_ID, x, y, z, yaw, 无深度时 xyz=0) ──
                std_msgs::msg::String msg;
                std::stringstream ss;
                ss << static_cast<int>(result.block_class) << ","
                   << 0 << "," << 0 << "," << 0 << ","
                   << smooth_angle;
                msg.data = ss.str();
                target_pub->publish(msg);

                if (frame_id++ % 5 == 0)
                {
                    auto t = std::chrono::steady_clock::now();
                    float ms = std::chrono::duration<float>(t - frame_start).count();
                    std::cout << "Class: " << block_class_name(result.block_class)
                              << " u=" << smooth_u << " v=" << smooth_v
                              << " angle_z=" << smooth_angle << "rad"
                              << " Time: " << std::fixed << std::setprecision(3) << ms << "s\n";
                }
            }
        }
        else
        {
            // 无检测 → 发布零 (5字段, 兼容 serial)
            std_msgs::msg::String msg;
            msg.data = "0,0,0,0,0";
            target_pub->publish(msg);
        }

        cv::imshow("USB Detect", vis);
        char key = (char)cv::waitKey(10);
        if (key == 'q' || key == 27) break;

        rclcpp::spin_some(node);
    }

    rclcpp::shutdown();
    return 0;
}
