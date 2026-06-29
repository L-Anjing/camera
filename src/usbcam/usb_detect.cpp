#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstdint>
#include <optional>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "common/yolo.hpp"
#include "usbcam/usbcam.hpp"
#include "utils/block_recognizer.hpp"
#include "utils/image_preprocess.hpp"
#include "utils/myinfer.hpp"
#include "utils/vision_draw.hpp"

namespace
{
volatile sig_atomic_t g_stop_flag = 0;

void sigintHandler(int)
{
    g_stop_flag = 1;
}
} // namespace

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("usb_detector");
    auto target_pub = node->create_publisher<std_msgs::msg::String>("/target_info", 10);

    signal(SIGINT, sigintHandler);

    RCLCPP_INFO(node->get_logger(), "Inference input: fixed grayscale");

    try
    {
        // 固定配置，模型路径按需手动替换
        const std::string config_path = "/home/li/workspace/src/camera_bridge/config/UsbRosConfig.yaml";
        std::string engine_path = "/home/li/workspace/src/camera_bridge/workspace/models/260629.engine";
        constexpr int kInputSize = 640;
        constexpr float kConfidence = 0.5f;
        constexpr float kNms = 0.5f;
        constexpr float kFaceSizeM = 0.05f;

        UsbCam cam;
        if (!cam.open(config_path, std::nullopt, node))
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to open USB camera");
            rclcpp::shutdown();
            return EXIT_FAILURE;
        }

        if (cam.uses_ros_topic())
        {
            RCLCPP_INFO(node->get_logger(), "Camera source: ROS topic %s", cam.ros_topic().c_str());
        }
        else
        {
            RCLCPP_INFO(node->get_logger(), "Camera source: /dev/video%d", cam.device_id());
        }

        Yolo yolo;
        yolo.Yolov8_Enable(engine_path);
        yolo.Set_Confidence_Threshold(kConfidence, kNms);

        BlockRecognizer block_recognizer;
        yolo::BoxArray detections;
        cv::namedWindow("USB Detect", cv::WINDOW_NORMAL);
        cv::resizeWindow("USB Detect", 1280, 800);

        int frame_id = 0;
        auto fps_tick = std::chrono::steady_clock::now();
        int fps_frames = 0;

        while (!g_stop_flag && rclcpp::ok())
        {
            const auto loop_start = std::chrono::steady_clock::now();

            // 取一帧相机数据
            cv::Mat frame;
            if (!cam.read(frame) || frame.empty())
            {
                rclcpp::spin_some(node);
                continue;
            }

            // 推理前统一做固定输入预处理
            cv::Mat infer_image = vision::build_infer_input(frame);

            // YOLO 检测
            detections.clear();
            yolo.Single_Inference_Letterbox(infer_image, detections, kInputSize);

            // 融合 block 和 face，得到最终候选
            FinalBlockResults blocks = block_recognizer.recognize(detections);
            // 选出最适合定位的目标
            TargetSelection best_target = select_best_target(cam, blocks, kFaceSizeM);

            // 叠加检测结果和位姿信息
            cv::Mat vis = frame.clone();
            vision::draw_yolo_detections(vis, detections);
            for (const auto &block : blocks)
            {
                vision::draw_block_result(vis, block);
            }

            if (best_target.valid())
            {
                vision::draw_target_overlay(vis, *best_target.result, best_target.pose);

                // 发布给下游串口/机器人节点
                std_msgs::msg::String msg;
                std::ostringstream ss;
                ss << static_cast<int>(best_target.result->block_class) << ","
                   << best_target.pose.robot_center.x() << ","
                   << best_target.pose.robot_center.y() << ","
                   << best_target.pose.robot_center.z() << ","
                   << best_target.pose.yaw_z;
                msg.data = ss.str();
                target_pub->publish(msg);

                if (frame_id % 5 == 0)
                {
                    const char *class_name = block_class_name(best_target.result->block_class);
                    std::cout << "[USB_Detect] Class: " << class_name
                              << " conf=" << best_target.result->confidence
                              << " cam=(" << std::fixed << std::setprecision(3)
                              << best_target.pose.camera_center.x() << ", "
                              << best_target.pose.camera_center.y() << ", "
                              << best_target.pose.camera_center.z() << ")"
                              << " robot=("
                              << best_target.pose.robot_center.x() << ", "
                              << best_target.pose.robot_center.y() << ", "
                              << best_target.pose.robot_center.z() << ")"
                              << " yaw=" << best_target.pose.yaw_z
                              << std::endl;
                }
            }
            else
            {
                std_msgs::msg::String msg;
                msg.data = "0,0,0,0,0";
                target_pub->publish(msg);
                // 没有可用目标时给出提示
                cv::putText(vis,
                            "No valid block target",
                            cv::Point(20, 40),
                            cv::FONT_HERSHEY_SIMPLEX,
                            0.9,
                            cv::Scalar(0, 0, 255),
                            2);
            }

            ++frame_id;
            ++fps_frames;
            const auto now = std::chrono::steady_clock::now();
            const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - fps_tick).count();
            if (elapsed_ms >= 1000)
            {
                const float fps = fps_frames * 1000.0f / std::max<int64_t>(1, elapsed_ms);
                cv::putText(vis,
                            cv::format("FPS: %.1f", fps),
                            cv::Point(20, 80),
                            cv::FONT_HERSHEY_SIMPLEX,
                            0.9,
                            cv::Scalar(0, 255, 0),
                            2);
                std::cout << "[USB_Detect] FPS: " << fps
                          << " dets=" << detections.size()
                          << " blocks=" << blocks.size()
                          << std::endl;
                fps_frames = 0;
                fps_tick = now;
            }

            const auto loop_end = std::chrono::steady_clock::now();
            const auto loop_ms = std::chrono::duration_cast<std::chrono::milliseconds>(loop_end - loop_start).count();
            cv::putText(vis,
                        cv::format("Loop: %ld ms", static_cast<long>(loop_ms)),
                        cv::Point(20, 115),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.7,
                        cv::Scalar(200, 200, 0),
                        2);

            cv::imshow("USB Detect", vis);
            const char key = static_cast<char>(cv::waitKey(1));
            if (key == 'q' || key == 27)
            {
                break;
            }

            rclcpp::spin_some(node);
        }

        rclcpp::shutdown();
        return EXIT_SUCCESS;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[USB_Detect] ERROR: " << e.what() << std::endl;
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }
}
