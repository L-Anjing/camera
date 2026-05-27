#include <iostream>
#include <csignal>

#include "utils/block_recognizer.hpp"
#include "insight9/camera_insight9.hpp"
#include "utils/myinfer.hpp"
#include "utils/vision_draw.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

volatile sig_atomic_t stop_flag = 0;

void sigintHandler(int)
{
    stop_flag = 1;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("insight9_detector");
    signal(SIGINT, sigintHandler);

    auto target_pub =
        node->create_publisher<std_msgs::msg::String>("/insight9/target_info", 10);

    try
    {
        // 初始化 - 使用绝对路径或从环境变量获取
        std::string config_path = "/home/pi/workspace/camera_ws/src/camera_bridge/config/Insight9Config.yaml";
        
        // 创建Insight9相机对象
        Insight9 insight9_device = Insight9::Create_FromFile(config_path, node);

        Yolo yolo;
        BlockRecognizer block_recognizer;

        std::string engine_path =
            "/home/pi/workspace/camera_ws/src/camera_bridge/workspace/models/260425.engine";

        yolo.Yolov8_Enable(engine_path);

        // 设置置信度阈值为 0.5 (50%) 和 NMS 阈值为 0.5
        yolo.Set_Confidence_Threshold(0.5f, 0.5f);

        yolo::BoxArray detections;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZ>);

        int frame_id = 0;
        int timeout_count = 0;
        const int max_timeout = 100;  // 连续超时次数的最大限制

        std::cout << "[INFO] Insight9 Detector ready. Waiting for ROS2 image topics...\n";

        while (!stop_flag)
        {
            cv::Mat color_image, depth_image;
            cv::Mat gray, gray3;

            // 尝试获取图像，超时则continue
            if (!insight9_device.Image_to_Cv(color_image, depth_image))
            {
                timeout_count++;
                if (timeout_count % 10 == 0)
                {
                    COUT_YELLOW_START;
                    std::cout << "[WARNING] Waiting for image data... (" << timeout_count << ")" << std::endl;
                    COUT_COLOR_END;
                }
                
                if (timeout_count > max_timeout)
                {
                    COUT_RED_START;
                    std::cerr << "[ERROR] No image data received after " << max_timeout << " attempts." << std::endl;
                    COUT_COLOR_END;
                    break;
                }

                rclcpp::spin_some(node);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            timeout_count = 0;  // 重置计数器

            if (color_image.empty() || depth_image.empty())
            {
                COUT_YELLOW_START;
                std::cerr << "[WARNING] Empty image received" << std::endl;
                COUT_COLOR_END;
                continue;
            }

            // 转换为灰度图用于YOLO推理
            cv::cvtColor(color_image, gray, cv::COLOR_BGR2GRAY);
            cv::cvtColor(gray, gray3, cv::COLOR_GRAY2BGR);

            // YOLO 推理
            yolo.Single_Inference(gray3, detections);

            // Block 融合 - 返回所有检测到的blocks结构体容器
            FinalBlockResults blocks_patterns = block_recognizer.recognize(detections);

            // 处理所有检测到的blocks
            if (!blocks_patterns.empty())
            {
                for (const auto &results : blocks_patterns)
                {
                    // 3D 计算
                    BoundingBox3D bbox =
                        insight9_device.Value_Block_to_Pcl(cloud, depth_image, results);
                    const char *class_name = block_class_name(results.block_class);

                    std_msgs::msg::String msg;
                    std::stringstream ss;
                    ss << bbox.cls_ID << ","
                        << bbox.center.x << ","
                        << bbox.center.y << ","
                        << bbox.center.z << ","
                        << bbox.principal_dir[0];  // yaw
                    msg.data = ss.str();
                    target_pub->publish(msg);

                    // 可选：打印调试信息（每5帧打印一次）
                    if (frame_id % 5 == 0)
                    {
                        COUT_GREEN_START;
                        std::cout << "Block Class: " << class_name
                                  << " | Confidence: " << results.confidence
                                  << " | Center: ["
                                  << bbox.center.x << ", "
                                  << bbox.center.y << ", "
                                  << bbox.center.z << "]"
                                  << " | Yaw: " << bbox.principal_dir[0] << std::endl;
                        COUT_COLOR_END;
                    }
                }
            }
            else
            {
                // 当没有检测到任何 block 时，发布占位消息
                std_msgs::msg::String msg;
                std::stringstream ss;
                ss << 0 << ","   // cls_ID (UNKNOWN)
                    << 0 << ","   // center.x
                    << 0 << ","   // center.y
                    << 0 << ","   // center.z
                    << 0;         // yaw
                msg.data = ss.str();
                target_pub->publish(msg);
            }

            frame_id++;

            // 按键控制
            char key = (char)cv::waitKey(10);
            if (key == 'q' || key == 27)
                break;

            rclcpp::spin_some(node);
        }

        rclcpp::shutdown();

        return EXIT_SUCCESS;
    }
    catch (const std::exception &e)
    {
        COUT_RED_START;
        std::cerr << "[ERROR] " << e.what() << std::endl;
        COUT_COLOR_END;
        return EXIT_FAILURE;
    }
}
