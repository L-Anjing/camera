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
        // 调试显示开关（主线程安全）
        const bool ENABLE_DEBUG_DISPLAY = true;
        const bool ENABLE_PCL_VIEWER = false;

        // 初始化 - 使用绝对路径或从环境变量获取
        std::string config_path = "/home/pi/workspace/camera_ws/src/camera_bridge/config/Insight9Config.yaml";
        
        // 创建Insight9相机对象
        auto insight9_device = Insight9::Create_FromFile(config_path, node);

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
        
        // PCL 点云显示初始化
        bool first_cloud = true;
        pcl::visualization::PCLVisualizer::Ptr viewer;
        if (ENABLE_PCL_VIEWER)
        {
            viewer = std::make_shared<pcl::visualization::PCLVisualizer>("Insight9 Point Cloud");
            viewer->setBackgroundColor(0, 0, 0);
            viewer->addCoordinateSystem(0.2);
        }

        int frame_id = 0;
        int timeout_count = 0;
        const int max_timeout = 100;  // 连续超时次数的最大限制

        std::cout << "[INFO] Insight9 Detector ready. Waiting for ROS2 image topics...\n";

        while (!stop_flag)
        {
            cv::Mat color_image, depth_image;
            cv::Mat gray, gray3;

            // 尝试获取图像，超时则continue
            if (!insight9_device->Image_to_Cv(color_image, depth_image))
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

            // YOLO Debug 显示（主线程安全）
            if (ENABLE_DEBUG_DISPLAY)
            {
                cv::Mat yolo_vis = color_image.clone();
                vision::draw_yolo_detections(yolo_vis, detections);
                cv::imshow("YOLO Detection", yolo_vis);
            }

            // Block 融合
            FinalBlockResults blocks_patterns = block_recognizer.recognize(detections);

            // Block 结果显示初始化
            cv::Mat block_vis;
            if (ENABLE_DEBUG_DISPLAY)
            {
                block_vis = color_image.clone();
            }

            // 处理所有检测到的blocks
            if (!blocks_patterns.empty())
            {
                for (const auto &results : blocks_patterns)
                {
                    // 绘制融合结果
                    if (ENABLE_DEBUG_DISPLAY)
                    {
                        vision::draw_block_result(block_vis, results);
                    }

                    // 3D 计算
                    BoundingBox3D bbox =
                        insight9_device->Value_Block_to_Pcl(cloud, depth_image, results);
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

                    // 调试输出（每5帧）
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

                    // PCL 点云显示
                    if (ENABLE_PCL_VIEWER && viewer)
                    {
                        if (first_cloud)
                        {
                            viewer->addPointCloud<pcl::PointXYZ>(cloud, "target_cloud");
                            viewer->resetCameraViewpoint("target_cloud");
                            first_cloud = false;
                        }
                        else
                        {
                            viewer->removePointCloud("target_cloud");
                            viewer->addPointCloud<pcl::PointXYZ>(cloud, "target_cloud");
                        }
                    }
                }
            }
            else
            {
                // 无检测时发布占位消息
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

            // OpenCV 调试窗口显示
            if (ENABLE_DEBUG_DISPLAY)
            {
                cv::Mat depth_display;
                cv::normalize(depth_image, depth_display, 0, 255, cv::NORM_MINMAX, CV_8UC1);
                
                cv::imshow("Insight9 Depth", depth_display);
                if (!block_vis.empty())
                {
                    cv::imshow("Block Detection", block_vis);
                }
            }
            
            // PCL 点云主线程更新
            if (ENABLE_PCL_VIEWER && viewer)
            {
                viewer->spinOnce(1);
                if (viewer->wasStopped())
                {
                    break;
                }
            }

            frame_id++;

            // 按键处理
            if (ENABLE_DEBUG_DISPLAY)
            {
                int key = cv::waitKey(10);
                if (key == 'q' || key == 27)
                {
                    break;
                }
            }
            else
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

            rclcpp::spin_some(node);
        }

        // 清理 GUI 资源
        if (ENABLE_DEBUG_DISPLAY)
        {
            cv::destroyAllWindows();
        }
        
        if (ENABLE_PCL_VIEWER && viewer)
        {
            viewer->close();
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
