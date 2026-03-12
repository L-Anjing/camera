#include <iostream>
#include <csignal>

#include "utils/block_recognizer.hpp"
#include "k4a/camera_k4a.hpp"
#include "utils/myinfer.hpp"
#include "utils/vision_draw.hpp"

#if defined(BUILD_WITH_ROS1)
#include <ros/ros.h>
#include <std_msgs/String.h>
#elif defined(BUILD_WITH_ROS2)
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#endif

volatile sig_atomic_t stop_flag = 0;

void sigintHandler(int)
{
    stop_flag = 1;
}

int main(
#if defined(BUILD_WITH_ROS1) || defined(BUILD_WITH_ROS2)
    int argc, char **argv
#endif
)
{
#if defined(BUILD_WITH_ROS1)
    ros::init(argc, argv, "k4a_detector");
    ros::NodeHandle nh;
    signal(SIGINT, sigintHandler);

    ros::Publisher target_pub =
        nh.advertise<std_msgs::String>("/k4a/target_info", 10);
#elif defined(BUILD_WITH_ROS2)
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("k4a_detector");
    signal(SIGINT, sigintHandler);

    auto target_pub =
        node->create_publisher<std_msgs::msg::String>("/k4a/target_info", 10);
#else
    std::cout << "[INFO] K4A Detector started (non-ROS mode)\n";
#endif

    try
    {
        // 初始化 - 使用绝对路径或从环境变量获取
        std::string config_path = "/home/li/camera_ws/src/camera_bridge/config/K4AConfig.yaml";
        K4a k4a_device = K4a::Create_FromFile(config_path);
        Yolo yolo;
        BlockRecognizer block_recognizer;

        std::string engine_path =
            "/home/li/camera_ws/src/camera_bridge/workspace/model_generate/"
            "yolo_dete_1_20/weights/best.engine";

        yolo.Yolov8_Enable(engine_path);

        // Set confidence threshold to 0.5 (50%) and NMS threshold to 0.5
        yolo.Set_Confidence_Threshold(0.5f, 0.5f);

        yolo::BoxArray detections;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZ>);
        // 点云可视化初始化
        // pcl::visualization::PCLVisualizer::Ptr viewer(
        //     new pcl::visualization::PCLVisualizer("PointCloud Viewer"));

        // viewer->setBackgroundColor(0, 0, 0);
        // viewer->addCoordinateSystem(0.2);

        // bool first_cloud = true;
        int frame_id = 0;

        while (!stop_flag)
        {
            cv::Mat color_image, depth_image;
            cv::Mat gray, gray3;

            k4a_device.Image_to_Cv(color_image, depth_image);
            if (color_image.empty() || depth_image.empty())
                continue;

            cv::cvtColor(color_image, gray, cv::COLOR_BGR2GRAY);
            cv::cvtColor(gray, gray3, cv::COLOR_GRAY2BGR);

            // YOLO 推理
            yolo.Single_Inference(gray3, detections);

            // YOLO Debug 显示
            cv::Mat yolo_vis = color_image.clone();
            vision::draw_yolo_detections(yolo_vis, detections);
            cv::imshow("YOLO Debug", yolo_vis);

            // Block 融合 - 返回所有检测到的blocks 结构体容器，包含多个结构体对象
            FinalBlockResults blocks_patterns = block_recognizer.recognize(detections);

            // 显示 Final Block 窗口
            cv::Mat block_vis = color_image.clone();

            // 处理所有检测到的blocks
            if (!blocks_patterns.empty())
            {
                for (const auto &results : blocks_patterns)
                {
                    // 绘制融合结果
                    vision::draw_block_result(block_vis, results);

                    // 3D 计算
                    BoundingBox3D bbox =
                        k4a_device.Value_Block_to_Pcl(cloud, depth_image, results);
                    const char *class_name = block_class_name(results.block_class);

                    if (frame_id++ % 5 == 0 )
                    {
                        std::cout << "Block Class: " << class_name
                                  << " Confidence: " << results.confidence
                                  << " Center: ["
                                  << bbox.center.x << ", "
                                  << bbox.center.y << ", "
                                  << bbox.center.z << ", "
                                  << bbox.principal_dir[0] << "]\n";
                    }
#if defined(BUILD_WITH_ROS1)

                    std_msgs::String msg;
                    std::stringstream ss;
                    ss << bbox.cls_ID << ","
                       << bbox.center.x << ","
                       << bbox.center.y << ","
                       << bbox.center.z << ","
                       << bbox.principal_dir[0]; // yaw
                    msg.data = ss.str();
                    target_pub.publish(msg);

#elif defined(BUILD_WITH_ROS2)
                          std_msgs::msg::String msg;
                          std::stringstream ss;
                          ss << bbox.cls_ID << ","
                              << bbox.center.x << ","
                              << bbox.center.y << ","
                              << bbox.center.z << ","
                              << bbox.principal_dir[0]; // yaw
                          msg.data = ss.str();
                          target_pub->publish(msg);

#endif

                    // // PCL 显示
                    // if (first_cloud)
                    // {
                    //     viewer->addPointCloud<pcl::PointXYZ>(cloud, "target_cloud");
                    //     viewer->resetCameraViewpoint("target_cloud");
                    //     first_cloud = false;
                    // }
                    // else
                    // {
                    //     viewer->updatePointCloud<pcl::PointXYZ>(cloud, "target_cloud");
                    // }
                }
            }
            else
            {
#if defined(BUILD_WITH_ROS1)
                // 当没有检测到任何 block 时，发布全为 0 的占位消息，方便下游更新状态
                std_msgs::String msg;
                std::stringstream ss;
                ss << 0 << ","   // cls_ID (UNKNOWN)
                   << 0 << ","   // center.x
                   << 0 << ","   // center.y
                   << 0 << ","   // center.z
                   << 0;          // yaw
                msg.data = ss.str();
                target_pub.publish(msg);
#elif defined(BUILD_WITH_ROS2)
                     std_msgs::msg::String msg;
                     std::stringstream ss;
                     ss << 0 << ","   // cls_ID (UNKNOWN)
                         << 0 << ","   // center.x
                         << 0 << ","   // center.y
                         << 0 << ","   // center.z
                         << 0;          // yaw
                     msg.data = ss.str();
                     target_pub->publish(msg);
#else
                // 非 ROS 编译时，打印一条全 0 的占位信息，保持行为一致
                if (frame_id++ % 5 == 0)
                {
                    std::cout << "Block Class: UNKNOWN"
                              << " Confidence: 0"
                              << " Center: [0, 0, 0, 0]\n";
                }
#endif
            }

            // cv::imshow("Final Block", block_vis);
            // cv::imshow("depth_iamge",depth_image);

            // viewer->spinOnce(10);

            char key = (char)cv::waitKey(10);
            if (key == 'q' || key == 27)
                break;

#if defined(BUILD_WITH_ROS1)
            ros::spinOnce();
#elif defined(BUILD_WITH_ROS2)
        rclcpp::spin_some(node);
#endif
        }

#ifdef BUILD_WITH_ROS2
    rclcpp::shutdown();
#endif

        return EXIT_SUCCESS;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[ERROR] " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
}
