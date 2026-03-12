#include "realsense/realsense.hpp"
#include "utils/myinfer.hpp"
#include "utils/utils.hpp"

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <iostream>
#include <csignal>

#if defined(BUILD_WITH_ROS1)
#include <ros/ros.h>
#elif defined(BUILD_WITH_ROS2)
#include <rclcpp/rclcpp.hpp>
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
    ros::init(argc, argv, "rs_viewer_ros");
    ros::NodeHandle nh;
    signal(SIGINT, sigintHandler);
#elif defined(BUILD_WITH_ROS2)
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("rs_viewer_ros");
    signal(SIGINT, sigintHandler);
#else
    std::cout << "[INFO] RealSense viewer started (non-ROS mode)" << std::endl;
#endif

    try
    {
        // 1) 初始化 (default：彩色+深度）
        RealSense myrealsense = RealSense::Create_FromFile("config/RsConfig.yaml");

        // 初始化Yolov8
        Yolo yolo;

        std::string engine_path = "workspace/model_generate/yolo_dete_1_20/weights/best.engine"; // 引擎路径
        yolo.Yolov8_Seg_Enable(engine_path);
        // yolo.Yolov8_Enable(engine_path);  //检测引擎
        yolo::BoxArray detections;


        while (!stop_flag)
        {
    #if defined(BUILD_WITH_ROS1)
            if (!ros::ok()) break;
    #elif defined(BUILD_WITH_ROS2)
            if (!rclcpp::ok()) break;
    #endif

            cv::Mat color_image, depth_image;
            cv::Mat depth_display;

            myrealsense.Image_to_Cv(color_image, depth_image); // 注意：此函数内部做了对齐，仅用于可视化

            yolo.Single_Inference(color_image, detections);
            vision::draw_yolo_detections(color_image, detections);

            vision::draw_yolo_masks(color_image, detections);

            if (!color_image.empty())
                cv::imshow("Color", color_image);
            if (!depth_image.empty())
                cv::imshow("Depth", depth_image);

            char key = (char)cv::waitKey(10);
            if (key == 'q' || key == 27) // q or Esc
            {
                break;
            }

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
    catch (const rs2::error &e)
    {
        std::cerr << "RealSense error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    //捕获其他标准异常
    catch (const std::exception &e)
    {
        std::cerr << "Std exception: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
}
