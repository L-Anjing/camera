#include "realsense/realsense.hpp"
#include "utils/myinfer.hpp"
#include "utils/utils.hpp"

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <iostream>
#include <csignal>

#include <rclcpp/rclcpp.hpp>

volatile sig_atomic_t stop_flag = 0;

void sigintHandler(int)
{
    stop_flag = 1;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("rs_viewer");
    signal(SIGINT, sigintHandler);

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
            if (!rclcpp::ok()) break;

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

            rclcpp::spin_some(node);
        }

        rclcpp::shutdown();

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
