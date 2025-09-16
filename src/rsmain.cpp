#include "main.hpp"
#include "realsense.hpp"
#include "myinfer.hpp"

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>

int main()
{
    try
    {
        // 1) 初始化（默认：彩色+深度）
        RealSense myrealsense = RealSense::Create_Default();

        // 初始化Yolov8 TensorRT推理引擎
        Yolo yolo;

        std::string engine_path = "/home/li/camera_cxx/workspace/model_temp/yolov8s.engine"; // 引擎路径
        // yolo.Yolov8_Seg_Enable(engine_path);
        yolo.Yolov8_Enable(engine_path);  //检测引擎适用于yolov8s.engine
        yolo::BoxArray detections;

        while (true)
        {
            // 2) 抓取一帧彩色与深度（仅用于显示；点云不依赖这一步）
            cv::Mat color, depth;
            myrealsense.Image_to_Cv(color, depth); // 注意：此函数内部做了对齐，仅用于可视化

            yolo.Single_Inference(color, detections);
            myrealsense.Color_With_Mask(color, detections);

            if (!color.empty())
                cv::imshow("Color", color);
            if (!depth.empty())
                cv::imshow("Depth (8-bit)", depth);

            // 4) 显示窗口等待一会儿

            char key = (char)cv::waitKey(10);
            if (key == 'q' || key == 27) // q or Esc
            {
                break;
            }
        }
        return EXIT_SUCCESS;
    }
    catch (const rs2::error &e)
    {
        std::cerr << "RealSense error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Std exception: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
}
