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
        std::string model_path = "/home/li/camera_cxx/workspace/model_temp/yolov8s.onnx";
        float conf_thres = 0.25f;
        float nms_thres = 0.45f;
        if (!yolo::load("/home/li/camera_cxx/workspace/model_temp/yolov8s.engine", yolo::Type::V8, conf_thres, nms_thres))
        {
            std::cerr << "Failed to load yolov8s.onnx." << std::endl;
            return EXIT_FAILURE;
        }
        // 初始化点云可视化
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("PointCloud Viewer"));
        viewer->setBackgroundColor(0, 0, 0);
        viewer->addCoordinateSystem(0.1);               // 显示坐标系
        viewer->initCameraParameters();

        // 设置点云的初始显示参数
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud"); // 设置点云点大小为3
        while (true)
        {
            // 2) 抓取一帧彩色与深度（仅用于显示；点云不依赖这一步）
            cv::Mat color, depth;
            myrealsense.Image_to_Cv(color, depth); // 注意：此函数内部做了对齐，仅用于可视化

            if (!color.empty())
                cv::imshow("Color", color);
            if (!depth.empty())
                cv::imshow("Depth (8-bit)", depth);

            yolo::BoxArray detections;
            yolo.Single_Inference(color, detections);
            myrealsense.Color_With_Mask(color, detections);

            // 3) 生成点云（函数内部会自己获取一帧深度，不受上一步对齐影响）
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            myrealsense.Value_Depth_to_Pcl(*cloud);

            if (!cloud->empty())
            {
                if (pcl::io::savePCDFileBinary("frame_cloud.pcd", *cloud) == 0)
                {
                    // 删除之前的点云
                    viewer->removePointCloud("cloud");

                    // 添加新的点云
                    viewer->addPointCloud(cloud, "cloud");
                    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
                }
                pcl::io::savePCDFileBinary("frame_cloud.pcd", *cloud);
                // std::cout << "Save point cloud:frame_cloud.pcd (" << cloud->size() << " points)" << std::endl;
            }
            else
            {
                std::cerr << "Point cloud is empty.\n";
            }

            // 4) 显示窗口等待一会儿

            char key = (char)cv::waitKey(1);
            if (key == 'q' || key == 27) // q or Esc
            {
                break;
            }
            viewer->spinOnce(10); // 刷新显示
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
