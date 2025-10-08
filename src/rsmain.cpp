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

        std::string engine_path = "/home/li/camera_cxx/workspace/model_temp/best.engine"; // 引擎路径
        yolo.Yolov8_Seg_Enable(engine_path);
        // yolo.Yolov8_Enable(engine_path);  //检测引擎适用于yolov8s.engine
        yolo::BoxArray detections;

        // PointCloud settings
        bool first_cloud = true;                                                       // 第一次生成点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); // 定义点云指针
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("PointCloud Viewer"));
        viewer->setBackgroundColor(0, 0, 0);
        viewer->addCoordinateSystem(0.2);

        while (true)
        {
            // 2) 抓取一帧彩色与深度（仅用于显示；点云不依赖这一步）
            cv::Mat color, depth;
            cv::Mat depth_display;

            myrealsense.Image_to_Cv(color, depth); // 注意：此函数内部做了对齐，仅用于可视化

            yolo.Single_Inference(color, detections);
            myrealsense.Color_With_Mask(color, detections);

            depth.convertTo(depth_display, CV_8U, 255.0 / 4000.0);
            myrealsense.Depth_With_Mask(depth_display, detections); // 用显示图，不用原始 CV_16U

            static int frame_id = 0;

            BoundingBox3D bbox = myrealsense.Value_Mask_to_Pcl(cloud, depth, detections);
            frame_id++;
            if (frame_id % 10 == 0)
            { // 每10帧打印一次
                std::cout << "BBox for class: " << bbox.cls_name << std::endl;
                std::cout << "  Min: [" << bbox.min_pt.x << ", " << bbox.min_pt.y << ", " << bbox.min_pt.z << "]" << std::endl;
                std::cout << "  Max: [" << bbox.max_pt.x << ", " << bbox.max_pt.y << ", " << bbox.max_pt.z << "]" << std::endl;
                std::cout << "  Center: [" << bbox.center.x << ", " << bbox.center.y << ", " << bbox.center.z << "]" << std::endl;
                std::cout << "  Principal dir: [" << bbox.principal_dir[0] << ", " << bbox.principal_dir[1] << ", " << bbox.principal_dir[2] << "]" << std::endl;
            }

            if (!color.empty())
                cv::imshow("Color", color);
            if (!depth.empty())
                cv::imshow("Depth (8-bit)", depth);

            //  更新 PCL 可视化器
            if (first_cloud)
            {
                viewer->addPointCloud<pcl::PointXYZ>(cloud, "target_cloud");
                viewer->resetCameraViewpoint("target_cloud"); // 自动对准目标点云
                first_cloud = false;
            }
            else
            {
                viewer->updatePointCloud<pcl::PointXYZ>(cloud, "target_cloud");
            }
            // 显示窗口等待一会儿
            viewer->spinOnce(10);

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
