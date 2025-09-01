#include "camera_k4a.hpp"
#include <opencv2/opencv.hpp>
#include <main.hpp>
#include <iostream>
#include "myinfer.hpp"

int main()
{
    try
    {
        K4a k4a_device; // 类里是构造函数，自动调用configuration函数
        // k4a_device.Configuration(); //不要有这一句，会报错
        Yolo yolo;                                                                           // 实例化类
        std::string engine_path = "/home/li/camera_cxx/workspace/model_temp/best.engine"; // 引擎路径
                                                                                                                              

        yolo::BoxArray detections;

        bool first_cloud = true;                                                       // 第一次生成点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); // 定义点云指针
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("PointCloud Viewer"));
        viewer->setBackgroundColor(0, 0, 0);
        viewer->addCoordinateSystem(0.1);
        viewer->addArrow(0, 0, 0, 0.5, 0, 0, 1, 0.01, "x");
        viewer->addArrow(0, 0, 0, 0, 0.5, 0, 0, 0.01, "y");
        viewer->addArrow(0, 0, 0, 0, 0, 0.5, 0, 0.01, "z");

        while (true)
        {
            cv::Mat color_image, depth_image;
            k4a_device.Image_to_Cv(color_image, depth_image); // 获取彩色和深度图像

            yolo.Yolov8_Seg_Enable(engine_path);
            yolo.Single_Inference(color_image, detections);

            k4a_device.Color_With_Mask(color_image, detections); // 显示带有检测结果的彩色图像
            k4a_device.Depth_With_Mask(depth_image, detections); // 显示带有检测结果的深度图像

            if (!color_image.empty())
                cv::imshow(" Seg Color Image", color_image); // 显示彩色图像
            if (!depth_image.empty())
                cv::imshow("Depth Image", depth_image); // 显示深度图像

            k4a_device.Value_Depth_to_Pcl(*cloud); // 生成点云
            std::cout << "Generated PointCloud with " << cloud->size() << " points." << std::endl;
            if (!cloud->empty())
            {
                if (first_cloud)
                {
                    viewer->addPointCloud(cloud, "cloud");
                    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
                    first_cloud = false;
                }
                else
                {
                    // 删除之前的点云
                    viewer->updatePointCloud(cloud, "cloud");
                }
            }
            else
            {
                std::cerr << "Point cloud is empty.\n";
            }

            char key = (char)cv::waitKey(1);
            if (key == 'q' || key == 27) // 按 q 或 Esc 键退出
                break;
            viewer->spinOnce(1); // 更新显示
        }
        return EXIT_SUCCESS;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Exception: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
}
