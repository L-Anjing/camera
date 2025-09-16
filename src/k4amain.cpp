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
        Yolo yolo;                                                                        // 实例化类
        std::string engine_path = "/home/li/camera_cxx/workspace/model_temp/best.engine"; // 引擎路径

        yolo::BoxArray detections;

        bool first_cloud = true;                                                       // 第一次生成点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); // 定义点云指针
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("PointCloud Viewer"));
        viewer->setBackgroundColor(0, 0, 0);
        viewer->addCoordinateSystem(0.05);
        // yolo.Yolov8_Enable(engine_path);  //检测引擎适用于yolov8s.engine
        yolo.Yolov8_Seg_Enable(engine_path); // 分割引擎适用于best.engine

        while (true)
        {
            cv::Mat color_image, depth_image;
            k4a_device.Image_to_Cv(color_image, depth_image); // 获取彩色和深度图像

            yolo.Single_Inference(color_image, detections);

            k4a_device.Color_With_Mask(color_image, detections); // 显示带有检测结果的彩色图像
            k4a_device.Depth_With_Mask(depth_image, detections); // 显示带有检测结果的深度图像
            static int frame_id = 0;

            BoundingBox3D bbox = k4a_device.Value_Mask_to_Pcl(cloud, depth_image, detections);
            frame_id++;
            if (frame_id % 10 == 0)
            { // 每10帧打印一次
                std::cout << "BBox for class: " << bbox.cls_name << std::endl;
                std::cout << "  Min: [" << bbox.min_pt.x << ", " << bbox.min_pt.y << ", " << bbox.min_pt.z << "]" << std::endl;
                std::cout << "  Max: [" << bbox.max_pt.x << ", " << bbox.max_pt.y << ", " << bbox.max_pt.z << "]" << std::endl;
                std::cout << "  Center: [" << bbox.center.x << ", " << bbox.center.y << ", " << bbox.center.z << "]" << std::endl;
                std::cout << "  Principal dir: [" << bbox.principal_dir[0] << ", " << bbox.principal_dir[1] << ", " << bbox.principal_dir[2] << "]" << std::endl;
            }

            if (!color_image.empty())
                cv::imshow(" Seg Color Image", color_image); // 显示彩色图像
            if (!depth_image.empty())
            {
                cv::Mat depth_display;
                depth_image.convertTo(depth_display, CV_8U, 255.0 / 4000.0); // 假设最大 4m
                cv::imshow("Depth Image", depth_display);
            }

            // 3. 更新 PCL 可视化器
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
