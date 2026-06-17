#include <iostream>
#include <csignal>
#include <algorithm>

#include "utils/block_recognizer.hpp"
#include "k4a/camera_k4a.hpp"
#include "utils/myinfer.hpp"
#include "utils/vision_draw.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <Eigen/Dense>

volatile sig_atomic_t stop_flag = 0;

void sigintHandler(int)
{
    stop_flag = 1;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("k4a_detector");
    signal(SIGINT, sigintHandler);

    auto target_pub =
        node->create_publisher<std_msgs::msg::String>("/k4a/target_info", 10);

    try
    {
        // 初始化 - 使用绝对路径或从环境变量获取
        std::string config_path = "/home/pi/workspace/camera_ws/src/camera_bridge/config/K4AConfig.yaml";
        K4a k4a_device = K4a::Create_FromFile(config_path);
        Yolo yolo;
        BlockRecognizer block_recognizer;

        std::string engine_path =
            "/home/pi/workspace/camera_ws/src/camera_bridge/workspace/models/260425.engine";

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

        int frame_id = 0;

        // ── 中值滤波（窗口=5）+ 离群抑制 ───────────────
        struct {
            float x[5], y[5], z[5];
            int n = 0, idx = 0;
        } med;
        constexpr float OUTLIER_DIST = 0.10f;     // 10cm 离群阈值

        while (!stop_flag)
        {
            cv::Mat color_image, depth_image;
            cv::Mat gray, gray3;

            k4a_device.Image_to_Cv(color_image, depth_image);
            if (color_image.empty() || depth_image.empty())
            {
                std::cerr << "get_capture timeout" << std::endl;
                continue;
            }
               

            cv::cvtColor(color_image, gray, cv::COLOR_BGR2GRAY);
            cv::cvtColor(gray, gray3, cv::COLOR_GRAY2BGR);

            // YOLO 推理
            detections.clear();
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
                for (auto &results : blocks_patterns)
                {
                    // 绘制融合结果
                    vision::draw_block_result(block_vis, results);

                    // 3D 计算（始终用 best_pattern — 几何最正面）
                    BoundingBox3D bbox =
                        k4a_device.Value_Block_to_Pcl(cloud, depth_image, results);
                    const char *class_name = block_class_name(results.block_class);

                    // ── 面选择：PCA 拟合面法向量，选最正对机器人的面 ──
                    // 预期方向 = 物体指向机器人原点 = -center_robot.normalized()
                    if (results.candidates.size() > 1)
                    {
                        const Eigen::Matrix3f &R = k4a_device.get_rotation();
                        Eigen::Vector3f dir_to_robot = -Eigen::Vector3f(
                            bbox.center.x, bbox.center.y, bbox.center.z);
                        dir_to_robot.normalize();

                        int best_label = results.detection.class_label;
                        float best_dot = -1.0f;

                        for (const auto &c : results.candidates)
                        {
                            Eigen::Vector3f n_cam = k4a_device.compute_roi_normal(
                                depth_image, c.box);
                            Eigen::Vector3f n_robot = R * n_cam;     // 转到机器人系
                            float d = std::abs(n_robot.dot(dir_to_robot));
                            if (d > best_dot)
                            {
                                best_dot = d;
                                best_label = c.class_label;
                            }
                        }

                        if (best_label != results.detection.class_label)
                        {
                            bbox.cls_ID = best_label;
                            bbox.cls_name = "NORMAL";
                        }
                    }

                    // ── 中值滤波（窗口=5）+ 离群抑制 ─────────
                    cv::Point3f out_center = bbox.center;
                    if (bbox.center.x == 0.0f && bbox.center.y == 0.0f && bbox.center.z == 0.0f)
                    {
                        med.n = 0;   // 零值 → 重置
                    }
                    else
                    {
                        // 离群检测（有历史时）
                        if (med.n >= 3)
                        {
                            int prev = (med.idx - 1 + 5) % 5;
                            float dx = bbox.center.x - med.x[prev];
                            float dy = bbox.center.y - med.y[prev];
                            float dz = bbox.center.z - med.z[prev];
                            if (std::sqrt(dx*dx + dy*dy + dz*dz) > OUTLIER_DIST)
                            {
                                goto skip_push;
                            }
                        }
                        // 入队
                        med.x[med.idx] = bbox.center.x;
                        med.y[med.idx] = bbox.center.y;
                        med.z[med.idx] = bbox.center.z;
                        med.idx = (med.idx + 1) % 5;
                        if (med.n < 5) ++med.n;

                        // 取中值（窗口≥3 时开始输出）
                        if (med.n >= 3)
                        {
                            float cx[5], cy[5], cz[5];
                            std::copy_n(med.x, med.n, cx);
                            std::copy_n(med.y, med.n, cy);
                            std::copy_n(med.z, med.n, cz);
                            std::sort(cx, cx + med.n);
                            std::sort(cy, cy + med.n);
                            std::sort(cz, cz + med.n);
                            out_center.x = cx[med.n / 2];
                            out_center.y = cy[med.n / 2];
                            out_center.z = cz[med.n / 2];
                        }
                    }
                    skip_push:
                    bbox.center = out_center;
                    bbox.principal_dir[0] = std::atan2(bbox.center.y, bbox.center.x);

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

                    std_msgs::msg::String msg;
                    std::stringstream ss;
                    ss << bbox.cls_ID << ","
                       << bbox.center.x << ","
                       << bbox.center.y << ","
                       << bbox.center.z << ","
                       << bbox.principal_dir[0]; // yaw
                    msg.data = ss.str();
                    target_pub->publish(msg);

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
                std_msgs::msg::String msg;
                std::stringstream ss;
                ss << 0 << ","   // cls_ID (UNKNOWN)
                   << 0 << ","   // center.x
                   << 0 << ","   // center.y
                   << 0 << ","   // center.z
                   << 0;          // yaw
                msg.data = ss.str();
                target_pub->publish(msg);
            }

            cv::imshow("Final Block", block_vis);
            // cv::imshow("depth_iamge",depth_image);

            // viewer->spinOnce(10);

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
        std::cerr << "[ERROR] " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
}
