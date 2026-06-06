#include <iostream>
#include <csignal>

#include "utils/block_recognizer.hpp"
#include "k4a/camera_k4a.hpp"
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

            // ── 零值溯源 ────────────────────────────────
            if (detections.empty())
            {
                std::cout << "[ZERO] YOLO 未检测到任何目标（置信度全部 < 0.5）\n";
            }

            // YOLO Debug 显示
            cv::Mat yolo_vis = color_image.clone();
            vision::draw_yolo_detections(yolo_vis, detections);
            cv::imshow("YOLO Debug", yolo_vis);

            // Block 融合 - 返回所有检测到的blocks 结构体容器，包含多个结构体对象
            FinalBlockResults blocks_patterns = block_recognizer.recognize(detections);

            // ── 零值溯源 ────────────────────────────────
            // blocks_patterns.empty() 的可能原因（按概率排序）：
            //   ① detections 中无 class_label==0（全是 face 或无任何框）
            //   ② 有 block 但无 face 的 IoF > 0.7（定位偏差大）
            //   ③ 有关联 face 但置信度 < 0.60（block_recognizer 内硬阈值过滤）
            //   ④ 有关联 face 但分类后 block_class 仍为 UNKNOWN
            if (!detections.empty() && blocks_patterns.empty())
            {
                std::cout << "[ZERO] YOLO 检测到 " << detections.size()
                          << " 个目标，但 recognize 融合后无有效 block"
                          << "（无 block/无关联 face/face 置信度不足/分类失败）\n";
            }

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

                    // ── 零值溯源 ────────────────────────────────
                    if (bbox.center.x == 0.0f && bbox.center.y == 0.0f && bbox.center.z == 0.0f)
                    {
                        std::cout << "[ZERO] block_class=" << class_name
                                  << " recognize 成功但 3D 中心为 (0,0,0)"
                                  << "（ROI 内深度点全部无效或被距离过滤）\n";
                    }

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
                // ── 零值溯源 ────────────────────────────────
                // 发布全零占位消息 (0,0,0,0,0)，通知下游无有效目标。
                // 上游日志中会输出具体原因：
                //   "YOLO 未检测到任何目标"  → YOLO 返回空
                //   "recognize 融合后无有效 block"  → 有检测但 block+face 关联/分类失败
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

            // ── 深度有效区域边界 ────────────────────────────
            // 在 block_vis 上画出深度相机（NFOV_UNBINNED）实际有深度数据的范围。
            // 彩色图 720P 约 90° HFOV，深度约 75° HFOV，
            // 左右边缘的深度值为 0，检测到那里就是无效 3D。
            if (!depth_image.empty())
            {
                const int sample_rows[] = {
                    depth_image.rows / 4,
                    depth_image.rows / 2,
                    3 * depth_image.rows / 4};
                int dl = depth_image.cols, dr = 0;
                for (int r : sample_rows)
                {
                    const uint16_t *row = depth_image.ptr<uint16_t>(r);
                    for (int c = 0; c < depth_image.cols; ++c)
                        if (row[c] != 0) { if (c < dl) dl = c; break; }
                    for (int c = depth_image.cols - 1; c >= 0; --c)
                        if (row[c] != 0) { if (c > dr) dr = c; break; }
                }
                if (dl < dr)
                {
                    cv::line(block_vis, cv::Point(dl, 0), cv::Point(dl, block_vis.rows),
                             cv::Scalar(0, 255, 255), 2);  // 黄色左边界
                    cv::line(block_vis, cv::Point(dr, 0), cv::Point(dr, block_vis.rows),
                             cv::Scalar(0, 255, 255), 2);  // 黄色右边界
                    cv::putText(block_vis, "Depth FOV", cv::Point(dl, 30),
                                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
                }
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
