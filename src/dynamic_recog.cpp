#include "grid_state.hpp"
#include "camera_k4a.hpp"
#include "ros_interface.hpp"
#include "myinfer.hpp"
#include "utils.hpp"

#include <chrono>
#include <thread>
#include <ros/ros.h>

int main(int argc, char** argv)
{
    K4a camera;
    Yolo yolo;
    GridState grid;

    std::string engine_path = "/home/li/camera_cxx/workspace/model_temp/best.engine";
    yolo::BoxArray detections;
    // yolo.Yolov8_Enable(engine_path); // 检测
    yolo.Yolov8_Seg_Enable(engine_path); // 分割

    ros::init(argc, argv, "dynamic_recognition");
    ros::NodeHandle nh;
    ROSInterface ros_interface;

    ros::Rate loop_rate(30); // 控制循环频率

    float fx, fy, cx, cy;
    camera.get_intrinsics(fx, fy, cx, cy);
    while (ros::ok())
    {
        ros_interface.spinOnce();
        Pose pose = ros_interface.getLidarPose();
        cv::Mat color_image, depth_image;
        cv::Mat depth_display;
        camera.Image_to_Cv(color_image, depth_image);

        yolo.Single_Inference(color_image, detections);

        for (auto &det : detections)
        {
            float u = (det.left + det.right) / 2.0f;
            float v = (det.top + det.bottom) / 2.0f;

            // 获取深度
            if (v < 0 || v >= depth_image.rows || u < 0 || u >= depth_image.cols)
                continue;
            float depth_value = depth_image.at<uint16_t>((int)v, (int)u) / 1000.0f; // mm --> m
            if (depth_value <= 0.1 || depth_value > 5.0)
                continue;

            float Xc = (u - cx) * depth_value / fx; // 相机坐标
            float Yc = (v - cy) * depth_value / fy;
            float Zc = depth_value; // 深度

            //  相机 -> 世界坐标

            float X_offset = 0.1f;     // 相机比雷达前10cm
            float Y_offset = 0.0f;     // 相机与雷达水平对齐
            float theta_offset = 0.0f; // 朝向偏差

            // 假设相机坐标(Xc, Yc, Zc)，根据标准：
            // 相机 Z 向前，对应机器人 X；
            // 相机 X 向右，对应机器人 -Y；
            // 相机 Y 向下，可忽略（地面任务中 z 不重要）

            // 相机->机器人坐标
            float Xr = Zc + X_offset;
            float Yr = -Xc + Y_offset;

            // 机器人->世界坐标
            float cosT = std::cos(pose.theta + theta_offset);
            float sinT = std::sin(pose.theta + theta_offset);

            float Xw = pose.x + cosT * Xr - sinT * Yr;
            float Yw = pose.y + sinT * Xr + cosT * Yr;

            // 更新4x3全局方块矩阵
            grid.updateWorldPosition(Xw, Yw, det.class_label + 1);

            // 输出
            std::cout << "检测到类别 " << det.class_label
                      << " 在世界坐标: (" << Xw << ", " << Yw << ")\n";
        }

        grid.print();

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}
