#include "realsense.hpp"
#include "myinfer.hpp"
#include "camera_k4a.hpp"

#include <iostream>

int main()
{
    RealSense one_eye = RealSense::Create_Default();
    // cv::Mat color = cv::imread("test_image.jpg");
    Yolo yolo;
    cv::Mat color, depth;
    yolo::BoxArray detections;
    if (one_eye.Capture(color, depth))
    {
        yolo.Single_Inference(color, detections);
    }
    else
    {
        std::cout << "Failed to capture frames from RealSense" << std::endl;
        return -1;
    }
    std::string engine_path = "workspace/model_temp/best.engine";
    yolo.Yolov8_Enable(engine_path);

    auto detection = yolo.Inference_Grid(color, true); // true 表示绘制结果
    cv::imshow("Detections", color);
    cv::waitKey(0);

    return 0;
}