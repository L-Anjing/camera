#include <stdio.h>
#include "k4a/camera_k4a.hpp"

#include <k4a/k4a.hpp>
#include <opencv2/opencv.hpp>
#include <string>

int main()
{

    K4a k4a_device = K4a::Create_FromFile("/home/pi/workspace/camera_ws/src/camera_bridge/config/K4AConfig.yaml");

    k4a_device.capture_images("/home/pi/workspace/camera_ws/src/camera_bridge/workspace/images","new_Real_8+"); // 手动拍摄


    return 0;
}