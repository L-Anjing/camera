#include <stdio.h>
#include "k4a/camera_k4a.hpp"

#include <k4a/k4a.hpp>
#include <opencv2/opencv.hpp>
#include <string>

int main()
{
    K4a k4a_device = K4a::Create_FromFile("config/AzureKinectSDKConfig.yaml");

    k4a_device.record_videos("workspace/videos","R2_Fake"); // 手动录制

    return 0;
}