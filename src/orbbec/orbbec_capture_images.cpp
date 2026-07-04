#include <stdio.h>
#include "orbbec/camera_orbbec.hpp"

#include <opencv2/opencv.hpp>
#include <string>

int main()
{
    Orbbec orbbec_device = Orbbec::Create_FromFile("/home/pi/workspace/camera_ws/src/camera_bridge/config/OrbbecConfig.yaml");

    orbbec_device.capture_images("/home/pi/workspace/camera_ws/src/camera_bridge/workspace/images", "orbbec");

    return 0;
}
