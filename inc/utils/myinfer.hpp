#ifndef __MYINFER_HPP__
#define __MYINFER_HPP__

#include <opencv2/opencv.hpp>


#include "utils/infer.hpp"
#include "common/yolo.hpp"

#include <chrono>

static const char *yolo_labels[] = {
    // TODO:Add labels
    "block", // 0

    "R1_face", // 1

    "R2_face_0",  // 2
    "R2_face_1",  // 3
    "R2_face_2",  // 4
    "R2_face_3",  // 5
    "R2_face_4",  // 6
    "R2_face_5",  // 7
    "R2_face_6",  // 8
    "R2_face_7",  // 9
    "R2_face_8",  // 10
    "R2_face_9",  // 11
    "R2_face_10", // 12
    "R2_face_11", // 13
    "R2_face_12", // 14
    "R2_face_13", // 15
    "R2_face_14", // 16

    "R2f_face_0",  // 17
    "R2f_face_1",  // 18
    "R2f_face_2",  // 19
    "R2f_face_3",  // 20
    "R2f_face_4",  // 21
    "R2f_face_5",  // 22
    "R2f_face_6",  // 23
    "R2f_face_7",  // 24
    "R2f_face_8",  // 25
    "R2f_face_9",  // 26
    "R2f_face_10", // 27
    "R2f_face_11", // 28
    "R2f_face_12", // 29
    "R2f_face_13", // 30
    "R2f_face_14", // 31
};

class Yolo
{
private:
    std::string engine;
    yolo::Type type;
    yolo::Image cvimg(const cv::Mat &image);
    bool load_flag;
    std::shared_ptr<yolo::Infer> yolo;
    float confidence_threshold;
    float nms_threshold;

public:
    void Yolov8_Enable(std::string &engine_);

    void Yolov8_Seg_Enable(std::string &engine_seg);
    
    void Set_Confidence_Threshold(float conf_thres, float nms_thres = 0.5f);

    void Single_Inference(std::string path);

    void Single_Inference(cv::Mat &image);

    void Single_Inference(cv::Mat &image, yolo::BoxArray &objs_out);



    Yolo();

    ~Yolo();
};

#endif