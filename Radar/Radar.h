#pragma once

#include "../YOLO/yoloApi.h"
#include "../ArmorDetector/ArmorDetector.h"
#include<opencv2/opencv.hpp>
#include<opencv2/tracking.hpp>

#ifndef RM_RADAR_RADAR_H
#define RM_RADAR_RADAR_H

#define GPU

class Radar {
public:
    Radar();

    Radar(rm::ArmorDetector &armorDetector);

    ~Radar();

    void load_img(const cv::Mat &srcImg);

    void find_robot();

    void track();

    cv::Mat getLastImg();

    bool deal;

private:
    void box_fix(cv::Rect &rect);

    cv::Mat _srcImg; //source img		// 原图像
    cv::Mat _roiImg; //roi from the result of last frame // 最后一帧图像的ROI
    cv::Mat _grayImg; //gray img of roi	// ROI的灰度图

#ifdef GPU
    cuda::GpuMat gpuSrcImg;
    cuda::GpuMat gpuGrayImg;
#endif

    Ptr<yoloApi> YOLOv3;
    Ptr<rm::ArmorDetector> armor_detector;
    std::vector<cv::Rect> YOLO_box;
    std::vector<std::string> YOLO_class;
    std::vector<RobotDescriptor> robot_box;
    Ptr<cv::MultiTracker> trackers;

};

#endif //RM_RADAR_RADAR_H