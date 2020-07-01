#pragma once
#include "opencv2/opencv.hpp"
#include "darknet.h"

#define SHOW_BOX

class YOLOv3Api {
public:
    YOLOv3Api();
    ~YOLOv3Api();
    std::vector<cv::Rect> get_boxes(cv::Mat &img);

private:
    void img_convert(const cv::Mat &img, float *dst);
    void img_resize(float *src, float *dst, int srcWidth, int srcHeight, int dstWidth, int dstHeight);
    void resize_inner(float *src, float *dst, int srcWidth, int srcHeight, int dstWidth, int dstHeight);

private:
    std::vector<cv::Rect> detect_boxes;
    std::vector<std::string> class_names_vec;
    std::string cfg_file, weight_file, class_file;
    network* net;
    int class_num;
    float thresh, nms;
};
