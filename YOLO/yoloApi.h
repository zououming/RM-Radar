#pragma once
#include "opencv2/opencv.hpp"
#include "darknet.h"

//#define SHOW_BOX

/*
* 	@Brief： This structure describes the team, color, number, unit, location, etc. of the robot
*	// 这个结构体描述机器人的队伍,颜色,编号,兵种,位置等
*/
struct RobotDescriptor {
    int team;
    cv::Scalar color;
    int numbering;
    std::string arms;
    cv::Rect2d position;
};

class yoloApi {
public:
    yoloApi();
    ~yoloApi();
    std::vector<cv::Rect> get_boxes(cv::Mat &img);
    std::vector<std::string> get_class();

private:
    void img_convert(const cv::Mat &img, float *dst);
    void img_resize(float *src, float *dst, int srcWidth, int srcHeight, int dstWidth, int dstHeight);
    void resize_inner(float *src, float *dst, int srcWidth, int srcHeight, int dstWidth, int dstHeight);

private:
    std::vector<cv::Rect> detect_boxes;
    std::vector<std::string> detect_classes;
    std::vector<std::string> class_names_vec;
    std::string cfg_file, weight_file, class_file;
    network* net;
    int class_num;
    float thresh, nms;
};
