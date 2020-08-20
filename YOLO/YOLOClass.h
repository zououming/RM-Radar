#pragma once
#include "opencv2/opencv.hpp"
#include "darknet.h"

struct RobotDescriptor{
    int team;
    cv::Scalar color;
    int numbering;
    std::string arms;
    cv::Rect position;
    cv::Point map_position;
};

class YOLOClass {
public:
    YOLOClass();
    YOLOClass(std::string _cfgFile, std::string _weightFile, std::string _classFile, float _thresh);
    std::vector<cv::Rect> get_boxes(cv::Mat &img);
    std::vector<std::string> get_class();

private:
    image matToImage(cv::Mat &mat);
    std::vector<cv::Rect> detect_boxes;
    std::vector<std::string> detect_classes;
    std::vector<std::string> class_names_vec;
    std::string cfg_file, weight_file, class_file;
    network net;
    int class_num;
    float thresh, nms;
};