#pragma once

#include "../YOLO/yoloApi.h"
#include "../ArmorDetector/ArmorDetector.h"
#include<opencv2/opencv.hpp>
#include<opencv2/tracking.hpp>

#ifndef RM_RADAR_H
#define RM_RADAR_H

#define GPU

class Radar {
public:
    Radar();

    /*
    *	@Brief: Constructor, passing in the ArmorDetector class
     *	构造函数，传入ArmorDetector类
    */
    Radar(rm::ArmorDetector &armorDetector);

    /*
    *	@Brief: Destructor, release YOLO class
     *	析构函数，释放YOLO类
    */
    ~Radar();

    /*
    *	@Brief: Set enemy color
     *	设置敌人颜色
    *	@Return: void
    */
    void set_enemy_color(int enemyColor);

    /*
    *	@Brief: Load the image to be detected into this class
     *	将需要检测的图像加载到该类中
    *	@Return: void
    */
    void load_img(const cv::Mat &srcImg);

    /*
    *	@Brief: Find all robots using the YOLO algorithm interface
     *	使用YOLO算法检测所有目标
    *	@Outputs: ALL the info of detection result
    *	@Return: void
    */
    void find_robot();

    /*
    *	@Brief: Track previously detected targets
     *	对检测到的目标进行追踪
    *	@Outputs: Finished image
    *	@Return: void
    */
    void track();

    /*
    *	@Brief: Draw a map based on the target center point
     *	将目标中心点画在小地图上
    *	@Outputs: Finished map
    *	@Return: void
    */
    void draw_map();

    /*
    *	@Brief: Get the transformation matrix of the image into the map
     *	求出图像映射到地图的变换矩阵
    *	@Outputs: Transformation matrix
    *	@Return: void
    *	@Other:
    *	    Click the three points of the original image with the mouse, you can press q to delete the points,
    *	    press r to clear all the points, after selecting the three points, press the enter key to end,
    *	    and then click the three points of the original image map in turn on the map, and press enter to end .
    *	    用鼠标单击原始图像的三个点，你可以按q删除最近选择的点，或者按r清除所有点，选择三个点后，按Enter键结束，
    *	    然后依次在地图上点击之前三个点的映射，最后按Enter键结束。
    */
    void get_transformation_mat();

    /*
    *	@Brief: Return the final processed image
     *	返回最终处理好的图像
    *	@Return: Finished image
    */
    cv::Mat getLastImg();

    /*
    *	@Brief: Return to the final processed map
     *	返回最终处理好的小地图
    *	@Return: Finished map
    */
    cv::Mat getLastMap();

    bool deal;  //是否处理好
    int _enemyColor; //敌人颜色
private:

    /*
    *	@Brief: Correct the detected rectangle
     *	对检测到的矩形进行修正
    *	@Outputs: Corrected YOLO_box
    *	@Return: void
    */
    void box_fix(cv::Rect &rect);

    /*
    *	@Brief: Callback function needed for get_transformation_mat()
     *	get_transformation_mat()需要用到的回调函数
    *	@Outputs: Vector of storage points
    *	@Return: void
    */
    static void window_get_points(int event, int x, int y, int flags, void* param);

    cv::Mat _srcImg;    //原图像
    cv::Mat _grayImg;   //ROI的灰度图
    cv::Mat _showImg;   //处理好的图像
    cv::Mat _srcMap;    //原地图
    cv::Mat _showMap;   //处理好的地图
    cv::Mat _transformation_Mat;  //变换矩阵

#ifdef GPU
    cuda::GpuMat gpuSrcImg;
    cuda::GpuMat gpuGrayImg;
#endif

    Ptr<yoloApi> YOLOv3;
    Ptr<rm::ArmorDetector> armor_detector;
    std::vector<cv::Rect> YOLO_box;          //YOLO检测出的所有矩形
    std::vector<std::string> YOLO_class;     //YOLO检测出的矩形对应类名
    std::vector<RobotDescriptor> robot_box;  //储存检测到的所有机器人信息
    Ptr<cv::MultiTracker> trackers;

};

#endif //RM_RADAR_H