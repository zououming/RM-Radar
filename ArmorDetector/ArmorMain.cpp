//
// Created by bazinga on 19-10-25.
//
//This file does not belong to any project target,code insight features might not work properly
#include <iostream>
#include "ArmorDetector.h"


cv::Mat srcImg;

int main()
{
    cv::Mat *src= nullptr;
    srcImg = cv::imread("1.png");
    cv::imshow("123",srcImg);
    cv::waitKey(0);
    src=&srcImg;
    rm::ArmorDetector armor_detector;
    armor_detector.loadImg(*src);
    armor_detector.detect();

    /*
    cv::VideoCapture capture;
    capture.open("j:\\RedCar.avi");
    while (capture.read(srcImg))
    {
        //srcImg = cv::imread("j:\\test.png");
        rm::ArmorDetector armor_detector;
        armor_detector.loadImg(srcImg);
        armor_detector.detect();
        //cv::imshow("srcImg", srcImg);
        char c = cv::waitKey(1);
        if (c == 27)
            break;
    }

    */
    return 0;
}