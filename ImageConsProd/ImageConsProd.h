#pragma once
#ifndef RM_NBUT2020_IMAGECONSPROD_H
#define RM_NBUT2020_IMAGECONSPROD_H

#include"../CameraClass/CameraClass.h"
#include <mutex>
//#define USE_VIDEO

class ImageConsProd {
public:
    ImageConsProd();
    ImageConsProd(Settings * _settings, OtherParam *_otherParam, CameraClass *cap1, std::mutex *_mutex)
    {
        settings = _settings;
        cap = cap1;
        otherParam = _otherParam;
        mutex = _mutex;
    }
    void ImageProducer();
    void ImageConsumer();
    void showImg();
    void ImageConsProd_init();

public:
    std::mutex *mutex;
    OtherParam *otherParam;
    Settings *settings;
    CameraClass *cap;
    rm::ArmorDetector *armor_detector;
    rm::AngleSolver *_solverPtr;
};


#endif //RM_NBUT2020_IMAGECONSPROD_H
