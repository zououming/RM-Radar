//
// Created by shady on 19-11-19.
//
#pragma once
#ifndef RM_NBUT2020_IMAGECONSPROD_H
#define RM_NBUT2020_IMAGECONSPROD_H

#include"../CameraClass/CameraClass.h"
//#define USE_VIDEO

class ImageConsProd {
public:
    ImageConsProd();
    ImageConsProd(Settings * _settings, OtherParam *_otherParam, CameraClass *cap1)
    {
        settings = _settings;
        cap = cap1;
        otherParam = _otherParam;
    }
    void ImageProducer();
    void ImageConsumer();
    void showImg();
    void ImageConsProd_init();

public:
    OtherParam *otherParam;
    Settings *settings;
    CameraClass *cap;
    rm::ArmorDetector *armor_detector;
    rm::AngleSolver *_solverPtr;
};


#endif //RM_NBUT2020_IMAGECONSPROD_H
