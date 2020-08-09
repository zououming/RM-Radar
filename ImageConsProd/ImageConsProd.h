#pragma once
#ifndef RM_NBUT2020_IMAGECONSPROD_H
#define RM_NBUT2020_IMAGECONSPROD_H

#include "../CameraClass/CameraClass.h"
#include "../Radar/Radar.h"
//#define USE_VIDEO

class ImageConsProd {
public:
    ImageConsProd();
    ImageConsProd(Settings * _settings, OtherParam *_otherParam, CameraClass *_cap, Radar *_radar)
    {
        settings = _settings;
        cap = _cap;
        otherParam = _otherParam;
        radar = _radar;
    }
    ImageConsProd(Settings * _settings, OtherParam *_otherParam, CameraClass *_cap, Radar *_leftRadar, Radar *_rightRadar)
    {
        settings = _settings;
        cap = _cap;
        otherParam = _otherParam;
        left_radar = _leftRadar;
        right_radar = _rightRadar;
    }
    void ImageProducer();
    void ImageConsumer();
    void showImg(int waitTime);
    void ImageConsProd_init();

public:
    OtherParam *otherParam;
    Settings *settings;
    CameraClass *cap;
    rm::ArmorDetector *armor_detector;
    Radar *radar;
    Radar *left_radar;
    Radar *right_radar;
};


#endif //RM_NBUT2020_IMAGECONSPROD_H
