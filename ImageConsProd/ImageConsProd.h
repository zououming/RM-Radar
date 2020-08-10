#pragma once
#ifndef RM_NBUT2020_IMAGECONSPROD_H
#define RM_NBUT2020_IMAGECONSPROD_H

#include "../CameraClass/CameraClass.h"
#include "../Radar/Radar.h"
//#define USE_VIDEO

class ImageConsProd {
public:
    ImageConsProd();

    ImageConsProd(Settings * _settings, OtherParam *_otherParam, CameraClass *_camera, Radar *_radar)
    {
        settings = _settings;
        otherParam = _otherParam;
        cameraList.emplace_back(_camera);
        radarList.emplace_back(_radar);
    }

    ImageConsProd(Settings * _settings, OtherParam *_otherParam, std::vector<CameraClass*> _cameraList, std::vector<Radar*> _radarList)
    {
        settings = _settings;
        otherParam = _otherParam;
        cameraList = _cameraList;
        radarList = _radarList;
    }

    void ImageProducer(uint32_t id);
    void ImageConsumer(uint32_t id);
    void ShowImg(int waitTime);
    void ImageConsProd_init();

public:
    OtherParam *otherParam;
    Settings *settings;
    std::vector<CameraClass*> cameraList;
    rm::ArmorDetector *armorDetector;
    std::vector<Radar*> radarList;
};


#endif //RM_NBUT2020_IMAGECONSPROD_H
