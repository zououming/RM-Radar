//
// Created by bazinga on 19-10-23.
//
#pragma once
#ifndef RM_NBUT2020_CAMERACLASS_H
#define RM_NBUT2020_CAMERACLASS_H
#define SHOW
#include "../Settings/Settings.hpp"
#include "../Serial/serialport.h"
#include "../ArmorDetector/ArmorDetector.h"
#include "GxIAPI.h"
#include "DxImageProc.h"
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include "thread"
#include <unistd.h>
#include <time.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//using namespace cv;
using namespace rm;

#define ACQ_BUFFER_NUM          5               ///< Acquisition Buffer Qty./采集缓冲器数量
#define ACQ_TRANSFER_SIZE       (64 * 1024)     ///< Size of data transfer block/数据传输块的大小
#define ACQ_TRANSFER_NUMBER_URB 64              ///< Qty. of data transfer block/数量。数据传输块
//#define FILE_NAME_LEN           50              ///< Save image file name length/保存图像文件名长度

#define PIXFMT_CVT_FAIL             -1             ///< PixelFormatConvert fail/PixelFormatConvert失败
#define PIXFMT_CVT_SUCCESS          0              ///< PixelFormatConvert success/PixelFormatConvert成功

//Show error message显示错误消息
#define GX_VERIFY(emStatus) \
    if (emStatus != GX_STATUS_SUCCESS)     \
    {                                      \
        GetErrorString(emStatus);          \
        return emStatus;                   \
    }

//Show error message, close device and lib显示错误消息，关闭设备和lib
#define GX_VERIFY_EXIT(emStatus) \
    if (emStatus != GX_STATUS_SUCCESS)     \
    {                                      \
        GetErrorString(emStatus);          \
        GXCloseDevice(g_hDevice);          \
        g_hDevice = NULL;                  \
        GXCloseLib();                      \
        printf("<App Exit!>\n");           \
        return emStatus;                   \
    }
#define GX_VERIFY_EXIT2(emStatus) \
    if (emStatus != GX_STATUS_SUCCESS)     \
    {                                      \
        GetErrorString(emStatus);          \
        GXCloseDevice(g_hDevice);          \
        g_hDevice = NULL;                  \
        GXCloseLib();                      \
        printf("<App Exit!>\n");           \
    }

class CameraClass {
public:
    GX_DEV_HANDLE g_hDevice = nullptr;                     ///< Device handle设备手柄
    bool g_bColorFilter = false;                        ///< Color filter support flag颜色过滤器支持标志
    int64_t g_i64ColorFilter = GX_COLOR_FILTER_BAYER_BG;    ///< Color filter of device设备滤色器
    bool g_bAcquisitionFlag = false;                    ///< Thread running flag线程运行标志
    //bool g_bSavePPMImage = false;                       ///< Save raw image flag保存原始图像标志
    //pthread_t g_nAcquisitonThreadID = 0;                ///< Thread ID of Acquisition thread
    //pthread_t g_nAcquisitonThreadID1 = 1;
    unsigned char* g_pRGBImageBuf = nullptr;               ///< Memory for RAW8toRGB24
    unsigned char* g_pRaw8Image = nullptr;                 ///< Memory for RAW16toRAW8

    int64_t g_nPayloadSize = 0;                         ///< Payload size

    bool img=false;

    //opencv
    cv::Mat  *m_p;
    //Mat imgs;

    //Camera
    GX_STATUS emStatus;
    uint32_t ui32DeviceNum = 0;

    Settings *settings;
    OtherParam *other_param;
    rm::ArmorDetector *armor_detector;

    SerialPort _port;
    VisionData vdata;

public:
    CameraClass(Settings *settings1);
    explicit CameraClass(Settings *settings1, SerialPort port);
    GX_STATUS cameraInit(uint32_t id);
    GX_STATUS cameraMode();
    ~CameraClass();
    void detectingThread();

    //Allocate the memory for pixel format transform
    void PreForAcquisition();

    //Release the memory allocated
    void UnPreForAcquisition();

    //Convert frame date to suitable pixel format将帧转换为合适的像素格式
    int PixelFormatConvert(PGX_FRAME_BUFFER  pFrameBuffer);


    //Acquisition thread function采集线程功能
    void ProcGetImage();



//Get description of error
    void GetErrorString(GX_STATUS emErrorStatus);

#ifndef SHOW
    void showImg();
#endif


};


#endif //RM_NBUT2020_CAMERACLASS_H
