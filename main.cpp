#pragma once
#include "Serial/serialport.h"
#include "ImageConsProd/ImageConsProd.h"
using namespace rm;
#define TEXT_PIC
#ifdef TEXT_PIC

using namespace std;
int main(int argc, char * argv[]) {
    char * config_file_name = "../Settings/param_config.xml";
    Settings setting(config_file_name);
    OtherParam other_param;
    YOLOClass YOLOv4("../YOLO/weightFile/my_yolov4.cfg", "../YOLO/weightFile/my_yolov4_last.weights", "../YOLO/weightFile/myData.names", 0.5);
    ArmorDetector armorDetector;
    CameraClass camera(&setting);
    Radar radar(&armorDetector, &YOLOv4);
    ImageConsProd image_cons_prod(&setting, &other_param, &camera, &radar);
    image_cons_prod.ImageConsProd_init();

    image_cons_prod.radarList[0]->setEnemyColor(rm::GREEN);

    cv::Mat img = cv::imread("/home/zououming/Pictures/1233.png");
    image_cons_prod.radarList[0]->loadImg(img);
    image_cons_prod.radarList[0]->getTransformationMat();

    image_cons_prod.radarList[0]->findRobot();
    image_cons_prod.radarList[0]->trackRobot();
    image_cons_prod.radarList[0]->drawMap();

    cv::imshow("./img.jpg", image_cons_prod.radarList[0]->getLastImg());
    cv::imshow("./map.jpg", image_cons_prod.radarList[0]->getLastMap());

    image_cons_prod.ShowImg(0);
}

#else
int main(int argc, char * argv[]) {
    char * config_file_name = "../Settings/param_config.xml";
    if (argc > 1)
        config_file_name = argv[1];
    Settings setting(config_file_name);
    OtherParam other_param;
    YoloApi YOLOv3;
    ArmorDetector armorDetector;

    std::vector<Radar*> radar_list;
    Radar left_radar(&armorDetector, &YOLOv3);
    Radar right_radar(&armorDetector, &YOLOv3);

    radar_list.emplace_back(&left_radar);
    radar_list.emplace_back(&right_radar);
#ifndef SERIAL
    SerialPort port("/dev/ttyUSB0"); // 利用udev给TTL-USB串口模块重新命名(解决/dev/ttyUSB0突变成/dev/ttyUSB1的问题)
    port.initSerialPort();
    CameraClass *cameraClass1=new CameraClass(&setting,port);
#else
    std::vector<CameraClass*> camera_list;
    CameraClass left_camera(&setting);
    CameraClass right_camera(&setting);

    camera_list.emplace_back(&left_camera);
    camera_list.emplace_back(&right_camera);
#endif
    ImageConsProd image_cons_prod(&setting, &other_param, camera_list, radar_list);
    GX_STATUS emStatus1 = camera_list[0]->cameraInit(1);
    GX_STATUS emStatus2 = camera_list[1]->cameraInit(2);

    if(emStatus1 != GX_STATUS_SUCCESS){
        std::cout<<"相机1初始化错误";
        return 0;
    }
    emStatus1 = camera_list[0]->cameraMode();
    if(emStatus1 != GX_STATUS_SUCCESS){
        std::cout<<"相机1设置错误";
        return 0;
    }

    if(emStatus2 != GX_STATUS_SUCCESS){
        std::cout<<"相机2初始化错误";
        return 0;
    }
    emStatus2 = camera_list[1]->cameraMode();
    if(emStatus1 != GX_STATUS_SUCCESS){
        std::cout<<"相机2设置错误";
        return 0;
    }

    image_cons_prod.ImageConsProd_init();

#ifdef USE_VIDEO
    std::thread t2(&ImageConsProd::ImageConsumer, image_cons_prod);
    t2.join();

#else
    std::thread show_thread(&ImageConsProd::ShowImg, image_cons_prod, -1);
    std::thread collection_thread1(&ImageConsProd::ImageProducer, image_cons_prod, 0); // pass by reference
    std::thread processing_thread1(&ImageConsProd::ImageConsumer, image_cons_prod, 0);
    std::thread collection_thread2(&ImageConsProd::ImageProducer, image_cons_prod, 1); // pass by reference
    std::thread processing_thread2(&ImageConsProd::ImageConsumer, image_cons_prod, 1);

    show_thread.join();
    collection_thread1.join();
    processing_thread1.join();
    collection_thread2.join();
    processing_thread2.join();

    for (auto camera : camera_list)
        camera->~CameraClass();
    destroyAllWindows();
    return 0;
#endif //USE_VIDEO
}
#endif