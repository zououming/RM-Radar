#pragma once
#include "Serial/serialport.h"
#include "ImageConsProd/ImageConsProd.h"
using namespace rm;
//#define TEXT_PIC
#ifdef TEXT_PIC

using namespace std;
int main(int argc, char * argv[]) {
    char * config_file_name = "../Settings/param_config.xml";
    Settings setting(config_file_name);
    OtherParam other_param;
    CameraClass camera(&setting);
    YoloApi YOLOv3;
    ArmorDetector armorDetector;
    Radar leftRadar(&armorDetector, &YOLOv3);
    Radar rightRadar(&armorDetector, &YOLOv3);

    ImageConsProd image_cons_prod(&setting, &other_param, &camera, &leftRadar, &rightRadar);
    image_cons_prod.ImageConsProd_init();

    image_cons_prod.left_radar->set_enemy_color(rm::BLUE);
    image_cons_prod.right_radar->set_enemy_color(rm::BLUE);

    Mat img = imread("/home/zououming/Pictures/1233.png");
    image_cons_prod.left_radar->load_img(img);
    image_cons_prod.left_radar->get_transformation_mat();

    Mat img1 = imread("/home/zououming/Pictures/3.png");
    image_cons_prod.right_radar->load_img(img1);
    image_cons_prod.right_radar->get_transformation_mat();

    image_cons_prod.left_radar->find_robot();
    image_cons_prod.left_radar->track();
    image_cons_prod.right_radar->find_robot();
    image_cons_prod.right_radar->track();

    Mat map = *image_cons_prod.left_radar + *image_cons_prod.right_radar;
    imshow("map", map);
    imshow("src", leftRadar.getLastImg());
    imshow("1", rightRadar.getLastImg());

    waitKey(0);
//    }
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
    Radar leftRadar(&armorDetector, &YOLOv3);
    Radar rightRadar(&armorDetector, &YOLOv3);
#ifndef SERIAL
    SerialPort port("/dev/ttyUSB0"); // 利用udev给TTL-USB串口模块重新命名(解决/dev/ttyUSB0突变成/dev/ttyUSB1的问题)
    port.initSerialPort();
    CameraClass *cameraClass1=new CameraClass(&setting,port);
#else
    CameraClass camera(&setting);
#endif
    ImageConsProd image_cons_prod(&setting, &other_param, &camera, &leftRadar, &rightRadar);
    GX_STATUS emStatus=camera.cameraInit();
    if(emStatus != GX_STATUS_SUCCESS){
        std::cout<<"初始化错误";
        return 0;
    }
    emStatus=camera.cameraMode();
    if(emStatus != GX_STATUS_SUCCESS){
        std::cout<<"设置错误";
        return 0;
    }

    image_cons_prod.ImageConsProd_init();

#ifdef USE_VIDEO
    std::thread t2(&ImageConsProd::ImageConsumer, image_cons_prod);
    t2.join();

#else
    std::thread t0(&ImageConsProd::showImg, image_cons_prod, -1);
    std::thread t1(&ImageConsProd::ImageProducer, image_cons_prod); // pass by reference
    std::thread t2(&ImageConsProd::ImageConsumer, image_cons_prod);

    t0.join();
    t1.join();
    t2.join();
    camera.~CameraClass();
    destroyAllWindows();
    return 0;
#endif //USE_VIDEO
}
#endif