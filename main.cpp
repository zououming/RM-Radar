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
    CameraClass camera(&setting);
    Radar radar;

    ImageConsProd image_cons_prod(&setting, &other_param, &camera, &radar);
    image_cons_prod.ImageConsProd_init();

    image_cons_prod.radar->set_enemy_color(rm::BLUE);
    image_cons_prod.armor_detector->setEnemyColor(rm::BLUE);

    Mat img = imread("/home/zououming/Pictures/1233.png");
    image_cons_prod.radar->load_img(img);
    image_cons_prod.radar->get_transformation_mat();
    image_cons_prod.radar->find_robot();
    image_cons_prod.radar->track();
    image_cons_prod.showImg(0);
//    }
}

#else
int main(int argc, char * argv[]) {
    char * config_file_name = "../Settings/param_config.xml";
    if (argc > 1)
        config_file_name = argv[1];
    Settings setting(config_file_name);
    OtherParam other_param;
    Radar radar;
#ifndef SERIAL
    SerialPort port("/dev/ttyUSB0"); // 利用udev给TTL-USB串口模块重新命名(解决/dev/ttyUSB0突变成/dev/ttyUSB1的问题)
    port.initSerialPort();
    CameraClass *cameraClass1=new CameraClass(&setting,port);
#else
    CameraClass camera(&setting);
#endif
    ImageConsProd image_cons_prod(&setting, &other_param, &camera, &radar);
    GX_STATUS emStatus=camera.cameraInit();
    if(emStatus != GX_STATUS_SUCCESS){
        cout<<"初始化错误";
        return 0;
    }
    emStatus=cameraClass1->cameraMode();
    if(emStatus != GX_STATUS_SUCCESS){
        cout<<"设置错误";
        return 0;
    }

    image_cons_prod.ImageConsProd_init();
    image_cons_prod.armor_detector->setEnemyColor(rm::BLUE);

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