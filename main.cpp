#pragma once
#include "Serial/serialport.h"
#include "ImageConsProd/ImageConsProd.h"
using namespace rm;

#define TEXT_PIC

#ifdef TEXT_PIC
int main(int argc, char * argv[]) {
    char * config_file_name = "../Settings/param_config.xml";
    Settings setting(config_file_name);
    OtherParam other_param;
    CameraClass *cameraClass1=new CameraClass(&setting);

    ImageConsProd image_cons_prod(&setting, &other_param,cameraClass1);
    image_cons_prod.ImageConsProd_init();

    image_cons_prod.armor_detector->setEnemyColor(RED);
    string img_path = "/home/zououming/Pictures/hsv/";
    Mat img;
    for( int i = 2; i < 12; i++ ) {
        string path = img_path + to_string(i) + ".png";
        img = imread(path);
        image_cons_prod.armor_detector->loadImg(img);
        image_cons_prod.armor_detector->detect();
    }
}


#else
int main(int argc, char * argv[]) {


    char * config_file_name = "../Settings/param_config.xml";
    if (argc > 1)
        config_file_name = argv[1];
    Settings setting(config_file_name);
    OtherParam other_param;
    // communicate with car
    //int fd2car = openPort("/dev/ttyTHS2");
    //configurePort(fd2car);
    //Mat *m_p= nullptr;
#ifndef SERIAL
    SerialPort port("/dev/ttyUSB0"); // 利用udev给TTL-USB串口模块重新命名(解决/dev/ttyUSB0突变成/dev/ttyUSB1的问题)
    port.initSerialPort();
    CameraClass *cameraClass1=new CameraClass(&setting,port);
#else
    CameraClass *cameraClass1=new CameraClass(&setting);
#endif



    ImageConsProd image_cons_prod(&setting, &otherParam,cameraClass1);
    image_cons_prod.ImageConsProd_init();
    std::thread t1(&ImageConsProd::ImageProducer, image_cons_prod); // pass by reference
    std::thread t2(&ImageConsProd::ImageConsumer, image_cons_prod);
    


#ifndef SHOW
    thread t0(&ImageConsProd::showImg,image_cons_prod);
    t0.join();
#endif
    t1.join();
    t2.join();
    cameraClass1->~CameraClass();
    destroyAllWindows();
    return 0;
}
#endif