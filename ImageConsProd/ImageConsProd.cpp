#include "../ImageConsProd/ImageConsProd.h"


void ImageConsProd::ImageConsProd_init() {
    for (auto cap : cameraList) {
        cap->settings = this->settings;   //cap是一个相机类，这个类里面也有很多要用到setting的，所以要让相机里面本身的setting变成当前的xml
        this->armorDetector = cap->armor_detector;
    }
}


void ImageConsProd::ImageProducer(uint32_t id) {
    std::cout << "start image producer" << std::endl;
    GX_STATUS emStatus = GX_STATUS_SUCCESS;

    //Thread running flag setup
    cameraList[id]->g_bAcquisitionFlag = true;
    PGX_FRAME_BUFFER pFrameBuffer = NULL;

    Mat m_pss;
    time_t lInit;
    time_t lEnd;
    uint32_t ui32FrameCount = 0;
    uint32_t ui32AcqFrameRate = 0;
    emStatus = GXDQBuf(cameraList[id]->g_hDevice, &pFrameBuffer, this->settings->camera_Enum);
    emStatus = GXSetFloat(cameraList[id]->g_hDevice, GX_FLOAT_EXPOSURE_TIME, 3000);
    m_pss.create(pFrameBuffer->nHeight,pFrameBuffer->nWidth,CV_8UC3);
    cameraList[id]->m_p=&m_pss;
    while(cameraList[id]->g_bAcquisitionFlag)
    {
        if(!ui32FrameCount)
        {
            time(&lInit);
        }

        // Get a frame from Queue
        emStatus = GXDQBuf(cameraList[id]->g_hDevice, &pFrameBuffer, this->settings->camera_Enum);

        if(emStatus != GX_STATUS_SUCCESS)
        {
            if (emStatus == GX_STATUS_TIMEOUT)
                continue;
            else
            {
                cameraList[id]->GetErrorString(emStatus);
                break;
            }
        }

        cameraList[id]->PixelFormatConvert(pFrameBuffer);

        memcpy(m_pss.data, cameraList[id]->g_pRGBImageBuf,pFrameBuffer->nHeight * pFrameBuffer->nWidth *3);

        cameraList[id]->img = true;

        if(pFrameBuffer->nStatus != GX_FRAME_STATUS_SUCCESS)
            printf("<Abnormal Acquisition: Exception code: %d>\n", pFrameBuffer->nStatus);
        else
        {
            ui32FrameCount++;
            time (&lEnd);
            // Print acquisition info each second.
            if (lEnd - lInit >= 1)
            {
//                printf("<Successful acquisition: FrameCount: %u Width: %d Height: %d FrameID: %llu>\n",
//                       ui32FrameCount, pFrameBuffer->nWidth, pFrameBuffer->nHeight, pFrameBuffer->nFrameID);
                ui32FrameCount = 0;
            }

//            if (g_bSavePPMImage)
//            {
//                int nRet = PixelFormatConvert(pFrameBuffer);
//                if (nRet == PIXFMT_CVT_SUCCESS)
//                {
//                    SavePPMFile(pFrameBuffer->nWidth, pFrameBuffer->nHeight);
//                }
//                else
//                {
//                    printf("PixelFormat Convert failed!\n");
//                }
//                g_bSavePPMImage = false;
//            }
        }

        emStatus = GXQBuf(cameraList[id]->g_hDevice, pFrameBuffer);
        if(emStatus != GX_STATUS_SUCCESS)
        {
            cameraList[id]->GetErrorString(emStatus);
            break;
        }
    }
    printf("<Acquisition thread Exit!>\n");
}


void ImageConsProd::ImageConsumer(uint32_t id) {
    sleep(1);
    //Mat src;

    /* Variables for armor detector modeule */
    int num;
    time_t lInit;
    time_t lEnd;
    uint32_t ui32FrameCount = 0;
    uint32_t ui32LastFrameCount = 0;
    uint32_t ui32AcqFrameRate = 0;
    std::vector<cv::Point2f> armorVertex;
    time(&lInit);

#ifdef USE_VIDEO
    Mat src;
    std::string video_name="/home/zououming/5.mp4";
    VideoCapture cap_video(video_name);
    VideoWriter writer("/home/zououming/RM-radar/cmake-build-debug/VideoTest.avi", CV_FOURCC('M', 'J', 'P', 'G'), 25.0, Size(1920/2, 1080/2));

    while (true)
    {
        cap_video >> src;
        if (src.data == NULL){
            std::cout << "break" << std::endl;
            break;
        }
        resize(src, src, Size(1920/2, 1080/2));

        this->radar->load_img(src);
        if(ui32FrameCount % this->settings->track_frame == 0)
            this->radar->find_robot();
        this->radar->track();

        writer << radar->getLastImg();
        waitKey(1);
        showImg(1);
        ui32FrameCount++;
        time (&lEnd);
        if (lEnd - lInit >= 1)
        {
            std::cout<<"每秒"<<ui32FrameCount - ui32LastFrameCount << "帧" << std::endl;
            ui32LastFrameCount = ui32FrameCount;
            time (&lInit);
        }
    }
    writer.release();

#else
    radarList[id]->set_enemy_color(rm::BLUE);
    radarList[id]->get_transformation_mat();

    while (true)
    {
        Mat src = cameraList[id]->m_p->clone();
        resize(src, src, Size(1920/2, 1080/2));

        if (src.data == NULL){
            std::cout << "break" << std::endl;
            continue;
        }
        if(!ui32FrameCount)
            time(&lInit);
        this->radarList[id]->load_img(src);

        if(ui32FrameCount % this->settings->track_frame == 0)
            num = this->radarList[id]->find_robot();

        printf("camera%d: %d \n", num);
        this->radarList[id]->track();

//        this->ShowImg(1);
//        if(armorFlag == ArmorDetector::ARMOR_LOCAL || armorFlag == ArmorDetector::ARMOR_GLOBAL)
//        {
//            this->armor_detector->Kalman4f();
//            armorVertex = this->armor_detector->getArmorVertex();
//            armorType = this->armor_detector->getArmorType();
//        }
        ui32FrameCount++;
        time (&lEnd);
        if (lEnd - lInit >= 1)
        {
            std::cout<<"每秒"<<ui32FrameCount - ui32LastFrameCount << "帧" << std::endl;
            ui32LastFrameCount = ui32FrameCount;
            time (&lInit);
        }
    }
#endif
}

void ImageConsProd::ShowImg(int waitTime) {
    Mat map;
    uint32_t radarNum = radarList.size();

    if(waitTime < 0) {
        time_t lInit, lEnd;
        time(&lInit);
        uint32_t ui32FrameCount = 0;
        while (1) {
            for (int i = 0; i < radarNum; i++) {
                if (!radarList[i]->deal)
                    continue;
                Mat img = radarList[i]->getLastImg();
                if (i > 0)
                    map = *radarList[0] + *radarList[i];
                else
                    map = radarList[0]->getLastMap();

                if (img.data != NULL)
                    imshow("camera"+std::to_string(i+1), img);
                radarList[i]->deal = false;
                ui32FrameCount++;
            }
            if (map.data != NULL)
                imshow("map", map);
            waitKey(1);
            time(&lEnd);
            if(lEnd - lInit >= 1){
                std::cout << "show: " << ui32FrameCount << std::endl;
                ui32FrameCount = 0;
                time(&lInit);
            }
        }
    }
    else {
        for (int i = 0; i < radarNum; i++) {
            Mat img = radarList[i]->getLastImg();
            if (i > 0)
                map = *radarList[0] + *radarList[i];
            else
                map = radarList[0]->getLastMap();

            if (img.data != NULL)
                imshow("camera"+std::to_string(i+1), img);
        }
        if (map.data != NULL) {
            imshow("map", map);
        }
        waitKey(waitTime);
    }
}

#ifndef SHOW
void ImageConsProd::showImg() {
        time_t lInit;
        time_t lInit1;
        time_t lEnd;
        time(&lInit);
        time(&lInit1);
        int count=0;
        int count1=0;
        //namedWindow("img",0);
        //resizeWindow("img",1440,900);
        char lj[15]={0};

        while (cap->g_bAcquisitionFlag)
        {

            time(&lEnd);

            if (cap->img){
                if (lEnd-lInit1<0){
                    if (count==0)count1++;
                    sprintf(lj,"ss/%d_%d.jpg",count1,count);
                    imwrite(lj,*cap->m_p);
                }

                imshow("img",*cap->m_p);
                count++;
                waitKey(1);

            }
            if (lEnd-lInit>=1){
                cout<<"显示帧率："<<count<<endl;
                count=0;
                lInit=lEnd;
            }

        }
    }
#endif


