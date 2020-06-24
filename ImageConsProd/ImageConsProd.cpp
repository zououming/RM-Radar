//
// Created by shady on 19-11-19.
//

#include "../ImageConsProd/ImageConsProd.h"
#define USE_VIDEO

void ImageConsProd::ImageConsProd_init() {
#ifndef USE_VIDEO
    GX_STATUS emStatus=cap->cameraInit();
    if(emStatus != GX_STATUS_SUCCESS){
        cout<<"初始化错误";
        return ;
    }
    emStatus=cap->cameraMode();
    if(emStatus != GX_STATUS_SUCCESS){
        cout<<"设置错误";
        return ;
    }
#endif
    this->cap->settings=this->settings;   //cap是一个相机类，这个类里面也有很多要用到setting的，所以要让相机里面本身的setting变成当前的xml
    this->armor_detector=new ArmorDetector();
    //Initialize angle solver
    this->_solverPtr=new AngleSolver();
    AngleSolverParam angleParam;
    angleParam.readFile(9);
    this->_solverPtr->init(angleParam);
    this->_solverPtr->setResolution();
    this->_solverPtr->initset(this->settings);
    //_solverPtr->setResolution(_videoCapturePtr->getResolution());

}
void ImageConsProd::ImageProducer() {
#ifdef USE_VIDEO
    settings->save_result=0;
    string video_name="../test/test.MOV";
    Mat m_pss;
    m_pss.create(2000,2000,CV_8UC3);
    cap->m_p=&m_pss;
    time_t lInit;
    time_t lEnd;
    uint32_t ui32FrameCount = 0;
    uint32_t ui32AcqFrameRate = 0;
    VideoCapture cap_video(video_name);
    if(!cap_video.isOpened())
    {
        return;
    }
    while(1)
    {
        if(!ui32FrameCount)
        {
            time(&lInit);
        }
        cap_video>>m_pss;
        if(m_pss.data==NULL)
        {
           continue;
        }
    }
#else
    GX_STATUS emStatus = GX_STATUS_SUCCESS;


    //Thread running flag setup
    cap->g_bAcquisitionFlag = true;
    PGX_FRAME_BUFFER pFrameBuffer = NULL;


    Mat m_pss;
    time_t lInit;
    time_t lEnd;
    uint32_t ui32FrameCount = 0;
    uint32_t ui32AcqFrameRate = 0;
    emStatus = GXDQBuf(cap->g_hDevice, &pFrameBuffer, this->settings->camera_Enum);
    //sleep(1);
    m_pss.create(pFrameBuffer->nHeight,pFrameBuffer->nWidth,CV_8UC3);
    cap->m_p=&m_pss;

    //imwrite("/home/bazinga/CLionProjects/RM_nbut2020/1.jpg",m_pss);
    while(cap->g_bAcquisitionFlag)
    {
        if(!ui32FrameCount)
        {
            time(&lInit);
        }

        // Get a frame from Queue
        emStatus = GXDQBuf(cap->g_hDevice, &pFrameBuffer, this->settings->camera_Enum);

        if(emStatus != GX_STATUS_SUCCESS)
        {
            if (emStatus == GX_STATUS_TIMEOUT)
            {
                continue;
            }
            else
            {
                cap->GetErrorString(emStatus);
                break;
            }
        }

        cap->PixelFormatConvert(pFrameBuffer);


        memcpy(m_pss.data,cap->g_pRGBImageBuf,pFrameBuffer->nHeight * pFrameBuffer->nWidth *3);

        //cvtColor(m_pss, *m_p, CV_BGR2RGB);
        //imwrite("/home/bazinga/CLionProjects/RM_nbut2020/1.jpg",m_pss);
        //resize(m_p, imgs, Size(640,480), (0, 0), (0, 0),INTER_LINEAR);
        cap->img=true;
        //imshow("xj",m_p);
        //waitKey(30);
        if(pFrameBuffer->nStatus != GX_FRAME_STATUS_SUCCESS)
        {
            printf("<Abnormal Acquisition: Exception code: %d>\n", pFrameBuffer->nStatus);
        }
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

        emStatus = GXQBuf(cap->g_hDevice, pFrameBuffer);
        if(emStatus != GX_STATUS_SUCCESS)
        {
            cap->GetErrorString(emStatus);
            break;
        }
    }
    printf("<Acquisition thread Exit!>\n");
#endif
}










void ImageConsProd::ImageConsumer() {
    sleep(2);
    this->armor_detector->setEnemyColor(BLUE);
    //Mat src;

    /* Variables for angle solve module */
    int angleFlag;
    Vec2f targetAngle;

    /* Variables for armor detector modeule */
    int armorFlag;
    int armorType;
    time_t lInit;
    time_t lEnd;
    uint32_t ui32FrameCount = 0;
    uint32_t ui32AcqFrameRate = 0;
    std::vector<cv::Point2f> armorVertex;
    time(&lInit);
    for(;;){
        if(!ui32FrameCount)
        {
            time(&lInit);
        }
        Mat src=*cap->m_p;
        this->armor_detector->loadImg(src);
        //src=imread("/home/bazinga/CLionProjects/RM_nbut2020/1.jpg");
        armorFlag=this->armor_detector->detect();
        if(armorFlag == ArmorDetector::ARMOR_LOCAL || armorFlag == ArmorDetector::ARMOR_GLOBAL)
        {
//            this->armor_detector->Kalman4f();
            armorVertex = this->armor_detector->getArmorVertex();
            armorType = this->armor_detector->getArmorType();

            this->_solverPtr->setTarget(armorVertex, armorType);
            angleFlag = this->_solverPtr->solve();
            if(angleFlag != rm::AngleSolver::ANGLE_ERROR)
            {
                targetAngle =this-> _solverPtr->getCompensateAngle();

                cout << "Deviation1: " << targetAngle << endl;
            }

        }
        ui32FrameCount++;
        time (&lEnd);
        if (lEnd - lInit >= 1)
        {
            cout<<"每秒"<<ui32FrameCount<<endl;
            ui32FrameCount = 0;
        }

        //cout << "Deviation: " << targetAngle << endl;
        //cout<<i<<endl;
#ifndef SHOW
        //        if (i){
            imshow("11",src);
            waitKey(1);
//        }
#endif
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


