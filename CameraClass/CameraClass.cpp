//
// Created by bazinga on 19-10-23.
//

#include "CameraClass.h"

GX_STATUS CameraClass::cameraInit() {
    emStatus = GXInitLib();
    if(emStatus != GX_STATUS_SUCCESS)
    {
        GetErrorString(emStatus);
        return emStatus;
    }
    //Get device enumerated number获取设备枚举数
    emStatus = GXUpdateDeviceList(&ui32DeviceNum, this->settings->camera_Enum);
    if(emStatus != GX_STATUS_SUCCESS)
    {
        GetErrorString(emStatus);
        GXCloseLib();
        return emStatus;
    }

    //If no device found, app exit如果找不到设备，应用程序退出
    if(ui32DeviceNum <= 0)
    {
        printf("<No device found>\n");
        GXCloseLib();
        return emStatus;
    }

    //Open first device enumerated打开枚举的第一个设备
    emStatus = GXOpenDeviceByIndex(1, &g_hDevice);
    if(emStatus != GX_STATUS_SUCCESS)
    {
        GetErrorString(emStatus);
        GXCloseLib();
        return emStatus;
    }
    //Get Device Info获取设备信息
    printf("***********************************************\n");
    //Get libary version获取libary版本
    printf("<Libary Version : %s>\n", GXGetLibVersion());
    size_t nSize = 0;
    //Get string length of Vendor name获取供应商名称的字符串长度
    emStatus = GXGetStringLength(g_hDevice, GX_STRING_DEVICE_VENDOR_NAME, &nSize);
    GX_VERIFY_EXIT(emStatus);
    //Alloc memory for Vendor name供应商名称的分配内存
    char *pszVendorName = new char[nSize];
    //Get Vendor name获取供应商名称
    emStatus = GXGetString(g_hDevice, GX_STRING_DEVICE_VENDOR_NAME, pszVendorName, &nSize);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        delete[] pszVendorName;
        pszVendorName = NULL;
        GX_VERIFY_EXIT(emStatus);
    }

    printf("<Vendor Name : %s>\n", pszVendorName);
    //Release memory for Vendor name释放供应商名称的内存
    delete[] pszVendorName;
    pszVendorName = NULL;

    //Get string length of Model name获取模型名称的字符串长度
    emStatus = GXGetStringLength(g_hDevice, GX_STRING_DEVICE_MODEL_NAME, &nSize);
    GX_VERIFY_EXIT(emStatus);
    //Alloc memory for Model name
    char *pszModelName = new char[nSize];
    //Get Model name
    emStatus = GXGetString(g_hDevice, GX_STRING_DEVICE_MODEL_NAME, pszModelName, &nSize);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        delete[] pszModelName;
        pszModelName = NULL;
        GX_VERIFY_EXIT(emStatus);
    }

    printf("<Model Name : %s>\n", pszModelName);
    //Release memory for Model name
    delete[] pszModelName;
    pszModelName = NULL;

    //Get string length of Serial number获取序列号的字符串长度
    emStatus = GXGetStringLength(g_hDevice, GX_STRING_DEVICE_SERIAL_NUMBER, &nSize);
    GX_VERIFY_EXIT(emStatus);
    //Alloc memory for Serial number
    char *pszSerialNumber = new char[nSize];
    //Get Serial Number
    emStatus = GXGetString(g_hDevice, GX_STRING_DEVICE_SERIAL_NUMBER, pszSerialNumber, &nSize);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        delete[] pszSerialNumber;
        pszSerialNumber = NULL;
        GX_VERIFY_EXIT(emStatus);
    }

    printf("<Serial Number : %s>\n", pszSerialNumber);
    //Release memory for Serial number
    delete[] pszSerialNumber;
    pszSerialNumber = NULL;

    //Get string length of Device version获取设备版本的字符串长度
    emStatus = GXGetStringLength(g_hDevice, GX_STRING_DEVICE_VERSION, &nSize);
    GX_VERIFY_EXIT(emStatus);
    char *pszDeviceVersion = new char[nSize];
    //Get Device Version
    emStatus = GXGetString(g_hDevice, GX_STRING_DEVICE_VERSION, pszDeviceVersion, &nSize);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        delete[] pszDeviceVersion;
        pszDeviceVersion = NULL;
        GX_VERIFY_EXIT(emStatus);
    }

    printf("<Device Version : %s>\n", pszDeviceVersion);
    //Release memory for Device version
    delete[] pszDeviceVersion;
    pszDeviceVersion = NULL;
    printf("***********************************************\n");

    //Get the type of Bayer conversion. whether is a color camera.获取Bayer转换的类型。是否是彩色摄像头。
    emStatus = GXIsImplemented(g_hDevice, GX_ENUM_PIXEL_COLOR_FILTER, &g_bColorFilter);
    GX_VERIFY_EXIT(emStatus);

    //This app only support color cameras此应用程序仅支持彩色摄像头
    if (!g_bColorFilter)
    {
        printf("<This app only support color cameras! App Exit!>\n");
        GXCloseDevice(g_hDevice);
        g_hDevice = NULL;
        GXCloseLib();
        return 0;
    }
    else
    {
        emStatus = GXGetEnum(g_hDevice, GX_ENUM_PIXEL_COLOR_FILTER, &g_i64ColorFilter);
        GX_VERIFY_EXIT(emStatus);
    }

    emStatus = GXGetInt(g_hDevice, GX_INT_PAYLOAD_SIZE, &g_nPayloadSize);
    GX_VERIFY(emStatus);
}

CameraClass::CameraClass(Settings * settings1) {
    this->emStatus = GX_STATUS_SUCCESS;
    //this->m_p=m_ps;
    this->settings = settings1;
    this->armor_detector = new ArmorDetector();

    //Initialize angle solver
    this->_solverPtr = new AngleSolver();
    AngleSolverParam angleParam;
    angleParam.readFile(9);
    _solverPtr->init(angleParam);
    _solverPtr->setResolution();

}

CameraClass::CameraClass(Settings * settings1,SerialPort port) {
    {
        this->emStatus = GX_STATUS_SUCCESS;
        //this->m_p=m_ps;
        this->settings=settings1;
        this->armor_detector=new ArmorDetector();

        //Initialize angle solver
        this->_solverPtr=new AngleSolver();
        AngleSolverParam angleParam;
        angleParam.readFile(8);
        _solverPtr->init(angleParam);
        _solverPtr->setResolution();
        _port=port;
        //_solverPtr->setResolution(_videoCapturePtr->getResolution());



    }
}

GX_STATUS CameraClass::cameraMode() {
    //Set acquisition mode设置采集模式
    emStatus = GXSetEnum(g_hDevice, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
    GX_VERIFY_EXIT(emStatus);
    //设置采集帧率
    emStatus = GXSetEnum(g_hDevice, GX_ENUM_ACQUISITION_FRAME_RATE_MODE,GX_ACQUISITION_FRAME_RATE_MODE_ON);
    emStatus = GXSetFloat(g_hDevice, GX_FLOAT_ACQUISITION_FRAME_RATE, this->settings->frame);
    //Set trigger mode设置触发模式
    emStatus = GXSetEnum(g_hDevice, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF);
    GX_VERIFY_EXIT(emStatus);

    //Set buffer quantity of acquisition queue设置采集队列的缓冲数量
    uint64_t nBufferNum = ACQ_BUFFER_NUM;
    emStatus = GXSetAcqusitionBufferNumber(g_hDevice, nBufferNum);
    GX_VERIFY_EXIT(emStatus);

    bool bStreamTransferSize = false;
    emStatus = GXIsImplemented(g_hDevice, GX_DS_INT_STREAM_TRANSFER_SIZE, &bStreamTransferSize);
    GX_VERIFY_EXIT(emStatus);

    if(bStreamTransferSize)
    {
        //Set size of data transfer block设置数据传输块的大小
        emStatus = GXSetInt(g_hDevice, GX_DS_INT_STREAM_TRANSFER_SIZE, ACQ_TRANSFER_SIZE);
        GX_VERIFY_EXIT(emStatus);
    }

    bool bStreamTransferNumberUrb = false;
    emStatus = GXIsImplemented(g_hDevice, GX_DS_INT_STREAM_TRANSFER_NUMBER_URB, &bStreamTransferNumberUrb);
    GX_VERIFY_EXIT(emStatus);

    if(bStreamTransferNumberUrb)
    {
        //Set qty. of data transfer block设置数量。数据传输块
        emStatus = GXSetInt(g_hDevice, GX_DS_INT_STREAM_TRANSFER_NUMBER_URB, ACQ_TRANSFER_NUMBER_URB);
        GX_VERIFY_EXIT(emStatus);
    }

//    设置自动白平衡光照模式
    emStatus = GXSetEnum(g_hDevice,GX_ENUM_AWB_LAMP_HOUSE,GX_AWB_LAMP_HOUSE_ADAPTIVE);

//    Set Balance White Mode : Continuous设置平衡白模式
    emStatus = GXSetEnum(g_hDevice, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_CONTINUOUS);

//手 动 白 平 衡 调 节------------------------
//使 用 GXGetEnumEntryNums、GXGetEnumDescription 两 个 接 口
//查 询 当 前 相 机 支 持 的 GX_ENUM_BALANCE_RATIO_SELECTOR 类 型
//参 考 接 口 说 明 部 分 , 此 处 省 略
//假 设 当 前 相 机 支 持 GX_BALANCE_RATIO_SELECTOR_RED
//选 择 白 平 衡 通 道
//    emStatus = GXSetEnum(g_hDevice,GX_ENUM_BALANCE_RATIO_SELECTOR,
//                       GX_BALANCE_RATIO_SELECTOR_RED);
////status=GXSetEnum(hDevice,GX_ENUM_BALANCE_RATIO_SELECTOR,GX_BALANCE_RATIO_SELECTOR_GREEN);
////status=GXSetEnum(hDevice,GX_ENUM_BALANCE_RAT IO_SELECTOR,GX_BALANCE_RATIO_SELECTOR_BLUE);
////获 取 白 平 衡 调 节 范 围
//    GX_FLOAT_RANGE ratioRange;
//    emStatus = GXGetFloatRange(g_hDevice, GX_FLOAT_BALANCE_RATIO, &ratioRange);
////设 置 最 小 白 平 衡 系 数
//    emStatus = GXSetFloat(g_hDevice, GX_FLOAT_BALANCE_RATIO, ratioRange.dMin);
////设 置 最 大 白 平 衡 系 数
//    emStatus = GXSetFloat(g_hDevice, GX_FLOAT_BALANCE_RATIO, ratioRange.dMax);
////设 置 自 动 白 平 衡 感 兴 趣 区 域(代 码 中 参 数 为 举 例 , 用 户 根 据 自 己 需 要 自 行 修 改 参 数 值 )
//    emStatus =GXSetInt(g_hDevice,GX_INT_AAROI_WIDTH,100);
//    emStatus =GXSetInt(g_hDevice,GX_INT_AWBROI_HEIGHT, 100);
//    emStatus =GXSetInt(g_hDevice,GX_INT_AWBROI_OFFSETX, 0);
//    emStatus =GXSetInt(g_hDevice,GX_INT_AWBROI_OFFSETY, 0);






    //设 置 连 续 自 动 增 益
    emStatus = GXSetEnum(g_hDevice, GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_CONTINUOUS);
    //设 置 连 续 自 动 黑 电 平
    emStatus = GXSetEnum(g_hDevice, GX_ENUM_BLACKLEVEL_AUTO, GX_BLACKLEVEL_AUTO_CONTINUOUS);


    //设 置 一 个 offset 偏 移 为 (0,0) ,600x400 尺 寸 的 区 域
//
//    int64_t nWidth1= 1280;
//    int64_t nHeight1= 720;
//    int64_t nOffsetX1 = 0;
//    int64_t nOffsetY1 = 0;
//    emStatus = GXSetInt(g_hDevice, GX_INT_WIDTH, nWidth1);
//    emStatus = GXSetInt(g_hDevice, GX_INT_HEIGHT, nHeight1);
//    emStatus = GXSetInt(g_hDevice, GX_INT_OFFSET_X, nOffsetX1);
//    emStatus = GXSetInt(g_hDevice, GX_INT_OFFSET_Y, nOffsetY1);

//配 置 一 个 2x2 的 Binning 和 2x2 的 Decimation

//    int64_t nBinningH = 200;
//    int64_t nBinningV = 200;
//    int64_t nDecimationH= 200;
//    int64_t nDecimationV= 200;
////设 置 水 平 和 垂 直 Binning 模 式 为 Sum 模 式
//    emStatus = GXSetEnum(g_hDevice,GX_ENUM_BINNING_HORIZONTAL_MODE,
//                         GX_BINNING_HORIZONTAL_MODE_SUM);
//    emStatus = GXSetEnum(g_hDevice,GX_ENUM_BINNING_VERTICAL_MODE,
//                         GX_BINNING_VERTICAL_MODE_SUM);
//    emStatus = GXSetInt(g_hDevice, GX_INT_BINNING_HORIZONTAL, nBinningH);
//    emStatus = GXSetInt(g_hDevice, GX_INT_BINNING_VERTICAL, nBinningV);
//    emStatus = GXSetInt(g_hDevice, GX_INT_DECIMATION_HORIZONTAL, nDecimationH);
//    emStatus = GXSetInt(g_hDevice, GX_INT_DECIMATION_VERTICAL, nDecimationV);


//设 置 最 小 曝 光 值
    emStatus = GXSetFloat(g_hDevice, GX_FLOAT_EXPOSURE_TIME, this->settings->exposureMIN);
//设 置 最 大 曝 光 值
    emStatus = GXSetFloat(g_hDevice, GX_FLOAT_EXPOSURE_TIME, this->settings->exposureMAX);
    //Allocate the memory for pixel format transform 为像素格式转换分配内存
    PreForAcquisition();

    //Device start acquisition设备启动采集
    emStatus = GXStreamOn(g_hDevice);
    if(emStatus != GX_STATUS_SUCCESS)
    {
        //Release the memory allocated释放分配的内存
        UnPreForAcquisition();
        GX_VERIFY_EXIT(emStatus);
    }
}
void CameraClass::detectingThread() {
    sleep(2);
    //this->armor_detector->setEnemyColor(BLUE);
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
        Mat src=*m_p;
        this->armor_detector->loadImg(src);
        //src=imread("/home/bazinga/CLionProjects/RM_nbut2020/1.jpg");
//        armorFlag=this->armor_detector->detect();
        if(armorFlag == ArmorDetector::ARMOR_LOCAL || armorFlag == ArmorDetector::ARMOR_GLOBAL)
        {
//            this->armor_detector->Kalman4f();
            armorVertex = this->armor_detector->getArmorVertex();
            armorType = this->armor_detector->getArmorType();

            _solverPtr->setTarget(armorVertex, armorType);
            angleFlag = _solverPtr->solve();
            if(angleFlag != rm::AngleSolver::ANGLE_ERROR)
            {
                targetAngle = _solverPtr->getCompensateAngle();

                std::cout << "Deviation1: " << targetAngle << std::endl;
                std::cout << "Distance:"<<_solverPtr->getDistance() << std::endl;
#ifndef SERIAL
                vdata={targetAngle[0],targetAngle[1],(float)_solverPtr->getDistance(),0,1,0};
                _port.TransformData(vdata);
                _port.send();
#endif
            }

        }
        ui32FrameCount++;
        time (&lEnd);
        if (lEnd - lInit >= 1)
        {
            std::cout<<"每秒"<<ui32FrameCount<<std::endl;
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

void CameraClass::ProcGetImage() {

    GX_STATUS emStatus = GX_STATUS_SUCCESS;


    //Thread running flag setup
    g_bAcquisitionFlag = true;
    PGX_FRAME_BUFFER pFrameBuffer = NULL;


    Mat m_pss;
    time_t lInit;
    time_t lEnd;
    uint32_t ui32FrameCount = 0;
    uint32_t ui32AcqFrameRate = 0;
    emStatus = GXDQBuf(g_hDevice, &pFrameBuffer, this->settings->camera_Enum);
    //sleep(1);
    m_pss.create(pFrameBuffer->nHeight,pFrameBuffer->nWidth,CV_8UC3);
    m_p=&m_pss;

    //imwrite("/home/bazinga/CLionProjects/RM_nbut2020/1.jpg",m_pss);
    while(g_bAcquisitionFlag)
    {
        if(!ui32FrameCount)
        {
            time(&lInit);
        }

        // Get a frame from Queue
        emStatus = GXDQBuf(g_hDevice, &pFrameBuffer, this->settings->camera_Enum);

        if(emStatus != GX_STATUS_SUCCESS)
        {
            if (emStatus == GX_STATUS_TIMEOUT)
            {
                continue;
            }
            else
            {
                GetErrorString(emStatus);
                break;
            }
        }

        PixelFormatConvert(pFrameBuffer);


        memcpy(m_pss.data,g_pRGBImageBuf,pFrameBuffer->nHeight * pFrameBuffer->nWidth *3);
        //cvtColor(m_pss, *m_p, CV_BGR2RGB);
        //imwrite("/home/bazinga/CLionProjects/RM_nbut2020/1.jpg",m_pss);
        //resize(m_p, imgs, Size(640,480), (0, 0), (0, 0),INTER_LINEAR);
        img=true;
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

        emStatus = GXQBuf(g_hDevice, pFrameBuffer);
        if(emStatus != GX_STATUS_SUCCESS)
        {
            GetErrorString(emStatus);
            break;
        }
    }
    printf("<Acquisition thread Exit!>\n");
}
#ifndef SHOW
    void CameraClass::showImg() {
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

        while (g_bAcquisitionFlag)
        {

            time(&lEnd);

            if (img){
                if (lEnd-lInit1<0){
                    if (count==0)count1++;
                    sprintf(lj,"ss/%d_%d.jpg",count1,count);
                    imwrite(lj,*m_p);
                }

                imshow("img",*m_p);
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

void CameraClass::PreForAcquisition() {
    g_pRGBImageBuf = new unsigned char[g_nPayloadSize * 3];
    g_pRaw8Image = new unsigned char[g_nPayloadSize];

    return;
}

int CameraClass::PixelFormatConvert(PGX_FRAME_BUFFER  pFrameBuffer) {
    GX_STATUS emStatus = GX_STATUS_SUCCESS;
    VxInt32 emDXStatus = DX_OK;

    // Convert RAW8 or RAW16 image to RGB24 image将raw8或raw16图像转换为rgb24图像
    switch (pFrameBuffer->nPixelFormat)
    {
        case GX_PIXEL_FORMAT_BAYER_GR8:
        case GX_PIXEL_FORMAT_BAYER_RG8:
        case GX_PIXEL_FORMAT_BAYER_GB8:
        case GX_PIXEL_FORMAT_BAYER_BG8:
        {
            // Convert to the RGB image转换为RGB图像

            emDXStatus = DxRaw8toRGB24((unsigned char*)pFrameBuffer->pImgBuf, g_pRGBImageBuf, pFrameBuffer->nWidth, pFrameBuffer->nHeight,
                                       RAW2RGB_NEIGHBOUR, DX_PIXEL_COLOR_FILTER(4), false);
            if (emDXStatus != DX_OK)
            {
                printf("DxRaw8toRGB24 Failed, Error Code: %d\n", emDXStatus);
                return PIXFMT_CVT_FAIL;
            }
            break;
        }
        case GX_PIXEL_FORMAT_BAYER_GR10:
        case GX_PIXEL_FORMAT_BAYER_RG10:
        case GX_PIXEL_FORMAT_BAYER_GB10:
        case GX_PIXEL_FORMAT_BAYER_BG10:
        case GX_PIXEL_FORMAT_BAYER_GR12:
        case GX_PIXEL_FORMAT_BAYER_RG12:
        case GX_PIXEL_FORMAT_BAYER_GB12:
        case GX_PIXEL_FORMAT_BAYER_BG12:
        {
            // Convert to the Raw8 image
            emDXStatus = DxRaw16toRaw8((unsigned char*)pFrameBuffer->pImgBuf, g_pRaw8Image, pFrameBuffer->nWidth, pFrameBuffer->nHeight, DX_BIT_2_9);
            if (emDXStatus != DX_OK)
            {
                printf("DxRaw16toRaw8 Failed, Error Code: %d\n", emDXStatus);
                return PIXFMT_CVT_FAIL;
            }
            // Convert to the RGB24 image
            emDXStatus = DxRaw8toRGB24((unsigned char*)g_pRaw8Image, g_pRGBImageBuf, pFrameBuffer->nWidth, pFrameBuffer->nHeight,
                                       RAW2RGB_NEIGHBOUR, DX_PIXEL_COLOR_FILTER(g_i64ColorFilter), false);
            if (emDXStatus != DX_OK)
            {
                printf("DxRaw8toRGB24 Failed, Error Code: %d\n", emDXStatus);
                return PIXFMT_CVT_FAIL;
            }
            break;
        }
        default:
        {
            printf("Error : PixelFormat of this camera is not supported\n");
            return PIXFMT_CVT_FAIL;
        }
    }
    return PIXFMT_CVT_SUCCESS;
}

void CameraClass::GetErrorString(GX_STATUS emErrorStatus) {
    char *error_info = NULL;
    size_t size = 0;
    GX_STATUS emStatus = GX_STATUS_SUCCESS;

    // Get length of error description
    emStatus = GXGetLastError(&emErrorStatus, NULL, &size);
    if(emStatus != GX_STATUS_SUCCESS)
    {
        printf("<Error when calling GXGetLastError>\n");
        return;
    }

    // Alloc error resources
    error_info = new char[size];
    if (error_info == NULL)
    {
        printf("<Failed to allocate memory>\n");
        return ;
    }

    // Get error description
    emStatus = GXGetLastError(&emErrorStatus, error_info, &size);
    if (emStatus != GX_STATUS_SUCCESS)
    {
        printf("<Error when calling GXGetLastError>\n");
    }
    else
    {
        printf("%s\n", (char*)error_info);
    }

    // Realease error resources
    if (error_info != NULL)
    {
        delete []error_info;
        error_info = NULL;
    }
}

CameraClass::~CameraClass() {
    //Stop Acquisition thread停止采集线程
    g_bAcquisitionFlag = false;

    img=false;

    //Device stop acquisition设备停止采集
    emStatus = GXStreamOff(g_hDevice);
    if(emStatus != GX_STATUS_SUCCESS)
    {
        //Release the memory allocated
        UnPreForAcquisition();
        GX_VERIFY_EXIT2(emStatus);
    }

    //Release the resources and stop acquisition thread释放资源并停止采集线程
    UnPreForAcquisition();

    //Close device关闭设备
    emStatus = GXCloseDevice(g_hDevice);
    if(emStatus != GX_STATUS_SUCCESS)
    {
        GetErrorString(emStatus);
        g_hDevice = NULL;
        GXCloseLib();

    }

    //Release libary释放libary
    emStatus = GXCloseLib();
    if(emStatus != GX_STATUS_SUCCESS)
    {
        GetErrorString(emStatus);

    }

    printf("<App exit!>\n");

}

void CameraClass::UnPreForAcquisition() {

    //Release resources
    if (g_pRaw8Image != NULL)
    {
        delete[] g_pRaw8Image;
        g_pRaw8Image = NULL;
    }
    if (g_pRGBImageBuf != NULL)
    {
        delete[] g_pRGBImageBuf;
        g_pRGBImageBuf = NULL;
    }

}
