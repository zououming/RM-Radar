/**************************************************************

MIT License

Copyright (c) 2018 SEU-SuperNova-CVRA

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

Authors:	Rick_Hang, <213162574@seu.edu.cn>
		BinYan Hu
**************************************************************/
#include"ArmorDetector.h"
#include<iostream>
#include<vector>
#include<math.h>
#include<string>
#include <unistd.h>
#include"opencv_extended.h"
//#include"../General/numeric_rm.h"

using namespace std;
using namespace cv;
using namespace cv::ml;

namespace rm
{

    enum
    {
        WIDTH_GREATER_THAN_HEIGHT,//宽度大于高度
        ANGLE_TO_UP//向上倾斜
    };
    /*
    *	@Brief:		// regulate the rotated rect 调整旋转矩形
    *	@Input:		// rotated rec				 旋转矩形
    *				// regulation mode			 调整方法
    *	@Return:	// regulated rec			 返回一个旋转矩形
    */
    cv::RotatedRect& adjustRec(cv::RotatedRect& rec, const int mode)
    {
        using std::swap;						// 交换

        float& width = rec.size.width;			// 矩形宽
        float& height = rec.size.height;		// 矩形高
        float& angle = rec.angle;				// 矩形旋转的角度

        if (mode == WIDTH_GREATER_THAN_HEIGHT)	// 宽度大于高度
        {
            if (width < height)
            {
                swap(width, height);
                angle += 90.0;
            }
        }

        while (angle >= 90.0) angle -= 180.0;		// 让角度在0~90之间
        while (angle < -90.0) angle += 180.0;

        if (mode == ANGLE_TO_UP)
        {
            if (angle >= 45.0)
            {
                swap(width, height);
                angle -= 90.0;
            }
            else if (angle < -45.0)
            {
                swap(width, height);
                angle += 90.0;
            }
        }

        return rec;
    }


    ArmorDescriptor::ArmorDescriptor()		// 构造函数	// 初始化为0   装甲描述符
    {
        rotationScore = 0;					// 目标得分
        sizeScore = 0;						// 大小得分
        vertex.resize(4);
        for (int i = 0; i < 4; i++)
        {
            vertex[i] = cv::Point2f(0, 0);
        }
        type = UNKNOWN_ARMOR;//未知的装甲



    }

    /******************************
     *  @ Param: const LightDescriptor& lLight // 左灯条
     * 			 const LightDescriptor& rLight // 右灯条
     * 			 const int armorTyep // 装甲板类型
     * 			 const cv::Mat & grayImg // ROI的灰度图
     * 			 float rotaScore	// ？
     * 			 ArmorParam _param  // 装甲板参数
     * ****************************/
    ArmorDescriptor::ArmorDescriptor(const LightDescriptor& lLight, const LightDescriptor& rLight, const int armorType, const cv::Mat& grayImg, float rotaScore, ArmorParam _param, Mat srcImg)//装甲描述符
    {
        //handle two lights
        lightPairs[0] = lLight.rec();	// 保存两个旋转矩形
        lightPairs[1] = rLight.rec();

        cv::Size exLSize(int(lightPairs[0].size.width), int(lightPairs[0].size.height * 2));
        cv::Size exRSize(int(lightPairs[1].size.width), int(lightPairs[1].size.height * 2));
        cv::RotatedRect exLLight(lightPairs[0].center, exLSize, lightPairs[0].angle);
        cv::RotatedRect exRLight(lightPairs[1].center, exRSize, lightPairs[1].angle);

        cv::Point2f pts_l[4];
        exLLight.points(pts_l);
        cv::Point2f upper_l = pts_l[2];
        cv::Point2f lower_l = pts_l[3];

        cv::Point2f pts_r[4];
        exRLight.points(pts_r);
        cv::Point2f upper_r = pts_r[1];
        cv::Point2f lower_r = pts_r[0];

        vertex.resize(4);
        vertex[0] = upper_l;
        vertex[1] = upper_r;
        vertex[2] = lower_r;
        vertex[3] = lower_l;

        for ( int i = 0; i < 4; i++ )
        cv::line(srcImg, vertex[i], vertex[(i+1)%4], cvex::GREEN, 1, 8, 0);

        //set armor type
        type = armorType;

        //get front view
        getFrontImg(grayImg);
        rotationScore = rotaScore;//轮换得分

        // calculate the size score
        float normalized_area = contourArea(vertex) / _param.area_normalized_base;//归一化面积
        sizeScore = exp(normalized_area);

        //   calculate the distance score
        Point2f srcImgCenter(grayImg.cols / 2, grayImg.rows / 2);
        float sightOffset = cvex::distance(srcImgCenter, cvex::crossPointOf(array<Point2f, 2>{vertex[0], vertex[2]}, array<Point2f, 2>{vertex[1], vertex[3]}));
        distScore = exp(-sightOffset / _param.sight_offset_normalized_base);

    }

    void ArmorDescriptor::getFrontImg(const Mat& grayImg)
    {
        using cvex::distance;
        const Point2f&
                tl = vertex[0],
                tr = vertex[1],
                br = vertex[2],
                bl = vertex[3];

        int width, height;
        if (type == BIG_ARMOR)//大装甲
        {
            width = 92;
            height = 50;
        }
        else
        {
            width = 50;
            height = 50;
        }

        Point2f src[4]{ Vec2f(tl), Vec2f(tr), Vec2f(br), Vec2f(bl) };
        Point2f dst[4]{ Point2f(0.0, 0.0), Point2f(width, 0.0), Point2f(width, height), Point2f(0.0, height) };
        const Mat perspMat = getPerspectiveTransform(src, dst);
        cv::warpPerspective(grayImg, frontImg, perspMat, Size(width, height));
//        int name_cont = rand();
//        cv::imwrite("/home/zououming/Pictures/train/"+to_string(name_cont)+".png", frontImg);
        cv::imshow("frontImg", frontImg);
        cv::waitKey(1);
    }

 void ArmorDetector::Pre_GetCenter() {
        center_x =(_targetArmor.vertex[1].x-_targetArmor.vertex[0].x)/2+_targetArmor.vertex[0].x+_roi.tl().x;
        center_y =(_targetArmor.vertex[3].y-_targetArmor.vertex[0].y)/2+_targetArmor.vertex[0].y +_roi.tl().y;

        cv::Point2d ss(center_x,center_y);
        circle(_debugImg, ss, 3, Scalar(255, 255, 255), -1);

        Mat prediction = KF->predict();
        Point predict_pt = Point((int)prediction.at<float>(0), (int)prediction.at<float>(1));

        measurement->at<float>(0) = (float)center_x;
        measurement->at<float>(1) = (float)center_y;
        KF->correct(*measurement);

        circle(_debugImg, predict_pt, 3, Scalar(34, 255, 255), -1);

//           circle(_debugImg,cv::Point2f(-(2*(center_x-_roi.tl().x-_targetArmor.vertex[0].x)-_targetArmor.vertex[1].x),
//                                        -(2*(center_y-_roi.tl().y-_targetArmor.vertex[0].y)-_targetArmor.vertex[3].y)), 3, Scalar(0, 0, 255), -1);
        center_x = (int)prediction.at<float>(0);
        center_y = (int)prediction.at<float>(1);

    }
    void  ArmorDetector::Pre_GetViex() {
        vector<Point> intVertex;
//        for (const auto& point : _targetArmor.vertex)
//        {
//            Point fuckPoint = point;
//            intVertex.emplace_back(fuckPoint);
//        }

        intVertex.emplace_back(Point2f(-(2*(center_x-_roi.tl().x-_targetArmor.vertex[0].x)-_targetArmor.vertex[1].x),
                                       -(2*(center_y-_roi.tl().y-_targetArmor.vertex[0].y)-_targetArmor.vertex[3].y)));
        intVertex.emplace_back(Point2f(-(2*(center_x-_roi.tl().x-_targetArmor.vertex[0].x)-_targetArmor.vertex[1].x)+(_targetArmor.vertex[1].x-_targetArmor.vertex[0].x),
                                       -(2*(center_y-_roi.tl().y-_targetArmor.vertex[0].y)-_targetArmor.vertex[3].y)));
        intVertex.emplace_back(Point2f(-(2*(center_x-_roi.tl().x-_targetArmor.vertex[0].x)-_targetArmor.vertex[1].x)+(_targetArmor.vertex[1].x-_targetArmor.vertex[0].x),
                                       -(2*(center_y-_roi.tl().y-_targetArmor.vertex[0].y)-_targetArmor.vertex[2].y)+_targetArmor.vertex[3].y-_targetArmor.vertex[0].y));

        intVertex.emplace_back(Point2f(-(2*(center_x-_roi.tl().x-_targetArmor.vertex[0].x)-_targetArmor.vertex[1].x),
                                       -(2*(center_y-_roi.tl().y-_targetArmor.vertex[0].y)-_targetArmor.vertex[3].y)+_targetArmor.vertex[3].y-_targetArmor.vertex[0].y));


        cvex::showContour(_debugWindowName, _debugImg, _debugImg, intVertex, cvex::GREEN, -1, _roi.tl());
    }
    ArmorDetector::ArmorDetector()
    {
        _flag = ARMOR_NO;
        _roi = Rect(cv::Point(0, 0), _srcImg.size());
        _isTracking = false;




#if defined(DEBUG_DETECTION) || defined(SHOW_RESULT)
        _debugWindowName = "debug info";
#endif // DEBUG_DETECTION || SHOW_RESULT



            KF=new KalmanFilter(stateNum, measureNum, 0);
            measurement=new Mat(cv::Mat::zeros(measureNum, 1, CV_32F));
            KF->transitionMatrix = (Mat_<float>(stateNum, stateNum) << 1, 0, 1, 0,//A 状态转移矩阵
                    0, 1, 0, 1,
                    0, 0, 1, 0,
                    0, 0, 0, 1);
            //这里没有设置控制矩阵B，默认为零

            setIdentity(KF->measurementMatrix);//H=[1,0,0,0;0,1,0,0] 测量矩阵
            setIdentity(KF->processNoiseCov, Scalar::all(1e-5));//Q高斯白噪声，单位阵
            setIdentity(KF->measurementNoiseCov, Scalar::all(1e-1));//R高斯白噪声，单位阵
            setIdentity(KF->errorCovPost, Scalar::all(1));//P后验误差估计协方差矩阵，初始化为单位阵
            randn(KF->statePost, Scalar::all(0), Scalar::all(0.1));//初始化状态为随机值


    }

    ArmorDetector::ArmorDetector(const ArmorParam& armorParam)
    {
        _param = armorParam;
        _flag = ARMOR_NO;
        _roi = Rect(cv::Point(0, 0), _srcImg.size());
        _isTracking = false;

#if defined(DEBUG_DETECTION) || defined(SHOW_RESULT)
        _debugWindowName = "debug info";
#endif // DEBUG_DETECTION || SHOW_RESULT
    }

    void ArmorDetector::init(const ArmorParam& armorParam)
    {
        _param = armorParam;
    }

    void ArmorDetector::loadImg(const cv::Mat& srcImg)			// 读取图片
    {
        _srcImg = srcImg;

#if defined(DEBUG_DETECTION) || defined(SHOW_RESULT)
        _debugImg = srcImg.clone();
#endif // DEBUG_DETECTION || SHOW_RESULT

        Rect imgBound = Rect(cv::Point(0, 0), _srcImg.size());	// 矩形框

        if (_flag == ARMOR_LOCAL && _trackCnt != _param.max_track_num)
        {
            cv::Rect bRect = boundingRect(_targetArmor.vertex) + _roi.tl();
            bRect = cvex::scaleRect(bRect, Vec2f(3, 2));	//以中心为锚点放大2倍
            _roi = bRect & imgBound;
            _roiImg = _srcImg(_roi).clone();
        }
        else
        {
            _roi = imgBound;
            _roiImg = _srcImg.clone();
            _trackCnt = 0;
        }

#ifdef DEBUG_DETECTION
        _roiImg = _srcImg.clone();
        rectangle(_debugImg, _roi, cvex::YELLOW);
#endif // DEBUG_DETECTION
    }


    int ArmorDetector::detect()
    {
        /*
        *	Detect lights and build light bars' desciptors
        */
        _armors.clear();

        std::vector<rm::LightDescriptor> lightInfos;
        {
            /*
            *	pre-treatment
            */
            cv::Mat binBrightImg, hsvImg;//二值化
            cvtColor(_roiImg, hsvImg, COLOR_BGR2HSV);
            cvtColor(_roiImg, _grayImg, COLOR_BGR2GRAY, 1);

            Scalar hsv_lower, hsv_upper;
            int lower[3], upper[3];
            if(_enemy_color == BLUE)
                for ( int i = 0; i < 3; i++ ) {
                    hsv_lower(i) = lower_blue_hsv[i];
                    hsv_upper(i) = upper_blue_hsv[i];
                    lower[i] = lower_blue_hsv[i];
                    upper[i] = upper_blue_hsv[i];
                }
            else
                for ( int i = 0; i < 3; i++ ) {
                    hsv_lower(i) = lower_red_hsv[i];
                    hsv_upper(i) = upper_red_hsv[i];
                    lower[i] = lower_red_hsv[i];
                    upper[i] = upper_red_hsv[i];
                }
            Mat element = getStructuringElement(MORPH_ELLIPSE, cv::Size(3, 3));
            Mat dilate_element = getStructuringElement(MORPH_ELLIPSE, cv::Size(3, 3));
            Mat erode_element = getStructuringElement(MORPH_ELLIPSE, cv::Size(3, 3));

#ifdef DEBUG_HSV
            {
                string adjust_window = "adjust HSV";
                namedWindow(adjust_window);

                createTrackbar("lower H", adjust_window, &lower[0], 180);
                createTrackbar("lower S", adjust_window, &lower[1], 255);
                createTrackbar("lower V", adjust_window, &lower[2], 255);
                createTrackbar("upper H", adjust_window, &upper[0], 180);
                createTrackbar("upper S", adjust_window, &upper[1], 255);
                createTrackbar("upper V", adjust_window, &upper[2], 255);

                createTrackbar("dilate size", adjust_window, &kernel_size[0], 20);
                createTrackbar("erode size", adjust_window, &kernel_size[1], 20);

                while (1) {
                    hsv_lower(0) = getTrackbarPos("lower H", adjust_window);
                    hsv_lower(1) = getTrackbarPos("lower S", adjust_window);
                    hsv_lower(2) = getTrackbarPos("lower V", adjust_window);
                    hsv_upper(0) = getTrackbarPos("upper H", adjust_window);
                    hsv_upper(1) = getTrackbarPos("upper S", adjust_window);
                    hsv_upper(2) = getTrackbarPos("upper V", adjust_window);

                    if(_enemy_color == BLUE)
                        for ( int i = 0; i < 3; i++ ) {
                            lower_blue_hsv[i] = hsv_lower(i);
                            upper_blue_hsv[i] = hsv_upper(i);
                        }
                    else
                        for ( int i = 0; i < 3; i++ ) {
                            lower_red_hsv[i] = hsv_lower(i);
                            upper_red_hsv[i] = hsv_upper(i);
                        }

                    kernel_size[0] = getTrackbarPos("dilate size", adjust_window);
                    kernel_size[1] = getTrackbarPos("erode size", adjust_window);

                    inRange(hsvImg, hsv_lower, hsv_upper, binBrightImg);
                    if (_enemy_color == RED) {
                        Mat red;
                        hsv_lower(0) = red_H[0];
                        hsv_upper(0) = red_H[1];
                        inRange(hsvImg, hsv_lower, hsv_upper, red);
                        bitwise_or(red, binBrightImg, binBrightImg);
                    }

                    if (kernel_size[1] >= 3) {
                        erode_element = getStructuringElement(MORPH_ELLIPSE, Size(kernel_size[1], kernel_size[1]));
                        erode(binBrightImg, binBrightImg, erode_element);
                    }
                    if (kernel_size[0] >= 3) {
                        dilate_element = getStructuringElement(MORPH_ELLIPSE, Size(kernel_size[0], kernel_size[0]));
                        dilate(binBrightImg, binBrightImg, dilate_element);
                    }
                    imshow("hsv", binBrightImg);

                    vector<RotatedRect> lightsRecs;
                    vector<vector<Point>> lightContours;                    // 寻找轮廓
                    cv::findContours(binBrightImg.clone(), lightContours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
                    for (const auto &contour : lightContours) {
                        float lightContourArea = contourArea(contour);        // 面积判断
                        if (contour.size() <= 5 ||
                            lightContourArea < _param.light_min_area)
                            continue;

                        RotatedRect lightRec = fitEllipse(contour);
                        adjustRec(lightRec, ANGLE_TO_UP);

                        //float solidity = lightContourArea / lightRec.size.area();
                        if (lightRec.size.width / lightRec.size.height > _param.light_max_ratio ||
                            lightContourArea / lightRec.size.area() < _param.light_contour_min_solidity)
                            continue;    // 长宽比判断

                        //Mat temp;
                        //cvex::showRectangle("light_right_position", _srcImg, temp, lightRec, cvex::GREEN,0, _roi.tl());

                        lightRec.size.width *= _param.light_color_detect_extend_ratio;
                        lightRec.size.height *= _param.light_color_detect_extend_ratio;
                        Rect lightRect = lightRec.boundingRect();

                        const Rect srcBound(Point(0, 0), _roiImg.size());
                        lightRect &= srcBound;
                        Mat lightImg = _roiImg(lightRect);
                        Mat lightMask = Mat::zeros(lightRect.size(), CV_8UC1);
                        Point2f lightVertexArray[4];//tuoyuan 4个顶点
                        lightRec.points(lightVertexArray);
                        std::vector<Point> lightVertex;
                        for (int i = 0; i < 4; i++) {
                            lightVertex.emplace_back(Point(lightVertexArray[i].x - lightRect.tl().x,
                                                           lightVertexArray[i].y - lightRect.tl().y));
                        }
                        fillConvexPoly(lightMask, lightVertex, 255);
                        if (lightImg.size().area() <= 0 || lightMask.size().area() <= 0) continue;
                        cv::dilate(lightMask, lightMask, element);
//                        lightInfos.push_back(LightDescriptor(lightRec));
                        lightsRecs.emplace_back(lightRec);
                    }
                    Mat debug = _srcImg.clone();
                    cvex::showRectangles("light", debug, debug  , lightsRecs, cvex::MAGENTA, -1, _roi.tl());
                    int Key = waitKey(1);
                    if (Key == 27) {
                        destroyWindow(adjust_window);
                        break;
                    }
                }
            }
#else
            inRange(hsvImg, hsv_lower, hsv_upper, binBrightImg);
                if( _enemy_color == RED ){
                    Mat red;
                    hsv_lower(0) = red_H[0];
                    hsv_upper(0) = red_H[1];
                    inRange(hsvImg, hsv_lower, hsv_upper, red);
                    bitwise_or(red, binBrightImg, binBrightImg);
                }
#endif

//            cvtColor(_roiImg, _grayImg, COLOR_BGR2GRAY, 1);
//            cv::threshold(_grayImg, binBrightImg, _param.brightness_threshold, 255, cv::THRESH_BINARY);
//
//            cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));//tuoyuan
//            dilate(binBrightImg, binBrightImg, element);

#ifdef DEBUG_PRETREATMENT
            imshow("brightness_binary", binBrightImg);
			waitKey();
#endif // DEBUG_PRETREATMENT

            /*
            *	find and filter light bars
            */
            vector<vector<Point>> lightContours;					// 寻找轮廓
            cv::findContours(binBrightImg.clone(), lightContours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
            for (const auto& contour : lightContours)
            {
                float lightContourArea = contourArea(contour);		// 面积判断
                if (contour.size() <= 5 ||
                    lightContourArea < _param.light_min_area) continue;

                RotatedRect lightRec = fitEllipse(contour);
                adjustRec(lightRec, ANGLE_TO_UP);

                //float solidity = lightContourArea / lightRec.size.area();
                if (lightRec.size.width / lightRec.size.height > _param.light_max_ratio ||
                    lightContourArea / lightRec.size.area() < _param.light_contour_min_solidity) continue;	// 长宽比判断

                //Mat temp;
                //cvex::showRectangle("light_right_position", _srcImg, temp, lightRec, cvex::GREEN,0, _roi.tl());

                lightRec.size.width *= _param.light_color_detect_extend_ratio;
                lightRec.size.height *= _param.light_color_detect_extend_ratio;
                Rect lightRect = lightRec.boundingRect();


                const Rect srcBound(Point(0, 0), _roiImg.size());
                lightRect &= srcBound;
                Mat lightImg = _roiImg(lightRect);
                Mat lightMask = Mat::zeros(lightRect.size(), CV_8UC1);
                Point2f lightVertexArray[4];//tuoyuan 4个顶点
                lightRec.points(lightVertexArray);
                std::vector<Point> lightVertex;
                for (int i = 0; i < 4; i++)
                {
                    lightVertex.emplace_back(Point(lightVertexArray[i].x - lightRect.tl().x,
                                                   lightVertexArray[i].y - lightRect.tl().y));
                }
                fillConvexPoly(lightMask, lightVertex, 255);

                if (lightImg.size().area() <= 0 || lightMask.size().area() <= 0) continue;
                cv::dilate(lightMask, lightMask, element);
                const Scalar meanVal = mean(lightImg, lightMask);

                //Mat debugColorImg = _srcImg.clone();
                //const String BGR = "(" + std::to_string(int(meanVal[0])) + ", " + std::to_string(int(meanVal[1])) + ", " + std::to_string(int(meanVal[2])) + ")";

                if (((_enemy_color == BLUE) && (meanVal[BLUE] - meanVal[RED] > 20.0)) || (_enemy_color == RED && meanVal[RED] - meanVal[BLUE] > 20.0))
                {
                    lightInfos.push_back(LightDescriptor(lightRec));

                    //putText(debugColorImg, BGR, Point(lightVertexArray[0]) + _roi.tl(), FONT_HERSHEY_SIMPLEX, 0.4, cvex::GREEN, 1); //fontScalar 0.34
                }
                //else
                //{
                //	putText(debugColorImg, BGR, Point(lightVertexArray[0]) + _roi.tl(), FONT_HERSHEY_SIMPLEX, 0.4, cvex::CYAN, 1); //fontScalar 0.34
                //}
                //imshow("BGR", debugColorImg);
                //waitKey(0);

            }

#ifdef DEBUG_DETECTION
            vector<RotatedRect> lightsRecs;
			for (auto& light : lightInfos)
			{
				lightsRecs.emplace_back(light.rec());
			}
//			cvex::showRectangles(_debugWindowName, _debugImg, _debugImg, lightsRecs, cvex::MAGENTA, 1, _roi.tl());
#endif //DEBUG_DETECTION

            if (lightInfos.empty())
            {
                //usleep(200);
                return _flag = ARMOR_NO;
            }
        }

        /*
        *	find and filter light bar pairs
        */
        {
            sort(lightInfos.begin(), lightInfos.end(), [](const LightDescriptor& ld1, const LightDescriptor& ld2)
            {
                return ld1.center.x < ld2.center.x;
            });
            vector<int> minRightIndices(lightInfos.size(), -1);
            for (size_t i = 0; i < lightInfos.size(); i++)
            {
                for (size_t j = i + 1; (j < lightInfos.size()); j++)
                {
                    const LightDescriptor& leftLight = lightInfos[i];
                    const LightDescriptor& rightLight = lightInfos[j];

#ifdef DEBUG_DETECTION
                    Mat pairImg = _srcImg.clone();
					vector<RotatedRect> curLightPair{ leftLight.rec(), rightLight.rec() };
					cvex::showRectangles("debug pairing", pairImg, pairImg, curLightPair, cvex::CYAN, 0, _roi.tl());
#endif // DEBUG_DETECTION

                    /*
                    *	Works for 2-3 meters situation
                    *	morphologically similar: // parallel
                                     // similar height
                    */
                    float angleDiff_ = abs(leftLight.angle - rightLight.angle);
                    float LenDiff_ratio = abs(leftLight.length - rightLight.length) / max(leftLight.length, rightLight.length);
                    if (angleDiff_ > _param.light_max_angle_diff_ ||
                        LenDiff_ratio > _param.light_max_height_diff_ratio_)
                    {
#ifdef DEBUG_DETECTION
                        cout << "angleDiff: " << angleDiff_ << endl;
                        cout << "LenDiff_ratio: " << LenDiff_ratio << endl;
#endif
                        continue;
                    }

                    /*
                    *	proper location: // y value of light bar close enough
                    *			 // ratio of length and width is proper
                    */
                    float dis = cvex::distance(leftLight.center, rightLight.center);
                    float meanLen = (leftLight.length + rightLight.length) / 2;
                    float yDiff = abs(leftLight.center.y - rightLight.center.y);
                    float yDiff_ratio = yDiff / meanLen;
                    float xDiff = abs(leftLight.center.x - rightLight.center.x);
                    float xDiff_ratio = xDiff / meanLen;
                    float ratio = dis / meanLen;
                    if (yDiff_ratio > _param.light_max_y_diff_ratio_ ||
                        xDiff_ratio < _param.light_min_x_diff_ratio_ ||
                        ratio > _param.armor_max_aspect_ratio_ ||
                        ratio < _param.armor_min_aspect_ratio_)
                    {
#ifdef DEBUG_DETECTION
                        cout << "yDiff_ratio: " << yDiff_ratio << "  xDiff_ratio: " << xDiff_ratio << endl;
                        cout << "ratio: " << ratio << endl;
#endif
                        continue;
                    }

                    // calculate pairs' info
                    int armorType = ratio > _param.armor_big_armor_ratio ? BIG_ARMOR : SMALL_ARMOR;
                    // calculate the rotation score
                    float ratiOff = (armorType == BIG_ARMOR) ? max(_param.armor_big_armor_ratio - ratio, float(0)) : max(_param.armor_small_armor_ratio - ratio, float(0));
                    float yOff = yDiff / meanLen;
                    float rotationScore = -(ratiOff * ratiOff + yOff * yOff);

                    ArmorDescriptor armor(leftLight, rightLight, armorType, _grayImg, rotationScore, _param, _srcImg);
                    _armors.emplace_back(armor);
                    break;
                }
            }

#ifdef  DEBUG_DETECTION
            vector<vector<Point>> armorVertexs;
			for (const auto& armor : _armors)
			{
				vector<Point> intVertex;
				for (const auto& point : armor.vertex)
				{
					intVertex.emplace_back(Point(point.x, point.y));
				}
				armorVertexs.emplace_back(intVertex);
			}
			Mat result = _srcImg.clone();
			cvex::showContours("result", result, _debugImg, armorVertexs, cvex::GREEN, -1, _roi.tl());
#endif //  DEBUG_DETECTION

            if (_armors.empty())
            {
                cout << "ARMOR_NO!!!" << endl;
                //cv::imshow(_debugWindowName, _debugImg);
                //cv::waitKey(1);
                return _flag = ARMOR_NO;
            }
        }

#ifdef GET_ARMOR_PIC
        _allCnt++;
		int i = 0;
		for (const auto& armor : _armors)
		{
			Mat regulatedFrontImg = armor.frontImg;
			if (armor.type == BIG_ARMOR)
			{
				regulatedFrontImg = regulatedFrontImg(Rect(21, 0, 50, 50));
			}
			imwrite("/home/nvidia/Documents/ArmorTrainingSample/" + to_string(_allCnt) + "_" + to_string(i) + ".bmp", regulatedFrontImg);
			i++;
		}
#endif // GET_ARMOR_PIC

        //delete the fake armors
        _armors.erase(remove_if(_armors.begin(), _armors.end(), [](ArmorDescriptor& i)
        {
            return !(i.isArmorPattern());
        }), _armors.end());

        if (_armors.empty())
        {
            _targetArmor.clear();

            if (_flag == ARMOR_LOCAL)
            {
                cout << "Tracking lost" << endl;
                return _flag = ARMOR_LOST;
            }
            else
            {
                cout << "No armor pattern detected." << endl;
                return _flag = ARMOR_NO;
            }
        }

        //calculate the final score
        for (auto& armor : _armors)
        {
            armor.finalScore = armor.sizeScore + armor.distScore + armor.rotationScore;
        }

        //choose the one with highest score, store it on _targetArmor
        std::sort(_armors.begin(), _armors.end(), [](const ArmorDescriptor& a, const ArmorDescriptor& b)
        {
            return a.finalScore > b.finalScore;
        });
//        _targetArmor = _armors[0];


//        Kalman4f();


        _targetArmor = _armors[0];

        //update the flag status
        _trackCnt++;
            Pre_GetCenter();



#if defined(DEBUG_DETECTION) || defined(SHOW_RESULT)
        Pre_GetViex();
//        cv::imshow(_debugWindowName, _debugImg);
//        cv::waitKey(0);
#endif //DEBUG_DETECTION || SHOW_RESULT

        return _flag = ARMOR_LOCAL;
    }

    void ArmorDetector::Kalman4f() {

//        for(int i=0;i<4;i++){
//
//            center_x =_armors[0].vertex[i].x+_roi.tl().x;
//            center_y =_armors[0].vertex[i].y +_roi.tl().y;
//
//
////            circle(_debugImg, cv::Point2d (center_x,center_y), 3, Scalar(255, 255, 255), -1);
//
//            prediction[i] = KF[i]->predict();
//            predict_pt[i] = Point((int)prediction[i].at<float>(0), (int)prediction[i].at<float>(1));
//
//            measurement[i]->at<float>(0) = (float)center_x;
//            measurement[i]->at<float>(1) = (float)center_y;
//            KF[i]->correct(*measurement[i]);
//
////            circle(_debugImg, predict_pt[i], 3, Scalar(34, 255, 255), -1);
//
//            center_x = (int)prediction[i].at<float>(0);
//            center_y = (int)prediction[i].at<float>(1);
//
//            _armors[0].vertex[i]=cv::Point2d (center_x,center_y);
//        }
//        _targetArmor = _armors[0];

    }

    bool ArmorDescriptor::isArmorPattern() const
    {
//            // cut the central part of the armor
//            Mat regulatedImg;
//            if(type == BIG_ARMOR)
//            {
//                regulatedImg = frontImg(Rect(21, 0, 50, 50));
//            }
//            else
//            {
//                regulatedImg = frontImg;
//            }
//
//            resize(regulatedImg,regulatedImg, Size(regulatedImg.size().width / 2, regulatedImg.size().height / 2));
//            // copy the data to make the matrix continuous
//            Mat temp;
//            regulatedImg.copyTo(temp);
//            Mat data = temp.reshape(1, 1);
//
//            data.convertTo(data, CV_32FC1);
//
//            Ptr<SVM> svm = StatModel::load<SVM>("/home/bazinga/CLionProjects/RM_nbut2020/ArmorDetector/SVM3.xml");
//
//
//            int result = (int)svm->predict(data);
//            if(result == 1) return true;
//            else return false;


        // to test the svm, uncomment the code block above
        // and comment the code below
        return true;
    }


    const std::vector<cv::Point2f> ArmorDetector::getArmorVertex() const
    {
        vector<cv::Point2f> realVertex;
//        for (int i = 0; i < 4; i++)
//        {
//            realVertex.emplace_back(Point2f(_targetArmor.vertex[i].x + _roi.tl().x,
//                                            _targetArmor.vertex[i].y + _roi.tl().y));


        realVertex.emplace_back(Point2f(-(2*(center_x-_roi.tl().x-_targetArmor.vertex[0].x)-_targetArmor.vertex[1].x),
                                       -(2*(center_y-_roi.tl().y-_targetArmor.vertex[0].y)-_targetArmor.vertex[3].y)));
        realVertex.emplace_back(Point2f(-(2*(center_x-_roi.tl().x-_targetArmor.vertex[0].x)-_targetArmor.vertex[1].x)+(_targetArmor.vertex[1].x-_targetArmor.vertex[0].x),
                                       -(2*(center_y-_roi.tl().y-_targetArmor.vertex[0].y)-_targetArmor.vertex[3].y)));
        realVertex.emplace_back(Point2f(-(2*(center_x-_roi.tl().x-_targetArmor.vertex[0].x)-_targetArmor.vertex[1].x)+(_targetArmor.vertex[1].x-_targetArmor.vertex[0].x),
                                       -(2*(center_y-_roi.tl().y-_targetArmor.vertex[0].y)-_targetArmor.vertex[2].y)+_targetArmor.vertex[3].y-_targetArmor.vertex[0].y));

        realVertex.emplace_back(Point2f(-(2*(center_x-_roi.tl().x-_targetArmor.vertex[0].x)-_targetArmor.vertex[1].x),
                                       -(2*(center_y-_roi.tl().y-_targetArmor.vertex[0].y)-_targetArmor.vertex[3].y)+_targetArmor.vertex[3].y-_targetArmor.vertex[0].y));
//        }

        return realVertex;

    }

    int ArmorDetector::getArmorType() const
    {
        return _targetArmor.type;
    }

#if defined(DEBUG_DETECTION) || defined(SHOW_RESULT)
    void ArmorDetector::showDebugImg() const
    {
        imshow(_debugWindowName, _debugImg);
        waitKey(1);
    }
#endif // DEBUG_DETECTION || SHOW_RESULT
}
