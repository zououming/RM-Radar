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
#pragma once
#include<opencv2/opencv.hpp>
#include<array>
#include"General.h"
#include<opencv2/ml.hpp>
#include "../Settings/Settings.hpp"
#include<string>
using namespace std;

/**************************************************************
 * DEBUG_PRETREATMENT 	only shows the image after the simple pretreatment	// 预处理
 * DEBUG_DETECTION	record the info of:
 * 				    	// roi area						: yellow			// roi的面积
 * 					// all the possible light bars				: magenta	// 所有可能的灯条
 * 					// the possible light pairs(DEBUG_PAIR overlapped)	: cyan	  // 可能的灯条对
 *					// the vertex of possible armor areas			: white		  // 顶点装甲板面积
 * 					// the vertex of result armor areas			: green			  // 顶点装甲板面积的结果
 * SHOW_RESULT		only shows the detection result(show the vertex of armors)		: green
 * GET_ARMOR_PIC	collect the samples of armor area
 *
 * Notice:		1. all the macro definition can be used individually			// 所有的宏定义都可以单独使用
 * 			2. if you want to focus on particular part of processing, I suggest commenting all the
 * 			   unrelated part of DEBUG_DETECTION in .cpp. Or you can just rewrite the debug interactive mode.
 * 			// 如果您希望关注处理的特定部分，我建议在.cpp中注释所有与DEBUG_DETECTION无关的部分。
 *			   或者您可以重写调试交互模式。
 * 			3. remeber to change the path of pictures if using GET_ARMOR_PIC // 注意修改图片路径
 **************************************************************/
//#define DEBUG_PRETREATMENT
#define DEBUG_DETECTION
#define SHOW_RESULT
//#define GET_ARMOR_PIC
#define DEBUG_HSV

namespace rm
{

    /*
    *	This struct store all parameters(svm excluded) of armor detection
    *	// 该结构存储装甲检测的所有参数(svm除外)
    */
    struct ArmorParam			// 装甲板参数
    {
        //Pre-treatment			// 预处理
        int brightness_threshold;		// 亮度阈值
        int color_threshold;			// 颜色阈值
        float light_color_detect_extend_ratio; 	// 灯条放大比例

        //Filter lights					// 过滤灯条？
        float light_min_area;			// 灯条最小面积
        float light_max_angle;			// 灯条最大面积
        float light_min_size;			// 灯条最小尺寸
        float light_contour_min_solidity;// 灯条轮廓最小密度？
        float light_max_ratio;			// 灯条最大比率

        //Filter pairs					// 灯条对
        float light_max_angle_diff_;	// 灯条最大角度差值
        float light_max_height_diff_ratio_; // hdiff / max(r.length, l.length)
        float light_max_y_diff_ratio_;  // ydiff / max(r.length, l.length)
        float light_min_x_diff_ratio_;		// x方向最大比率？

        //Filter armor					// 装甲板
        float armor_big_armor_ratio;	// 最大比率
        float armor_small_armor_ratio;	// 最小比率
        float armor_min_aspect_ratio_;	// 最小高宽比
        float armor_max_aspect_ratio_;	// 最大高宽比

        //other params					// 其他参数
        float sight_offset_normalized_base;// 视线偏移归一化？
        float area_normalized_base;		// 面积归一化
        int enemy_color;				// 敌人颜色
        int max_track_num = 3000;		// 最大轨道数量

        /*
        *	@Brief: 为各项参数赋默认值
        */
        ArmorParam()
        {
             string config_file_name = "../Settings/param_config.xml";
            Settings set_Armor(config_file_name);
            //pre-treatment
            brightness_threshold =set_Armor.brightness_threshold;						// 亮度阈值 = 210
            color_threshold = set_Armor.color_threshold;							// 颜色阈值 = 40
            light_color_detect_extend_ratio = set_Armor.light_color_detect_extend_ratio;			// 灯条放大倍数 = 1.1

            // Filter lights
            light_min_area = set_Armor.light_min_area;							// 最小面积 = 10
            light_max_angle =set_Armor.light_max_angle;							// 最大面积 = 45
            light_min_size = set_Armor.light_min_size;							// 最小尺寸 = 5.0 ？
            light_contour_min_solidity = set_Armor.light_contour_min_solidity;				// 最小轮廓密度 = 0.5 ？
            light_max_ratio = set_Armor.light_max_ratio;						// 最大轮廓密度 = 1.0
            /* 个人猜测是轮廓内颜色占比，确定敌方装甲板 */

            // Filter pairs
            light_max_angle_diff_ = set_Armor.light_max_angle_diff_; //20			// 灯条最大角度差 = 7.0
            light_max_height_diff_ratio_ = set_Armor.light_max_height_diff_ratio_; //0.5	// 灯条最小角度差 = 0.2
            light_max_y_diff_ratio_ = set_Armor.light_max_y_diff_ratio_; //100
            light_min_x_diff_ratio_ = set_Armor.light_min_x_diff_ratio_; //100

            // Filter armor
            armor_big_armor_ratio = set_Armor.armor_big_armor_ratio;					// 装甲板最大比率 = 3.2
            armor_small_armor_ratio = set_Armor.armor_small_armor_ratio;					// 装甲板最小比率 = 2
            //armor_max_height_ = 100.0;					// 装甲板最大高度 = 100
            //armor_max_angle_ = 30.0;						// 装甲板最小高度 = 30
            armor_min_aspect_ratio_ = set_Armor.armor_min_aspect_ratio_;					// 装甲板最小宽高比 = 1.0
            armor_max_aspect_ratio_ = set_Armor.armor_max_aspect_ratio_;					// 装甲板最大宽高比 = 5.0

            //other params
            sight_offset_normalized_base = set_Armor.sight_offset_normalized_base;				// 视线偏移标准 = 200 ？
            area_normalized_base = set_Armor.area_normalized_base;					// 角度归一化 = 1000 ？
            enemy_color = set_Armor.enemy_color;								// 地方颜色 = 蓝色
        }
    };

    /*
    *   This class describes the info of lights, including angle level, width, length, score
    *   // 这个类描述灯光的信息，包括角度水平，宽度，长度，分数
    */
    class LightDescriptor
    {
    public:
        LightDescriptor() {};							// 构造函数
        /*****************************
         * @ Brief： 重载构造函数 // 为矩形重新赋值
         * @ param：
         * 		cv::RotatedRect& light // 输入旋转矩形
         * ***************************/
        LightDescriptor(const cv::RotatedRect& light)
        {
            width = light.size.width;		// 宽度
            length = light.size.height;		// 高度
            center = light.center;			// 中心
            angle = light.angle;			// 角度
            area = light.size.area();		// 面积
        }
        const LightDescriptor& operator =(const LightDescriptor& ld)
        {
            this->width = ld.width;
            this->length = ld.length;
            this->center = ld.center;
            this->angle = ld.angle;
            this->area = ld.area;
            return *this;
        }

        /*
        *	@Brief: return the light as a cv::RotatedRect object
        *	// 返回一个cv::RotatedRect对象
        */
        cv::RotatedRect rec() const
        {
            return cv::RotatedRect(center, cv::Size2f(width, length), angle);
        }

    public:
        float width;
        float length;
        cv::Point2f center;
        float angle;
        float area;
    };

    /*
    * 	@Brief： This class describes the armor information, including maximum bounding box, vertex and so on
    *	// 这个类描述装甲信息，包括最大边界框、顶点等等
    */
    class ArmorDescriptor
    {
    public:
        /*
        *	@Brief: Initialize with all 0
        */
        ArmorDescriptor();	// 构造函数，初始化全部为0

        /*
        *	@Brief: calculate the rest of information(except for match&final score)of ArmroDescriptor based on:
                l&r light, part of members in ArmorDetector, and the armortype(for the sake of saving time)
                // 计算ArmroDescriptor剩余的信息(不包括match和final score)基于:
                // 蓝色和红色灯条，ArmorDetector中的部件，装甲板类型(为了节省时间)
        *	@Calls: ArmorDescriptor::getFrontImg()	// 调用函数： getFrontImg()
        */
        ArmorDescriptor(const LightDescriptor& lLight, const LightDescriptor& rLight, const int armorType, const cv::Mat& srcImg, const float rotationScore, ArmorParam param, cv::Mat Img);

        /*
        *	@Brief: empty the object				// 清空对象
        *	@Called :ArmorDetection._targetArmor	// 调用函数： targetArmor
        */
        void clear()
        {
            rotationScore = 0;			// 目标得分
            sizeScore = 0;				// 尺寸得分
            distScore = 0;				// 距离得分
            finalScore = 0;				// 最终得分
            for (int i = 0; i < 4; i++)	// 四个顶点全部（0， 0）
            {
                vertex[i] = cv::Point2f(0, 0);
            }
            type = UNKNOWN_ARMOR;		// 类型为未找到
        }

        /*
        *	@Brief: get the front img(prespective transformation) of armor(if big, return the middle part)
        *	// 获取装甲板透视变换前的图像
        *	@Inputs: grayImg of roi
        *	// 输入: 感兴趣区域的灰度图
        *	@Outputs: store the front img to ArmorDescriptor's public
        */
        void getFrontImg(const cv::Mat& grayImg);

        /*
        *	@Return: if the centeral pattern belong to an armor
        *	// 判断是否为装甲板
        */
        bool isArmorPattern() const;

    public:
        // 定义长度为2， RotatedRect 类型的数组
        std::array<cv::RotatedRect, 2> lightPairs; //0 left, 1 right
        float sizeScore;		//S1 = e^(size)
        float distScore;		//S2 = e^(-offset)
        float rotationScore;		//S3 = -(ratio^2 + yDiff^2)
        float finalScore;

        std::vector<cv::Point2f> vertex; //four vertex of armor area, lihgt bar area exclued!!装甲区四个顶点
        cv::Mat frontImg; //front img after prespective transformation from vertex,1 channel gray img

        //	0 -> small
        //	1 -> big
        //	-1 -> unkown
        int type;
    };

    /*
    *	This class implement all the functions of detecting the armor
    *	// 这个类实现了检测装甲的所有功能
    */
    class ArmorDetector
    {
    public:
        /*
        *	flag for the detection result
        *	// 标记检测结果
        */
        enum ArmorFlag
        {
            ARMOR_NO = 0,		// not found			// 没有找到
            ARMOR_LOST = 1,		// lose tracking		// 跟丢了
            ARMOR_GLOBAL = 2,	// armor found globally	// 全局找
            ARMOR_LOCAL = 3		// armor found locally(in tracking mode)
        };

    public:
        ArmorDetector();		// 构造函数
        ArmorDetector(const ArmorParam& armorParam);
        ~ArmorDetector() {}		// 析构函数

        /*
        *	@Brief: Initialize the armor parameters
        *			// 初始化参数
        *	@Others: API for client
        */
        void init(const ArmorParam& armorParam);

        /*
        *	@Brief: set the enemy's color
        *			// 设定敌方颜色
        *	@Others: API for client
        */
        void setEnemyColor(int enemy_color)
        {
            _enemy_color = enemy_color; //init
            _self_color = enemy_color == BLUE ? RED : BLUE;
        }

        /*
        *	@Brief: load image and set tracking roi
        *			// 读取图片，设定追踪区域
        *	@Input: frame
        *	@Others: API for client
        */
        void loadImg(const cv::Mat& srcImg);

        /*
        *	@Brief: core of detection algrithm, include all the main detection process
        *			// 检测算法的核心，包括所有主要的检测过程
        *	@Outputs: ALL the info of detection result
        *			// 所有检测结果的信息
        *	@Return: See enum ArmorFlag
        *	@Others: API for client
        */
        int detect();

        /*
        *	@Brief: get the vertex of armor
        *			// 得到装甲板的顶点
        *	@Return: vector of four cv::point2f object
        * 			// 返回四个坐标
        *	@Notice: Order->left-top, right-top, right-bottom, left-bottom
        *			// 左上， 右上， 右下， 左下
        *	@Others: API for client
        */
        const std::vector<cv::Point2f> getArmorVertex() const;

        /*
        *	@Brief: returns the type of the armor
        *			// 返回装甲板类型 0小， 1大
        *	@Return: 0 for small armor, 1 for big armor
        *	@Others: API for client
        */
        int getArmorType() const;
        void Pre_GetCenter();
        void Pre_GetViex();
        void Kalman4f();

#if defined(DEBUG_DETECTION) || defined(SHOW_RESULT)
        void showDebugImg() const;
#endif // DEBUG_DETECTION || SHOW_RESULT

    private:				// 私有
        ArmorParam _param;	// 参数
        int _enemy_color;	// 敌人颜色
        int _self_color;	// 自己颜色

        cv::Rect _roi; //relative coordinates // 相关坐标

        cv::Mat _srcImg; //source img		// 原图像
        cv::Mat _roiImg; //roi from the result of last frame // 最后一帧图像的ROI
        cv::Mat _grayImg; //gray img of roi	// ROI的灰度图

        int _trackCnt = 0;						// 跟踪数：0 ？

        std::vector<ArmorDescriptor> _armors;	// 装甲板

        ArmorDescriptor _targetArmor; //relative coordinates

        int _flag;
        bool _isTracking;

        int stateNum = 4;
        int measureNum = 2;
        cv::KalmanFilter *KF;
        cv::Mat *measurement;
        cv::Mat prediction;
        cv::Point predict_pt;
        float center_x;
        float center_y;

        int lower_blue_hsv[3] = {90, 85, 167}; //hsv
        int upper_blue_hsv[3] = {111, 255, 255};
        int lower_red_hsv[3] = {160, 120, 160};
        int upper_red_hsv[3] = {180, 255, 255};
        int red_H[2] = {0, 10};
        int kernel_size[2] = {3, 3};   //dilate_size erode_size

#if defined(DEBUG_DETECTION) || defined(SHOW_RESULT)
        std::string _debugWindowName;
        cv::Mat _debugImg;
#endif // DEBUG_DETECTION || SHOW_RESULT

#ifdef GET_ARMOR_PIC
        int _allCnt = 0;
#endif // GET_ARMOR_PIC

    };

}
