#include "Radar.h"
using namespace std;

Radar::Radar()
{
    this->YOLO = new YOLOClass();
    this->armor_detector = new rm::ArmorDetector;
    _srcMap = cv::imread("../Radar/map.png");
}

Radar::Radar(rm::ArmorDetector *armorDetector, YOLOClass *YOLO)
{
    this->YOLO = YOLO;
    this->armor_detector = armorDetector;
    _srcMap = cv::imread("../Radar/map.png");
}


Radar::~Radar()
{
    delete [] this->YOLO;
}


void Radar::setEnemyColor(int enemyColor)
{
    _enemyColor = enemyColor;
}


void Radar::loadImg(const cv::Mat &srcImg)
{
    _srcImg = srcImg;
}

int Radar::findRobot()
{
    deal = false;
    YOLO_box.clear();
    YOLO_class.clear();
    robot_box.clear();
    trackRoboters = cv::MultiTracker::create();

    YOLO_box = YOLO->get_boxes(_srcImg);
    YOLO_class = YOLO->get_class();
    if (YOLO_box.empty())
        return 0;
#ifdef GPU
    gpuSrcImg.upload(_srcImg);
    cv::cuda::cvtColor(gpuSrcImg, gpuGrayImg, cv::COLOR_BGR2GRAY, 1);
    gpuGrayImg.download(_grayImg);
#else
    cvtColor(_srcImg, _grayImg, COLOR_BGR2GRAY, 1);
#endif GPU

    armor_detector->loadImg(_srcImg, _grayImg);

    for (int i = 0; i < YOLO_box.size(); ++i)
    {
        boxFix(YOLO_box[i]);
        RobotDescriptor robot = armor_detector->detect(YOLO_box[i]);
        robot.team = YOLO_class[i] == "blue robot" ? rm::BLUE : rm::RED;
        robot.color = YOLO_class[i] == "blue robot" ? cv::Scalar(255, 0, 0): cv::Scalar(0, 0, 255);
        robot_box.emplace_back(robot);
        trackRoboters->add(cv::TrackerKCF::create(), _srcImg, robot.position);
    }
    return robot_box.size();
}


void Radar::boxFix(cv::Rect &rect)
{
    if (rect.x + rect.width > _srcImg.size[1] - 1)
        rect.x = _srcImg.size[1] - 1 - rect.width;
    if (rect.y + rect.height > _srcImg.size[0] - 1)
        rect.y = _srcImg.size[0] - 1 - rect.height;
}


void Radar::trackRobot()
{
    if (trackRoboters->empty())
        return;
    trackRoboters->update(_srcImg);
    _showImg = _srcImg.clone();
    vector<cv::Rect_<double>> new_position = trackRoboters->getObjects();

    string team, text;
    for (int i = 0; i < robot_box.size(); ++i){
        if (_enemyColor == robot_box[i].team || _enemyColor == rm::GREEN){
            robot_box[i].position = new_position[i];
            team = robot_box[i].team == rm::RED ? "red" : "blue";

            text = team + " " + robot_box[i].arms;
            cv::rectangle(_showImg, robot_box[i].position, robot_box[i].color, 2, 1);

            cv::Point text_point(int(robot_box[i].position.x), int(robot_box[i].position.y - 2));
            cv::putText(_showImg, text, text_point, cv::FONT_HERSHEY_SIMPLEX, 0.8, robot_box[i].color, 2);
        }
    }
    deal = true;
    mapTransformation();
}


void Radar::mapTransformation()
{
    cv::Mat map_point_mat;
    for (auto &robot : robot_box){
        if (_enemyColor == robot.team || _enemyColor == rm::GREEN){
            double center[] = {double(robot.position.x + robot.position.width / 2),
                               double(robot.position.y + robot.position.height / 2), 1};
            cv::Mat box_point_mat(3, 1, CV_64FC1, center);
            map_point_mat = _transformationMat * box_point_mat;

            cv::Point map_point(int(map_point_mat.at<double>(0)), int(map_point_mat.at<double>(1)));
            robot.map_position = map_point;
        }
    }
}


void Radar::drawMap()
{
    _showMap = _srcMap.clone();
    int circle_radius = int(_srcMap.size[1]/40);
    for (auto &robot : robot_box) {
        if (robot.map_position.x == -1)
            continue;

        cv::circle(_showMap, robot.map_position, circle_radius, robot.color, -1);

        if (robot.arms != "robot"){
            cv::Point2i text_point(int(robot.map_position.x - circle_radius / 2), int(robot.map_position.y + circle_radius / 2));
            cv::putText(_showMap, to_string(robot.numbering), text_point, cv::FONT_HERSHEY_SIMPLEX, 0.8,
                        cv::Scalar(255, 255, 255), 2);
        }
    }
}


void Radar::windowGetPoints(int event, int x, int y, int flags, void* param)
{
    auto* Points = (std::vector<cv::Point2f>*) param;

    if(event == CV_EVENT_LBUTTONDOWN){
        printf("(%d, %d)\n", x, y);
        Points->emplace_back(x, y);
    }
}


void Radar::getTransformationMat()
{
    cv::Mat showImg = _srcImg.clone();
    cv::Mat showMap = _srcMap.clone();
    std::vector<cv::Point2f> srcPoints, mapPoints;
    int key;
    printf("Click three dots on src.\n");
    while(1){
        showImg = _srcImg.clone();
        cv::setMouseCallback("src", windowGetPoints, (void*)&srcPoints);
        if (!srcPoints.empty()){
            for (int i = 0; i < srcPoints.size(); i++){
                cv::circle(showImg, srcPoints[i], 3, cv::Scalar(0, 255, 0), -1);
                cv::putText(showImg, to_string(i+1), srcPoints[i], cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
            }
        }
        cv::imshow("src", showImg);
        key = cv::waitKey(1);
        if (key == 'q')
            srcPoints.pop_back();
        else if (key == 'r')
            srcPoints.clear();
        else if (key == 13) //回车则退出
            break;
    }

    printf("Click the positions of the just three points in turn on the map.\n");
    while(1){
        showMap = _srcMap.clone();
        cv::setMouseCallback("map", windowGetPoints, (void*)&mapPoints);
        if (!mapPoints.empty()){
            for (int i = 0; i < mapPoints.size(); ++i){
                cv::circle(showMap, mapPoints[i], 3, cv::Scalar(0, 255, 0), -1);
                cv::putText(showMap, to_string(i+1), mapPoints[i], cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
            }
        }
        cv::imshow("map", showMap);
        key = cv::waitKey(1);
        if (key == 'q')
            mapPoints.pop_back();
        else if (key == 'r')
            mapPoints.clear();
        else if (key == 13)
            break;
    }
    cv::destroyWindow("src");
    cv::destroyWindow("map");

    _transformationMat = getAffineTransform(srcPoints, mapPoints);
    double add_array[] = {0, 0, 1};
    cv::Mat add_vec = cv::Mat(1, 3, CV_64FC1, add_array);
    cv::vconcat(_transformationMat, add_vec, _transformationMat);
}


cv::Mat Radar::getLastImg()
{
    return _showImg;
}


cv::Mat Radar::getLastMap()
{
    return _showMap;
}


std::vector<RobotDescriptor> Radar::getRobotBox()
{
    return robot_box;
}


cv::Mat Radar::operator+(Radar &radar1)
{
    for (auto &robot : radar1.getRobotBox())
        this->robot_box.emplace_back(robot);
    this->drawMap();
    return this->getLastMap();
}