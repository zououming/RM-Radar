#include "Radar.h"
using namespace std;

Radar::Radar()
{
    this->YOLO = new YOLOClass();
    this->armor_detector = new rm::ArmorDetector;
}


Radar::Radar(rm::ArmorDetector *armorDetector, YOLOClass *YOLO, int enemyColor)
{
    this->YOLO = YOLO;
    this->armor_detector = armorDetector;
    this->enemy_color = enemyColor;
}


Radar::Radar(rm::ArmorDetector *armorDetector, YOLOClass *YOLO, int enemyColor, int trackRobotFrame)
{
    this->YOLO = YOLO;
    this->armor_detector = armorDetector;
    this->enemy_color = enemyColor;
    this->track_robot_frame = trackRobotFrame;
}


void Radar::run(const cv::Mat &srcImg) {
    _srcImg = srcImg;
    if(frame_count++ % track_robot_frame == 0) {
        int num = findRobot();
        frame_count = 0;
    }
    trackRobot();
    mapTransformation();
}


int Radar::findRobot()
{
    deal = false;
    YOLO_box.clear();
    YOLO_class.clear();
    robot_box.clear();
    track_roboters = cv::MultiTracker::create();

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
        track_roboters->add(cv::TrackerKCF::create(), _srcImg, robot.position);
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
    if (track_roboters->empty())
        return;
    track_roboters->update(_srcImg);
    _showImg = _srcImg.clone();
    vector<cv::Rect_<double>> new_position = track_roboters->getObjects();

    string team, text;
    for (int i = 0; i < robot_box.size(); ++i){
        if (enemy_color == robot_box[i].team || enemy_color == rm::GREEN){
            robot_box[i].position = new_position[i];
            team = robot_box[i].team == rm::RED ? "red" : "blue";

            text = team + " " + robot_box[i].arms;
            cv::rectangle(_showImg, robot_box[i].position, robot_box[i].color, 2, 1);

            cv::Point text_point(int(robot_box[i].position.x), int(robot_box[i].position.y - 2));
            cv::putText(_showImg, text, text_point, cv::FONT_HERSHEY_SIMPLEX, 0.8, robot_box[i].color, 2);
        }
    }
    deal = true;
}


void Radar::mapTransformation()
{
    cv::Mat map_point_mat;
    for (auto &robot : robot_box){
        if (enemy_color == robot.team || enemy_color == rm::GREEN){
            double center[] = {double(robot.position.x + robot.position.width / 2),
                               double(robot.position.y + robot.position.height / 2), 1};
            cv::Mat box_point_mat(3, 1, CV_64FC1, center);
            map_point_mat = _transformationMat * box_point_mat;

            cv::Point map_point(int(map_point_mat.at<double>(0)), int(map_point_mat.at<double>(1)));
            robot.map_position = map_point;
        }
    }
    drawMap();
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
        Points->emplace_back(x, y);
        printf("point %d: (%d, %d)\n", Points->size(), x, y);
    }
}


bool Radar::getTransformationMat(cv::Mat& img, cv::Mat& map)
{
    _srcImg = img.clone();
    _srcMap = map.clone();

    cv::Mat showImg = _srcImg.clone();
    cv::Mat showMap = _srcMap.clone();
    std::vector<cv::Point2f> srcPoints, mapPoints;
    int key;
    printf("Click three dots on src.\n");
    while(1){
        showImg = _srcImg.clone();
        cv::setMouseCallback("src", windowGetPoints, (void*)&srcPoints);
        drawPoints(srcPoints, showImg, "src");
        key = cv::waitKey(1);
        if (key == 'q' && !srcPoints.empty()){
            printf("Delete point %d\n", srcPoints.size());
            srcPoints.pop_back();
        }
        else if (key == 'r') {
            printf("Delete all points\n");
            srcPoints.clear();
        }
        else if (key == 13 || srcPoints.size() == 3) { //回车则退出
            drawPoints(srcPoints, showImg, "src");
            break;
        }
    }

    printf("Click the positions of the just three points in turn on the map.\n");

    while(1){
        showMap = _srcMap.clone();
        cv::setMouseCallback("map", windowGetPoints, (void*)&mapPoints);
        drawPoints(mapPoints, showMap, "map");
        key = cv::waitKey(1);
        if (key == 'q' && !mapPoints.empty()) {
            printf("Delete point %d\n", mapPoints.size());
            mapPoints.pop_back();
        }
        else if (key == 'r') {
            printf("Delete all points\n");
            mapPoints.clear();
        }
        else if (key == 13 || mapPoints.size() == 3) { //回车则退出
            drawPoints(mapPoints, showMap, "map");
            break;
        }
    }
    cv::destroyWindow("src");
    cv::destroyWindow("map");

    if(srcPoints.size() != 3 || mapPoints.size() != 3) {
        printf("Both pictures must have three points!\n");
        return false;
    }

    _transformationMat = getAffineTransform(srcPoints, mapPoints);
    double add_array[] = {0, 0, 1};
    cv::Mat add_vec = cv::Mat(1, 3, CV_64FC1, add_array);
    cv::vconcat(_transformationMat, add_vec, _transformationMat);
    std::cout << "transformation mat:\n"<< _transformationMat << std::endl;

    return true;
}


void Radar::drawPoints(std::vector<cv::Point2f> points, cv::Mat img, std::string window_name){
    if (!points.empty()){
        for (int i = 0; i < points.size(); ++i){
            cv::circle(img, points[i], 3, cv::Scalar(0, 255, 0), -1);
            cv::putText(img, to_string(i+1), points[i], cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
        }
    }
    cv::imshow(window_name, img);
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


bool Radar::isHandled(){
    return this->deal;
}


void Radar::setFlag(){
    this->deal = false;
}