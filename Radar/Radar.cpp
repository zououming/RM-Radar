#include "Radar.h"
using namespace std;


Radar::Radar() {
    this->YOLOv3 = new YoloApi();
    this->armor_detector = new rm::ArmorDetector;
    _srcMap = imread("../Radar/map.png");
}


Radar::Radar(rm::ArmorDetector *armorDetector, YoloApi *YOLOv3) {
    this->YOLOv3 = YOLOv3;
    this->armor_detector = armorDetector;
    _srcMap = imread("../Radar/map.png");
}


Radar::~Radar() {
    delete [] this->YOLOv3;
}


void Radar::set_enemy_color(int enemyColor){
    _enemyColor = enemyColor;
}


void Radar::load_img(const cv::Mat &srcImg) {
    _srcImg = srcImg;
}


int Radar::find_robot() {
    deal = false;
    YOLO_box.clear();
    YOLO_class.clear();
    robot_box.clear();
    trackers = MultiTracker::create();

    YOLO_box = YOLOv3->get_boxes(_srcImg);
    YOLO_class = YOLOv3->get_class();

    if (YOLO_box.empty())
        return 0;
#ifdef GPU
    gpuSrcImg.upload(_srcImg);
    cuda::cvtColor(gpuSrcImg, gpuGrayImg, COLOR_BGR2GRAY, 1);
    gpuGrayImg.download(_grayImg);
#else
    cvtColor(_srcImg, _grayImg, COLOR_BGR2GRAY, 1);
#endif GPU

    armor_detector->loadImg(_srcImg, _grayImg);

    for (int i = 0; i < YOLO_box.size(); i++) {
        box_fix(YOLO_box[i]);
        RobotDescriptor robot = armor_detector->detect(YOLO_box[i]);
        robot.team = YOLO_class[i] == "blue robot" ? rm::BLUE : rm::RED;
        robot.color = YOLO_class[i] == "blue robot" ? Scalar(255, 0, 0): Scalar(0, 0, 255);
        robot_box.emplace_back(robot);
        trackers->add(cv::TrackerKCF::create(), _srcImg, robot.position);
    }
    return robot_box.size();
}


void Radar::box_fix(cv::Rect &rect) {
    if (rect.x + rect.width > _srcImg.size[1] - 1)
        rect.x = _srcImg.size[1] - 1 - rect.width;
    if (rect.y + rect.height > _srcImg.size[0] - 1)
        rect.y = _srcImg.size[0] - 1 - rect.height;
}


void Radar::track() {
    if (trackers->empty())
        return;
    trackers->update(_srcImg);
    _showImg = _srcImg.clone();
    vector<Rect_<double>> new_position = trackers->getObjects();

    string team, text;
    for (int i = 0; i < robot_box.size(); i++) {
        if (_enemyColor == robot_box[i].team || _enemyColor == rm::GREEN) {
            robot_box[i].position = new_position[i];
            team = robot_box[i].team == rm::RED ? "red" : "blue";

            text = team + " " + robot_box[i].arms;
            cv::rectangle(_showImg, robot_box[i].position, robot_box[i].color, 2, 1);

            Point text_point(int(robot_box[i].position.x), int(robot_box[i].position.y - 2));
            cv::putText(_showImg, text, text_point, FONT_HERSHEY_SIMPLEX, 0.8, robot_box[i].color, 2);
        }
    }
    deal = true;
    map_transformation();
}


void Radar::map_transformation(){
    cv::Mat map_point_mat;
    for (auto &robot : robot_box) {
        if (_enemyColor == robot.team || _enemyColor == rm::GREEN) {
            double center[] = {double(robot.position.x + robot.position.width / 2),
                               double(robot.position.y + robot.position.height / 2), 1};
            cv::Mat box_point_mat(3, 1, CV_64FC1, center);
            map_point_mat = _transformation_Mat * box_point_mat;

            Point map_point(int(map_point_mat.at<double>(0)), int(map_point_mat.at<double>(1)));
            robot.map_position = map_point;
        }
    }
}


void Radar::draw_map(){
    _showMap = _srcMap.clone();
    int circle_radius = int(_srcMap.size[1]/35);
    for (auto &robot : robot_box) {
        if (robot.map_position.x == -1)
            continue;

        cv::circle(_showMap, robot.map_position, circle_radius, robot.color, -1);

        if (robot.arms != "robot") {
            Point2i text_point(int(robot.map_position.x - circle_radius / 2), int(robot.map_position.y + circle_radius / 2));
            cv::putText(_showMap, to_string(robot.numbering), text_point, FONT_HERSHEY_SIMPLEX, 0.8,
                        Scalar(255, 255, 255), 2);
        }
    }
}


void Radar::window_get_points(int event, int x, int y, int flags, void* param){
    auto* Points = (std::vector<Point2f>*) param;

    if(event == CV_EVENT_LBUTTONDOWN) {
        printf("(%d, %d)\n", x, y);
        Points->emplace_back(x, y);
    }
}


void Radar::get_transformation_mat(){
    Mat showImg = _srcImg.clone();
    Mat showMap = _srcMap.clone();
    std::vector<Point2f> srcPoints, mapPoints;
    int key;
    while(1) {
        showImg = _srcImg.clone();
        cv::setMouseCallback("src", window_get_points, (void*)&srcPoints);
        if (!srcPoints.empty()){
            for (int i = 0; i < srcPoints.size(); i++) {
                cv::circle(showImg, srcPoints[i], 3, Scalar(0, 255, 0), -1);
                cv::putText(showImg, to_string(i+1), srcPoints[i], FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 255, 255), 2);
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

    while(1) {
        showMap = _srcMap.clone();
        cv::setMouseCallback("map", window_get_points, (void*)&mapPoints);
        if (!mapPoints.empty()){
            for (int i = 0; i < mapPoints.size(); i++) {
                cv::circle(showMap, mapPoints[i], 3, Scalar(0, 255, 0), -1);
                cv::putText(showMap, to_string(i+1), mapPoints[i], FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 255, 255), 2);
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

    _transformation_Mat = getAffineTransform(srcPoints, mapPoints);
    double add_array[] = {0, 0, 1};
    Mat add_vec = Mat(1, 3, CV_64FC1, add_array);
    cv::vconcat(_transformation_Mat, add_vec, _transformation_Mat);
}


Mat Radar::getLastImg() {
    return _showImg;
}


Mat Radar::getLastMap() {
    return _showMap;
}


std::vector<RobotDescriptor> Radar::getRobotBox(){
    return robot_box;
}


cv::Mat Radar::operator+(Radar &radar1){
    for (auto &robot : radar1.getRobotBox())
        this->robot_box.emplace_back(robot);
    this->draw_map();
    return this->getLastMap();
}