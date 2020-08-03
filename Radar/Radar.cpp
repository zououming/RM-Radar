#include "Radar.h"
using namespace std;

Radar::Radar() {
    this->YOLOv3 = new yoloApi();
    this->armor_detector = new rm::ArmorDetector;
}

Radar::Radar(rm::ArmorDetector &armorDetector) {
    this->YOLOv3 = new yoloApi();
    this->armor_detector = &armorDetector;
}

Radar::~Radar() {
    delete [] this->YOLOv3;
}

void Radar::load_img(const cv::Mat &srcImg) {
    _srcImg = srcImg;
}

void Radar::find_robot() {
    YOLO_box.clear();
    YOLO_class.clear();
    robot_box.clear();
    trackers = MultiTracker::create();

    YOLO_box = YOLOv3->get_boxes(_srcImg);
    YOLO_class = YOLOv3->get_class();

    if (!YOLO_box.size())
        return;
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
        robot_box.emplace_back(robot);
        trackers->add(cv::TrackerKCF::create(), _srcImg, robot.position);
    }
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
    _roiImg = _srcImg.clone();
    vector<Rect_<double>> new_position = trackers->getObjects();

    cv::Scalar line_color(0, 0, 0);
    string team, text;
    for (int i = 0; i < robot_box.size(); i++) {
        robot_box[i].position = new_position[i];
        line_color = robot_box[i].team == rm::RED ? Scalar(0, 0, 255): Scalar(255, 0, 0);
        team = robot_box[i].team == rm::RED ? "red" : "blue";

        text = team + " " + robot_box[i].arms;
        rectangle(_roiImg, robot_box[i].position, line_color, 2, 1);

        Point text_point(int(robot_box[i].position.x), int(robot_box[i].position.y - 2));
        putText(_roiImg, text, text_point, FONT_HERSHEY_SIMPLEX, 0.8, line_color, 2);
    }
    deal = true;
}

Mat Radar::getLastImg() {
    return _roiImg;
}