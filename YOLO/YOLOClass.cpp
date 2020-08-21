#include "darknet.h"
#include "YOLOClass.h"
using namespace cv;

YOLOClass::YOLOClass(){}

YOLOClass::YOLOClass(std::string _cfgFile, std::string _weightFile, std::string _classFile, float _thresh)
{
    cfg_file = _cfgFile;
    weight_file = _weightFile;
    class_file = _classFile;
    thresh = _thresh;
    nms = 0.45;

    std::ifstream class_names_file(class_file);
    if(class_names_file.is_open()){
        std::string class_name = "";
        while(getline(class_names_file, class_name))
            class_names_vec.push_back(class_name);
    }
    class_num = class_names_vec.size();

    net = *load_network_custom((char*)cfg_file.c_str(), (char*)weight_file.c_str(), 0, 1);
    calculate_binary_weights(net);
}


image YOLOClass::matToImage(cv::Mat &mat)
{
    Mat rgb_img;
    cvtColor(mat, rgb_img, cv::COLOR_RGB2BGR);
    int w = mat.cols;
    int h = mat.rows;
    int c = mat.channels();
    image img = make_image(w, h, c);
    unsigned char *data = (unsigned char *)rgb_img.data;
    int step = mat.step;
    for (int y = 0; y < h; ++y) {
        for (int k = 0; k < c; ++k) {
            for (int x = 0; x < w; ++x) {
                img.data[k*w*h + y*w + x] = data[y*step + x*c + k] / 255.0f;
            }
        }
    }
    return img;
}


std::vector<cv::Rect> YOLOClass::get_boxes(cv::Mat &img)
{
    image Image = matToImage(img);
    image sized = resize_image(Image, net.w, net.h);
    network_predict(net, sized.data);

    int box_num = 0;
    detection* det = get_network_boxes(&net, img.cols, img.rows, thresh, 0, 0, 1, &box_num, 0);

    layer l = net.layers[net.n - 1];
    if(nms) {
        if(l.nms_kind == DEFAULT_NMS)
            do_nms_sort(det, box_num, l.classes, nms);
        else
            diounms_sort(det, box_num, l.classes, nms, l.nms_kind, l.beta_nms);
    }

    detect_boxes.clear();
    detect_classes.clear();
    std::cout<<box_num<<std::endl;
    for(int i = 0; i < box_num; ++i){
        float max_thresh = -1;
        int class_numbering = -1;
        for(int j = 0; j < class_num; ++j){
            if(det[i].prob[j] > thresh && det[i].prob[j] > max_thresh)
            {
                class_numbering = j;
                max_thresh = det[i].prob[j];
            }
        }

        if(class_numbering >= 0){
            int left = (det[i].bbox.x - det[i].bbox.w / 2.) * img.cols;
            int right = (det[i].bbox.x + det[i].bbox.w / 2.) * img.cols;
            int top = (det[i].bbox.y - det[i].bbox.h / 2.) * img.rows;
            int bottom = (det[i].bbox.y + det[i].bbox.h / 2.) * img.rows;

            left = max(0, left);
            right = min(right, img.cols - 1);
            top = max(0, top);
            bottom = min(bottom, img.rows - 1);

            Rect box(left, top, abs(right - left), abs(top - bottom));
            detect_boxes.push_back(box);
            detect_classes.push_back(class_names_vec[class_numbering]);
        }
    }
    free_detections(det, box_num);

    return detect_boxes;
}


std::vector<std::string> YOLOClass::get_class()
{
    return detect_classes;
}
