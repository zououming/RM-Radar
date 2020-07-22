#include "yoloApi.h"
using namespace cv;

yoloApi::yoloApi(){
    cfg_file = "/home/zououming/darknet/cfg/yolov3.cfg";
    weight_file = "/home/zououming/darknet/yolov3.weights";
    class_file = "/home/zououming/darknet/data/coco.names";
    class_num = 80;
    thresh = 0.75;
    nms = 0.35;

    std::ifstream class_names_file(class_file);
    if(class_names_file.is_open()){
        std::string class_name = "";
        while(getline(class_names_file, class_name))
            class_names_vec.push_back(class_name);
    }

    net = load_network((char*)cfg_file.c_str(), (char*)weight_file.c_str(), 0);
    set_batch_network(net, 1);
}

yoloApi::~yoloApi(){
    delete [] net;
}

std::vector<cv::Rect> yoloApi::get_boxes(cv::Mat &img){
    Mat rgb_img;
    cvtColor(img, rgb_img, cv::COLOR_BGR2RGB);

    size_t src_size = rgb_img.rows * rgb_img.cols * 3 * sizeof(float);
    auto* src_img = new float[src_size];
    img_convert(img, src_img);    //将图像转为yolo格式

    size_t resize_size = net->w * net->h * 3 * sizeof(float);
    auto* resize_img = new float[resize_size];
    img_resize(src_img, resize_img, img.cols, img.rows, net->w, net->h);

    network_predict(net, resize_img); //网络推理
    int box_num = 0;
    detection* det = get_network_boxes(net, rgb_img.cols, rgb_img.rows, thresh, 0.5, 0, 1, &box_num);

    if(nms)
        do_nms_sort(det, box_num, class_num, nms);

    detect_boxes.clear();

    for(int i = 0; i < box_num; i++){
        bool flag = false;
        for(int j = 0; j < class_num; j++)
            if(det[i].prob[j] > thresh)
                if(!flag)
                    flag = true;

        if(flag){
            int left = (det[i].bbox.x - det[i].bbox.w / 2.) * img.cols;
            int right = (det[i].bbox.x + det[i].bbox.w / 2.) * img.cols;
            int top = (det[i].bbox.y - det[i].bbox.h / 2.) * img.rows;
            int bot = (det[i].bbox.y + det[i].bbox.h / 2.) * img.rows;

            left = max(0, left);
            right = min(img.cols - 1, right);
            top = max(0, top);
            bot = min(img.rows - 1, bot);

            Rect box(left, top, fabs(left - right), fabs(top - bot));
            detect_boxes.push_back(box);
        }
    }
    free_detections(det, box_num);

#ifdef SHOW_BOX
    std::cout << detect_boxes.size() << std::endl;
    for (auto &box : detect_boxes)
        rectangle(img, box, Scalar(0, 255, 0), 1);
    imshow("show_box", img);
    waitKey(1);
#endif

    delete [] src_img;
    delete [] resize_img;
    return detect_boxes;
}

void yoloApi::img_convert(const cv::Mat &img, float *dst){
    uchar* data = img.data;

    for(int k = 0; k < img.channels(); k++)
        for(int i = 0; i < img.rows; i++)
            for(int j = 0; j < img.cols; j++)
                dst[k * img.cols * img.rows + i * img.cols + j] = data[(i * img.cols+j) * img.channels() + k]/255.;
}

void yoloApi::img_resize(float *src, float *dst, int srcWidth, int srcHeight, int dstWidth, int dstHeight){
    int new_w = srcWidth;
    int new_h = srcHeight;
    if (((float)dstWidth / srcWidth) < ((float)dstHeight / srcHeight)) {
        new_w = dstWidth;
        new_h = (srcHeight * dstWidth) / srcWidth;
    }
    else{
        new_h = dstHeight;
        new_w = (srcWidth * dstHeight) / srcHeight;
    }

    size_t sizeInner = new_w * new_h * 3 * sizeof(float);
    auto *ImReInner = new float[sizeInner];
    resize_inner(src, ImReInner, srcWidth, srcHeight, new_w, new_h);

    for(int i = 0; i < dstWidth * dstHeight * 3; i++)
        dst[i] = 0.5;

    for(int k = 0; k < 3; k++)
        for(int y = 0; y < new_h; y++)
            for(int x = 0; x < new_w; x++) {
                float val = ImReInner[k * new_w * new_h + y * new_w + x];
                dst[k * dstHeight * dstWidth + ((dstHeight - new_h) / 2 + y) * dstWidth + (dstWidth - new_w) / 2 + x] = val;
            }
    delete [] ImReInner;
}

void yoloApi::resize_inner(float *src, float *dst, int srcWidth, int srcHeight, int dstWidth, int dstHeight){
    size_t sizePa = dstWidth * srcHeight * 3 * sizeof(float);
    auto * part = new float[sizePa];

    float w_scale = (float)(srcWidth - 1) / (dstWidth - 1);
    float h_scale = (float)(srcHeight - 1) / (dstHeight - 1);

    for(int k = 0; k < 3; k++){
        for(int r = 0; r < srcHeight; r++){
            for(int c = 0; c < dstWidth; c++){
                float val = 0;
                if(c == dstWidth-1 || srcWidth == 1)
                    val = src[k * srcWidth * srcHeight + r * srcWidth + srcWidth - 1];
                else {
                    float sx = c * w_scale;
                    int ix = (int) sx;
                    float dx = sx - ix;
                    val=(1 - dx) * src[k * srcWidth * srcHeight + r * srcWidth + ix]
                        + dx * src[k * srcWidth * srcHeight + r * srcWidth + ix + 1];
                }
                part[k * srcHeight * dstWidth + r * dstWidth + c] = val;
            }
        }
    }

    for(int k = 0; k < 3; k++){
        for(int r = 0; r < dstHeight; r++){
            float sy = r * h_scale;
            int iy = (int) sy;
            float dy = sy - iy;
            for(int c = 0; c < dstWidth; c++){
                float val = (1 - dy) * part[k * dstWidth * srcHeight + iy * dstWidth + c];
                dst[k * dstWidth * dstHeight + r * dstWidth + c] = val;
            }
            if(r == dstHeight-1 || srcHeight == 1)
                continue;
            for(int c = 0; c < dstWidth; ++c){
                float val = dy * part[k * dstWidth * srcHeight + (iy + 1) * dstWidth + c];
                dst[k * dstWidth * dstHeight + r * dstWidth + c] += val;
            }
        }
    }
    delete [] part;
}