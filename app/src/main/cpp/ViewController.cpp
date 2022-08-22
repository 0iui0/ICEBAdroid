#include <fstream>
#include "ViewController.hpp"
#include "ice-ba/App/EuRoC_test.h"
using namespace std;
ViewController* ViewController::instance = nullptr;

ViewController::ViewController() {
    LOGI("ViewController Constructor");
    this->instance = this;
}

ViewController::~ViewController() {
    LOGI("ViewController Destructor");
}

void ViewController::viewDidLoad() {
    LOGI("lalala viewDidLoad");
    int status = init();
    if (status != 0)
        return;
    mainLoop = std::thread(run_iceba_mono);
}

void ViewController::processImage(cv::Mat &dstRgba, long timeStamp, bool isScreenRotated) {

    //TODO:互斥取图像
    if(!img_in_raw.empty()){
        m_img.lock();

        cv::Mat resize;
        cv::resize(img_in_raw,resize,dstRgba.size());
        cv::cvtColor(resize, dstRgba, cv::COLOR_GRAY2RGBA);
        m_img.unlock();

        if(!trajectory.empty()){
            m_trac.lock();
            cv::Mat trac_resize;
            cv::resize(trajectory,trac_resize,cv::Size(200,200));
            m_trac.unlock();
            cv::cvtColor(trac_resize, trac_resize, cv::COLOR_BGR2RGBA);

            //弄到一起
            cv::Mat imageROI;
            imageROI = dstRgba(cv::Rect(0,dstRgba.rows - trac_resize.rows, trac_resize.cols,trac_resize.rows));
            trac_resize.copyTo(imageROI);
        }


    }
}


