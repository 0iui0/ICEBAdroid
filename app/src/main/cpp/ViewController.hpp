#ifndef VINS_MOBILE_ANDROIDPORT_VIEWCONTROLLER_H
#define VINS_MOBILE_ANDROIDPORT_VIEWCONTROLLER_H

#include <queue>
#import <sys/utsname.h>
#include <opencv2/opencv.hpp>

// added in the continous process of tranlating objective c code
#include <condition_variable> // std::condition_variable con
#ifdef __ANDROID__
#include <android/log.h>
#include <jni.h>
#include <android/looper.h>
#include <android/sensor.h>

#define LOG_TAG "ViewController.cpp"
#define LOGI(...)  __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...)  __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
#define printf(...)  __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
#else
#define LOGI(...)
#define LOGE(...)
#define printf(...)
#endif
typedef double NSTimeInterval;
#define APPNAME "VINS_Android"
#include <thread>

using namespace std;
using namespace cv;

extern "C" {
    #include <time.h>
}



class ViewController {


private:
    // singleton for static callbacks
    static ViewController* instance;
    std::thread mainLoop;

public:
    ViewController();
    ~ViewController();

    void testMethod(){
        LOGI("Testmethod is working");
    }

    std::mutex viewUpdateMutex;
    std::string tvXText;
    std::string tvYText;
    std::string tvZText;
    std::string tvFeatureText;
    std::string tvTotalText{"TOTAL:"};
    std::string tvBufText;
    std::string tvLoopText{"LOOP:"};
    bool initImageVisible = true;


    void viewDidLoad();


    /**
     * Takes RGBA doesnt change the format!
     */
    void processImage(cv::Mat& image, long timeStamp, bool isScreenRotated=false);

};


#endif //VINS_MOBILE_ANDROIDPORT_VIEWCONTROLLER_H
