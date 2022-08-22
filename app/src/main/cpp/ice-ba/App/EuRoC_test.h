//
// Created by cy-desktop on 21-4-28.
//

#ifndef ICE_BA_ANDROID_1_EUROC_TEST_H
#define ICE_BA_ANDROID_1_EUROC_TEST_H

#include <opencv2/core.hpp>


//初始化一些参数
int init(const char* paraFolder);

void setCallback(void* fun);

//运行slam系统
int run_iceba();

int pushImu(const float* gyr, const float* acc, uint64_t timestamp);
int pushStereoImg(const cv::Mat& imgL, const cv::Mat& imgR, uint64_t timestamp);

void onPoseCallback(const float* R, const float* T);

void getSmoothImg(cv::Mat& img);
void getTrajectoryImg(cv::Mat& tra);//主要是debug用

int release();

// class ICEBA{
//     ICEBA(); //get an instance

//     //初始化一些参数
//     int init(const char *paraFolder);

//     //释放资源，整个slam框架结束的时候使用
//     int release();

//     //设置回调函数
//     void setPoseCallback(void *fun);
//     void setPlaneCallback(void *fun);//将检测的平面信息给上层应用


//     //运行slam系统
//     int run_iceba();

//     int pushImu(const float *gyr, const float *acc, uint64_t timestamp);
//     int pushStereoImg(const cv::Mat &imgL, const cv::Mat &imgR, uint64_t timestamp);

//     void onPoseCallback(const float *R, const float *T);//将姿态返回给上层应用

//     //返回平面法像量，中心点，边界信息(可选,因为关于边界的判定需要的内容应该由我这边写)
//     void onPlaneCallback(const float *V, const float *P, const float* bouder=NULL, int size=0 );//将平面信息返回给上层应用



//     void getSmoothImg(cv::Mat &img);//获取原始平滑图像
//     void getTrajectoryImg(void *tra); //主要是debug用
//     void getVirtualImage(void *img);//也是用来debug用的，用于显示虚拟物体,除非上层应用没有时间，否着不实现这个功能

//     //图像

//     //锁

//     //buf

//     //last state
//     IBA::CameraIMUState last_state;//由vio得到的相机最新状态的pvqb

// };

#endif //ICE_BA_ANDROID_1_EUROC_TEST_H
