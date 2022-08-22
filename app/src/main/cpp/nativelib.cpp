#include <jni.h>
#include <string>
#include <time.h>
#include "ice-ba/App/EuRoC_test.h"

#define LOG_TAG "nativelip.cpp"

// debug logging
#ifdef __ANDROID__
#include <android/log.h>
#define LOGI(...)  __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...)  __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
#else
#define LOGI(...)
#define LOGE(...)
#endif

#include <ice-ba/App/status.h>
//图像相关
#define WIDTH 640                     //图像宽度
#define WIDTHTWICE WIDTH * 2          //数据里面图像的宽度，数据里面把两张图像粘在一起了
#define HEIGHT 480                    //图像的高度
#define IMGDATALEN WIDTHTWICE *HEIGHT //图像数据的长度

//IMU相关
//45纬度的重力加速度大小，后期要根据纬度进行计算
#define gravityG 9.80665f

//现在的imu设备的加速度原始数据单位是4096LSB/g
//需要的数据单位是m/s^2,需要将取出来的数值除以LSBTOMS2
#define ACCLSB 4096.0f
#define LSBTOMS2 ACCLSB *gravityG

//现在的imu设备的陀螺仪原始数据单位是16.4LSB/°/s
//需要的数据单位是rad/s,需要将取出来的数值除以LSBTORADS
#define GYRLSB 16.4f
#define PI 3.1415926535898f
#define LSBTORADS GYRLSB / 180 * PI

JavaVM* jvm;//java虚拟机
jobject jobj;//全局对象


//初始化的时候会调进来一次，在这个方法里持有jvm的引用
JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM *vm, void *reserved){
    jvm=vm;
    JNIEnv* env = NULL;
    jint result =1;
    if(jvm){
        LOGE("lalala m_vm init success");
    }else{
        LOGE("lalala m_vm init failed");
    }
    if (vm->GetEnv((void**)&env, JNI_VERSION_1_4) != JNI_OK){
        return result;
    }
    return JNI_VERSION_1_4;
}

void onPoseCallback(const float* R, const float* T){
    JNIEnv *env;
    jvm->AttachCurrentThread(&env,0);

    jfloatArray rJava = env->NewFloatArray(4);
    env->SetFloatArrayRegion(rJava,0,4,R);
    jfloatArray tJava = env->NewFloatArray(3);
    env->SetFloatArrayRegion(tJava,0,3,T);

    jclass javaClass = env->GetObjectClass(jobj);

    jmethodID javaCallback = env->GetMethodID(javaClass,"onPoseCallback","([F[F)V");

    env->CallVoidMethod(jobj,javaCallback,rJava,tJava);
    env->DeleteLocalRef(rJava);
    env->DeleteLocalRef(tJava);
    jvm->DetachCurrentThread();
}

// global variable to viewController 
// because its much less of a hassle 
// than to pass pointers to Java and back
char *Jstring2CStr(JNIEnv *env, jstring jstr) {
    char *rtn = NULL;
    jclass clsstring = env->FindClass("java/lang/String");
    jstring strencode = env->NewStringUTF("GB2312");
    jmethodID mid = env->GetMethodID(clsstring, "getBytes", "(Ljava/lang/String;)[B");
    jbyteArray barr = (jbyteArray) env->CallObjectMethod(jstr, mid,
                                                         strencode); // String .getByte("GB2312");
    jsize alen = env->GetArrayLength(barr);
    jbyte *ba = env->GetByteArrayElements(barr, JNI_FALSE);
    if (alen > 0) {
        rtn = (char *) malloc(alen + 1);         //"\0"
        memcpy(rtn, ba, alen);
        rtn[alen] = 0;
    }
    env->ReleaseByteArrayElements(barr, ba, 0);  //
    return rtn;
}

extern "C"
JNIEXPORT jint JNICALL
Java_com_example_slam_iceba_init(JNIEnv *env, jobject clazz, jstring folder_path) {
    LOGE("lalala Java_com_example_slam_iceba_init");
    jobj=env->NewGlobalRef(clazz);
    if(jobj==NULL)
        LOGE("lalala init gJobject null");
    setCallback((void*)onPoseCallback);

    char *folder = Jstring2CStr(env, folder_path);
    std::string folder_str=folder;
    if (folder_str.empty())//文件夹路径空
        return STATUS::PARA_FOLDER_EMPTY;

    if(folder_str[folder_str.size()-1]!='/')
        folder_str+="/";

    int status=init(folder_str.c_str());
    if(status!=STATUS::OK){
        LOGE("lalala init fail %s status %d", folder_str.c_str(),status);
        return status;
    }

    status=run_iceba();
    if(status!=STATUS::OK)
        return status;

    return STATUS::OK;
}
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_slam_iceba_pushImu(JNIEnv *env, jobject clazz, jbyteArray data) {
    jbyte *jdataP = env->GetByteArrayElements(data, nullptr);

    long long gyroT;
    memcpy(&gyroT, jdataP + 44, 8);
//    LOGE("lalala gyroT timestamp is %lld",gyroT);
    int16_t gyroX, gyroY, gyroZ;
    memcpy(&gyroX, jdataP + 60, 2);
    memcpy(&gyroY, jdataP + 64, 2);
    memcpy(&gyroZ, jdataP + 68, 2);
    float gyrRAD[3];
    gyrRAD[0] = (float)gyroX / LSBTORADS;
    gyrRAD[1] = (float)gyroY / LSBTORADS;
    gyrRAD[2] = (float)gyroZ / LSBTORADS;

    long long accT;
    memcpy(&accT, jdataP + 72, 8);
    int16_t accX, accY, accZ;
    memcpy(&accX, jdataP + 88, 2);
    memcpy(&accY, jdataP + 92, 2);
    memcpy(&accZ, jdataP + 96, 2);
    float accMS2[3];
    accMS2[0] = (float)accX / LSBTOMS2;
    accMS2[1] = (float)accY / LSBTOMS2;
    accMS2[2] = (float)accZ / LSBTOMS2;
//    LOGE("lalala acc X  %f  Y  %f  Z  %f", accXMS2,accYMS2,accZMS2);
    //获取地磁计数据
    // long long magT;
    // memcpy(&magT, jdataP + 100, 8);
    // int16_t magX, magY, magZ;
    // memcpy(&magX, jdataP + 116, 2);
    // memcpy(&magY, jdataP + 120, 2);
    // memcpy(&magZ, jdataP + 124, 2);
    // LOGE("lalala magT  %lld ", magT);
    // LOGE("lalala magX  %d ", magX);
    // LOGE("lalala magY  %d ", magY);
    // LOGE("lalala magZ  %d ", magZ);


    int status=pushImu(gyrRAD,accMS2,accT);

    env->ReleaseByteArrayElements(data, jdataP, JNI_FALSE);

    if(status!=STATUS::OK)
        return status;
    return STATUS::OK;
}
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_slam_iceba_pushImage(JNIEnv *env, jobject clazz, jbyteArray data) {
    jbyte *jdataP = env->GetByteArrayElements(data, nullptr);
    long long timestamp;
    memcpy(&timestamp, jdataP + IMGDATALEN, 8);

    if (timestamp == 0){
        LOGE("lalala in this frame image timestamp is %lld", timestamp);
        return TIMESTAMP_ZERO;
    }
    //convet image to show
    cv::Mat image(HEIGHT, WIDTHTWICE, CV_8UC1, jdataP);


    cv::Mat imgL = image(cv::Rect(0, 0, WIDTH, HEIGHT));
    cv::Mat imgR = image(cv::Rect(WIDTH, 0, WIDTH, HEIGHT));

    int status=pushStereoImg(imgL,imgR,timestamp);

    env->ReleaseByteArrayElements(data, jdataP, JNI_FALSE);

    if(status!=STATUS::OK)
        return status;
    return STATUS::OK;
}
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_slam_iceba_getTrajectoryImg(JNIEnv *env, jobject clazz, jbyteArray data) {
    jbyte *jdataP = env->GetByteArrayElements(data, nullptr);
    cv::Mat image(400,400,CV_8UC4,jdataP);
    getTrajectoryImg(image);
    env->ReleaseByteArrayElements(data, jdataP, JNI_FALSE);

    return STATUS::OK;
}
extern "C"
JNIEXPORT jint JNICALL
Java_com_example_slam_iceba_release(JNIEnv *env, jobject clazz) {
    int status=release();

    if(status!=STATUS::OK)
        return status;
}

#include <android/native_window.h>
#include <android/native_window_jni.h>
#include <opencv2/opencv.hpp>

extern "C"
JNIEXPORT void JNICALL
Java_com_example_slam_iceba_onImageAvailable(JNIEnv *env, jobject type,
                                                                             jint width, jint height,
                                                                             jobject bufferY,
                                                                             jobject surface,
                                                                             jlong timeStamp,
                                                                             jboolean isScreenRotated,
                                                                             jfloat virtualCamDistance) {

    long timeStemp=timeStamp+11666666;


    uint8_t *srcLumaPtr = reinterpret_cast<uint8_t *>(env->GetDirectBufferAddress(bufferY));

    if (srcLumaPtr == nullptr) {
        LOGE("blit NULL pointer ERROR");
        return;
    }


    ANativeWindow *win = ANativeWindow_fromSurface(env, surface);

    ANativeWindow_acquire(win);
    ANativeWindow_Buffer buf;


    ANativeWindow_setBuffersGeometry(win, width, height, 0);

    if (int32_t err = ANativeWindow_lock(win, &buf, NULL)) {
        LOGE("ANativeWindow_lock failed with error code %d\n", err);
        ANativeWindow_release(win);
        return;
    }

//    LOGI("buf.stride: %d", buf.stride);
    uint8_t *dstPtr = reinterpret_cast<uint8_t *>(buf.bits);
    cv::Mat dstRgba(height, buf.stride, CV_8UC4, dstPtr); // TextureView buffer, use stride as width
    cv::Mat gray(height, width, CV_8UC1, srcLumaPtr);

    cv::Mat img_in_raw;
    getSmoothImg(img_in_raw);
    if(!img_in_raw.empty()){
        cv::Mat resize;
        cv::resize(img_in_raw,resize,dstRgba.size());
        cv::cvtColor(resize, dstRgba, cv::COLOR_GRAY2RGBA);
        cv::Mat trajectory;
        getTrajectoryImg(trajectory);
        if(!trajectory.empty()){
            cv::Mat trac_resize;
            cv::resize(trajectory,trac_resize,cv::Size(200,200));
            cv::cvtColor(trac_resize, trac_resize, cv::COLOR_BGR2RGBA);

            //弄到一起
            cv::Mat imageROI;
            imageROI = dstRgba(cv::Rect(0,dstRgba.rows - trac_resize.rows, trac_resize.cols,trac_resize.rows));
            trac_resize.copyTo(imageROI);
        }

    }
    ANativeWindow_unlockAndPost(win);
    ANativeWindow_release(win);
}

//jobject* _surface;
//int sWidth;
//int sHeight;

//extern "C"
//JNIEXPORT void JNICALL
//Java_com_example_slam_iceba_setSurface(JNIEnv *env, jobject thiz, jint width, jint height,
//                                          jobject surface) {
//    _surface=&surface;
//}