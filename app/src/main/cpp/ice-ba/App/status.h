//
// Created by cy-desktop on 21-4-28.
//

#ifndef ICE_BA_STATUS
#define ICE_BA_STATUS

enum STATUS{
    OK=0,
    
    //出现下面的错误码，不允许运行slam算法
    PARA_FOLDER_EMPTY=-1,//传进来的文件夹路径是空的
    FOLDER_NOT_EXIST=-2,//文件夹不存在
    LOAD_PARA_FAIL=-3,//加载参数失败
    PARASETTING_FILE_NOT_EXIST=-4,//没有参数设置文件
    FILE_NOT_EXIT=-5,//文件不存在
    IMG_NAME_EMPTY=-6,//w图像文件的名字空，估计是离线数据里面没有记录文件名字的文件
    IMU_FILE_NOT_EXIST=-7,//imu文件不存在
    HAVE_NOT_INIT=-8,//没有进行初始化
    WRONG_USE_API=-9,//传进来图像的时候，不可以又单张传又俩张一起传，推荐直接俩张一起传
    IMU_INVALID=-10,//当前imu不可用

    //可以继续运行，但是可能会对精度有影响
    GEN_MASK_FAIL=1,//获取图像掩膜出错，可能是相机内参错误或者和畸变模型不匹配
    TIMESTAMP_ZERO=2,//时间戳为0


    //可以正常运行的一些状态
    WAIT_IMU=3,//等待imu,图像的时间戳比imu早，这张图像不会采用，等待时间戳大于imu的图像
    WAIT_IMG=4//等待图像

};

#endif //ICE_BA_STATUS
