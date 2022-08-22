package com.example.iceba;

import android.view.Surface;

import java.io.Serializable;
import java.nio.ByteBuffer;

public class ICEBA implements Serializable {

    // Used to load the 'native-lib' library on application startup.
    static { System.loadLibrary("NativeLib"); }

    /**
     * @description: slam初始化，最先调用
     * @param: folderPath 参数文件的文件夹目录，要求绝对路径
     * @return: int 0为正常，其余的看错误列表
     */
    public native int init(String folderPath);


    //imu 在初始化完成后，每次从传感器里面取到一次imu数据调用一次该函数
    public native int pushImu(byte[] data);

    /**
     * @description: image 在初始化完成后，每次从传感器里面取到一次图像数据调用一次该函数
     * @param: data 传进来的图像数据
     * @return: int 0为正常，其余的看错误列表
     */
    public native int pushImage(byte[] data);

    public void onPoseCallback(float[] R, float[] T){
//        for(int i = 0; i < R.length; i++){
//            Log.d("lalala", "================R["+ i +"]:" + R[i]);
//        }
//        for(int i = 0; i < T.length; i++){
//            Log.d("lalala", "================T["+ i +"]:" + T[i]);
//        }
    }
    
    /**
     * @description: 获取轨迹图像，主要是前期使用
     * @param: data 传进来的buf,将会在里面填写图像数据，大小400*400，格式是BGRA8888
     * @return: int 0为正常，其余的看错误列表
     */
    public native int getTrajectoryImg(byte[] data);

    //for debug
    public native void onImageAvailable(int width, int height, ByteBuffer bufferY,
                                               Surface surface, long timeStamp,
                                               boolean isScreenRotated, float virtualCamDistance);

//    public native void setSurface(int width, int height, Surface surface);
    



    public native int release();



}
