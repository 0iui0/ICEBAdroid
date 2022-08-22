/******************************************************************************
 * Copyright 2017 Baidu Robotic Vision Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "status.h"
#include "EuRoC_test.h"
#include "IBA/IBA.h"
#include "feature_utils.h"
#include "image_utils.h"
#include "xp_quaternion.h"
#include "param.h"  // calib
#include "basic_datatype.h"
#include "iba_helper.h"
#include "pose_viewer.h"
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <glog/logging.h>
#include <opencv2/core.hpp>

#include <algorithm>
#include <string>
#include <fstream>
#include <vector>
#include <thread>
#include <condition_variable>

#define TS(name) int64 t_##name = cv::getTickCount()
#define TE(name)  LOG(INFO) << "TIMER_" #name ": "+std::to_string(1000.*((cv::getTickCount() - t_##name) / cv::getTickFrequency()));

// debug logging
#ifdef __ANDROID__
#include <android/log.h>
#define LOGI(...)  __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...)  __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
#else
#define LOGI(...)
#define LOGE(...)
#endif

namespace fs = boost::filesystem;
using std::string;
using std::vector;
#ifdef __ANDROID__
  #include <ViewController.hpp>
#endif

cv::Mat img_in_smooth;
cv::Mat trajectory;
std::mutex m_img;
std::mutex m_trac;

void getSmoothImg(cv::Mat& img){std::unique_lock<std::mutex> sbguard(m_img, std::try_to_lock);img=img_in_smooth;}
void getTrajectoryImg(cv::Mat&tra){std::unique_lock<std::mutex> sbguard(m_trac, std::try_to_lock);tra=trajectory;}//主要是debug用

string paraFStr;//参数配置文件所在的路径
string imgFolder;
const string paraName = "para.yaml";//参数配置文件的名字
bool dataSet;//是否使用数据集运行
bool FLAGS_stereo;//是否双目
string FLAGS_iba_param_path;//先直接使用txt先，后面也加到yaml里面
string FLAGS_gba_camera_save_path;
bool bInit;
bool stereoApi;
bool monoApi;

//暂时直接使用默认值的一些参数
const int FLAGS_grid_row_num=1;
const int FLAGS_grid_col_num = 1;
const int FLAGS_max_num_per_grid = 70;
const double FLAGS_feat_quality = 0.07;
const double FLAGS_feat_min_dis = 10;
const bool FLAGS_not_use_fast = false;
const int FLAGS_pyra_level = 2;
int FLAGS_start_idx = 0;
int FLAGS_end_idx = -1;
const double FLAGS_uniform_radius = 40;
const int FLAGS_ft_len = 125;
const double FLAGS_ft_droprate = 0.05;
const bool FLAGS_show_feat_only = false;
const int FLAGS_fast_thresh = 10;
const double FLAGS_min_feature_distance_over_baseline_ratio = 4;
const double FLAGS_max_feature_distance_over_baseline_ratio = 3000;

std::thread trackT;
std::thread leftT;
std::thread rightT;
//image name
vector<string> img_file_paths;
vector<string> slave_img_file_paths;
std::condition_variable con;
std::mutex m_imubuf,m_img0buf,m_img1buf,m_stereobuf;
std::list<XP::ImuData> imu_samples;
std::list<std::pair< cv::Mat,float>> leftImgBuf;
std::list<std::pair< cv::Mat,float>> rightImgBuf; 
std::list<std::pair<std::pair< cv::Mat,cv::Mat>,float>> stereoImgBuf; 


std::mutex m_con_mutex;

XP::DuoCalibParam duo_calib_param;
std::vector<cv::Mat_<uchar> > masks(2);

XP::FeatureTrackDetector* feat_track_detector;
XP::ImgFeaturePropagator* slave_img_feat_propagator;

XP::PoseViewer pose_viewer;
Eigen::Matrix4f T_Cl_Cr;
IBA::Solver solver;
Eigen::Vector3f last_position;
//std::mutex m_state;//用来互斥访问last_state
//IBA::CameraIMUState last_state;//由vio得到的相机最新状态的pvqb
//XP::ImuData pre_imu;
uint64_t offset_ts_ns;//第一帧imu的时间戳，后续的imu和图像的时间戳都要减去他,也就是时间戳从0开始
uint64_t offset_ts_ns_new;//减去了imu与cam时间差的第一帧imu时间戳
uint64_t timeshift_cam_imu;//imu加上时间偏移部分

std::vector<cv::KeyPoint> pre_image_key_points;
cv::Mat pre_image_features;

typedef void(*funPtrType)(const float* , const float* );
funPtrType callbackFun;

constexpr int reserve_num = 5000;
bool getAllImage;
//用来设置系统的初始状态或者是清除系统状态
void reset(){
    LOGE("lalala reset");
  getAllImage=false;
  imgFolder="";
  FLAGS_iba_param_path="";
  FLAGS_gba_camera_save_path="";
  stereoApi=false;
  monoApi=false;
  imu_samples.clear();
  bInit=false;
  dataSet=false;
  FLAGS_stereo=false;
  offset_ts_ns=0;
}
void setCallback(void* fun){
  callbackFun=(funPtrType)fun;
}
int release(){
    LOGE("lalala release");
  google::ShutdownGoogleLogging();
  solver.Stop();
  solver.Destroy();
  reset();
  return STATUS::OK;
}

//get images names
size_t load_image_data(const string& image_folder,
                       std::vector<string> &limg_name,
                       std::vector<string> &rimg_name) {
  LOG(INFO) << "Loading " << image_folder;
  std::string l_path = image_folder + "/mav0/cam0/data.csv";
  std::string r_path = image_folder + "/mav0/cam1/data.csv";
  std::string l_img_prefix = image_folder + "/mav0/cam0/data/";
  std::string r_img_prefix = image_folder + "/mav0/cam1/data/";
  std::ifstream limg_file(l_path);
  std::ifstream rimg_file(r_path);
  if (!limg_file.is_open() || !rimg_file.is_open()) {
    LOG(WARNING) << image_folder << " cannot be opened";
    return 0;
  }
  std::string line;
  std::string time;
  while (getline(limg_file,line)) {
    if (line[0] == '#')
      continue;
    std::istringstream is(line);
    int i = 0;
    while (getline(is, time, ',')){
      bool is_exist = boost::filesystem::exists(l_img_prefix + time + ".png");
      if (i == 0 && is_exist){
        limg_name.push_back(time + ".png");
        rimg_name.push_back(time + ".png");
      }
      i++;
    }
  }
  limg_file.close();
  rimg_file.close();
  LOG(INFO)<< "loaded " << limg_name.size() << " images";
  return limg_name.size();
}

int load_imu_data(const string& imu_file_str,
                     std::list<XP::ImuData>* imu_samples_ptr,
                     uint64_t &offset_ts_ns) {
  CHECK(imu_samples_ptr != nullptr);
  LOG(INFO) << "Loading " << imu_file_str;
  std::ifstream imu_file(imu_file_str.c_str());
  if (!imu_file.is_open()) {
    LOG(WARNING) << imu_file_str << " cannot be opened";
    return STATUS::IMU_FILE_NOT_EXIST;
  }
  std::list<XP::ImuData>& imu_datas = *imu_samples_ptr;
  imu_datas.clear();
  // read imu data
  std::string line;
  std::string item;
  double c[6];
  uint64_t t;
  bool set_offset_time = false;
  while (getline(imu_file,line)) {
    if (line[0] == '#')
      continue;
    std::istringstream is(line);
    int i = 0;
    while (getline(is, item, ',')) {
      std::stringstream ss;
      ss << item;
      if (i == 0)
        ss >> t;
      else
        ss >> c[i-1];
      i++;
    }
    if (!set_offset_time) {
      set_offset_time = true;
      offset_ts_ns = t;
    }
    XP::ImuData imu_sample;
    float _t_100us = (t - offset_ts_ns)/1e5;
    imu_sample.time_stamp = _t_100us/1e4;
    imu_sample.ang_v(0) = c[0];
    imu_sample.ang_v(1) = c[1];
    imu_sample.ang_v(2) = c[2];
    imu_sample.accel(0) = c[3];
    imu_sample.accel(1) = c[4];
    imu_sample.accel(2) = c[5];

    VLOG(3) << "accel " << imu_sample.accel.transpose()
            << " gyro " << imu_sample.ang_v.transpose();
    imu_datas.push_back(imu_sample);
  }
  imu_file.close();
  LOG(INFO)<< "loaded " << imu_datas.size() << " imu samples";
  return STATUS::OK;
}

int load_asl_calib(const std::string &paraFStr,
                    XP::DuoCalibParam &calib_param) {

  //先判断是否存在参数配置文件，直接所有的配置参数都由这个文件给出来，即使是使用数据集的模式也这样使用
  string paraPath=paraFStr+paraName;

  if (access(paraPath.c_str(), 0) == -1){//file exist 可以判断文件夹，可用相对路径
      return STATUS::PARASETTING_FILE_NOT_EXIST;
      LOGE("lalala paraPath %s",paraPath.c_str());
  }

  YAML::Node settingNode = YAML::LoadFile(paraPath);

  dataSet = settingNode["dataSet"].as<bool>(); //判断是数据集离线运行还是在线运行
  FLAGS_stereo = settingNode["FLAGS_stereo"].as<bool>(); //判断是单目还是双目

  std::string ibaParaFile = settingNode["FLAGS_iba_param_path"].as<std::string>();
  FLAGS_iba_param_path= paraFStr+ibaParaFile;//iba-ce单目或者双目参数文件的地址
  std::string gbaCameraFile = settingNode["gbaCameraFile"].as<std::string>();
  FLAGS_gba_camera_save_path= paraPath+gbaCameraFile;//结果的保存地址
  imgFolder = settingNode["imgFolder"].as<std::string>();
  if(imgFolder.size()<10)
      imgFolder=paraFStr+imgFolder;//认为是要从手机端做本地测试

  std::string model = settingNode["distortion_model"].as<std::string>();
  if(model=="equidistant")
    calib_param.Camera.fishEye=true;
  else
    calib_param.Camera.fishEye=false;

  //传感器时间偏移
  calib_param.timeshift_cam_imu = settingNode["timeshift_cam_imu"].as<double>();
  timeshift_cam_imu = (uint64)(calib_param.timeshift_cam_imu*1e9);

  YAML::Node cam0_calib = settingNode["cam0"];
  YAML::Node cam1_calib = settingNode["cam1"];
  YAML::Node imu0_calib = settingNode["imu0"];

  // intrinsics
  std::vector<float> v_float = cam0_calib["intrinsics"].as<std::vector<float>>();
  //左相机的和右相机的内参K矩阵
  calib_param.Camera.cv_camK_lr[0] << v_float[0], 0, v_float[2],
      0, v_float[1], v_float[3],
      0, 0, 1;
  calib_param.Camera.cameraK_lr[0] << v_float[0], 0, v_float[2],
      0, v_float[1], v_float[3],
      0, 0, 1;
  v_float = cam1_calib["intrinsics"].as<std::vector<float>>();
  calib_param.Camera.cv_camK_lr[1] << v_float[0], 0, v_float[2],
      0, v_float[1], v_float[3],
      0, 0, 1;
  calib_param.Camera.cameraK_lr[1] << v_float[0], 0, v_float[2],
      0, v_float[1], v_float[3],
      0, 0, 1;
  // distortion_coefficients
  std::vector<double> v_double0 = cam0_calib["distortion_coefficients"].as<std::vector<double>>();
  std::vector<double> v_double1 = cam1_calib["distortion_coefficients"].as<std::vector<double>>();

  if (v_double0.size() > 4){
    calib_param.Camera.cv_dist_coeff_lr[0] = (cv::Mat_<float>(8, 1) << static_cast<float>(v_double0[0]), static_cast<float>(v_double0[1]),
                                              static_cast<float>(v_double0[2]), static_cast<float>(v_double0[3]), static_cast<float>(v_double0[4]),
                                              static_cast<float>(v_double0[5]), static_cast<float>(v_double0[6]), static_cast<float>(v_double0[7]));
    calib_param.Camera.cv_dist_coeff_lr[1] = (cv::Mat_<float>(8, 1) << static_cast<float>(v_double1[0]), static_cast<float>(v_double1[1]),
                                              static_cast<float>(v_double1[2]), static_cast<float>(v_double1[3]), static_cast<float>(v_double1[4]),
                                              static_cast<float>(v_double1[5]), static_cast<float>(v_double1[6]), static_cast<float>(v_double1[7]));
  }
  else{
    calib_param.Camera.cv_dist_coeff_lr[0] = (cv::Mat_<float>(8, 1) << static_cast<float>(v_double0[0]), static_cast<float>(v_double0[1]),
                                              static_cast<float>(v_double0[2]), static_cast<float>(v_double0[3]), 0.0, 0.0, 0.0, 0.0);

    calib_param.Camera.cv_dist_coeff_lr[1] = (cv::Mat_<float>(8, 1) << static_cast<float>(v_double1[0]), static_cast<float>(v_double1[1]),
                                              static_cast<float>(v_double1[2]), static_cast<float>(v_double1[3]), 0.0, 0.0, 0.0, 0.0);
  }

  //TBS
  //左右相机,imu到本体坐标系的外参,本体系在euroc中固连于imu上
  //Tbc0,Tbc1
  v_double0 = cam0_calib["T_BS"]["data"].as<std::vector<double>>();
  v_double1 = cam1_calib["T_BS"]["data"].as<std::vector<double>>();
  Eigen::Matrix4d b_t_c0;
  Eigen::Matrix4d b_t_c1;

  //判断外参要不要转置，直接使用kalibr标定出来的外参是转置之后才能进行使用的
  int TBS_INV = settingNode["T_BS_INV"].as<int>();
  if (TBS_INV == 0)
  {
    b_t_c0 = Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(&v_double0[0]);
    b_t_c1 = Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(&v_double1[0]);
  }
  else
  {
    //tum_vi是imu到cam所以要求逆 不知道为啥突然间Eigen求逆会崩溃了，所以改用opencv
    cv::Matx44d cv_b_t_c0(&v_double0[0]);
    cv::Matx44d cv_b_t_c1(&v_double1[0]);
    cv_b_t_c0 = cv_b_t_c0.inv();

    cv_b_t_c1 = cv_b_t_c1.inv();

    b_t_c0 = Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(cv_b_t_c0.val);
    b_t_c1 = Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(cv_b_t_c1.val);
  }

  std::vector<double> v_double = imu0_calib["T_BS"]["data"].as<std::vector<double>>();
  Eigen::Matrix4d b_t_i = Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(&v_double[0]);
  // ASL {B}ody frame is the IMU
  // {D}evice frame is the left camera
  Eigen::Matrix4d d_t_cam0 = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d d_t_b = d_t_cam0 * b_t_c0.inverse();
  Eigen::Matrix4d d_t_cam1 = d_t_b * b_t_c1;
  Eigen::Matrix4d d_t_imu = d_t_b * b_t_i;
  calib_param.Camera.D_T_C_lr[0] = Eigen::Matrix4f::Identity();
  calib_param.Camera.D_T_C_lr[1] = d_t_cam1.cast<float>();
  // Image size
  std::vector<int> v_int = settingNode["resolution"].as<std::vector<int>>();
  calib_param.Camera.img_size = cv::Size(v_int[0], v_int[1]);
  // IMU
  calib_param.Imu.accel_TK = Eigen::Matrix3f::Identity();
  calib_param.Imu.accel_bias = Eigen::Vector3f::Zero();
  calib_param.Imu.gyro_TK = Eigen::Matrix3f::Identity();
  calib_param.Imu.gyro_bias = Eigen::Vector3f::Zero();
  calib_param.Imu.accel_noise_var = Eigen::Vector3f{0.0016, 0.0016, 0.0016};
  calib_param.Imu.angv_noise_var = Eigen::Vector3f{0.0001, 0.0001, 0.0001};
  calib_param.Imu.D_T_I = d_t_imu.cast<float>();
  calib_param.device_id = "ASL";
  calib_param.sensor_type = XP::DuoCalibParam::SensorType::UNKNOWN;

  calib_param.initUndistortMap(calib_param.Camera.img_size);

  return STATUS::OK;
}

float get_timestamp_from_img_name(const string& img_name,
                                  uint64_t offset_ns) {
  string ts_ns_string = fs::path(img_name).stem().string();
  int64_t offset_t = boost::lexical_cast<uint64_t>(ts_ns_string) - offset_ns;
  int64_t t = offset_t/1e5;
  return static_cast<float>(t)/1e4;
}

bool convert_to_asl_timestamp(const string& file_in,
                              const string& file_out,
                              uint64_t offset_ns) {
  FILE *fp_in = fopen(file_in.c_str(), "r");
  FILE *fp_out = fopen(file_out.c_str(), "w");
  if (!fp_in || !fp_out) {
    LOG(ERROR) << "convert to asl timestamp error";
    return false;
  }
  float t;
  float x, y, z;
  float qx, qy, qz, qw;
  while (fscanf(fp_in, "%f %f %f %f %f %f %f %f", &t, &x, &y, &z, &qx, &qy, &qz, &qw) == 8) {
    double t_s = t + static_cast<double>(offset_ns*1e-9);
    fprintf(fp_out, "%lf %f %f %f %f %f %f %f\n", t_s, x, y, z, qx, qy, qz, qw);
  }
  fclose(fp_in);
  fclose(fp_out);
  return true;
}

inline bool cmp_by_class_id(const cv::KeyPoint& lhs, const cv::KeyPoint& rhs)  {
  return lhs.class_id < rhs.class_id;
}

template <typename T>
void InitPOD(T& t) {
  memset(&t, 0, sizeof(t));
}

//create current frame,if track key point is too less,build a keyframe
bool create_iba_frame(const vector<cv::KeyPoint>& kps_l,
                      const vector<cv::KeyPoint>& kps_r,
                      const vector<XP::ImuData>& imu_samples,
                      const float rig_time,
                      IBA::CurrentFrame* ptrCF, IBA::KeyFrame* ptrKF) {
  CHECK(std::is_sorted(kps_l.begin(), kps_l.end(), cmp_by_class_id));
  CHECK(std::is_sorted(kps_r.begin(), kps_r.end(), cmp_by_class_id));
  CHECK(std::includes(kps_l.begin(), kps_l.end(), kps_r.begin(), kps_r.end(), cmp_by_class_id));

  // IBA will handle *unknown* initial depth values
  IBA::Depth kUnknownDepth;
  kUnknownDepth.d = 0.0f;
  kUnknownDepth.s2 = 0.0f;
  static int last_added_point_id = -1;
  static int iba_iFrm = 0;
  auto kp_it_l = kps_l.cbegin(), kp_it_r = kps_r.cbegin();

  IBA::CurrentFrame& CF = *ptrCF;
  IBA::KeyFrame& KF = *ptrKF;

  CF.iFrm = iba_iFrm;
  InitPOD(CF.C); // needed to ensure the dumped frame deterministic even for unused field
  CF.C.C.R[0][0] = CF.C.v[0] = CF.C.ba[0] = CF.C.bw[0] = FLT_MAX;
  // MapPointMeasurement, process in ascending class id, left camera to right
  // Note the right keypoints is a subset of the left ones
  IBA::MapPointMeasurement mp_mea;
  InitPOD(mp_mea);
  mp_mea.x.S[0][0] = mp_mea.x.S[1][1] = 1.f;
  mp_mea.x.S[0][1] = mp_mea.x.S[1][0] = 0.f;
  for (; kp_it_l != kps_l.cend() && kp_it_l->class_id <= last_added_point_id; ++kp_it_l) {
    mp_mea.idx = kp_it_l->class_id;
    mp_mea.x.x[0] = kp_it_l->pt.x;
    mp_mea.x.x[1] = kp_it_l->pt.y;
    mp_mea.right = false;
    CF.zs.push_back(mp_mea);
    if (kp_it_r != kps_r.cend() && kp_it_r->class_id == kp_it_l->class_id) {
      mp_mea.x.x[0] = kp_it_r->pt.x;
      mp_mea.x.x[1] = kp_it_r->pt.y;
      mp_mea.right = true;
      CF.zs.push_back(mp_mea);
      ++kp_it_r;
    }
  }
  std::transform(imu_samples.begin(), imu_samples.end(), std::back_inserter(CF.us), XP::to_iba_imu);
  CF.t = rig_time;
  CF.d = kUnknownDepth;
  bool need_new_kf = std::distance(kp_it_l, kps_l.end()) >= 20 || CF.zs.size() < 20;
  if (std::distance(kp_it_l, kps_l.end()) == 0)
    need_new_kf = false;
  if (!need_new_kf) KF.iFrm = -1;
  else
    LOG(INFO) << "new keyframe " << KF.iFrm;

  if (!need_new_kf) {
    KF.iFrm = -1;
    //  to make it deterministic
    InitPOD(KF.C);
    InitPOD(KF.d);
  } else {
    KF.iFrm = CF.iFrm;
    KF.C = CF.C.C;
    // MapPointMeasurement, duplication of CF
    KF.zs = CF.zs;
    // MapPoint
    for(; kp_it_l != kps_l.cend(); ++kp_it_l) {
      IBA::MapPoint mp;
      InitPOD(mp.X);
      mp.X.idx = kp_it_l->class_id;
      mp.X.X[0] = FLT_MAX;
      mp_mea.iFrm = iba_iFrm;
      mp_mea.x.x[0] = kp_it_l->pt.x;
      mp_mea.x.x[1] = kp_it_l->pt.y;
      mp_mea.right = false;
      mp.zs.push_back(mp_mea);

      if (kp_it_r != kps_r.cend() && kp_it_r->class_id == kp_it_l->class_id) {
        mp_mea.x.x[0] = kp_it_r->pt.x;
        mp_mea.x.x[1] = kp_it_r->pt.y;
        mp_mea.right = true;
        mp.zs.push_back(mp_mea);
        kp_it_r++;
      } else {
        LOG(WARNING) << "add new feature point " << kp_it_l->class_id << " only found in left image";
      }
      KF.Xs.push_back(mp);
    }
    last_added_point_id = std::max(KF.Xs.back().X.idx, last_added_point_id);
    KF.d = kUnknownDepth;
  }
  ++iba_iFrm;
  return true;
}
int getImageNames(std::string imgs_folder){
  //get image files names
  img_file_paths.clear();
  slave_img_file_paths.clear();

  img_file_paths.reserve(reserve_num);
  slave_img_file_paths.reserve(reserve_num);
  fs::path p(imgs_folder + "/mav0/cam0");
  if (!fs::is_directory(p)) {
    LOG(ERROR) << p << " is not a directory";
    return STATUS::FOLDER_NOT_EXIST;
  }

  vector<string> limg_name, rimg_name;
  load_image_data(imgs_folder, limg_name, rimg_name);
  for (int i=0; i<limg_name.size(); i++) {
    string l_png = p.string() + "/data/" + limg_name[i];
    img_file_paths.push_back(l_png);
    slave_img_file_paths.push_back(imgs_folder + "/mav0/cam1/data/" + rimg_name[i]);
  }
  if (!FLAGS_stereo) {
    slave_img_file_paths.clear();
  }

  if (img_file_paths.size() == 0) {
    LOG(ERROR) << "No image files for detection";
    return STATUS::IMG_NAME_EMPTY;
  }
  return STATUS::OK;
}
int loadCalibPara(string paraFStr){
  int status=STATUS::OK;
  try {
    int status = load_asl_calib(paraFStr, duo_calib_param);
    if (status != STATUS::OK)
      return status;
  } catch (...){
    LOG(ERROR) << "Load calibration file error";
    return STATUS::LOAD_PARA_FAIL;
  }
  // Create masks based on FOVs computed from intrinsics
  for (int lr = 0; lr < 2; ++lr) {
    float fov;
    if (!XP::generate_cam_mask(duo_calib_param.Camera.cv_camK_lr[lr],
                              duo_calib_param.Camera.cv_dist_coeff_lr[lr],
                              duo_calib_param.Camera.fishEye,
                              duo_calib_param.Camera.img_size,
                              &masks[lr],
                              &fov)) 
    status=STATUS::GEN_MASK_FAIL;
    
  }
  T_Cl_Cr = duo_calib_param.Camera.D_T_C_lr[0].inverse() * duo_calib_param.Camera.D_T_C_lr[1];
  return status;
}

/**
 * @brief 对整个slam系统初始化
 */
#ifdef __ANDROID__
int init(const char* paraFolder){
  char **argv = new char *[10];
  string name = "ice_ba";
  paraFStr=paraFolder;
  string log_dir = paraFStr+"log/";
  argv[0] = const_cast<char *>(name.c_str());
#else
int init(int argc, char **argv){

  reset();
  paraFStr = argv[1];
  //初始化log
  google::ParseCommandLineFlags(&argc, &argv, true);
  string log_dir = "../log/";
#endif

  //判断参数是否合法
  if (paraFStr.empty()){ //文件夹路径空
    #ifndef __ANDROID__
      google::ShowUsageWithFlags(argv[0]);
    #endif
    return STATUS::PARA_FOLDER_EMPTY;
  }

  if (access(paraFStr.c_str(), 0) == -1) { //folder not exist
      LOGE("lalala could not access file %s", paraFStr.c_str());
      return STATUS::FOLDER_NOT_EXIST;
  }

//  google::InitGoogleLogging(argv[0]); //注释这行就会有日志输出
  if (!fs::is_directory(log_dir)) {
    fs::create_directories(log_dir);
  }
  google::SetLogDestination(google::INFO, log_dir.c_str());
  google::InstallFailureSignalHandler();

  //加载参数
  int status = loadCalibPara(paraFStr);
  if (status != STATUS::OK){
    LOG(ERROR) << "could not load calib para";
    return status;
  }

  //加载离线相关数据
  if (dataSet){
      LOGE("lalala init imgFolder %s",imgFolder.c_str());
      //加载图像名字
    status = getImageNames(imgFolder);
    if (status != STATUS::OK){
      LOG(ERROR) << "get image names failed";
      return status;
    }

    // Load IMU samples to predict OF point locations
    std::string imu_file = imgFolder + "/mav0/imu0/data.csv";
    status = load_imu_data(imu_file, &imu_samples, offset_ts_ns);
    if (status != STATUS::OK)
      return status;

    // Adjust end image index for detection
    if (FLAGS_end_idx < 0 || FLAGS_end_idx > img_file_paths.size()){
      FLAGS_end_idx = img_file_paths.size();
    }

    FLAGS_start_idx = std::max(0, FLAGS_start_idx);
    // remove all frames before the first IMU data   丢弃在第一帧imu之前的图像  t_imu = t_cam + shift
    offset_ts_ns_new = offset_ts_ns - (uint64_t)(duo_calib_param.timeshift_cam_imu * 1e9); //这里仅仅是为了迎合数据集有时间差的变量
    while (FLAGS_start_idx < FLAGS_end_idx && get_timestamp_from_img_name(img_file_paths[FLAGS_start_idx], offset_ts_ns_new) <= imu_samples.front().time_stamp)
      FLAGS_start_idx++;
  }

    feat_track_detector = new XP::FeatureTrackDetector(FLAGS_ft_len,
                                                       FLAGS_ft_droprate,
                                                       !FLAGS_not_use_fast,
                                                       FLAGS_uniform_radius,
                                                       duo_calib_param.Camera.img_size);
    slave_img_feat_propagator = new XP::ImgFeaturePropagator(
            duo_calib_param.Camera.cameraK_lr[1],  // cur_camK
            duo_calib_param.Camera.cameraK_lr[0],  // ref_camK
            duo_calib_param.Camera.cv_dist_coeff_lr[1],  // cur_dist_coeff
            duo_calib_param.Camera.cv_dist_coeff_lr[0],  // ref_dist_coeff
            duo_calib_param.Camera.fishEye,
            masks[1],
            FLAGS_pyra_level,
            FLAGS_min_feature_distance_over_baseline_ratio,
            FLAGS_max_feature_distance_over_baseline_ratio);

  pose_viewer.set_clear_canvas_before_draw(true);

  last_position = Eigen::Vector3f::Zero();

  solver.Create(to_iba_calibration(duo_calib_param),
                257,
                IBA_VERBOSE_NONE,
                IBA_DEBUG_NONE,
                257,
                FLAGS_iba_param_path,
                "" /* iba directory */);
  solver.SetCallbackLBA([&](const int iFrm, const float ts) {
    static float travel_dist = 0.f;
#ifndef __DUO_VIO_TRACKER_NO_DEBUG__
      VLOG(1) << "===== start ibaCallback at ts = " << ts;
#endif
      // as we may be able to send out information directly in the callback arguments
      IBA::SlidingWindow sliding_window;
      solver.GetSlidingWindow(&sliding_window);
      const IBA::CameraIMUState &X = sliding_window.CsLF.back();
//      m_state.lock();
//      last_state = X;//更新最新vio姿态
//      m_state.unlock();
      const IBA::CameraPose &C = X.C;
      Eigen::Matrix4f W_vio_T_S = Eigen::Matrix4f::Identity();  // W_vio_T_S
      for (int i = 0; i < 3; ++i) {
        W_vio_T_S(i, 3) = C.p[i];
        for (int j = 0; j < 3; ++j) {
          W_vio_T_S(i, j) = C.R[j][i];  // C.R is actually R_SW
        }
      }
      Eigen::Matrix<float, 9, 1> speed_and_biases;
      for (int i = 0; i < 3; ++i) {
        speed_and_biases(i) = X.v[i];
        speed_and_biases(i + 3) = X.ba[i];
        speed_and_biases(i + 6) = X.bw[i];
      }
      Eigen::Vector3f cur_position = W_vio_T_S.topRightCorner(3, 1);
      travel_dist += (cur_position - last_position).norm();
      last_position = cur_position;
      pose_viewer.addPose(W_vio_T_S, speed_and_biases, travel_dist);
     #ifdef __ANDROID__
      XP::XpQuaternion q;
      q.SetFromRotationMatrix(W_vio_T_S.block<3,3>(0,0));
//      LOGE("lalala on solver CallbackLBA callbackFun");
      callbackFun((float*)q.data(), C.p);
      #endif

  });
  solver.Start();
  LOGE("lalala set bInit true");
  bInit=true;
  LOGE("lalala set bInit true %d",bInit);
  return STATUS::OK;
}

////在初始化成功之后，直接根具imu给出姿态
//void pubImuPose(const XP::ImuData& imu){
//
//  //获取上一帧imu时候的传感器状态
//  m_state.lock();
//  Eigen::Vector3f P1(last_state.C.p);
//  XP::XpQuaternion Q1;
//  Eigen::Matrix3f q_r((float*)last_state.C.R);
//  Q1.SetFromRotationMatrix(q_r);
//  Eigen::Vector3f V1(last_state.v);
//  Eigen::Vector3f Ba(last_state.ba);
//  Eigen::Vector3f Bw(last_state.bw);
//  m_state.unlock();
//
//  //使用中值积分
//
//  //角速度均值
//  Eigen::Vector3f w_mean=(pre_imu.ang_v+imu.ang_v)/2-Bw;
//  //旋转
//  float delta_t=imu.time_stamp-pre_imu.time_stamp;
//  Eigen::Vector3f w_mean_dt=w_mean*delta_t;
//  XP::XpQuaternion Q12;
//  Q12.SetFromDeltaTheta(w_mean_dt);
//  XP::XpQuaternion Q2=quat_multiply(Q1,Q12);
//
//  //加速度均值  注意这里的重力加速度是在世界坐标系的
//  const Eigen::Vector3f g(0,0,9.80665);
//  Eigen::Vector3f a_mean=(Q1.ToRotationMatrix()*(pre_imu.accel-Ba)-g+Q2.ToRotationMatrix()*(imu.accel-Ba)-g)/2;
//  //V
//  Eigen::Vector3f V2=V1+a_mean*delta_t;
//  //P
//  Eigen::Vector3f P2=P1+V1*delta_t+a_mean*delta_t*delta_t/2;
//
//  //将imu积分得到的状态保存下来 需要修改pvq b不需要更改
//  m_state.lock();
//  memcpy(last_state.C.p,P2.data(),3*sizeof(float));
//  memcpy(last_state.v,V2.data(),3*sizeof(float));
//  memcpy(last_state.C.R,Q2.ToRotationMatrix().data(),9*sizeof(float));//TODO：cy 旋转转来转去麻烦,后期改掉
//  m_state.unlock();
//  pre_imu=imu;
//
//  //回调发布位姿
//  #ifdef __ANDROID__
//  callbackFun((float *)Q2.data(), P2.data());
//  #endif
//}

void intergration(const std::vector<XP::ImuData> &imu_meas, cv::Matx33f &old_R_new){
  XP::XpQuaternion I_new_q_I_old; // The rotation between the new {I} and old {I}
  for (size_t i = 1; i < imu_meas.size(); ++i){
    XP::XpQuaternion q_end;
    XP::IntegrateQuaternion(imu_meas[i - 1].ang_v,
                            imu_meas[i].ang_v,
                            I_new_q_I_old,
                            imu_meas[i].time_stamp - imu_meas[i - 1].time_stamp,
                            &q_end);
    I_new_q_I_old = q_end;
  }
  Eigen::Matrix3f I_new_R_I_old = I_new_q_I_old.ToRotationMatrix();
  Eigen::Matrix4f I_T_C =
      duo_calib_param.Imu.D_T_I.inverse() * duo_calib_param.Camera.D_T_C_lr[0];
  Eigen::Matrix3f I_R_C = I_T_C.topLeftCorner<3, 3>();
  Eigen::Matrix3f C_new_R_C_old = I_R_C.transpose() * I_new_R_I_old * I_R_C;
  for (int i = 0; i < 3; ++i){
    for (int j = 0; j < 3; ++j){
      old_R_new(j, i) = C_new_R_C_old(i, j);
    }
  }

  if (VLOG_IS_ON(1)){
    XP::XpQuaternion C_new_q_C_old;
    C_new_q_C_old.SetFromRotationMatrix(C_new_R_C_old);
    VLOG(1) << "C_new_R_C_old = \n"
            << C_new_R_C_old;
    VLOG(1) << "ea =\n"
            << C_new_q_C_old.ToEulerRadians() * 180 / M_PI;
  }
}

void loadIMU(const float time_stamp, std::vector<XP::ImuData> &imu_meas){
  imu_meas.reserve(10);
  for (auto it_imu = imu_samples.begin(); it_imu != imu_samples.end();){
    if (it_imu->time_stamp < time_stamp){
      imu_meas.push_back(*it_imu);
      it_imu++;
      imu_samples.pop_front();
    }
    else
      break;
  }
  VLOG(1) << "imu_meas size = " << imu_meas.size();
  if (imu_meas.size() > 0){
    VLOG(1) << "imu ts prev -> curr " << imu_meas.front().time_stamp
            << " -> " << imu_meas.back().time_stamp;
  }
}

//TODO:cy 后面要修改成严格同步的，并且加上插值部分
int getDataFromBuf(cv::Mat &imgL, cv::Mat &imgR, float& imgTimestamp, std::vector<XP::ImuData> &imu_meas){
  m_stereobuf.lock();
  auto &data = stereoImgBuf.front();
  m_stereobuf.unlock();

  imgTimestamp = data.second;
  if (imu_samples.back().time_stamp <= imgTimestamp)
    return STATUS::WAIT_IMG;
  m_imubuf.lock();
  loadIMU(imgTimestamp, imu_meas);
  m_imubuf.unlock();

  //判断imu
  imgL = data.first.first;
  if(FLAGS_stereo)
    imgR = data.first.second;
  m_stereobuf.lock();
  stereoImgBuf.pop_front();
  m_stereobuf.unlock();

  return STATUS::OK;
}

void track(){

  bool firstF = true;

  while (!getAllImage || stereoImgBuf.size()>1)
  {

  std::unique_lock<std::mutex> lck(m_con_mutex);
  con.wait(lck);

  float time_stamp;
  std::vector<XP::ImuData> imu_meas;
  cv::Mat  slave_img_smooth;
  if (getDataFromBuf(img_in_smooth, slave_img_smooth, time_stamp, imu_meas) != STATUS::OK)//已经验证过这部分是没有问题的，读取的数据是和原始的ICE-BA一模一样
    continue;
//      LOGE("lalala process one frame data");
  std::vector<cv::KeyPoint> key_pnts;
  // cv::Mat orb_feat;
  std::vector<cv::KeyPoint> key_pnts_slave;
  // cv::Mat orb_feat_slave;

  // use optical flow  from the 1st frame
    if (!firstF) {
      VLOG(1) << "pre_image_key_points.size(): " << pre_image_key_points.size();
      const int request_feat_num = FLAGS_max_num_per_grid * FLAGS_grid_row_num * FLAGS_grid_col_num;
      TS(build_img_pyramids);
      feat_track_detector->build_img_pyramids(img_in_smooth, XP::FeatureTrackDetector::BUILD_TO_CURR);
      TE(build_img_pyramids);
      if (imu_meas.size() > 1) {
        // Here we simply the transformation chain to rotation only and assume zero translation
        cv::Matx33f old_R_new;
        intergration(imu_meas,old_R_new);
        TS(optical_flow_and_detect);
        feat_track_detector->optical_flow_and_detect(masks[0],
                                                    pre_image_features,
                                                    pre_image_key_points,
                                                    request_feat_num,
                                                    FLAGS_pyra_level,
                                                    FLAGS_fast_thresh,
                                                    &key_pnts,
                                                    nullptr,
                                                    duo_calib_param.Camera.fishEye,
                                                    cv::Vec2f(0, 0),  // shift init pixels
                                                    &duo_calib_param.Camera.cv_camK_lr[0],
                                                    &duo_calib_param.Camera.cv_dist_coeff_lr[0],
                                                    &old_R_new);
        TE(optical_flow_and_detect);
      } else {
        feat_track_detector->optical_flow_and_detect(masks[0],
                                                    pre_image_features,
                                                    pre_image_key_points,
                                                    request_feat_num,
                                                    FLAGS_pyra_level,
                                                    FLAGS_fast_thresh,
                                                    &key_pnts,
                                                    nullptr);
      }
      feat_track_detector->update_img_pyramids();
      VLOG(1) << "after OF key_pnts.size(): " << key_pnts.size() << " requested # "
              << FLAGS_max_num_per_grid * FLAGS_grid_row_num * FLAGS_grid_col_num;
    } else {
      // first frame
      feat_track_detector->detect(img_in_smooth,
                                 masks[0],
                                 FLAGS_max_num_per_grid * FLAGS_grid_row_num * FLAGS_grid_col_num,
                                 FLAGS_pyra_level,
                                 FLAGS_fast_thresh,
                                 &key_pnts,
                                 nullptr);
      feat_track_detector->build_img_pyramids(img_in_smooth, XP::FeatureTrackDetector::BUILD_TO_PREV);
      firstF = false;
    }
    if (slave_img_smooth.rows > 0) {
      // CHECK(orb_feat_slave.empty());
      TS(Propagate_slave_Features);
      slave_img_feat_propagator->PropagateFeatures(slave_img_smooth,  // cur
                                                  img_in_smooth,  // ref
                                                  key_pnts,
                                                  T_Cl_Cr,  // T_ref_cur
                                                  &key_pnts_slave,
                                                  nullptr,
                                                  false);  // draw_debug
      TE(Propagate_slave_Features);
    }

    std::sort(key_pnts.begin(), key_pnts.end(), cmp_by_class_id);
    std::sort(key_pnts_slave.begin(), key_pnts_slave.end(), cmp_by_class_id);
    // push to IBA
    IBA::CurrentFrame CF;
    IBA::KeyFrame KF;
    create_iba_frame(key_pnts, key_pnts_slave, imu_meas, time_stamp, &CF, &KF);
    solver.PushCurrentFrame(CF, KF.iFrm == -1 ? nullptr : &KF);

    pre_image_key_points = key_pnts;
    // pre_image_features = orb_feat.clone();
    // show pose
#ifndef __ANDROID__
cv::imshow("img",img_in_smooth);
#endif
    pose_viewer.displayTo("trajectory");
    std::unique_lock<std::mutex> sbguard(m_trac, std::try_to_lock);
    pose_viewer.getTrajectoryImg(trajectory);
  }
}

int loadDataSet(){
  for (int it_img = FLAGS_start_idx; it_img < FLAGS_end_idx; ++it_img){
    TS(process_one_image);
    cv::Mat slave_img_in;     //右相机图像
    cv::Mat temp = cv::imread(img_file_paths[it_img], cv::IMREAD_GRAYSCALE);
    if(FLAGS_stereo)
      slave_img_in = cv::imread(slave_img_file_paths[it_img], cv::IMREAD_GRAYSCALE);

    CHECK_EQ(temp.rows, duo_calib_param.Camera.img_size.height);
    CHECK_EQ(temp.cols, duo_calib_param.Camera.img_size.width);
    if (temp.rows == 0){
      LOG(ERROR) << "Cannot load " << img_file_paths[it_img];
      return STATUS::FILE_NOT_EXIT;
    }

    // get timestamp from image file name (s)
    string ts_ns_string = fs::path(img_file_paths[it_img]).stem().string();
    pushStereoImg(temp, slave_img_in, boost::lexical_cast<uint64_t>(ts_ns_string));

    TE(process_one_image);
  }
  getAllImage=true;
  return STATUS::OK;
}

int pushStereoImg(const cv::Mat& imgL, const cv::Mat& imgR, uint64_t timestamp){
  if (!bInit)
    return HAVE_NOT_INIT;
  if (timestamp == 0)
    return TIMESTAMP_ZERO;
  if(!stereoApi)
    stereoApi=true;
  if(monoApi&&stereoApi)
    return WRONG_USE_API;

  //先判断是否有imu,不然直接丢弃数据
  //且第一帧imu之前的数据全部丢弃
  if(offset_ts_ns==0 || timestamp < offset_ts_ns_new)
    return STATUS::WAIT_IMU;

  //直接将平滑操作提前，这样就可以避免对原始的图像进行拷贝
  cv::Mat imgL_smooth,imgR_smooth;
  {
    std::unique_lock<std::mutex> sbguard(m_img, std::try_to_lock);
    cv::blur(imgL, imgL_smooth, cv::Size(3, 3));
  }
  if (FLAGS_stereo)
    cv::blur(imgR, imgR_smooth, cv::Size(3, 3));

  //给相机加上时间差，用imu减的话可能导致imu时间戳的值是负数，麻烦 timu=tcam+tshift
  timestamp -= offset_ts_ns_new;
  int64_t t = timestamp/1e5;//TODO:cy 如果不像源码一样不要后面五位的话，轨迹会差挺多的，这部分很有问题啊
  float ftime=static_cast<float>(t)/1e4;
  //默认一定有图像数据的，不进行判断了
  m_stereobuf.lock();
  stereoImgBuf.push_back(std::make_pair(std::make_pair(imgL_smooth, imgR_smooth), ftime));
  m_stereobuf.unlock();

  //唤醒跟踪线程，问题是这样的话会有一帧imu时间的延迟
  if (ftime <= imu_samples.back().time_stamp){

    //丢弃第一帧图像之前的imu,避免先传进来imu很久之后再传进来图像
    //但是要确保第一帧之前是留有一帧以上的imu数据的 后面看看vins是怎么做的
    static bool first = true;
    if (first){
      m_imubuf.lock();
      int dopNum = 0;
      if (imu_samples.size() >= 2)
        //从第二帧imu数据开始
        for (auto it_imu = imu_samples.begin(); it_imu != imu_samples.end();){
          it_imu++;
          if (it_imu->time_stamp < ftime){
            imu_samples.pop_front();
            dopNum++;
          }
          else
            break;
        }
      LOGE("lalala drop %d imu because the first image do not need so much imu", dopNum);
      first = false;
      m_imubuf.unlock();
    }

    //唤醒追踪线程
    std::unique_lock<std::mutex> lck(m_con_mutex);
    con.notify_one();
  }
  else{
      LOGE("lalala not wakeup track thread imu_samples.back().time_stamp %f ftime %f",imu_samples.back().time_stamp,ftime);
  }

  return STATUS::OK;
  
}



int pushImu(const float* gyr, const float* acc, uint64_t timestamp){
  if (!bInit){
      LOGE("lalala bInit had not been setting true %d",bInit);
      return HAVE_NOT_INIT;
  }
  if (timestamp == 0)
    return TIMESTAMP_ZERO;

  //设置时间
  if (offset_ts_ns == 0){
    if(timestamp<(uint64_t)(duo_calib_param.timeshift_cam_imu * 1e9)){
      LOGE("lalala imu timestamp %d smaller than timeshift_cam_imu %ld",timestamp,duo_calib_param.timeshift_cam_imu);
      return STATUS::IMU_INVALID;
    }
    offset_ts_ns = timestamp;
    offset_ts_ns_new=offset_ts_ns-(uint64_t)(duo_calib_param.timeshift_cam_imu * 1e9);
    LOGE("lalala set offset_ts_ns %d timeshift_cam_imu %ld offset_ts_ns_new %d",offset_ts_ns,timeshift_cam_imu,offset_ts_ns_new);
  }
  

  XP::ImuData imu_sample;
  float _t_100us = (timestamp - offset_ts_ns) / 1e5;
  imu_sample.time_stamp = _t_100us / 1e4;
  imu_sample.ang_v(0) = gyr[0];
  imu_sample.ang_v(1) = gyr[1];
  imu_sample.ang_v(2) = gyr[2];
  imu_sample.accel(0) = acc[0];
  imu_sample.accel(1) = acc[1];
  imu_sample.accel(2) = acc[2];

  m_imubuf.lock();
  imu_samples.push_back(imu_sample);
  m_imubuf.unlock();

  //TODO:cy 添加判断条件
  //初始化成功
  //初始化稳定之后
  // pubImuPose(imu_sample);
  return STATUS::OK;
}


#ifdef __ANDROID__
int run_iceba(){
#else
int main(int argc, char **argv){
  if (init(argc, argv) != 0)
    return -1;
#endif
  // 运行追踪线程
  trackT = std::thread(track);

  //本地加载参数运行
  if (dataSet){
    int status = loadDataSet();
    if (status)
      return status;

    trackT.join();
    cv::waitKey(0);
#ifndef __ANDROID__
    std::string temp_file = "/tmp/" + std::to_string(offset_ts_ns) + ".txt";
    solver.SaveCamerasGBA(temp_file, false /* append */, true /* pose only */);
    // for comparsion with asl groundtruth
    convert_to_asl_timestamp(temp_file, FLAGS_gba_camera_save_path, offset_ts_ns);
#endif
    release();
  }

  return STATUS::OK;
}
