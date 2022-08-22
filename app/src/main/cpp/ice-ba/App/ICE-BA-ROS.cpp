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

//ros
#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h> 
#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>

#define TS(name) int64 t_##name = cv::getTickCount()
#define TE(name)  LOG(INFO) << "TIMER_" #name ": "+std::to_string(1000.*((cv::getTickCount() - t_##name) / cv::getTickFrequency()));

namespace fs = boost::filesystem;
using std::string;
using std::vector;
using namespace std;

#include <gflags/gflags.h>
DEFINE_string(imgs_folder, "/home/cy/data/euroc/MH_01", "The folder containing l and r folders, and the calib.yaml");
DEFINE_int32(grid_row_num, 1, "Number of rows of detection grids");
DEFINE_int32(grid_col_num, 1, "Number of cols of detection grids");
DEFINE_int32(max_num_per_grid, 70, "Max number of points per grid");
DEFINE_double(feat_quality, 0.07, "Tomasi-Shi feature quality level");
DEFINE_double(feat_min_dis, 10, "Tomasi-Shi feature minimal distance");
DEFINE_bool(not_use_fast, false, "Whether or not use FAST");
DEFINE_int32(pyra_level, 2, "Total pyramid levels");
DEFINE_double(uniform_radius, 40, "< 5 disables uniformaty enforcement");
DEFINE_int32(ft_len, 125, "The feature track length threshold when dropout kicks in");
DEFINE_double(ft_droprate, 0.05, "The drop out rate when a feature track exceeds ft_len");
DEFINE_bool(show_feat_only, false, "wether or not show detection results only");
DEFINE_int32(fast_thresh, 10, "FAST feature threshold (only meaningful if use_fast=true)");
DEFINE_double(min_feature_distance_over_baseline_ratio,
              4, "Used for slave image feature detection");
DEFINE_double(max_feature_distance_over_baseline_ratio,
              3000, "Used for slave image feature detection");
DEFINE_string(iba_param_path, "../config/config_of_stereo.txt", "iba parameters path");
DEFINE_string(gba_camera_save_path, "/home/cy/data/euroc/result/MH_01_easy.txt", "Save the camera states to when finished");
DEFINE_bool(stereo, false, "monocular or stereo mode");


std::condition_variable con;
std::mutex m_imubuf,m_img0buf,m_img1buf;
queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<sensor_msgs::ImageConstPtr> img0_buf;
queue<sensor_msgs::ImageConstPtr> img1_buf;

std::mutex m_syn_buf;
using stereo_Img = std::pair<cv::Mat,cv::Mat>;
using stereo_Img_info_d = std::pair<double,stereo_Img>;
queue<stereo_Img_info_d> stereo_buf;
queue<std::vector<XP::ImuData>> imu_meas_buf;

XP::DuoCalibParam duo_calib_param;
std::vector<cv::Mat_<uchar> > masks(2);

XP::FeatureTrackDetector* feat_track_detector;
XP::ImgFeaturePropagator* slave_img_feat_propagator;

XP::PoseViewer pose_viewer;
Eigen::Matrix4f T_Cl_Cr;
IBA::Solver solver;
Eigen::Vector3f last_position;
uint64_t offset_ts_ns;
bool set_offset_time = false;

std::vector<cv::KeyPoint> pre_image_key_points;
cv::Mat pre_image_features;


constexpr int reserve_num = 5000;

const double oneFTime = 0.1;            //euroc中相机实际帧率20
const double trackOFTime = oneFTime * 2; //假设追踪的运行帧率



void load_asl_calib(const std::string &asl_path,
                    XP::DuoCalibParam &calib_param) {
  std::string cam0_yaml = asl_path + "/mav0/cam0/sensor.yaml";
  std::string cam1_yaml = asl_path + "/mav0/cam1/sensor.yaml";
  std::string imu0_yaml = asl_path + "/mav0/imu0/sensor.yaml";
  YAML::Node cam0_calib = YAML::LoadFile(cam0_yaml);
  YAML::Node cam1_calib = YAML::LoadFile(cam1_yaml);
  YAML::Node imu0_calib = YAML::LoadFile(imu0_yaml);

  std::string model = cam0_calib["distortion_model"].as<std::string>();
  if(model=="equidistant")
    calib_param.Camera.fishEye=true;
  else
    calib_param.Camera.fishEye=false;

  //传感器时间偏移
  double timeshift_cam_imu = cam0_calib["timeshift_cam_imu"].as<double>();
  calib_param.timeshift_cam_imu = timeshift_cam_imu;

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

  std::cout << "v_double size: " << v_double0.size() << std::endl;
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
  int TBS_INV = cam0_calib["T_BS_INV"].as<int>();
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
    std::cout << "cv_b_t_c0\n " << cv_b_t_c0 << std::endl;
    cv_b_t_c0 = cv_b_t_c0.inv();
    std::cout << "cv_b_t_c0 inv\n " << cv_b_t_c0 << std::endl;

    std::cout << "cv_b_t_c1\n " << cv_b_t_c1 << std::endl;
    cv_b_t_c1 = cv_b_t_c1.inv();
    std::cout << "cv_b_t_c1 inv\n " << cv_b_t_c1 << std::endl;

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
  std::vector<int> v_int = cam0_calib["resolution"].as<std::vector<int>>();
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

int loadCalibPara(){
  try {
    load_asl_calib(FLAGS_imgs_folder, duo_calib_param);
  } catch (...){
    LOG(ERROR) << "Load calibration file error";
    return -1;
  }
  // Create masks based on FOVs computed from intrinsics
  for (int lr = 0; lr < 2; ++lr) {
    float fov;
    if (XP::generate_cam_mask(duo_calib_param.Camera.cv_camK_lr[lr],
                              duo_calib_param.Camera.cv_dist_coeff_lr[lr],
                              duo_calib_param.Camera.fishEye,
                              duo_calib_param.Camera.img_size,
                              &masks[lr],
                              &fov)) {
      std::cout << "camera " << lr << " fov: " << fov << " deg\n";
    }
  }
  T_Cl_Cr = duo_calib_param.Camera.D_T_C_lr[0].inverse() * duo_calib_param.Camera.D_T_C_lr[1];
  return 0;
}



int init(int argc, char **argv){
  google::ParseCommandLineFlags(&argc, &argv, true);
  string log_dir = "../log/";

  google::InitGoogleLogging(argv[0]); //注释这行就会有日志输出
  if (!fs::is_directory(log_dir)) {
    fs::create_directories(log_dir);
  }
  google::SetLogDestination(google::INFO, log_dir.c_str());
  google::InstallFailureSignalHandler();
  if (FLAGS_imgs_folder.empty()) {
    google::ShowUsageWithFlags(argv[0]);
    LOG(ERROR) << "FLAGS_imgs_folder empty";
    return -1;
  }


  int status = loadCalibPara();
  if (status != 0){
      LOG(ERROR) <<"could not load calib para";
      return status;
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
  float travel_dist = 0.f;
  solver.Create(to_iba_calibration(duo_calib_param),
                257,
                IBA_VERBOSE_NONE,
                IBA_DEBUG_NONE,
                257,
                FLAGS_iba_param_path,
                "" /* iba directory */);
  solver.SetCallbackLBA([&](const int iFrm, const float ts) {
#ifndef __DUO_VIO_TRACKER_NO_DEBUG__
      VLOG(1) << "===== start ibaCallback at ts = " << ts;
#endif
      // as we may be able to send out information directly in the callback arguments
      IBA::SlidingWindow sliding_window;
      solver.GetSlidingWindow(&sliding_window);
      const IBA::CameraIMUState &X = sliding_window.CsLF.back();
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
  });
  solver.Start();
  return 0;
}


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

void track(){
  while (1)
  {

      std::unique_lock<std::mutex> lk(m_syn_buf);
      con.wait(lk);
      lk.unlock();

      static bool firstF = true;
      static double alltime = 0.0;
      static int num = 0;

      if (false)
      {
          std::cout << "-----------------------------------------------------------------" << std::endl;
          std::cout << "not include load data track all need time is " << alltime << std::endl;
          std::cout << "track global fps is : " << num / alltime << std::endl;
          std::cout << "-----------------------------------------------------------------" << std::endl;
          break;
      }

  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

  
  m_syn_buf.lock();
  const auto& data=stereo_buf.front();
  float time_stamp=data.first;
  cv::Mat img_in_smooth=data.second.first;
  cv::Mat slave_img_smooth=data.second.second;
  stereo_buf.pop();
  std::vector<XP::ImuData> imu_meas=imu_meas_buf.front();
  imu_meas_buf.pop();
  m_syn_buf.unlock();

  cv::blur(img_in_smooth, img_in_smooth, cv::Size(3, 3));
  cv::blur(slave_img_smooth, slave_img_smooth, cv::Size(3, 3));



  std::vector<cv::KeyPoint> key_pnts;
  cv::Mat orb_feat;
  std::vector<cv::KeyPoint> key_pnts_slave;
  cv::Mat orb_feat_slave;

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
                                                    &orb_feat,
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
                                                    &orb_feat);
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
                                 &orb_feat);
      feat_track_detector->build_img_pyramids(img_in_smooth, XP::FeatureTrackDetector::BUILD_TO_PREV);
      firstF = false;
    }
    if (slave_img_smooth.rows > 0) {
      CHECK(orb_feat_slave.empty());
      TS(Propagate_slave_Features);
      slave_img_feat_propagator->PropagateFeatures(slave_img_smooth,  // cur
                                                  img_in_smooth,  // ref
                                                  key_pnts,
                                                  T_Cl_Cr,  // T_ref_cur
                                                  &key_pnts_slave,
                                                  &orb_feat_slave,
                                                  false);  // draw_debug
      TE(Propagate_slave_Features);
    }

    std::sort(key_pnts.begin(), key_pnts.end(), cmp_by_class_id);
    std::sort(key_pnts_slave.begin(), key_pnts_slave.end(), cmp_by_class_id);
    // push to IBA
    TS(push_to_iba);
    IBA::CurrentFrame CF;
    IBA::KeyFrame KF;
    create_iba_frame(key_pnts, key_pnts_slave, imu_meas, time_stamp, &CF, &KF);
    solver.PushCurrentFrame(CF, KF.iFrm == -1 ? nullptr : &KF);

    pre_image_key_points = key_pnts;
    pre_image_features = orb_feat.clone();
    TE(push_to_iba);
    // show pose
    pose_viewer.displayTo("trajectory");

    //for (size_t i = 0; i < key_pnts.size(); i++)
    //cv::circle(img_in_raw,key_pnts[i].pt,5,cv::Scalar(255),-1);
    // cv::namedWindow("image",0);
    // cv::imshow("image",img_in_raw);

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
    static double track100time = 0.0;
    track100time += ttrack;
    alltime += ttrack;
    if (++num % 100 == 0 ){
      std::cout << "-----------------------------------------------------------------" << std::endl;
      std::cout << "track fps is " << 100.0 / track100time << std::endl;
      std::cout << "-----------------------------------------------------------------" << std::endl;

      track100time = 0;
    }
  }
}

void mutexPop(std::mutex& m, queue<sensor_msgs::ImuConstPtr>& buf){
  m.lock();
  buf.pop();
  m.unlock();
}
void mutexPop(std::mutex& m, queue<sensor_msgs::ImageConstPtr>& buf){
  m.lock();
  buf.pop();
  m.unlock();
}

cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(img_msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }


    cv::Mat img = cv_ptr->image.clone();
    return img;
}

void sync_process()
{
  while (1)
  {
    cv::Mat imLeft;
    cv::Mat imRight;
    std_msgs::Header header;
    double time = 0;
    //make sure got enough imu frame before a image frame
    if (!img0_buf.empty() && !img1_buf.empty())
    {
      double time0 = img0_buf.front()->header.stamp.toSec();
      double time1 = img1_buf.front()->header.stamp.toSec();
      // 0.003s sync tolerance
      double sync_tolerance = 0.003;
      if (time0 < time1 - sync_tolerance)
      {
        mutexPop(m_img0buf, img0_buf);
        printf("throw img0\n");
      }
      else if (time0 > time1 + sync_tolerance)
      {
        mutexPop(m_img1buf, img1_buf);
        printf("throw img1\n");
      }
      else
      {
        time = img0_buf.front()->header.stamp.toSec();
        header = img0_buf.front()->header;
        imLeft = getImageFromMsg(img0_buf.front());
        mutexPop(m_img0buf, img0_buf);
        imRight = getImageFromMsg(img1_buf.front());
        mutexPop(m_img1buf, img1_buf);

        m_syn_buf.lock();
        stereo_buf.push(std::make_pair(time-offset_ts_ns, std::make_pair(imLeft, imRight)));
        m_syn_buf.unlock();
        std::vector<XP::ImuData> imu_meas;
        // Load imu measurements from previous frame
        imu_meas.clear();
        while (!imu_buf.empty() && imu_buf.front()->header.stamp.toSec() <= time)
        {
          XP::ImuData imu_sample;
          imu_sample.time_stamp = imu_buf.front()->header.stamp.toSec()- offset_ts_ns;
          imu_sample.accel(0) = imu_buf.front()->linear_acceleration.x;
          imu_sample.accel(1) = imu_buf.front()->linear_acceleration.y;
          imu_sample.accel(2) = imu_buf.front()->linear_acceleration.z;
          imu_sample.ang_v(0) = imu_buf.front()->angular_velocity.x;
          imu_sample.ang_v(1) = imu_buf.front()->angular_velocity.y;
          imu_sample.ang_v(2) = imu_buf.front()->angular_velocity.z;
          imu_meas.push_back(imu_sample);
          mutexPop(m_imubuf, imu_buf);
        }
        m_syn_buf.lock();
        imu_meas_buf.push(imu_meas);
        m_syn_buf.unlock();
        // //~~show(for debug)~~
        // cv::Mat stereo;
        // cv::hconcat(imLeft, imRight, stereo);
        // cv::imshow("fisheyes", stereo);
        // cv::waitKey(1);
        con.notify_one();
      }
    }
    usleep(100);
  }
}

void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
  if (!set_offset_time){
    set_offset_time = true;
    offset_ts_ns = imu_msg->header.stamp.toSec();
  }

    m_imubuf.lock();
    imu_buf.push(imu_msg);
    m_imubuf.unlock();
    return;
}
void img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_img0buf.lock();
    img0_buf.push(img_msg);
    m_img0buf.unlock();
}
void img1_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_img1buf.lock();
    img1_buf.push(img_msg);
    m_img1buf.unlock();
}

int main(int argc, char **argv){
  if (init(argc, argv) != 0)
    return -1;

  ros::init(argc, argv, "ice_ba_ros");
  ros::start();

  std::thread sync_thread{sync_process};
  std::thread trackT = std::thread(track);

  ros::NodeHandle n;
  ros::Subscriber sub_imu = n.subscribe("/imu0", 2000, imu_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub_img0 = n.subscribe("/cam0/image_raw", 100, img0_callback);
  ros::Subscriber sub_img1 = n.subscribe("/cam1/image_raw", 100, img1_callback);

  ros::spin();

  std::string temp_file = "/tmp/" + std::to_string(offset_ts_ns) + ".txt";
  solver.SaveCamerasGBA(temp_file, false /* append */, true /* pose only */);
  // for comparsion with asl groundtruth
  convert_to_asl_timestamp(temp_file, FLAGS_gba_camera_save_path, offset_ts_ns);

  google::ShutdownGoogleLogging();
  solver.Stop();
  solver.Destroy();

  return 0;
}