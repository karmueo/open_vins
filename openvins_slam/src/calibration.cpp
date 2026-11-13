#include "openvins_slam/calibration.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <stdexcept>
#include "cam/CamEqui.h"
#include "utils/opencv_yaml_parse.h"
#include "utils/quat_ops.h"

namespace openvins_slam {
namespace {

template <int N>
Eigen::Matrix<double, N, 1> vector(const YAML::Node &node, const std::string &name) {
  if (!node || !node.IsSequence() || node.size() != N) throw std::runtime_error(name + " must have " + std::to_string(N) + " values");
  Eigen::Matrix<double, N, 1> result;
  for (int i = 0; i < N; ++i) result(i) = node[i].as<double>();
  if (!result.allFinite()) throw std::runtime_error(name + " contains non-finite values");
  return result;
}

double positive(const YAML::Node &node, const char *name) {
  if (!node[name]) throw std::runtime_error(std::string("missing calibration field ") + name);
  const double v = node[name].as<double>();
  if (!std::isfinite(v) || v <= 0) throw std::runtime_error(std::string("invalid calibration field ") + name);
  return v;
}

} // namespace

Calibration Calibration::load(const std::string &path) {
  const YAML::Node root = YAML::LoadFile(path);
  Calibration c;
  for (int i = 0; i < 2; ++i) {
    const std::string name = "cam" + std::to_string(i);
    const auto n = root[name];
    if (!n || !n["distortion_model"] || n["distortion_model"].as<std::string>() != "fisheye")
      throw std::runtime_error(name + ": only fisheye calibration is supported");
    auto &cam = c.camera[i];
    const auto intrinsic = vector<4>(n["intrinsics"], name + ".intrinsics");
    const auto distortion = vector<4>(n["distortion_coeffs"], name + ".distortion_coeffs");
    cam.intrinsics << intrinsic, distortion;
    if (intrinsic(0) <= 0 || intrinsic(1) <= 0) throw std::runtime_error(name + ": invalid focal length");
    const auto resolution = vector<2>(n["resolution"], name + ".resolution");
    cam.width = static_cast<int>(resolution(0));
    cam.height = static_cast<int>(resolution(1));
    if (cam.width <= 0 || cam.height <= 0 || resolution(0) != cam.width || resolution(1) != cam.height || cam.width % 2 || cam.height % 2)
      throw std::runtime_error(name + ": resolution must contain positive even integers");
    const auto T = n["T_cam_imu"];
    if (!T || !T.IsSequence() || T.size() != 4) throw std::runtime_error(name + ": invalid T_cam_imu");
    for (int r = 0; r < 4; ++r) cam.T_cam_imu.row(r) = vector<4>(T[r], name + ".T_cam_imu").transpose();
    cam.T_cam_imu.block<3, 1>(0, 3) *= 0.001;
    const Eigen::Matrix3d R = cam.T_cam_imu.block<3, 3>(0, 0);
    if ((R.transpose() * R - Eigen::Matrix3d::Identity()).norm() > 1e-5 || std::abs(R.determinant() - 1) > 1e-5 ||
        (cam.T_cam_imu.row(3) - Eigen::RowVector4d(0, 0, 0, 1)).norm() > 1e-10)
      throw std::runtime_error(name + ": T_cam_imu is not a rigid transform");
  }
  if (c.camera[0].width != c.camera[1].width || c.camera[0].height != c.camera[1].height)
    throw std::runtime_error("the two cameras must have the same dimensions");
  c.accel_bias = vector<3>(root["imu"]["accelerometer_bias"], "imu.accelerometer_bias");
  c.gyro_bias = vector<3>(root["imu"]["gyroscope_bias"], "imu.gyroscope_bias") * (M_PI / 180.0);
  const YAML::Node imu_time_offset = root["ROS.ImuTimeOffsetSec"];
  if (imu_time_offset && !imu_time_offset.IsNull()) {
    c.imu_time_offset = imu_time_offset.as<double>();
    if (!std::isfinite(c.imu_time_offset) || std::abs(c.imu_time_offset) > 10)
      throw std::runtime_error("invalid IMU time offset");
  }
  c.gyro_noise = positive(root, "gyroscope_noise_density");
  c.gyro_walk = positive(root, "gyroscope_random_walk");
  c.accel_noise = positive(root, "accelerometer_noise_density");
  c.accel_walk = positive(root, "accelerometer_random_walk");
  return c;
}

ov_msckf::VioManagerOptions Calibration::make_options(bool half_resolution) const {
  ov_msckf::VioManagerOptions p;
  const auto path = ament_index_cpp::get_package_share_directory("ov_msckf") + "/config/euroc_mav/estimator_config.yaml";
  auto parser = std::make_shared<ov_core::YamlParser>(path);
  p.print_and_load(parser);
  if (!parser->successful()) throw std::runtime_error("cannot load baseline OpenVINS estimator configuration: " + path);

  p.state_options.num_cameras = 2;
  p.state_options.do_calib_imu_intrinsics = false;
  p.state_options.do_calib_imu_g_sensitivity = false;
  p.calib_camimu_dt = 0;
  p.downsample_cameras = half_resolution;
  p.use_stereo = true;
  p.use_aruco = false;
  p.use_multi_threading_subs = false;
  p.use_multi_threading_pubs = false;
  p.num_opencv_threads = 1;
  p.use_mask = false;
  p.imu_noises.sigma_w = gyro_noise;
  p.imu_noises.sigma_wb = gyro_walk;
  p.imu_noises.sigma_a = accel_noise;
  p.imu_noises.sigma_ab = accel_walk;
  auto &init = p.init_options;
  init.num_cameras = 2;
  init.use_stereo = true;
  init.downsample_cameras = half_resolution;
  init.calib_camimu_dt = 0;
  init.sigma_w = gyro_noise;
  init.sigma_wb = gyro_walk;
  init.sigma_a = accel_noise;
  init.sigma_ab = accel_walk;
  init.init_dyn_use = true;
  init.init_dyn_mle_opt_calib = false;
  init.init_dyn_mle_max_threads = 1;
  init.init_dyn_mle_max_iter = 50;
  init.init_dyn_mle_max_time = 1e6;
  init.init_dyn_bias_g.setZero();
  init.init_dyn_bias_a.setZero();
  for (int i = 0; i < 2; ++i) {
    Eigen::VectorXd intrinsics = camera[i].intrinsics;
    if (half_resolution) intrinsics.head<4>() *= 0.5;
    const int scale = half_resolution ? 2 : 1;
    auto model = std::make_shared<ov_core::CamEqui>(camera[i].width / scale, camera[i].height / scale);
    model->set_value(intrinsics);
    Eigen::VectorXd extrinsic(7);
    extrinsic.head<4>() = ov_core::rot_2_quat(camera[i].T_cam_imu.block<3, 3>(0, 0));
    extrinsic.tail<3>() = camera[i].T_cam_imu.block<3, 1>(0, 3);
    p.camera_intrinsics[i] = model;
    p.camera_extrinsics[i] = extrinsic;
    init.camera_intrinsics[i] = model;
    init.camera_extrinsics[i] = extrinsic;
  }
  return p;
}

} // namespace openvins_slam
