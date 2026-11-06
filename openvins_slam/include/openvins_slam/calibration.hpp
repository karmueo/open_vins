#pragma once

#include <array>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "core/VioManagerOptions.h"

namespace openvins_slam {

struct CameraCalibration {
  Eigen::Matrix4d T_cam_imu = Eigen::Matrix4d::Identity();
  Eigen::Matrix<double, 8, 1> intrinsics;
  int width = 0;
  int height = 0;
};

struct Calibration {
  std::array<CameraCalibration, 2> camera;
  Eigen::Vector3d accel_bias;
  Eigen::Vector3d gyro_bias;
  double imu_time_offset = 0;
  double gyro_noise = 0;
  double gyro_walk = 0;
  double accel_noise = 0;
  double accel_walk = 0;

  static Calibration load(const std::string &path);
  ov_msckf::VioManagerOptions make_options(bool half_resolution) const;
};

} // namespace openvins_slam
