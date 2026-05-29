/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "utils/StereoCameraInfo.h"

#include "utils/opencv_yaml_parse.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include <array>
#include <cassert>

namespace ov_msckf {
namespace {

struct CameraCalibration {
  std::string frame_id;
  std::vector<double> intrinsics;
  std::vector<double> distortion_coeffs;
  std::vector<int> resolution;
  Eigen::Matrix4d T_imu_cam = Eigen::Matrix4d::Identity();
};

CameraCalibration load_camera_calibration(ov_core::YamlParser &parser, const std::string &camera_name) {
  CameraCalibration calibration;
  calibration.frame_id = camera_name;

  parser.parse_external("relative_config_imucam", camera_name, "intrinsics", calibration.intrinsics);
  parser.parse_external("relative_config_imucam", camera_name, "distortion_coeffs", calibration.distortion_coeffs);
  parser.parse_external("relative_config_imucam", camera_name, "resolution", calibration.resolution);
  parser.parse_external("relative_config_imucam", camera_name, "T_imu_cam", calibration.T_imu_cam);
  calibration.T_imu_cam.block<1, 3>(3, 0).setZero();
  calibration.T_imu_cam(3, 3) = 1.0;

  assert(calibration.intrinsics.size() == 4);
  assert(calibration.distortion_coeffs.size() == 4);
  assert(calibration.resolution.size() == 2);

  return calibration;
}

cv::Mat camera_matrix(const std::vector<double> &intrinsics) {
  cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
  K.at<double>(0, 0) = intrinsics.at(0);
  K.at<double>(1, 1) = intrinsics.at(1);
  K.at<double>(0, 2) = intrinsics.at(2);
  K.at<double>(1, 2) = intrinsics.at(3);
  return K;
}

cv::Mat distortion_vector(const std::vector<double> &distortion_coeffs) {
  cv::Mat D = cv::Mat::zeros(5, 1, CV_64F);
  D.at<double>(0) = distortion_coeffs.at(0);
  D.at<double>(1) = distortion_coeffs.at(1);
  D.at<double>(2) = distortion_coeffs.at(2);
  D.at<double>(3) = distortion_coeffs.at(3);
  return D;
}

void fill_original_intrinsics(sensor_msgs::msg::CameraInfo &info, const CameraCalibration &calibration) {
  info.header.frame_id = calibration.frame_id;
  info.width = static_cast<uint32_t>(calibration.resolution.at(0));
  info.height = static_cast<uint32_t>(calibration.resolution.at(1));
  info.distortion_model = "plumb_bob";

  info.d.assign(5, 0.0);
  for (size_t i = 0; i < calibration.distortion_coeffs.size(); i++) {
    info.d.at(i) = calibration.distortion_coeffs.at(i);
  }

  info.k.fill(0.0);
  info.k.at(0) = calibration.intrinsics.at(0);
  info.k.at(2) = calibration.intrinsics.at(2);
  info.k.at(4) = calibration.intrinsics.at(1);
  info.k.at(5) = calibration.intrinsics.at(3);
  info.k.at(8) = 1.0;
}

void fill_mat3(std::array<double, 9> &dest, const cv::Mat &src) {
  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 3; c++) {
      dest.at(3 * r + c) = src.at<double>(r, c);
    }
  }
}

void fill_mat34(std::array<double, 12> &dest, const cv::Mat &src) {
  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 4; c++) {
      dest.at(4 * r + c) = src.at<double>(r, c);
    }
  }
}

} // namespace

StereoCameraInfoPair make_stereo_camera_info_from_config(const std::string &config_path, const std::string &left_camera,
                                                         const std::string &right_camera) {
  ov_core::YamlParser parser(config_path);
  CameraCalibration left = load_camera_calibration(parser, left_camera);
  CameraCalibration right = load_camera_calibration(parser, right_camera);

  assert(left.resolution.at(0) == right.resolution.at(0));
  assert(left.resolution.at(1) == right.resolution.at(1));

  Eigen::Matrix4d T_right_left = Eigen::Matrix4d::Identity();
  T_right_left.block<3, 3>(0, 0) = right.T_imu_cam.block<3, 3>(0, 0).transpose() * left.T_imu_cam.block<3, 3>(0, 0);
  T_right_left.block<3, 1>(0, 3) =
      right.T_imu_cam.block<3, 3>(0, 0).transpose() * (left.T_imu_cam.block<3, 1>(0, 3) - right.T_imu_cam.block<3, 1>(0, 3));
  cv::Mat R_right_left(3, 3, CV_64F);
  cv::Mat t_right_left(3, 1, CV_64F);
  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 3; c++) {
      R_right_left.at<double>(r, c) = T_right_left(r, c);
    }
    t_right_left.at<double>(r) = T_right_left(r, 3);
  }

  cv::Mat R_left, R_right, P_left, P_right, Q;
  const cv::Size image_size(left.resolution.at(0), left.resolution.at(1));
  cv::stereoRectify(camera_matrix(left.intrinsics), distortion_vector(left.distortion_coeffs), camera_matrix(right.intrinsics),
                    distortion_vector(right.distortion_coeffs), image_size, R_right_left, t_right_left, R_left, R_right, P_left,
                    P_right, Q, cv::CALIB_ZERO_DISPARITY, 0.0, image_size);

  StereoCameraInfoPair infos;
  fill_original_intrinsics(infos.left, left);
  fill_original_intrinsics(infos.right, right);
  fill_mat3(infos.left.r, R_left);
  fill_mat3(infos.right.r, R_right);
  fill_mat34(infos.left.p, P_left);
  fill_mat34(infos.right.p, P_right);
  return infos;
}

} // namespace ov_msckf
