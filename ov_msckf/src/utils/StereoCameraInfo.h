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

#ifndef OV_MSCKF_STEREO_CAMERA_INFO_H
#define OV_MSCKF_STEREO_CAMERA_INFO_H

#include <Eigen/Eigen>
#include <sensor_msgs/msg/camera_info.hpp>

#include <string>
#include <vector>

namespace ov_msckf {

struct StereoCameraInfoPair {
  sensor_msgs::msg::CameraInfo left;
  sensor_msgs::msg::CameraInfo right;
};

StereoCameraInfoPair make_stereo_camera_info_from_config(const std::string &config_path, const std::string &left_camera,
                                                         const std::string &right_camera);

} // namespace ov_msckf

#endif // OV_MSCKF_STEREO_CAMERA_INFO_H
