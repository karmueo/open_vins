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

#include <cassert>
#include <cmath>
#include <iostream>

namespace {

int check(bool condition, const std::string &message) {
  if (!condition) {
    std::cerr << message << std::endl;
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}

} // namespace

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cerr << "usage: test_stereo_camera_info <estimator_config.yaml>" << std::endl;
    return EXIT_FAILURE;
  }

  const auto infos = ov_msckf::make_stereo_camera_info_from_config(argv[1], "cam0", "cam1");

  if (check(infos.left.header.frame_id == "cam0", "left frame id mismatch") != EXIT_SUCCESS)
    return EXIT_FAILURE;
  if (check(infos.right.header.frame_id == "cam1", "right frame id mismatch") != EXIT_SUCCESS)
    return EXIT_FAILURE;
  if (check(infos.left.width == 752 && infos.left.height == 480, "left resolution mismatch") != EXIT_SUCCESS)
    return EXIT_FAILURE;
  if (check(infos.right.width == 752 && infos.right.height == 480, "right resolution mismatch") != EXIT_SUCCESS)
    return EXIT_FAILURE;

  if (check(infos.left.distortion_model == "plumb_bob", "left distortion model mismatch") != EXIT_SUCCESS)
    return EXIT_FAILURE;
  if (check(infos.right.distortion_model == "plumb_bob", "right distortion model mismatch") != EXIT_SUCCESS)
    return EXIT_FAILURE;
  if (check(infos.left.d.size() == 5 && infos.right.d.size() == 5, "distortion vector size mismatch") != EXIT_SUCCESS)
    return EXIT_FAILURE;
  if (check(std::abs(infos.left.d.at(0) - -0.28340811) < 1e-8, "left distortion mismatch") != EXIT_SUCCESS)
    return EXIT_FAILURE;
  if (check(std::abs(infos.right.d.at(0) - -0.28368365) < 1e-8, "right distortion mismatch") != EXIT_SUCCESS)
    return EXIT_FAILURE;

  if (check(std::abs(infos.left.k.at(0) - 458.654) < 1e-8, "left K mismatch") != EXIT_SUCCESS)
    return EXIT_FAILURE;
  if (check(std::abs(infos.right.k.at(0) - 457.587) < 1e-8, "right K mismatch") != EXIT_SUCCESS)
    return EXIT_FAILURE;
  if (check(infos.left.r.at(0) != 0.0 && infos.right.r.at(0) != 0.0, "rectification R was not filled") != EXIT_SUCCESS)
    return EXIT_FAILURE;
  if (check(infos.left.p.at(0) > 300.0 && infos.left.p.at(0) < 600.0, "left rectified focal length is unreasonable") !=
      EXIT_SUCCESS)
    return EXIT_FAILURE;
  if (check(infos.left.p.at(2) > 0.0 && infos.left.p.at(2) < static_cast<double>(infos.left.width),
            "left rectified cx is outside image bounds") != EXIT_SUCCESS)
    return EXIT_FAILURE;
  if (check(infos.left.p.at(5) > 300.0 && infos.left.p.at(5) < 600.0, "left rectified fy is unreasonable") != EXIT_SUCCESS)
    return EXIT_FAILURE;
  if (check(infos.left.p.at(6) > 0.0 && infos.left.p.at(6) < static_cast<double>(infos.left.height),
            "left rectified cy is outside image bounds") != EXIT_SUCCESS)
    return EXIT_FAILURE;
  if (check(infos.right.p.at(3) < -1e-3, "right projection baseline has the wrong sign") != EXIT_SUCCESS)
    return EXIT_FAILURE;

  return EXIT_SUCCESS;
}
