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

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace {

sensor_msgs::msg::CameraInfo with_image_header(const sensor_msgs::msg::CameraInfo &base_info,
                                               const sensor_msgs::msg::Image::ConstSharedPtr &image_msg) {
  sensor_msgs::msg::CameraInfo info = base_info;
  info.header = image_msg->header;
  if (info.header.frame_id.empty()) {
    info.header.frame_id = base_info.header.frame_id;
  }
  return info;
}

} // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("run_stereo_camera_info", options);

  std::string config_path = "config/euroc_mav/estimator_config.yaml";
  std::string left_image_topic = "/cam0/image_raw";
  std::string right_image_topic = "/cam1/image_raw";
  std::string left_info_topic = "/cam0/camera_info";
  std::string right_info_topic = "/cam1/camera_info";
  std::string left_camera = "cam0";
  std::string right_camera = "cam1";
  node->get_parameter_or<std::string>("config_path", config_path, config_path);
  node->get_parameter_or<std::string>("left_image_topic", left_image_topic, left_image_topic);
  node->get_parameter_or<std::string>("right_image_topic", right_image_topic, right_image_topic);
  node->get_parameter_or<std::string>("left_info_topic", left_info_topic, left_info_topic);
  node->get_parameter_or<std::string>("right_info_topic", right_info_topic, right_info_topic);
  node->get_parameter_or<std::string>("left_camera", left_camera, left_camera);
  node->get_parameter_or<std::string>("right_camera", right_camera, right_camera);

  const auto camera_infos = ov_msckf::make_stereo_camera_info_from_config(config_path, left_camera, right_camera);
  auto camera_info_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  auto left_pub = node->create_publisher<sensor_msgs::msg::CameraInfo>(left_info_topic, camera_info_qos);
  auto right_pub = node->create_publisher<sensor_msgs::msg::CameraInfo>(right_info_topic, camera_info_qos);

  auto left_sub = node->create_subscription<sensor_msgs::msg::Image>(
      left_image_topic, rclcpp::SensorDataQoS(),
      [camera_infos, left_pub](const sensor_msgs::msg::Image::ConstSharedPtr msg) {
        left_pub->publish(with_image_header(camera_infos.left, msg));
      });
  auto right_sub = node->create_subscription<sensor_msgs::msg::Image>(
      right_image_topic, rclcpp::SensorDataQoS(),
      [camera_infos, right_pub](const sensor_msgs::msg::Image::ConstSharedPtr msg) {
        right_pub->publish(with_image_header(camera_infos.right, msg));
      });

  RCLCPP_INFO(node->get_logger(), "Publishing CameraInfo from %s for %s and %s", config_path.c_str(), left_image_topic.c_str(),
              right_image_topic.c_str());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
