#include "openvins_slam/runtime_options.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace openvins_slam {
void configure_options(rclcpp::Node &node, ov_msckf::VioManagerOptions &p) {
  auto integer = [&](const char *name, int &value, int minimum) {
    const auto requested = node.declare_parameter<int64_t>(name, value);
    if (requested < minimum || requested > std::numeric_limits<int>::max())
      throw std::runtime_error(std::string(name) + " must be an integer in [" + std::to_string(minimum) + ", INT_MAX]");
    value = static_cast<int>(requested);
    RCLCPP_INFO(node.get_logger(), "Effective %s=%d", name, value);
  };
  integer("num_pts", p.num_pts, 2);
  integer("init_max_features", p.init_options.init_max_features, 15);
  integer("max_clones", p.state_options.max_clone_size, 2);
  integer("max_slam", p.state_options.max_slam_features, 0);
  integer("max_slam_in_update", p.state_options.max_slam_in_update, 1);
  integer("max_msckf_in_update", p.state_options.max_msckf_in_update, 1);
  integer("fast_threshold", p.fast_threshold, 1);
  integer("grid_x", p.grid_x, 1);
  integer("grid_y", p.grid_y, 1);
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.dynamic_typing = true;
  const auto distance = node.declare_parameter("min_px_dist", rclcpp::ParameterValue(p.min_px_dist), descriptor);
  double pixels;
  if (distance.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
    pixels = distance.get<int64_t>();
  else if (distance.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
    pixels = distance.get<double>();
  else
    throw std::runtime_error("min_px_dist must be a finite positive number");
  if (!std::isfinite(pixels) || pixels <= 0 || pixels > std::numeric_limits<int>::max())
    throw std::runtime_error("min_px_dist must be finite and in (0, INT_MAX]");
  // The core tracker uses an integer pixel exclusion radius.
  p.min_px_dist = static_cast<int>(std::ceil(pixels));
  for (const auto &entry : p.camera_intrinsics) {
    const auto &camera = entry.second;
    if (p.grid_x > camera->w()) throw std::runtime_error("grid_x must not exceed the tracking image width");
    if (p.grid_y > camera->h()) throw std::runtime_error("grid_y must not exceed the tracking image height");
    if (p.min_px_dist > std::min(camera->w(), camera->h()))
      throw std::runtime_error("min_px_dist must not exceed the smaller tracking image dimension");
  }
  auto boolean = [&](const char *name, bool &value) {
    value = node.declare_parameter<bool>(name, value);
    RCLCPP_INFO(node.get_logger(), "Effective %s=%s", name, value ? "true" : "false");
  };
  boolean("calib_cam_intrinsics", p.state_options.do_calib_camera_intrinsics);
  boolean("calib_cam_extrinsics", p.state_options.do_calib_camera_pose);
  boolean("calib_cam_timeoffset", p.state_options.do_calib_camera_timeoffset);
  boolean("try_zupt", p.try_zupt);
  RCLCPP_INFO(node.get_logger(), "Effective min_px_dist=%d (requested %.9g), points_per_camera=%d, init_points_per_camera=%d",
              p.min_px_dist, pixels, p.num_pts / 2, p.init_options.init_max_features / 2);
}
} // namespace openvins_slam
