#pragma once

#include <array>
#include <cstdint>
#include <Eigen/Core>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include "openvins_slam/calibration.hpp"

namespace ov_msckf { class State; }

namespace openvins_slam {

using Matrix15d = Eigen::Matrix<double, 15, 15>;
using Matrix6x15d = Eigen::Matrix<double, 6, 15>;

struct Snapshot {
  std::array<geometry_msgs::msg::PoseWithCovarianceStamped, 3> poses;
  nav_msgs::msg::Odometry odom;
  tf2_msgs::msg::TFMessage tf;
};

// ROS pose covariance uses global fixed-axis perturbations and position first.
Matrix6x15d pose_jacobian(const Eigen::Matrix3d &R_imu_to_global, const Eigen::Vector3d &camera_in_imu);
Matrix6x15d twist_jacobian(const Eigen::Matrix3d &R_global_to_imu, const Eigen::Vector3d &velocity_global);
Snapshot make_snapshot(const std::shared_ptr<ov_msckf::State> &state, const Calibration &calibration,
                       int64_t stamp_ns, const Eigen::Vector3d &gyro_corrected, double gyro_noise_var);
tf2_msgs::msg::TFMessage make_static_tf(const Calibration &calibration, int64_t stamp_ns);
builtin_interfaces::msg::Time ros_time(int64_t stamp_ns);

} // namespace openvins_slam
