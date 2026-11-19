#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "openvins_slam/calibration.hpp"
#include "openvins_slam/imu_window.hpp"
#include "openvins_slam/mcap_io.hpp"
#include "openvins_slam/snapshot.hpp"
#include "openvins_slam/trajectory_writer.hpp"

namespace openvins_slam {

class VioProcessor {
public:
  VioProcessor(const rclcpp::Node::SharedPtr &node, const Calibration &calibration, bool realtime,
               bool visualize, bool apply_bias, bool record_debug, bool record_points, std::string imu_source,
               std::string stereo_source, McapOutput *output, TrajectoryWriter *trajectory,
               ov_msckf::VioManagerOptions options);

  static int64_t stamp_ns(const builtin_interfaces::msg::Time &t);

  void pace(uint64_t log_ns);
  void on_source_message();
  void add_imu(int64_t raw_ns, const Eigen::Vector3d &gyro, const Eigen::Vector3d &accel);
  void add_camera(int64_t ns, std::vector<uint8_t> compressed_bytes);
  void finish();
  void report_performance(std::chrono::steady_clock::duration total_duration) const;
  void hold();

private:
  struct CameraSample {
    int64_t ns;
    std::vector<uint8_t> compressed_bytes;
  };

  template <typename Fn>
  void measure_slam(Fn &&feed);

  void set_anchor(int64_t sensor_ns);
  void process_ready();
  void process_image(const CameraSample &sample, const Eigen::Vector3d &gyro, double gyro_var);
  void publish_debug(int64_t ns);
  void publish_last(bool include_debug = false);

  rclcpp::Node::SharedPtr node_;
  const Calibration calibration_;
  bool publish_results_, apply_bias_, record_debug_, record_points_;
  bool online_time_ = false, online_extrinsics_ = false, have_propagation_ = false;
  double init_window_time_ = 2.0;
  int64_t first_imu_ns_ = 0, last_propagation_ns_ = 0;
  std::string imu_source_, stereo_source_;
  McapOutput *output_;
  TrajectoryWriter *trajectory_;
  std::unique_ptr<ov_msckf::VioManager> manager_;
  std::deque<ImuSample> imu_;
  std::deque<CameraSample> pending_;
  int64_t offset_ns_ = 0, anchor_ns_ = 0, last_imu_raw_ns_ = 0, last_camera_ns_ = 0;
  int64_t initialization_ns_ = 0;
  bool have_anchor_ = false, have_pace_start_ = false;
  uint64_t first_log_ns_ = 0, source_messages_ = 0, imu_count_ = 0, image_count_ = 0;
  uint64_t fed_images_ = 0, output_count_ = 0, skipped_early_ = 0, skipped_tail_ = 0;
  uint64_t msckf_updates_ = 0, frames_with_msckf_ = 0, max_slam_features_ = 0;
  std::chrono::steady_clock::time_point wall_start_, started_;
  std::chrono::steady_clock::duration slam_compute_duration_{};
  const std::array<std::string, 3> pose_names_ = {"/openvins/pose_imu", "/openvins/pose_cam0", "/openvins/pose_cam1"};
  const std::array<std::string, 3> path_names_ = {"/openvins/path_imu", "/openvins/path_cam0", "/openvins/path_cam1"};
  std::array<nav_msgs::msg::Path, 3> paths_;
  std::array<rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr, 3> pose_publishers_;
  std::array<rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr, 3> path_publishers_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_publisher_, static_tf_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_publisher_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;
  Snapshot last_snapshot_;
  tf2_msgs::msg::TFMessage static_tf_;
  sensor_msgs::msg::Image last_image_;
  sensor_msgs::msg::PointCloud2 last_points_;
  rosgraph_msgs::msg::Clock last_clock_;
  bool has_snapshot_ = false, has_static_tf_ = false, has_image_ = false, has_points_ = false, has_clock_ = false;
};

// Shared node setup and transactional commit helpers for slam_from_bag and slam_from_raw.
void reject_legacy_parameters(rclcpp::Node &node);
std::string resolve_config_name(rclcpp::Node &node);
std::string resolve_calibration_path(const std::string &cameras,
                                     const std::vector<std::filesystem::path> &candidates,
                                     const rclcpp::Logger &logger, const std::string &missing_hint);
void resolve_imu_time_offset(rclcpp::Node &node, Calibration &calibration);
void commit_outputs(TrajectoryWriter *trajectory, McapOutput *output, const std::string &outbag);

} // namespace openvins_slam
