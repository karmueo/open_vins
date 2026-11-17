#include "openvins_slam/vio_processor.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <utility>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rcl/arguments.h>
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "state/State.h"
#include "utils/sensor_data.h"

namespace openvins_slam {
namespace {

constexpr int64_t kNanos = 1000000000LL;

template <class T>
void write_message(McapOutput *out, const std::string &topic, const std::string &type, const T &message, int64_t time_ns) {
  if (!out) return;
  rclcpp::SerializedMessage serialized;
  rclcpp::Serialization<T> serializer;
  serializer.serialize_message(&message, &serialized);
  out->write_serialized(topic, type, serialized, static_cast<uint64_t>(time_ns));
}

} // namespace

int64_t VioProcessor::stamp_ns(const builtin_interfaces::msg::Time &t) {
  if (t.sec < 0 || t.nanosec >= kNanos) throw std::runtime_error("invalid sensor header timestamp");
  return static_cast<int64_t>(t.sec) * kNanos + t.nanosec;
}

VioProcessor::VioProcessor(const rclcpp::Node::SharedPtr &node, const Calibration &calibration, bool realtime,
                           bool visualize, bool apply_bias, bool record_debug, bool record_points,
                           std::string imu_source, std::string stereo_source, McapOutput *output,
                           TrajectoryWriter *trajectory, ov_msckf::VioManagerOptions options)
    : node_(node), calibration_(calibration), realtime_(realtime), publish_results_(realtime || visualize),
      apply_bias_(apply_bias), record_debug_(record_debug), record_points_(record_points),
      imu_source_(std::move(imu_source)), stereo_source_(std::move(stereo_source)), output_(output),
      trajectory_(trajectory) {
  online_time_ = options.state_options.do_calib_camera_timeoffset;
  online_extrinsics_ = options.state_options.do_calib_camera_pose;
  init_window_time_ = options.init_options.init_window_time;
  manager_ = std::make_unique<ov_msckf::VioManager>(options);
  auto regular = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  for (int i = 0; i < 3; ++i) {
    pose_publishers_[i] = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(pose_names_[i], regular);
    path_publishers_[i] = node_->create_publisher<nav_msgs::msg::Path>(path_names_[i], regular);
    paths_[i].header.frame_id = "odom";
  }
  odom_publisher_ = node_->create_publisher<nav_msgs::msg::Odometry>("/openvins/odom", regular);
  tf_publisher_ = node_->create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(100).reliable());
  static_tf_publisher_ = node_->create_publisher<tf2_msgs::msg::TFMessage>("/tf_static", regular);
  image_publisher_ = node_->create_publisher<sensor_msgs::msg::Image>("/openvins/tracking_image", regular);
  points_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/openvins/points", regular);
  clock_publisher_ = node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::QoS(1).reliable());
  if (publish_results_) node_->set_parameter(rclcpp::Parameter("use_sim_time", true));
  offset_ns_ = static_cast<int64_t>(std::llround(calibration_.imu_time_offset * kNanos));
  started_ = std::chrono::steady_clock::now();
}

template <typename Fn>
void VioProcessor::measure_slam(Fn &&feed) {
  if (realtime_) {
    feed();
    return;
  }
  const auto start = std::chrono::steady_clock::now();
  feed();
  slam_compute_duration_ += std::chrono::steady_clock::now() - start;
}

void VioProcessor::pace(uint64_t log_ns) {
  if (!have_pace_start_) {
    first_log_ns_ = log_ns;
    wall_start_ = std::chrono::steady_clock::now();
    have_pace_start_ = true;
  }
  if (log_ns < first_log_ns_) throw std::runtime_error("input log timestamps are not monotonic");
  const auto target = wall_start_ + std::chrono::nanoseconds(log_ns - first_log_ns_);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < target) {
    const auto remaining = target - std::chrono::steady_clock::now();
    std::this_thread::sleep_for(std::min(remaining, std::chrono::steady_clock::duration(std::chrono::milliseconds(20))));
  }
}

void VioProcessor::on_source_message() {
  ++source_messages_;
  if ((source_messages_ % 2000) == 0) rclcpp::spin_some(node_);
}

void VioProcessor::set_anchor(int64_t sensor_ns) {
  if (!have_anchor_) { anchor_ns_ = sensor_ns; have_anchor_ = true; }
}

void VioProcessor::add_imu(int64_t raw_ns, const Eigen::Vector3d &gyro, const Eigen::Vector3d &accel) {
  if (imu_count_ && raw_ns <= last_imu_raw_ns_) throw std::runtime_error("IMU header timestamps must increase strictly");
  last_imu_raw_ns_ = raw_ns;
  ++imu_count_;
  set_anchor(raw_ns);
  if ((offset_ns_ > 0 && raw_ns > std::numeric_limits<int64_t>::max() - offset_ns_) ||
      (offset_ns_ < 0 && raw_ns < -offset_ns_)) throw std::runtime_error("IMU time offset overflows timestamp");
  ImuSample sample;
  sample.ns = raw_ns + offset_ns_;
  if (imu_count_ == 1) first_imu_ns_ = sample.ns;
  sample.gyro = gyro;
  sample.accel = accel;
  if (!sample.gyro.allFinite() || !sample.accel.allFinite()) throw std::runtime_error("non-finite IMU reading");
  if (apply_bias_) { sample.gyro -= calibration_.gyro_bias; sample.accel -= calibration_.accel_bias; }
  ov_core::ImuData data;
  data.timestamp = static_cast<double>(sample.ns - anchor_ns_) / kNanos;
  data.wm = sample.gyro;
  data.am = sample.accel;
  measure_slam([&] { manager_->feed_measurement_imu(data); });
  imu_.push_back(sample);
  process_ready();
}

void VioProcessor::add_camera(int64_t ns, std::vector<uint8_t> compressed_bytes) {
  if (image_count_ && ns <= last_camera_ns_) throw std::runtime_error("camera header timestamps must increase strictly");
  last_camera_ns_ = ns;
  ++image_count_;
  set_anchor(ns);
  pending_.push_back({ns, std::move(compressed_bytes)});
  process_ready();
}

void VioProcessor::process_ready() {
  while (!pending_.empty() && !imu_.empty()) {
    const double residual = online_time_ ? manager_->get_state()->_calib_dt_CAMtoIMU->value()(0) : 0.0;
    const auto t = imu_target_ns(pending_.front().ns, residual);
    const auto coverage = imu_coverage(t, first_imu_ns_, imu_.front().ns, imu_.back().ns,
                                       online_time_ && have_propagation_, last_propagation_ns_);
    if (coverage == ImuCoverage::Early) {
      ++skipped_early_;
      pending_.pop_front();
      continue;
    }
    if (coverage == ImuCoverage::Wait) break;
    // Match the core's retained window; a later negative residual can require older readings.
    int64_t retain = t;
    if (online_time_) {
      const auto state = manager_->get_state();
      double oldest = state->margtimestep();
      if (!manager_->initialized() || !std::isfinite(oldest))
        oldest = double(t - anchor_ns_) / kNanos - init_window_time_ - 0.10;
      retain = std::min(t, anchor_ns_ + static_cast<int64_t>(oldest * kNanos));
    }
    retain_imu_window(imu_, retain);
    const auto gyro = interpolate_gyro(imu_, t, calibration_.gyro_noise);
    process_image(pending_.front(), gyro.gyro, gyro.variance);
    const auto state = manager_->get_state();
    if (manager_->initialized() && std::abs(state->_timestamp - double(pending_.front().ns - anchor_ns_) / kNanos) < 1e-8) {
      last_propagation_ns_ = t;
      have_propagation_ = true;
    }
    pending_.pop_front();
  }
}

void VioProcessor::process_image(const CameraSample &sample, const Eigen::Vector3d &gyro, double gyro_var) {
  const auto &bytes = sample.compressed_bytes;
  if (bytes.empty()) throw std::runtime_error("empty compressed stereo image");
  cv::Mat compressed(1, static_cast<int>(bytes.size()), CV_8UC1, const_cast<unsigned char *>(bytes.data()));
  cv::Mat stereo = cv::imdecode(compressed, cv::IMREAD_GRAYSCALE);
  if (stereo.empty()) throw std::runtime_error("cannot decode compressed stereo image");
  const int width = calibration_.camera[0].width;
  const int height = calibration_.camera[0].height;
  if (stereo.cols != width * 2 || stereo.rows != height) throw std::runtime_error("stereo image dimensions differ from calibration");
  ov_core::CameraData input;
  input.timestamp = double(sample.ns - anchor_ns_) / kNanos;
  input.sensor_ids = {0, 1};
  input.images = {stereo(cv::Rect(0, 0, width, height)), stereo(cv::Rect(width, 0, width, height))};
  input.masks = {cv::Mat::zeros(height, width, CV_8UC1), cv::Mat::zeros(height, width, CV_8UC1)};
  measure_slam([&] { manager_->feed_measurement_camera(input); });
  ++fed_images_;
  if (publish_results_) {
    last_clock_.clock = ros_time(sample.ns);
    has_clock_ = true;
    clock_publisher_->publish(last_clock_);
  }
  if (!online_extrinsics_ && !has_static_tf_) {
    static_tf_ = make_static_tf(calibration_, sample.ns);
    has_static_tf_ = true;
    write_message(output_, "/tf_static", "tf2_msgs/msg/TFMessage", static_tf_, sample.ns);
    if (publish_results_) static_tf_publisher_->publish(static_tf_);
  }
  if (publish_results_ || (output_ && (record_debug_ || record_points_))) publish_debug(sample.ns);
  if (!manager_->initialized()) return;
  auto state = manager_->get_state();
  imu_target_ns(sample.ns, state->_calib_dt_CAMtoIMU->value()(0));
  if (std::abs(state->_timestamp - input.timestamp) > 1e-8) return;
  if (!initialization_ns_) initialization_ns_ = sample.ns;
  const auto accepted_features = manager_->get_good_features_MSCKF().size();
  msckf_updates_ += accepted_features;
  frames_with_msckf_ += accepted_features != 0;
  max_slam_features_ = std::max(max_slam_features_, state->_features_SLAM.size());
  last_snapshot_ = make_snapshot(state, calibration_, sample.ns, gyro, gyro_var);
  has_snapshot_ = true;
  if (trajectory_) trajectory_->write(sample.ns, last_snapshot_.odom.pose.pose);
  for (int i = 0; i < 3; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = last_snapshot_.poses[i].header;
    pose.pose = last_snapshot_.poses[i].pose.pose;
    paths_[i].header = pose.header;
    paths_[i].poses.push_back(pose);
    write_message(output_, pose_names_[i], "geometry_msgs/msg/PoseWithCovarianceStamped", last_snapshot_.poses[i], sample.ns);
  }
  write_message(output_, "/openvins/odom", "nav_msgs/msg/Odometry", last_snapshot_.odom, sample.ns);
  write_message(output_, "/tf", "tf2_msgs/msg/TFMessage", last_snapshot_.tf, sample.ns);
  ++output_count_;
  if (publish_results_) publish_last();
}

void VioProcessor::publish_debug(int64_t ns) {
  if (publish_results_ || (output_ && record_debug_)) {
    const auto image = manager_->get_historical_viz_image();
    if (!image.empty()) {
      std_msgs::msg::Header h;
      h.frame_id = "cam0_optical";
      h.stamp = ros_time(ns);
      last_image_ = *cv_bridge::CvImage(h, "bgr8", image).toImageMsg();
      has_image_ = true;
      if (publish_results_) image_publisher_->publish(last_image_);
      if (record_debug_) write_message(output_, "/openvins/tracking_image", "sensor_msgs/msg/Image", last_image_, ns);
    }
  }
  if (!publish_results_ && !(output_ && (record_points_ || record_debug_))) return;
  double current_time = -1;
  std::unordered_map<size_t, Eigen::Vector3d> positions, uvd;
  manager_->get_active_tracks(current_time, positions, uvd);
  if (std::abs(current_time - double(ns - anchor_ns_) / kNanos) > 1e-8) return;
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "odom";
  cloud.header.stamp = ros_time(ns);
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  size_t n = 0;
  for (const auto &entry : positions) if (entry.second.allFinite()) ++n;
  modifier.resize(n);
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
  for (const auto &entry : positions) {
    if (!entry.second.allFinite()) continue;
    *x = static_cast<float>(entry.second.x()); *y = static_cast<float>(entry.second.y()); *z = static_cast<float>(entry.second.z());
    ++x; ++y; ++z;
  }
  last_points_ = std::move(cloud);
  has_points_ = true;
  if (publish_results_) points_publisher_->publish(last_points_);
  if (record_points_ || record_debug_) write_message(output_, "/openvins/points", "sensor_msgs/msg/PointCloud2", last_points_, ns);
}

void VioProcessor::publish_last(bool include_debug) {
  for (int i = 0; i < 3; ++i) {
    pose_publishers_[i]->publish(last_snapshot_.poses[i]);
    path_publishers_[i]->publish(paths_[i]);
  }
  odom_publisher_->publish(last_snapshot_.odom);
  tf_publisher_->publish(last_snapshot_.tf);
  if (include_debug && has_image_) image_publisher_->publish(last_image_);
  if (include_debug && has_points_) points_publisher_->publish(last_points_);
}

void VioProcessor::finish() {
  process_ready();
  skipped_tail_ += pending_.size();
  pending_.clear();
  if (imu_count_ == 0) throw std::runtime_error("input contains no messages for IMU " + imu_source_);
  if (image_count_ == 0) throw std::runtime_error("input contains no messages for stereo " + stereo_source_);
  if (output_count_ == 0) throw std::runtime_error("OpenVINS did not initialize or produce an updated state");
  // Record each complete trajectory once, at its last accepted pose time.
  // Live Path publishing remains available during processing.
  for (int i = 0; i < 3; ++i)
    write_message(output_, path_names_[i], "nav_msgs/msg/Path", paths_[i], stamp_ns(paths_[i].header.stamp));
  std::cout << "SUMMARY source_messages=" << source_messages_ << " imu=" << imu_count_
            << " images=" << image_count_ << " fed_images=" << fed_images_ << " pose_updates=" << output_count_
            << " skipped_no_early_imu=" << skipped_early_ << " skipped_no_tail_imu=" << skipped_tail_
            << " msckf_updates=" << msckf_updates_ << " frames_with_msckf=" << frames_with_msckf_
            << " max_slam_features=" << max_slam_features_
            << " initialized_at_sensor_ns=" << initialization_ns_
            << " elapsed_seconds=" << std::chrono::duration<double>(std::chrono::steady_clock::now() - started_).count()
            << std::endl;
  const auto state = manager_->get_state();
  const double residual = state->_calib_dt_CAMtoIMU->value()(0);
  std::cout << std::setprecision(12) << "FINAL_CALIBRATION fixed_imu_correction_sec=" << calibration_.imu_time_offset
            << " residual_cam_to_corrected_imu_sec=" << residual
            << " effective_raw_imu_correction_sec=" << calibration_.imu_time_offset - residual << '\n';
  for (int i = 0; i < 2; ++i) {
    const auto camera = state->_cam_intrinsics_cameras.at(i);
    std::cout << "FINAL_CALIBRATION cam" << i << " tracking_resolution=" << camera->w() << 'x' << camera->h()
              << " intrinsics=" << state->_cam_intrinsics.at(i)->value().transpose()
              << " extrinsics_q_ItoC_p_IinC=" << state->_calib_IMUtoCAM.at(i)->value().transpose() << '\n';
  }
}

void VioProcessor::report_performance(std::chrono::steady_clock::duration total_duration) const {
  const double total_seconds = std::chrono::duration<double>(total_duration).count();
  const double slam_seconds = std::chrono::duration<double>(slam_compute_duration_).count();
  const double average_fps = total_seconds > 0 ? static_cast<double>(fed_images_) / total_seconds : 0.0;
  const double slam_average_fps = slam_seconds > 0 ? static_cast<double>(fed_images_) / slam_seconds : 0.0;
  // Keep formatting local so the existing summary and other logs retain their precision.
  std::ostringstream line;
  line << std::fixed << std::setprecision(6)
       << "PERFORMANCE processed_frames=" << fed_images_
       << " total_seconds=" << total_seconds << " average_fps=" << average_fps
       << " slam_compute_seconds=" << slam_seconds << " slam_average_fps=" << slam_average_fps;
  std::cout << line.str() << std::endl;
}

void VioProcessor::hold() {
  while (rclcpp::ok()) {
    try {
      if (has_snapshot_) publish_last(true);
      if (has_clock_) clock_publisher_->publish(last_clock_);
      if (has_static_tf_) static_tf_publisher_->publish(static_tf_);
      rclcpp::spin_some(node_);
    } catch (const std::exception &) {
      // Ctrl+C may invalidate the ROS context between ok() and publish().
      if (!rclcpp::ok()) return;
      throw;
    }
    for (int i = 0; i < 10 && rclcpp::ok(); ++i) std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void reject_legacy_parameters(rclcpp::Node &node) {
  const auto &overrides = node.get_node_parameters_interface()->get_parameter_overrides();
  for (const auto *legacy : {"outbag", "trajectory_file", "save_trajectory", "rviz_enable"})
    if (overrides.count(legacy))
      throw std::runtime_error(std::string("removed parameter ") + legacy + "; use outdir, save_mcap and rviz2");
}

std::string resolve_config_name(rclcpp::Node &node) {
  const auto &overrides = node.get_node_parameters_interface()->get_parameter_overrides();
  std::string inferred_name;
  if (!overrides.count("config_name")) {
    const auto context = node.get_node_base_interface()->get_context()->get_rcl_context();
    const auto allocator = rcl_get_default_allocator();
    char **files = nullptr;
    const int count = rcl_arguments_get_param_files_count(&context->global_arguments);
    if (rcl_arguments_get_param_files(&context->global_arguments, allocator, &files) != RCL_RET_OK)
      throw std::runtime_error("cannot read ROS parameter file names");
    if (count > 0) inferred_name = std::filesystem::path(files[count - 1]).stem().string();
    for (int i = 0; i < count; ++i) allocator.deallocate(files[i], allocator.state);
    if (files) allocator.deallocate(files, allocator.state);
  }
  return node.declare_parameter<std::string>("config_name", inferred_name);
}

std::string resolve_calibration_path(const std::string &cameras,
                                     const std::vector<std::filesystem::path> &candidates,
                                     const rclcpp::Logger &logger, const std::string &missing_hint) {
  if (!cameras.empty()) return cameras;
  std::error_code filesystem_error;
  for (const auto &candidate : candidates) {
    if (std::filesystem::is_regular_file(candidate, filesystem_error)) {
      RCLCPP_INFO(logger, "Parameter cameras not set; using calibration file: %s", candidate.c_str());
      return candidate.string();
    }
  }
  throw std::runtime_error("parameter cameras is empty and " + missing_hint + "; pass -p cameras:=/path/to/calib.yaml");
}

void resolve_imu_time_offset(rclcpp::Node &node, Calibration &calibration) {
  const double file_offset = calibration.imu_time_offset;
  const bool offset_overridden =
      node.get_node_parameters_interface()->get_parameter_overrides().count("imu_time_offset_sec") > 0;
  calibration.imu_time_offset = node.declare_parameter<double>("imu_time_offset_sec", file_offset);
  if (!std::isfinite(calibration.imu_time_offset) || std::abs(calibration.imu_time_offset) > 10)
    throw std::runtime_error("imu_time_offset_sec must be finite and within +/-10 seconds");
  if (offset_overridden) {
    RCLCPP_WARN(node.get_logger(), "Explicit IMU time correction: %.9f s (calibration file: %.9f s)",
                calibration.imu_time_offset, file_offset);
  } else if (file_offset != 0.0) {
    RCLCPP_INFO(node.get_logger(), "Using IMU time correction from calibration: %.9f s", calibration.imu_time_offset);
  } else {
    RCLCPP_INFO(node.get_logger(), "Using IMU time correction: %.9f s (calibration value or 0.0 if absent)",
                calibration.imu_time_offset);
  }
}

void commit_outputs(TrajectoryWriter *trajectory, McapOutput *output, const std::string &outbag) {
  if (!rclcpp::ok()) throw std::runtime_error("processing interrupted; outputs were not committed");
  if (trajectory) trajectory->prepare();
  if (!rclcpp::ok()) throw std::runtime_error("processing interrupted; outputs were not committed");
  bool output_committed = false;
  if (output) {
    output->commit();
    output_committed = true;
  }
  if (trajectory) {
    try {
      if (!rclcpp::ok()) throw std::runtime_error("processing interrupted before trajectory file commit");
      trajectory->commit();
    } catch (const std::exception &e) {
      if (output_committed)
        throw std::runtime_error("output MCAP already committed at " + outbag + "; trajectory file not committed: " + e.what());
      throw;
    }
  }
}

} // namespace openvins_slam
