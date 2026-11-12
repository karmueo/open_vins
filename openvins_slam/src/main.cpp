#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/path.hpp>
#include "core/VioManager.h"
#include "state/State.h"
#include "utils/sensor_data.h"
#include "openvins_slam/calibration.hpp"
#include "openvins_slam/mcap_io.hpp"
#include "openvins_slam/snapshot.hpp"
#include "openvins_slam/trajectory_writer.hpp"

namespace openvins_slam {
namespace {

constexpr int64_t kNanos = 1000000000LL;

int64_t stamp_ns(const builtin_interfaces::msg::Time &t) {
  if (t.sec < 0 || t.nanosec >= kNanos) throw std::runtime_error("invalid sensor header timestamp");
  return static_cast<int64_t>(t.sec) * kNanos + t.nanosec;
}

template <class T>
T deserialize(const mcap::Message &message) {
  rclcpp::SerializedMessage serialized(message.dataSize);
  auto &raw = serialized.get_rcl_serialized_message();
  if (message.dataSize) std::memcpy(raw.buffer, message.data, message.dataSize);
  raw.buffer_length = message.dataSize;
  T result;
  rclcpp::Serialization<T> serializer;
  serializer.deserialize_message(&serialized, &result);
  return result;
}

template <class T>
void write_message(McapOutput *out, const std::string &topic, const std::string &type, const T &message, int64_t time_ns) {
  if (!out) return;
  rclcpp::SerializedMessage serialized;
  rclcpp::Serialization<T> serializer;
  serializer.serialize_message(&message, &serialized);
  out->write_serialized(topic, type, serialized, static_cast<uint64_t>(time_ns));
}

struct ImuSample {
  int64_t ns;
  Eigen::Vector3d gyro;
  Eigen::Vector3d accel;
};

struct CameraSample {
  int64_t ns;
  sensor_msgs::msg::CompressedImage image;
};

class BagProcessor {
public:
  BagProcessor(const rclcpp::Node::SharedPtr &node, const Calibration &calibration, bool realtime,
               bool apply_bias, bool record_debug, const std::string &imu_topic, const std::string &stereo_topic,
               McapOutput *output, TrajectoryWriter *trajectory, bool half_resolution)
      : node_(node), calibration_(calibration), realtime_(realtime), apply_bias_(apply_bias),
        record_debug_(record_debug), imu_topic_(imu_topic), stereo_topic_(stereo_topic), output_(output), trajectory_(trajectory) {
    auto options = calibration_.make_options(half_resolution);
    options.try_zupt = node_->declare_parameter<bool>("try_zupt", options.try_zupt);
    RCLCPP_INFO(node_->get_logger(), "ZUPT %s", options.try_zupt ? "enabled" : "disabled");
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
    if (realtime_) node_->set_parameter(rclcpp::Parameter("use_sim_time", true));
    offset_ns_ = static_cast<int64_t>(std::llround(calibration_.imu_time_offset * kNanos));
    started_ = std::chrono::steady_clock::now();
  }

  bool consume(const mcap::MessageView &view) {
    if (!rclcpp::ok()) return false;
    if (realtime_) pace(view.message.logTime);
    if (!rclcpp::ok()) return false;
    ++source_messages_;
    if (output_) output_->write_original(view);
    const auto &topic = view.channel->topic;
    if (topic == imu_topic_) {
      if (!view.schema || view.schema->name != "sensor_msgs/msg/Imu" || view.channel->messageEncoding != "cdr")
        throw std::runtime_error("IMU topic must be sensor_msgs/msg/Imu in CDR format");
      add_imu(deserialize<sensor_msgs::msg::Imu>(view.message));
    } else if (topic == stereo_topic_) {
      if (!view.schema || view.schema->name != "sensor_msgs/msg/CompressedImage" || view.channel->messageEncoding != "cdr")
        throw std::runtime_error("stereo topic must be sensor_msgs/msg/CompressedImage in CDR format");
      add_camera(deserialize<sensor_msgs::msg::CompressedImage>(view.message));
    }
    if ((source_messages_ % 2000) == 0) rclcpp::spin_some(node_);
    return rclcpp::ok();
  }

  void finish() {
    process_ready();
    skipped_tail_ += pending_.size();
    pending_.clear();
    if (imu_count_ == 0) throw std::runtime_error("input contains no messages for IMU topic " + imu_topic_);
    if (image_count_ == 0) throw std::runtime_error("input contains no messages for stereo topic " + stereo_topic_);
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
  }

  void report_performance(std::chrono::steady_clock::duration total_duration) const {
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

  void hold() {
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

private:
  template <typename Fn>
  void measure_slam(Fn &&feed) {
    if (realtime_) {
      feed();
      return;
    }
    const auto start = std::chrono::steady_clock::now();
    feed();
    slam_compute_duration_ += std::chrono::steady_clock::now() - start;
  }

  void pace(uint64_t log_ns) {
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

  void set_anchor(int64_t sensor_ns) {
    if (!have_anchor_) { anchor_ns_ = sensor_ns; have_anchor_ = true; }
  }

  void add_imu(const sensor_msgs::msg::Imu &msg) {
    const int64_t raw_ns = stamp_ns(msg.header.stamp);
    if (imu_count_ && raw_ns <= last_imu_raw_ns_) throw std::runtime_error("IMU header timestamps must increase strictly");
    last_imu_raw_ns_ = raw_ns;
    ++imu_count_;
    set_anchor(raw_ns);
    if ((offset_ns_ > 0 && raw_ns > std::numeric_limits<int64_t>::max() - offset_ns_) ||
        (offset_ns_ < 0 && raw_ns < -offset_ns_)) throw std::runtime_error("IMU time offset overflows timestamp");
    ImuSample sample;
    sample.ns = raw_ns + offset_ns_;
    sample.gyro = Eigen::Vector3d(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
    sample.accel = Eigen::Vector3d(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
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

  void add_camera(sensor_msgs::msg::CompressedImage msg) {
    const int64_t ns = stamp_ns(msg.header.stamp);
    if (image_count_ && ns <= last_camera_ns_) throw std::runtime_error("camera header timestamps must increase strictly");
    last_camera_ns_ = ns;
    ++image_count_;
    set_anchor(ns);
    pending_.push_back({ns, std::move(msg)});
    process_ready();
  }

  void process_ready() {
    while (!pending_.empty() && !imu_.empty()) {
      const auto t = pending_.front().ns;
      if (t < imu_.front().ns) {
        ++skipped_early_;
        pending_.pop_front();
        continue;
      }
      if (imu_.back().ns <= t) break; // keep a real sample on each side for interpolation
      while (imu_.size() > 2 && imu_[1].ns <= t) imu_.pop_front();
      if (imu_.size() < 2 || imu_[0].ns > t || imu_[1].ns <= t) throw std::runtime_error("IMU bracketing failed");
      const double alpha = double(t - imu_[0].ns) / double(imu_[1].ns - imu_[0].ns);
      const Eigen::Vector3d gyro = (1.0 - alpha) * imu_[0].gyro + alpha * imu_[1].gyro;
      const double dt = double(imu_[1].ns - imu_[0].ns) / kNanos;
      const double gyro_var = ((1.0 - alpha) * (1.0 - alpha) + alpha * alpha) *
                              calibration_.gyro_noise * calibration_.gyro_noise / dt;
      process_image(pending_.front(), gyro, gyro_var);
      pending_.pop_front();
    }
  }

  void process_image(const CameraSample &sample, const Eigen::Vector3d &gyro, double gyro_var) {
    const auto &bytes = sample.image.data;
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
    if (realtime_) {
      last_clock_.clock = ros_time(sample.ns);
      has_clock_ = true;
      clock_publisher_->publish(last_clock_);
    }
    if (!has_static_tf_) {
      static_tf_ = make_static_tf(calibration_, sample.ns);
      has_static_tf_ = true;
      write_message(output_, "/tf_static", "tf2_msgs/msg/TFMessage", static_tf_, sample.ns);
      if (realtime_) static_tf_publisher_->publish(static_tf_);
    }
    if ((realtime_ || (output_ && record_debug_))) publish_debug(sample.ns);
    if (!manager_->initialized()) return;
    auto state = manager_->get_state();
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
    if (realtime_) publish_last();
  }

  void publish_debug(int64_t ns) {
    const auto image = manager_->get_historical_viz_image();
    if (!image.empty()) {
      std_msgs::msg::Header h;
      h.frame_id = "cam0_optical";
      h.stamp = ros_time(ns);
      last_image_ = *cv_bridge::CvImage(h, "bgr8", image).toImageMsg();
      has_image_ = true;
      if (realtime_) image_publisher_->publish(last_image_);
      if (record_debug_) write_message(output_, "/openvins/tracking_image", "sensor_msgs/msg/Image", last_image_, ns);
    }
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
    if (realtime_) points_publisher_->publish(last_points_);
    if (record_debug_) write_message(output_, "/openvins/points", "sensor_msgs/msg/PointCloud2", last_points_, ns);
  }

  void publish_last(bool include_debug = false) {
    for (int i = 0; i < 3; ++i) {
      pose_publishers_[i]->publish(last_snapshot_.poses[i]);
      path_publishers_[i]->publish(paths_[i]);
    }
    odom_publisher_->publish(last_snapshot_.odom);
    tf_publisher_->publish(last_snapshot_.tf);
    if (include_debug && has_image_) image_publisher_->publish(last_image_);
    if (include_debug && has_points_) points_publisher_->publish(last_points_);
  }

  rclcpp::Node::SharedPtr node_;
  const Calibration calibration_;
  bool realtime_, apply_bias_, record_debug_;
  std::string imu_topic_, stereo_topic_;
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

} // namespace
} // namespace openvins_slam

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto node = std::make_shared<rclcpp::Node>("slam_from_bag");
    const auto cameras = node->declare_parameter<std::string>("cameras", "");
    const auto in_bag = node->declare_parameter<std::string>("in_bag", "");
    const auto outbag = node->declare_parameter<std::string>("outbag", "");
    const bool realtime = node->declare_parameter<bool>("realtime", false);
    const bool half_resolution = node->declare_parameter<bool>("half_resolution", true);
    const bool apply_imu_bias = node->declare_parameter<bool>("apply_imu_bias", true);
    const bool record_debug = node->declare_parameter<bool>("record_debug", false);
    const bool save_trajectory = node->declare_parameter<bool>("save_trajectory", false);
    const auto trajectory_file = node->declare_parameter<std::string>("trajectory_file", "");
    const auto stereo_topic = node->declare_parameter<std::string>("stereo_topic", "/stereo/image_raw");
    const auto imu_topic = node->declare_parameter<std::string>("imu_topic", "/imu");
    if (in_bag.empty()) throw std::runtime_error("parameter in_bag is required");
    std::string calibration_path = cameras;
    if (calibration_path.empty()) {
      const auto inferred_path = std::filesystem::path(in_bag).parent_path() / "calib.yaml";
      std::error_code filesystem_error;
      if (!std::filesystem::is_regular_file(inferred_path, filesystem_error)) {
        throw std::runtime_error("parameter cameras is empty and calib.yaml was not found next to input MCAP: " +
                                 inferred_path.string() + "; pass -p cameras:=/path/to/calib.yaml");
      }
      calibration_path = inferred_path.string();
      RCLCPP_INFO(node->get_logger(), "Parameter cameras not set; using calibration file: %s", calibration_path.c_str());
    }
    if (!realtime && outbag.empty()) throw std::runtime_error("outbag is required when realtime=false");
    if (stereo_topic.empty() || imu_topic.empty() || stereo_topic == imu_topic) throw std::runtime_error("invalid input topic names");
    auto calibration = openvins_slam::Calibration::load(calibration_path);
    const double file_offset = calibration.imu_time_offset;
    const bool offset_overridden =
        node->get_node_parameters_interface()->get_parameter_overrides().count("imu_time_offset_sec") > 0;
    calibration.imu_time_offset = node->declare_parameter<double>("imu_time_offset_sec", file_offset);
    if (!std::isfinite(calibration.imu_time_offset) || std::abs(calibration.imu_time_offset) > 10)
      throw std::runtime_error("imu_time_offset_sec must be finite and within +/-10 seconds");
    if (offset_overridden) {
      RCLCPP_WARN(node->get_logger(), "Explicit IMU time correction: %.9f s (calibration file: %.9f s)",
                  calibration.imu_time_offset, file_offset);
    } else if (file_offset != 0.0) {
      RCLCPP_INFO(node->get_logger(), "Using IMU time correction from calibration: %.9f s", calibration.imu_time_offset);
    } else {
      RCLCPP_INFO(node->get_logger(), "Using IMU time correction: %.9f s (calibration value or 0.0 if absent)",
                  calibration.imu_time_offset);
    }
    openvins_slam::McapInput input(in_bag);
    std::unique_ptr<openvins_slam::TrajectoryWriter> trajectory;
    if (save_trajectory) trajectory = std::make_unique<openvins_slam::TrajectoryWriter>(trajectory_file, in_bag, outbag);
    std::unique_ptr<openvins_slam::McapOutput> output;
    if (!outbag.empty()) output = std::make_unique<openvins_slam::McapOutput>(outbag, in_bag, input);
    openvins_slam::BagProcessor processor(node, calibration, realtime, apply_imu_bias, record_debug,
                                          imu_topic, stereo_topic, output.get(), trajectory.get(), half_resolution);
    const auto processing_start = std::chrono::steady_clock::now();
    input.for_each([&](const mcap::MessageView &view) { return processor.consume(view); });
    if (!rclcpp::ok()) throw std::runtime_error("processing interrupted; output was not committed");
    processor.finish();
    if (!rclcpp::ok()) throw std::runtime_error("processing interrupted; outputs were not committed");
    if (trajectory) trajectory->prepare();
    if (!rclcpp::ok()) throw std::runtime_error("processing interrupted; outputs were not committed");
    bool output_committed = false;
    if (output) { output->commit(); output_committed = true; }
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
    const auto processing_end = std::chrono::steady_clock::now();
    if (!realtime && rclcpp::ok()) processor.report_performance(processing_end - processing_start);
    if (realtime) processor.hold();
  } catch (const std::exception &e) {
    std::cerr << "slam_from_bag: " << e.what() << std::endl;
    result = rclcpp::ok() ? 1 : 130;
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return result;
}
