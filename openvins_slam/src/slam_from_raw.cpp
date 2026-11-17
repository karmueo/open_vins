#include <chrono>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "openvins_slam/calibration.hpp"
#include "openvins_slam/mcap_io.hpp"
#include "openvins_slam/output_paths.hpp"
#include "openvins_slam/raw_io.hpp"
#include "openvins_slam/runtime_options.hpp"
#include "openvins_slam/snapshot.hpp"
#include "openvins_slam/trajectory_writer.hpp"
#include "openvins_slam/vio_processor.hpp"

namespace {

template <class T>
void write_sensor_message(openvins_slam::McapOutput *out, const std::string &topic, const std::string &type,
                          const T &message, int64_t time_ns) {
  if (!out) return;
  rclcpp::SerializedMessage serialized;
  rclcpp::Serialization<T> serializer;
  serializer.serialize_message(&message, &serialized);
  out->write_serialized(topic, type, serialized, static_cast<uint64_t>(time_ns), false);
}

std::filesystem::path normalized_dir_path(const std::string &dir) {
  std::string trimmed = dir;
  while (trimmed.size() > 1 && (trimmed.back() == '/' || trimmed.back() == '\\')) trimmed.pop_back();
  return std::filesystem::path(trimmed);
}

} // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto node = std::make_shared<rclcpp::Node>("slam_from_raw");
    const auto cameras = node->declare_parameter<std::string>("cameras", "");
    const auto in_raw = node->declare_parameter<std::string>("in_raw", "");
    openvins_slam::reject_legacy_parameters(*node);
    const auto outdir = node->declare_parameter<std::string>("outdir", "");
    const bool save_mcap = node->declare_parameter<bool>("save_mcap", true);
    const bool visualize = node->declare_parameter<bool>("rviz2", false);
    const auto config_name = openvins_slam::resolve_config_name(*node);
    const auto paths = openvins_slam::raw_output_paths(outdir, in_raw, config_name, save_mcap);
    const auto &outbag = paths.mcap;
    RCLCPP_INFO(node->get_logger(), "TUM trajectory: %s", paths.trajectory.c_str());
    if (save_mcap) RCLCPP_INFO(node->get_logger(), "Output MCAP: %s", outbag.c_str());
    const bool realtime = node->declare_parameter<bool>("realtime", false);
    const bool half_resolution = node->declare_parameter<bool>("half_resolution", true);
    const bool apply_imu_bias = node->declare_parameter<bool>("apply_imu_bias", true);
    const bool record_debug = node->declare_parameter<bool>("record_debug", false);
    const bool record_points = node->declare_parameter<bool>("record_points", true);
    const auto stereo_topic = node->declare_parameter<std::string>("stereo_topic", "/stereo/image_raw");
    const auto imu_topic = node->declare_parameter<std::string>("imu_topic", "/imu");
    if (in_raw.empty()) throw std::runtime_error("parameter in_raw is required");
    const auto raw_root = normalized_dir_path(in_raw);
    const auto local_calib = raw_root / "calib.yaml";
    const auto parent_calib = raw_root.parent_path() / "calib.yaml";
    const auto calibration_path = openvins_slam::resolve_calibration_path(
        cameras, {local_calib, parent_calib}, node->get_logger(),
        "calib.yaml was not found in " + raw_root.string() + " or " + raw_root.parent_path().string());
    if (stereo_topic.empty() || imu_topic.empty() || stereo_topic == imu_topic)
      throw std::runtime_error("invalid input topic names");
    auto calibration = openvins_slam::Calibration::load(calibration_path);
    openvins_slam::resolve_imu_time_offset(*node, calibration);
    auto options = calibration.make_options(half_resolution);
    openvins_slam::configure_options(*node, options);
    openvins_slam::RawDataset input(in_raw);
    RCLCPP_INFO(node->get_logger(), "Raw input: images=%zu (%s), imu=%zu (%s)",
                input.image_count(), input.image_timestamps_path().c_str(),
                input.imu_count(), input.imu_csv_path().c_str());
    auto trajectory = std::make_unique<openvins_slam::TrajectoryWriter>(paths.trajectory, raw_root.string(), outbag);
    std::unique_ptr<openvins_slam::McapOutput> output;
    if (!outbag.empty()) output = std::make_unique<openvins_slam::McapOutput>(outbag);
    openvins_slam::VioProcessor processor(node, calibration, realtime, visualize, apply_imu_bias, record_debug,
                                          record_points, "raw CSV " + input.imu_csv_path(),
                                          "raw images " + input.image_timestamps_path(),
                                          output.get(), trajectory.get(), options);
    const auto processing_start = std::chrono::steady_clock::now();
    input.for_each(
        [&](const openvins_slam::RawImuSample &sample) {
          if (!rclcpp::ok()) return false;
          if (realtime) processor.pace(static_cast<uint64_t>(sample.time_ns));
          if (!rclcpp::ok()) return false;
          processor.on_source_message();
          if (output) {
            sensor_msgs::msg::Imu msg;
            msg.header.stamp = openvins_slam::ros_time(sample.time_ns);
            msg.header.frame_id = "imu";
            msg.angular_velocity.x = sample.gyro.x();
            msg.angular_velocity.y = sample.gyro.y();
            msg.angular_velocity.z = sample.gyro.z();
            msg.linear_acceleration.x = sample.accel.x();
            msg.linear_acceleration.y = sample.accel.y();
            msg.linear_acceleration.z = sample.accel.z();
            write_sensor_message(output.get(), imu_topic, "sensor_msgs/msg/Imu", msg, sample.time_ns);
          }
          processor.add_imu(sample.time_ns, sample.gyro, sample.accel);
          return rclcpp::ok();
        },
        [&](const openvins_slam::RawImageEntry &entry, std::vector<uint8_t> &&bytes, const std::string &format) {
          if (!rclcpp::ok()) return false;
          if (realtime) processor.pace(static_cast<uint64_t>(entry.cam_time_ns));
          if (!rclcpp::ok()) return false;
          processor.on_source_message();
          if (output) {
            sensor_msgs::msg::CompressedImage msg;
            msg.header.stamp = openvins_slam::ros_time(entry.cam_time_ns);
            msg.header.frame_id = "cam0_optical";
            msg.format = format;
            msg.data = bytes;
            write_sensor_message(output.get(), stereo_topic, "sensor_msgs/msg/CompressedImage", msg, entry.cam_time_ns);
          }
          processor.add_camera(entry.cam_time_ns, std::move(bytes));
          return rclcpp::ok();
        });
    if (!rclcpp::ok()) throw std::runtime_error("processing interrupted; output was not committed");
    processor.finish();
    openvins_slam::commit_outputs(trajectory.get(), output.get(), outbag);
    const auto processing_end = std::chrono::steady_clock::now();
    if (!realtime && rclcpp::ok()) processor.report_performance(processing_end - processing_start);
    if (realtime || visualize) processor.hold();
  } catch (const std::exception &e) {
    std::cerr << "slam_from_raw: " << e.what() << std::endl;
    result = rclcpp::ok() ? 1 : 130;
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return result;
}
