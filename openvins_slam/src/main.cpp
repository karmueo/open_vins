#include <chrono>
#include <cstring>
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
#include "openvins_slam/runtime_options.hpp"
#include "openvins_slam/trajectory_writer.hpp"
#include "openvins_slam/vio_processor.hpp"

namespace {

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

} // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto node = std::make_shared<rclcpp::Node>("slam_from_bag");
    const auto cameras = node->declare_parameter<std::string>("cameras", "");
    const auto in_bag = node->declare_parameter<std::string>("in_bag", "");
    openvins_slam::reject_legacy_parameters(*node);
    const auto outdir = node->declare_parameter<std::string>("outdir", "");
    const bool save_mcap = node->declare_parameter<bool>("save_mcap", true);
    const bool visualize = node->declare_parameter<bool>("rviz2", false);
    const auto config_name = openvins_slam::resolve_config_name(*node);
    const auto paths = openvins_slam::output_paths(outdir, in_bag, config_name, save_mcap);
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
    if (in_bag.empty()) throw std::runtime_error("parameter in_bag is required");
    const auto inferred_calib = std::filesystem::path(in_bag).parent_path() / "calib.yaml";
    const auto calibration_path = openvins_slam::resolve_calibration_path(
        cameras, {inferred_calib}, node->get_logger(),
        "calib.yaml was not found next to input MCAP: " + inferred_calib.string());
    if (stereo_topic.empty() || imu_topic.empty() || stereo_topic == imu_topic)
      throw std::runtime_error("invalid input topic names");
    auto calibration = openvins_slam::Calibration::load(calibration_path);
    openvins_slam::resolve_imu_time_offset(*node, calibration);
    auto options = calibration.make_options(half_resolution);
    openvins_slam::configure_options(*node, options);
    openvins_slam::McapInput input(in_bag);
    auto trajectory = std::make_unique<openvins_slam::TrajectoryWriter>(paths.trajectory, in_bag, outbag);
    std::unique_ptr<openvins_slam::McapOutput> output;
    if (!outbag.empty()) output = std::make_unique<openvins_slam::McapOutput>(outbag, in_bag, input);
    openvins_slam::VioProcessor processor(node, calibration, realtime, visualize, apply_imu_bias, record_debug,
                                          record_points, "topic " + imu_topic, "topic " + stereo_topic, output.get(),
                                          trajectory.get(), options);
    const auto processing_start = std::chrono::steady_clock::now();
    input.for_each([&](const mcap::MessageView &view) {
      if (!rclcpp::ok()) return false;
      if (realtime) processor.pace(view.message.logTime);
      if (!rclcpp::ok()) return false;
      processor.on_source_message();
      if (output) output->write_original(view);
      const auto &topic = view.channel->topic;
      if (topic == imu_topic) {
        if (!view.schema || view.schema->name != "sensor_msgs/msg/Imu" || view.channel->messageEncoding != "cdr")
          throw std::runtime_error("IMU topic must be sensor_msgs/msg/Imu in CDR format");
        const auto msg = deserialize<sensor_msgs::msg::Imu>(view.message);
        processor.add_imu(openvins_slam::VioProcessor::stamp_ns(msg.header.stamp),
                          Eigen::Vector3d(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z),
                          Eigen::Vector3d(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z));
      } else if (topic == stereo_topic) {
        if (!view.schema || view.schema->name != "sensor_msgs/msg/CompressedImage" || view.channel->messageEncoding != "cdr")
          throw std::runtime_error("stereo topic must be sensor_msgs/msg/CompressedImage in CDR format");
        auto msg = deserialize<sensor_msgs::msg::CompressedImage>(view.message);
        processor.add_camera(openvins_slam::VioProcessor::stamp_ns(msg.header.stamp), std::move(msg.data));
      }
      return rclcpp::ok();
    });
    if (!rclcpp::ok()) throw std::runtime_error("processing interrupted; output was not committed");
    processor.finish();
    openvins_slam::commit_outputs(trajectory.get(), output.get(), outbag);
    const auto processing_end = std::chrono::steady_clock::now();
    if (!realtime && rclcpp::ok()) processor.report_performance(processing_end - processing_start);
    if (realtime || visualize) processor.hold();
  } catch (const std::exception &e) {
    std::cerr << "slam_from_bag: " << e.what() << std::endl;
    result = rclcpp::ok() ? 1 : 130;
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return result;
}
