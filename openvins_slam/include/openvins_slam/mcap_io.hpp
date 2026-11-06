// Indexed MCAP input and transactional single-file output for ROS 2 bags.
#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include <mcap/reader.hpp>
#include <rclcpp/serialized_message.hpp>

namespace openvins_slam {

// Message payloads and views are valid only during the for_each callback.
class McapInput {
public:
  explicit McapInput(const std::string &path);
  ~McapInput();
  McapInput(const McapInput &) = delete;
  McapInput &operator=(const McapInput &) = delete;

  // Returning false stops iteration early. Parsing problems throw exceptions.
  void for_each(const std::function<bool(const mcap::MessageView &)> &callback);
  mcap::McapReader &reader();

private:
  mcap::McapReader reader_;
};

// Copies input records without interpreting their types and appends ROS 2 CDR
// messages. Until commit succeeds, only a uniquely named temporary file exists.
class McapOutput {
public:
  McapOutput(const std::string &output_path, const std::string &input_path, McapInput &input);
  ~McapOutput();
  McapOutput(const McapOutput &) = delete;
  McapOutput &operator=(const McapOutput &) = delete;

  void write_original(const mcap::MessageView &view);
  void write_serialized(const std::string &topic, const std::string &ros_type,
                        const rclcpp::SerializedMessage &data, uint64_t stamp_ns);
  void commit();

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace openvins_slam
