#pragma once

#include <cstdint>
#include <cstdio>
#include <string>

#include <geometry_msgs/msg/pose.hpp>

namespace openvins_slam {

// Writes rows to a sibling temporary file and publishes it only after prepare().
class TrajectoryWriter {
public:
  TrajectoryWriter(const std::string &requested_path, const std::string &input_path, const std::string &output_path);
  ~TrajectoryWriter();
  TrajectoryWriter(const TrajectoryWriter &) = delete;
  TrajectoryWriter &operator=(const TrajectoryWriter &) = delete;

  void write(int64_t timestamp_ns, const geometry_msgs::msg::Pose &pose);
  void prepare();
  void commit();
  const std::string &path() const { return output_; }

private:
  void write_text(const std::string &text);
  void cleanup() noexcept;

  std::string output_;
  std::string temporary_;
  std::FILE *file_ = nullptr;
  bool prepared_ = false;
};

} // namespace openvins_slam
