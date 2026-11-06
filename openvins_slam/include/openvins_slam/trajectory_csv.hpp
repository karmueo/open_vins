#pragma once

#include <cstdint>
#include <cstdio>
#include <string>

#include <geometry_msgs/msg/pose.hpp>

namespace openvins_slam {

// Writes rows to a sibling temporary file and publishes it only after prepare().
class TrajectoryCsv {
public:
  TrajectoryCsv(const std::string &requested_path, const std::string &input_path, const std::string &output_path);
  ~TrajectoryCsv();
  TrajectoryCsv(const TrajectoryCsv &) = delete;
  TrajectoryCsv &operator=(const TrajectoryCsv &) = delete;

  void write(uint64_t frame_index, int64_t timestamp_ns, uint64_t bag_timestamp_ns,
             int64_t first_image_ns, const geometry_msgs::msg::Pose &pose);
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
