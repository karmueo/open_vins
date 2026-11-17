#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace openvins_slam {

struct RawImuSample {
  int64_t time_ns = 0;
  Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel = Eigen::Vector3d::Zero();
};

struct RawImageEntry {
  int64_t cam_time_ns = 0;
  std::string image_file;
  std::string image_path;
};

class RawDataset {
public:
  explicit RawDataset(const std::string &directory);

  const std::string &directory() const { return directory_; }
  const std::string &images_directory() const { return images_directory_; }
  const std::string &image_timestamps_path() const { return image_timestamps_path_; }
  const std::string &imu_csv_path() const { return imu_csv_path_; }
  std::size_t imu_count() const { return imu_.size(); }
  std::size_t image_count() const { return images_.size(); }

  // Streams IMU and image entries in non-decreasing nanosecond timestamp order,
  // reading image bytes lazily per frame. Returning false stops early.
  void for_each(const std::function<bool(const RawImuSample &)> &on_imu,
                const std::function<bool(const RawImageEntry &, std::vector<uint8_t> &&, const std::string &format)> &on_image) const;

private:
  std::string directory_;
  std::string images_directory_;
  std::string image_timestamps_path_;
  std::string imu_csv_path_;
  std::vector<RawImuSample> imu_;
  std::vector<RawImageEntry> images_;
};

} // namespace openvins_slam
