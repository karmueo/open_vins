/**
 * @file raw_io.hpp
 * @brief 声明 RAW 数据集及按时间顺序遍历图像和 IMU 的接口。
 */
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
  /**
   * @brief 加载图片索引和 IMU 样本，可同时预加载压缩图像。
   * @param[in] directory RAW 数据目录。
   * @param[in] cache 是否预加载全部压缩图像。
   * @param[in] should_cancel 可选的取消回调，每次预加载读图前调用。
   * @throws std::runtime_error 预加载期间取消，或 RAW 输入读取失败。
   */
  explicit RawDataset(const std::string &directory, bool cache = true,
                      const std::function<bool()> &should_cancel = {});

  const std::string &directory() const { return directory_; }
  const std::string &images_directory() const { return images_directory_; }
  const std::string &image_timestamps_path() const { return image_timestamps_path_; }
  const std::string &imu_csv_path() const { return imu_csv_path_; }
  std::size_t imu_count() const { return imu_.size(); }
  std::size_t image_count() const { return images_.size(); }

  /**
   * @brief 按时间戳归并图像和 IMU，按缓存设置获取图像字节。
   * @param[in] on_imu IMU 样本回调，返回 false 时提前停止。
   * @param[in] on_image 图片回调，返回 false 时提前停止。
   */
  void for_each(const std::function<bool(const RawImuSample &)> &on_imu,
                const std::function<bool(const RawImageEntry &, std::vector<uint8_t> &&, const std::string &format)> &on_image) const;

private:
  std::string directory_;
  std::string images_directory_;
  std::string image_timestamps_path_;
  std::string imu_csv_path_;
  std::vector<RawImuSample> imu_;
  std::vector<RawImageEntry> images_;
  std::vector<std::vector<uint8_t>> cached_images_; ///< 按图片索引保存的压缩字节，空表示逐帧读图。
};

} // namespace openvins_slam
