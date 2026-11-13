#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <algorithm>
#include <deque>
#include <Eigen/Core>

namespace openvins_slam {
struct ImuSample {
  int64_t ns;
  Eigen::Vector3d gyro;
  Eigen::Vector3d accel;
};

struct GyroSample {
  Eigen::Vector3d gyro;
  double variance;
};

inline void retain_imu_window(std::deque<ImuSample> &samples, int64_t oldest) {
  while (samples.size() > 2 && samples[1].ns <= oldest) samples.pop_front();
}

inline GyroSample interpolate_gyro(const std::deque<ImuSample> &samples, int64_t target, double noise) {
  const auto after = std::upper_bound(samples.begin(), samples.end(), target,
                                     [](int64_t ns, const ImuSample &sample) { return ns < sample.ns; });
  if (after == samples.begin() || after == samples.end()) throw std::runtime_error("IMU bracketing failed");
  const auto before = std::prev(after);
  const double alpha = double(target - before->ns) / double(after->ns - before->ns);
  const double dt = double(after->ns - before->ns) / 1000000000.0;
  return {(1 - alpha) * before->gyro + alpha * after->gyro,
          ((1 - alpha) * (1 - alpha) + alpha * alpha) * noise * noise / dt};
}

// Work in integer sensor time: no rounding of epoch-sized floating point seconds.
inline int64_t imu_target_ns(int64_t camera_ns, double residual) {
  const long double delta = std::round(static_cast<long double>(residual) * 1000000000.L);
  const long double target = static_cast<long double>(camera_ns) + delta;
  if (!std::isfinite(residual) || target < 0 || target > std::numeric_limits<int64_t>::max())
    throw std::runtime_error("online camera-IMU time residual is non-finite or overflows sensor time");
  return static_cast<int64_t>(target);
}

enum class ImuCoverage { Early, Wait, Ready };
inline ImuCoverage imu_coverage(int64_t target, int64_t first, int64_t retained, int64_t latest,
                               bool propagated, int64_t previous) {
  if (propagated && target <= previous)
    throw std::runtime_error("online camera-IMU time residual causes backwards IMU propagation");
  if (target < first) return ImuCoverage::Early;
  if (target < retained) throw std::runtime_error("online time calibration lost required IMU history");
  return latest <= target ? ImuCoverage::Wait : ImuCoverage::Ready;
}
} // namespace openvins_slam
