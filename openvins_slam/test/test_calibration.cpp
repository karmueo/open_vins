#include <gtest/gtest.h>
#include <cmath>
#include <fstream>
#include <string>
#include <unistd.h>
#include "openvins_slam/calibration.hpp"
#include "utils/quat_ops.h"

namespace {

class CalibrationTest : public ::testing::Test {
protected:
  void SetUp() override {
    char name[] = "/tmp/openvins_calibration_XXXXXX";
    const int fd = mkstemp(name);
    ASSERT_GE(fd, 0);
    close(fd);
    path_ = name;
    std::ofstream out(path_);
    out << R"(
cam0:
  T_cam_imu: [[1, 0, 0, 45], [0, 1, 0, 2], [0, 0, 1, -19], [0, 0, 0, 1]]
  intrinsics: [600, 601, 960, 540]
  distortion_model: fisheye
  distortion_coeffs: [0.1, 0.2, 0.3, 0.4]
  resolution: [1920, 1080]
cam1:
  T_cam_imu: [[1, 0, 0, -55], [0, 1, 0, 2], [0, 0, 1, -19], [0, 0, 0, 1]]
  intrinsics: [602, 603, 961, 541]
  distortion_model: fisheye
  distortion_coeffs: [0.2, 0.3, 0.4, 0.5]
  resolution: [1920, 1080]
imu:
  accelerometer_bias: [1, 2, 3]
  gyroscope_bias: [180, 90, 0]
accelerometer_noise_density: 0.006
accelerometer_random_walk: 0.003
gyroscope_noise_density: 0.001
gyroscope_random_walk: 0.0002
ROS.ImuTimeOffsetSec: -0.005
)";
  }
  void TearDown() override { if (!path_.empty()) unlink(path_.c_str()); }
  std::string path_;
};

TEST_F(CalibrationTest, ConvertsUnitsAndBothOptionSets) {
  const auto c = openvins_slam::Calibration::load(path_);
  EXPECT_NEAR(c.camera[0].T_cam_imu(0, 3), 0.045, 1e-12);
  EXPECT_NEAR(c.camera[1].T_cam_imu(0, 3), -0.055, 1e-12);
  EXPECT_NEAR(c.gyro_bias.x(), M_PI, 1e-12);
  EXPECT_NEAR(c.gyro_bias.y(), M_PI / 2, 1e-12);
  EXPECT_DOUBLE_EQ(c.accel_bias.y(), 2);
  EXPECT_DOUBLE_EQ(c.imu_time_offset, -0.005);
  const auto p = c.make_options(true);
  EXPECT_DOUBLE_EQ(p.calib_camimu_dt, 0);
  EXPECT_DOUBLE_EQ(p.init_options.calib_camimu_dt, 0);
  EXPECT_TRUE(p.downsample_cameras);
  EXPECT_TRUE(p.init_options.downsample_cameras);
  EXPECT_EQ(p.camera_intrinsics.at(0)->w(), 960);
  EXPECT_EQ(p.init_options.camera_intrinsics.at(0)->h(), 540);
  EXPECT_DOUBLE_EQ(p.camera_intrinsics.at(0)->get_value()(0), 300);
  EXPECT_DOUBLE_EQ(p.init_options.camera_intrinsics.at(1)->get_value()(2), 480.5);
  EXPECT_NEAR(p.camera_extrinsics.at(0)(4), 0.045, 1e-12);
  EXPECT_TRUE((ov_core::quat_2_Rot(p.camera_extrinsics.at(0).head(4)) - Eigen::Matrix3d::Identity()).norm() < 1e-12);
  EXPECT_DOUBLE_EQ(p.imu_noises.sigma_w, 0.001);
  EXPECT_DOUBLE_EQ(p.init_options.sigma_a, 0.006);
  EXPECT_FALSE(p.state_options.do_calib_camera_pose);
  EXPECT_FALSE(p.state_options.do_calib_camera_intrinsics);
  EXPECT_FALSE(p.state_options.do_calib_camera_timeoffset);
  EXPECT_FALSE(p.use_multi_threading_subs);
  EXPECT_EQ(p.num_opencv_threads, 1);
  EXPECT_TRUE(p.init_options.init_dyn_use);
  const auto full = c.make_options(false);
  EXPECT_EQ(full.camera_intrinsics.at(0)->w(), 1920);
  EXPECT_DOUBLE_EQ(full.camera_intrinsics.at(0)->get_value()(0), 600);
}

TEST_F(CalibrationTest, MissingTimeOffsetDefaultsToZero) {
  std::ifstream in(path_);
  std::string s((std::istreambuf_iterator<char>(in)), {});
  const auto pos = s.find("ROS.ImuTimeOffsetSec:");
  ASSERT_NE(pos, std::string::npos);
  const auto end = s.find('\n', pos);
  ASSERT_NE(end, std::string::npos);
  s.erase(pos, end - pos + 1);
  std::ofstream(path_) << s;
  const auto c = openvins_slam::Calibration::load(path_);
  EXPECT_DOUBLE_EQ(c.imu_time_offset, 0);
}

TEST_F(CalibrationTest, RejectsInvalidTimeOffset) {
  std::ifstream in(path_);
  std::string s((std::istreambuf_iterator<char>(in)), {});
  const std::string field = "ROS.ImuTimeOffsetSec: -0.005";
  const auto pos = s.find(field);
  ASSERT_NE(pos, std::string::npos);
  s.replace(pos, field.size(), "ROS.ImuTimeOffsetSec: 11.0");
  std::ofstream(path_) << s;
  EXPECT_THROW(openvins_slam::Calibration::load(path_), std::runtime_error);
}

TEST_F(CalibrationTest, RejectsUnsupportedModel) {
  std::ifstream in(path_);
  std::string s((std::istreambuf_iterator<char>(in)), {});
  const auto pos = s.find("distortion_model: fisheye");
  ASSERT_NE(pos, std::string::npos);
  s.replace(pos, std::string("distortion_model: fisheye").size(), "distortion_model: unsupported");
  std::ofstream(path_) << s;
  EXPECT_THROW(openvins_slam::Calibration::load(path_), std::runtime_error);
}

} // namespace
