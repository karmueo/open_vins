#include <gtest/gtest.h>
#include <cmath>
#include <fstream>
#include <limits>
#include <string>
#include <unistd.h>
#include "openvins_slam/calibration.hpp"
#include "utils/quat_ops.h"
#include "openvins_slam/runtime_options.hpp"
#include "openvins_slam/imu_window.hpp"
#include "cam/CamEqui.h"

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
  EXPECT_TRUE(p.state_options.do_calib_camera_pose);
  EXPECT_TRUE(p.state_options.do_calib_camera_intrinsics);
  EXPECT_TRUE(p.state_options.do_calib_camera_timeoffset);
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

class RuntimeOptionsTest : public ::testing::Test {
protected:
  void SetUp() override { if (!rclcpp::ok()) rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(RuntimeOptionsTest, InheritsBaselineAndAppliesOverridesBeforeStateCreation) {
  ov_msckf::VioManagerOptions p;
  p.num_pts = 200;
  p.state_options.do_calib_camera_pose = true;
  auto node = std::make_shared<rclcpp::Node>("inherited_options");
  openvins_slam::configure_options(*node, p);
  EXPECT_EQ(p.num_pts, 200);
  EXPECT_TRUE(p.state_options.do_calib_camera_pose);
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("num_pts", 401), rclcpp::Parameter("init_max_features", 101),
      rclcpp::Parameter("max_clones", 15), rclcpp::Parameter("max_slam", 0),
      rclcpp::Parameter("max_slam_in_update", 30), rclcpp::Parameter("max_msckf_in_update", 80),
      rclcpp::Parameter("fast_threshold", 12), rclcpp::Parameter("grid_x", 6), rclcpp::Parameter("grid_y", 4),
      rclcpp::Parameter("min_px_dist", 7.2), rclcpp::Parameter("calib_cam_extrinsics", false),
      rclcpp::Parameter("calib_cam_intrinsics", false), rclcpp::Parameter("calib_cam_timeoffset", true)});
  auto override_node = std::make_shared<rclcpp::Node>("overridden_options", options);
  openvins_slam::configure_options(*override_node, p);
  EXPECT_EQ(p.num_pts / 2, 200);
  EXPECT_EQ(p.init_options.init_max_features, 101);
  EXPECT_EQ(p.state_options.max_clone_size, 15);
  EXPECT_EQ(p.state_options.max_slam_features, 0);
  EXPECT_EQ(p.state_options.max_slam_in_update, 30);
  EXPECT_EQ(p.state_options.max_msckf_in_update, 80);
  EXPECT_EQ(p.fast_threshold, 12);
  EXPECT_EQ(p.grid_x, 6);
  EXPECT_EQ(p.grid_y, 4);
  EXPECT_EQ(p.min_px_dist, 8);
  EXPECT_FALSE(p.state_options.do_calib_camera_pose);
  EXPECT_FALSE(p.state_options.do_calib_camera_intrinsics);
  EXPECT_TRUE(p.state_options.do_calib_camera_timeoffset);
}

TEST_F(RuntimeOptionsTest, RejectsInvalidCountsDistancesAndTypes) {
  const std::vector<rclcpp::Parameter> cases = {
      {"num_pts", 1}, {"init_max_features", 1}, {"max_clones", 1}, {"max_slam", -1},
      {"max_slam_in_update", 0}, {"max_msckf_in_update", 0}, {"fast_threshold", 0},
      {"grid_x", 0}, {"grid_y", -1}, {"min_px_dist", 0.0}, {"min_px_dist", -1.0},
      {"min_px_dist", std::numeric_limits<double>::infinity()},
      {"min_px_dist", std::numeric_limits<double>::quiet_NaN()},
      {"num_pts", int64_t{2147483648}}, {"num_pts", 2.5}, {"calib_cam_extrinsics", "yes"}};
  for (const auto &parameter : cases) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({parameter});
    rclcpp::Node node("invalid_options", options);
    ov_msckf::VioManagerOptions p;
    EXPECT_THROW(openvins_slam::configure_options(node, p), std::exception) << parameter.get_name();
  }
}

TEST_F(RuntimeOptionsTest, RejectsInitializationFeatureCountsBelowCoreMinimum) {
  for (int count = 2; count < 15; ++count) {
    SCOPED_TRACE(count);
    rclcpp::NodeOptions overrides;
    overrides.parameter_overrides({rclcpp::Parameter("init_max_features", count)});
    rclcpp::Node node("invalid_initialization_features", overrides);
    ov_msckf::VioManagerOptions p;
    try {
      openvins_slam::configure_options(node, p);
      FAIL() << "Expected initialization feature count to be rejected";
    } catch (const std::runtime_error &error) {
      EXPECT_NE(std::string(error.what()).find("init_max_features must be an integer in [15, INT_MAX]"),
                std::string::npos);
    }
  }
}

TEST_F(RuntimeOptionsTest, AcceptsInitializationFeatureCountAtCoreMinimum) {
  rclcpp::NodeOptions overrides;
  overrides.parameter_overrides({rclcpp::Parameter("init_max_features", 15)});
  rclcpp::Node node("minimum_initialization_features", overrides);
  ov_msckf::VioManagerOptions p;
  ASSERT_NO_THROW(openvins_slam::configure_options(node, p));
  EXPECT_EQ(p.init_options.init_max_features, 15);
  // Verify that the accepted boundary also passes the core initializer's validation.
  p.init_options.print_and_load_initializer();
}

TEST_F(RuntimeOptionsTest, RejectsTrackerGeometryThatWouldProduceEmptyGrids) {
  for (const auto &parameter : std::vector<rclcpp::Parameter>{{"grid_x", 65}, {"grid_y", 49}, {"min_px_dist", 48.1}}) {
    rclcpp::NodeOptions overrides;
    overrides.parameter_overrides({parameter});
    rclcpp::Node node("invalid_tracker_geometry", overrides);
    ov_msckf::VioManagerOptions p;
    p.camera_intrinsics[0] = std::make_shared<ov_core::CamEqui>(64, 48);
    EXPECT_THROW(openvins_slam::configure_options(node, p), std::runtime_error);
  }
}

TEST(ImuWindow, HandlesResidualSignCoverageAndInvalidHistory) {
  using namespace openvins_slam;
  constexpr int64_t camera = 1000000000;
  EXPECT_EQ(imu_target_ns(camera, 0.025), 1025000000);
  EXPECT_EQ(imu_target_ns(camera, -0.025), 975000000);
  // Fixed correction applies to readings only; the target contains just the residual.
  EXPECT_EQ(imu_coverage(imu_target_ns(camera, 0.025), 900000000, 950000000, 1010000000, false, 0), ImuCoverage::Wait);
  EXPECT_EQ(imu_coverage(imu_target_ns(camera, -0.025), 900000000, 950000000, 1010000000, false, 0), ImuCoverage::Ready);
  EXPECT_EQ(imu_coverage(800000000, 900000000, 950000000, 1010000000, false, 0), ImuCoverage::Early);
  EXPECT_THROW(imu_coverage(940000000, 900000000, 950000000, 1010000000, false, 0), std::runtime_error);
  EXPECT_THROW(imu_coverage(990000000, 900000000, 950000000, 1010000000, true, 995000000), std::runtime_error);
  EXPECT_THROW(imu_target_ns(camera, std::numeric_limits<double>::quiet_NaN()), std::runtime_error);
  EXPECT_THROW(imu_target_ns(camera, std::numeric_limits<double>::max()), std::runtime_error);
}

TEST(ImuWindow, KeepsSlidingHistoryAndInterpolatesBothResidualSigns) {
  using namespace openvins_slam;
  std::deque<ImuSample> samples;
  for (int i = 0; i <= 10; ++i)
    samples.push_back({1000000000 + i * 10000000, Eigen::Vector3d(i, 2 * i, -i), Eigen::Vector3d::Zero()});
  retain_imu_window(samples, 1020000000);
  EXPECT_EQ(samples.front().ns, 1020000000);
  const auto positive = interpolate_gyro(samples, imu_target_ns(1050000000, 0.015), 0.01);
  const auto negative = interpolate_gyro(samples, imu_target_ns(1050000000, -0.015), 0.01);
  EXPECT_DOUBLE_EQ(positive.gyro.x(), 6.5);
  EXPECT_DOUBLE_EQ(negative.gyro.x(), 3.5);
  EXPECT_DOUBLE_EQ(positive.gyro.y(), 13);
  EXPECT_NEAR(positive.variance, 0.005, 1e-15);
  EXPECT_THROW(interpolate_gyro(samples, 1010000000, 0.01), std::runtime_error);
  EXPECT_THROW(interpolate_gyro(samples, 1110000000, 0.01), std::runtime_error);
}

} // namespace
