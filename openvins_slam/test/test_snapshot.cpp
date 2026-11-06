#include "openvins_slam/snapshot.hpp"

#include <array>
#include <cmath>

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include "types/IMU.h"
#include "utils/quat_ops.h"

namespace {

using Vector15d = Eigen::Matrix<double, 15, 1>;
using Vector16d = Eigen::Matrix<double, 16, 1>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using openvins_slam::Matrix15d;
using openvins_slam::Matrix6x15d;

struct Pose {
  Eigen::Vector3d position;
  Eigen::Matrix3d rotation;
};

Vector16d nominal_value() {
  Vector16d value;
  const Eigen::Matrix3d global_to_imu =
      (Eigen::AngleAxisd(0.73, Eigen::Vector3d(1.0, -2.0, 0.5).normalized()) *
       Eigen::AngleAxisd(-0.32, Eigen::Vector3d::UnitZ())).toRotationMatrix();
  value.segment<4>(0) = ov_core::rot_2_quat(global_to_imu);
  value.segment<3>(4) = Eigen::Vector3d(1.2, -0.4, 2.1);
  value.segment<3>(7) = Eigen::Vector3d(2.3, 0.7, -1.1);
  value.segment<3>(10) = Eigen::Vector3d(0.02, -0.03, 0.04);
  value.segment<3>(13) = Eigen::Vector3d(0.1, 0.04, -0.2);
  return value;
}

Pose pose_of(const ov_type::IMU &state, const Eigen::Vector3d &lever_arm,
             const Eigen::Matrix3d &camera_to_imu) {
  const Eigen::Matrix3d imu_to_global = state.Rot().transpose();
  return {state.pos() + imu_to_global * lever_arm, imu_to_global * camera_to_imu};
}

Vector6d twist_of(const ov_type::IMU &state, const Eigen::Vector3d &gyro) {
  Vector6d result;
  result.head<3>() = state.Rot() * state.vel();
  result.tail<3>() = gyro - state.bias_g();
  return result;
}

Eigen::Vector3d global_rotation_delta(const Eigen::Matrix3d &a, const Eigen::Matrix3d &b) {
  const Eigen::AngleAxisd difference(a * b.transpose());
  return difference.axis() * difference.angle();
}

// Generate a strictly positive definite full state covariance with nonzero
// cross-correlations; this detects incorrect extraction or ordering of blocks.
Matrix15d covariance_fixture() {
  Matrix15d A;
  for (int row = 0; row < 15; ++row)
    for (int col = 0; col < 15; ++col)
      A(row, col) = std::sin(0.37 * (row + 1) * (col + 2)) / (1.0 + row + col);
  return A * A.transpose() + 1e-5 * Matrix15d::Identity();
}

void expect_symmetric_psd(const Matrix6d &matrix) {
  EXPECT_LT((matrix - matrix.transpose()).cwiseAbs().maxCoeff(), 1e-12);
  Eigen::SelfAdjointEigenSolver<Matrix6d> solver(matrix);
  ASSERT_EQ(solver.info(), Eigen::Success);
  EXPECT_GE(solver.eigenvalues().minCoeff(), -1e-10);
}

TEST(SnapshotJacobians, ImuAndBothCameraPosesMatchTrueImuUpdate) {
  const auto nominal = nominal_value();
  ov_type::IMU imu;
  imu.set_value(nominal);
  const Eigen::Matrix3d imu_to_global = imu.Rot().transpose();

  // Three distinct body-to-sensor transforms exercise zero and nonzero arms,
  // asymmetric stereo geometry, and nonidentity camera optical rotations.
  const std::array<Eigen::Vector3d, 3> arms = {
      Eigen::Vector3d::Zero(), Eigen::Vector3d(0.07, -0.025, 0.011),
      Eigen::Vector3d(-0.031, 0.064, -0.018)};
  const std::array<Eigen::Matrix3d, 3> rotations = {
      Eigen::Matrix3d::Identity(),
      Eigen::AngleAxisd(0.42, Eigen::Vector3d(1.0, 0.5, -0.25).normalized()).toRotationMatrix(),
      Eigen::AngleAxisd(-0.66, Eigen::Vector3d(-0.2, 0.3, 1.0).normalized()).toRotationMatrix()};

  constexpr double step = 1e-6;
  for (size_t sensor = 0; sensor < arms.size(); ++sensor) {
    Matrix6x15d numeric;
    for (int i = 0; i < 15; ++i) {
      Vector15d dx = Vector15d::Zero();
      dx(i) = step;
      imu.set_value(nominal);
      imu.update(dx);
      const Pose plus = pose_of(imu, arms[sensor], rotations[sensor]);
      imu.set_value(nominal);
      imu.update(-dx);
      const Pose minus = pose_of(imu, arms[sensor], rotations[sensor]);
      numeric.block<3, 1>(0, i) = (plus.position - minus.position) / (2.0 * step);
      // ROS fixed-axis covariance represents the left/global perturbation.
      numeric.block<3, 1>(3, i) = global_rotation_delta(plus.rotation, minus.rotation) / (2.0 * step);
    }
    const Matrix6x15d analytic = openvins_slam::pose_jacobian(imu_to_global, arms[sensor]);
    EXPECT_LT((analytic - numeric).cwiseAbs().maxCoeff(), 1e-7) << "sensor " << sensor << "\nanalytic:\n"
                                                               << analytic << "\nnumeric:\n" << numeric;
    expect_symmetric_psd(analytic * covariance_fixture() * analytic.transpose());
  }
}

TEST(SnapshotJacobians, BodyTwistMatchesTrueImuUpdateIncludingGyroBias) {
  const auto nominal = nominal_value();
  ov_type::IMU imu;
  imu.set_value(nominal);
  const Eigen::Matrix3d global_to_imu = imu.Rot();
  const Eigen::Vector3d velocity_global = imu.vel();
  const Eigen::Vector3d corrected_gyro(0.31, -0.42, 0.18);

  constexpr double step = 1e-6;
  Matrix6x15d numeric;
  for (int i = 0; i < 15; ++i) {
    Vector15d dx = Vector15d::Zero();
    dx(i) = step;
    imu.set_value(nominal);
    imu.update(dx);
    const Vector6d plus = twist_of(imu, corrected_gyro);
    imu.set_value(nominal);
    imu.update(-dx);
    const Vector6d minus = twist_of(imu, corrected_gyro);
    numeric.col(i) = (plus - minus) / (2.0 * step);
  }
  const Matrix6x15d analytic = openvins_slam::twist_jacobian(global_to_imu, velocity_global);
  EXPECT_LT((analytic - numeric).cwiseAbs().maxCoeff(), 1e-7) << "analytic:\n" << analytic << "\nnumeric:\n" << numeric;
  Matrix6d covariance = analytic * covariance_fixture() * analytic.transpose();
  covariance.bottomRightCorner<3, 3>().diagonal().array() += 0.003;
  expect_symmetric_psd(covariance);
}

} // namespace
