#include "openvins_slam/snapshot.hpp"

#include <array>
#include <cmath>

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include "types/IMU.h"
#include "utils/quat_ops.h"
#include "state/State.h"
#include "state/StateHelper.h"

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

TEST(SnapshotJacobians, OnlineExtrinsicsJointCovarianceAndDynamicTransforms) {
  ov_msckf::StateOptions options;
  options.num_cameras = 2;
  options.do_calib_camera_pose = true;
  options.do_calib_camera_intrinsics = false;
  options.do_calib_camera_timeoffset = false;
  auto state = std::make_shared<ov_msckf::State>(options);
  state->_imu->set_value(nominal_value());
  openvins_slam::Calibration calibration;
  for (int camera = 0; camera < 2; ++camera) {
    auto ext = state->_calib_IMUtoCAM.at(camera);
    Eigen::Matrix<double, 7, 1> nominal;
    nominal.head<4>() = ov_core::rot_2_quat(Eigen::AngleAxisd(0.42 + 0.1 * camera,
        Eigen::Vector3d(1, -2, 3).normalized()).toRotationMatrix());
    nominal.tail<3>() = Eigen::Vector3d(0.1, -0.03, 0.02 + 0.05 * camera);
    ext->set_value(nominal);
    // Deliberately different from the online state, to catch stale calibration use.
    calibration.camera[camera].T_cam_imu.setIdentity();
    const auto imu_nominal = state->_imu->value();
    auto pose = [&] {
      return pose_of(*state->_imu, -ext->Rot().transpose() * ext->pos(), ext->Rot().transpose());
    };
    Eigen::Matrix<double, 6, 21> numeric;
    constexpr double step = 1e-6;
    for (int k = 0; k < 21; ++k) {
      Eigen::Matrix<double, 21, 1> dx = Eigen::Matrix<double, 21, 1>::Zero();
      dx(k) = step;
      state->_imu->set_value(imu_nominal);
      ext->set_value(nominal);
      state->_imu->update(dx.head<15>());
      ext->update(dx.tail<6>());
      const auto plus = pose();
      state->_imu->set_value(imu_nominal);
      ext->set_value(nominal);
      state->_imu->update(-dx.head<15>());
      ext->update(-dx.tail<6>());
      const auto minus = pose();
      numeric.block<3, 1>(0, k) = (plus.position - minus.position) / (2 * step);
      numeric.block<3, 1>(3, k) = global_rotation_delta(plus.rotation, minus.rotation) / (2 * step);
    }
    state->_imu->set_value(imu_nominal);
    ext->set_value(nominal);
    Eigen::Matrix<double, 6, 21> analytic;
    const auto S = state->_imu->Rot().transpose().eval();
    analytic << openvins_slam::pose_jacobian(S, -ext->Rot().transpose() * ext->pos()),
        openvins_slam::extrinsic_pose_jacobian(S * ext->Rot().transpose(), ext->pos());
    EXPECT_LT((analytic - numeric).cwiseAbs().maxCoeff(), 1e-7);
    Eigen::Matrix<double, 21, 21> A;
    for (int r = 0; r < 21; ++r)
      for (int c = 0; c < 21; ++c) A(r, c) = std::sin(0.31 * (r + 1) * (c + 2)) / (1 + r + c);
    const Eigen::Matrix<double, 21, 21> joint = A * A.transpose() + 1e-5 * Eigen::Matrix<double, 21, 21>::Identity();
    ov_msckf::StateHelper::set_initial_covariance(state, joint, {state->_imu, ext});
    const auto snapshot = openvins_slam::make_snapshot(state, calibration, 1234567890, Eigen::Vector3d::Zero(), 0.001);
    const auto expected_pose = pose();
    const auto &actual = snapshot.poses[camera + 1].pose;
    EXPECT_NEAR(actual.pose.position.x, expected_pose.position.x(), 1e-12);
    EXPECT_NEAR(actual.pose.position.y, expected_pose.position.y(), 1e-12);
    EXPECT_NEAR(actual.pose.position.z, expected_pose.position.z(), 1e-12);
    Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> actual_cov(actual.covariance.data());
    const Matrix6d expected_cov = analytic * joint * analytic.transpose();
    EXPECT_LT((actual_cov - expected_cov).cwiseAbs().maxCoeff(), 1e-12);
    expect_symmetric_psd(actual_cov);
    ASSERT_EQ(snapshot.tf.transforms.size(), 3u);
    const auto &tf = snapshot.tf.transforms[camera + 1];
    EXPECT_EQ(tf.header.frame_id, "imu");
    EXPECT_EQ(tf.child_frame_id, "cam" + std::to_string(camera) + "_optical");
    EXPECT_NEAR(tf.transform.translation.x, (-ext->Rot().transpose() * ext->pos()).x(), 1e-12);
  }
}

} // namespace
