#include "openvins_slam/snapshot.hpp"

#include <Eigen/Geometry>
#include <cmath>
#include <stdexcept>
#include "state/State.h"
#include "state/StateHelper.h"
#include "utils/quat_ops.h"

namespace openvins_slam {
namespace {

Eigen::Matrix3d skew(const Eigen::Vector3d &v) {
  Eigen::Matrix3d x;
  x << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
  return x;
}

void set_xyz(geometry_msgs::msg::Point &dst, const Eigen::Vector3d &v) {
  dst.x = v.x(); dst.y = v.y(); dst.z = v.z();
}
void set_xyz(geometry_msgs::msg::Vector3 &dst, const Eigen::Vector3d &v) {
  dst.x = v.x(); dst.y = v.y(); dst.z = v.z();
}
void set_quaternion(geometry_msgs::msg::Quaternion &dst, const Eigen::Matrix3d &R) {
  Eigen::Quaterniond q(R);
  q.normalize();
  dst.x = q.x(); dst.y = q.y(); dst.z = q.z(); dst.w = q.w();
}
template <typename Array>
void set_covariance(Array &dst, const Eigen::Matrix<double, 6, 6> &cov) {
  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < 6; ++j) dst[i * 6 + j] = cov(i, j);
}

} // namespace

builtin_interfaces::msg::Time ros_time(int64_t stamp_ns) {
  if (stamp_ns < 0) throw std::runtime_error("negative output timestamp");
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(stamp_ns / 1000000000LL);
  stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000LL);
  return stamp;
}

Matrix6x15d pose_jacobian(const Eigen::Matrix3d &S, const Eigen::Vector3d &r) {
  Matrix6x15d J = Matrix6x15d::Zero();
  J.block<3, 3>(0, 0) = -S * skew(r);
  J.block<3, 3>(0, 3).setIdentity();
  J.block<3, 3>(3, 0) = S;
  return J;
}

Matrix6x15d twist_jacobian(const Eigen::Matrix3d &R, const Eigen::Vector3d &v) {
  Matrix6x15d J = Matrix6x15d::Zero();
  J.block<3, 3>(0, 0) = skew(R * v);
  J.block<3, 3>(0, 6) = R;
  J.block<3, 3>(3, 9) = -Eigen::Matrix3d::Identity();
  return J;
}

Eigen::Matrix<double, 6, 6> extrinsic_pose_jacobian(const Eigen::Matrix3d &C, const Eigen::Vector3d &p) {
  Eigen::Matrix<double, 6, 6> J = Eigen::Matrix<double, 6, 6>::Zero();
  J.block<3, 3>(0, 0) = C * skew(p);
  J.block<3, 3>(0, 3) = -C;
  J.block<3, 3>(3, 0) = C;
  return J;
}

Snapshot make_snapshot(const std::shared_ptr<ov_msckf::State> &state, const Calibration &calibration,
                       int64_t stamp_ns, const Eigen::Vector3d &gyro, double gyro_noise_var) {
  Snapshot out;
  for (const auto &entry : state->_cam_intrinsics) {
    if (!entry.second->value().allFinite()) throw std::runtime_error("estimator produced non-finite camera intrinsics");
  }
  const auto stamp = ros_time(stamp_ns);
  const Eigen::Matrix3d R = state->_imu->Rot();
  const Eigen::Matrix3d S = R.transpose();
  const Eigen::Vector3d p = state->_imu->pos();
  const Eigen::Vector3d v = state->_imu->vel();
  const Matrix15d covariance = ov_msckf::StateHelper::get_marginal_covariance(state, {state->_imu});
  if (!state->_imu->value().allFinite() || !R.allFinite() || !p.allFinite() || !v.allFinite() || !gyro.allFinite() ||
      !std::isfinite(gyro_noise_var) || gyro_noise_var < 0 || !covariance.allFinite())
    throw std::runtime_error("estimator produced a non-finite state");
  for (int i = 0; i < 3; ++i) {
    Eigen::Vector3d r = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R_I_C = Eigen::Matrix3d::Identity();
    if (i > 0) {
      const auto extrinsic = state->_calib_IMUtoCAM.at(i - 1);
      if (!extrinsic->value().allFinite()) throw std::runtime_error("estimator produced non-finite camera extrinsics");
      R_I_C = extrinsic->Rot().transpose();
      r = -R_I_C * extrinsic->pos();
    }
    auto &pose = out.poses[i];
    pose.header.stamp = stamp;
    pose.header.frame_id = "odom";
    set_xyz(pose.pose.pose.position, p + S * r);
    set_quaternion(pose.pose.pose.orientation, S * R_I_C);
    const auto J = pose_jacobian(S, r);
    if (i > 0 && state->_options.do_calib_camera_pose) {
      const auto extrinsic = state->_calib_IMUtoCAM.at(i - 1);
      const auto joint = ov_msckf::StateHelper::get_marginal_covariance(state, {state->_imu, extrinsic});
      Eigen::Matrix<double, 6, 21> joint_J;
      joint_J << J, extrinsic_pose_jacobian(S * R_I_C, extrinsic->pos());
      const Eigen::Matrix<double, 6, 6> camera_covariance = joint_J * joint * joint_J.transpose();
      if (!camera_covariance.allFinite()) throw std::runtime_error("non-finite camera pose covariance");
      set_covariance(pose.pose.covariance, camera_covariance);
    } else {
      set_covariance(pose.pose.covariance, J * covariance * J.transpose());
    }
  }
  out.odom.header = out.poses[0].header;
  out.odom.child_frame_id = "imu";
  out.odom.pose = out.poses[0].pose;
  set_xyz(out.odom.twist.twist.linear, R * v);
  set_xyz(out.odom.twist.twist.angular, gyro - state->_imu->bias_g());
  const auto J = twist_jacobian(R, v);
  Eigen::Matrix<double, 6, 6> twist_cov = J * covariance * J.transpose();
  twist_cov.block<3, 3>(3, 3).diagonal().array() += gyro_noise_var;
  set_covariance(out.odom.twist.covariance, twist_cov);
  geometry_msgs::msg::TransformStamped tf;
  tf.header = out.poses[0].header;
  tf.child_frame_id = "imu";
  tf.transform.translation.x = out.poses[0].pose.pose.position.x;
  tf.transform.translation.y = out.poses[0].pose.pose.position.y;
  tf.transform.translation.z = out.poses[0].pose.pose.position.z;
  tf.transform.rotation = out.poses[0].pose.pose.orientation;
  out.tf.transforms.push_back(tf);
  if (state->_options.do_calib_camera_pose) {
    for (int i = 0; i < 2; ++i) {
      const auto extrinsic = state->_calib_IMUtoCAM.at(i);
      tf.header.frame_id = "imu";
      tf.child_frame_id = "cam" + std::to_string(i) + "_optical";
      const Eigen::Matrix3d camera_to_imu = extrinsic->Rot().transpose();
      set_xyz(tf.transform.translation, -camera_to_imu * extrinsic->pos());
      set_quaternion(tf.transform.rotation, camera_to_imu);
      out.tf.transforms.push_back(tf);
    }
  }
  return out;
}

tf2_msgs::msg::TFMessage make_static_tf(const Calibration &calibration, int64_t stamp_ns) {
  tf2_msgs::msg::TFMessage out;
  for (int i = 0; i < 2; ++i) {
    const auto &T = calibration.camera[i].T_cam_imu;
    const Eigen::Matrix3d R = T.block<3, 3>(0, 0).transpose();
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = ros_time(stamp_ns);
    tf.header.frame_id = "imu";
    tf.child_frame_id = "cam" + std::to_string(i) + "_optical";
    set_xyz(tf.transform.translation, -R * T.block<3, 1>(0, 3));
    set_quaternion(tf.transform.rotation, R);
    out.transforms.push_back(tf);
  }
  return out;
}

} // namespace openvins_slam
