/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include "alignment/AlignTrajectory.h"
#include "alignment/AlignUtils.h"
#include "utils/Loader.h"
#include "utils/colors.h"
#include "utils/print.h"
#include "utils/quat_ops.h"

class LiveAlignTrajectory : public rclcpp::Node {
public:
  LiveAlignTrajectory() : Node("live_align_trajectory") {

    // Verbosity setting
    std::string verbosity = declare_parameter<std::string>("verbosity", "INFO");
    ov_core::Printer::setPrintLevel(verbosity);

    // Load what type of alignment we should use
    alignment_type = declare_parameter<std::string>("alignment_type", "posyaw");

    // If we don't have it, or it is empty then error
    std::string path_to_gt = declare_parameter<std::string>("path_gt", "");
    if (path_to_gt.empty()) {
      RCLCPP_ERROR(get_logger(), "[LOAD]: Please provide a groundtruth file path");
      throw std::runtime_error("missing path_gt parameter");
    }

    // Load our groundtruth from file
    boost::filesystem::path infolder(path_to_gt);
    std::vector<Eigen::Matrix3d> cov_ori_temp, cov_pos_temp;
    if (infolder.extension() == ".csv") {
      ov_eval::Loader::load_data_csv(path_to_gt, times_gt, poses_gt, cov_ori_temp, cov_pos_temp);
    } else {
      ov_eval::Loader::load_data(path_to_gt, times_gt, poses_gt, cov_ori_temp, cov_pos_temp);
    }

    // Our subscribe and publish nodes
    std::string topic_pathimu = declare_parameter<std::string>("topic_pathimu", "/ov_msckf/pathimu");
    std::string topic_pathgt = declare_parameter<std::string>("topic_pathgt", "/ov_msckf/pathgt");
    sub_path = create_subscription<nav_msgs::msg::Path>(topic_pathimu, 1, std::bind(&LiveAlignTrajectory::align_and_publish, this, std::placeholders::_1));
    pub_path = create_publisher<nav_msgs::msg::Path>(topic_pathgt, 2);
    RCLCPP_INFO(get_logger(), "Subscribing: %s", topic_pathimu.c_str());
    RCLCPP_INFO(get_logger(), "Publishing: %s", topic_pathgt.c_str());
  }

private:
  void align_and_publish(const nav_msgs::msg::Path::SharedPtr msg) {

    // Convert the message into correct vector format
    // tx ty tz qx qy qz qw
    std::vector<double> times_temp;
    std::vector<Eigen::Matrix<double, 7, 1>> poses_temp;
    for (auto const &pose : msg->poses) {
      times_temp.push_back(static_cast<double>(pose.header.stamp.sec) + 1e-9 * static_cast<double>(pose.header.stamp.nanosec));
      Eigen::Matrix<double, 7, 1> pose_tmp;
      pose_tmp << pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, pose.pose.orientation.x, pose.pose.orientation.y,
          pose.pose.orientation.z, pose.pose.orientation.w;
      poses_temp.push_back(pose_tmp);
    }

    // Intersect timestamps
    std::vector<double> gt_times_temp = times_gt;
    std::vector<Eigen::Matrix<double, 7, 1>> gt_poses_temp = poses_gt;
    ov_eval::AlignUtils::perform_association(0, 0.02, times_temp, gt_times_temp, poses_temp, gt_poses_temp);

    // Return failure if we didn't have any common timestamps
    if (poses_temp.size() < 3) {
      PRINT_ERROR(RED "[TRAJ]: unable to get enough common timestamps between trajectories.\n" RESET);
      PRINT_ERROR(RED "[TRAJ]: does the estimated trajectory publish the rosbag timestamps??\n" RESET);
      return;
    }

    // Perform alignment of the trajectories
    Eigen::Matrix3d R_ESTtoGT;
    Eigen::Vector3d t_ESTinGT;
    double s_ESTtoGT;
    ov_eval::AlignTrajectory::align_trajectory(poses_temp, gt_poses_temp, R_ESTtoGT, t_ESTinGT, s_ESTtoGT, alignment_type);
    Eigen::Vector4d q_ESTtoGT = ov_core::rot_2_quat(R_ESTtoGT);
    PRINT_DEBUG("[TRAJ]: q_ESTtoGT = %.3f, %.3f, %.3f, %.3f | p_ESTinGT = %.3f, %.3f, %.3f | s = %.2f\n", q_ESTtoGT(0),
                q_ESTtoGT(1), q_ESTtoGT(2), q_ESTtoGT(3), t_ESTinGT(0), t_ESTinGT(1), t_ESTinGT(2), s_ESTtoGT);

    // Finally lets calculate the aligned trajectories
    // NOTE: We downsample the number of poses as needed to prevent rviz crashes.
    nav_msgs::msg::Path arr_groundtruth;
    arr_groundtruth.header = msg->header;
    for (size_t i = 0; i < gt_times_temp.size(); i += std::floor(static_cast<double>(gt_times_temp.size()) / 16384.0) + 1) {

      // Convert into the correct frame
      double timestamp = gt_times_temp.at(i);
      Eigen::Matrix<double, 7, 1> pose_inGT = gt_poses_temp.at(i);
      Eigen::Vector3d pos_IinEST = R_ESTtoGT.transpose() * (pose_inGT.block(0, 0, 3, 1) - t_ESTinGT) / s_ESTtoGT;
      Eigen::Vector4d quat_ESTtoI = ov_core::quat_multiply(pose_inGT.block(3, 0, 4, 1), q_ESTtoGT);

      // Finally push back
      geometry_msgs::msg::PoseStamped posetemp;
      posetemp.header = msg->header;
      posetemp.header.stamp.sec = static_cast<int32_t>(std::floor(timestamp));
      posetemp.header.stamp.nanosec = static_cast<uint32_t>(1e9 * (timestamp - std::floor(timestamp)));
      posetemp.pose.orientation.x = quat_ESTtoI(0);
      posetemp.pose.orientation.y = quat_ESTtoI(1);
      posetemp.pose.orientation.z = quat_ESTtoI(2);
      posetemp.pose.orientation.w = quat_ESTtoI(3);
      posetemp.pose.position.x = pos_IinEST(0);
      posetemp.pose.position.y = pos_IinEST(1);
      posetemp.pose.position.z = pos_IinEST(2);
      arr_groundtruth.poses.push_back(posetemp);
    }
    pub_path->publish(arr_groundtruth);
  }

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path;
  std::vector<double> times_gt;
  std::vector<Eigen::Matrix<double, 7, 1>> poses_gt;
  std::string alignment_type;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LiveAlignTrajectory>());
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
