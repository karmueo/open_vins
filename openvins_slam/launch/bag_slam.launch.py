"""Run the MCAP VIO player and open the matching RViz configuration."""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _make_vio_node(context):
    bool_parameter = lambda name: ParameterValue(LaunchConfiguration(name), value_type=bool)
    string_parameter = lambda name: ParameterValue(LaunchConfiguration(name), value_type=str)
    parameters = {
        "cameras": string_parameter("cameras"),
        "in_bag": string_parameter("in_bag"),
        "outbag": string_parameter("outbag"),
        "realtime": bool_parameter("realtime"),
        "half_resolution": bool_parameter("half_resolution"),
        "apply_imu_bias": bool_parameter("apply_imu_bias"),
        "record_debug": bool_parameter("record_debug"),
        "save_trajectory": bool_parameter("save_trajectory"),
        "trajectory_file": string_parameter("trajectory_file"),
        "stereo_topic": string_parameter("stereo_topic"),
        "imu_topic": string_parameter("imu_topic"),
        "use_sim_time": bool_parameter("realtime"),
    }
    offset = LaunchConfiguration("imu_time_offset_sec").perform(context).strip()
    if offset:
        try:
            offset_value = float(offset)
        except ValueError as exc:
            raise ValueError("imu_time_offset_sec must be a finite number of seconds") from exc
        if not math.isfinite(offset_value):
            raise ValueError("imu_time_offset_sec must be a finite number of seconds")
        parameters["imu_time_offset_sec"] = offset_value
    return [Node(
        package="openvins_slam",
        executable="slam_from_bag",
        name="slam_from_bag",
        output="screen",
        parameters=[parameters],
    )]


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("cameras", default_value="", description="Path to calib.yaml; defaults to calib.yaml beside the input MCAP"),
        DeclareLaunchArgument("in_bag", description="Path to the input MCAP file"),
        DeclareLaunchArgument("outbag", default_value="", description="Optional output MCAP path"),
        DeclareLaunchArgument("realtime", default_value="true", description="Replay at the recorded rate"),
        DeclareLaunchArgument("half_resolution", default_value="true", description="Track at half camera resolution"),
        DeclareLaunchArgument("apply_imu_bias", default_value="true", description="Subtract calibrated IMU bias"),
        DeclareLaunchArgument("record_debug", default_value="false", description="Store tracking image and points"),
        DeclareLaunchArgument("save_trajectory", default_value="false", description="Save IMU trajectory in TUM format"),
        DeclareLaunchArgument("trajectory_file", default_value="", description="Optional TUM path; defaults beside input MCAP"),
        DeclareLaunchArgument("stereo_topic", default_value="/stereo/image_raw", description="Compressed stereo input topic"),
        DeclareLaunchArgument("imu_topic", default_value="/imu", description="IMU input topic"),
        DeclareLaunchArgument("imu_time_offset_sec", default_value="", description="Optional IMU timestamp offset; defaults to calib.yaml value or 0.0"),
        DeclareLaunchArgument("rviz_enable", default_value="true", description="Open RViz2"),
    ]
    bool_parameter = lambda name: ParameterValue(LaunchConfiguration(name), value_type=bool)
    config = os.path.join(get_package_share_directory("openvins_slam"), "launch", "bag_slam.rviz")

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="openvins_rviz",
        condition=IfCondition(LaunchConfiguration("rviz_enable")),
        arguments=["-d", config],
        parameters=[{"use_sim_time": bool_parameter("realtime")}],
        output="screen",
    )
    return LaunchDescription(arguments + [OpaqueFunction(function=_make_vio_node), rviz])
