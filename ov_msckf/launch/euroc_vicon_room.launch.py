"""一键启动 OpenVINS EuRoC Vicon Room 回放、VIO、真值对齐和 RViz2 可视化。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context):
    """根据 launch 参数创建 OpenVINS 运行链路和可选 rosbag2 回放动作。"""
    # 数据集名称，用于拼接 bag 路径和选择项目内真值文件。
    dataset = LaunchConfiguration("dataset").perform(context)
    # ROS2 bag 根目录，默认指向本机已转换的 EuRoC Vicon Room 数据集。
    bag_root = LaunchConfiguration("bag_root").perform(context)
    # ROS2 bag 完整路径。
    bag_path = os.path.join(bag_root, dataset)
    # 是否让 rosbag2 发布 /clock。
    use_clock = LaunchConfiguration("use_clock").perform(context).lower() == "true"
    # bag 回放启动延迟，给 VIO/RViz 节点留出订阅初始化时间。
    bag_start_delay = float(LaunchConfiguration("bag_start_delay").perform(context))
    # ov_msckf 包共享目录。
    package_share = get_package_share_directory("ov_msckf")
    # 被复用的通用订阅 launch 文件路径。
    subscribe_launch_path = os.path.join(package_share, "launch", "subscribe.launch.py")

    if not os.path.isdir(bag_path):
        return [LogInfo(msg=f"ERROR: bag path does not exist: {bag_path} - not starting EuRoC Vicon Room launch")]

    # 传给 subscribe.launch.py 的 OpenVINS/RViz/真值对齐参数。
    subscribe_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(subscribe_launch_path),
        launch_arguments={
            "namespace": LaunchConfiguration("namespace"),
            "ov_enable": LaunchConfiguration("ov_enable"),
            "rviz_enable": LaunchConfiguration("rviz_enable"),
            "gt_align_enable": LaunchConfiguration("gt_align_enable"),
            "config": LaunchConfiguration("config"),
            "dataset": LaunchConfiguration("dataset"),
            "path_gt": LaunchConfiguration("path_gt"),
            "config_path": LaunchConfiguration("config_path"),
            "verbosity": LaunchConfiguration("verbosity"),
            "use_stereo": LaunchConfiguration("use_stereo"),
            "max_cameras": LaunchConfiguration("max_cameras"),
            "save_total_state": LaunchConfiguration("save_total_state"),
            "filepath_est": LaunchConfiguration("filepath_est"),
            "filepath_std": LaunchConfiguration("filepath_std"),
            "filepath_gt": LaunchConfiguration("filepath_gt"),
        }.items(),
    )

    # rosbag2 回放命令，仅播放 OpenVINS 需要的双目图像和 IMU 话题。
    bag_play_cmd = [
        "ros2",
        "bag",
        "play",
        bag_path,
        "--topics",
        "/cam0/image_raw",
        "/cam1/image_raw",
        "/imu0",
    ]
    if use_clock:
        bag_play_cmd.append("--clock")

    # 延迟启动 bag，降低启动瞬间丢首帧导致初始化不稳定的概率。
    bag_play = TimerAction(
        period=bag_start_delay,
        actions=[
            ExecuteProcess(
                cmd=bag_play_cmd,
                output="screen",
                condition=IfCondition(LaunchConfiguration("play_bag")),
            )
        ],
    )

    return [subscribe_launch, bag_play]


def generate_launch_description():
    """生成 EuRoC Vicon Room 一键运行 launch 描述。"""
    return LaunchDescription(
        [
            DeclareLaunchArgument(name="dataset", default_value="V1_01_easy", description="EuRoC Vicon Room dataset name"),
            DeclareLaunchArgument(name="bag_root", default_value="/mnt/data/slam/vicon_room1_ros2", description="ROS2 bag root directory"),
            DeclareLaunchArgument(name="play_bag", default_value="true", description="play the selected ROS2 bag"),
            DeclareLaunchArgument(
                name="bag_start_delay", default_value="2.0", description="seconds to wait before rosbag2 playback starts"
            ),
            DeclareLaunchArgument(name="use_clock", default_value="false", description="publish /clock from rosbag2 playback"),
            DeclareLaunchArgument(name="namespace", default_value="ov_msckf", description="OpenVINS namespace"),
            DeclareLaunchArgument(name="ov_enable", default_value="true", description="enable OpenVINS node"),
            DeclareLaunchArgument(name="rviz_enable", default_value="true", description="enable RViz2 node"),
            DeclareLaunchArgument(name="gt_align_enable", default_value="true", description="publish live-aligned groundtruth path"),
            DeclareLaunchArgument(name="config", default_value="euroc_mav", description="OpenVINS config directory name"),
            DeclareLaunchArgument(name="path_gt", default_value="", description="optional path to ASL/EuRoC groundtruth CSV file"),
            DeclareLaunchArgument(name="config_path", default_value="", description="optional path to estimator_config.yaml"),
            DeclareLaunchArgument(name="verbosity", default_value="INFO", description="ALL, DEBUG, INFO, WARNING, ERROR, SILENT"),
            DeclareLaunchArgument(name="use_stereo", default_value="true", description="enable stereo feature constraints"),
            DeclareLaunchArgument(name="max_cameras", default_value="2", description="number of cameras"),
            DeclareLaunchArgument(name="save_total_state", default_value="false", description="save estimated total state to text files"),
            DeclareLaunchArgument(
                name="filepath_est", default_value="/tmp/ov_estimate.txt", description="estimated total state output path"
            ),
            DeclareLaunchArgument(
                name="filepath_std",
                default_value="/tmp/ov_estimate_std.txt",
                description="estimated standard deviation output path",
            ),
            DeclareLaunchArgument(name="filepath_gt", default_value="/tmp/ov_groundtruth.txt", description="groundtruth state output path"),
            OpaqueFunction(function=launch_setup),
        ]
    )
