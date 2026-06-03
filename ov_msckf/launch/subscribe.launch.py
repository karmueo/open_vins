from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
import os
import sys

launch_args = [
    DeclareLaunchArgument(name="namespace", default_value="ov_msckf", description="namespace"),
    DeclareLaunchArgument(
        name="ov_enable", default_value="true", description="enable OpenVINS node"
    ),
    DeclareLaunchArgument(
        name="rviz_enable", default_value="false", description="enable rviz node"
    ),
    DeclareLaunchArgument(
        name="gt_align_enable", default_value="true", description="publish live-aligned groundtruth path"
    ),
    DeclareLaunchArgument(
        name="config",
        default_value="euroc_mav",
        description="euroc_mav, tum_vi, rpng_aruco...",
    ),
    DeclareLaunchArgument(
        name="dataset",
        default_value="V1_01_easy",
        description="dataset name used to find groundtruth in ov_data/<config>/<dataset>.csv",
    ),
    DeclareLaunchArgument(
        name="path_gt",
        default_value="",
        description="path to ASL/EuRoC groundtruth CSV file. If empty, uses ov_data/<config>/<dataset>.csv when available",
    ),
    DeclareLaunchArgument(
        name="config_path",
        default_value="",
        description="path to estimator_config.yaml. If not given, determined based on provided 'config' above",
    ),
    DeclareLaunchArgument(
        name="verbosity",
        default_value="INFO",
        description="ALL, DEBUG, INFO, WARNING, ERROR, SILENT",
    ),
    DeclareLaunchArgument(
        name="use_stereo",
        default_value="true",
        description="if we have more than 1 camera, if we should try to track stereo constraints between pairs",
    ),
    DeclareLaunchArgument(
        name="max_cameras",
        default_value="2",
        description="how many cameras we have 1 = mono, 2 = stereo, >2 = binocular (all mono tracking)",
    ),
    DeclareLaunchArgument(
        name="save_total_state",
        default_value="false",
        description="record the total state with calibration and features to a txt file",
    ),
    DeclareLaunchArgument(
        name="filepath_est",
        default_value="/tmp/ov_estimate.txt",
        description="path to save the estimated total state when save_total_state is true",
    ),
    DeclareLaunchArgument(
        name="filepath_std",
        default_value="/tmp/ov_estimate_std.txt",
        description="path to save the estimated total state standard deviation when save_total_state is true",
    ),
    DeclareLaunchArgument(
        name="filepath_gt",
        default_value="/tmp/ov_groundtruth.txt",
        description="path to save groundtruth state when available",
    )
]

def launch_setup(context):
    config_path = LaunchConfiguration("config_path").perform(context)
    config = LaunchConfiguration("config").perform(context)
    if not config_path:
        configs_dir = os.path.join(get_package_share_directory("ov_msckf"), "config")
        available_configs = os.listdir(configs_dir)
        if config in available_configs:
            config_path = os.path.join(
                            get_package_share_directory("ov_msckf"),
                            "config",config,"estimator_config.yaml"
                        )
        else:
            return [
                LogInfo(
                    msg="ERROR: unknown config: '{}' - Available configs are: {} - not starting OpenVINS".format(
                        config, ", ".join(available_configs)
                    )
                )
            ]
    else:
        if not os.path.isfile(config_path):
            return [
                LogInfo(
                    msg="ERROR: config_path file: '{}' - does not exist. - not starting OpenVINS".format(
                        config_path)
                    )
            ]
    path_gt = LaunchConfiguration("path_gt").perform(context)
    path_gt_requested = bool(path_gt)
    if not path_gt:
        dataset = LaunchConfiguration("dataset").perform(context)
        try:
            path_gt = os.path.join(
                get_package_share_directory("ov_data"),
                config,
                dataset + ".csv",
            )
        except PackageNotFoundError:
            path_gt = ""
    if path_gt and not os.path.isfile(path_gt):
        if not path_gt_requested:
            path_gt = ""
        else:
            return [
                LogInfo(
                    msg="ERROR: path_gt file: '{}' - does not exist. - not starting OpenVINS".format(
                        path_gt)
                    )
            ]

    node1 = Node(
        package="ov_msckf",
        executable="run_subscribe_msckf",
        condition=IfCondition(LaunchConfiguration("ov_enable")),
        namespace=LaunchConfiguration("namespace"),
        output='screen',
        parameters=[
            {"verbosity": LaunchConfiguration("verbosity")},
            {"use_stereo": LaunchConfiguration("use_stereo")},
            {"max_cameras": LaunchConfiguration("max_cameras")},
            {"save_total_state": LaunchConfiguration("save_total_state")},
            {"filepath_est": LaunchConfiguration("filepath_est")},
            {"filepath_std": LaunchConfiguration("filepath_std")},
            {"filepath_gt": LaunchConfiguration("filepath_gt")},
            {"config_path": config_path},
        ],
    )

    node2 = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(LaunchConfiguration("rviz_enable")),
        arguments=[
            "-d"
            + os.path.join(
                get_package_share_directory("ov_msckf"), "launch", "display_ros2.rviz"
            ),
            "--ros-args",
            "--log-level",
            "warn",
            ],
    )

    nodes = [node1, node2]
    if path_gt:
        node3 = Node(
            package="ov_eval",
            executable="live_align_trajectory",
            condition=IfCondition(LaunchConfiguration("gt_align_enable")),
            output='screen',
            parameters=[
                {"verbosity": LaunchConfiguration("verbosity")},
                {"alignment_type": "posyaw"},
                {"path_gt": path_gt},
                {"topic_pathimu": "/ov_msckf/pathimu"},
                {"topic_pathgt": "/ov_msckf/pathgt"},
            ],
        )
        nodes.append(node3)

    return nodes


def generate_launch_description():
    opfunc = OpaqueFunction(function=launch_setup)
    ld = LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld
