from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
import os


launch_args = [
    DeclareLaunchArgument(
        name="config",
        default_value="euroc_mav",
        description="config folder under ov_msckf/config",
    ),
    DeclareLaunchArgument(
        name="config_path",
        default_value="",
        description="path to estimator_config.yaml. If not given, determined from config",
    ),
    DeclareLaunchArgument(
        name="approximate_sync",
        default_value="false",
        description="whether disparity_node should use approximate synchronization",
    ),
    DeclareLaunchArgument(
        name="stereo_algorithm",
        default_value="1",
        description="Stereo algorithm: Block Matching (0) or Semi-Global Block Matching (1)",
    ),
    DeclareLaunchArgument(
        name="disparity_range",
        default_value="32",
        description="number of disparities to search, must be a multiple of 16",
    ),
    DeclareLaunchArgument(
        name="sgbm_mode",
        default_value="2",
        description="OpenCV StereoSGBM mode: SGBM (0), HH (1), SGBM_3WAY (2), HH4 (3)",
    ),
]


def launch_setup(context):
    config_path = LaunchConfiguration("config_path").perform(context)
    if not config_path:
        configs_dir = os.path.join(get_package_share_directory("ov_msckf"), "config")
        config = LaunchConfiguration("config").perform(context)
        config_path = os.path.join(configs_dir, config, "estimator_config.yaml")

    if not os.path.isfile(config_path):
        return [
            LogInfo(
                msg="ERROR: config_path file: '{}' does not exist - not starting stereo image processing".format(
                    config_path
                )
            )
        ]

    camera_info = Node(
        package="ov_msckf",
        executable="run_stereo_camera_info",
        name="run_stereo_camera_info",
        output="screen",
        parameters=[
            {"config_path": config_path},
            {"left_image_topic": "/cam0/image_raw"},
            {"right_image_topic": "/cam1/image_raw"},
            {"left_info_topic": "/cam0/camera_info"},
            {"right_info_topic": "/cam1/camera_info"},
            {"left_camera": "cam0"},
            {"right_camera": "cam1"},
        ],
    )

    left_rectify = Node(
        package="image_proc",
        executable="rectify_node",
        name="left_rectify_node",
        namespace="/stereo/left",
        output="screen",
        remappings=[
            ("image", "/cam0/image_raw"),
            ("camera_info", "/cam0/camera_info"),
            ("image_rect", "image_rect"),
        ],
    )

    right_rectify = Node(
        package="image_proc",
        executable="rectify_node",
        name="right_rectify_node",
        namespace="/stereo/right",
        output="screen",
        remappings=[
            ("image", "/cam1/image_raw"),
            ("camera_info", "/cam1/camera_info"),
            ("image_rect", "image_rect"),
        ],
    )

    disparity = Node(
        package="stereo_image_proc",
        executable="disparity_node",
        name="disparity_node",
        namespace="/stereo",
        output="screen",
        parameters=[
            {"approximate_sync": ParameterValue(LaunchConfiguration("approximate_sync"), value_type=bool)},
            {"stereo_algorithm": ParameterValue(LaunchConfiguration("stereo_algorithm"), value_type=int)},
            {"disparity_range": ParameterValue(LaunchConfiguration("disparity_range"), value_type=int)},
            {"sgbm_mode": ParameterValue(LaunchConfiguration("sgbm_mode"), value_type=int)},
        ],
        remappings=[
            ("left/image_rect", "left/image_rect"),
            ("right/image_rect", "right/image_rect"),
            ("left/camera_info", "/cam0/camera_info"),
            ("right/camera_info", "/cam1/camera_info"),
        ],
    )

    return [camera_info, left_rectify, right_rectify, disparity]


def generate_launch_description():
    ld = LaunchDescription(launch_args)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
