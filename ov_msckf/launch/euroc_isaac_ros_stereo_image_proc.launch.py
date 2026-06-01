from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import PackageNotFoundError, get_package_prefix, get_package_share_directory
from ament_index_python.resources import get_resource
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
        name="backend",
        default_value="CUDA",
        description="Isaac ROS stereo VPI backend: CUDA or JETSON",
    ),
    DeclareLaunchArgument(
        name="max_disparity",
        default_value="128.0",
        description="maximum disparity for Isaac ROS SGM",
    ),
    DeclareLaunchArgument(
        name="confidence_threshold",
        default_value="60000",
        description="confidence threshold for Isaac ROS SGM",
    ),
    DeclareLaunchArgument(
        name="use_pointcloud",
        default_value="true",
        description="whether to launch Isaac ROS PointCloudNode",
    ),
    DeclareLaunchArgument(
        name="use_color",
        default_value="false",
        description="whether PointCloudNode should colorize the output cloud",
    ),
    DeclareLaunchArgument(
        name="image_width",
        default_value="752",
        description="EuRoC image width",
    ),
    DeclareLaunchArgument(
        name="image_height",
        default_value="480",
        description="EuRoC image height",
    ),
]


def _as_bool(value):
    return value.lower() in ("1", "true", "yes", "on")


def _missing_isaac_ros_resources():
    missing = []
    for package_name in ("isaac_ros_image_proc", "isaac_ros_stereo_image_proc"):
        try:
            get_package_prefix(package_name)
        except PackageNotFoundError:
            missing.append("{} package".format(package_name))
            continue
        try:
            get_resource("rclcpp_components", package_name)
        except LookupError:
            missing.append("{} rclcpp_components resource".format(package_name))
    return missing


def launch_setup(context):
    config_path = LaunchConfiguration("config_path").perform(context)
    if not config_path:
        configs_dir = os.path.join(get_package_share_directory("ov_msckf"), "config")
        config = LaunchConfiguration("config").perform(context)
        config_path = os.path.join(configs_dir, config, "estimator_config.yaml")

    if not os.path.isfile(config_path):
        return [
            LogInfo(
                msg="ERROR: config_path file: '{}' does not exist - not starting Isaac ROS stereo image processing".format(
                    config_path
                )
            )
        ]

    missing_isaac_ros_resources = _missing_isaac_ros_resources()
    if missing_isaac_ros_resources:
        return [
            LogInfo(
                msg=(
                    "ERROR: Isaac ROS stereo launch requires built and sourced Isaac ROS packages, "
                    "but missing: {}. Build with: "
                    "colcon build --packages-up-to ov_msckf isaac_ros_image_proc isaac_ros_stereo_image_proc "
                    "--cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF; "
                    "then run: source install/setup.bash"
                ).format(", ".join(missing_isaac_ros_resources))
            )
        ]

    image_width = ParameterValue(LaunchConfiguration("image_width"), value_type=int)
    image_height = ParameterValue(LaunchConfiguration("image_height"), value_type=int)

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

    composable_nodes = [
        ComposableNode(
            package="isaac_ros_image_proc",
            plugin="nvidia::isaac_ros::image_proc::ImageFormatConverterNode",
            name="left_mono_to_bgr8",
            namespace="/stereo/left",
            parameters=[
                {
                    "encoding_desired": "bgr8",
                    "image_width": image_width,
                    "image_height": image_height,
                }
            ],
            remappings=[
                ("image_raw", "/cam0/image_raw"),
                ("image", "image_bgr8"),
            ],
        ),
        ComposableNode(
            package="isaac_ros_image_proc",
            plugin="nvidia::isaac_ros::image_proc::ImageFormatConverterNode",
            name="right_mono_to_bgr8",
            namespace="/stereo/right",
            parameters=[
                {
                    "encoding_desired": "bgr8",
                    "image_width": image_width,
                    "image_height": image_height,
                }
            ],
            remappings=[
                ("image_raw", "/cam1/image_raw"),
                ("image", "image_bgr8"),
            ],
        ),
        ComposableNode(
            package="isaac_ros_image_proc",
            plugin="nvidia::isaac_ros::image_proc::RectifyNode",
            name="left_rectify",
            namespace="/stereo/left",
            parameters=[
                {
                    "output_width": image_width,
                    "output_height": image_height,
                }
            ],
            remappings=[
                ("image_raw", "image_bgr8"),
                ("camera_info", "/cam0/camera_info"),
                ("image_rect", "image_rect"),
                ("camera_info_rect", "camera_info_rect"),
            ],
        ),
        ComposableNode(
            package="isaac_ros_image_proc",
            plugin="nvidia::isaac_ros::image_proc::RectifyNode",
            name="right_rectify",
            namespace="/stereo/right",
            parameters=[
                {
                    "output_width": image_width,
                    "output_height": image_height,
                }
            ],
            remappings=[
                ("image_raw", "image_bgr8"),
                ("camera_info", "/cam1/camera_info"),
                ("image_rect", "image_rect"),
                ("camera_info_rect", "camera_info_rect"),
            ],
        ),
        ComposableNode(
            package="isaac_ros_stereo_image_proc",
            plugin="nvidia::isaac_ros::stereo_image_proc::DisparityNode",
            name="disparity",
            namespace="/stereo",
            parameters=[
                {
                    "backend": LaunchConfiguration("backend"),
                    "max_disparity": ParameterValue(LaunchConfiguration("max_disparity"), value_type=float),
                    "confidence_threshold": ParameterValue(LaunchConfiguration("confidence_threshold"), value_type=int),
                }
            ],
            remappings=[
                ("left/image_rect", "left/image_rect"),
                ("left/camera_info", "left/camera_info_rect"),
                ("right/image_rect", "right/image_rect"),
                ("right/camera_info", "right/camera_info_rect"),
                ("disparity", "disparity"),
            ],
        ),
    ]

    if _as_bool(LaunchConfiguration("use_pointcloud").perform(context)):
        composable_nodes.append(
            ComposableNode(
                package="isaac_ros_stereo_image_proc",
                plugin="nvidia::isaac_ros::stereo_image_proc::PointCloudNode",
                name="point_cloud",
                namespace="/stereo",
                parameters=[
                    {
                        "use_color": ParameterValue(LaunchConfiguration("use_color"), value_type=bool),
                        "unit_scaling": 1.0,
                        "output_width": image_width,
                        "output_height": image_height,
                    }
                ],
                remappings=[
                    ("left/image_rect_color", "left/image_rect"),
                    ("left/camera_info", "left/camera_info_rect"),
                    ("right/camera_info", "right/camera_info_rect"),
                    ("disparity", "disparity"),
                    ("points2", "points2"),
                ],
            )
        )

    container = ComposableNodeContainer(
        package="rclcpp_components",
        executable="component_container_mt",
        name="euroc_isaac_ros_stereo_image_proc_container",
        namespace="",
        composable_node_descriptions=composable_nodes,
        output="screen",
    )

    return [camera_info, container]


def generate_launch_description():
    ld = LaunchDescription(launch_args)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
