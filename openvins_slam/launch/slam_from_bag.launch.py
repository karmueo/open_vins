"""Launch offline MCAP processing with a standard ROS 2 parameter file."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.utilities import to_parameters_list


def _make_node(context):
    requested_file = Path(LaunchConfiguration("params_file").perform(context)).expanduser()
    params_file = requested_file.resolve()
    if not params_file.is_file():
        raise FileNotFoundError(f"params_file is not a regular file: {params_file}")
    values = {p.name: p.value for p in to_parameters_list(context, "slam_from_bag", "", [params_file])}
    visualize = values.get("rviz2", False)
    if type(visualize) is not bool:
        raise ValueError("rviz2 must be true or false")
    realtime = values.get("realtime", False)
    if type(realtime) is not bool:
        raise ValueError("realtime must be true or false")
    nodes = [Node(
        package="openvins_slam",
        executable="slam_from_bag",
        name="slam_from_bag",
        output="screen",
        # OpenVINS uses printf; a terminal keeps its logs line buffered under launch.
        emulate_tty=True,
        parameters=[str(params_file), {"config_name": requested_file.stem}],
    )]
    if visualize:
        nodes.append(Node(
            package="rviz2", executable="rviz2", name="openvins_rviz",
            arguments=["-d", str(Path(get_package_share_directory("openvins_slam")) / "launch" / "bag_slam.rviz")],
            parameters=[{"use_sim_time": True}], output="screen",
        ))
    return nodes


def generate_launch_description():
    default_params = Path(get_package_share_directory("openvins_slam")) / "config" / "slam_from_bag.yaml"
    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=str(default_params),
            description="Path to a standard ROS 2 YAML parameter file for slam_from_bag",
        ),
        OpaqueFunction(function=_make_node),
    ])
