"""Launch offline MCAP processing with a standard ROS 2 parameter file."""

from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.utilities import to_parameters_list


def _make_node(context):
    if "params_file" in context.launch_configurations:
        raise ValueError("removed launch argument params_file; use config:=/path/to/config.yaml")
    requested_file = Path(LaunchConfiguration("config").perform(context)).expanduser()
    config = requested_file.resolve()
    if not config.is_file():
        raise FileNotFoundError(f"config is not a regular file: {config}")
    values = {p.name: p.value for p in to_parameters_list(context, "slam_from_bag", "", [config])}
    for name in ("in_bag", "cameras", "outdir"):
        if name in values:
            raise ValueError(f"remove {name} from config YAML; pass it as a launch argument: {name}:=/path/to/...")
    visualize = values.get("rviz2", False)
    if type(visualize) is not bool:
        raise ValueError("rviz2 must be true or false")
    realtime = values.get("realtime", False)
    if type(realtime) is not bool:
        raise ValueError("realtime must be true or false")

    bag_argument = LaunchConfiguration("in_bag", default="").perform(context)
    if not bag_argument.strip():
        raise ValueError("launch argument in_bag is required; pass in_bag:=/path/to/input.mcap")
    in_bag = Path(bag_argument).expanduser().absolute()
    if not in_bag.is_file():
        raise FileNotFoundError(f"in_bag is not a regular file: {in_bag}")
    cameras_argument = LaunchConfiguration("cameras", default="").perform(context)
    cameras = (Path(cameras_argument).expanduser().absolute()
               if cameras_argument.strip() else in_bag.parent / "calib.yaml")
    if not cameras.is_file():
        raise FileNotFoundError(f"cameras is not a regular file: {cameras}; pass cameras:=/path/to/calib.yaml")
    outdir_argument = LaunchConfiguration("outdir", default="").perform(context)
    if outdir_argument.strip():
        outdir = Path(outdir_argument).expanduser().absolute()
    else:
        outdir = Path.cwd() / "output" / f"{in_bag.stem}_{datetime.now():%Y%m%d%H%M%S}"
        # Reserve this run's directory; never reuse a result from the same second.
        outdir.mkdir(parents=True, exist_ok=False)

    nodes = [Node(
        package="openvins_slam",
        executable="slam_from_bag",
        name="slam_from_bag",
        output="screen",
        # OpenVINS uses printf; a terminal keeps its logs line buffered under launch.
        emulate_tty=True,
        parameters=[str(config), {
            "config_name": requested_file.stem,
            "in_bag": ParameterValue(str(in_bag), value_type=str),
            "cameras": ParameterValue(str(cameras), value_type=str),
            "outdir": ParameterValue(str(outdir), value_type=str),
        }],
    )]
    if visualize:
        nodes.append(Node(
            package="rviz2", executable="rviz2", name="openvins_rviz",
            arguments=["-d", str(Path(get_package_share_directory("openvins_slam")) / "launch" / "bag_slam.rviz")],
            parameters=[{"use_sim_time": True}], output="screen",
        ))
    return nodes


def generate_launch_description():
    default_config = Path(get_package_share_directory("openvins_slam")) / "config" / "slam_from_bag.yaml"
    return LaunchDescription([
        DeclareLaunchArgument(
            "config",
            default_value=str(default_config),
            description="Path to a standard ROS 2 YAML parameter file for slam_from_bag",
        ),
        DeclareLaunchArgument("in_bag", description="Path to the input MCAP file (required)"),
        DeclareLaunchArgument("cameras", default_value="", description="Calibration file; defaults to calib.yaml beside in_bag"),
        DeclareLaunchArgument("outdir", default_value="", description="Output directory; defaults to ./output/<bag_stem>_<YYYYMMDDHHMMSS>"),
        OpaqueFunction(function=_make_node),
    ])
