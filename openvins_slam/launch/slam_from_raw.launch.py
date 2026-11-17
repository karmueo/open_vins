"""Launch raw-directory stereo-inertial VIO processing with a ROS 2 parameter file."""

from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.utilities import to_parameters_list


def _extract_parameters(context, config):
    raw_params = to_parameters_list(context, "slam_from_raw", "", [config])
    bag_params = to_parameters_list(context, "slam_from_bag", "", [config])
    raw_values = {p.name: p.value for p in raw_params}
    bag_values = {p.name: p.value for p in bag_params}
    for source in (raw_values, bag_values):
        for name in ("in_bag", "in_raw", "cameras", "outdir"):
            if name in source:
                raise ValueError(f"remove {name} from config YAML; pass it as a launch argument: {name}:=/path/to/...")
    return raw_values if raw_values else bag_values


def _make_node(context):
    if "params_file" in context.launch_configurations:
        raise ValueError("removed launch argument params_file; use config:=/path/to/config.yaml")
    requested_file = Path(LaunchConfiguration("config").perform(context)).expanduser()
    config = requested_file.resolve()
    if not config.is_file():
        raise FileNotFoundError(f"config is not a regular file: {config}")
    values = _extract_parameters(context, config)
    visualize = values.get("rviz2", False)
    if type(visualize) is not bool:
        raise ValueError("rviz2 must be true or false")
    realtime = values.get("realtime", False)
    if type(realtime) is not bool:
        raise ValueError("realtime must be true or false")

    raw_argument = LaunchConfiguration("in_raw", default="").perform(context)
    if not raw_argument.strip():
        raise ValueError("launch argument in_raw is required; pass in_raw:=/path/to/raw_dir")
    in_raw = Path(raw_argument).expanduser().absolute()
    if not in_raw.is_dir():
        raise FileNotFoundError(f"in_raw is not a directory: {in_raw}")

    cameras_argument = LaunchConfiguration("cameras", default="").perform(context)
    if cameras_argument.strip():
        cameras = Path(cameras_argument).expanduser().absolute()
    else:
        local_calib = in_raw / "calib.yaml"
        parent_calib = in_raw.parent / "calib.yaml"
        cameras = local_calib if local_calib.is_file() else (parent_calib if parent_calib.is_file() else local_calib)
    if not cameras.is_file():
        raise FileNotFoundError(f"cameras is not a regular file: {cameras}; pass cameras:=/path/to/calib.yaml")

    outdir_argument = LaunchConfiguration("outdir", default="").perform(context)
    if outdir_argument.strip():
        outdir = Path(outdir_argument).expanduser().absolute()
    else:
        outdir = Path.cwd() / "output" / f"{in_raw.name}_{datetime.now():%Y%m%d%H%M%S}"
        # Reserve this run's directory; never reuse a result from the same second.
        outdir.mkdir(parents=True, exist_ok=False)

    node_params = {
        name: (ParameterValue(value, value_type=str) if isinstance(value, str) else value)
        for name, value in values.items()
    }
    nodes = [Node(
        package="openvins_slam",
        executable="slam_from_raw",
        name="slam_from_raw",
        output="screen",
        # OpenVINS uses printf; a terminal keeps its logs line buffered under launch.
        emulate_tty=True,
        parameters=[node_params, {
            "config_name": requested_file.stem,
            "in_raw": ParameterValue(str(in_raw), value_type=str),
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
            description="Path to a ROS 2 YAML parameter file (supports slam_from_raw or slam_from_bag keys)",
        ),
        DeclareLaunchArgument("in_raw", description="Path to the input raw dataset directory (required)"),
        DeclareLaunchArgument(
            "cameras",
            default_value="",
            description="Calibration file; defaults to calib.yaml inside in_raw or its parent directory",
        ),
        DeclareLaunchArgument(
            "outdir",
            default_value="",
            description="Output directory; defaults to ./output/<in_raw_name>_<YYYYMMDDHHMMSS>",
        ),
        OpaqueFunction(function=_make_node),
    ])
