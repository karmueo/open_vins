"""使用仿真时钟回放 MCAP，并提供 RViz 图像映射及服务控制提示。"""

import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _parse_bool(context, name):
    """读取布尔启动参数，非法值抛出 ValueError。"""
    raw = LaunchConfiguration(name, default="false").perform(context).strip().lower()
    if raw not in ("true", "false"):
        raise ValueError(f"{name} must be true or false, got: {raw!r}")
    return raw == "true"


def _parse_positive_float(context, name, allow_zero=False):
    """读取有限正数参数，按 allow_zero 决定是否接受零。"""
    raw = LaunchConfiguration(name, default="").perform(context).strip()
    try:
        value = float(raw)
    except ValueError as exc:
        raise ValueError(f"{name} must be a finite number, got: {raw!r}") from exc
    if not math.isfinite(value) or (value < 0 if allow_zero else value <= 0):
        comparator = ">= 0" if allow_zero else "> 0"
        raise ValueError(f"{name} must be finite and {comparator}, got: {raw!r}")
    return value


def _make_actions(context):
    """校验输入路径和参数，构造显示、播放器及控制提示动作。"""
    bag_argument = LaunchConfiguration("in_bag", default="").perform(context)
    if not bag_argument.strip():
        raise ValueError("launch argument in_bag is required; pass in_bag:=/path/to/result.mcap")
    in_bag = Path(bag_argument).expanduser().absolute()
    if not in_bag.is_file():
        raise FileNotFoundError(f"in_bag is not a regular file: {in_bag}")

    rviz_argument = LaunchConfiguration("rviz_config", default="").perform(context)
    if rviz_argument.strip():
        rviz_config = Path(rviz_argument).expanduser().absolute()
    else:
        rviz_config = Path(get_package_share_directory("openvins_slam")) / "launch" / "replay_mcap.rviz"
    if not rviz_config.is_file():
        raise FileNotFoundError(f"rviz_config is not a regular file: {rviz_config}")

    rate = _parse_positive_float(context, "rate", allow_zero=False)
    delay_sec = _parse_positive_float(context, "delay_sec", allow_zero=True)
    loop = _parse_bool(context, "loop")
    start_paused = _parse_bool(context, "start_paused")
    stereo_topic = LaunchConfiguration("stereo_topic", default="/stereo/image_raw").perform(context).strip()
    if not stereo_topic or not stereo_topic.startswith("/"):
        raise ValueError(f"stereo_topic must be an absolute ROS topic name, got: {stereo_topic!r}")

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="openvins_rviz",
        arguments=["-d", str(rviz_config)],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    path_helper = Node(
        package="openvins_slam",
        executable="replay_path_node.py",
        name="openvins_replay_path",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # 固定播放器名称，使恢复和暂停服务路径稳定；launch 子进程通过服务控制。
    play_cmd = [
        "ros2", "bag", "play", "--input", str(in_bag), "mcap", "--clock", "--rate", str(rate),
        "--disable-keyboard-controls", "--remap", "__node:=openvins_replay_player",
    ]
    # 与默认 RViz 的视频订阅保持一致，也兼容已经带 /compressed 后缀的输入。
    if stereo_topic != "/stereo/image_raw/compressed":
        play_cmd.append(f"{stereo_topic}:=/stereo/image_raw/compressed")
    if loop:
        play_cmd.append("--loop")
    if start_paused:
        play_cmd.append("--start-paused")

    bag_play = ExecuteProcess(cmd=play_cmd, output="screen", emulate_tty=True)
    playback_action = TimerAction(period=delay_sec, actions=[bag_play]) if delay_sec > 0 else bag_play

    return [
        rviz,
        path_helper,
        playback_action,
        LogInfo(msg=(
            "Playback control (run in another ROS terminal):\n"
            "Resume:\n"
            "ros2 service call /openvins_replay_player/resume rosbag2_interfaces/srv/Resume '{}'\n"
            "Pause:\n"
            "ros2 service call /openvins_replay_player/pause rosbag2_interfaces/srv/Pause '{}'"
        )),
        RegisterEventHandler(OnProcessExit(target_action=rviz, on_exit=[Shutdown(reason="RViz2 closed")])),
    ]


def generate_launch_description():
    """声明 MCAP 回放参数并返回启动描述。"""
    return LaunchDescription([
        DeclareLaunchArgument("in_bag", description="Path to the generated OpenVINS MCAP file (required)"),
        DeclareLaunchArgument("rate", default_value="1.0", description="Playback speed multiplier (> 0)"),
        DeclareLaunchArgument("loop", default_value="false", description="Loop bag playback (true/false)"),
        DeclareLaunchArgument(
            "start_paused", default_value="false",
            description="Start paused; resume via /openvins_replay_player/resume service",
        ),
        DeclareLaunchArgument("delay_sec", default_value="1.5", description="Delay before starting ros2 bag play (>= 0)"),
        DeclareLaunchArgument(
            "stereo_topic",
            default_value="/stereo/image_raw",
            description="Compressed stereo topic in MCAP; remapped to /stereo/image_raw/compressed for RViz2",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value="",
            description="Optional RViz2 config path; defaults to openvins_slam/launch/replay_mcap.rviz",
        ),
        OpaqueFunction(function=_make_actions),
    ])
