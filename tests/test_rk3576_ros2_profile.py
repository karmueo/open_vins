#!/usr/bin/env python3

import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path):
    return (ROOT / path).read_text(encoding="utf-8")


def parse_simple_yaml(path):
    values = {}
    for raw_line in read(path).splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line or ":" not in line or line.startswith("%"):
            continue
        key, value = line.split(":", 1)
        values[key.strip()] = value.strip().strip('"')
    return values


class TestRK3576ROS2Profile(unittest.TestCase):

    def test_rk3576_estimator_profile_has_bounded_frontend_load(self):
        values = parse_simple_yaml("config/euroc_mav/estimator_config_rk3576.yaml")
        expected = {
            "downsample_cameras": "true",
            "histogram_method": "NONE",
            "num_opencv_threads": "2",
            "max_slam": "20",
            "max_msckf_in_update": "25",
            "calib_cam_extrinsics": "false",
            "calib_cam_intrinsics": "false",
            "calib_cam_timeoffset": "false",
            "multi_threading_subs": "true",
            "executor_threads": "2",
            "max_camera_queue": "4",
            "drop_stale_camera_frames": "true",
        }
        for key, expected_value in expected.items():
            self.assertEqual(values.get(key), expected_value, key)
        self.assertIsNotNone(values.get("num_pts"))
        self.assertIsNotNone(values.get("track_frequency"))

    def test_subscribe_launch_exposes_runtime_load_and_threading_overrides(self):
        source = read("ov_msckf/launch/subscribe.launch.py")
        for name in (
            "downsample_cameras",
            "num_pts",
            "track_frequency",
            "histogram_method",
            "num_opencv_threads",
            "max_slam",
            "max_msckf_in_update",
            "calib_cam_extrinsics",
            "calib_cam_intrinsics",
            "calib_cam_timeoffset",
            "record_timing_information",
            "publish_global_to_imu_tf",
            "publish_calibration_tf",
            "multi_threading_subs",
            "executor_threads",
            "max_camera_queue",
            "drop_stale_camera_frames",
        ):
            self.assertIn(f'name="{name}"', source)
            self.assertIn(f'"{name}": LaunchConfiguration("{name}")', source)

    def test_vicon_room_launch_defaults_are_board_friendly(self):
        source = read("ov_msckf/launch/euroc_vicon_room.launch.py")
        defaults = {
            "rviz_enable": "false",
            "gt_align_enable": "true",
            "save_total_state": "true",
            "verbosity": "WARNING",
            "publish_global_to_imu_tf": "true",
            "publish_calibration_tf": "true",
        }
        for name, value in defaults.items():
            pattern = rf'DeclareLaunchArgument\(name="{name}", default_value="{value}"'
            self.assertRegex(source, pattern)
        self.assertIn("estimator_config_rk3576.yaml", source)

    def test_vicon_room_launch_does_not_override_yaml_tuning_by_default(self):
        source = read("ov_msckf/launch/euroc_vicon_room.launch.py")
        for name in (
            "downsample_cameras",
            "num_pts",
            "track_frequency",
            "histogram_method",
            "num_opencv_threads",
            "max_slam",
            "max_msckf_in_update",
            "calib_cam_extrinsics",
            "calib_cam_intrinsics",
            "calib_cam_timeoffset",
            "record_timing_information",
            "multi_threading_subs",
            "executor_threads",
            "max_camera_queue",
            "drop_stale_camera_frames",
        ):
            pattern = rf'DeclareLaunchArgument\(name="{name}", default_value=""'
            self.assertRegex(source, pattern)

    def test_ros2_entry_uses_configurable_threading(self):
        source = read("ov_msckf/src/run_subscribe_msckf.cpp")
        self.assertNotIn("params.use_multi_threading_subs = true;", source)
        self.assertIn('parse_config("executor_threads"', source)
        self.assertRegex(source, r"MultiThreadedExecutor\s+executor\([^;]*executor_threads")

    def test_ros2_entry_destroys_visualizer_before_shutdown(self):
        source = read("ov_msckf/src/run_subscribe_msckf.cpp")
        final_viz = source.index("viz->visualize_final();")
        reset_viz = source.index("viz.reset();")
        shutdown = source.index("rclcpp::shutdown();")
        self.assertLess(final_viz, reset_viz)
        self.assertLess(reset_viz, shutdown)

    def test_ros2_visualizer_uses_persistent_camera_worker(self):
        source = read("ov_msckf/src/ros/ROS2Visualizer.cpp")
        self.assertIn("camera_worker_thread", read("ov_msckf/src/ros/ROS2Visualizer.h"))
        self.assertIn("camera_queue_cv", read("ov_msckf/src/ros/ROS2Visualizer.h"))
        self.assertIn("process_camera_queue", source)
        callback_body = re.search(r"void ROS2Visualizer::callback_inertial[\s\S]*?\n}\n\nvoid ROS2Visualizer::callback_monocular", source)
        self.assertIsNotNone(callback_body)
        self.assertNotIn("std::thread", callback_body.group(0))
        self.assertNotIn("detach", callback_body.group(0))
        self.assertIn("drop_stale_camera_frames", source)
        self.assertIn("max_camera_queue", source)


if __name__ == "__main__":
    unittest.main()
