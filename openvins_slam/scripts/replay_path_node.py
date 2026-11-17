#!/usr/bin/env python3
"""Accumulate /openvins/pose_imu into a live /openvins/path_imu_live Path during MCAP playback."""

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class ReplayPathNode(Node):
    def __init__(self):
        super().__init__('openvins_replay_path')
        self.declare_parameter('pose_topic', '/openvins/pose_imu')
        self.declare_parameter('path_topic', '/openvins/path_imu_live')
        pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        path_topic = self.get_parameter('path_topic').get_parameter_value().string_value

        sub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        pub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(Path, path_topic, pub_qos)
        self._subscription = self.create_subscription(
            PoseWithCovarianceStamped, pose_topic, self._on_pose, sub_qos
        )
        self._path = Path()
        self._path.header.frame_id = 'odom'
        self._last_stamp_ns = None

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if self._last_stamp_ns is not None and stamp_ns < self._last_stamp_ns:
            self._path.poses.clear()
        self._last_stamp_ns = stamp_ns

        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        if not pose_stamped.header.frame_id:
            pose_stamped.header.frame_id = 'odom'
        pose_stamped.pose = msg.pose.pose

        self._path.header = pose_stamped.header
        self._path.poses.append(pose_stamped)
        self._publisher.publish(self._path)


def main():
    rclpy.init()
    node = ReplayPathNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
