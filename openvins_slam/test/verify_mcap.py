#!/usr/bin/env python3
"""Independent checks for preservation and standard VIO messages in an MCAP."""
import argparse
import collections
import hashlib
import math
import struct

from mcap.reader import NonSeekingReader, make_reader
from mcap_ros2.decoder import DecoderFactory

POSES = ("/openvins/pose_imu", "/openvins/pose_cam0", "/openvins/pose_cam1")
PATHS = ("/openvins/path_imu", "/openvins/path_cam0", "/openvins/path_cam1")
RESULT_TOPICS = set(POSES + PATHS + ("/openvins/odom", "/tf", "/tf_static"))


def fingerprint(path, topics=None):
    grouped = collections.defaultdict(list)
    with open(path, "rb") as stream:
        # SeekingReader queues every chunk before emitting records when log_time_order=False.
        # Stream sequentially to keep validation memory bounded for multi-gigabyte bags.
        reader = NonSeekingReader(stream)
        for schema, channel, message in reader.iter_messages(topics=topics, log_time_order=False):
            schema_key = (schema.name, schema.encoding, hashlib.sha256(schema.data).digest()) if schema else (None, None, None)
            key = (channel.topic, channel.message_encoding, schema_key, tuple(sorted(channel.metadata.items())))
            digest = hashlib.sha256()
            digest.update(struct.pack("<QQQ", message.sequence, message.log_time, message.publish_time))
            digest.update(message.data)
            grouped[key].append(digest.digest())
    return {key: sorted(values) for key, values in grouped.items()}


def ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def finite_pose(pose):
    p, q = pose.position, pose.orientation
    values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
    assert all(math.isfinite(v) for v in values), values
    assert abs(sum(v * v for v in values[3:]) - 1.0) < 1e-6, values


def pose_record(header, pose):
    p, q = pose.position, pose.orientation
    return (ns(header.stamp), header.frame_id, p.x, p.y, p.z, q.x, q.y, q.z, q.w)


def covariance(values):
    assert len(values) == 36 and all(math.isfinite(v) for v in values)
    for i in range(6):
        assert values[6 * i + i] >= -1e-10
        for j in range(6):
            assert math.isclose(values[6 * i + j], values[6 * j + i], rel_tol=1e-6, abs_tol=1e-9)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source")
    parser.add_argument("output")
    args = parser.parse_args()
    before = fingerprint(args.source)
    source_topics = {key[0] for key in before}
    after = fingerprint(args.output, topics=list(source_topics))
    assert before == after, "original schema/channel/sequence/timestamp/CDR differ"
    counts = collections.Counter()
    last_stamp = {}
    final_paths = {}
    pose_records = collections.defaultdict(list)
    poses = {}
    transforms = {}
    static = set()
    with open(args.output, "rb") as stream:
        reader = make_reader(stream, decoder_factories=[DecoderFactory()])
        for _, channel, record, msg in reader.iter_decoded_messages(topics=list(RESULT_TOPICS), log_time_order=True):
            topic = channel.topic
            counts[topic] += 1
            if topic in POSES:
                t = ns(msg.header.stamp)
                assert msg.header.frame_id == "odom" and t > last_stamp.get(topic, -1)
                last_stamp[topic] = t
                finite_pose(msg.pose.pose)
                covariance(msg.pose.covariance)
                pose_records[topic].append(pose_record(msg.header, msg.pose.pose))
                if topic == POSES[0]:
                    p, q = msg.pose.pose.position, msg.pose.pose.orientation
                    poses[t] = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
            elif topic in PATHS:
                t = ns(msg.header.stamp)
                assert msg.header.frame_id == "odom" and t > last_stamp.get(topic, -1)
                last_stamp[topic] = t
                assert topic not in final_paths, f"multiple recorded Paths for {topic}"
                assert msg.poses and ns(msg.poses[-1].header.stamp) == t
                assert record.log_time == t and record.publish_time == t
                final_paths[topic] = [pose_record(p.header, p.pose) for p in msg.poses]
            elif topic == "/openvins/odom":
                t = ns(msg.header.stamp)
                assert msg.header.frame_id == "odom" and msg.child_frame_id == "imu" and t > last_stamp.get(topic, -1)
                last_stamp[topic] = t
                finite_pose(msg.pose.pose)
                covariance(msg.pose.covariance)
                covariance(msg.twist.covariance)
                v, w = msg.twist.twist.linear, msg.twist.twist.angular
                assert all(math.isfinite(x) for x in (v.x, v.y, v.z, w.x, w.y, w.z))
            elif topic == "/tf":
                for tf in msg.transforms:
                    assert tf.header.frame_id == "odom" and tf.child_frame_id == "imu"
                    v, q = tf.transform.translation, tf.transform.rotation
                    transforms[ns(tf.header.stamp)] = (v.x, v.y, v.z, q.x, q.y, q.z, q.w)
            elif topic == "/tf_static":
                for tf in msg.transforms:
                    assert tf.header.frame_id == "imu"
                    static.add(tf.child_frame_id)
    assert counts[POSES[0]] > 0, "no pose estimates"
    assert all(counts[t] == counts[POSES[0]] for t in POSES + ("/openvins/odom", "/tf")), counts
    assert all(counts[t] == 1 for t in PATHS), counts
    for pose_topic, path_topic in zip(POSES, PATHS):
        assert final_paths[path_topic] == pose_records[pose_topic], f"Path differs from per-frame poses: {path_topic}"
    assert static == {"cam0_optical", "cam1_optical"}
    assert poses == transforms, "TF and IMU pose differ"
    print("Original channels and serialized messages preserved:", sum(len(v) for v in before.values()))
    print("Pose updates:", counts[POSES[0]], "final path length:", len(final_paths[PATHS[0]]))
    print("PASS: schemas, original messages, poses, trajectories, odometry, and TF")


if __name__ == "__main__":
    main()
