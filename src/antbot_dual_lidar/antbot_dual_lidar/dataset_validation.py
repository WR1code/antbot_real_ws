"""Validate Phase 2A MCAP contents, timestamps, and sensor coverage."""

import argparse
from collections import defaultdict
from pathlib import Path

import yaml


REQUIRED_TOPICS = (
    "/clock",
    "/antbot/lidar/front_left/points_raw_native",
    "/antbot/lidar/rear_right/points_raw_native",
    "/antbot/imu/data_raw",
    "/antbot/imu/data",
    "/antbot/ground_truth/odom",
    "/odom",
    "/tf",
    "/tf_static",
    "/joint_states",
    "/cmd_vel",
)
HEADER_TOPICS = {
    "/antbot/lidar/front_left/points_raw_native",
    "/antbot/lidar/rear_right/points_raw_native",
    "/antbot/imu/data_raw",
    "/antbot/imu/data",
    "/antbot/ground_truth/odom",
    "/odom",
    "/joint_states",
}


RESET_TOPICS = (
    "/clock",
    "/antbot/lidar/front_left/points_raw_native",
    "/antbot/lidar/rear_right/points_raw_native",
    "/antbot/imu/data_raw",
    "/antbot/imu/data",
    "/antbot/ground_truth/odom",
)


def assess_dataset(counts, ranges, regressions, expect_reset=False):
    missing = [topic for topic in REQUIRED_TOPICS if counts.get(topic, 0) == 0]
    coverage_topics = (
        "/antbot/lidar/front_left/points_raw_native",
        "/antbot/lidar/rear_right/points_raw_native",
        "/antbot/imu/data",
    )
    coverage = None
    if all(topic in ranges for topic in coverage_topics):
        start = max(ranges[topic][0] for topic in coverage_topics)
        end = min(ranges[topic][1] for topic in coverage_topics)
        coverage = {
            "common_start_ns": int(start),
            "common_end_ns": int(end),
            "common_duration_sec": max(0.0, (end - start) * 1e-9),
        }
    reset_consistent = (
        all(regressions.get(topic, 0) > 0 for topic in RESET_TOPICS)
        if expect_reset else not any(regressions.values())
    )
    return {
        "passed": not missing and reset_consistent and coverage is not None
        and coverage["common_duration_sec"] > 0.0,
        "missing_topics": missing,
        "message_counts": {topic: int(counts.get(topic, 0)) for topic in REQUIRED_TOPICS},
        "timestamp_regressions": {
            topic: int(count) for topic, count in regressions.items() if count
        },
        "sensor_time_ranges_ns": {
            topic: [int(value[0]), int(value[1])] for topic, value in ranges.items()
            if topic in coverage_topics
        },
        "imu_lidar_common_coverage": coverage,
        "reset_expected": bool(expect_reset),
        "reset_consistent_across_clock_and_sensors": bool(reset_consistent),
    }


def _message_stamp_ns(topic, message):
    if topic == "/clock":
        return message.clock.sec * 1_000_000_000 + message.clock.nanosec
    if topic in HEADER_TOPICS:
        return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
    return None


def inspect_bag(path, expect_reset=False):
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as error:
        raise RuntimeError("rosbag2_py and ROS 2 Python environment are required") from error
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    message_types = {
        topic: get_message(type_name)
        for topic, type_name in types.items()
        if topic in HEADER_TOPICS or topic == "/clock"
    }
    counts = defaultdict(int)
    ranges = {}
    last_stamps = {}
    regressions = defaultdict(int)
    while reader.has_next():
        topic, serialized, _bag_stamp = reader.read_next()
        counts[topic] += 1
        if topic not in message_types:
            continue
        message = deserialize_message(serialized, message_types[topic])
        stamp = _message_stamp_ns(topic, message)
        if stamp is None:
            continue
        if topic in last_stamps and stamp < last_stamps[topic]:
            regressions[topic] += 1
        last_stamps[topic] = stamp
        if topic not in ranges:
            ranges[topic] = [stamp, stamp]
        else:
            ranges[topic][0] = min(ranges[topic][0], stamp)
            ranges[topic][1] = max(ranges[topic][1], stamp)
    return assess_dataset(counts, ranges, regressions, expect_reset=expect_reset)


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_path", type=Path)
    parser.add_argument("--capture-metadata", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    capture = None
    if args.capture_metadata and args.capture_metadata.is_file():
        capture = yaml.safe_load(args.capture_metadata.read_text())
    expect_reset = bool(capture and capture.get("case") == "12_reset_recovery")
    result = inspect_bag(args.bag_path, expect_reset=expect_reset)
    result["bag_path"] = str(args.bag_path)
    if capture:
        result["capture"] = capture
    output = args.output or Path(str(args.bag_path) + ".validation.yaml")
    output.write_text(yaml.safe_dump(result, sort_keys=False), encoding="utf-8")
    print(yaml.safe_dump(result, sort_keys=False), end="")
    raise SystemExit(0 if result["passed"] else 1)
