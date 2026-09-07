"""Generate evidence-rich manifests for the five real front-lidar MCAP bags."""

import argparse
import hashlib
import os
import subprocess
from collections import defaultdict
from pathlib import Path

import numpy as np
import yaml


REQUIRED = (
    "/clock",
    "/antbot/imu/data",
    "/antbot/lidar/front_left/points_raw_native",
    "/antbot/lidar/front_left/points_deskew_truth",
    "/antbot/ground_truth/odom",
    "/tf",
    "/tf_static",
)


def classify_scan_coverage(imu_stamps, scan_intervals):
    """Classify bag-boundary scans separately from valid full-scan coverage."""
    imu = np.asarray(imu_stamps, dtype=np.int64)
    intervals = np.asarray(scan_intervals, dtype=np.int64).reshape((-1, 2))
    result = {"total_frames": int(len(intervals)), "boundary_dropped_frames": 0,
              "complete_coverage_frames": 0, "internal_uncovered_frames": 0}
    if not len(imu):
        result["boundary_dropped_frames"] = int(len(intervals))
    else:
        for start, end in intervals:
            if start < imu[0] or end > imu[-1]:
                result["boundary_dropped_frames"] += 1
                continue
            before = np.searchsorted(imu, start, side="right") - 1
            after = np.searchsorted(imu, end, side="left")
            if before >= 0 and after < len(imu):
                result["complete_coverage_frames"] += 1
            else:
                result["internal_uncovered_frames"] += 1
    valid = result["total_frames"] - result["boundary_dropped_frames"]
    result["valid_frames"] = valid
    result["valid_coverage_ratio"] = (
        result["complete_coverage_frames"] / valid if valid else 0.0
    )
    result["valid_frames_100_percent_covered"] = (
        valid > 0
        and result["complete_coverage_frames"] == valid
        and result["internal_uncovered_frames"] == 0
    )
    return result


def clock_stamp_statistics(values):
    stamps = np.asarray(values, dtype=np.int64)
    differences = np.diff(stamps)
    groups = []
    if len(stamps):
        length = 1
        for difference in differences:
            if difference == 0:
                length += 1
            else:
                groups.append(length)
                length = 1
        groups.append(length)
    unique, counts = np.unique(stamps, return_counts=True)
    repeated_total = int(np.sum(np.maximum(counts - 1, 0)))
    consecutive = int(np.count_nonzero(differences == 0))
    return {
        "total_messages": int(len(stamps)),
        "unique_stamps": int(len(unique)),
        "consecutive_duplicates": consecutive,
        "nonconsecutive_duplicates": max(0, repeated_total - consecutive),
        "regressions": int(np.count_nonzero(differences < 0)),
        "maximum_duplicate_group_length": int(max(groups, default=0)),
    }


def _sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def inspect(path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Imu, PointCloud2
    from nav_msgs.msg import Odometry
    from sensor_msgs_py import point_cloud2

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    classes = {
        "/clock": Clock,
        "/antbot/imu/data": Imu,
        "/antbot/lidar/front_left/points_raw_native": PointCloud2,
        "/antbot/lidar/front_left/points_deskew_truth": PointCloud2,
        "/antbot/ground_truth/odom": Odometry,
    }
    counts = defaultdict(int)
    stamps = defaultdict(list)
    offset_min = None
    offset_max = None
    scan_intervals = []
    while reader.has_next():
        topic, data, bag_stamp = reader.read_next()
        counts[topic] += 1
        if topic not in classes:
            stamps[topic].append(int(bag_stamp))
            continue
        message = deserialize_message(data, classes[topic])
        if topic == "/clock":
            stamp = message.clock.sec * 1_000_000_000 + message.clock.nanosec
        else:
            stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        stamps[topic].append(stamp)
        if topic == "/antbot/lidar/front_left/points_raw_native":
            values = point_cloud2.read_points(
                message, field_names=["time_offset_ns"], skip_nans=False
            )["time_offset_ns"]
            if len(values):
                low, high = int(np.min(values)), int(np.max(values))
                offset_min = low if offset_min is None else min(offset_min, low)
                offset_max = high if offset_max is None else max(offset_max, high)
                scan_intervals.append((stamp + low, stamp + high))
    topic_stats = {}
    for topic in REQUIRED:
        values = np.asarray(stamps.get(topic, []), dtype=np.int64)
        differences = np.diff(values)
        span = int(values[-1] - values[0]) if len(values) > 1 else 0
        topic_stats[topic] = {
            "message_count": int(counts.get(topic, 0)),
            "average_frequency_hz": (
                float((len(values) - 1) * 1e9 / span) if span > 0 else 0.0
            ),
            "max_gap_sec": float(differences.max() * 1e-9) if len(differences) else None,
            "timestamp_regressions": int(np.count_nonzero(differences < 0)),
            "duplicate_timestamps": int(np.count_nonzero(differences == 0)),
        }
    imu_values = stamps["/antbot/imu/data"]
    scan_values = stamps["/antbot/lidar/front_left/points_raw_native"]
    scan_period = int(np.median(np.diff(scan_values))) if len(scan_values) > 2 else 100_000_000
    coverage = classify_scan_coverage(imu_values, scan_intervals)
    mcap_files = sorted(Path(path).glob("*.mcap"))
    size = sum(item.stat().st_size for item in mcap_files)
    result = {
        "bag_path": str(Path(path).resolve()),
        "mcap_files": [
            {"name": item.name, "size_bytes": item.stat().st_size, "sha256": _sha256(item)}
            for item in mcap_files
        ],
        "total_size_bytes": size,
        "start_sim_time_ns": stamps["/clock"][0] if stamps["/clock"] else None,
        "end_sim_time_ns": stamps["/clock"][-1] if stamps["/clock"] else None,
        "topic_statistics": topic_stats,
        "lidar_missing_frames": int(sum(
            max(0, round(gap / scan_period) - 1) for gap in np.diff(scan_values)
        )) if len(scan_values) > 1 else None,
        "imu_scan_coverage": coverage,
        "clock_stamp_statistics": clock_stamp_statistics(stamps["/clock"]),
        "time_offset_ns": {"minimum": offset_min, "maximum": offset_max},
    }
    result["duration_sec"] = (
        (result["end_sim_time_ns"] - result["start_sim_time_ns"]) * 1e-9
        if result["start_sim_time_ns"] is not None else 0.0
    )
    result["passed"] = (
        all(counts.get(topic, 0) > 0 for topic in REQUIRED)
        and all(value["timestamp_regressions"] == 0 for value in topic_stats.values())
        and result["imu_scan_coverage"]["valid_frames_100_percent_covered"]
        and offset_min is not None
    )
    return result


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("root", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args(argv)
    project = Path(os.environ.get("ANTBOT_SOURCE_ROOT", Path.cwd())).resolve()
    config_paths = (
        project / "isaac/config/imu_profiles.yaml",
        project / "ros2_ws/src/antbot_dual_lidar/config/lio_input_contract.yaml",
    )
    manifest = {
        "schema_version": 2,
        "status": "recorded",
        "git_commit": subprocess.run(
            ["git", "-C", str(project), "rev-parse", "HEAD"],
            check=True, capture_output=True, text=True,
        ).stdout.strip(),
        "git_dirty": bool(subprocess.run(
            ["git", "-C", str(project), "status", "--porcelain"],
            check=True, capture_output=True, text=True,
        ).stdout),
        "isaac_sim_version": "6.0.1-rc.7",
        "imu_profile": "ideal",
        "time_model_used_for_deskew_topic": (
            "DEFERRED_TO_REAL_HARDWARE_OR_LIO_RESEARCH"
        ),
        "deskew_reference_time": "header_stamp",
        "config_hashes": {item.name: _sha256(item) for item in config_paths},
        "datasets": [],
    }
    for path in sorted(item for item in args.root.iterdir() if item.is_dir()):
        result = inspect(path)
        result["case"] = path.name
        result["status"] = "RECORDED_VALIDATED" if result["passed"] else "RECORDED_FAILED_VALIDATION"
        manifest["datasets"].append(result)
    args.output.write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")
    print(yaml.safe_dump(manifest, sort_keys=False), end="")
    raise SystemExit(0 if all(item["passed"] for item in manifest["datasets"]) else 1)
