"""Measure observed angular return coverage without claiming geometric visibility."""

import argparse
import csv
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class AngularAccumulator:
    def __init__(self, bin_degrees):
        self.bin_degrees = bin_degrees
        self.edges = np.arange(-180.0, 180.0 + bin_degrees, bin_degrees)
        bins = len(self.edges) - 1
        self.frames = 0
        self.count = np.zeros(bins, dtype=np.int64)
        self.min_range = np.full(bins, np.inf)
        self.min_z = np.full(bins, np.inf)
        self.max_z = np.full(bins, -np.inf)

    def add(self, message):
        values = point_cloud2.read_points_numpy(
            message, field_names=("x", "y", "z"), skip_nans=True
        )
        xyz = np.asarray(values, dtype=np.float32)
        if not len(xyz):
            return
        self.frames += 1
        angles = np.degrees(np.arctan2(xyz[:, 1], xyz[:, 0]))
        ranges = np.linalg.norm(xyz, axis=1)
        indices = np.clip(
            np.floor((angles + 180.0) / self.bin_degrees).astype(np.int64),
            0,
            len(self.count) - 1,
        )
        self.count += np.bincount(indices, minlength=len(self.count))
        np.minimum.at(self.min_range, indices, ranges)
        np.minimum.at(self.min_z, indices, xyz[:, 2])
        np.maximum.at(self.max_z, indices, xyz[:, 2])

    def rows(self, sensor):
        for index, count in enumerate(self.count):
            yield {
                "sensor": sensor,
                "angle_start_deg": self.edges[index],
                "angle_end_deg": self.edges[index + 1],
                "frames": self.frames,
                "return_count": int(count),
                "mean_returns_per_frame": count / self.frames if self.frames else 0.0,
                "min_range_m": self.min_range[index] if np.isfinite(self.min_range[index]) else "",
                "min_z_m": self.min_z[index] if np.isfinite(self.min_z[index]) else "",
                "max_z_m": self.max_z[index] if np.isfinite(self.max_z[index]) else "",
            }


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--bin-degrees", type=float, default=1.0)
    parser.add_argument("--output", type=Path, required=True)
    parsed, ros_args = parser.parse_known_args(args=args)
    if parsed.duration <= 0 or not 0 < parsed.bin_degrees <= 45:
        parser.error("duration must be positive and bin-degrees must be in (0, 45]")
    rclpy.init(args=ros_args)
    node = rclpy.create_node("antbot_dual_lidar_coverage_probe")
    accumulators = {
        "front_left": AngularAccumulator(parsed.bin_degrees),
        "rear_right": AngularAccumulator(parsed.bin_degrees),
    }
    subscriptions = [
        node.create_subscription(
            PointCloud2,
            "/antbot/lidar/front_left/points",
            accumulators["front_left"].add,
            qos_profile_sensor_data,
        ),
        node.create_subscription(
            PointCloud2,
            "/antbot/lidar/rear_right/points",
            accumulators["rear_right"].add,
            qos_profile_sensor_data,
        ),
    ]
    start = time.monotonic()
    while time.monotonic() - start < parsed.duration:
        rclpy.spin_once(node, timeout_sec=0.1)
    fieldnames = list(next(accumulators["front_left"].rows("front_left")).keys())
    parsed.output.parent.mkdir(parents=True, exist_ok=True)
    with parsed.output.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for sensor, accumulator in accumulators.items():
            writer.writerows(accumulator.rows(sensor))
    print(
        f"COVERAGE_RESULT output={parsed.output} "
        + " ".join(f"{name}_frames={value.frames}" for name, value in accumulators.items())
    )
    subscriptions.clear()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
