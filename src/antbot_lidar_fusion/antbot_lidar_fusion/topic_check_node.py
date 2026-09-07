#!/usr/bin/env python3
"""Bounded runtime acceptance checker for the dual lidar demonstration."""

import argparse
import math
import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener


class TopicCheckNode(Node):
    """Collect message statistics and validate frames, point count, clock and TF."""

    def __init__(self) -> None:
        super().__init__("dual_lidar_topic_check")
        self.counts = {"clock": 0, "front": 0, "rear": 0, "cloud": 0}
        self.first_wall = {}
        self.last_wall = {}
        self.first_stamp_ns = {}
        self.last_stamp_ns = {}
        self.frames = {"front": "", "rear": "", "cloud": ""}
        self.finite_ranges = {"front": 0, "rear": 0}
        self.cloud_points = 0
        self.create_subscription(Clock, "/clock", lambda _: self._seen("clock"), 10)
        self.create_subscription(
            LaserScan, "/scan_0", lambda msg: self._scan("front", msg), qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan, "/scan_1", lambda msg: self._scan("rear", msg), qos_profile_sensor_data
        )
        self.create_subscription(
            PointCloud2,
            "/antbot/lidar/combined_points",
            self._cloud,
            qos_profile_sensor_data,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _seen(self, key: str) -> None:
        now = time.monotonic()
        self.counts[key] += 1
        self.first_wall.setdefault(key, now)
        self.last_wall[key] = now

    def _scan(self, key: str, msg: LaserScan) -> None:
        self._seen(key)
        self.frames[key] = msg.header.frame_id
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self.first_stamp_ns.setdefault(key, stamp_ns)
        self.last_stamp_ns[key] = stamp_ns
        self.finite_ranges[key] = max(
            self.finite_ranges[key],
            sum(
                math.isfinite(distance)
                and msg.range_min <= distance <= msg.range_max
                for distance in msg.ranges
            ),
        )

    def _cloud(self, msg: PointCloud2) -> None:
        self._seen("cloud")
        self.frames["cloud"] = msg.header.frame_id
        self.cloud_points = max(self.cloud_points, int(msg.width * msg.height))

    def frequency(self, key: str) -> float:
        if self.counts[key] < 2:
            return 0.0
        return (self.counts[key] - 1) / (self.last_wall[key] - self.first_wall[key])

    def simulation_frequency(self, key: str) -> float:
        elapsed_ns = self.last_stamp_ns.get(key, 0) - self.first_stamp_ns.get(key, 0)
        if self.counts[key] < 2 or elapsed_ns <= 0:
            return 0.0
        return (self.counts[key] - 1) * 1_000_000_000 / elapsed_ns

    def tf_ok(self, child: str) -> bool:
        try:
            self.tf_buffer.lookup_transform(
                "base_link", child, Time(), timeout=Duration(seconds=0.2)
            )
            return True
        except TransformException:
            return False


def main(args=None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=20.0)
    parsed, ros_args = parser.parse_known_args(args=args)
    rclpy.init(args=ros_args)
    node = TopicCheckNode()
    deadline = time.monotonic() + parsed.timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if all(value >= 3 for value in node.counts.values()) and node.cloud_points > 0:
            break

    front_hz, rear_hz = node.frequency("front"), node.frequency("rear")
    front_sim_hz = node.simulation_frequency("front")
    rear_sim_hz = node.simulation_frequency("rear")
    checks = {
        "clock updates": node.counts["clock"] >= 2,
        "front scan updates": node.counts["front"] >= 2,
        "rear scan updates": node.counts["rear"] >= 2,
        "front frame_id": node.frames["front"] == "lidar_2d_front_link",
        "rear frame_id": node.frames["rear"] == "lidar_2d_back_link",
        "front scan nonempty": node.finite_ranges["front"] > 0,
        "rear scan nonempty": node.finite_ranges["rear"] > 0,
        "cloud frame_id": node.frames["cloud"] == "base_link",
        "combined cloud nonempty": node.cloud_points > 0,
        "front TF": node.tf_ok("lidar_2d_front_link"),
        "rear TF": node.tf_ok("lidar_2d_back_link"),
        "front frequency plausible": 5.0 <= front_hz <= 15.0,
        "rear frequency plausible": 5.0 <= rear_hz <= 15.0,
    }
    print(
        f"COUNTS clock={node.counts['clock']} front={node.counts['front']} "
        f"rear={node.counts['rear']} cloud={node.counts['cloud']}"
    )
    print(f"FREQUENCY front={front_hz:.2f}Hz rear={rear_hz:.2f}Hz")
    print(
        f"SIM_FREQUENCY front={front_sim_hz:.2f}Hz rear={rear_sim_hz:.2f}Hz "
        f"FINITE_RANGES front={node.finite_ranges['front']} rear={node.finite_ranges['rear']}"
    )
    print(f"FRAMES {node.frames}")
    print(f"MAX_COMBINED_POINTS {node.cloud_points}")
    for name, passed in checks.items():
        print(f"{'PASS' if passed else 'FAIL'}: {name}")
    success = all(checks.values())
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if success else 1)


if __name__ == "__main__":
    main(sys.argv[1:])
