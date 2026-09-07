"""Repeatable ROS/DDS and host-resource probe for dual-lidar experiments."""

import argparse
import json
from pathlib import Path
import statistics
import subprocess
import time

import psutil
import rclpy
from rclpy.event_handler import SubscriptionEventCallbacks
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2


class CloudMetrics:
    def __init__(self, expected_period_ns):
        self.expected_period_ns = expected_period_ns
        self.count = 0
        self.bytes = []
        self.points = []
        self.stamps = []
        self.wall = []
        self.middleware_lost = 0
        self.incompatible_qos = 0

    def cloud(self, message):
        self.count += 1
        self.bytes.append(len(message.data))
        self.points.append(int(message.width * message.height))
        self.stamps.append(
            message.header.stamp.sec * 1_000_000_000
            + message.header.stamp.nanosec
        )
        self.wall.append(time.monotonic())

    def message_lost(self, event):
        self.middleware_lost += int(event.total_count_change)

    def qos_incompatible(self, event):
        self.incompatible_qos += int(event.total_count_change)

    def report(self):
        sim_intervals = [
            (right - left) * 1e-9
            for left, right in zip(self.stamps, self.stamps[1:])
            if right > left
        ]
        wall_intervals = [
            right - left for left, right in zip(self.wall, self.wall[1:])
        ]
        estimated_drops = sum(
            max(0, int(round(interval * 1e9 / self.expected_period_ns)) - 1)
            for interval in sim_intervals
        )

        def percentile(values, fraction):
            if not values:
                return None
            ordered = sorted(values)
            return ordered[min(len(ordered) - 1, round((len(ordered) - 1) * fraction))]

        return {
            "count": self.count,
            "mean_points": statistics.fmean(self.points) if self.points else None,
            "mean_bytes": statistics.fmean(self.bytes) if self.bytes else None,
            "sim_hz": 1.0 / statistics.fmean(sim_intervals) if sim_intervals else 0.0,
            "wall_hz": 1.0 / statistics.fmean(wall_intervals) if wall_intervals else 0.0,
            "period_p95_sec": percentile(sim_intervals, 0.95),
            "period_p99_sec": percentile(sim_intervals, 0.99),
            "estimated_stamp_drops": estimated_drops,
            "middleware_message_lost": self.middleware_lost,
            "incompatible_qos_events": self.incompatible_qos,
        }


def matching_processes():
    patterns = {
        "isaac": "run_antbot_dual_lidar.py",
        "preprocessor": "cloud_preprocessor",
        "rviz": "rviz2",
        "rosbag": "ros2 bag record",
    }
    matches = {name: [] for name in patterns}
    for process in psutil.process_iter(["pid", "cmdline"]):
        try:
            command = " ".join(process.info["cmdline"] or [])
            for name, pattern in patterns.items():
                if pattern in command and process.pid != psutil.Process().pid:
                    process.cpu_percent(None)
                    matches[name].append(process)
        except (psutil.AccessDenied, psutil.NoSuchProcess):
            continue
    return matches


def gpu_sample():
    try:
        result = subprocess.run(
            [
                "nvidia-smi",
                "--query-gpu=utilization.gpu,memory.used,memory.total",
                "--format=csv,noheader,nounits",
            ],
            check=True,
            capture_output=True,
            text=True,
            timeout=2.0,
        )
        gpu, used, total = (float(value.strip()) for value in result.stdout.splitlines()[0].split(","))
        return {"utilization_percent": gpu, "memory_used_mib": used, "memory_total_mib": total}
    except (OSError, subprocess.SubprocessError, ValueError, IndexError):
        return None


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--expected-frequency", type=float, default=10.0)
    parsed, ros_args = parser.parse_known_args(args=args)
    if parsed.duration <= 0.0 or parsed.expected_frequency <= 0.0:
        parser.error("duration and expected-frequency must be positive")
    rclpy.init(args=ros_args)
    node = rclpy.create_node("antbot_dual_lidar_performance_probe")
    topics = {
        "front_raw": "/antbot/lidar/front_left/points",
        "rear_raw": "/antbot/lidar/rear_right/points",
        "front_filtered": "/antbot/lidar/front_left/points_filtered",
        "rear_filtered": "/antbot/lidar/rear_right/points_filtered",
    }
    metrics = {
        name: CloudMetrics(int(1e9 / parsed.expected_frequency)) for name in topics
    }
    subscriptions = []
    for name, topic in topics.items():
        callbacks = SubscriptionEventCallbacks(
            message_lost=metrics[name].message_lost,
            incompatible_qos=metrics[name].qos_incompatible,
        )
        subscriptions.append(
            node.create_subscription(
                PointCloud2,
                topic,
                metrics[name].cloud,
                qos_profile_sensor_data,
                event_callbacks=callbacks,
            )
        )
    clock = {"count": 0, "last_ns": None}

    def clock_callback(message):
        clock["count"] += 1
        clock["last_ns"] = message.clock.sec * 1_000_000_000 + message.clock.nanosec

    subscriptions.append(
        node.create_subscription(Clock, "/clock", clock_callback, qos_profile_sensor_data)
    )
    processes = matching_processes()
    resource_samples = []
    start = time.monotonic()
    next_sample = start + 1.0
    while time.monotonic() - start < parsed.duration:
        rclpy.spin_once(node, timeout_sec=0.1)
        now = time.monotonic()
        if now >= next_sample:
            sample = {"elapsed_sec": now - start, "gpu": gpu_sample(), "processes": {}}
            for name, matches in processes.items():
                rows = []
                for process in matches:
                    try:
                        rows.append(
                            {
                                "pid": process.pid,
                                "cpu_percent": process.cpu_percent(None),
                                "rss_mib": process.memory_info().rss / (1024 * 1024),
                            }
                        )
                    except (psutil.AccessDenied, psutil.NoSuchProcess):
                        continue
                sample["processes"][name] = rows
            resource_samples.append(sample)
            next_sample = now + 1.0
    report = {
        "wall_duration_sec": time.monotonic() - start,
        "clock": clock,
        "topics": {name: value.report() for name, value in metrics.items()},
        "resources": resource_samples,
        "notes": {
            "subscriber_queue_depth": 5,
            "queue_backlog_inference": (
                "rclpy does not expose queue occupancy; infer overload from stamp gaps, "
                "middleware message_lost, and wall/simulation frequency divergence"
            ),
        },
    }
    encoded = json.dumps(report, indent=2, sort_keys=True)
    if parsed.output:
        parsed.output.parent.mkdir(parents=True, exist_ok=True)
        parsed.output.write_text(encoded + "\n")
    print("PERFORMANCE_RESULT " + encoded)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

