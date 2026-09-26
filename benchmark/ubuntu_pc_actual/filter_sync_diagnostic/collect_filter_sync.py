#!/usr/bin/env python3
"""Read-only dual MID360 filtering and timestamp diagnostic.

This node subscribes to the two IP-specific driver topics.  It does not
publish, alter messages, or change any driver/fusion parameter.
"""

import argparse
import csv
import json
import math
from pathlib import Path
import statistics
import threading
import time

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


TOPICS = {
    "front": "/livox/lidar_192_168_1_116",
    "rear": "/livox/lidar_192_168_1_139",
}
MIN_RANGE = 0.1
MAX_RANGE = 100.0
BLIND_WIDTH_DEG = 90.0

# Existing project TF, source lidar frame -> base_link.
TRANSFORMS = {
    "front": {
        "translation": [0.322, 0.222, 0.414],
        "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
        "rpy_deg": [0.0, 0.0, 0.0],
    },
    "rear": {
        "translation": [-0.322, -0.222, 0.414],
        "quaternion_xyzw": [0.0, 0.0, 1.0, 0.0],
        "rpy_deg": [0.0, 0.0, 180.0],
    },
}


def rotate_inverse_yaw(vector_xy, yaw_deg):
    yaw = math.radians(yaw_deg)
    x, y = vector_xy
    return (
        math.cos(yaw) * x + math.sin(yaw) * y,
        -math.sin(yaw) * x + math.cos(yaw) * y,
    )


def blind_center(transform):
    tx, ty, _ = transform["translation"]
    local_x, local_y = rotate_inverse_yaw((-tx, -ty), transform["rpy_deg"][2])
    return math.degrees(math.atan2(local_y, local_x))


BLIND_CENTERS = {name: blind_center(tf) for name, tf in TRANSFORMS.items()}


def xyz_view(message):
    fields = {field.name: field for field in message.fields}
    for name in ("x", "y", "z"):
        if name not in fields or fields[name].datatype != 7 or fields[name].count != 1:
            raise ValueError(f"{name} must be scalar FLOAT32")
    endian = ">" if message.is_bigendian else "<"
    dtype = np.dtype({
        "names": ["x", "y", "z"],
        "formats": [endian + "f4", endian + "f4", endian + "f4"],
        "offsets": [fields["x"].offset, fields["y"].offset, fields["z"].offset],
        "itemsize": message.point_step,
    })
    count = int(message.width) * int(message.height)
    return np.ndarray((count,), dtype=dtype, buffer=message.data)


class Diagnostic(Node):
    def __init__(self):
        super().__init__("dual_mid360_filter_sync_diagnostic")
        self.lock = threading.Lock()
        self.events = []
        self.frame_index = {"front": 0, "rear": 0}
        self.totals = {
            name: {
                "frames": 0,
                "raw": 0,
                "nan": 0,
                "inf": 0,
                "invalid": 0,
                "below_min": 0,
                "above_max": 0,
                "exact_zero": 0,
                "angular_pipeline": 0,
                "final": 0,
                "angular_only_kept": 0,
                "angular_denominator": 0,
                "hist": np.zeros(72, dtype=np.int64),
                "hist_in_range": np.zeros(72, dtype=np.int64),
            }
            for name in TOPICS
        }
        group = ReentrantCallbackGroup()
        self.subscription_refs = []
        for name, topic in TOPICS.items():
            self.subscription_refs.append(self.create_subscription(
                PointCloud2,
                topic,
                lambda message, sensor=name: self.callback(sensor, message),
                qos_profile_sensor_data,
                callback_group=group,
            ))

    def callback(self, sensor, message):
        arrival_system_ns = time.time_ns()
        arrival_steady_ns = time.monotonic_ns()
        header_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
        xyz = xyz_view(message)
        x = xyz["x"].astype(np.float64, copy=False)
        y = xyz["y"].astype(np.float64, copy=False)
        z = xyz["z"].astype(np.float64, copy=False)

        nan_mask = np.isnan(x) | np.isnan(y) | np.isnan(z)
        inf_mask = (~nan_mask) & (np.isinf(x) | np.isinf(y) | np.isinf(z))
        valid = ~(nan_mask | inf_mask)
        range_squared = x * x + y * y + z * z
        below = valid & (range_squared < MIN_RANGE * MIN_RANGE)
        above = valid & (range_squared > MAX_RANGE * MAX_RANGE)
        exact_zero = valid & (range_squared == 0.0)
        in_range = valid & ~below & ~above

        angles = np.degrees(np.arctan2(y[valid], x[valid]))
        center = BLIND_CENTERS[sensor]
        wrapped_difference = (angles - center + 180.0) % 360.0 - 180.0
        blind_valid = np.abs(wrapped_difference) <= BLIND_WIDTH_DEG * 0.5
        angular_only_kept = int(np.count_nonzero(~blind_valid))
        histogram, _ = np.histogram(angles, bins=np.linspace(-180.0, 180.0, 73))

        all_angles = np.degrees(np.arctan2(y, x))
        wrapped_all = (all_angles - center + 180.0) % 360.0 - 180.0
        blind_all = np.abs(wrapped_all) <= BLIND_WIDTH_DEG * 0.5
        angular_pipeline = in_range & blind_all
        kept = in_range & ~blind_all
        in_range_histogram, _ = np.histogram(
            all_angles[in_range], bins=np.linspace(-180.0, 180.0, 73)
        )

        counts = {
            "raw": int(xyz.shape[0]),
            "nan": int(np.count_nonzero(nan_mask)),
            "inf": int(np.count_nonzero(inf_mask)),
            "invalid": int(np.count_nonzero(~valid)),
            "below_min": int(np.count_nonzero(below)),
            "above_max": int(np.count_nonzero(above)),
            "exact_zero": int(np.count_nonzero(exact_zero)),
            "angular_pipeline": int(np.count_nonzero(angular_pipeline)),
            "final": int(np.count_nonzero(kept)),
            "angular_only_kept": angular_only_kept,
            "angular_denominator": int(np.count_nonzero(valid)),
        }
        with self.lock:
            self.frame_index[sensor] += 1
            frame_index = self.frame_index[sensor]
            total = self.totals[sensor]
            total["frames"] += 1
            for key, value in counts.items():
                total[key] += value
            total["hist"] += histogram
            total["hist_in_range"] += in_range_histogram
            self.events.append({
                "sensor": sensor,
                "frame_index": frame_index,
                "header_ns": header_ns,
                "arrival_system_ns": arrival_system_ns,
                "arrival_steady_ns": arrival_steady_ns,
                **counts,
            })


def simulate_sync(events, tolerance_ms=30.0, queue_size=10):
    tolerance_ns = int(tolerance_ms * 1e6)
    queues = {"front": [], "rear": []}
    pairs = []
    drops = []
    for event in sorted(events, key=lambda row: (row["arrival_steady_ns"], row["sensor"])):
        queue = queues[event["sensor"]]
        queue.append(event)
        while len(queue) > queue_size:
            dropped = queue.pop(0)
            drops.append({
                "reason": "queue_limit",
                "dropped_sensor": event["sensor"],
                "dropped_frame_index": dropped["frame_index"],
                "dropped_header_ns": dropped["header_ns"],
                "best_abs_delta_ms": "",
            })
        while queues["front"] and queues["rear"]:
            best = None
            for front_index, front in enumerate(queues["front"]):
                for rear_index, rear in enumerate(queues["rear"]):
                    delta = abs(front["header_ns"] - rear["header_ns"])
                    if best is None or delta < best[0]:
                        best = (delta, front_index, rear_index, front, rear)
            delta, front_index, rear_index, front, rear = best
            if delta <= tolerance_ns:
                queues["front"].pop(front_index)
                queues["rear"].pop(rear_index)
                pairs.append({
                    "front_frame_index": front["frame_index"],
                    "rear_frame_index": rear["frame_index"],
                    "front_header_ns": front["header_ns"],
                    "rear_header_ns": rear["header_ns"],
                    "signed_header_delta_ms": (front["header_ns"] - rear["header_ns"]) / 1e6,
                    "abs_header_delta_ms": delta / 1e6,
                    "front_arrival_system_ns": front["arrival_system_ns"],
                    "rear_arrival_system_ns": rear["arrival_system_ns"],
                    "signed_arrival_delta_ms": (
                        front["arrival_steady_ns"] - rear["arrival_steady_ns"]
                    ) / 1e6,
                    "abs_arrival_delta_ms": abs(
                        front["arrival_steady_ns"] - rear["arrival_steady_ns"]
                    ) / 1e6,
                })
                continue
            front_old = queues["front"][0]
            rear_old = queues["rear"][0]
            if front_old["header_ns"] < rear_old["header_ns"] - tolerance_ns:
                dropped_sensor = "front"
            elif rear_old["header_ns"] < front_old["header_ns"] - tolerance_ns:
                dropped_sensor = "rear"
            else:
                break
            dropped = queues[dropped_sensor].pop(0)
            drops.append({
                "reason": "timestamp_delta",
                "dropped_sensor": dropped_sensor,
                "dropped_frame_index": dropped["frame_index"],
                "dropped_header_ns": dropped["header_ns"],
                "best_abs_delta_ms": delta / 1e6,
            })
    return pairs, drops, queues


def write_csv(path, rows, fieldnames):
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def percentile(values, percentile_value):
    return float(np.percentile(np.asarray(values, dtype=np.float64), percentile_value))


def describe(values):
    return {
        "samples": len(values),
        "mean": statistics.fmean(values),
        "min": min(values),
        "p50": percentile(values, 50),
        "p90": percentile(values, 90),
        "p95": percentile(values, 95),
        "p99": percentile(values, 99),
        "max": max(values),
    }


def finalize(node, output_dir, wall_duration):
    output_dir.mkdir(parents=True, exist_ok=True)
    with node.lock:
        events = list(node.events)
        totals = node.totals
    pairs, drops, remaining = simulate_sync(events)

    event_fields = [
        "sensor", "frame_index", "header_ns", "arrival_system_ns", "arrival_steady_ns",
        "raw", "nan", "inf", "invalid", "below_min", "above_max", "exact_zero",
        "angular_pipeline", "final", "angular_only_kept", "angular_denominator",
    ]
    write_csv(output_dir / "frame_events.csv", events, event_fields)
    pair_fields = list(pairs[0].keys()) if pairs else []
    if pair_fields:
        write_csv(output_dir / "sync_pairs.csv", pairs, pair_fields)
    drop_fields = [
        "reason", "dropped_sensor", "dropped_frame_index", "dropped_header_ns",
        "best_abs_delta_ms",
    ]
    write_csv(output_dir / "sync_drops.csv", drops, drop_fields)

    histogram_edges = np.linspace(-180.0, 180.0, 73)
    for sensor in ("front", "rear"):
        frame_count = totals[sensor]["frames"]
        histogram_rows = []
        for index, count in enumerate(totals[sensor]["hist"]):
            histogram_rows.append({
                "bin_start_deg": histogram_edges[index],
                "bin_end_deg": histogram_edges[index + 1],
                "points": int(count),
                "avg_points_per_frame": float(count) / frame_count,
                "points_per_second": float(count) / wall_duration,
            })
        write_csv(
            output_dir / f"{sensor}_angle_histogram.csv",
            histogram_rows,
            list(histogram_rows[0].keys()),
        )
        in_range_histogram_rows = []
        for index, count in enumerate(totals[sensor]["hist_in_range"]):
            in_range_histogram_rows.append({
                "bin_start_deg": histogram_edges[index],
                "bin_end_deg": histogram_edges[index + 1],
                "points": int(count),
                "avg_points_per_frame": float(count) / frame_count,
                "points_per_second": float(count) / wall_duration,
            })
        write_csv(
            output_dir / f"{sensor}_angle_histogram_in_range.csv",
            in_range_histogram_rows,
            list(in_range_histogram_rows[0].keys()),
        )

    absolute_header = [row["abs_header_delta_ms"] for row in pairs]
    signed_header = [row["signed_header_delta_ms"] for row in pairs]
    absolute_arrival = [row["abs_arrival_delta_ms"] for row in pairs]
    signed_arrival = [row["signed_arrival_delta_ms"] for row in pairs]
    delta_bins = [
        ("0-5", 0.0, 5.0),
        ("5-10", 5.0, 10.0),
        ("10-15", 10.0, 15.0),
        ("15-20", 15.0, 20.0),
        ("20-25", 20.0, 25.0),
        ("25-30", 25.0, 30.0),
        ("30-35", 30.0, 35.0),
        ("35-40", 35.0, 40.0),
        ("40-50", 40.0, 50.0),
        ("50-100", 50.0, 100.0),
        (">100", 100.0, math.inf),
    ]
    delta_histogram = []
    for label, lower, upper in delta_bins:
        count = sum(lower <= value < upper for value in absolute_header)
        delta_histogram.append({
            "bin_ms": label,
            "lower_ms_inclusive": lower,
            "upper_ms_exclusive": upper,
            "count": count,
            "percent": 100.0 * count / len(absolute_header),
        })
    write_csv(
        output_dir / "sync_delta_histogram.csv",
        delta_histogram,
        list(delta_histogram[0].keys()),
    )
    summary = {
        "wall_duration_sec": wall_duration,
        "parameters": {
            "min_range": MIN_RANGE,
            "max_range": MAX_RANGE,
            "blind_width_deg": BLIND_WIDTH_DEG,
            "sync_tolerance_ms": 30.0,
            "sync_queue_size": 10,
        },
        "transforms": TRANSFORMS,
        "blind_centers_deg": BLIND_CENTERS,
        "filter_totals": {
            sensor: {
                key: (value.tolist() if isinstance(value, np.ndarray) else value)
                for key, value in totals[sensor].items()
                if key not in ("hist", "hist_in_range")
            }
            for sensor in ("front", "rear")
        },
        "sync": {
            "pairs": len(pairs),
            "drops": len(drops),
            "drop_reasons": {
                reason: sum(row["reason"] == reason for row in drops)
                for reason in ("timestamp_delta", "queue_limit")
            },
            "drop_sensors": {
                sensor: sum(row["dropped_sensor"] == sensor for row in drops)
                for sensor in ("front", "rear")
            },
            "remaining_queues": {sensor: len(queue) for sensor, queue in remaining.items()},
            "absolute_header_delta_ms": describe(absolute_header),
            "signed_header_delta_ms": {
                "mean": statistics.fmean(signed_header),
                "median": statistics.median(signed_header),
                "front_leads_count": sum(value < 0 for value in signed_header),
                "rear_leads_count": sum(value > 0 for value in signed_header),
                "equal_count": sum(value == 0 for value in signed_header),
            },
            "absolute_arrival_delta_ms": describe(absolute_arrival),
            "signed_arrival_delta_ms": {
                "mean": statistics.fmean(signed_arrival),
                "median": statistics.median(signed_arrival),
            },
            "threshold_percent": {
                f"lt_{threshold}_ms": 100.0 * sum(value < threshold for value in absolute_header) / len(absolute_header)
                for threshold in (10, 20, 30, 40, 50)
            },
        },
    }
    (output_dir / "diagnostic_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=300.0)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()

    rclpy.init()
    node = Diagnostic()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    started = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - started < args.duration:
            # spin_once dispatches at most one ready callback.  Keep the timeout
            # well below the aggregate 20 Hz input rate so neither stream is
            # artificially throttled by the diagnostic executor itself.
            executor.spin_once(timeout_sec=0.005)
    finally:
        executor.shutdown(timeout_sec=10.0)
        wall_duration = time.monotonic() - started
        finalize(node, args.output_dir, wall_duration)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
