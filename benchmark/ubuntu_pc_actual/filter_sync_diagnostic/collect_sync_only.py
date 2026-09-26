#!/usr/bin/env python3
"""Low-overhead dual MID360 header/arrival timing collector."""

import argparse
import csv
import json
import math
from pathlib import Path
import statistics
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


TOPICS = {
    "front": "/livox/lidar_192_168_1_116",
    "rear": "/livox/lidar_192_168_1_139",
}


class Collector(Node):
    def __init__(self):
        super().__init__("dual_mid360_sync_timing_diagnostic")
        self.events = []
        self.indices = {"front": 0, "rear": 0}
        self.subscription_refs = []
        for sensor, topic in TOPICS.items():
            self.subscription_refs.append(self.create_subscription(
                PointCloud2,
                topic,
                lambda message, name=sensor: self.callback(name, message),
                qos_profile_sensor_data,
            ))

    def callback(self, sensor, message):
        arrival_system_ns = time.time_ns()
        arrival_steady_ns = time.monotonic_ns()
        self.indices[sensor] += 1
        self.events.append({
            "sensor": sensor,
            "frame_index": self.indices[sensor],
            "header_ns": int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec),
            "arrival_system_ns": arrival_system_ns,
            "arrival_steady_ns": arrival_steady_ns,
            "points": int(message.width) * int(message.height),
        })


def simulate(events, tolerance_ns=30_000_000, queue_size=10):
    queues = {"front": [], "rear": []}
    pairs = []
    drops = []
    for event in sorted(events, key=lambda row: row["arrival_steady_ns"]):
        queue = queues[event["sensor"]]
        queue.append(event)
        while len(queue) > queue_size:
            dropped = queue.pop(0)
            drops.append({
                "reason": "queue_limit", "dropped_sensor": event["sensor"],
                "dropped_frame_index": dropped["frame_index"],
                "dropped_header_ns": dropped["header_ns"], "best_abs_delta_ms": "",
                "arrival_order": len(pairs) + len(drops),
            })
        while queues["front"] and queues["rear"]:
            best = None
            for fi, front in enumerate(queues["front"]):
                for ri, rear in enumerate(queues["rear"]):
                    delta = abs(front["header_ns"] - rear["header_ns"])
                    if best is None or delta < best[0]:
                        best = (delta, fi, ri, front, rear)
            delta, fi, ri, front, rear = best
            if delta <= tolerance_ns:
                queues["front"].pop(fi)
                queues["rear"].pop(ri)
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
                side = "front"
            elif rear_old["header_ns"] < front_old["header_ns"] - tolerance_ns:
                side = "rear"
            else:
                break
            dropped = queues[side].pop(0)
            drops.append({
                "reason": "timestamp_delta", "dropped_sensor": side,
                "dropped_frame_index": dropped["frame_index"],
                "dropped_header_ns": dropped["header_ns"],
                "best_abs_delta_ms": delta / 1e6,
                "arrival_order": len(pairs) + len(drops),
            })
    return pairs, drops, queues


def percentile(values, value):
    return float(np.percentile(np.asarray(values, dtype=np.float64), value))


def describe(values):
    return {
        "samples": len(values), "mean": statistics.fmean(values), "min": min(values),
        "p50": percentile(values, 50), "p90": percentile(values, 90),
        "p95": percentile(values, 95), "p99": percentile(values, 99), "max": max(values),
    }


def write_csv(path, rows, fields):
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def analyze_intervals(events, sensor):
    rows = [row for row in events if row["sensor"] == sensor]
    headers = np.asarray([row["header_ns"] for row in rows], dtype=np.int64)
    arrivals = np.asarray([row["arrival_steady_ns"] for row in rows], dtype=np.int64)
    header_intervals = np.diff(headers) / 1e6
    arrival_intervals = np.diff(arrivals) / 1e6
    duration = (headers[-1] - headers[0]) / 1e9
    return {
        "frames": len(rows),
        "header_span_sec": duration,
        "header_frequency_hz": (len(rows) - 1) / duration,
        "header_interval_ms": describe(header_intervals.tolist()),
        "arrival_interval_ms": describe(arrival_intervals.tolist()),
        "gaps_over_150ms": int(np.count_nonzero(header_intervals > 150.0)),
        "estimated_missing_periods": int(sum(max(0, round(value / 100.0) - 1) for value in header_intervals)),
    }


def finalize(node, output_dir, wall_duration):
    output_dir.mkdir(parents=True, exist_ok=True)
    events = node.events
    pairs, drops, remaining = simulate(events)
    write_csv(output_dir / "sync_only_events.csv", events, list(events[0].keys()))
    write_csv(output_dir / "sync_only_pairs.csv", pairs, list(pairs[0].keys()))
    write_csv(
        output_dir / "sync_only_drops.csv", drops,
        ["reason", "dropped_sensor", "dropped_frame_index", "dropped_header_ns",
         "best_abs_delta_ms", "arrival_order"],
    )

    absolute_header = [row["abs_header_delta_ms"] for row in pairs]
    signed_header = [row["signed_header_delta_ms"] for row in pairs]
    absolute_arrival = [row["abs_arrival_delta_ms"] for row in pairs]
    signed_arrival = [row["signed_arrival_delta_ms"] for row in pairs]
    bins = [(0, 5), (5, 10), (10, 15), (15, 20), (20, 25), (25, 30),
            (30, 35), (35, 40), (40, 50), (50, 100), (100, math.inf)]
    hist = []
    for lower, upper in bins:
        count = sum(lower <= value < upper for value in absolute_header)
        hist.append({
            "bin_ms": f">100" if math.isinf(upper) else f"{lower}-{upper}",
            "lower_ms_inclusive": lower, "upper_ms_exclusive": upper,
            "count": count, "percent": 100.0 * count / len(absolute_header),
        })
    write_csv(output_dir / "sync_delta_histogram.csv", hist, list(hist[0].keys()))

    summary = {
        "wall_duration_sec": wall_duration,
        "streams": {sensor: analyze_intervals(events, sensor) for sensor in TOPICS},
        "pairs": len(pairs),
        "drops": len(drops),
        "drop_reasons": {
            reason: sum(row["reason"] == reason for row in drops)
            for reason in ("timestamp_delta", "queue_limit")
        },
        "drop_sensors": {
            sensor: sum(row["dropped_sensor"] == sensor for row in drops)
            for sensor in TOPICS
        },
        "remaining_queues": {sensor: len(queue) for sensor, queue in remaining.items()},
        "absolute_header_delta_ms": describe(absolute_header),
        "signed_header_delta_ms": {
            "mean": statistics.fmean(signed_header),
            "median": statistics.median(signed_header),
            "front_leads_count": sum(value < 0 for value in signed_header),
            "rear_leads_count": sum(value > 0 for value in signed_header),
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
    }
    (output_dir / "sync_only_summary.json").write_text(json.dumps(summary, indent=2) + "\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=300.0)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = Collector()
    started = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - started < args.duration:
            rclpy.spin_once(node, timeout_sec=0.005)
    finally:
        duration = time.monotonic() - started
        finalize(node, args.output_dir, duration)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
