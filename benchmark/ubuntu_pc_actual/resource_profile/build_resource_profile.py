#!/usr/bin/env python3
"""Normalize existing A/B/C evidence into a resource-profile dataset."""

import csv
from datetime import datetime
import json
import math
from pathlib import Path
import re
import statistics
from zoneinfo import ZoneInfo

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
A_DIR = ROOT / "20260917_142845_A"
B_DIR = ROOT / "20260917_150255_B"
C_DIR = ROOT / "20260917_151026_C"
SYNC_DIR = ROOT / "filter_sync_diagnostic" / "20260917_170912_300s"
OUTPUT = Path(__file__).resolve().parent / "20260917_ABC"
TOTAL_RAM_KB = 65644560
TZ = ZoneInfo("Asia/Shanghai")


def epoch_for(clock_text):
    hour, minute, second = map(int, clock_text.split(":"))
    return datetime(2026, 9, 17, hour, minute, second, tzinfo=TZ).timestamp()


def percentile(values, q):
    return float(np.percentile(np.asarray(values, dtype=float), q))


def stats(values):
    values = list(map(float, values))
    return {
        "count": len(values),
        "mean": statistics.fmean(values),
        "median": statistics.median(values),
        "p95": percentile(values, 95),
        "p99": percentile(values, 99),
        "max": max(values),
        "min": min(values),
    }


def write_csv(path, rows, fieldnames):
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def parse_mpstat(path, stage):
    rows = []
    for line in path.read_text().splitlines():
        tokens = line.split()
        if len(tokens) < 12 or not re.fullmatch(r"\d{2}:\d{2}:\d{2}", tokens[0]):
            continue
        if tokens[1] != "all":
            continue
        rows.append({
            "stage": stage,
            "timestamp": epoch_for(tokens[0]),
            "clock": tokens[0],
            "system_cpu_pct": 100.0 - float(tokens[-1]),
        })
    return rows


def parse_sar_memory(path):
    result = {}
    for line in path.read_text().splitlines():
        tokens = line.split()
        if len(tokens) < 4 or not re.fullmatch(r"\d{2}:\d{2}:\d{2}", tokens[0]):
            continue
        try:
            available = float(tokens[2])
        except ValueError:
            continue
        result[tokens[0]] = {
            "ram_available_kb": available,
            "ram_used_kb": TOTAL_RAM_KB - available,
        }
    return result


def parse_pidstat(path, stage, names):
    rows_by_key = {}
    mode = None
    for line in path.read_text().splitlines():
        if "%CPU" in line and "Command" in line:
            mode = "cpu"
            continue
        if "RSS" in line and "Command" in line:
            mode = "memory"
            continue
        if "kB_rd/s" in line:
            mode = "io"
            continue
        tokens = line.split()
        if len(tokens) < 3 or not re.fullmatch(r"\d{2}:\d{2}:\d{2}", tokens[0]):
            continue
        if not tokens[2].isdigit():
            continue
        pid = int(tokens[2])
        if pid not in names:
            continue
        key = (tokens[0], pid)
        row = rows_by_key.setdefault(key, {
            "stage": stage,
            "timestamp": epoch_for(tokens[0]),
            "clock": tokens[0],
            "process": names[pid],
            "pid": pid,
            "cpu_pct": "",
            "rss_kb": "",
            "vsz_kb": "",
        })
        if mode == "cpu":
            row["cpu_pct"] = float(tokens[7])
        elif mode == "memory":
            row["vsz_kb"] = float(tokens[5])
            row["rss_kb"] = float(tokens[6])
    return sorted(rows_by_key.values(), key=lambda row: (row["timestamp"], row["pid"]))


def parse_sar_network(path, stage):
    rows = {}
    mode = None
    for line in path.read_text().splitlines():
        if "rxpck/s" in line and "rxkB/s" in line:
            mode = "dev"
            continue
        if "rxerr/s" in line and "rxdrop/s" in line:
            mode = "edev"
            continue
        tokens = line.split()
        if len(tokens) < 4 or not re.fullmatch(r"\d{2}:\d{2}:\d{2}", tokens[0]):
            continue
        if tokens[1] != "eno1":
            continue
        row = rows.setdefault(tokens[0], {
            "stage": stage,
            "timestamp": epoch_for(tokens[0]),
            "clock": tokens[0],
            "interface": "eno1",
            "rx_mb_s": "",
            "tx_mb_s": "",
            "rx_packets_s": "",
            "tx_packets_s": "",
            "rx_errors_s": "",
            "tx_errors_s": "",
            "rx_dropped_s": "",
            "tx_dropped_s": "",
            "rx_fifo_s": "",
        })
        if mode == "dev":
            row["rx_packets_s"] = float(tokens[2])
            row["tx_packets_s"] = float(tokens[3])
            row["rx_mb_s"] = float(tokens[4]) * 1024.0 / 1e6
            row["tx_mb_s"] = float(tokens[5]) * 1024.0 / 1e6
        elif mode == "edev":
            row["rx_errors_s"] = float(tokens[2])
            row["tx_errors_s"] = float(tokens[3])
            row["rx_dropped_s"] = float(tokens[5])
            row["tx_dropped_s"] = float(tokens[6])
            row["rx_fifo_s"] = float(tokens[9])
    return sorted(rows.values(), key=lambda row: row["timestamp"])


def parse_link_counter(path):
    lines = path.read_text().splitlines()
    result = {}
    for index, line in enumerate(lines):
        stripped = line.strip()
        if stripped.startswith("RX:"):
            values = lines[index + 1].split()
            result.update({
                "rx_bytes": int(values[0]), "rx_packets": int(values[1]),
                "rx_errors": int(values[2]), "rx_dropped": int(values[3]),
            })
        elif stripped.startswith("TX:"):
            values = lines[index + 1].split()
            result.update({
                "tx_bytes": int(values[0]), "tx_packets": int(values[1]),
                "tx_errors": int(values[2]), "tx_dropped": int(values[3]),
            })
    return result


def process_summary(rows, stage, process):
    selected = [row for row in rows if row["stage"] == stage and row["process"] == process]
    cpu = stats(row["cpu_pct"] for row in selected if row["cpu_pct"] != "")
    rss = stats(row["rss_kb"] for row in selected if row["rss_kb"] != "")
    vsz = stats(row["vsz_kb"] for row in selected if row["vsz_kb"] != "")
    times = np.asarray([row["timestamp"] for row in selected if row["rss_kb"] != ""], dtype=float)
    rss_values = np.asarray([row["rss_kb"] for row in selected if row["rss_kb"] != ""], dtype=float)
    slope_kb_min = float(np.polyfit(times - times[0], rss_values, 1)[0] * 60.0)
    return {"cpu_pct": cpu, "rss_kb": rss, "vsz_kb": vsz, "rss_slope_kb_per_min": slope_kb_min}


def main():
    OUTPUT.mkdir(parents=True, exist_ok=True)

    system_rows = []
    with (A_DIR / "system.csv").open() as handle:
        for row in csv.DictReader(handle):
            used = float(row["ram_used_kb"])
            system_rows.append({
                "stage": "A", "timestamp": float(row["timestamp"]), "clock": "",
                "system_cpu_pct": float(row["total_cpu_pct"]),
                "ram_used_kb": used, "ram_available_kb": TOTAL_RAM_KB - used,
                "ram_total_kb": TOTAL_RAM_KB,
                "load_1m": "", "load_5m": "", "load_15m": "",
            })
    for stage, directory in (("B", B_DIR), ("C", C_DIR)):
        memory = parse_sar_memory(directory / "sar_memory.txt")
        for row in parse_mpstat(directory / "mpstat.txt", stage):
            mem = memory.get(row["clock"], {})
            row.update(mem)
            row.update({
                "ram_total_kb": TOTAL_RAM_KB,
                "load_1m": "", "load_5m": "", "load_15m": "",
            })
            system_rows.append(row)
    write_csv(OUTPUT / "system_resource.csv", system_rows, [
        "stage", "timestamp", "clock", "system_cpu_pct", "ram_used_kb",
        "ram_available_kb", "ram_total_kb", "load_1m", "load_5m", "load_15m",
    ])

    process_rows = []
    process_rows.extend(parse_pidstat(
        B_DIR / "pidstat_driver.txt", "B", {1160570: "driver"}
    ))
    process_rows.extend(parse_pidstat(
        C_DIR / "pidstat_processes.txt", "C", {
            1160570: "driver", 1168637: "robot_state_publisher",
            1168785: "frame_relay", 1168996: "fusion",
        }
    ))
    write_csv(OUTPUT / "process_resource.csv", process_rows, [
        "stage", "timestamp", "clock", "process", "pid", "cpu_pct", "rss_kb", "vsz_kb",
    ])

    network_rows = []
    with (A_DIR / "network.csv").open() as handle:
        for row in csv.DictReader(handle):
            if row["interface"] != "eno1":
                continue
            network_rows.append({
                "stage": "A", "timestamp": float(row["timestamp"]), "clock": "",
                "interface": "eno1",
                "rx_mb_s": float(row["rx_bytes_per_sec"]) / 1e6,
                "tx_mb_s": float(row["tx_bytes_per_sec"]) / 1e6,
                "rx_packets_s": "", "tx_packets_s": "", "rx_errors_s": "",
                "tx_errors_s": "", "rx_dropped_s": "", "tx_dropped_s": "", "rx_fifo_s": "",
            })
    network_rows.extend(parse_sar_network(B_DIR / "sar_network.txt", "B"))
    network_rows.extend(parse_sar_network(C_DIR / "sar_network.txt", "C"))
    write_csv(OUTPUT / "network_resource.csv", network_rows, [
        "stage", "timestamp", "clock", "interface", "rx_mb_s", "tx_mb_s",
        "rx_packets_s", "tx_packets_s", "rx_errors_s", "tx_errors_s",
        "rx_dropped_s", "tx_dropped_s", "rx_fifo_s",
    ])

    topic_rows = [
        {"stage": "B", "topic": "/livox/lidar_192_168_1_116", "messages": "~100",
         "duration_sec": 10.0, "hz": 10.000299, "points_per_message": 20000.3168,
         "points_per_sec": 200009.16, "payload_mb_s": 5.200238, "point_step": 26,
         "measurement": "direct PointCloud2 subscription"},
        {"stage": "B", "topic": "/livox/lidar_192_168_1_139", "messages": "~100",
         "duration_sec": 10.0, "hz": 10.000330, "points_per_message": 20000.64,
         "points_per_sec": 200013.00, "payload_mb_s": 5.200338, "point_step": 26,
         "measurement": "direct PointCloud2 subscription"},
        {"stage": "C", "topic": "/mid360/merged", "messages": 2949,
         "duration_sec": 300.010635031, "hz": 2949 / 300.010635031,
         "points_per_message": 23899.83384198, "points_per_sec": 234927.03847888,
         "payload_mb_s": 6.10810300045, "point_step": 26,
         "measurement": "formal fusion_metrics window"},
    ]
    write_csv(OUTPUT / "topic_rate.csv", topic_rows, [
        "stage", "topic", "messages", "duration_sec", "hz", "points_per_message",
        "points_per_sec", "payload_mb_s", "point_step", "measurement",
    ])

    formal_start = int((C_DIR / "formal_start_ns.txt").read_text())
    formal_end = int((C_DIR / "formal_end_ns.txt").read_text())
    with (C_DIR / "fusion_metrics.csv").open() as handle:
        all_metrics = list(csv.DictReader(handle))
    metrics = [row for row in all_metrics if formal_start <= int(row["stamp_ns"]) <= formal_end]
    callback_rows = []
    for key, label in (
        ("front_crop_ms", "front callback crop work proxy"),
        ("rear_crop_ms", "rear callback crop work proxy"),
        ("front_tf_ms", "front TF"), ("rear_tf_ms", "rear TF"),
        ("merge_ms", "merge"), ("total_ms", "fusion total pair processing"),
    ):
        summary = stats(float(row[key]) for row in metrics)
        callback_rows.append({
            "window": "C formal", "metric": label, "samples": summary["count"],
            "mean_ms": summary["mean"], "median_ms": summary["median"],
            "p95_ms": summary["p95"], "p99_ms": summary["p99"], "max_ms": summary["max"],
            "notes": "in-node processing metric",
        })
    sync_summary = json.loads((SYNC_DIR / "sync_only_summary.json").read_text())
    for source_key, label in (
        ("absolute_arrival_delta_ms", "front/rear callback-enter absolute delta"),
        ("absolute_header_delta_ms", "front/rear header absolute delta"),
    ):
        summary = sync_summary[source_key]
        callback_rows.append({
            "window": "separate 300 s lightweight timing run", "metric": label,
            "samples": summary["samples"], "mean_ms": summary["mean"],
            "median_ms": summary["p50"], "p95_ms": summary["p95"],
            "p99_ms": summary["p99"], "max_ms": summary["max"],
            "notes": "external callback entry; not scheduler tracing",
        })
    for sensor in ("front", "rear"):
        summary = sync_summary["streams"][sensor]["arrival_interval_ms"]
        callback_rows.append({
            "window": "separate 300 s lightweight timing run",
            "metric": f"{sensor} callback-enter inter-arrival",
            "samples": summary["samples"], "mean_ms": summary["mean"],
            "median_ms": summary["p50"], "p95_ms": summary["p95"],
            "p99_ms": summary["p99"], "max_ms": summary["max"],
            "notes": "external monotonic timestamp",
        })
    write_csv(OUTPUT / "callback_timing.csv", callback_rows, [
        "window", "metric", "samples", "mean_ms", "median_ms", "p95_ms",
        "p99_ms", "max_ms", "notes",
    ])

    c_system = [row for row in system_rows if row["stage"] == "C"]
    c_network = [row for row in network_rows if row["stage"] == "C"]
    c_process = [row for row in process_rows if row["stage"] == "C"]
    before = [row for row in all_metrics if int(row["stamp_ns"]) < formal_start][-1]
    previous_miss = int(before["sync_miss"])
    orphan_rows = []
    for metric in metrics:
        current_miss = int(metric["sync_miss"])
        delta = current_miss - previous_miss
        previous_miss = current_miss
        if delta <= 0:
            continue
        event_time = int(metric["stamp_ns"]) / 1e9
        system_window = [row for row in c_system if abs(row["timestamp"] - event_time) <= 2.0]
        network_window = [row for row in c_network if abs(row["timestamp"] - event_time) <= 2.0]
        metric_window = [row for row in metrics if abs(int(row["stamp_ns"]) / 1e9 - event_time) <= 2.0]
        process_windows = {
            name: [row for row in c_process if row["process"] == name and abs(row["timestamp"] - event_time) <= 2.0]
            for name in ("driver", "fusion", "frame_relay")
        }
        def mean_or_blank(rows, key):
            values = [float(row[key]) for row in rows if row[key] != ""]
            return statistics.fmean(values) if values else ""
        def max_or_blank(rows, key):
            values = [float(row[key]) for row in rows if row[key] != ""]
            return max(values) if values else ""
        orphan_rows.append({
            "event_timestamp": event_time,
            "event_iso": datetime.fromtimestamp(event_time, TZ).isoformat(),
            "miss_count_delta": delta,
            "system_cpu_mean_pct_pm2s": mean_or_blank(system_window, "system_cpu_pct"),
            "system_cpu_max_pct_pm2s": max_or_blank(system_window, "system_cpu_pct"),
            "driver_cpu_mean_pct_pm2s": mean_or_blank(process_windows["driver"], "cpu_pct"),
            "driver_cpu_max_pct_pm2s": max_or_blank(process_windows["driver"], "cpu_pct"),
            "fusion_cpu_mean_pct_pm2s": mean_or_blank(process_windows["fusion"], "cpu_pct"),
            "fusion_cpu_max_pct_pm2s": max_or_blank(process_windows["fusion"], "cpu_pct"),
            "relay_cpu_mean_pct_pm2s": mean_or_blank(process_windows["frame_relay"], "cpu_pct"),
            "ram_used_mean_kb_pm2s": mean_or_blank(system_window, "ram_used_kb"),
            "rx_mb_s_mean_pm2s": mean_or_blank(network_window, "rx_mb_s"),
            "rx_mb_s_max_pm2s": max_or_blank(network_window, "rx_mb_s"),
            "rx_drop_rate_max_pm2s": max_or_blank(network_window, "rx_dropped_s"),
            "next_success_total_ms": float(metric["total_ms"]),
            "callback_total_p95_ms_pm2s": percentile([float(row["total_ms"]) for row in metric_window], 95),
            "callback_total_max_ms_pm2s": max(float(row["total_ms"]) for row in metric_window),
        })
    write_csv(OUTPUT / "orphan_correlation.csv", orphan_rows, [
        "event_timestamp", "event_iso", "miss_count_delta",
        "system_cpu_mean_pct_pm2s", "system_cpu_max_pct_pm2s",
        "driver_cpu_mean_pct_pm2s", "driver_cpu_max_pct_pm2s",
        "fusion_cpu_mean_pct_pm2s", "fusion_cpu_max_pct_pm2s",
        "relay_cpu_mean_pct_pm2s", "ram_used_mean_kb_pm2s",
        "rx_mb_s_mean_pm2s", "rx_mb_s_max_pm2s", "rx_drop_rate_max_pm2s",
        "next_success_total_ms", "callback_total_p95_ms_pm2s", "callback_total_max_ms_pm2s",
    ])

    stage_summary = {}
    for stage in ("A", "B", "C"):
        selected_system = [row for row in system_rows if row["stage"] == stage]
        selected_network = [row for row in network_rows if row["stage"] == stage]
        stage_summary[stage] = {
            "system_cpu_pct": stats(row["system_cpu_pct"] for row in selected_system),
            "ram_used_kb": stats(row["ram_used_kb"] for row in selected_system),
            "ram_available_kb": stats(row["ram_available_kb"] for row in selected_system),
            "eno1_rx_mb_s": stats(row["rx_mb_s"] for row in selected_network),
            "eno1_tx_mb_s": stats(row["tx_mb_s"] for row in selected_network),
        }
    processes = {
        "B_driver": process_summary(process_rows, "B", "driver"),
        "C_driver": process_summary(process_rows, "C", "driver"),
        "C_fusion": process_summary(process_rows, "C", "fusion"),
        "C_frame_relay": process_summary(process_rows, "C", "frame_relay"),
        "C_robot_state_publisher": process_summary(process_rows, "C", "robot_state_publisher"),
    }
    counter_deltas = {}
    for stage, directory in (("B", B_DIR), ("C", C_DIR)):
        start = parse_link_counter(directory / "eno1_start.txt")
        end = parse_link_counter(directory / "eno1_end.txt")
        counter_deltas[stage] = {key: end[key] - start[key] for key in start}
    summary = {
        "stages": stage_summary,
        "processes": processes,
        "link_counter_deltas": counter_deltas,
        "orphans": {
            "event_rows": len(orphan_rows),
            "miss_total": sum(row["miss_count_delta"] for row in orphan_rows),
        },
        "limitations": {
            "load_average": "not captured in the historical A/B/C sampler",
            "udp_kernel_counter_delta": "not captured at historical start/end",
            "rx_missed_errors": "not present in historical ip -s snapshots",
        },
    }
    (OUTPUT / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(OUTPUT)


if __name__ == "__main__":
    main()
