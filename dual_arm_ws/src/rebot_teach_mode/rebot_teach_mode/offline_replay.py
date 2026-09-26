"""Offline Piper-H replay preparation. This module never opens a ROS or CAN connection."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from scipy.interpolate import PchipInterpolator
import yaml


JOINTS = tuple(f"joint{i}" for i in range(1, 7))


def load_limits(path: Path) -> tuple[np.ndarray, np.ndarray]:
    config = yaml.safe_load(path.read_text(encoding="utf-8"))["joint_limits"]
    velocity = np.array([config[name]["max_velocity"] for name in JOINTS], dtype=float)
    acceleration = np.array([config[name]["max_acceleration"] for name in JOINTS], dtype=float)
    if not (np.isfinite(velocity).all() and np.isfinite(acceleration).all()
            and (velocity > 0).all() and (acceleration > 0).all()):
        raise ValueError("all six velocity and acceleration limits must be positive")
    if any(not config[name].get("has_velocity_limits") or
           not config[name].get("has_acceleration_limits") for name in JOINTS):
        raise ValueError("all six joints must have velocity and acceleration limits")
    return velocity, acceleration


def read_raw(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray | None]:
    document = json.loads(path.read_text(encoding="utf-8"))
    if tuple(document["joint_names"]) != JOINTS:
        raise ValueError("expected ordered Piper-H joint1..joint6 feedback")
    points = document["points"]
    if len(points) < 2:
        raise ValueError("at least two raw points are required")
    t = np.asarray([point["time_from_start"] for point in points], dtype=float)
    q = np.asarray([point["positions"] for point in points], dtype=float)
    if q.shape != (len(t), 6) or not np.isfinite(q).all():
        raise ValueError("raw trajectory must contain six finite angles per point")
    if not np.isfinite(t).all() or abs(t[0]) > 1e-9 or (np.diff(t) <= 0).any():
        raise ValueError("raw relative monotonic timestamps must start at zero and increase")
    if (np.abs(np.diff(q, axis=0)) > math.pi).any():
        raise ValueError("adjacent joint angles cross pi; inspect unwrap before replay")
    stamps = [point.get("feedback_timestamp") for point in points]
    feedback = None
    if all(stamp is not None for stamp in stamps):
        feedback = np.asarray(stamps, dtype=float)
        if not np.isfinite(feedback).all() or (np.diff(feedback) <= 0).any():
            raise ValueError("feedback timestamps must strictly increase")
    return t, q, feedback


def derivative_peaks(interpolator: PchipInterpolator, times: np.ndarray) -> dict:
    """Find extrema analytically within every PCHIP segment, not on a 100 Hz grid."""
    c = interpolator.c  # cubic, quadratic, linear, constant coefficients
    h = np.diff(times)[:, None]
    v0 = c[2]
    v1 = 3 * c[0] * h * h + 2 * c[1] * h + c[2]
    vertex = np.divide(-c[1], 3 * c[0], out=np.full_like(c[0], -1.0),
                       where=np.abs(c[0]) > 1e-15)
    vertex = np.where((vertex > 0) & (vertex < h), vertex, -1.0)
    vv = np.where(vertex > 0, 3 * c[0] * vertex * vertex + 2 * c[1] * vertex + c[2], 0.0)
    a0 = 2 * c[1]
    a1 = 6 * c[0] * h + 2 * c[1]
    max_v, max_a, time_v, time_a = [], [], [], []
    for j in range(6):
        candidates_v = np.stack((np.abs(v0[:, j]), np.abs(v1[:, j]), np.abs(vv[:, j])))
        branch, segment = np.unravel_index(np.argmax(candidates_v), candidates_v.shape)
        max_v.append(float(candidates_v[branch, segment]))
        offset = 0.0 if branch == 0 else float(h[segment, 0]) if branch == 1 else float(vertex[segment, j])
        time_v.append(float(times[segment] + offset))
        candidates_a = np.stack((np.abs(a0[:, j]), np.abs(a1[:, j])))
        branch, segment = np.unravel_index(np.argmax(candidates_a), candidates_a.shape)
        max_a.append(float(candidates_a[branch, segment]))
        time_a.append(float(times[segment + branch]))
    return {"max_velocity_rad_s": max_v, "max_velocity_time": time_v,
            "max_acceleration_rad_s2": max_a, "max_acceleration_time": time_a}


def prepare(raw_path: Path, limits_path: Path, output_dir: Path,
            rate_hz: float = 100.0) -> dict:
    """Preserve raw JSON; write independently retimed PCHIP samples and diagnostics."""
    if not math.isfinite(rate_hz) or rate_hz <= 0:
        raise ValueError("replay rate must be positive")
    raw_sha = hashlib.sha256(raw_path.read_bytes()).hexdigest()
    t, q, feedback = read_raw(raw_path)
    velocity_limit, acceleration_limit = load_limits(limits_path)
    spline = PchipInterpolator(t, q, axis=0, extrapolate=False)
    peaks = derivative_peaks(spline, t)
    scale = max(1.0, float(np.max(np.asarray(peaks["max_velocity_rad_s"]) / velocity_limit)),
                float(np.sqrt(np.max(np.asarray(peaks["max_acceleration_rad_s2"]) / acceleration_limit))))
    scale *= 1.001  # small numerical margin; one scale for all six joints
    replay_duration = float(t[-1] * scale)
    sample_count = math.ceil(replay_duration * rate_hz) + 1
    if sample_count > 5_000_000:
        raise ValueError(f"replay needs {sample_count} samples; inspect timestamp artifacts first")
    gaps = []
    dt = np.diff(t)
    for index in np.flatnonzero(dt > 0.030):
        delta = q[index + 1] - q[index]
        entry = {"start_time": float(t[index]), "gap_duration": float(dt[index]),
                 "joint_delta_across_gap": delta.tolist(),
                 "warning": bool(dt[index] > 0.100 and np.max(np.abs(delta)) > 0.05)}
        gaps.append(entry)
    frame_delta = np.diff(feedback) if feedback is not None else None
    metadata = {
        "format": "piperh_offline_replay_v1", "source_file": str(raw_path.resolve()),
        "source_sha256": raw_sha, "time_source": "raw_relative_monotonic_arrival",
        "interpolation": "PCHIP_no_smoothing", "raw_duration": float(t[-1]),
        "replay_duration": replay_duration, "raw_samples": len(t),
        "replay_samples": sample_count, "replay_rate_hz": rate_hz, "time_scale": scale,
        "limits_source": str(limits_path.resolve()),
        "limits_status": "project_MoveIt_configuration_not_verified_as_Piper_H_physical_capability",
        "hardware_execution_approved": False,
        "velocity_limit_rad_s": velocity_limit.tolist(),
        "acceleration_limit_rad_s2": acceleration_limit.tolist(),
        **peaks,
        "replay_max_velocity_rad_s":
            (np.asarray(peaks["max_velocity_rad_s"]) / scale).tolist(),
        "replay_max_acceleration_rad_s2":
            (np.asarray(peaks["max_acceleration_rad_s2"]) / scale**2).tolist(),
        "large_gap_detected": bool(gaps), "large_timestamp_gaps": gaps,
        "gap_counts": {"over_30_ms": int(np.sum(dt > .030)),
                       "over_50_ms": int(np.sum(dt > .050)),
                       "over_100_ms": int(np.sum(dt > .100))},
        "max_gap": gaps[int(np.argmax([g["gap_duration"] for g in gaps]))] if gaps else None,
        "feedback_arrival_interval_mismatch_over_10_ms":
            int(np.sum(np.abs(frame_delta - dt) > .010)) if frame_delta is not None else None,
        "warnings": [],
    }
    if any(g["warning"] for g in gaps):
        metadata["warnings"].append("轨迹部分大采样空档两侧关节变化较大，缺少足够真实测量，无法保证精确复现。")
    if frame_delta is not None and metadata["feedback_arrival_interval_mismatch_over_10_ms"]:
        metadata["warnings"].append("相对 monotonic 时间为 ROS 到达时间；部分间隔与反馈帧时间不符，速度和加速度峰值可能被到达抖动放大。")
    if scale > 10:
        metadata["warnings"].append("统一时间缩放超过 10 倍；该离线结果仅供审查，不应直接用于真机回放。")
    output_dir.mkdir(parents=True, exist_ok=True)
    array_path = output_dir / "replay_100hz.npy"
    dtype = [("replay_time", "<f8")] + [(f"j{i}", "<f8") for i in range(1, 7)]
    samples = np.lib.format.open_memmap(array_path, mode="w+", dtype=dtype,
                                        shape=(sample_count,))
    for start in range(0, sample_count, 100_000):
        stop = min(start + 100_000, sample_count)
        replay_time = np.minimum(np.arange(start, stop, dtype=float) / rate_hz,
                                 replay_duration)
        positions = spline(replay_time / scale)
        samples["replay_time"][start:stop] = replay_time
        for j in range(6):
            samples[f"j{j+1}"][start:stop] = positions[:, j]
    samples.flush()
    del samples
    # The spline itself passes exactly through each measured sample. Also check
    # the nearest fixed-rate replay sample after undoing the common time scale.
    target_index = np.clip(np.rint(t * scale * rate_hz).astype(int), 0, sample_count - 1)
    saved = np.load(array_path, mmap_mode="r")
    near = np.column_stack([saved[f"j{j}"][target_index] for j in range(1, 7)])
    metadata["max_nearest_grid_error_rad"] = np.max(np.abs(near - q), axis=0).tolist()
    metadata["max_interpolant_knot_error_rad"] = float(np.max(np.abs(spline(t) - q)))
    if hashlib.sha256(raw_path.read_bytes()).hexdigest() != raw_sha:
        raise RuntimeError("raw trajectory changed during offline preparation")
    (output_dir / "metadata.json").write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return metadata


def plot_comparison(raw_path: Path, output_dir: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t, q, _ = read_raw(raw_path)
    metadata = json.loads((output_dir / "metadata.json").read_text(encoding="utf-8"))
    replay = np.load(output_dir / "replay_100hz.npy", mmap_mode="r")
    step = max(1, len(replay) // 8000)
    indices = np.arange(0, len(replay), step)
    fig, axes = plt.subplots(6, 1, figsize=(12, 12), sharex=True)
    for j, ax in enumerate(axes, 1):
        ax.plot(replay["replay_time"][indices] / metadata["time_scale"],
                replay[f"j{j}"][indices], lw=1, label="replay (time aligned)")
        ax.plot(t, q[:, j - 1], ".", ms=1, alpha=.4, label="raw feedback")
        ax.set_ylabel(f"J{j} (rad)")
        ax.grid(alpha=.25)
        if j == 1:
            ax.legend()
    axes[-1].set_xlabel("raw relative monotonic time (s)")
    fig.tight_layout()
    fig.savefig(output_dir / "raw_vs_replay.png", dpi=130)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("raw", type=Path)
    parser.add_argument("--limits", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--rate", type=float, default=100.0)
    args = parser.parse_args()
    report = prepare(args.raw, args.limits, args.output_dir, args.rate)
    plot_comparison(args.raw, args.output_dir)
    print(json.dumps({key: report[key] for key in
          ("raw_duration", "raw_samples", "replay_duration", "replay_samples",
           "replay_rate_hz", "time_scale", "gap_counts", "max_gap",
           "max_velocity_rad_s", "max_acceleration_rad_s2", "warnings")},
          ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
