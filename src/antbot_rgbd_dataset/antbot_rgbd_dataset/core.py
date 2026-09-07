"""ROS-independent synchronization, selection, transforms, and atomic dataset I/O."""

from __future__ import annotations

from collections import OrderedDict
import csv
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import shutil
import tempfile
from typing import Any, Callable

import cv2
import numpy as np
import yaml


STREAMS = ("rgb", "depth", "color_info", "depth_info")
CSV_FIELDS = (
    "frame_index",
    "timestamp_sec",
    "timestamp_nanosec",
    "timestamp_ns",
    "rgb_path",
    "depth_npy_path",
    "depth_png_path",
    "pose_path",
    "camera_path",
    "sharpness",
    "valid_depth_ratio",
    "min_depth_m",
    "max_depth_m",
    "mean_depth_m",
    "valid_depth_pixels",
    "zero_depth_pixels",
    "nan_depth_pixels",
    "inf_depth_pixels",
    "translation_from_previous_m",
    "rotation_from_previous_deg",
    "selection_reason",
)


def stamp_ns(message: Any) -> int:
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class ExactStampSynchronizer:
    """Four-stream exact synchronizer with bounded per-stream caches."""

    def __init__(self, capacity: int = 30):
        if capacity < 1:
            raise ValueError("capacity must be positive")
        self.capacity = capacity
        self._cache = {name: OrderedDict() for name in STREAMS}
        self.sync_drop_count = 0
        self.duplicate_input_count = 0
        self.timestamp_regression_count = 0
        self._last_stamp = {name: None for name in STREAMS}

    def add(self, stream: str, message: Any) -> tuple[Any, Any, Any, Any] | None:
        if stream not in self._cache:
            raise KeyError(stream)
        timestamp = stamp_ns(message)
        previous = self._last_stamp[stream]
        if previous is not None:
            if timestamp < previous:
                self.timestamp_regression_count += 1
            elif timestamp == previous:
                self.duplicate_input_count += 1
        self._last_stamp[stream] = timestamp
        cache = self._cache[stream]
        if timestamp in cache:
            self.duplicate_input_count += 1
        cache[timestamp] = message
        while len(cache) > self.capacity:
            cache.popitem(last=False)
            self.sync_drop_count += 1
        if not all(timestamp in self._cache[name] for name in STREAMS):
            return None
        bundle = tuple(self._cache[name].pop(timestamp) for name in STREAMS)
        self._prune_older_than(timestamp)
        return bundle

    def _prune_older_than(self, timestamp: int) -> None:
        for cache in self._cache.values():
            stale = [key for key in cache if key < timestamp]
            self.sync_drop_count += len(stale)
            for key in stale:
                del cache[key]

    def flush_unmatched(self) -> None:
        self.sync_drop_count += sum(len(cache) for cache in self._cache.values())
        for cache in self._cache.values():
            cache.clear()


def quaternion_to_matrix(quaternion_xyzw: np.ndarray) -> np.ndarray:
    q = np.asarray(quaternion_xyzw, dtype=np.float64)
    norm = float(np.linalg.norm(q))
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("invalid quaternion")
    x, y, z, w = q / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def transform_matrix(translation_xyz: np.ndarray, quaternion_xyzw: np.ndarray) -> np.ndarray:
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = quaternion_to_matrix(quaternion_xyzw)
    matrix[:3, 3] = np.asarray(translation_xyz, dtype=np.float64)
    return matrix


def pose_delta(previous: np.ndarray, current: np.ndarray) -> tuple[float, float]:
    translation = float(np.linalg.norm(current[:3, 3] - previous[:3, 3]))
    delta_rotation = previous[:3, :3].T @ current[:3, :3]
    cosine = float(np.clip((np.trace(delta_rotation) - 1.0) * 0.5, -1.0, 1.0))
    return translation, math.degrees(math.acos(cosine))


def laplacian_variance(rgb: np.ndarray) -> float:
    if rgb.ndim == 2:
        gray = rgb
    else:
        gray = cv2.cvtColor(rgb[:, :, :3], cv2.COLOR_RGB2GRAY)
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


@dataclass(frozen=True)
class DepthStatistics:
    valid_pixels: int
    total_pixels: int
    valid_ratio: float
    minimum_m: float | None
    maximum_m: float | None
    mean_m: float | None
    zero_count: int
    nan_count: int
    inf_count: int


def depth_statistics(depth_m: np.ndarray, minimum_m: float, maximum_m: float) -> DepthStatistics:
    depth = np.asarray(depth_m)
    finite = np.isfinite(depth)
    valid = finite & (depth >= minimum_m) & (depth <= maximum_m)
    values = depth[valid]
    return DepthStatistics(
        valid_pixels=int(values.size),
        total_pixels=int(depth.size),
        valid_ratio=float(values.size / depth.size) if depth.size else 0.0,
        minimum_m=float(values.min()) if values.size else None,
        maximum_m=float(values.max()) if values.size else None,
        mean_m=float(values.mean(dtype=np.float64)) if values.size else None,
        zero_count=int(np.count_nonzero(depth == 0)),
        nan_count=int(np.count_nonzero(np.isnan(depth))),
        inf_count=int(np.count_nonzero(np.isinf(depth))),
    )


@dataclass(frozen=True)
class KeyframePolicy:
    translation_threshold_m: float = 0.15
    rotation_threshold_deg: float = 8.0
    minimum_interval_sec: float = 0.30
    maximum_interval_sec: float = 2.0
    stationary_translation_epsilon_m: float = 0.005
    stationary_rotation_epsilon_deg: float = 0.2
    enable_blur_filter: bool = True
    minimum_laplacian_variance: float = 80.0
    minimum_valid_depth_ratio: float = 0.30


@dataclass(frozen=True)
class Selection:
    selected: bool
    reasons: tuple[str, ...]
    translation_m: float
    rotation_deg: float
    rejection: str | None = None


def select_keyframe(
    policy: KeyframePolicy,
    timestamp_ns: int,
    pose: np.ndarray,
    sharpness: float,
    valid_depth_ratio: float,
    previous_timestamp_ns: int | None,
    previous_pose: np.ndarray | None,
    manual: bool = False,
) -> Selection:
    if valid_depth_ratio < policy.minimum_valid_depth_ratio:
        return Selection(False, (), 0.0, 0.0, "DEPTH_QUALITY")
    if policy.enable_blur_filter and sharpness < policy.minimum_laplacian_variance:
        return Selection(False, (), 0.0, 0.0, "BLUR")
    if previous_pose is None or previous_timestamp_ns is None:
        return Selection(True, ("MANUAL",) if manual else ("FIRST_FRAME",), 0.0, 0.0)
    translation, rotation = pose_delta(previous_pose, pose)
    interval = (timestamp_ns - previous_timestamp_ns) / 1e9
    if timestamp_ns <= previous_timestamp_ns:
        return Selection(False, (), translation, rotation, "NON_MONOTONIC_TIMESTAMP")
    if manual:
        return Selection(True, ("MANUAL",), translation, rotation)
    if interval < policy.minimum_interval_sec:
        return Selection(False, (), translation, rotation, "MINIMUM_INTERVAL")
    reasons = []
    if translation >= policy.translation_threshold_m:
        reasons.append("TRANSLATION")
    if rotation >= policy.rotation_threshold_deg:
        reasons.append("ROTATION")
    moving = (
        translation >= policy.stationary_translation_epsilon_m
        or rotation >= policy.stationary_rotation_epsilon_deg
    )
    if interval >= policy.maximum_interval_sec and moving:
        reasons.append("MAX_INTERVAL")
    return Selection(bool(reasons), tuple(reasons), translation, rotation, None if reasons else "MOTION")


def canonical_camera_info(message: Any) -> dict[str, Any]:
    roi = message.roi
    return {
        "width": int(message.width),
        "height": int(message.height),
        "distortion_model": str(message.distortion_model),
        "D": [float(value) for value in message.d],
        "K": [float(value) for value in message.k],
        "R": [float(value) for value in message.r],
        "P": [float(value) for value in message.p],
        "binning_x": int(message.binning_x),
        "binning_y": int(message.binning_y),
        "roi": {
            "x_offset": int(roi.x_offset),
            "y_offset": int(roi.y_offset),
            "height": int(roi.height),
            "width": int(roi.width),
            "do_rectify": bool(roi.do_rectify),
        },
        "frame_id": str(message.header.frame_id),
    }


def camera_info_equal(left: dict[str, Any], right: dict[str, Any], atol: float = 1e-12) -> bool:
    for key in ("width", "height", "distortion_model", "binning_x", "binning_y", "roi", "frame_id"):
        if left[key] != right[key]:
            return False
    return all(np.allclose(left[key], right[key], rtol=0.0, atol=atol) for key in ("D", "K", "R", "P"))


def _atomic_write(path: Path, writer: Callable[[Any], None], mode: str = "wb") -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary_name = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(fd, mode) as handle:
            writer(handle)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary_name, path)
    except Exception:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise


def atomic_json(path: Path, value: Any) -> None:
    _atomic_write(path, lambda handle: json.dump(value, handle, indent=2, ensure_ascii=False), "w")


def atomic_yaml(path: Path, value: Any) -> None:
    _atomic_write(path, lambda handle: yaml.safe_dump(value, handle, sort_keys=False), "w")


def atomic_npy(path: Path, array: np.ndarray) -> None:
    _atomic_write(path, lambda handle: np.save(handle, array, allow_pickle=False))


def atomic_png(path: Path, array: np.ndarray) -> None:
    success, encoded = cv2.imencode(".png", array)
    if not success:
        raise IOError(f"failed to encode PNG: {path}")
    _atomic_write(path, lambda handle: handle.write(encoded.tobytes()))


class DatasetWriter:
    """Crash-consistent keyframe writer; CSV is committed after all frame assets."""

    def __init__(
        self,
        output_root: Path,
        dataset_name: str,
        save_depth_npy: bool,
        save_depth_png: bool,
        overwrite_existing: bool = False,
    ):
        self.root = Path(output_root) / dataset_name
        if self.root.exists() and any(self.root.iterdir()) and not overwrite_existing:
            raise FileExistsError(f"dataset already exists and is not empty: {self.root}")
        if self.root.exists() and overwrite_existing:
            shutil.rmtree(self.root)
        for directory in (
            "rgb", "depth", "depth_png", "poses", "camera", "frame_metadata", "previews"
        ):
            (self.root / directory).mkdir(parents=True, exist_ok=True)
        self.save_depth_npy = save_depth_npy
        self.save_depth_png = save_depth_png
        self.rows: list[dict[str, Any]] = []
        self.trajectory: list[dict[str, Any]] = []
        self._initialize_csv()

    def _initialize_csv(self) -> None:
        _atomic_write(
            self.root / "frames.csv",
            lambda handle: csv.DictWriter(handle, fieldnames=CSV_FIELDS).writeheader(),
            "w",
        )

    def write_frame(
        self,
        timestamp_ns: int,
        rgb: np.ndarray,
        depth_m: np.ndarray,
        pose: dict[str, Any],
        camera: dict[str, Any],
        stats: DepthStatistics,
        sharpness: float,
        selection: Selection,
        frame_metadata: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        index = len(self.rows)
        stem = f"{index:06d}"
        relative = {
            "rgb_path": f"rgb/{stem}.png",
            "depth_npy_path": f"depth/{stem}.npy" if self.save_depth_npy else "",
            "depth_png_path": f"depth_png/{stem}.png" if self.save_depth_png else "",
            "pose_path": f"poses/{stem}.json",
            "camera_path": f"camera/{stem}.json",
        }
        targets = [self.root / value for value in relative.values() if value]
        frame_metadata_path = self.root / "frame_metadata" / f"{stem}.yaml"
        if frame_metadata is not None:
            targets.append(frame_metadata_path)
        try:
            atomic_png(self.root / relative["rgb_path"], cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
            if self.save_depth_npy:
                atomic_npy(self.root / relative["depth_npy_path"], depth_m.astype(np.float32, copy=False))
            if self.save_depth_png:
                valid = np.isfinite(depth_m) & (depth_m > 0.0) & (depth_m <= 65.535)
                depth_mm = np.zeros(depth_m.shape, dtype=np.uint16)
                depth_mm[valid] = np.rint(depth_m[valid] * 1000.0).astype(np.uint16)
                atomic_png(self.root / relative["depth_png_path"], depth_mm)
            atomic_json(self.root / relative["pose_path"], pose)
            atomic_json(self.root / relative["camera_path"], camera)
            if frame_metadata is not None:
                atomic_yaml(frame_metadata_path, frame_metadata)
        except Exception:
            for target in targets:
                target.unlink(missing_ok=True)
            raise
        sec, nanosec = divmod(timestamp_ns, 1_000_000_000)
        row = {
            "frame_index": index,
            "timestamp_sec": sec,
            "timestamp_nanosec": nanosec,
            "timestamp_ns": timestamp_ns,
            **relative,
            "sharpness": sharpness,
            "valid_depth_ratio": stats.valid_ratio,
            "min_depth_m": stats.minimum_m,
            "max_depth_m": stats.maximum_m,
            "mean_depth_m": stats.mean_m,
            "valid_depth_pixels": stats.valid_pixels,
            "zero_depth_pixels": stats.zero_count,
            "nan_depth_pixels": stats.nan_count,
            "inf_depth_pixels": stats.inf_count,
            "translation_from_previous_m": selection.translation_m,
            "rotation_from_previous_deg": selection.rotation_deg,
            "selection_reason": "|".join(selection.reasons),
        }
        self.rows.append(row)
        fixed_frame = pose["fixed_frame"]
        dynamic_matrix_key = f"T_{fixed_frame}_camera"
        trajectory_frame = {
            "frame_index": index,
            "timestamp_ns": timestamp_ns,
            "fixed_frame": fixed_frame,
            "camera_frame": pose["camera_frame"],
            "transform_definition": pose["transform_definition"],
            "translation_m": pose["translation_m"],
            "quaternion_xyzw": pose["quaternion_xyzw"],
            "T_fixed_camera": pose[dynamic_matrix_key],
            dynamic_matrix_key: pose[dynamic_matrix_key],
        }
        if fixed_frame == "odom":
            trajectory_frame["T_odom_camera"] = pose[dynamic_matrix_key]
        self.trajectory.append(trajectory_frame)
        self._checkpoint()
        return row

    def _checkpoint(self) -> None:
        _atomic_write(
            self.root / "frames.csv",
            lambda handle: self._write_rows(handle),
            "w",
        )
        atomic_json(
            self.root / "trajectory.json",
            {
                "transform_definition": self.trajectory[0]["transform_definition"]
                if self.trajectory
                else None,
                "frames": self.trajectory,
            },
        )
        atomic_json(
            self.root / "checkpoint.json",
            {
                "frame_count": len(self.rows),
                "last_timestamp_ns": self.rows[-1]["timestamp_ns"] if self.rows else None,
                "complete": False,
            },
        )

    def _write_rows(self, handle: Any) -> None:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
        writer.writeheader()
        writer.writerows(self.rows)

    def finalize(self, metadata: dict[str, Any], complete: bool = True) -> None:
        atomic_yaml(self.root / "dataset_metadata.yaml", metadata)
        atomic_json(
            self.root / "checkpoint.json",
            {
                "frame_count": len(self.rows),
                "last_timestamp_ns": self.rows[-1]["timestamp_ns"] if self.rows else None,
                "complete": complete,
            },
        )


def sharpness_distribution(values: list[float]) -> dict[str, float | None]:
    if not values:
        return {key: None for key in ("minimum", "p10", "median", "p90", "maximum")}
    array = np.asarray(values, dtype=np.float64)
    return {
        "minimum": float(array.min()),
        "p10": float(np.percentile(array, 10)),
        "median": float(np.median(array)),
        "p90": float(np.percentile(array, 90)),
        "maximum": float(array.max()),
    }


def utc_timestamp() -> str:
    return datetime.now(timezone.utc).isoformat()
