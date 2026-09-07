"""Lightweight RGB-D point-cloud and coverage state for the live RViz preview."""

from __future__ import annotations

from collections import Counter, deque
from dataclasses import dataclass
import math
import os
from pathlib import Path
import tempfile

import numpy as np


def save_preview_archive(path, points, colors, frame_id):
    """Atomically save a colored preview cloud for later ROS publication."""
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{destination.name}.", suffix=".tmp", dir=destination.parent
    )
    try:
        with os.fdopen(descriptor, "wb") as stream:
            np.savez_compressed(
                stream,
                points=np.asarray(points, dtype=np.float32),
                colors=np.asarray(colors, dtype=np.uint8),
                frame_id=np.asarray(str(frame_id)),
            )
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, destination)
    except Exception:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise


def load_preview_archive(path):
    """Load and strictly validate a colored preview cloud archive."""
    with np.load(Path(path), allow_pickle=False) as archive:
        points = np.asarray(archive["points"], dtype=np.float32)
        colors = np.asarray(archive["colors"], dtype=np.uint8)
        frame_id = str(np.asarray(archive["frame_id"]).item())
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("preview points must have shape Nx3")
    if colors.shape != points.shape:
        raise ValueError("preview colors must have the same Nx3 shape as points")
    if not frame_id:
        raise ValueError("preview frame_id must not be empty")
    if not np.isfinite(points).all():
        raise ValueError("preview points contain non-finite values")
    return points, colors, frame_id


def project_rgbd(frame, pixel_stride: int, depth_minimum_m: float, depth_maximum_m: float):
    """Project a strided, aligned RGB-D frame into the configured world frame."""
    if pixel_stride < 1:
        raise ValueError("pixel_stride must be at least one")
    depth = frame.depth_m[::pixel_stride, ::pixel_stride]
    color = frame.color[::pixel_stride, ::pixel_stride]
    v, u = np.mgrid[
        0:frame.depth_m.shape[0]:pixel_stride,
        0:frame.depth_m.shape[1]:pixel_stride,
    ]
    valid = (
        np.isfinite(depth)
        & (depth >= depth_minimum_m)
        & (depth <= depth_maximum_m)
    )
    z = depth[valid].astype(np.float64, copy=False)
    intr = frame.camera_intrinsics
    points_camera = np.column_stack((
        (u[valid] - intr.cx) * z / intr.fx,
        (v[valid] - intr.cy) * z / intr.fy,
        z,
    ))
    points_world = (
        points_camera @ frame.T_world_camera[:3, :3].T
        + frame.T_world_camera[:3, 3]
    )
    return points_world.astype(np.float32), color[valid].astype(np.uint8, copy=False)


def depth_continuity_mask(
    depth_m: np.ndarray,
    absolute_threshold_m: float = 0.08,
    relative_threshold: float = 0.03,
) -> np.ndarray:
    """Reject pixels touching a depth jump, where flying points occur."""
    depth = np.asarray(depth_m)
    if depth.ndim != 2:
        raise ValueError("depth image must be two-dimensional")
    if absolute_threshold_m < 0 or relative_threshold < 0:
        raise ValueError("depth discontinuity thresholds must be non-negative")
    valid = np.isfinite(depth) & (depth > 0)
    keep = valid.copy()
    for first, second, first_slice, second_slice in (
        (depth[:, :-1], depth[:, 1:], (slice(None), slice(None, -1)),
         (slice(None), slice(1, None))),
        (depth[:-1, :], depth[1:, :], (slice(None, -1), slice(None)),
         (slice(1, None), slice(None))),
    ):
        pair_valid = np.isfinite(first) & (first > 0) & np.isfinite(second) & (second > 0)
        limit = np.maximum(
            float(absolute_threshold_m),
            float(relative_threshold) * np.minimum(first, second),
        )
        difference = np.zeros_like(first, dtype=np.float64)
        np.subtract(first, second, out=difference, where=pair_valid)
        discontinuity = pair_valid & (np.abs(difference) > limit)
        first_keep = keep[first_slice]
        second_keep = keep[second_slice]
        first_keep[discontinuity] = False
        second_keep[discontinuity] = False
    return keep


def project_rgbd_filtered(
    frame,
    pixel_stride: int,
    depth_minimum_m: float,
    depth_maximum_m: float,
    edge_absolute_threshold_m: float,
    edge_relative_threshold: float,
):
    """Project RGB-D after removing depth-discontinuity edge pixels."""
    if pixel_stride < 1:
        raise ValueError("pixel_stride must be at least one")
    depth = frame.depth_m[::pixel_stride, ::pixel_stride]
    color = frame.color[::pixel_stride, ::pixel_stride]
    v, u = np.mgrid[
        0:frame.depth_m.shape[0]:pixel_stride,
        0:frame.depth_m.shape[1]:pixel_stride,
    ]
    valid = (
        depth_continuity_mask(
            depth, edge_absolute_threshold_m, edge_relative_threshold
        )
        & (depth >= depth_minimum_m)
        & (depth <= depth_maximum_m)
    )
    z = depth[valid].astype(np.float64, copy=False)
    intr = frame.camera_intrinsics
    points_camera = np.column_stack((
        (u[valid] - intr.cx) * z / intr.fx,
        (v[valid] - intr.cy) * z / intr.fy,
        z,
    ))
    points_world = (
        points_camera @ frame.T_world_camera[:3, :3].T
        + frame.T_world_camera[:3, 3]
    )
    return points_world.astype(np.float32), color[valid].astype(np.uint8, copy=False)


def voxel_reduce(points, colors, voxel_size_m: float):
    """Keep one representative point/color per occupied voxel."""
    if voxel_size_m <= 0:
        raise ValueError("voxel_size_m must be positive")
    if not len(points):
        return points, colors
    keys = np.floor(points / voxel_size_m).astype(np.int32)
    _, indices = np.unique(keys, axis=0, return_index=True)
    return points[indices], colors[indices]


def direction_sector(rotation, reference_forward=None):
    """Classify optical +Z into front/left/back/right around the world Z axis."""
    forward = np.asarray(rotation, dtype=np.float64) @ np.array([0.0, 0.0, 1.0])
    horizontal = forward[:2]
    if np.linalg.norm(horizontal) < 1e-6:
        return "front"
    angle = math.atan2(horizontal[1], horizontal[0])
    if reference_forward is not None:
        reference = np.asarray(reference_forward, dtype=np.float64)[:2]
        if np.linalg.norm(reference) >= 1e-6:
            angle -= math.atan2(reference[1], reference[0])
    angle = (angle + math.pi) % (2.0 * math.pi) - math.pi
    if -math.pi / 4 <= angle < math.pi / 4:
        return "front"
    if math.pi / 4 <= angle < 3 * math.pi / 4:
        return "left"
    if -3 * math.pi / 4 <= angle < -math.pi / 4:
        return "right"
    return "back"


@dataclass(frozen=True)
class CoverageSnapshot:
    occupied_voxels: int
    weak_voxels: int
    new_voxels: int
    recent_growth_percent: float
    bounds_m: tuple[float, float, float]
    direction: str
    weak_direction: str
    direction_counts: dict


class LivePreviewAccumulator:
    """Bounded-resolution preview cloud plus coarser observation coverage."""

    def __init__(self, preview_voxel_m=0.05, coverage_voxel_m=0.20, growth_window_s=10.0):
        if preview_voxel_m <= 0 or coverage_voxel_m <= 0 or growth_window_s <= 0:
            raise ValueError("live preview resolutions and growth window must be positive")
        self.preview_voxel_m = float(preview_voxel_m)
        self.coverage_voxel_m = float(coverage_voxel_m)
        self.growth_window_ns = int(float(growth_window_s) * 1e9)
        self.preview = {}
        self.coverage = Counter()
        self.direction_counts = Counter()
        self.growth = deque()
        self.camera_positions = []
        self.reference_forward = None

    def add_keyframe(self, timestamp_ns, points, colors, camera_transform):
        preview_keys = np.floor(points / self.preview_voxel_m).astype(np.int32)
        for key, point, color in zip(preview_keys, points, colors):
            value = tuple(int(v) for v in key)
            if value in self.preview:
                point_sum, color_sum, count = self.preview[value]
                point_sum += point
                color_sum += color
                self.preview[value] = (point_sum, color_sum, count + 1)
            else:
                self.preview[value] = (
                    point.astype(np.float64, copy=True),
                    color.astype(np.float64, copy=True),
                    1,
                )

        coverage_keys = np.floor(points / self.coverage_voxel_m).astype(np.int32)
        unique_keys = np.unique(coverage_keys, axis=0)
        new_voxels = 0
        for key in unique_keys:
            value = tuple(int(v) for v in key)
            if not self.coverage[value]:
                new_voxels += 1
            self.coverage[value] += 1

        rotation = np.asarray(camera_transform)[:3, :3]
        forward = rotation @ np.array([0.0, 0.0, 1.0])
        if self.reference_forward is None and np.linalg.norm(forward[:2]) >= 1e-6:
            self.reference_forward = forward
        sector = direction_sector(rotation, self.reference_forward)
        self.direction_counts[sector] += 1
        self.camera_positions.append(np.asarray(camera_transform)[:3, 3].copy())

        self.growth.append((int(timestamp_ns), new_voxels))
        cutoff = int(timestamp_ns) - self.growth_window_ns
        while self.growth and self.growth[0][0] < cutoff:
            self.growth.popleft()
        return self.snapshot(new_voxels, sector)

    def snapshot(self, new_voxels=0, direction="front"):
        occupied = len(self.coverage)
        recent_new = sum(value for _, value in self.growth)
        growth_percent = 100.0 * recent_new / max(occupied, 1)
        if self.preview:
            points = np.asarray([
                value[0] / value[2] for value in self.preview.values()
            ])
            bounds = tuple(float(value) for value in np.ptp(points, axis=0))
        else:
            bounds = (0.0, 0.0, 0.0)
        names = ("front", "left", "right", "back")
        weak_direction = min(names, key=lambda name: self.direction_counts[name])
        return CoverageSnapshot(
            occupied, sum(count <= 2 for count in self.coverage.values()),
            int(new_voxels), growth_percent, bounds, direction, weak_direction,
            {name: int(self.direction_counts[name]) for name in names},
        )

    def accumulated_arrays(self, minimum_observations=1):
        if int(minimum_observations) < 1:
            raise ValueError("minimum_observations must be positive")
        if not self.preview:
            return np.empty((0, 3), np.float32), np.empty((0, 3), np.uint8)
        values = [
            value for value in self.preview.values()
            if value[2] >= int(minimum_observations)
        ]
        if not values:
            return np.empty((0, 3), np.float32), np.empty((0, 3), np.uint8)
        return (
            np.asarray([value[0] / value[2] for value in values], dtype=np.float32),
            np.clip(
                np.rint([value[1] / value[2] for value in values]), 0, 255
            ).astype(np.uint8),
        )

    def coverage_arrays(self):
        if not self.coverage:
            return np.empty((0, 3), np.float32), np.empty((0,), np.int32)
        keys = np.asarray(list(self.coverage), dtype=np.float32)
        centers = (keys + 0.5) * self.coverage_voxel_m
        return centers, np.asarray(list(self.coverage.values()), dtype=np.int32)
