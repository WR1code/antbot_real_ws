"""Observation-count coverage analysis using unified RGBDFrame values."""

from __future__ import annotations

from collections import Counter
import numpy as np


def analyze_frame_coverage(
    frames, voxel_size_m: float = 0.20, pixel_stride: int = 8,
    depth_minimum_m: float = 0.10, depth_maximum_m: float = 8.0
):
    if voxel_size_m <= 0 or pixel_stride < 1:
        raise ValueError("invalid coverage sampling parameters")
    observations = Counter()
    all_points, all_depth, camera_positions = [], [], []
    frame_count = 0
    for frame in frames:
        frame.validate()
        frame_count += 1
        depth = frame.depth_m[::pixel_stride, ::pixel_stride]
        v, u = np.mgrid[0:frame.depth_m.shape[0]:pixel_stride,
                        0:frame.depth_m.shape[1]:pixel_stride]
        valid = (
            np.isfinite(depth) & (depth >= depth_minimum_m)
            & (depth <= depth_maximum_m)
        )
        z = depth[valid].astype(np.float64)
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
        voxels = np.floor(points_world / voxel_size_m).astype(np.int64)
        for voxel in np.unique(voxels, axis=0):
            observations[tuple(int(value) for value in voxel)] += 1
        all_points.append(points_world)
        all_depth.append(z)
        camera_positions.append(frame.T_world_camera[:3, 3])
    if not all_points:
        raise ValueError("coverage analysis received no frames")
    points = np.concatenate(all_points)
    depths = np.concatenate(all_depth)
    centers = (np.asarray(list(observations), dtype=np.float64) + 0.5) * voxel_size_m
    counts = np.asarray(list(observations.values()), dtype=np.int32)
    axis_statistics = {}
    for index, axis in enumerate("xyz"):
        axis_statistics[axis] = {
            name: float(value) for name, value in zip(
                ("min", "p1", "p50", "p99", "max"),
                np.percentile(points[:, index], [0, 1, 50, 99, 100])
            )
        }
    depth_percentiles = {
        name: float(value) for name, value in zip(
            ("p50", "p90", "p95", "p99"), np.percentile(depths, [50, 90, 95, 99])
        )
    }
    report = {
        "input_frame_count": frame_count,
        "voxel_size_m": voxel_size_m,
        "pixel_stride": pixel_stride,
        "depth_range_m": [depth_minimum_m, depth_maximum_m],
        "sampled_point_count": int(len(points)),
        "camera_positions": np.asarray(camera_positions).tolist(),
        "point_bounds_and_percentiles": axis_statistics,
        "depth_percentiles_m": depth_percentiles,
        "far_depth_ratios": {
            "greater_than_5m": float((depths > 5).mean()),
            "greater_than_6m": float((depths > 6).mean()),
            "greater_than_7m": float((depths > 7).mean()),
        },
        "occupied_voxel_count": int(len(counts)),
        "observation_count": {
            "minimum": int(counts.min()), "p50": float(np.median(counts)),
            "p90": float(np.percentile(counts, 90)), "maximum": int(counts.max()),
        },
        "low_coverage_voxels": {
            "observed_once": int((counts == 1).sum()),
            "observed_at_most_twice": int((counts <= 2).sum()),
            "observed_once_ratio": float((counts == 1).mean()),
            "observed_at_most_twice_ratio": float((counts <= 2).mean()),
        },
        "interpretation": (
            "Low observation counts indicate candidate recapture regions; they are "
            "not deleted, smoothed, or treated as definitive geometric holes."
        ),
    }
    return report, centers, counts
