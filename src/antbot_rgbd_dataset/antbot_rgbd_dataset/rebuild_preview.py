"""Rebuild a deghosted RGB-D preview from a recorded Phase 4B dataset."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import numpy as np
import yaml

from .live_preview import (
    LivePreviewAccumulator,
    load_preview_archive,
    project_rgbd,
    project_rgbd_filtered,
    save_preview_archive,
    voxel_reduce,
)
from .sources.dataset import DatasetRGBDSource


def estimate_rigid_transform(source: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, float]:
    """Estimate T_target_source for corresponding row-vector points."""
    source = np.asarray(source, dtype=np.float64)
    target = np.asarray(target, dtype=np.float64)
    if source.shape != target.shape or source.ndim != 2 or source.shape[1] != 3:
        raise ValueError("rigid transform inputs must be corresponding Nx3 arrays")
    if len(source) < 3:
        raise ValueError("at least three correspondences are required")
    if len(source) > 50_000:
        indices = np.linspace(0, len(source) - 1, 50_000, dtype=np.int64)
        source = source[indices]
        target = target[indices]
    source_center = source.mean(axis=0)
    target_center = target.mean(axis=0)
    covariance = (source - source_center).T @ (target - target_center)
    left, _singular, right_t = np.linalg.svd(covariance)
    rotation = right_t.T @ left.T
    if np.linalg.det(rotation) < 0:
        right_t[-1, :] *= -1
        rotation = right_t.T @ left.T
    translation = target_center - source_center @ rotation.T
    transformed = source @ rotation.T + translation
    rms = float(np.sqrt(np.mean(np.sum((transformed - target) ** 2, axis=1))))
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = translation
    return matrix, rms


def _capture_configuration(dataset: Path) -> dict:
    summary_path = dataset / "capture_summary.yaml"
    if not summary_path.is_file():
        return {}
    summary = yaml.safe_load(summary_path.read_text(encoding="utf-8")) or {}
    return dict(summary.get("configuration", {}))


def rebuild(
    dataset_path: Path,
    reference_preview_path: Path,
    output_path: Path,
    *,
    depth_minimum_m: float,
    depth_maximum_m: float,
    voxel_size_m: float,
    pixel_stride: int,
    edge_absolute_m: float,
    edge_relative: float,
    minimum_observations: int,
) -> dict:
    dataset = DatasetRGBDSource(dataset_path)
    reference_points, _reference_colors, target_frame = load_preview_archive(
        reference_preview_path
    )
    legacy_preview = {}
    cleaned = LivePreviewAccumulator(
        preview_voxel_m=voxel_size_m,
        coverage_voxel_m=max(0.20, voxel_size_m),
        growth_window_s=10.0,
    )
    frame_count = 0
    source_frame = None
    for frame in dataset:
        source_frame = frame.world_frame_id
        legacy_points, legacy_colors = project_rgbd(
            frame, pixel_stride, depth_minimum_m, depth_maximum_m
        )
        legacy_points, legacy_colors = voxel_reduce(
            legacy_points, legacy_colors, voxel_size_m
        )
        for key, point, color in zip(
            np.floor(legacy_points / voxel_size_m).astype(np.int32),
            legacy_points,
            legacy_colors,
        ):
            legacy_preview[tuple(int(value) for value in key)] = (
                point.copy(), color.copy()
            )

        points, colors = project_rgbd_filtered(
            frame, pixel_stride, depth_minimum_m, depth_maximum_m,
            edge_absolute_m, edge_relative,
        )
        points, colors = voxel_reduce(points, colors, voxel_size_m)
        cleaned.add_keyframe(
            frame.timestamp_ns, points, colors, frame.T_world_camera
        )
        frame_count += 1

    legacy_points = np.asarray(
        [value[0] for value in legacy_preview.values()], dtype=np.float32
    )
    if len(legacy_points) != len(reference_points):
        raise ValueError(
            "reference preview does not match this dataset: "
            f"{len(reference_points)} points != reconstructed {len(legacy_points)}"
        )
    if source_frame is None:
        raise ValueError("dataset contains no RGB-D frames")
    if source_frame == target_frame:
        alignment = np.eye(4)
        alignment_rms_m = 0.0
    else:
        alignment, alignment_rms_m = estimate_rigid_transform(
            legacy_points, reference_points
        )
        if alignment_rms_m > 0.002:
            raise ValueError(
                "reference preview alignment is inconsistent; "
                f"RMS={alignment_rms_m:.6f} m"
            )

    points, colors = cleaned.accumulated_arrays(minimum_observations)
    points = (
        points @ alignment[:3, :3].T + alignment[:3, 3]
    ).astype(np.float32)
    save_preview_archive(output_path, points, colors, target_frame)
    return {
        "frames": frame_count,
        "reference_points": int(len(reference_points)),
        "cleaned_points": int(len(points)),
        "removed_percent": round(
            100.0 * (1.0 - len(points) / max(len(reference_points), 1)), 2
        ),
        "alignment_rms_m": alignment_rms_m,
        "frame_id": target_frame,
    }


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", required=True, type=Path)
    parser.add_argument("--reference-preview", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--voxel-size-m", type=float)
    parser.add_argument("--pixel-stride", type=int)
    parser.add_argument("--depth-minimum-m", type=float)
    parser.add_argument("--depth-maximum-m", type=float)
    parser.add_argument("--edge-absolute-m", type=float, default=0.08)
    parser.add_argument("--edge-relative", type=float, default=0.03)
    parser.add_argument("--minimum-observations", type=int, default=2)
    parsed = parser.parse_args(sys.argv[1:] if args is None else args)
    if parsed.output.exists():
        parser.error(f"output already exists: {parsed.output}")
    config = _capture_configuration(parsed.dataset)
    result = rebuild(
        parsed.dataset,
        parsed.reference_preview,
        parsed.output,
        depth_minimum_m=(
            parsed.depth_minimum_m
            if parsed.depth_minimum_m is not None
            else float(config.get("depth_min", 0.20))
        ),
        depth_maximum_m=(
            parsed.depth_maximum_m
            if parsed.depth_maximum_m is not None
            else float(config.get("depth_max", 20.0))
        ),
        voxel_size_m=(
            parsed.voxel_size_m
            if parsed.voxel_size_m is not None
            else float(config.get("preview_voxel_size_m", 0.03))
        ),
        pixel_stride=(
            parsed.pixel_stride
            if parsed.pixel_stride is not None
            else int(config.get("preview_pixel_stride", 2))
        ),
        edge_absolute_m=parsed.edge_absolute_m,
        edge_relative=parsed.edge_relative,
        minimum_observations=parsed.minimum_observations,
    )
    print(yaml.safe_dump(result, sort_keys=False).strip())


if __name__ == "__main__":
    main()
