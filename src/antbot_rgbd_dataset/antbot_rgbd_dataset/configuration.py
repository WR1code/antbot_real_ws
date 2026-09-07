"""Strict Phase 4B configuration validation."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml


REQUIRED_CAPTURE_FIELDS = {
    "color_topic", "depth_topic", "camera_info_topic", "pose_mode", "pose_source",
    "world_frame", "camera_frame", "depth_encoding_mode",
    "depth_aligned_to_color", "max_rgb_depth_delta_ms", "max_pose_delta_ms",
    "sync_queue_size", "translation_threshold_m", "rotation_threshold_deg",
    "min_keyframe_interval_s", "max_keyframe_interval_s", "output_root",
    "dataset_name", "robot_id", "sensor_id",
}


def validate_capture_parameters(values: dict[str, Any]) -> dict[str, Any]:
    missing = sorted(REQUIRED_CAPTURE_FIELDS - values.keys())
    if missing:
        raise ValueError(f"missing required Phase 4B configuration fields: {missing}")
    if values["pose_mode"] not in ("tf", "topic"):
        raise ValueError("pose_mode must be 'tf' or 'topic'")
    if not values["depth_aligned_to_color"]:
        raise ValueError(
            "Phase 4B colored capture requires depth_aligned_to_color=true; "
            "offline reprojection is not implemented"
        )
    if float(values["max_rgb_depth_delta_ms"]) < 0 or float(values["max_pose_delta_ms"]) < 0:
        raise ValueError("synchronization tolerances must be non-negative")
    if int(values["sync_queue_size"]) < 1:
        raise ValueError("sync_queue_size must be positive")
    if float(values["min_keyframe_interval_s"]) >= float(values["max_keyframe_interval_s"]):
        raise ValueError("minimum keyframe interval must be below maximum interval")
    for field in ("preview_voxel_size_m", "coverage_voxel_size_m", "preview_max_rate_hz"):
        if field in values and float(values[field]) <= 0:
            raise ValueError(f"{field} must be positive")
    if "preview_pixel_stride" in values and int(values["preview_pixel_stride"]) < 1:
        raise ValueError("preview_pixel_stride must be positive")
    for field in ("preview_edge_filter_absolute_m", "preview_edge_filter_relative"):
        if field in values and float(values[field]) < 0:
            raise ValueError(f"{field} must be non-negative")
    if "preview_min_observations" in values and int(values["preview_min_observations"]) < 1:
        raise ValueError("preview_min_observations must be positive")
    if "target_keyframes" in values and int(values["target_keyframes"]) < 1:
        raise ValueError("target_keyframes must be positive")
    return values


def load_ros_parameter_yaml(path: str | Path, node_name: str = "phase4b_capture_node") -> dict:
    document = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if node_name not in document or "ros__parameters" not in document[node_name]:
        raise ValueError(f"configuration lacks {node_name}.ros__parameters")
    return validate_capture_parameters(document[node_name]["ros__parameters"])
