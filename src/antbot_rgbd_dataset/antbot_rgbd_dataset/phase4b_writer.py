"""Crash-consistent Phase 4B writer compatible with the Phase 3 loader."""

from __future__ import annotations

from collections import Counter
from dataclasses import asdict
from pathlib import Path
import shutil

import numpy as np

from .core import (
    DatasetWriter, Selection, atomic_yaml, depth_statistics, utc_timestamp
)
from .keyframes import KeyframeDecision
from .models import RGBDFrame
from .transforms import rotation_to_quaternion_xyzw


class Phase4BDatasetWriter:
    def __init__(
        self, output_root: str | Path, dataset_name: str,
        overwrite_existing: bool = False, minimum_free_space_gb: float = 2.0
    ):
        output_root = Path(output_root).expanduser()
        output_root.mkdir(parents=True, exist_ok=True)
        free = shutil.disk_usage(output_root).free
        if free < minimum_free_space_gb * 1024**3:
            raise OSError(
                f"insufficient disk space: {free / 1024**3:.2f} GiB available, "
                f"{minimum_free_space_gb:.2f} GiB required"
            )
        self.writer = DatasetWriter(
            output_root, dataset_name, save_depth_npy=True,
            save_depth_png=True, overwrite_existing=overwrite_existing
        )
        self.created_at = utc_timestamp()
        self.counters = Counter()
        self.trajectory_length_m = 0.0
        self.complete = False

    @property
    def root(self) -> Path:
        return self.writer.root

    def write(
        self, frame: RGBDFrame, decision: KeyframeDecision,
        sharpness: float, depth_minimum_m: float, depth_maximum_m: float
    ) -> dict:
        frame.validate(require_color_alignment=True)
        stats = depth_statistics(frame.depth_m, depth_minimum_m, depth_maximum_m)
        index = len(self.writer.rows)
        matrix_key = f"T_{frame.world_frame_id}_camera"
        matrix = frame.T_world_camera
        pose = {
            "frame_index": index,
            "timestamp_ns": frame.timestamp_ns,
            "fixed_frame": frame.world_frame_id,
            "camera_frame": frame.camera_frame_id,
            "transform_definition": (
                f"p_{frame.world_frame_id} = "
                f"T_{frame.world_frame_id}_camera * p_camera"
            ),
            "translation_m": dict(zip(("x", "y", "z"), map(float, matrix[:3, 3]))),
            "quaternion_xyzw": frame.metadata.get(
                "quaternion_xyzw", rotation_to_quaternion_xyzw(matrix[:3, :3])
            ),
            matrix_key: matrix.tolist(),
            "T_odom_camera": matrix.tolist() if frame.world_frame_id == "odom" else None,
        }
        intr = frame.camera_intrinsics
        camera_info = {
            "width": intr.width, "height": intr.height,
            "distortion_model": intr.distortion_model,
            "D": list(intr.distortion_coefficients),
            "K": [intr.fx, 0.0, intr.cx, 0.0, intr.fy, intr.cy, 0.0, 0.0, 1.0],
            "P": [intr.fx, 0.0, intr.cx, 0.0, 0.0, intr.fy, intr.cy, 0.0, 0.0, 0.0, 1.0, 0.0],
            "frame_id": intr.frame_id,
            "timestamp_ns": intr.timestamp_ns,
            "rectified": intr.rectified,
        }
        camera = {
            "frame_index": index, "timestamp_ns": frame.timestamp_ns,
            "color_camera_info": camera_info, "depth_camera_info": camera_info,
        }
        selection = Selection(
            decision.selected, decision.reasons, decision.translation_m,
            decision.rotation_deg, decision.rejection
        )
        valid = np.isfinite(frame.depth_m) & (frame.depth_m >= depth_minimum_m) & (
            frame.depth_m <= depth_maximum_m
        )
        values = frame.depth_m[valid]
        frame_metadata = {
            "frame_index": index, "timestamp_ns": frame.timestamp_ns,
            "color_timestamp_ns": frame.color_timestamp_ns,
            "depth_timestamp_ns": frame.depth_timestamp_ns,
            "pose_timestamp_ns": frame.pose_timestamp_ns,
            "rgb_depth_delta_ms": frame.rgb_depth_delta_ms,
            "pose_delta_ms": frame.pose_delta_ms,
            matrix_key: matrix.tolist(),
            "translation_from_previous_keyframe_m": decision.translation_m,
            "rotation_from_previous_keyframe_deg": decision.rotation_deg,
            "depth_valid_ratio": stats.valid_ratio,
            "depth_median_m": float(np.median(values)) if values.size else None,
            "depth_p95_m": float(np.percentile(values, 95)) if values.size else None,
            "pose_source": frame.pose_source,
            "depth_aligned_to_color": frame.depth_aligned_to_color,
            "color_order": frame.color_order, "depth_unit": frame.depth_unit,
            "robot_id": frame.robot_id, "sensor_id": frame.sensor_id,
            "frame_ids": {
                "color": frame.color_frame_id, "depth": frame.depth_frame_id,
                "camera": frame.camera_frame_id, "world": frame.world_frame_id,
            },
            "quality": frame.metadata.get("quality", {}),
        }
        row = self.writer.write_frame(
            frame.timestamp_ns, frame.color, frame.depth_m, pose, camera,
            stats, sharpness, selection, frame_metadata=frame_metadata
        )
        self.counters["frames_saved"] += 1
        self.trajectory_length_m += decision.translation_m
        self._write_intrinsics(frame)
        return row

    def _write_intrinsics(self, frame: RGBDFrame) -> None:
        intr = frame.camera_intrinsics
        atomic_yaml(self.root / "intrinsics.yaml", {
            "image_width": intr.width, "image_height": intr.height,
            "fx": intr.fx, "fy": intr.fy, "cx": intr.cx, "cy": intr.cy,
            "K": [intr.fx, 0.0, intr.cx, 0.0, intr.fy, intr.cy, 0.0, 0.0, 1.0],
            "P": [intr.fx, 0.0, intr.cx, 0.0, 0.0, intr.fy, intr.cy, 0.0, 0.0, 0.0, 1.0, 0.0],
            "distortion_model": intr.distortion_model,
            "D": list(intr.distortion_coefficients),
            "depth_unit": "meter", "depth_semantics": "distance_to_image_plane",
            "camera_frame": frame.camera_frame_id, "rectified": intr.rectified,
        })

    def finalize(self, summary: dict, complete: bool = True) -> None:
        rows = self.writer.rows
        first_frame = summary.get("first_frame")
        intr = first_frame.camera_intrinsics if first_frame else None
        metadata = {
            "dataset_name": self.root.name, "created_at": self.created_at,
            "completed_at": utc_timestamp(), "coordinate_frame": (
                first_frame.world_frame_id if first_frame else summary.get("world_frame")
            ),
            "camera_frame": (
                first_frame.camera_frame_id if first_frame else summary.get("camera_frame")
            ),
            "transform_definition": "T_A_B maps points in B into A",
            "source_type": "live_ros2", "pose_source": summary.get("pose_source"),
            "depth_unit": "meter", "depth_semantics": "distance_to_image_plane",
            "rgb_encoding": "rgb8", "depth_encoding": "32FC1",
            "camera_info_constant": True if rows else None,
            "depth_npy": {"dtype": "float32", "unit": "meter"},
            "depth_png": {"enabled": True, "dtype": "uint16", "unit": "millimeter"},
            "frame_count": len(rows),
            "trajectory_length_m": self.trajectory_length_m,
            "start_timestamp_ns": rows[0]["timestamp_ns"] if rows else None,
            "end_timestamp_ns": rows[-1]["timestamp_ns"] if rows else None,
            "image_width": intr.width if intr else None,
            "image_height": intr.height if intr else None,
            "intrinsics": (
                {"fx": intr.fx, "fy": intr.fy, "cx": intr.cx, "cy": intr.cy}
                if intr else None
            ),
            "robot_id": summary.get("robot_id"), "sensor_id": summary.get("sensor_id"),
            "capture_configuration": summary.get("configuration", {}),
            "counters": summary.get("counters", {}),
            "complete": complete,
        }
        self.writer.finalize(metadata, complete=complete)
        serializable_summary = {k: v for k, v in summary.items() if k != "first_frame"}
        serializable_summary.update({
            "dataset_path": str(self.root), "frames_saved": len(rows),
            "complete": complete, "completed_at": utc_timestamp(),
        })
        atomic_yaml(self.root / "capture_summary.yaml", serializable_summary)
        atomic_yaml(self.root / "validation.yaml", {
            "passed": bool(complete and rows),
            "frame_count": len(rows),
            "no_temporary_files": not any(self.root.rglob("*.tmp")),
            "errors": [] if complete and rows else ["capture incomplete or empty"],
        })
        self.complete = complete
