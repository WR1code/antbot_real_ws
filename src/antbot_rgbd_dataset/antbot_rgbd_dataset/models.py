"""Source-independent RGB-D data contract. This module has no ROS dependency.

Transform naming follows one rule: T_A_B maps points expressed in frame B into
frame A. Therefore T_world_camera is always camera-to-world.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np


@dataclass(frozen=True)
class CameraIntrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    distortion_model: str = "plumb_bob"
    distortion_coefficients: tuple[float, ...] = ()
    frame_id: str = ""
    timestamp_ns: int = 0
    rectified: bool = False

    def validate(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("camera width and height must be positive")
        if not np.isfinite([self.fx, self.fy, self.cx, self.cy]).all():
            raise ValueError("camera intrinsics must be finite")
        if self.fx <= 0 or self.fy <= 0:
            raise ValueError("camera focal lengths must be positive")
        if not (0 <= self.cx < self.width and 0 <= self.cy < self.height):
            raise ValueError("camera principal point is outside the image")


@dataclass(frozen=True)
class RGBDFrame:
    timestamp_ns: int
    frame_index: int
    color: np.ndarray
    depth_m: np.ndarray
    camera_intrinsics: CameraIntrinsics
    T_world_camera: np.ndarray
    pose_source: str
    color_frame_id: str
    depth_frame_id: str
    camera_frame_id: str
    world_frame_id: str
    color_timestamp_ns: int
    depth_timestamp_ns: int
    pose_timestamp_ns: int
    color_order: str = "RGB"
    depth_unit: str = "meter"
    depth_aligned_to_color: bool = True
    robot_id: str = "antbot"
    sensor_id: str = "rgbd"
    metadata: dict[str, Any] = field(default_factory=dict)

    @property
    def rgb_depth_delta_ms(self) -> float:
        return abs(self.color_timestamp_ns - self.depth_timestamp_ns) / 1e6

    @property
    def pose_delta_ms(self) -> float:
        return abs(self.timestamp_ns - self.pose_timestamp_ns) / 1e6

    def validate(self, require_color_alignment: bool = True) -> None:
        self.camera_intrinsics.validate()
        if self.timestamp_ns < 0 or min(
            self.color_timestamp_ns, self.depth_timestamp_ns, self.pose_timestamp_ns
        ) < 0:
            raise ValueError("timestamps must be non-negative integer nanoseconds")
        if self.color_order != "RGB":
            raise ValueError(f"core requires RGB channel order, got {self.color_order!r}")
        if self.depth_unit != "meter" or self.depth_m.dtype != np.float32:
            raise ValueError("core requires float32 depth in meters")
        expected = (self.camera_intrinsics.height, self.camera_intrinsics.width)
        if self.color.dtype != np.uint8 or self.color.shape != (*expected, 3):
            raise ValueError(f"invalid color image: {self.color.dtype} {self.color.shape}")
        if self.depth_m.shape != expected:
            raise ValueError(
                f"depth/color dimensions differ: depth={self.depth_m.shape}, color={expected}"
            )
        if require_color_alignment and not self.depth_aligned_to_color:
            raise ValueError(
                "colored reconstruction rejected: depth_aligned_to_color is false "
                "and no reprojection calibration was supplied"
            )
        matrix = np.asarray(self.T_world_camera)
        if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
            raise ValueError("T_world_camera must be a finite 4x4 matrix")
        if not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-8):
            raise ValueError("T_world_camera has an invalid homogeneous row")
        rotation = matrix[:3, :3]
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5):
            raise ValueError("T_world_camera rotation is not orthonormal")
        if not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-5):
            raise ValueError("T_world_camera rotation determinant is not +1")

