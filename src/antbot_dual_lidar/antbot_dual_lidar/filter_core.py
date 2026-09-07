"""NumPy-only filtering primitives kept independent of ROS for unit testing."""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class FilterConfig:
    min_range: float
    max_range: float
    min_height: float
    max_height: float
    voxel_leaf_size: float
    body_filter_enabled: bool
    body_min_x: float
    body_max_x: float
    body_min_y: float
    body_max_y: float
    body_min_z: float
    body_max_z: float

    def validate(self) -> None:
        pairs = (
            ("range", self.min_range, self.max_range),
            ("height", self.min_height, self.max_height),
            ("body x", self.body_min_x, self.body_max_x),
            ("body y", self.body_min_y, self.body_max_y),
            ("body z", self.body_min_z, self.body_max_z),
        )
        for name, low, high in pairs:
            if not np.isfinite(low) or not np.isfinite(high) or low >= high:
                raise ValueError(f"invalid {name} limits: {low} >= {high}")
        if self.min_range < 0.0:
            raise ValueError("min_range must be non-negative")
        if not np.isfinite(self.voxel_leaf_size) or self.voxel_leaf_size < 0.0:
            raise ValueError("voxel_leaf_size must be finite and non-negative")


def quaternion_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Return a homogeneous rotation matrix for a normalized quaternion."""
    norm = x * x + y * y + z * z + w * w
    if norm < 1e-15:
        raise ValueError("zero-length quaternion")
    scale = 2.0 / norm
    xx, yy, zz = x * x * scale, y * y * scale, z * z * scale
    xy, xz, yz = x * y * scale, x * z * scale, y * z * scale
    wx, wy, wz = w * x * scale, w * y * scale, w * z * scale
    return np.array(
        [
            [1.0 - yy - zz, xy - wz, xz + wy],
            [xy + wz, 1.0 - xx - zz, yz - wx],
            [xz - wy, yz + wx, 1.0 - xx - yy],
        ],
        dtype=np.float64,
    )


def transform_xyz(points: np.ndarray, translation, quaternion) -> np.ndarray:
    """Transform an N x 3 array without mutating the input."""
    xyz = np.asarray(points, dtype=np.float64)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError("points must have shape (N, 3)")
    rotation = quaternion_matrix(*quaternion)
    return xyz @ rotation.T + np.asarray(translation, dtype=np.float64)


def filter_xyz(points: np.ndarray, config: FilterConfig, return_indices=False):
    """Return filtered points and per-stage rejection counts."""
    config.validate()
    xyz = np.asarray(points, dtype=np.float64)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError("points must have shape (N, 3)")
    stats = {"input": int(len(xyz))}
    indices = np.arange(len(xyz))
    finite = np.isfinite(xyz).all(axis=1)
    xyz = xyz[finite]
    indices = indices[finite]
    stats["non_finite"] = int((~finite).sum())

    ranges = np.linalg.norm(xyz, axis=1)
    range_mask = (ranges >= config.min_range) & (ranges <= config.max_range)
    stats["range"] = int((~range_mask).sum())
    xyz = xyz[range_mask]
    indices = indices[range_mask]

    height_mask = (xyz[:, 2] >= config.min_height) & (xyz[:, 2] <= config.max_height)
    stats["height"] = int((~height_mask).sum())
    xyz = xyz[height_mask]
    indices = indices[height_mask]

    stats["body"] = 0
    if config.body_filter_enabled:
        inside = (
            (xyz[:, 0] >= config.body_min_x)
            & (xyz[:, 0] <= config.body_max_x)
            & (xyz[:, 1] >= config.body_min_y)
            & (xyz[:, 1] <= config.body_max_y)
            & (xyz[:, 2] >= config.body_min_z)
            & (xyz[:, 2] <= config.body_max_z)
        )
        stats["body"] = int(inside.sum())
        xyz = xyz[~inside]
        indices = indices[~inside]

    stats["voxel"] = 0
    if config.voxel_leaf_size > 0.0 and len(xyz):
        voxel = np.floor(xyz / config.voxel_leaf_size).astype(np.int64)
        _, unique_indices = np.unique(voxel, axis=0, return_index=True)
        kept = np.sort(unique_indices)
        stats["voxel"] = int(len(xyz) - len(kept))
        xyz = xyz[kept]
        indices = indices[kept]
    stats["output"] = int(len(xyz))
    result = xyz.astype(np.float32, copy=False)
    if return_indices:
        return result, stats, indices
    return result, stats
