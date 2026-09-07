"""ROS-independent pose interpolation and per-point motion compensation."""

from bisect import bisect_left
from dataclasses import dataclass
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class Pose:
    stamp_ns: int
    translation: np.ndarray
    quaternion_xyzw: np.ndarray


def normalize_quaternion(quaternion):
    quaternion = np.asarray(quaternion, dtype=np.float64)
    norm = float(np.linalg.norm(quaternion))
    if norm <= 1e-12:
        raise ValueError("zero-length quaternion")
    return quaternion / norm


def quaternion_slerp(first, second, fraction):
    """Shortest-path SLERP for ROS-order (x, y, z, w) quaternions."""
    first = normalize_quaternion(first)
    second = normalize_quaternion(second)
    dot = float(np.dot(first, second))
    if dot < 0.0:
        second = -second
        dot = -dot
    dot = min(1.0, max(-1.0, dot))
    fraction = float(fraction)
    if dot > 0.9995:
        return normalize_quaternion(first + fraction * (second - first))
    theta = np.arccos(dot)
    sine = np.sin(theta)
    return (
        np.sin((1.0 - fraction) * theta) / sine * first
        + np.sin(fraction * theta) / sine * second
    )


def quaternion_matrix(quaternion):
    x, y, z, w = normalize_quaternion(quaternion)
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


class PoseBuffer:
    """Bounded, reset-aware pose history with continuous interpolation."""

    def __init__(self, duration_sec=10.0):
        if duration_sec <= 0.0:
            raise ValueError("duration_sec must be positive")
        self.duration_ns = int(duration_sec * 1_000_000_000)
        self._poses = []
        self.reset_count = 0

    def clear(self):
        self._poses.clear()

    def add(self, stamp_ns, translation, quaternion_xyzw):
        stamp_ns = int(stamp_ns)
        pose = Pose(
            stamp_ns,
            np.asarray(translation, dtype=np.float64),
            normalize_quaternion(quaternion_xyzw),
        )
        if pose.translation.shape != (3,):
            raise ValueError("translation must have shape (3,)")
        if self._poses and stamp_ns < self._poses[-1].stamp_ns:
            self.clear()
            self.reset_count += 1
        if self._poses and stamp_ns == self._poses[-1].stamp_ns:
            self._poses[-1] = pose
        else:
            self._poses.append(pose)
        cutoff = stamp_ns - self.duration_ns
        first_kept = bisect_left([item.stamp_ns for item in self._poses], cutoff)
        if first_kept:
            del self._poses[:first_kept]

    @property
    def bounds(self):
        if not self._poses:
            return None
        return self._poses[0].stamp_ns, self._poses[-1].stamp_ns

    def interpolate(self, stamp_ns):
        if not self._poses:
            return None
        stamp_ns = int(stamp_ns)
        stamps = [item.stamp_ns for item in self._poses]
        index = bisect_left(stamps, stamp_ns)
        if index < len(stamps) and stamps[index] == stamp_ns:
            return self._poses[index]
        if index == 0 or index == len(stamps):
            return None
        lower = self._poses[index - 1]
        upper = self._poses[index]
        fraction = (stamp_ns - lower.stamp_ns) / (upper.stamp_ns - lower.stamp_ns)
        return Pose(
            stamp_ns,
            lower.translation + fraction * (upper.translation - lower.translation),
            quaternion_slerp(lower.quaternion_xyzw, upper.quaternion_xyzw, fraction),
        )

    def interpolate_many(self, stamps_ns):
        """Vectorized interpolation returning translations and xyzw quaternions."""
        query = np.asarray(stamps_ns, dtype=np.int64)
        if query.ndim != 1:
            raise ValueError("stamps_ns must be one-dimensional")
        if not len(query):
            return np.empty((0, 3)), np.empty((0, 4))
        if not self._poses:
            return None
        stamps = np.fromiter(
            (pose.stamp_ns for pose in self._poses), dtype=np.int64, count=len(self._poses)
        )
        if query.min() < stamps[0] or query.max() > stamps[-1]:
            return None
        upper = np.searchsorted(stamps, query, side="left")
        exact = (upper < len(stamps)) & (stamps[np.minimum(upper, len(stamps) - 1)] == query)
        lower = np.maximum(upper - 1, 0)
        upper = np.minimum(upper, len(stamps) - 1)
        lower[exact] = upper[exact]
        translations = np.asarray([pose.translation for pose in self._poses])
        quaternions = np.asarray([pose.quaternion_xyzw for pose in self._poses])
        denominator = stamps[upper] - stamps[lower]
        fraction = np.divide(
            query - stamps[lower],
            denominator,
            out=np.zeros(len(query), dtype=np.float64),
            where=denominator != 0,
        )
        output_translation = (
            translations[lower]
            + fraction[:, None] * (translations[upper] - translations[lower])
        )
        output_quaternion = quaternion_slerp_many(
            quaternions[lower], quaternions[upper], fraction
        )
        return output_translation, output_quaternion


def quaternion_slerp_many(first, second, fraction):
    """Vectorized shortest-path SLERP for arrays of ROS-order quaternions."""
    first = np.asarray(first, dtype=np.float64)
    second = np.asarray(second, dtype=np.float64)
    fraction = np.asarray(fraction, dtype=np.float64)
    if first.ndim != 2 or first.shape[1] != 4 or second.shape != first.shape:
        raise ValueError("quaternion arrays must have shape (N, 4)")
    if fraction.shape != (len(first),):
        raise ValueError("fraction must have shape (N,)")
    first = first / np.linalg.norm(first, axis=1, keepdims=True)
    second = second / np.linalg.norm(second, axis=1, keepdims=True)
    dot = np.einsum("ij,ij->i", first, second)
    negative = dot < 0.0
    second = second.copy()
    second[negative] *= -1.0
    dot = np.abs(dot)
    dot = np.clip(dot, -1.0, 1.0)
    output = np.empty_like(first)
    linear = dot > 0.9995
    if np.any(linear):
        output[linear] = (
            first[linear]
            + fraction[linear, None] * (second[linear] - first[linear])
        )
    spherical = ~linear
    if np.any(spherical):
        theta = np.arccos(dot[spherical])
        sine = np.sin(theta)
        weight_first = np.sin((1.0 - fraction[spherical]) * theta) / sine
        weight_second = np.sin(fraction[spherical] * theta) / sine
        output[spherical] = (
            weight_first[:, None] * first[spherical]
            + weight_second[:, None] * second[spherical]
        )
    return output / np.linalg.norm(output, axis=1, keepdims=True)


def quaternion_matrices(quaternions):
    """Return an (N, 3, 3) rotation matrix array for xyzw quaternions."""
    values = np.asarray(quaternions, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != 4:
        raise ValueError("quaternions must have shape (N, 4)")
    values = values / np.linalg.norm(values, axis=1, keepdims=True)
    x, y, z, w = values.T
    output = np.empty((len(values), 3, 3), dtype=np.float64)
    output[:, 0, 0] = 1 - 2 * (y * y + z * z)
    output[:, 0, 1] = 2 * (x * y - z * w)
    output[:, 0, 2] = 2 * (x * z + y * w)
    output[:, 1, 0] = 2 * (x * y + z * w)
    output[:, 1, 1] = 1 - 2 * (x * x + z * z)
    output[:, 1, 2] = 2 * (y * z - x * w)
    output[:, 2, 0] = 2 * (x * z - y * w)
    output[:, 2, 1] = 2 * (y * z + x * w)
    output[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return output


POINT_TIME_CONVENTIONS = (
    "header_plus_offset",
    "header_minus_offset",
    "header_plus_offset_minus_scan_period",
    "header_midpoint",
)


def point_times_ns(header_stamp_ns, offsets_ns, convention, scan_period_sec=0.1):
    offsets = np.asarray(offsets_ns, dtype=np.int64)
    header = int(header_stamp_ns)
    period = int(round(float(scan_period_sec) * 1_000_000_000))
    if convention == "header_plus_offset":
        return header + offsets
    if convention == "header_minus_offset":
        return header - offsets
    if convention == "header_plus_offset_minus_scan_period":
        return header + offsets - period
    if convention == "header_midpoint":
        return header + offsets - period // 2
    raise ValueError(f"unsupported point-time convention: {convention}")


def compose_pose(base_pose, base_to_sensor_translation, base_to_sensor_quaternion):
    base_to_sensor_translation = np.asarray(base_to_sensor_translation, dtype=np.float64)
    base_to_sensor_rotation = quaternion_matrix(base_to_sensor_quaternion)
    base_rotation = quaternion_matrix(base_pose.quaternion_xyzw)
    sensor_rotation = base_rotation @ base_to_sensor_rotation
    sensor_translation = base_pose.translation + base_rotation @ base_to_sensor_translation
    return sensor_translation, sensor_rotation


def deskew_points(
    xyz,
    point_stamps_ns: Iterable[int],
    reference_stamp_ns,
    poses,
    base_to_sensor_translation,
    base_to_sensor_quaternion,
):
    """Transform each sensor-frame point into the sensor frame at reference time.

    Returns ``None`` when any required pose is outside the truth buffer. This
    intentionally rejects incomplete scans instead of silently mixing frames.
    """
    xyz = np.asarray(xyz, dtype=np.float64)
    stamps = np.asarray(point_stamps_ns, dtype=np.int64)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError("xyz must have shape (N, 3)")
    if len(xyz) != len(stamps):
        raise ValueError("point timestamp count does not match xyz")
    if len(xyz) == 0:
        return xyz.copy()
    interpolated = poses.interpolate_many(
        np.concatenate((stamps, np.asarray([reference_stamp_ns], dtype=np.int64)))
    )
    if interpolated is None:
        return None
    base_translation, base_quaternion = interpolated
    base_rotation = quaternion_matrices(base_quaternion)
    extrinsic_translation = np.asarray(base_to_sensor_translation, dtype=np.float64)
    extrinsic_rotation = quaternion_matrix(base_to_sensor_quaternion)
    sensor_translation = (
        base_translation
        + np.einsum("nij,j->ni", base_rotation, extrinsic_translation)
    )
    sensor_rotation = np.einsum("nij,jk->nik", base_rotation, extrinsic_rotation)
    point_translation = sensor_translation[:-1]
    point_rotation = sensor_rotation[:-1]
    ref_translation = sensor_translation[-1]
    ref_rotation = sensor_rotation[-1]
    world_points = np.einsum("nij,nj->ni", point_rotation, xyz) + point_translation
    return (world_points - ref_translation) @ ref_rotation


def points_to_world(
    xyz,
    point_stamps_ns,
    poses,
    base_to_sensor_translation,
    base_to_sensor_quaternion,
):
    """Project native per-ray sensor coordinates to truth world endpoints."""
    xyz = np.asarray(xyz, dtype=np.float64)
    stamps = np.asarray(point_stamps_ns, dtype=np.int64)
    if xyz.ndim != 2 or xyz.shape[1] != 3 or stamps.shape != (len(xyz),):
        raise ValueError("xyz must be (N, 3) and timestamps must be (N,)")
    if not len(xyz):
        return xyz.copy()
    interpolated = poses.interpolate_many(stamps)
    if interpolated is None:
        return None
    base_translation, base_quaternion = interpolated
    base_rotation = quaternion_matrices(base_quaternion)
    extrinsic_translation = np.asarray(base_to_sensor_translation, dtype=np.float64)
    extrinsic_rotation = quaternion_matrix(base_to_sensor_quaternion)
    sensor_translation = (
        base_translation
        + np.einsum("nij,j->ni", base_rotation, extrinsic_translation)
    )
    sensor_rotation = np.einsum("nij,jk->nik", base_rotation, extrinsic_rotation)
    return np.einsum("nij,nj->ni", sensor_rotation, xyz) + sensor_translation
