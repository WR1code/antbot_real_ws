"""Transform direction and real-robot pose quality protection."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


def query_pose_at_timestamp(query, sensor_timestamp_ns: int, max_delta_ms: float):
    """Execute a pose/TF query at the sensor stamp and enforce returned stamp age."""
    try:
        result = query(sensor_timestamp_ns)
    except Exception as exc:
        return None, f"POSE_QUERY_FAILED: {type(exc).__name__}: {exc}"
    if result is None:
        return None, "POSE_QUERY_FAILED"
    pose_timestamp_ns = int(result[2])
    if abs(pose_timestamp_ns - sensor_timestamp_ns) / 1e6 > max_delta_ms:
        return None, "POSE_TIME_DELTA"
    return result, None


def invert_transform(T_A_B: np.ndarray) -> np.ndarray:
    """Return T_B_A from T_A_B."""
    value = np.asarray(T_A_B, dtype=np.float64)
    if value.shape != (4, 4):
        raise ValueError("transform must be 4x4")
    inverse = np.linalg.inv(value)
    if not np.allclose(inverse @ value, np.eye(4), atol=1e-8):
        raise ValueError("transform inversion failed")
    return inverse


def transform_delta(T_world_previous: np.ndarray, T_world_current: np.ndarray) -> tuple[float, float]:
    translation = float(np.linalg.norm(
        T_world_current[:3, 3] - T_world_previous[:3, 3]
    ))
    relative_rotation = T_world_previous[:3, :3].T @ T_world_current[:3, :3]
    cosine = float(np.clip((np.trace(relative_rotation) - 1) / 2, -1, 1))
    return translation, math.degrees(math.acos(cosine))


def rotation_to_quaternion_xyzw(rotation: np.ndarray) -> dict[str, float]:
    """Convert a proper rotation matrix to a normalized xyzw quaternion."""
    matrix = np.asarray(rotation, dtype=np.float64)
    eigenvalues, eigenvectors = np.linalg.eigh(np.array([
        [matrix[0, 0]-matrix[1, 1]-matrix[2, 2], matrix[1, 0]+matrix[0, 1],
         matrix[2, 0]+matrix[0, 2], matrix[1, 2]-matrix[2, 1]],
        [matrix[1, 0]+matrix[0, 1], matrix[1, 1]-matrix[0, 0]-matrix[2, 2],
         matrix[2, 1]+matrix[1, 2], matrix[2, 0]-matrix[0, 2]],
        [matrix[2, 0]+matrix[0, 2], matrix[2, 1]+matrix[1, 2],
         matrix[2, 2]-matrix[0, 0]-matrix[1, 1], matrix[0, 1]-matrix[1, 0]],
        [matrix[1, 2]-matrix[2, 1], matrix[2, 0]-matrix[0, 2],
         matrix[0, 1]-matrix[1, 0], matrix.trace()],
    ]) / 3.0)
    q = eigenvectors[:, np.argmax(eigenvalues)]
    if q[3] < 0:
        q = -q
    return dict(zip(("x", "y", "z", "w"), map(float, q / np.linalg.norm(q))))


@dataclass(frozen=True)
class PoseQualityConfig:
    max_translation_jump_m: float = 1.0
    max_rotation_jump_deg: float = 60.0
    max_linear_speed_mps: float = 3.0
    max_angular_speed_deg_s: float = 180.0


def check_pose_quality(
    config: PoseQualityConfig,
    previous_timestamp_ns: int | None,
    T_world_previous: np.ndarray | None,
    timestamp_ns: int,
    T_world_camera: np.ndarray,
) -> tuple[bool, str | None, dict]:
    if T_world_previous is None or previous_timestamp_ns is None:
        return True, None, {"translation_m": 0.0, "rotation_deg": 0.0}
    if timestamp_ns <= previous_timestamp_ns:
        return False, "NON_MONOTONIC_POSE_TIMESTAMP", {}
    translation, rotation = transform_delta(T_world_previous, T_world_camera)
    elapsed = (timestamp_ns - previous_timestamp_ns) / 1e9
    metrics = {
        "translation_m": translation,
        "rotation_deg": rotation,
        "linear_speed_mps": translation / elapsed,
        "angular_speed_deg_s": rotation / elapsed,
    }
    checks = (
        ("TRANSLATION_JUMP", translation, config.max_translation_jump_m),
        ("ROTATION_JUMP", rotation, config.max_rotation_jump_deg),
        ("LINEAR_SPEED", metrics["linear_speed_mps"], config.max_linear_speed_mps),
        ("ANGULAR_SPEED", metrics["angular_speed_deg_s"], config.max_angular_speed_deg_s),
    )
    for reason, value, maximum in checks:
        if maximum > 0 and value > maximum:
            return False, reason, metrics
    return True, None, metrics
