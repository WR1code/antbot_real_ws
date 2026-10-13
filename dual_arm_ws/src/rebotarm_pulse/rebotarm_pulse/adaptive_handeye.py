"""Pure geometry and estimation helpers for adaptive Link5 hand-eye calibration.

The runtime node intentionally keeps these routines ROS independent.  This makes
the transform convention testable and permits a recorded calibration run to be
re-solved without moving the robot again.
"""

from __future__ import annotations

from dataclasses import dataclass
from itertools import combinations
import math
import os
from pathlib import Path
import tempfile
from typing import Iterable, Sequence

import cv2
import numpy as np
import yaml


_SOLVERS = {
    "TSAI": cv2.CALIB_HAND_EYE_TSAI,
    "PARK": cv2.CALIB_HAND_EYE_PARK,
    "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
    "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
    "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
}


@dataclass(frozen=True)
class Transform:
    """Rigid transform ``parent_T_child`` using a 3x3 rotation and 3-vector."""

    rotation: np.ndarray
    translation: np.ndarray

    def __post_init__(self) -> None:
        rotation = np.asarray(self.rotation, dtype=np.float64).reshape((3, 3))
        translation = np.asarray(self.translation, dtype=np.float64).reshape(3)
        if not np.all(np.isfinite(rotation)) or not np.all(np.isfinite(translation)):
            raise ValueError("transform contains non-finite values")
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=2.0e-5):
            raise ValueError("transform rotation is not orthonormal")
        if np.linalg.det(rotation) < 0.999:
            raise ValueError("transform rotation is not right handed")
        object.__setattr__(self, "rotation", rotation.copy())
        object.__setattr__(self, "translation", translation.copy())

    @staticmethod
    def identity() -> "Transform":
        return Transform(np.eye(3), np.zeros(3))

    def inverse(self) -> "Transform":
        rotation = self.rotation.T
        return Transform(rotation, -(rotation @ self.translation))

    def __matmul__(self, other: "Transform") -> "Transform":
        return Transform(
            self.rotation @ other.rotation,
            self.translation + self.rotation @ other.translation,
        )

    def point(self, value: Sequence[float]) -> np.ndarray:
        return self.translation + self.rotation @ np.asarray(value, dtype=np.float64).reshape(3)


@dataclass(frozen=True)
class PosePair:
    base_T_link5: Transform
    camera_T_target: Transform


@dataclass(frozen=True)
class CalibrationCandidate:
    solver: str
    link5_T_camera: Transform
    train_translation_rms_m: float
    train_rotation_rms_deg: float
    holdout_translation_p95_m: float
    holdout_rotation_p95_deg: float


@dataclass(frozen=True)
class HemispherePose:
    radius_m: float
    theta_deg: float
    phi_deg: float
    roll_deg: float
    base_T_camera: Transform
    joint_goal: tuple[float, ...] | None = None


def plan_adaptive_increment(
    calibration_count: int,
    maximum_count: int,
    batch_count: int,
    minimum_valid_count: int = 4,
) -> tuple[int, int]:
    """Return the requested and required sample counts for the next increment.

    A final partial batch must not retain the normal batch's minimum-valid
    requirement.  For example, with 77/80 calibration samples remaining, the
    caller requests three samples and all three are sufficient for that final
    increment.
    """
    if calibration_count < 0:
        raise ValueError("calibration_count must not be negative")
    if maximum_count <= 0 or batch_count <= 0 or minimum_valid_count <= 0:
        raise ValueError("adaptive increment counts must be positive")
    remaining = max(0, maximum_count - calibration_count)
    requested = min(batch_count, remaining)
    required = min(minimum_valid_count, requested)
    return requested, required


def skew(vector: Sequence[float]) -> np.ndarray:
    x, y, z = np.asarray(vector, dtype=np.float64).reshape(3)
    return np.asarray([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def rotation_exp(vector: Sequence[float]) -> np.ndarray:
    rotation, _ = cv2.Rodrigues(np.asarray(vector, dtype=np.float64).reshape(3, 1))
    return rotation


def rotation_log(rotation: np.ndarray) -> np.ndarray:
    vector, _ = cv2.Rodrigues(np.asarray(rotation, dtype=np.float64).reshape((3, 3)))
    return vector.reshape(3)


def compute_sampling_score(candidate_pose: Transform, samples: Sequence[PosePair] = (),
                           translation_scale_m: float = 0.1,
                           regularization: float = 1.0e-6) -> dict:
    """Robot-motion-only information proxy, NOT extrinsic covariance.

    Use unique Base->Link5 poses, relative to the first pose. In scaled
    [translation, rotation] tangent coordinates J=Ad(A)-I, where
    Ad(A)=[[R,skew(t/scale)@R],[0,R]]. F=sum(J.T@J).
    Gain=logdet(F+J_new.T@J_new+lambda*I)-logdet(F+lambda*I).
    Identical poses add no geometric diversity and return zero gain.
    No camera noise model or unknown extrinsic is used; diagnostic only.
    """
    if (not math.isfinite(translation_scale_m) or translation_scale_m <= 0
            or not math.isfinite(regularization) or regularization <= 0):
        raise ValueError("sampling scale and regularization must be finite and positive")
    poses = []
    for sample in samples:
        pose = sample.base_T_link5 if isinstance(sample, PosePair) else sample
        if not any(np.allclose(pose.translation, old.translation, atol=1e-9, rtol=0)
                   and np.allclose(pose.rotation, old.rotation, atol=1e-9, rtol=0)
                   for old in poses):
            poses.append(pose)
    if len(poses) < 4:
        return {"score": 0.0, "information_gain": 0.0, "degenerate": True}

    def jacobian(pose):
        relative = poses[0].inverse() @ pose
        rotation = relative.rotation
        return np.block([
            [rotation - np.eye(3), skew(relative.translation / translation_scale_m) @ rotation],
            [np.zeros((3, 3)), rotation - np.eye(3)],
        ])

    fisher = sum((jacobian(pose).T @ jacobian(pose) for pose in poses[1:]), np.zeros((6, 6)))
    degenerate = bool(np.linalg.matrix_rank(fisher, tol=1e-8) < 6)
    if any(np.allclose(candidate_pose.translation, old.translation, atol=1e-9, rtol=0)
           and np.allclose(candidate_pose.rotation, old.rotation, atol=1e-9, rtol=0)
           for old in poses):
        return {"score": 0.0, "information_gain": 0.0, "degenerate": degenerate}
    baseline = fisher + regularization * np.eye(6)
    extra = jacobian(candidate_pose)
    gain = max(0.0, float(np.linalg.slogdet(baseline + extra.T @ extra)[1]
                          - np.linalg.slogdet(baseline)[1]))
    return {"score": gain / (1.0 + gain), "information_gain": gain,
            "degenerate": degenerate}


def quaternion_to_rotation(x: float, y: float, z: float, w: float) -> np.ndarray:
    values = np.asarray([x, y, z, w], dtype=np.float64)
    norm = float(np.linalg.norm(values))
    if not math.isfinite(norm) or norm <= 1.0e-12:
        raise ValueError("invalid zero-length quaternion")
    x, y, z, w = values / norm
    return np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ]
    )


def rotation_to_quaternion(rotation: np.ndarray) -> tuple[float, float, float, float]:
    matrix = np.asarray(rotation, dtype=np.float64).reshape((3, 3))
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        values = (
            (matrix[2, 1] - matrix[1, 2]) / scale,
            (matrix[0, 2] - matrix[2, 0]) / scale,
            (matrix[1, 0] - matrix[0, 1]) / scale,
            0.25 * scale,
        )
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            scale = math.sqrt(max(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2], 0.0)) * 2.0
            values = (0.25 * scale, (matrix[0, 1] + matrix[1, 0]) / scale,
                      (matrix[0, 2] + matrix[2, 0]) / scale, (matrix[2, 1] - matrix[1, 2]) / scale)
        elif index == 1:
            scale = math.sqrt(max(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2], 0.0)) * 2.0
            values = ((matrix[0, 1] + matrix[1, 0]) / scale, 0.25 * scale,
                      (matrix[1, 2] + matrix[2, 1]) / scale, (matrix[0, 2] - matrix[2, 0]) / scale)
        else:
            scale = math.sqrt(max(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1], 0.0)) * 2.0
            values = ((matrix[0, 2] + matrix[2, 0]) / scale,
                      (matrix[1, 2] + matrix[2, 1]) / scale, 0.25 * scale,
                      (matrix[1, 0] - matrix[0, 1]) / scale)
    norm = math.sqrt(sum(value * value for value in values))
    return tuple(float(value / norm) for value in values)


def robust_transform_mean(transforms: Sequence[Transform], z_limit: float = 3.5) -> Transform:
    """Return a robust SE(3) mean after MAD rejection in tangent coordinates."""
    if len(transforms) < 3:
        raise ValueError("at least three transforms are required")
    translations = np.asarray([item.translation for item in transforms])
    centre_t = np.median(translations, axis=0)
    reference = transforms[0].rotation
    logs = np.asarray([rotation_log(reference.T @ item.rotation) for item in transforms])
    centre_w = np.median(logs, axis=0)
    features = np.column_stack((translations - centre_t, logs - centre_w))
    radial = np.linalg.norm(features / np.asarray([0.002, 0.002, 0.002, 0.01, 0.01, 0.01]), axis=1)
    median = float(np.median(radial))
    mad = float(np.median(np.abs(radial - median)))
    threshold = median + float(z_limit) * max(1.4826 * mad, 1.0e-9)
    keep = radial <= threshold
    if int(np.count_nonzero(keep)) < 3:
        raise ValueError("fewer than three inlier transforms remain")
    mean_t = np.mean(translations[keep], axis=0)
    mean_w = np.mean(logs[keep], axis=0)
    return Transform(reference @ rotation_exp(mean_w), mean_t)


def _transform_mean(transforms: Sequence[Transform]) -> Transform:
    """Return an SE(3) mean without rejecting any observations."""
    if not transforms:
        raise ValueError("at least one transform is required")
    translations = np.asarray([item.translation for item in transforms])
    reference = transforms[0].rotation
    logs = np.asarray([rotation_log(reference.T @ item.rotation) for item in transforms])
    return Transform(
        reference @ rotation_exp(np.mean(logs, axis=0)),
        np.mean(translations, axis=0),
    )


def aggregate_pose_pairs(frames: Sequence[PosePair]) -> PosePair:
    if len(frames) < 3:
        raise ValueError("at least three synchronized frames are required")
    return PosePair(
        robust_transform_mean([item.base_T_link5 for item in frames]),
        robust_transform_mean([item.camera_T_target for item in frames]),
    )


def board_poses(pairs: Sequence[PosePair], link5_T_camera: Transform) -> list[Transform]:
    return [item.base_T_link5 @ link5_T_camera @ item.camera_T_target for item in pairs]


def pose_dispersion(transforms: Sequence[Transform]) -> dict[str, float]:
    if len(transforms) < 2:
        raise ValueError("at least two transforms are required")
    if len(transforms) >= 3:
        try:
            mean = robust_transform_mean(transforms)
        except ValueError:
            # A tiny evaluation set (notably the three-pose bootstrap holdout)
            # cannot reject even one outlier while retaining the three samples
            # required by robust_transform_mean.  Dispersion must still be
            # measurable: score every observation around the unrejected mean
            # instead of making a valid hand-eye solution look like a solver
            # failure.
            mean = _transform_mean(transforms)
    else:
        mean = transforms[0]
    translation = np.asarray([np.linalg.norm(item.translation - mean.translation) for item in transforms])
    rotation = np.asarray([
        math.degrees(np.linalg.norm(rotation_log(mean.rotation.T @ item.rotation)))
        for item in transforms
    ])
    return {
        "translation_rms_m": float(math.sqrt(np.mean(translation * translation))),
        "translation_p95_m": float(np.percentile(translation, 95)),
        "translation_max_m": float(np.max(translation)),
        "rotation_rms_deg": float(math.sqrt(np.mean(rotation * rotation))),
        "rotation_p95_deg": float(np.percentile(rotation, 95)),
        "rotation_max_deg": float(np.max(rotation)),
    }


def solve_handeye(train: Sequence[PosePair], holdout: Sequence[PosePair] = ()) -> list[CalibrationCandidate]:
    if len(train) < 4:
        raise ValueError("at least four training pose pairs are required")
    robot_r = [item.base_T_link5.rotation for item in train]
    robot_t = [item.base_T_link5.translation.reshape((3, 1)) for item in train]
    target_r = [item.camera_T_target.rotation for item in train]
    target_t = [item.camera_T_target.translation.reshape((3, 1)) for item in train]
    candidates: list[CalibrationCandidate] = []
    for name, method in _SOLVERS.items():
        try:
            rotation, translation = cv2.calibrateHandEye(
                robot_r, robot_t, target_r, target_t, method=method
            )
            transform = Transform(rotation, np.asarray(translation).reshape(3))
            train_metrics = pose_dispersion(board_poses(train, transform))
            holdout_metrics = (
                pose_dispersion(board_poses(holdout, transform))
                if len(holdout) >= 2 else train_metrics
            )
        except (cv2.error, ValueError, FloatingPointError):
            continue
        candidates.append(CalibrationCandidate(
            name, transform,
            train_metrics["translation_rms_m"], train_metrics["rotation_rms_deg"],
            holdout_metrics["translation_p95_m"], holdout_metrics["rotation_p95_deg"],
        ))
    return sorted(
        candidates,
        key=lambda item: item.holdout_translation_p95_m + 0.01 * item.holdout_rotation_p95_deg,
    )


def rotation_observability(pairs: Sequence[PosePair]) -> dict[str, float]:
    if len(pairs) < 3:
        return {"minimum_relative_deg": 0.0, "maximum_relative_deg": 0.0, "sv_ratio": 0.0}
    vectors = []
    angles = []
    reference = pairs[0].base_T_link5.rotation
    for item in pairs[1:]:
        vector = rotation_log(reference.T @ item.base_T_link5.rotation)
        vectors.append(vector)
        angles.append(math.degrees(float(np.linalg.norm(vector))))
    singular = np.linalg.svd(np.asarray(vectors), compute_uv=False)
    ratio = 0.0 if singular[0] <= 1.0e-12 else float(singular[-1] / singular[0])
    return {
        "minimum_relative_deg": float(min(angles)),
        "maximum_relative_deg": float(max(angles)),
        "sv_ratio": ratio,
    }


def split_bootstrap_train_holdout(
    pairs: Sequence[PosePair], holdout_count: int, min_sv_ratio: float,
    min_maximum_rotation_deg: float, min_translation_span_m: float,
):
    """Select a time-stratified holdout without weakening training geometry."""
    pairs = list(pairs)
    if holdout_count < 1 or len(pairs) < holdout_count + 3:
        raise ValueError("bootstrap split requires a non-empty holdout and at least three training poses")
    candidate_indices = range(1, len(pairs))  # Anchor always remains in training.
    ideal = np.linspace(1, len(pairs) - 1, holdout_count + 2)[1:-1]
    best = None
    for holdout_indices in combinations(candidate_indices, holdout_count):
        selected = set(holdout_indices)
        train_indices = tuple(index for index in range(len(pairs)) if index not in selected)
        train = [pairs[index] for index in train_indices]
        observability = rotation_observability(train)
        positions = np.asarray([item.base_T_link5.translation for item in train])
        translation_span = float(np.max(np.linalg.norm(
            positions[:, None, :] - positions[None, :, :], axis=2
        )))
        if (
            observability["sv_ratio"] < min_sv_ratio
            or observability["maximum_relative_deg"] < min_maximum_rotation_deg
            or translation_span < min_translation_span_m
        ):
            continue
        # Prefer one validation sample from each phase of the ordered motion set.
        stratification_cost = float(np.sum(np.abs(np.asarray(holdout_indices) - ideal)))
        score = (stratification_cost, holdout_indices)
        if best is None or score < best[0]:
            best = (score, train_indices, holdout_indices, observability, translation_span)
    if best is None:
        full_observability = rotation_observability(pairs)
        raise ValueError(
            "no stratified bootstrap holdout preserves the required training geometry; "
            f"full_observability={full_observability}"
        )
    _, train_indices, holdout_indices, observability, translation_span = best
    return (
        [pairs[index] for index in train_indices],
        [pairs[index] for index in holdout_indices],
        {
            "train_indices": list(train_indices),
            "holdout_indices": list(holdout_indices),
            "train_observability": observability,
            "train_translation_span_m": translation_span,
        },
    )


def solver_consensus(candidates: Sequence[CalibrationCandidate]) -> dict[str, float]:
    if len(candidates) < 2:
        return {"translation_max_m": math.inf, "rotation_max_deg": math.inf}
    reference = candidates[0].link5_T_camera
    translation = []
    rotation = []
    for item in candidates[1:]:
        transform = item.link5_T_camera
        translation.append(float(np.linalg.norm(transform.translation - reference.translation)))
        rotation.append(math.degrees(float(np.linalg.norm(rotation_log(reference.rotation.T @ transform.rotation)))))
    return {"translation_max_m": max(translation), "rotation_max_deg": max(rotation)}


def look_at_rotation(camera_position, board_centre, board_x_axis, roll_deg: float) -> np.ndarray:
    position = np.asarray(camera_position, dtype=np.float64).reshape(3)
    centre = np.asarray(board_centre, dtype=np.float64).reshape(3)
    z_axis = centre - position
    z_axis /= np.linalg.norm(z_axis)
    projected = np.asarray(board_x_axis, dtype=np.float64).reshape(3)
    projected = projected - float(projected @ z_axis) * z_axis
    norm = float(np.linalg.norm(projected))
    if norm < 1.0e-8:
        raise ValueError("board x axis is parallel to camera optical axis")
    x_axis = projected / norm
    y_axis = np.cross(z_axis, x_axis)
    base = np.column_stack((x_axis, y_axis, z_axis))
    roll = math.radians(float(roll_deg))
    local = np.asarray([
        [math.cos(roll), -math.sin(roll), 0.0],
        [math.sin(roll), math.cos(roll), 0.0],
        [0.0, 0.0, 1.0],
    ])
    return base @ local


def look_at_roll_deg(camera_pose: Transform, board_centre, board_x_axis) -> float:
    """Return camera roll in the same absolute convention as look_at_rotation."""
    zero_roll = look_at_rotation(
        camera_pose.translation, board_centre, board_x_axis, 0.0,
    )
    relative = zero_roll.T @ camera_pose.rotation
    return math.degrees(math.atan2(float(relative[1, 0]), float(relative[0, 0])))


def generate_hemisphere(
    base_T_target: Transform,
    target_centre: Sequence[float],
    anchor_camera_position: Sequence[float],
    radii_m: Iterable[float],
    max_theta_deg: float,
    rolls_deg: Iterable[float],
) -> list[HemispherePose]:
    centre = base_T_target.point(target_centre)
    target_x = base_T_target.rotation[:, 0]
    normal = base_T_target.rotation[:, 2]
    if float((np.asarray(anchor_camera_position) - centre) @ normal) < 0.0:
        normal = -normal
    axis1 = target_x - float(target_x @ normal) * normal
    axis1 /= np.linalg.norm(axis1)
    axis2 = np.cross(normal, axis1)
    rings = ((10.0, 4), (20.0, 8), (30.0, 12), (40.0, 16), (45.0, 18))
    output: list[HemispherePose] = []
    for radius in radii_m:
        for theta_deg, count in rings:
            if theta_deg > float(max_theta_deg) + 1.0e-9:
                continue
            theta = math.radians(theta_deg)
            for index in range(count):
                phi_deg = 360.0 * index / count + (theta_deg % 20.0) * 0.5
                phi = math.radians(phi_deg)
                direction = (
                    math.sin(theta) * math.cos(phi) * axis1
                    + math.sin(theta) * math.sin(phi) * axis2
                    + math.cos(theta) * normal
                )
                camera_position = centre + float(radius) * direction
                for roll_deg in rolls_deg:
                    output.append(HemispherePose(
                        float(radius), theta_deg, phi_deg, float(roll_deg),
                        Transform(
                            look_at_rotation(camera_position, centre, axis1, roll_deg),
                            camera_position,
                        ),
                    ))
    return output


def target_to_link5_pose(camera_pose: Transform, link5_T_camera: Transform) -> Transform:
    return camera_pose @ link5_T_camera.inverse()


def calibration_document(
    name: str,
    base_frame: str,
    link5_frame: str,
    camera_frame: str,
    marker_frame: str,
    transform: Transform,
) -> dict:
    x, y, z, w = rotation_to_quaternion(transform.rotation)
    return {
        "parameters": {
            "name": name,
            "calibration_type": "eye_in_hand",
            "robot_base_frame": base_frame,
            "robot_effector_frame": link5_frame,
            "tracking_base_frame": camera_frame,
            "tracking_marker_frame": marker_frame,
            "freehand_robot_movement": True,
            "move_group_namespace": "/piperh",
            "move_group": "arm",
        },
        "transform": {
            "translation": {"x": float(transform.translation[0]), "y": float(transform.translation[1]), "z": float(transform.translation[2])},
            "rotation": {"x": x, "y": y, "z": z, "w": w},
        },
    }


def atomic_write_yaml(path: Path | str, document: dict) -> None:
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{destination.name}.", suffix=".tmp", dir=str(destination.parent)
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            yaml.safe_dump(document, stream, sort_keys=False, allow_unicode=True)
            stream.flush()
            os.fsync(stream.fileno())
        with open(temporary, "r", encoding="utf-8") as stream:
            loaded = yaml.safe_load(stream)
        if loaded != document:
            raise RuntimeError("calibration verification read-back differs")
        os.replace(temporary, destination)
        directory_fd = os.open(destination.parent, os.O_RDONLY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
