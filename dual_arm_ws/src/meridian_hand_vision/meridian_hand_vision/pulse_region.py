"""Estimate and stabilize an approximate radial-pulse region from RGB-D data."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class CameraIntrinsics:
    """Pinhole intrinsics for the registered color image."""

    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    @classmethod
    def from_camera_info(cls, message) -> "CameraIntrinsics | None":
        if (
            len(message.k) < 6
            or message.width <= 0
            or message.height <= 0
            or message.k[0] <= 0.0
            or message.k[4] <= 0.0
        ):
            return None
        return cls(
            fx=float(message.k[0]),
            fy=float(message.k[4]),
            cx=float(message.k[2]),
            cy=float(message.k[5]),
            width=int(message.width),
            height=int(message.height),
        )


@dataclass(frozen=True)
class PulseRegion:
    """Approximate radial-pulse region in image and camera coordinates."""

    pixel: tuple[int, int]
    point_m: tuple[float, float, float]
    radius_m: float


@dataclass(frozen=True)
class ArmDirection:
    """Approximate palm-to-forearm axis in the optical camera frame."""

    vector: tuple[float, float, float]


def _landmark_pixels(landmarks, width: int, height: int) -> list[tuple[int, int]]:
    return [
        (
            int(np.clip(point.x, 0.0, 1.0) * (width - 1)),
            int(np.clip(point.y, 0.0, 1.0) * (height - 1)),
        )
        for point in landmarks
    ]


def pulse_pixel(landmarks, width: int, height: int) -> tuple[int, int]:
    """Place a target just proximal to the wrist on its anatomical thumb side.

    MediaPipe landmark indices identify the thumb side directly, so this works
    for either hand without relying on a possibly mirrored handedness label.
    The location is an interaction region, not a medical landmark diagnosis.
    """
    points = np.asarray(_landmark_pixels(landmarks, width, height), dtype=np.float64)
    wrist = points[0]
    palm_center = np.mean(points[[5, 9, 13, 17]], axis=0)
    palm_to_forearm = wrist - palm_center
    longitudinal_norm = float(np.linalg.norm(palm_to_forearm))
    if longitudinal_norm < 1.0:
        raise ValueError("hand landmarks do not define a usable wrist direction")
    palm_to_forearm /= longitudinal_norm

    # Index MCP (5) is on the anatomical radial/thumb side and pinky MCP (17)
    # is on the ulnar side. Move slightly toward the thumb and slightly onto
    # the forearm so the marker does not sit in the palm.
    radial = points[5] - points[17]
    palm_width = float(np.linalg.norm(radial))
    if palm_width < 8.0:
        raise ValueError("hand landmarks do not define a usable palm width")
    radial /= palm_width
    target = wrist + 0.18 * palm_width * palm_to_forearm + 0.20 * palm_width * radial
    return (
        int(np.clip(round(target[0]), 0, width - 1)),
        int(np.clip(round(target[1]), 0, height - 1)),
    )


def median_depth_at(
    depth_m: np.ndarray,
    pixel: tuple[int, int],
    radius_px: int,
    minimum_m: float = 0.10,
    maximum_m: float = 2.0,
) -> float | None:
    """Return a robust local depth after rejecting invalid and distant pixels."""
    height, width = depth_m.shape
    x, y = pixel
    radius_px = max(1, int(radius_px))
    x0, x1 = max(0, x - radius_px), min(width, x + radius_px + 1)
    y0, y1 = max(0, y - radius_px), min(height, y + radius_px + 1)
    patch = depth_m[y0:y1, x0:x1]
    valid = patch[
        np.isfinite(patch) & (patch >= minimum_m) & (patch <= maximum_m)
    ]
    if valid.size < 5:
        return None
    median = float(np.median(valid))
    close = valid[np.abs(valid - median) <= 0.025]
    return float(np.median(close)) if close.size >= 5 else None


def deproject_pixel(
    pixel: tuple[int, int],
    depth_m: float,
    intrinsics: CameraIntrinsics,
    image_width: int,
    image_height: int,
    mirrored: bool = False,
) -> tuple[float, float, float]:
    """Deproject a registered color pixel into its optical camera frame."""
    u, v = pixel
    if mirrored:
        u = image_width - 1 - u

    scale_x = intrinsics.width / float(image_width)
    scale_y = intrinsics.height / float(image_height)
    u_info = (u + 0.5) * scale_x - 0.5
    v_info = (v + 0.5) * scale_y - 0.5
    return (
        (u_info - intrinsics.cx) * depth_m / intrinsics.fx,
        (v_info - intrinsics.cy) * depth_m / intrinsics.fy,
        depth_m,
    )


def estimate_pulse_region(
    landmarks,
    depth_m: np.ndarray,
    intrinsics: CameraIntrinsics,
    mirrored: bool = False,
) -> PulseRegion | None:
    """Estimate one approximate pulse region from a single registered frame."""
    region, _reason, _details = estimate_pulse_region_diagnostic(
        landmarks, depth_m, intrinsics, mirrored
    )
    return region


def estimate_pulse_region_diagnostic(
    landmarks,
    depth_m: np.ndarray,
    intrinsics: CameraIntrinsics,
    mirrored: bool = False,
) -> tuple[PulseRegion | None, str, dict]:
    """Estimate a region and preserve the exact rejection stage for diagnostics."""
    height, width = depth_m.shape
    try:
        pixel = pulse_pixel(landmarks, width, height)
    except ValueError as error:
        return None, "INVALID_3D_POINT", {"detail": str(error)}
    points = _landmark_pixels(landmarks, width, height)
    palm_width_px = max(8, int(np.linalg.norm(np.asarray(points[5]) - points[17])))
    depth = median_depth_at(depth_m, pixel, max(3, palm_width_px // 9))
    if depth is None:
        return None, "INVALID_DEPTH", {"raw_pixel_uv": pixel, "raw_depth": None}
    point = deproject_pixel(
        pixel, depth, intrinsics, width, height, mirrored=mirrored
    )
    if not all(math.isfinite(value) for value in point):
        return None, "INVALID_3D_POINT", {
            "raw_pixel_uv": pixel, "raw_depth": depth, "camera_xyz": point,
        }
    radius_m = max(0.006, min(0.018, depth * palm_width_px / intrinsics.fx * 0.18))
    return (
        PulseRegion(pixel=pixel, point_m=point, radius_m=radius_m),
        "NONE",
        {"raw_pixel_uv": pixel, "raw_depth": depth, "camera_xyz": point},
    )


def estimate_arm_direction(
    landmarks,
    depth_m: np.ndarray,
    intrinsics: CameraIntrinsics,
    mirrored: bool = False,
) -> ArmDirection | None:
    """Estimate a 3D forearm axis from palm center toward the wrist.

    MediaPipe has no forearm landmark. This is therefore an approximate axis
    continuing the hand longitudinal direction, with registered depth at both
    endpoints. Callers must treat d and -d as the same anatomical axis.
    """
    height, width = depth_m.shape
    points = _landmark_pixels(landmarks, width, height)
    wrist = points[0]
    palm = tuple(
        int(round(value))
        for value in np.mean(np.asarray([points[5], points[9], points[13], points[17]]), axis=0)
    )
    palm_width = max(8, int(np.linalg.norm(np.asarray(points[5]) - points[17])))
    radius = max(3, palm_width // 9)
    wrist_depth = median_depth_at(depth_m, wrist, radius)
    palm_depth = median_depth_at(depth_m, palm, radius)
    if wrist_depth is None or palm_depth is None:
        return None
    wrist_xyz = np.asarray(
        deproject_pixel(wrist, wrist_depth, intrinsics, width, height, mirrored),
        dtype=np.float64,
    )
    palm_xyz = np.asarray(
        deproject_pixel(palm, palm_depth, intrinsics, width, height, mirrored),
        dtype=np.float64,
    )
    vector = wrist_xyz - palm_xyz
    length = float(np.linalg.norm(vector))
    if not np.isfinite(length) or length < 0.015:
        return None
    vector /= length
    return ArmDirection(tuple(float(value) for value in vector))


class StableArmDirectionFilter:
    """Stabilize an unoriented 3D axis while accepting d/-d equivalence."""

    def __init__(self, window_size: int = 12, maximum_angle_deg: float = 12.0) -> None:
        if window_size < 2 or maximum_angle_deg <= 0.0:
            raise ValueError("direction filter parameters must be positive")
        self._samples: deque[np.ndarray] = deque(maxlen=window_size)
        self.maximum_angle_deg = float(maximum_angle_deg)

    def clear(self) -> None:
        self._samples.clear()

    def update(self, direction: ArmDirection | None) -> ArmDirection | None:
        if direction is None:
            self.clear()
            return None
        vector = np.asarray(direction.vector, dtype=np.float64)
        length = float(np.linalg.norm(vector))
        if not np.isfinite(length) or length < 1e-6:
            self.clear()
            return None
        vector /= length
        if self._samples and float(np.dot(vector, self._samples[0])) < 0.0:
            vector = -vector
        self._samples.append(vector)
        if len(self._samples) < self._samples.maxlen:
            return None
        center = np.mean(np.asarray(self._samples), axis=0)
        center_length = float(np.linalg.norm(center))
        if not np.isfinite(center_length) or center_length < 1e-6:
            return None
        center /= center_length
        minimum_dot = min(
            abs(float(np.dot(sample, center))) for sample in self._samples
        )
        spread = math.degrees(math.acos(float(np.clip(minimum_dot, -1.0, 1.0))))
        if spread > self.maximum_angle_deg:
            return None
        return ArmDirection(tuple(float(value) for value in center))


class StablePulseFilter:
    """P95 spatial gate over real samples; camera FPS is never a gate."""

    def __init__(
        self,
        window_sec: float = 0.6,
        stable_duration_sec: float = 0.0,
        min_samples: int = 3,
        stable_radius_m: float = 0.005,
        stable_hold_radius_m: float = 0.008,
        unstable_hold_sec: float = 0.2,
        dropout_grace_sec: float = 0.5,
        soft_timeout_sec: float = 1.5,
        hard_timeout_sec: float = 2.0,
        hard_jump_m: float = 0.015,
    ) -> None:
        if (window_sec <= 0.0 or stable_duration_sec < 0.0 or min_samples < 3
                or stable_radius_m <= 0.0
                or stable_hold_radius_m < stable_radius_m or unstable_hold_sec <= 0.0
                or not 0.0 < dropout_grace_sec < soft_timeout_sec < hard_timeout_sec
                or hard_jump_m <= stable_hold_radius_m):
            raise ValueError("invalid time-based pulse stability parameters")
        self.window_sec = float(window_sec)
        self.stable_duration_sec = float(stable_duration_sec)
        self.min_samples = int(min_samples)
        self.stable_radius_m = float(stable_radius_m)
        self.stable_hold_radius_m = float(stable_hold_radius_m)
        self.unstable_hold_sec = float(unstable_hold_sec)
        self.dropout_grace_sec = float(dropout_grace_sec)
        self.soft_timeout_sec = float(soft_timeout_sec)
        self.hard_timeout_sec = float(hard_timeout_sec)
        self.hard_jump_m = float(hard_jump_m)
        self._samples: deque[tuple[float, PulseRegion]] = deque()
        self._stable_since: float | None = None
        self._unstable_since: float | None = None
        self._last_valid_time: float | None = None
        self.frozen_region: PulseRegion | None = None
        self.stable_latched = False
        self._hand_identity: str | None = None
        self._jitter_m = 0.0
        self._drift_m = 0.0
        self._center_m: tuple[float, float, float] | None = None
        self._oldest_distance_m = 0.0
        self.acquisition_state = "WAIT_DETECTION"
        self.reset_count = 0
        self.last_reset_reason = "NONE"
        self.consecutive_missing_frames = 0
        self.reacquire_distance_m: float | None = None
        self.reacquire_result = "NONE"
        self.last_gap_sec: float | None = None
        self._last_sample_rate_hz = 0.0

    def observe_gap(self, timestamp: float) -> None:
        """Advance timeout state without inventing a missing frame or sample."""
        if not math.isfinite(timestamp):
            raise ValueError("pulse gap timestamp must be finite")
        self._expire_if_hard_timeout(timestamp)

    def clear(self, reason: str = "NEW_TARGET") -> None:
        if self._samples or self._last_valid_time is not None or self.stable_latched:
            self.reset_count += 1
            self.last_reset_reason = reason
        self._samples.clear()
        self._stable_since = None
        self._unstable_since = None
        self._last_valid_time = None
        self.frozen_region = None
        self.stable_latched = False
        self._hand_identity = None
        self._jitter_m = 0.0
        self._drift_m = 0.0
        self._center_m = None
        self._oldest_distance_m = 0.0
        self.acquisition_state = "WAIT_DETECTION"
        self.consecutive_missing_frames = 0
        self.reacquire_distance_m = None
        self.reacquire_result = "NONE"
        self.last_gap_sec = None
        self._last_sample_rate_hz = 0.0

    def _prune(self, now: float) -> None:
        while self._samples and now - self._samples[0][0] > self.window_sec:
            self._samples.popleft()

    def _expire_if_hard_timeout(self, now: float) -> None:
        if (
            self._last_valid_time is not None
            and now - self._last_valid_time > self.hard_timeout_sec
        ):
            self.clear("TARGET_HARD_TIMEOUT")

    def _window_metrics(self) -> tuple[np.ndarray, np.ndarray]:
        xyz = np.asarray(
            [sample.point_m for _, sample in self._samples], dtype=np.float64
        )
        center = np.median(xyz, axis=0)
        distances = np.linalg.norm(xyz - center, axis=1)
        self._center_m = tuple(float(value) for value in center)
        self._jitter_m = float(np.percentile(distances, 95))
        self._oldest_distance_m = float(distances[0])
        self._drift_m = float(np.linalg.norm(xyz[-1] - center))
        return center, distances

    def _release_stable_latch(self, reason: str) -> None:
        if self.stable_latched:
            self.reset_count += 1
            self.last_reset_reason = reason
        self.stable_latched = False
        self.frozen_region = None
        self._stable_since = None
        self._unstable_since = None
        self.acquisition_state = "REACQUIRING"

    def update(
        self, region: PulseRegion | None, timestamp: float,
        hand_identity: str | None = None,
    ) -> PulseRegion | None:
        if not math.isfinite(timestamp):
            raise ValueError("pulse sample timestamp must be finite")
        self._expire_if_hard_timeout(timestamp)
        if self._last_valid_time is not None and timestamp < self._last_valid_time:
            # Reject an out-of-order observation without erasing good history.
            self.reacquire_result = "OUT_OF_ORDER_SAMPLE"
            return None
        if region is None:
            self.consecutive_missing_frames += 1
            return None
        point = np.asarray(region.point_m, dtype=np.float64)
        if point.shape != (3,) or not np.all(np.isfinite(point)):
            raise ValueError("pulse 3D point must be finite XYZ")
        if hand_identity and self._hand_identity and hand_identity != self._hand_identity:
            self.clear("HAND_IDENTITY_CHANGED")
        gap_sec = (
            timestamp - self._last_valid_time
            if self._last_valid_time is not None else None
        )
        prior_missing = self.consecutive_missing_frames
        self.last_gap_sec = gap_sec
        self.consecutive_missing_frames = 0
        self._prune(timestamp)
        if hand_identity:
            self._hand_identity = hand_identity
        prior_rate_hz = self._last_sample_rate_hz
        self._samples.append((timestamp, region))
        self._last_valid_time = timestamp
        center, _distances = self._window_metrics()
        ordinary_interval_limit = max(
            self.dropout_grace_sec / 4.0,
            2.0 / max(prior_rate_hz, 1.0),
        )
        if (
            len(self._samples) >= 2 and gap_sec is not None
            and gap_sec <= min(self.window_sec, ordinary_interval_limit)
        ):
            span = self._samples[-1][0] - self._samples[0][0]
            if span > 0.0:
                self._last_sample_rate_hz = (len(self._samples) - 1) / span
        if self.stable_latched and self.frozen_region is not None:
            self.reacquire_distance_m = float(np.linalg.norm(
                center - np.asarray(self.frozen_region.point_m)
            ))
            raw_distance_m = float(np.linalg.norm(
                point - np.asarray(self.frozen_region.point_m)
            ))
            motion_excess = self.reacquire_distance_m > self.stable_hold_radius_m
            spatial_excess = self._jitter_m > self.stable_hold_radius_m
            if (self._jitter_m > self.hard_jump_m
                    or self.reacquire_distance_m > self.hard_jump_m
                    or raw_distance_m > self.hard_jump_m):
                distance = self.reacquire_distance_m
                self.clear("TARGET_SPATIAL_DRIFT")
                self.reacquire_distance_m = distance
                self.reacquire_result = "TARGET_CHANGED"
                self.acquisition_state = "NEW_TARGET_ACQUISITION"
                # Keep this real sample as the first candidate of the new target.
                self._samples.append((timestamp, region))
                self._last_valid_time = timestamp
                if hand_identity:
                    self._hand_identity = hand_identity
                self._window_metrics()
                return None
            if spatial_excess or motion_excess:
                if self._unstable_since is None:
                    self._unstable_since = timestamp
                self.reacquire_result = "OBSERVE_SPATIAL_DRIFT"
                if timestamp - self._unstable_since >= self.unstable_hold_sec:
                    self._release_stable_latch("TARGET_SPATIAL_DRIFT")
                return None
            self._unstable_since = None
            self.reacquire_result = (
                "REACQUIRED_SAME_TARGET"
                if (
                    gap_sec is not None
                    and gap_sec > ordinary_interval_limit
                )
                or prior_missing else "CONTINUOUS_TARGET"
            )
            self.acquisition_state = "TARGET_STABLE"
            return PulseRegion(region.pixel, self._center_m, region.radius_m)
        self.acquisition_state = "NEW_TARGET_ACQUISITION"
        if len(self._samples) < self.min_samples:
            return None
        if self._jitter_m > self.stable_radius_m:
            self.acquisition_state = "ACQUISITION_CONVERGING"
            return None
        self._stable_since = timestamp
        pixels = np.asarray([sample.pixel for _, sample in self._samples], dtype=np.float64)
        radius = float(np.median([sample.radius_m for _, sample in self._samples]))
        stable_region = PulseRegion(
            pixel=tuple(int(round(value)) for value in np.median(pixels, axis=0)),
            point_m=tuple(float(value) for value in center),
            radius_m=radius,
        )
        self.frozen_region = stable_region
        self.stable_latched = True
        self.acquisition_state = "TARGET_STABLE"
        self.reacquire_result = "NEW_STABLE_TARGET"
        return stable_region

    def diagnostics(self, now: float) -> dict:
        self._expire_if_hard_timeout(now)
        recent = [item for item in self._samples if now - item[0] <= self.window_sec]
        sample_count = len(recent)
        span = recent[-1][0] - recent[0][0] if sample_count >= 2 else 0.0
        if span > 0.0:
            self._last_sample_rate_hz = (sample_count - 1) / span
        age = now - self._last_valid_time if self._last_valid_time is not None else None
        if age is None:
            target_state = (
                "TARGET_LOST" if self.last_reset_reason == "TARGET_HARD_TIMEOUT"
                else "TARGET_TRACKING"
            )
        elif age <= self.dropout_grace_sec:
            target_state = "TARGET_STABLE" if self.stable_latched else "TARGET_TRACKING"
        elif age <= self.soft_timeout_sec:
            target_state = "TARGET_TEMPORARILY_STALE"
        elif age <= self.hard_timeout_sec:
            target_state = "WAIT_TARGET_RECOVERY"
        else:
            target_state = "TARGET_LOST"
        if (self.reacquire_result == "OBSERVE_SPATIAL_DRIFT"
                and not self.stable_latched and age is not None):
            target_state = "TARGET_OBSERVING_DRIFT"
        return {
            "vision_sample_rate_hz": (sample_count - 1) / span if span > 0.0 else 0.0,
            "vision_valid_samples": sample_count,
            "spatial_jitter_mm": self._jitter_m * 1000.0,
            "spatial_drift_mm": self._drift_m * 1000.0,
            "target_jitter_mm": self._jitter_m * 1000.0,
            "target_motion_mm": (
                self.reacquire_distance_m * 1000.0
                if self.reacquire_distance_m is not None else self._drift_m * 1000.0
            ),
            "target_age_ms": age * 1000.0 if age is not None else None,
            "stable_duration_ms": (
                max(0.0, now - self._stable_since) * 1000.0
                if self._stable_since is not None else 0.0
            ),
            "stable_required_ms": 0.0,
            "target_stable": self.stable_latched,
            "acquisition_state": self.acquisition_state,
            "window_oldest_age_ms": (
                (now - recent[0][0]) * 1000.0 if recent else None
            ),
            "window_valid_samples": sample_count,
            "window_center_xyz": self._center_m,
            "jitter_p95_mm": self._jitter_m * 1000.0,
            "oldest_sample_distance_mm": self._oldest_distance_m * 1000.0,
            "stable_reset_count": self.reset_count,
            "last_stable_reset_reason": self.last_reset_reason,
            "stable_reset_reason": self.last_reset_reason,
            "last_valid_target_age_ms": age * 1000.0 if age is not None else None,
            "dropout_grace_ms": self.dropout_grace_sec * 1000.0,
            "consecutive_missing_frames": self.consecutive_missing_frames,
            "missing_frames_est": (
                max(self.consecutive_missing_frames, round(age * self._last_sample_rate_hz))
                if age is not None else 0
            ),
            "temporary_stale": target_state in (
                "TARGET_TEMPORARILY_STALE", "WAIT_TARGET_RECOVERY"
            ),
            "stable_latched": self.stable_latched,
            "frozen_target": self.frozen_region.point_m if self.frozen_region else None,
            "reacquire_distance_mm": (
                self.reacquire_distance_m * 1000.0
                if self.reacquire_distance_m is not None else None
            ),
            "reacquire_result": self.reacquire_result,
            "target_tracking_state": target_state,
            "gap_action": (
                "KEEP_STABLE" if self.stable_latched and age is not None
                and age <= self.dropout_grace_sec else
                "WAIT_FRESH_MEASUREMENT" if self.stable_latched else
                "TRACK_NEW_TARGET"
            ),
        }


def point_distance(first: tuple[float, float, float], second: tuple[float, float, float]) -> float:
    """Small dependency-free helper used by tests and diagnostics."""
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(first, second)))
