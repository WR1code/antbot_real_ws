from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import math

import cv2
import numpy as np

from .pulse_region import CameraIntrinsics, deproject_pixel


@dataclass(frozen=True)
class WristTemporalDiagnostic:
    raw_wrist_pixel: tuple[float, float] | None
    raw_wrist_3d: tuple[float, float, float] | None
    raw_wrist_available: bool
    raw_wrist_reason: str
    depth_seed_pixel: tuple[int, int] | None
    depth_seed_3d: tuple[float, float, float] | None
    depth_seed_valid: bool
    wrist_seed_pixel_distance_px: float | None
    wrist_seed_3d_distance_mm: float | None
    filtered_wrist_pixel: tuple[float, float] | None
    filtered_wrist_3d: tuple[float, float, float] | None
    filter_state: str
    filter_age_frames: int
    held_age_ms: float | None
    raw_displacement_px: float | None
    filtered_displacement_px: float | None
    raw_displacement_mm: float | None
    filtered_displacement_mm: float | None
    raw_velocity_mm_s: float | None
    raw_filtered_error_mm: float | None
    pixel_jump: bool
    jump_3d: bool
    velocity_high: bool
    raw_filtered_error_high: bool

    def diagnostics(self) -> dict:
        return {
            "raw_wrist_pixel": self.raw_wrist_pixel,
            "raw_wrist_3d": self.raw_wrist_3d,
            "raw_wrist_available": self.raw_wrist_available,
            "raw_wrist_reason": self.raw_wrist_reason,
            "depth_seed_pixel": self.depth_seed_pixel,
            "depth_seed_3d": self.depth_seed_3d,
            "depth_seed_valid": self.depth_seed_valid,
            "wrist_seed_pixel_distance_px": self.wrist_seed_pixel_distance_px,
            "wrist_seed_3d_distance_mm": self.wrist_seed_3d_distance_mm,
            "filtered_wrist_pixel": self.filtered_wrist_pixel,
            "filtered_wrist_3d": self.filtered_wrist_3d,
            "filter_state": self.filter_state,
            "filter_age_frames": self.filter_age_frames,
            "held_age_ms": self.held_age_ms,
            "raw_displacement_px": self.raw_displacement_px,
            "filtered_displacement_px": self.filtered_displacement_px,
            "raw_displacement_mm": self.raw_displacement_mm,
            "filtered_displacement_mm": self.filtered_displacement_mm,
            "raw_velocity_mm_s": self.raw_velocity_mm_s,
            "raw_filtered_error_mm": self.raw_filtered_error_mm,
            "threshold_flags": {
                "pixel_jump": self.pixel_jump,
                "jump_3d": self.jump_3d,
                "velocity_high": self.velocity_high,
                "raw_filtered_error_high": self.raw_filtered_error_high,
            },
        }


@dataclass(frozen=True)
class MaskRobustnessDiagnostic:
    mask_area_px: int
    connected_point_count: int
    valid_depth_count: int
    valid_depth_ratio: float
    baseline_mask_area: float | None
    baseline_connected_points: float | None
    baseline_depth_ratio: float | None
    baseline_branch_length: float | None
    area_ratio_to_baseline: float | None
    connected_ratio_to_baseline: float | None
    branch_length_ratio_to_baseline: float | None
    raw_wrist_to_mask_distance_px: float | None
    depth_seed_to_raw_wrist_distance_px: float | None
    raw_wrist_to_cloud_distance_mm: float | None
    status: str

    def diagnostics(self) -> dict:
        return {
            "mask_area_px": self.mask_area_px,
            "connected_point_count": self.connected_point_count,
            "valid_depth_count": self.valid_depth_count,
            "valid_depth_ratio": self.valid_depth_ratio,
            "baseline_mask_area": self.baseline_mask_area,
            "baseline_connected_points": self.baseline_connected_points,
            "baseline_depth_ratio": self.baseline_depth_ratio,
            "baseline_branch_length": self.baseline_branch_length,
            "area_ratio_to_baseline": self.area_ratio_to_baseline,
            "connected_ratio_to_baseline": self.connected_ratio_to_baseline,
            "branch_length_ratio_to_baseline": self.branch_length_ratio_to_baseline,
            "raw_wrist_to_mask_distance_px": self.raw_wrist_to_mask_distance_px,
            "depth_seed_to_raw_wrist_distance_px": (
                self.depth_seed_to_raw_wrist_distance_px
            ),
            "raw_wrist_to_cloud_distance_mm": self.raw_wrist_to_cloud_distance_mm,
            "status": self.status,
        }


@dataclass(frozen=True)
class BranchSurvivalDiagnostic:
    branch_count: int
    selected_branch_id: int | None
    selected_track_id: int | None
    effective_length_m: float | None
    continuous_span_m: float | None
    valid_section_count: int
    inlier_section_count: int
    branch_reason: str
    estimator_reason: str
    active_track_id: int | None
    active_track_missing: bool
    longest_branch_length: float | None
    longest_branch_reason: str | None

    def diagnostics(self) -> dict:
        return {
            "branch_count": self.branch_count,
            "selected_branch_id": self.selected_branch_id,
            "selected_track_id": self.selected_track_id,
            "effective_length_m": self.effective_length_m,
            "continuous_span_m": self.continuous_span_m,
            "valid_section_count": self.valid_section_count,
            "inlier_section_count": self.inlier_section_count,
            "branch_reason": self.branch_reason,
            "estimator_reason": self.estimator_reason,
            "active_track_id": self.active_track_id,
            "active_track_missing": self.active_track_missing,
            "longest_branch_length": self.longest_branch_length,
            "longest_branch_reason": self.longest_branch_reason,
        }


@dataclass(frozen=True)
class WristFailureAttribution:
    root_cause: str
    confidence: float
    first_abnormal_stage: str
    downstream_effects: tuple[str, ...]
    evidence: dict
    wrist_first_bad_frame: int | None
    mask_first_bad_frame: int | None
    branch_first_bad_frame: int | None
    depth_seed_first_bad_frame: int | None

    def diagnostics(self) -> dict:
        return {
            "root_cause": self.root_cause,
            "confidence": self.confidence,
            "first_abnormal_stage": self.first_abnormal_stage,
            "downstream_effects": self.downstream_effects,
            "evidence": self.evidence,
            "wrist_first_bad_frame": self.wrist_first_bad_frame,
            "mask_first_bad_frame": self.mask_first_bad_frame,
            "branch_first_bad_frame": self.branch_first_bad_frame,
            "depth_seed_first_bad_frame": self.depth_seed_first_bad_frame,
        }


@dataclass
class WristTemporalShadowState:
    pixel_window: deque = field(default_factory=lambda: deque(maxlen=5))
    point_window: deque = field(default_factory=lambda: deque(maxlen=5))
    filtered_pixel: np.ndarray | None = None
    filtered_point: np.ndarray | None = None
    previous_raw_pixel: np.ndarray | None = None
    previous_raw_point: np.ndarray | None = None
    previous_filtered_pixel: np.ndarray | None = None
    previous_filtered_point: np.ndarray | None = None
    previous_raw_time: float | None = None
    previous_filter_time: float | None = None
    last_valid_time: float | None = None
    filter_age_frames: int = 0
    frame_index: int = 0
    mask_area_baseline: deque = field(default_factory=lambda: deque(maxlen=30))
    connected_baseline: deque = field(default_factory=lambda: deque(maxlen=30))
    depth_ratio_baseline: deque = field(default_factory=lambda: deque(maxlen=30))
    branch_length_baseline: deque = field(default_factory=lambda: deque(maxlen=30))
    # Current frame T plus the ten preceding frames T-10 ... T.
    anomaly_window: deque = field(default_factory=lambda: deque(maxlen=11))


def _tuple_or_none(value: np.ndarray | None):
    if value is None:
        return None
    return tuple(float(item) for item in value)


def _median_depth_at(
    depth_m: np.ndarray, pixel: tuple[float, float], radius: int = 4
) -> float | None:
    height, width = depth_m.shape
    column = int(round(pixel[0]))
    row = int(round(pixel[1]))
    x0, x1 = max(0, column - radius), min(width, column + radius + 1)
    y0, y1 = max(0, row - radius), min(height, row + radius + 1)
    patch = depth_m[y0:y1, x0:x1]
    valid = patch[np.isfinite(patch) & (patch >= 0.10) & (patch <= 2.0)]
    return float(np.median(valid)) if valid.size else None


def _sample_raw_wrist(
    wrist_landmark,
    depth_m: np.ndarray,
    intrinsics: CameraIntrinsics | None,
    mirrored: bool,
) -> tuple[np.ndarray | None, np.ndarray | None, str]:
    if wrist_landmark is None:
        return None, None, "NO_DETECTION"
    if depth_m.ndim != 2 or depth_m.size == 0:
        return None, None, "INVALID_CAMERA_INFO"
    try:
        x = float(wrist_landmark.x)
        y = float(wrist_landmark.y)
    except (AttributeError, TypeError, ValueError):
        return None, None, "LANDMARK_INVALID"
    if not math.isfinite(x + y):
        return None, None, "LANDMARK_INVALID"
    height, width = depth_m.shape
    pixel = np.asarray((
        np.clip(x, 0.0, 1.0) * (width - 1),
        np.clip(y, 0.0, 1.0) * (height - 1),
    ), dtype=np.float64)
    if intrinsics is None:
        return pixel, None, "INVALID_CAMERA_INFO"
    depth = _median_depth_at(depth_m, tuple(pixel))
    if depth is None:
        return pixel, None, "WRIST_DEPTH_INVALID"
    point = np.asarray(deproject_pixel(
        (int(round(pixel[0])), int(round(pixel[1]))),
        depth,
        intrinsics,
        width,
        height,
        mirrored=mirrored,
    ), dtype=np.float64)
    if not np.all(np.isfinite(point)):
        return pixel, None, "WRIST_3D_INVALID"
    return pixel, point, "NONE"


def _distance_to_mask(mask: np.ndarray, pixel: np.ndarray | None) -> float | None:
    if pixel is None or mask.ndim != 2 or not np.any(mask > 0):
        return None
    distance = cv2.distanceTransform(
        np.uint8(mask == 0), cv2.DIST_L2, cv2.DIST_MASK_PRECISE
    )
    row = int(np.clip(round(float(pixel[1])), 0, mask.shape[0] - 1))
    column = int(np.clip(round(float(pixel[0])), 0, mask.shape[1] - 1))
    return float(distance[row, column])


def _distance_to_cloud_mm(
    mask: np.ndarray,
    depth_m: np.ndarray,
    raw_point: np.ndarray | None,
    intrinsics: CameraIntrinsics | None,
    mirrored: bool,
) -> float | None:
    if raw_point is None or intrinsics is None or mask.shape != depth_m.shape:
        return None
    valid = (
        (mask > 0) & np.isfinite(depth_m) & (depth_m >= 0.10) & (depth_m <= 2.0)
    )
    rows, columns = np.nonzero(valid)
    if rows.size == 0:
        return None
    stride = max(1, int(math.ceil(rows.size / 5000)))
    rows = rows[::stride]
    columns = columns[::stride]
    z = depth_m[rows, columns].astype(np.float64)
    height, width = depth_m.shape
    display_columns = width - 1.0 - columns if mirrored else columns
    scale_x = intrinsics.width / float(width)
    scale_y = intrinsics.height / float(height)
    u = (display_columns + 0.5) * scale_x - 0.5
    v = (rows + 0.5) * scale_y - 0.5
    x = (u - intrinsics.cx) * z / intrinsics.fx
    y = (v - intrinsics.cy) * z / intrinsics.fy
    cloud = np.column_stack((x, y, z))
    return float(np.min(np.linalg.norm(cloud - raw_point, axis=1)) * 1000.0)


def _update_temporal_filter(
    state: WristTemporalShadowState,
    raw_pixel: np.ndarray | None,
    raw_point: np.ndarray | None,
    timestamp_sec: float,
    *,
    tau_sec: float = 0.20,
    hold_timeout_sec: float = 0.50,
) -> WristTemporalDiagnostic:
    previous_filtered_pixel = (
        state.filtered_pixel.copy() if state.filtered_pixel is not None else None
    )
    previous_filtered_point = (
        state.filtered_point.copy() if state.filtered_point is not None else None
    )
    raw_displacement_px = raw_displacement_mm = raw_velocity = None
    if raw_pixel is not None and state.previous_raw_pixel is not None:
        raw_displacement_px = float(np.linalg.norm(
            raw_pixel - state.previous_raw_pixel
        ))
    if raw_point is not None and state.previous_raw_point is not None:
        raw_displacement_mm = float(np.linalg.norm(
            raw_point - state.previous_raw_point
        ) * 1000.0)
        if state.previous_raw_time is not None:
            delta_time = max(1e-3, timestamp_sec - state.previous_raw_time)
            raw_velocity = raw_displacement_mm / delta_time
    pixel_jump = bool(raw_displacement_px is not None and raw_displacement_px > 25.0)
    jump_3d = bool(raw_displacement_mm is not None and raw_displacement_mm > 25.0)
    large_jump = pixel_jump or jump_3d

    filter_state = "TRACKING"
    held_age_ms = None
    if raw_pixel is None or raw_point is None:
        held_age = (
            timestamp_sec - state.last_valid_time
            if state.last_valid_time is not None else math.inf
        )
        held_age_ms = held_age * 1000.0 if math.isfinite(held_age) else None
        if held_age <= hold_timeout_sec and state.filtered_point is not None:
            filter_state = "HELD_NO_DETECTION"
        else:
            filter_state = "RESET_TIMEOUT"
            state.filtered_pixel = None
            state.filtered_point = None
            state.pixel_window.clear()
            state.point_window.clear()
            state.filter_age_frames = 0
    else:
        if large_jump:
            filter_state = "RESET_LARGE_JUMP"
            state.pixel_window.clear()
            state.point_window.clear()
            state.filtered_pixel = None
            state.filtered_point = None
            state.filter_age_frames = 0
        state.pixel_window.append(raw_pixel.copy())
        state.point_window.append(raw_point.copy())
        median_pixel = np.median(np.asarray(state.pixel_window), axis=0)
        median_point = np.median(np.asarray(state.point_window), axis=0)
        delta_time = (
            max(1e-3, timestamp_sec - state.previous_filter_time)
            if state.previous_filter_time is not None else tau_sec
        )
        alpha = float(np.clip(1.0 - math.exp(-delta_time / tau_sec), 0.0, 1.0))
        if state.filtered_pixel is None or state.filtered_point is None:
            state.filtered_pixel = median_pixel
            state.filtered_point = median_point
        else:
            state.filtered_pixel += alpha * (median_pixel - state.filtered_pixel)
            state.filtered_point += alpha * (median_point - state.filtered_point)
        state.previous_filter_time = timestamp_sec
        state.last_valid_time = timestamp_sec
        state.filter_age_frames += 1

    filtered_displacement_px = filtered_displacement_mm = None
    if state.filtered_pixel is not None and previous_filtered_pixel is not None:
        filtered_displacement_px = float(np.linalg.norm(
            state.filtered_pixel - previous_filtered_pixel
        ))
    if state.filtered_point is not None and previous_filtered_point is not None:
        filtered_displacement_mm = float(np.linalg.norm(
            state.filtered_point - previous_filtered_point
        ) * 1000.0)
    raw_filtered_error = (
        float(np.linalg.norm(raw_point - state.filtered_point) * 1000.0)
        if raw_point is not None and state.filtered_point is not None else None
    )
    if raw_pixel is not None:
        state.previous_raw_pixel = raw_pixel.copy()
    if raw_point is not None:
        state.previous_raw_point = raw_point.copy()
        state.previous_raw_time = timestamp_sec
    state.previous_filtered_pixel = (
        state.filtered_pixel.copy() if state.filtered_pixel is not None else None
    )
    state.previous_filtered_point = (
        state.filtered_point.copy() if state.filtered_point is not None else None
    )
    return WristTemporalDiagnostic(
        raw_wrist_pixel=_tuple_or_none(raw_pixel),
        raw_wrist_3d=_tuple_or_none(raw_point),
        # Availability describes MediaPipe landmark[0].  Its 3D depth may be
        # unavailable independently and must not be mislabeled as detection loss.
        raw_wrist_available=raw_pixel is not None,
        raw_wrist_reason="NONE" if raw_point is not None else "UNAVAILABLE",
        depth_seed_pixel=None,
        depth_seed_3d=None,
        depth_seed_valid=False,
        wrist_seed_pixel_distance_px=None,
        wrist_seed_3d_distance_mm=None,
        filtered_wrist_pixel=_tuple_or_none(state.filtered_pixel),
        filtered_wrist_3d=_tuple_or_none(state.filtered_point),
        filter_state=filter_state,
        filter_age_frames=state.filter_age_frames,
        held_age_ms=held_age_ms,
        raw_displacement_px=raw_displacement_px,
        filtered_displacement_px=filtered_displacement_px,
        raw_displacement_mm=raw_displacement_mm,
        filtered_displacement_mm=filtered_displacement_mm,
        raw_velocity_mm_s=raw_velocity,
        raw_filtered_error_mm=raw_filtered_error,
        pixel_jump=pixel_jump,
        jump_3d=jump_3d,
        velocity_high=bool(raw_velocity is not None and raw_velocity > 300.0),
        raw_filtered_error_high=bool(
            raw_filtered_error is not None and raw_filtered_error > 20.0
        ),
    )


def _selected_branch_diagnostic(
    estimate, tracking
) -> tuple[BranchSurvivalDiagnostic, object | None]:
    branches = tuple(getattr(estimate, "branches", ()) or ())
    selected_id = getattr(estimate, "selected_branch", None)
    selected = next((item for item in branches if item.branch_id == selected_id), None)
    tracked = tuple(getattr(tracking, "branches", ()) or ())
    selected_tracked = next(
        (item for item in tracked if item.branch_id == selected_id), None
    )
    active_track_id = getattr(tracking, "active_track_id", None)
    active_visible = any(
        item.track_id == active_track_id for item in tracked
    ) if active_track_id is not None else False
    lengths = [
        (item.features.effective_length_m, item.reason)
        for item in branches if item.features is not None
    ]
    longest = max(lengths, key=lambda value: value[0]) if lengths else (None, None)
    features = selected.features if selected is not None else None
    branch_reason = (
        selected.reason if selected is not None else getattr(estimate, "reason", "NO_VALID_BRANCH")
    )
    return BranchSurvivalDiagnostic(
        branch_count=len(branches),
        selected_branch_id=selected_id,
        selected_track_id=(
            selected_tracked.track_id if selected_tracked is not None else None
        ),
        effective_length_m=(
            features.effective_length_m if features is not None else None
        ),
        continuous_span_m=(
            features.continuous_span_m if features is not None else None
        ),
        valid_section_count=(selected.valid_section_count if selected else 0),
        inlier_section_count=(selected.inlier_section_count if selected else 0),
        branch_reason=branch_reason,
        estimator_reason=getattr(estimate, "reason", "UNKNOWN"),
        active_track_id=active_track_id,
        active_track_missing=active_track_id is not None and not active_visible,
        longest_branch_length=longest[0],
        longest_branch_reason=longest[1],
    ), selected_tracked


def _baseline_value(values: deque) -> float | None:
    return float(np.median(values)) if values else None


def _first_sustained(window: deque, key: str) -> int | None:
    records = list(window)
    for first, second in zip(records, records[1:]):
        if first[key] and second[key]:
            return int(first["frame"])
    return None


def _attribute_failure(
    state: WristTemporalShadowState,
    temporal: WristTemporalDiagnostic,
    mask: MaskRobustnessDiagnostic,
    branch: BranchSurvivalDiagnostic,
) -> WristFailureAttribution:
    wrist_bad = (
        not temporal.raw_wrist_available
        or temporal.pixel_jump
        or temporal.jump_3d
        or temporal.velocity_high
        or temporal.raw_filtered_error_high
    )
    depth_seed_bad = bool(
        temporal.raw_wrist_available
        and (
            not temporal.depth_seed_valid
            or (
                temporal.wrist_seed_pixel_distance_px is not None
                and temporal.wrist_seed_pixel_distance_px > 15.0
            )
            or (
                temporal.wrist_seed_3d_distance_mm is not None
                and temporal.wrist_seed_3d_distance_mm > 20.0
            )
        )
    )
    mask_bad = mask.status != "MASK_STABLE"
    branch_bad = bool(
        branch.branch_reason != "NONE"
        or branch.estimator_reason != "NONE"
        or branch.active_track_missing
        or (
            mask.branch_length_ratio_to_baseline is not None
            and mask.branch_length_ratio_to_baseline < 0.60
        )
    )
    state.anomaly_window.append({
        "frame": state.frame_index,
        "wrist": wrist_bad,
        "depth_seed": depth_seed_bad,
        "mask": mask_bad,
        "branch": branch_bad,
    })
    first = {
        "A_WRIST_DETECTION": _first_sustained(state.anomaly_window, "wrist"),
        "D_DEPTH_SEED": _first_sustained(state.anomaly_window, "depth_seed"),
        "B_MASK": _first_sustained(state.anomaly_window, "mask"),
        "C_BRANCH_EXTRACTION": _first_sustained(state.anomaly_window, "branch"),
    }
    current_failure = wrist_bad or depth_seed_bad or mask_bad or branch_bad
    available = {key: value for key, value in first.items() if value is not None}
    if not current_failure:
        root = "F_UNKNOWN"
        confidence = 0.0
        first_stage = "NONE"
    elif available:
        earliest = min(available.values())
        earliest_causes = [key for key, value in available.items() if value == earliest]
        if (
            "A_WRIST_DETECTION" in earliest_causes
            and not temporal.raw_wrist_available
        ):
            # Missing landmark necessarily makes downstream mask/branch inputs
            # empty in the same frame; those are effects, not independent causes.
            root = "A_WRIST_DETECTION"
            confidence = 0.95
            first_stage = root
        elif (
            "D_DEPTH_SEED" in earliest_causes
            and not wrist_bad
        ):
            root = "D_DEPTH_SEED"
            confidence = 0.90
            first_stage = root
        elif (
            "B_MASK" in earliest_causes
            and not wrist_bad
            and not depth_seed_bad
        ):
            root = "B_MASK"
            confidence = 0.90
            first_stage = root
        elif len(earliest_causes) > 1:
            root = "E_COMPOUND"
            confidence = 0.60
            first_stage = "+".join(earliest_causes)
        else:
            root = earliest_causes[0]
            confidence = 0.90
            first_stage = root
    else:
        active = [
            name for name, enabled in (
                ("A_WRIST_DETECTION", wrist_bad),
                ("D_DEPTH_SEED", depth_seed_bad),
                ("B_MASK", mask_bad),
                ("C_BRANCH_EXTRACTION", branch_bad),
            ) if enabled
        ]
        if not temporal.raw_wrist_available:
            root = "A_WRIST_DETECTION"
            confidence = 0.70
        elif depth_seed_bad and not wrist_bad:
            root = "D_DEPTH_SEED"
            confidence = 0.65
        elif mask_bad and not wrist_bad and not depth_seed_bad:
            root = "B_MASK"
            confidence = 0.65
        else:
            root = (
                active[0] if len(active) == 1
                else "E_COMPOUND" if active else "F_UNKNOWN"
            )
            confidence = 0.55 if len(active) == 1 else 0.35
        first_stage = root
    downstream = []
    if not temporal.raw_wrist_available:
        downstream.append("NO_DETECTION")
    if depth_seed_bad:
        downstream.append("DEPTH_SEED_INVALID")
    if mask_bad:
        downstream.append(mask.status)
    if branch_bad:
        downstream.append(branch.branch_reason)
    if branch.active_track_missing:
        downstream.append("TRACK_MISSING")
    return WristFailureAttribution(
        root_cause=root,
        confidence=confidence,
        first_abnormal_stage=first_stage,
        downstream_effects=tuple(dict.fromkeys(downstream)),
        evidence={
            "raw_displacement_px": temporal.raw_displacement_px,
            "raw_displacement_mm": temporal.raw_displacement_mm,
            "raw_velocity_mm_s": temporal.raw_velocity_mm_s,
            "raw_filtered_error_mm": temporal.raw_filtered_error_mm,
            "wrist_seed_pixel_distance_px": temporal.wrist_seed_pixel_distance_px,
            "wrist_seed_3d_distance_mm": temporal.wrist_seed_3d_distance_mm,
            "mask_area_ratio": mask.area_ratio_to_baseline,
            "connected_ratio": mask.connected_ratio_to_baseline,
            "valid_depth_ratio": mask.valid_depth_ratio,
            "branch_length_ratio": mask.branch_length_ratio_to_baseline,
            "branch_reason": branch.branch_reason,
        },
        wrist_first_bad_frame=first["A_WRIST_DETECTION"],
        mask_first_bad_frame=first["B_MASK"],
        branch_first_bad_frame=first["C_BRANCH_EXTRACTION"],
        depth_seed_first_bad_frame=first["D_DEPTH_SEED"],
    )


def update_wrist_input_robustness_shadow(
    state: WristTemporalShadowState,
    wrist_landmark,
    depth_m: np.ndarray,
    intrinsics: CameraIntrinsics | None,
    mask_estimate,
    axis_estimate,
    tracking_estimate,
    timestamp_sec: float,
    *,
    mirrored: bool = False,
) -> dict:
    """Update read-only wrist-input diagnostics without feeding the pipeline."""
    state.frame_index += 1
    raw_pixel, raw_point, raw_reason = _sample_raw_wrist(
        wrist_landmark, depth_m, intrinsics, mirrored
    )
    temporal_base = _update_temporal_filter(
        state, raw_pixel, raw_point, timestamp_sec
    )
    depth_seed_pixel = getattr(mask_estimate, "wrist_seed_pixel", None)
    depth_seed_3d_value = getattr(mask_estimate, "wrist_seed_point_m", None)
    depth_seed_point = (
        np.asarray(depth_seed_3d_value, dtype=np.float64)
        if depth_seed_3d_value is not None else None
    )
    depth_seed_valid = bool(
        depth_seed_pixel is not None
        and depth_seed_point is not None
        and depth_seed_point.shape == (3,)
        and np.all(np.isfinite(depth_seed_point))
    )
    seed_pixel_distance = (
        float(np.linalg.norm(raw_pixel - np.asarray(depth_seed_pixel)))
        if raw_pixel is not None and depth_seed_pixel is not None else None
    )
    seed_3d_distance = (
        float(np.linalg.norm(raw_point - depth_seed_point) * 1000.0)
        if raw_point is not None and depth_seed_valid else None
    )
    temporal = WristTemporalDiagnostic(
        **{
            **temporal_base.__dict__,
            "raw_wrist_reason": raw_reason,
            "depth_seed_pixel": depth_seed_pixel,
            "depth_seed_3d": _tuple_or_none(depth_seed_point),
            "depth_seed_valid": depth_seed_valid,
            "wrist_seed_pixel_distance_px": seed_pixel_distance,
            "wrist_seed_3d_distance_mm": seed_3d_distance,
        }
    )

    branch, selected_tracked = _selected_branch_diagnostic(
        axis_estimate, tracking_estimate
    )
    mask = np.asarray(getattr(mask_estimate, "mask", np.zeros_like(depth_m)))
    mask_area = int(np.count_nonzero(mask > 0))
    valid_depth_count = int(np.count_nonzero(
        (mask > 0) & np.isfinite(depth_m) & (depth_m >= 0.10) & (depth_m <= 2.0)
    )) if mask.shape == depth_m.shape else 0
    valid_depth_ratio = valid_depth_count / max(1, mask_area)
    connected_points = int(getattr(mask_estimate, "connected_point_count", 0))
    branch_length = branch.effective_length_m
    baseline_ready = bool(
        selected_tracked is not None
        and selected_tracked.consecutive_hits >= 8
        and selected_tracked.valid
        and branch_length is not None
        and branch_length >= 0.080
    )
    if baseline_ready:
        previous_area = _baseline_value(state.mask_area_baseline)
        if previous_area is None or mask_area >= 0.70 * previous_area:
            state.mask_area_baseline.append(float(mask_area))
            state.connected_baseline.append(float(connected_points))
            state.depth_ratio_baseline.append(float(valid_depth_ratio))
            state.branch_length_baseline.append(float(branch_length))
    baseline_area = _baseline_value(state.mask_area_baseline)
    baseline_connected = _baseline_value(state.connected_baseline)
    baseline_depth = _baseline_value(state.depth_ratio_baseline)
    baseline_length = _baseline_value(state.branch_length_baseline)
    area_ratio = mask_area / baseline_area if baseline_area else None
    connected_ratio = (
        connected_points / baseline_connected if baseline_connected else None
    )
    length_ratio = (
        branch_length / baseline_length
        if branch_length is not None and baseline_length
        else None
    )
    raw_to_mask = _distance_to_mask(mask, raw_pixel)
    raw_to_cloud = _distance_to_cloud_mm(
        mask, depth_m, raw_point, intrinsics, mirrored
    )
    if mask_area == 0:
        mask_status = "MASK_EMPTY"
    elif valid_depth_ratio < 0.60:
        mask_status = "DEPTH_SPARSE"
    elif (
        (raw_to_mask is not None and raw_to_mask > 15.0)
        or (raw_to_cloud is not None and raw_to_cloud > 20.0)
    ):
        mask_status = "WRIST_OUTSIDE_COMPONENT"
    elif (
        (area_ratio is not None and area_ratio < 0.40)
        or (connected_ratio is not None and connected_ratio < 0.40)
    ):
        mask_status = "MASK_COLLAPSED"
    elif (
        (area_ratio is not None and area_ratio < 0.70)
        or (connected_ratio is not None and connected_ratio < 0.70)
    ):
        mask_status = "MASK_DEGRADED"
    else:
        mask_status = "MASK_STABLE"
    mask_diagnostic = MaskRobustnessDiagnostic(
        mask_area_px=mask_area,
        connected_point_count=connected_points,
        valid_depth_count=valid_depth_count,
        valid_depth_ratio=float(valid_depth_ratio),
        baseline_mask_area=baseline_area,
        baseline_connected_points=baseline_connected,
        baseline_depth_ratio=baseline_depth,
        baseline_branch_length=baseline_length,
        area_ratio_to_baseline=area_ratio,
        connected_ratio_to_baseline=connected_ratio,
        branch_length_ratio_to_baseline=length_ratio,
        raw_wrist_to_mask_distance_px=raw_to_mask,
        depth_seed_to_raw_wrist_distance_px=seed_pixel_distance,
        raw_wrist_to_cloud_distance_mm=raw_to_cloud,
        status=mask_status,
    )
    attribution = _attribute_failure(state, temporal, mask_diagnostic, branch)
    return {
        "frame_index": state.frame_index,
        "wrist_temporal": temporal.diagnostics(),
        "mask_robustness": mask_diagnostic.diagnostics(),
        "branch_survival": branch.diagnostics(),
        "attribution": attribution.diagnostics(),
        "shadow_only": True,
    }
