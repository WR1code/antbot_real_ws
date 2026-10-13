from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import math

import numpy as np


@dataclass(frozen=True)
class _DegradedFrame:
    degraded: bool
    axis: tuple[float, float, float] | None
    length_m: float | None
    rms_m: float | None
    source_reset: bool


@dataclass
class PrimaryDegradedTemporalShadowState:
    """Five-frame diagnostic window; never feeds the selector or filter."""

    window_size: int = 5
    frames: deque[_DegradedFrame] = field(init=False)
    degraded_streak: int = 0

    def __post_init__(self) -> None:
        if self.window_size != 5:
            raise ValueError("PRIMARY_DEGRADED shadow window must be 5 frames")
        self.frames = deque(maxlen=self.window_size)


def _axis_or_none(direction) -> tuple[float, float, float] | None:
    if direction is None:
        return None
    axis = np.asarray(direction.vector, dtype=np.float64)
    norm = float(np.linalg.norm(axis))
    if axis.shape != (3,) or not np.all(np.isfinite(axis)) or norm <= 1e-12:
        return None
    axis /= norm
    return tuple(float(value) for value in axis)


def _finite_nonnegative(value) -> float | None:
    if value is None:
        return None
    result = float(value)
    if not math.isfinite(result) or result < 0.0:
        return None
    return result


def _unsigned_angle_deg(first, second) -> float:
    cosine = abs(float(np.dot(first, second)))
    return float(np.degrees(np.arccos(np.clip(cosine, 0.0, 1.0))))


def update_primary_degraded_temporal_shadow(
    state: PrimaryDegradedTemporalShadowState,
    estimate,
) -> dict:
    """Evaluate temporal credibility without changing the selected source."""
    source_reset = bool(estimate.source_changed)
    if source_reset:
        state.frames.clear()
        state.degraded_streak = 0

    axis = _axis_or_none(estimate.primary_direction)
    degraded = bool(
        estimate.primary_quality_level == "PRIMARY_DEGRADED"
        and estimate.reason == "INLIER_RATIO_LOW"
        and axis is not None
    )
    state.degraded_streak = state.degraded_streak + 1 if degraded else 0
    state.frames.append(_DegradedFrame(
        degraded=degraded,
        axis=axis if degraded else None,
        length_m=(
            _finite_nonnegative(estimate.axis_length_m) if degraded else None
        ),
        rms_m=_finite_nonnegative(estimate.rms_m) if degraded else None,
        source_reset=source_reset,
    ))

    degraded_frames = [frame for frame in state.frames if frame.degraded]
    axes = [frame.axis for frame in degraded_frames if frame.axis is not None]
    axis_deltas = [
        _unsigned_angle_deg(first, second)
        for first, second in zip(axes, axes[1:])
    ]
    lengths = [
        frame.length_m for frame in degraded_frames
        if frame.length_m is not None
    ]
    residuals = [
        frame.rms_m for frame in degraded_frames if frame.rms_m is not None
    ]
    axis_delta_median = (
        float(np.median(axis_deltas)) if axis_deltas else None
    )
    axis_delta_p95 = (
        float(np.percentile(axis_deltas, 95)) if axis_deltas else None
    )
    median_length = float(np.median(lengths)) if lengths else None
    length_change_ratio = (
        float((max(lengths) - min(lengths)) / median_length)
        if len(lengths) >= 2 and median_length is not None
        and median_length > 1e-12 else None
    )
    rms_median = float(np.median(residuals)) if residuals else None
    rms_p95 = (
        float(np.percentile(residuals, 95)) if residuals else None
    )
    source_change_count = sum(frame.source_reset for frame in state.frames)

    rejection_reasons: list[str] = []
    if len(state.frames) < state.window_size:
        rejection_reasons.append("WINDOW_NOT_FULL")
    if len(degraded_frames) < 4:
        rejection_reasons.append("DEGRADED_COUNT_LOW")
    if len(axis_deltas) < 3:
        rejection_reasons.append("INSUFFICIENT_AXIS_DELTAS")
    elif axis_delta_median is None or axis_delta_median >= 3.0:
        rejection_reasons.append("AXIS_DELTA_MEDIAN_HIGH")
    if axis_delta_p95 is None or axis_delta_p95 >= 10.0:
        rejection_reasons.append("AXIS_DELTA_P95_HIGH")
    if length_change_ratio is None or length_change_ratio >= 0.15:
        rejection_reasons.append("LENGTH_CHANGE_HIGH")
    if rms_median is None or rms_median >= 0.004:
        rejection_reasons.append("RMS_MEDIAN_HIGH")
    if rms_p95 is None or rms_p95 >= 0.006:
        rejection_reasons.append("RMS_P95_HIGH")
    if source_change_count > 0:
        rejection_reasons.append("SOURCE_RESET_IN_WINDOW")
    if not degraded:
        rejection_reasons.append("CURRENT_NOT_INLIER_RATIO_DEGRADED")

    valid = not rejection_reasons
    temporal_state = (
        "DEGRADED_TEMPORALLY_VALID"
        if valid else "DEGRADED_UNSTABLE"
    )
    shadow_would_keep_primary = bool(
        valid and estimate.active_source == "FALLBACK_MEDIAPIPE"
    )
    return {
        "current_source": estimate.active_source,
        "primary_quality": estimate.primary_quality_level,
        "degraded_temporal_state": temporal_state,
        "shadow_would_keep_primary": shadow_would_keep_primary,
        "reason": (
            "TEMPORAL_CRITERIA_MET"
            if valid else ";".join(rejection_reasons)
        ),
        "degraded_streak": int(state.degraded_streak),
        "window_frame_count": len(state.frames),
        "degraded_frame_count": len(degraded_frames),
        "axis_delta_median_deg": axis_delta_median,
        "axis_delta_p95_deg": axis_delta_p95,
        "length_change_ratio": length_change_ratio,
        "rms_median": rms_median,
        "rms_p95": rms_p95,
        "source_change_count": int(source_change_count),
        "valid": valid,
        "shadow_only": True,
    }
