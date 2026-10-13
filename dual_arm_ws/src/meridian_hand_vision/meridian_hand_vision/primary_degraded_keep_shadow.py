from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import math

import numpy as np


def _axis_or_none(direction) -> np.ndarray | None:
    if direction is None:
        return None
    vector = np.asarray(direction.vector, dtype=np.float64)
    norm = float(np.linalg.norm(vector))
    if vector.shape != (3,) or not np.all(np.isfinite(vector)) or norm < 1e-6:
        return None
    return vector / norm


def _unsigned_angle_deg(first, second) -> float | None:
    if first is None or second is None:
        return None
    first_vector = np.asarray(first, dtype=np.float64)
    second_vector = np.asarray(second, dtype=np.float64)
    first_norm = float(np.linalg.norm(first_vector))
    second_norm = float(np.linalg.norm(second_vector))
    if first_norm <= 1e-12 or second_norm <= 1e-12:
        return None
    cosine = abs(float(np.dot(first_vector, second_vector))) / (
        first_norm * second_norm
    )
    return float(np.degrees(np.arccos(np.clip(cosine, 0.0, 1.0))))


@dataclass
class PrimaryDegradedKeepShadowState:
    """Causal keep-primary simulation isolated from live selector/filter state."""

    filter_window_size: int = 12
    maximum_angle_deg: float = 12.0
    actual_previous_source: str = "INVALID"
    shadow_previous_source: str = "INVALID"
    actual_source_switch_count: int = 0
    shadow_source_switch_count: int = 0
    actual_filter_clear_count: int = 0
    shadow_filter_clear_count: int = 0
    actual_fallback_frames: int = 0
    shadow_primary_frames: int = 0
    shadow_fallback_frames: int = 0
    retained_degraded_count: int = 0
    actual_stable_frames: int = 0
    shadow_stable_frames: int = 0
    shadow_only_stable_frames: int = 0
    actual_only_stable_frames: int = 0
    comparable_direction_frames: int = 0
    direction_angle_sum_deg: float = 0.0
    direction_angle_max_deg: float = 0.0
    retained_v2_angle_count: int = 0
    retained_v2_angle_sum_deg: float = 0.0
    retained_v2_angle_max_deg: float = 0.0
    filter_samples: deque[np.ndarray] = field(init=False)

    def __post_init__(self) -> None:
        if self.filter_window_size < 2 or self.maximum_angle_deg <= 0.0:
            raise ValueError("shadow filter parameters must be positive")
        self.filter_samples = deque(maxlen=self.filter_window_size)


def _update_shadow_filter(
    state: PrimaryDegradedKeepShadowState,
    direction,
) -> tuple[float, float, float] | None:
    vector = _axis_or_none(direction)
    if vector is None:
        return None
    if state.filter_samples and float(
        np.dot(vector, state.filter_samples[0])
    ) < 0.0:
        vector = -vector
    state.filter_samples.append(vector)
    if len(state.filter_samples) < state.filter_window_size:
        return None
    center = np.mean(np.asarray(state.filter_samples), axis=0)
    center_norm = float(np.linalg.norm(center))
    if not math.isfinite(center_norm) or center_norm <= 1e-12:
        return None
    center /= center_norm
    spread = max(
        _unsigned_angle_deg(sample, center) or 0.0
        for sample in state.filter_samples
    )
    if spread > state.maximum_angle_deg:
        return None
    return tuple(float(value) for value in center)


def update_primary_degraded_keep_shadow(
    state: PrimaryDegradedKeepShadowState,
    estimate,
    temporal_diagnostic: dict,
    actual_stable_direction,
    v2_2_axis=None,
) -> dict:
    """Simulate a temporal-DEGRADED override without mutating live objects."""
    actual_source = str(estimate.active_source)
    if bool(estimate.source_changed):
        state.actual_source_switch_count += 1
        state.actual_filter_clear_count += 1
    state.actual_previous_source = actual_source

    primary_axis = _axis_or_none(estimate.primary_direction)
    keep_primary = bool(
        actual_source == "FALLBACK_MEDIAPIPE"
        and temporal_diagnostic.get("degraded_temporal_state")
        == "DEGRADED_TEMPORALLY_VALID"
        and temporal_diagnostic.get("valid") is True
        and primary_axis is not None
    )
    shadow_source = "PRIMARY_3D_FOREARM" if keep_primary else actual_source
    shadow_direction = (
        estimate.primary_direction if keep_primary else estimate.direction
    )
    shadow_source_changed = shadow_source != state.shadow_previous_source
    if shadow_source_changed:
        state.shadow_source_switch_count += 1
        state.shadow_filter_clear_count += 1
        state.filter_samples.clear()
    state.shadow_previous_source = shadow_source

    shadow_stable_direction = _update_shadow_filter(state, shadow_direction)
    actual_stable_axis = _axis_or_none(actual_stable_direction)
    if actual_stable_axis is not None:
        state.actual_stable_frames += 1
    if shadow_stable_direction is not None:
        state.shadow_stable_frames += 1
    if actual_stable_axis is None and shadow_stable_direction is not None:
        state.shadow_only_stable_frames += 1
    elif actual_stable_axis is not None and shadow_stable_direction is None:
        state.actual_only_stable_frames += 1
    direction_difference = _unsigned_angle_deg(
        actual_stable_axis, shadow_stable_direction
    )
    if direction_difference is not None:
        state.comparable_direction_frames += 1
        state.direction_angle_sum_deg += direction_difference
        state.direction_angle_max_deg = max(
            state.direction_angle_max_deg, direction_difference
        )

    if actual_source == "FALLBACK_MEDIAPIPE":
        state.actual_fallback_frames += 1
        if keep_primary:
            state.shadow_primary_frames += 1
        else:
            state.shadow_fallback_frames += 1

    retained_risk = None
    if keep_primary:
        state.retained_degraded_count += 1
        v2_angle = _unsigned_angle_deg(primary_axis, v2_2_axis)
        if v2_angle is not None:
            state.retained_v2_angle_count += 1
            state.retained_v2_angle_sum_deg += v2_angle
            state.retained_v2_angle_max_deg = max(
                state.retained_v2_angle_max_deg, v2_angle
            )
        retained_risk = {
            "axis_delta_p95": temporal_diagnostic.get(
                "axis_delta_p95_deg"
            ),
            "axis_delta_p95_deg": temporal_diagnostic.get(
                "axis_delta_p95_deg"
            ),
            "rms": estimate.rms_m,
            "rms_m": estimate.rms_m,
            "length_change": temporal_diagnostic.get(
                "length_change_ratio"
            ),
            "length_change_ratio": temporal_diagnostic.get(
                "length_change_ratio"
            ),
            "angle_to_v2_2_deg": v2_angle,
            "v2_2_angle_deg": v2_angle,
        }

    return {
        "current_actual_source": actual_source,
        "current_shadow_source": shadow_source,
        "shadow_keep_applied": keep_primary,
        "reason": (
            "TEMPORALLY_VALID_DEGRADED_KEPT"
            if keep_primary
            else temporal_diagnostic.get("reason", "NO_SHADOW_OVERRIDE")
        ),
        "actual_source_switch_count": state.actual_source_switch_count,
        "shadow_source_switch_count": state.shadow_source_switch_count,
        "saved_switch_count": (
            state.actual_source_switch_count
            - state.shadow_source_switch_count
        ),
        "actual_filter_clear": state.actual_filter_clear_count,
        "shadow_filter_clear": state.shadow_filter_clear_count,
        "actual_stable_direction": (
            tuple(float(value) for value in actual_stable_axis)
            if actual_stable_axis is not None else None
        ),
        "shadow_simulated_direction": shadow_stable_direction,
        "actual_vs_shadow_angle_deg": direction_difference,
        "actual_fallback_frames": state.actual_fallback_frames,
        "shadow_primary_frames": state.shadow_primary_frames,
        "shadow_fallback_frames": state.shadow_fallback_frames,
        "retained_degraded_risk": retained_risk,
        "risk_summary": {
            "retained_degraded_count": state.retained_degraded_count,
            "v2_2_angle_count": state.retained_v2_angle_count,
            "v2_2_angle_mean_deg": (
                state.retained_v2_angle_sum_deg
                / state.retained_v2_angle_count
                if state.retained_v2_angle_count > 0 else None
            ),
            "v2_2_angle_max_deg": (
                state.retained_v2_angle_max_deg
                if state.retained_v2_angle_count > 0 else None
            ),
        },
        "direction_comparison_summary": {
            "actual_stable_frames": state.actual_stable_frames,
            "shadow_stable_frames": state.shadow_stable_frames,
            "shadow_only_stable_frames": state.shadow_only_stable_frames,
            "actual_only_stable_frames": state.actual_only_stable_frames,
            "comparable_frames": state.comparable_direction_frames,
            "mean_angle_deg": (
                state.direction_angle_sum_deg
                / state.comparable_direction_frames
                if state.comparable_direction_frames > 0 else None
            ),
            "max_angle_deg": (
                state.direction_angle_max_deg
                if state.comparable_direction_frames > 0 else None
            ),
        },
        "shadow_only": True,
    }
