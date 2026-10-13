from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import math

import numpy as np


PRIMARY = "PRIMARY_3D_FOREARM"
FALLBACK = "FALLBACK_MEDIAPIPE"
HOLD = "HOLD_LAST_PRIMARY_DIRECTION"
NONE = "NONE"


def _axis_or_none(direction) -> tuple[float, float, float] | None:
    if direction is None:
        return None
    vector = np.asarray(direction.vector, dtype=np.float64)
    if vector.shape != (3,) or not np.all(np.isfinite(vector)):
        return None
    norm = float(np.linalg.norm(vector))
    if norm < 1e-6:
        return None
    vector /= norm
    return tuple(float(value) for value in vector)


def _plain_axis_or_none(axis) -> tuple[float, float, float] | None:
    if axis is None:
        return None
    vector = np.asarray(axis, dtype=np.float64)
    if vector.shape != (3,) or not np.all(np.isfinite(vector)):
        return None
    norm = float(np.linalg.norm(vector))
    if norm < 1e-6:
        return None
    vector /= norm
    return tuple(float(value) for value in vector)


def _unsigned_angle_deg(first, second) -> float | None:
    if first is None or second is None:
        return None
    cosine = abs(float(np.dot(first, second)))
    return float(np.degrees(np.arccos(np.clip(cosine, 0.0, 1.0))))


def _percentile(values: list[float], percentile: float) -> float | None:
    if not values:
        return None
    return float(np.percentile(np.asarray(values, dtype=np.float64), percentile))


def _provider_family(source: str) -> str:
    if source in (PRIMARY, HOLD):
        return PRIMARY
    return source


@dataclass
class _StrategyState:
    initialized: bool = False
    previous_source: str = NONE
    previous_axis: tuple[float, float, float] | None = None
    state_transition_count: int = 0
    source_switch_count: int = 0
    filter_clear_count: int = 0
    fallback_mediapipe_frames: int = 0
    hold_frames: int = 0
    none_frames: int = 0
    primary_frames: int = 0
    direction_deltas_deg: list[float] = field(default_factory=list)
    v2_angles_deg: list[float] = field(default_factory=list)
    hold_v2_angles_deg: list[float] = field(default_factory=list)
    hold_without_v2_frames: int = 0
    mediapipe_v2_angles_deg: list[float] = field(default_factory=list)
    mediapipe_primary_angles_deg: list[float] = field(default_factory=list)


@dataclass
class _PhaseStats:
    frames: int = 0
    primary_frames: int = 0
    degraded_keep_frames: int = 0
    fallback_mediapipe_frames: int = 0
    hold_frames: int = 0
    none_frames: int = 0
    previous_axis: tuple[float, float, float] | None = None
    direction_deltas_deg: list[float] = field(default_factory=list)
    v2_angles_deg: list[float] = field(default_factory=list)
    hold_v2_angles_deg: list[float] = field(default_factory=list)


@dataclass
class _BPlusSimulation:
    timeout_sec: float
    strategy: _StrategyState = field(default_factory=_StrategyState)
    last_primary_axis: tuple[float, float, float] | None = None
    last_primary_time_sec: float | None = None
    degraded_keep_frames: int = 0
    hold_active: bool = False
    hold_start_time_sec: float | None = None
    hold_start_phase: str = "UNLABELED"
    completed_hold_durations_sec: list[float] = field(default_factory=list)
    phase_hold_durations_sec: dict[str, list[float]] = field(default_factory=dict)
    phase_stats: dict[str, _PhaseStats] = field(default_factory=dict)


@dataclass
class SelectorFallbackStrategyShadowState:
    """Independent A/B/C fallback simulations; never feeds live selection."""

    hold_timeout_sec: float = 0.5
    strategy_a: _StrategyState = field(default_factory=_StrategyState)
    strategy_b: _StrategyState = field(default_factory=_StrategyState)
    strategy_c: _StrategyState = field(default_factory=_StrategyState)
    last_primary_axis: tuple[float, float, float] | None = None
    last_primary_time_sec: float | None = None
    hold_active: bool = False
    hold_start_time_sec: float | None = None
    hold_start_source: str | None = None
    hold_start_axis: tuple[float, float, float] | None = None
    hold_last_v2_angle_deg: float | None = None
    hold_count: int = 0
    completed_hold_count: int = 0
    total_completed_hold_duration_sec: float = 0.0
    max_completed_hold_duration_sec: float = 0.0
    last_completed_hold: dict | None = None
    phase_axis_window: deque = field(default_factory=lambda: deque(maxlen=8))
    last_inferred_phase: str = "UNLABELED"
    strategy_a_phase_stats: dict[str, _PhaseStats] = field(default_factory=dict)
    strategy_b_phase_stats: dict[str, _PhaseStats] = field(default_factory=dict)
    strategy_c_phase_stats: dict[str, _PhaseStats] = field(default_factory=dict)
    strategy_b_plus_02: _BPlusSimulation = field(
        default_factory=lambda: _BPlusSimulation(0.2)
    )
    strategy_b_plus_05: _BPlusSimulation = field(
        default_factory=lambda: _BPlusSimulation(0.5)
    )
    strategy_b_plus_10: _BPlusSimulation = field(
        default_factory=lambda: _BPlusSimulation(1.0)
    )

    def __post_init__(self) -> None:
        if not math.isfinite(self.hold_timeout_sec) or self.hold_timeout_sec <= 0.0:
            raise ValueError("hold_timeout_sec must be positive")


def _update_strategy(
    state: _StrategyState,
    source: str,
    axis: tuple[float, float, float] | None,
    v2_axis: tuple[float, float, float] | None,
    primary_axis: tuple[float, float, float] | None,
    mediapipe_axis: tuple[float, float, float] | None,
) -> dict:
    source_changed = state.initialized and source != state.previous_source
    provider_source_changed = bool(
        state.initialized
        and _provider_family(source) != _provider_family(state.previous_source)
    )
    if source_changed:
        state.state_transition_count += 1
    if provider_source_changed:
        state.source_switch_count += 1
        state.filter_clear_count += 1
    state.initialized = True
    state.previous_source = source

    if source == PRIMARY:
        state.primary_frames += 1
    elif source == FALLBACK:
        state.fallback_mediapipe_frames += 1
    elif source == HOLD:
        state.hold_frames += 1
    else:
        state.none_frames += 1

    direction_delta = _unsigned_angle_deg(state.previous_axis, axis)
    if direction_delta is not None:
        state.direction_deltas_deg.append(direction_delta)
    state.previous_axis = axis

    v2_angle = _unsigned_angle_deg(axis, v2_axis)
    if v2_angle is not None:
        state.v2_angles_deg.append(v2_angle)
    if source == HOLD:
        if v2_angle is None:
            state.hold_without_v2_frames += 1
        else:
            state.hold_v2_angles_deg.append(v2_angle)
    if source == FALLBACK:
        mediapipe_v2_angle = _unsigned_angle_deg(mediapipe_axis, v2_axis)
        mediapipe_primary_angle = _unsigned_angle_deg(
            mediapipe_axis, primary_axis
        )
        if mediapipe_v2_angle is not None:
            state.mediapipe_v2_angles_deg.append(mediapipe_v2_angle)
        if mediapipe_primary_angle is not None:
            state.mediapipe_primary_angles_deg.append(
                mediapipe_primary_angle
            )

    return {
        "current_source": source,
        "current_direction": axis,
        "source_changed": source_changed,
        "provider_source_changed": provider_source_changed,
        "state_transition_count": state.state_transition_count,
        "source_switch_count": state.source_switch_count,
        "fallback_mediapipe_frames": state.fallback_mediapipe_frames,
        "primary_frames": state.primary_frames,
        "hold_frames": state.hold_frames,
        "none_frames": state.none_frames,
        "filter_clear_count": state.filter_clear_count,
        "direction_delta_current_deg": direction_delta,
        "direction_delta_p95_deg": _percentile(
            state.direction_deltas_deg, 95.0
        ),
        "direction_delta_p95": _percentile(
            state.direction_deltas_deg, 95.0
        ),
        "direction_delta_max_deg": (
            max(state.direction_deltas_deg)
            if state.direction_deltas_deg else None
        ),
        "direction_delta_max": (
            max(state.direction_deltas_deg)
            if state.direction_deltas_deg else None
        ),
        "v2_2_axis_angle_current_deg": v2_angle,
        "v2_2_axis_angle_mean_deg": (
            float(np.mean(state.v2_angles_deg))
            if state.v2_angles_deg else None
        ),
        "v2_2_axis_angle_p95_deg": _percentile(state.v2_angles_deg, 95.0),
        "v2_2_axis_angle_max_deg": (
            max(state.v2_angles_deg) if state.v2_angles_deg else None
        ),
        "hold_risk": {
            "comparable_frames": len(state.hold_v2_angles_deg),
            "without_v2_reference_frames": state.hold_without_v2_frames,
            "angle_p95_deg": _percentile(state.hold_v2_angles_deg, 95.0),
            "angle_max_deg": (
                max(state.hold_v2_angles_deg)
                if state.hold_v2_angles_deg else None
            ),
            "over_15deg_frames": sum(
                angle > 15.0 for angle in state.hold_v2_angles_deg
            ),
            "over_30deg_frames": sum(
                angle > 30.0 for angle in state.hold_v2_angles_deg
            ),
        },
        "mediapipe_risk": {
            "usage_frames": state.fallback_mediapipe_frames,
            "vs_v2_2_comparable_frames": len(
                state.mediapipe_v2_angles_deg
            ),
            "vs_v2_2_median_deg": _percentile(
                state.mediapipe_v2_angles_deg, 50.0
            ),
            "vs_v2_2_p95_deg": _percentile(
                state.mediapipe_v2_angles_deg, 95.0
            ),
            "vs_primary_comparable_frames": len(
                state.mediapipe_primary_angles_deg
            ),
            "vs_primary_median_deg": _percentile(
                state.mediapipe_primary_angles_deg, 50.0
            ),
            "vs_primary_p95_deg": _percentile(
                state.mediapipe_primary_angles_deg, 95.0
            ),
            "vs_primary_max_deg": (
                max(state.mediapipe_primary_angles_deg)
                if state.mediapipe_primary_angles_deg else None
            ),
        },
    }


def _infer_experiment_phase(
    state: SelectorFallbackStrategyShadowState,
    gesture: str | None,
    primary_axis: tuple[float, float, float] | None,
    mediapipe_axis: tuple[float, float, float] | None,
) -> tuple[str, float, dict]:
    """Infer a diagnostic phase; it is explicitly not experimental ground truth."""
    if gesture == "fist":
        state.phase_axis_window.clear()
        state.last_inferred_phase = "FIST"
        return "FIST", 0.7, {
            "basis": "RAW_GESTURE_CLASSIFIER",
            "palm_span_deg": None,
            "primary_span_deg": None,
        }
    if primary_axis is None or mediapipe_axis is None:
        state.phase_axis_window.clear()
        state.last_inferred_phase = "UNLABELED"
        return "UNLABELED", 0.0, {
            "basis": "INSUFFICIENT_AXIS_INPUT",
            "palm_span_deg": None,
            "primary_span_deg": None,
        }

    state.phase_axis_window.append((primary_axis, mediapipe_axis))
    primary_reference, palm_reference = state.phase_axis_window[0]
    primary_span = max(
        _unsigned_angle_deg(primary_reference, item[0]) or 0.0
        for item in state.phase_axis_window
    )
    palm_span = max(
        _unsigned_angle_deg(palm_reference, item[1]) or 0.0
        for item in state.phase_axis_window
    )
    if len(state.phase_axis_window) >= 4 and palm_span >= 8.0 and primary_span <= 5.0:
        phase = "PALM_ROTATION"
        confidence = min(0.95, 0.55 + (palm_span - 8.0) / 40.0)
    else:
        phase = "STATIC"
        confidence = 0.65 if len(state.phase_axis_window) >= 4 else 0.35
    state.last_inferred_phase = phase
    return phase, confidence, {
        "basis": "AXIS_WINDOW_INFERENCE",
        "window_frames": len(state.phase_axis_window),
        "palm_span_deg": palm_span,
        "primary_span_deg": primary_span,
        "palm_rotation_threshold_deg": 8.0,
        "primary_stability_threshold_deg": 5.0,
    }


def _update_phase_stats(
    phase_stats: dict[str, _PhaseStats],
    phase: str,
    source: str,
    axis: tuple[float, float, float] | None,
    v2_axis: tuple[float, float, float] | None,
    *,
    degraded_keep: bool = False,
) -> None:
    stats = phase_stats.setdefault(phase, _PhaseStats())
    stats.frames += 1
    if source == PRIMARY:
        stats.primary_frames += 1
        if degraded_keep:
            stats.degraded_keep_frames += 1
    elif source == FALLBACK:
        stats.fallback_mediapipe_frames += 1
    elif source == HOLD:
        stats.hold_frames += 1
    else:
        stats.none_frames += 1
    delta = _unsigned_angle_deg(stats.previous_axis, axis)
    if delta is not None:
        stats.direction_deltas_deg.append(delta)
    stats.previous_axis = axis
    v2_angle = _unsigned_angle_deg(axis, v2_axis)
    if v2_angle is not None:
        stats.v2_angles_deg.append(v2_angle)
        if source == HOLD:
            stats.hold_v2_angles_deg.append(v2_angle)


def _phase_stats_diagnostics(phase_stats: dict[str, _PhaseStats]) -> dict:
    result = {}
    for phase in ("STATIC", "PALM_ROTATION", "FIST", "UNLABELED"):
        stats = phase_stats.get(phase, _PhaseStats())
        result[phase] = {
            "frames": stats.frames,
            "primary_frames": stats.primary_frames,
            "degraded_keep_frames": stats.degraded_keep_frames,
            "fallback_mediapipe_frames": stats.fallback_mediapipe_frames,
            "hold_frames": stats.hold_frames,
            "none_frames": stats.none_frames,
            "media_pipe_usage_ratio": (
                stats.fallback_mediapipe_frames / stats.frames
                if stats.frames > 0 else None
            ),
            "direction_delta_p95_deg": _percentile(
                stats.direction_deltas_deg, 95.0
            ),
            "direction_delta_max_deg": (
                max(stats.direction_deltas_deg)
                if stats.direction_deltas_deg else None
            ),
            "v2_2_angle_mean_deg": (
                float(np.mean(stats.v2_angles_deg))
                if stats.v2_angles_deg else None
            ),
            "direction_vs_v2_mean": (
                float(np.mean(stats.v2_angles_deg))
                if stats.v2_angles_deg else None
            ),
            "v2_2_angle_p95_deg": _percentile(
                stats.v2_angles_deg, 95.0
            ),
            "direction_vs_v2_p95": _percentile(
                stats.v2_angles_deg, 95.0
            ),
            "hold_vs_v2_angle_p95_deg": _percentile(
                stats.hold_v2_angles_deg, 95.0
            ),
            "hold_vs_v2_angle_max_deg": (
                max(stats.hold_v2_angles_deg)
                if stats.hold_v2_angles_deg else None
            ),
        }
    return result


def _update_b_plus_simulation(
    simulation: _BPlusSimulation,
    *,
    strong: bool,
    temporally_valid_degraded: bool,
    primary_axis: tuple[float, float, float] | None,
    mediapipe_axis: tuple[float, float, float] | None,
    v2_axis: tuple[float, float, float] | None,
    now: float,
    phase: str,
) -> dict:
    primary_allowed = strong or temporally_valid_degraded
    if primary_allowed:
        source, selected_axis = PRIMARY, primary_axis
        simulation.last_primary_axis = primary_axis
        simulation.last_primary_time_sec = now
        if temporally_valid_degraded:
            simulation.degraded_keep_frames += 1
    else:
        age_sec = (
            now - simulation.last_primary_time_sec
            if simulation.last_primary_time_sec is not None else None
        )
        if (
            simulation.last_primary_axis is not None
            and age_sec is not None
            and 0.0 <= age_sec <= simulation.timeout_sec
        ):
            source, selected_axis = HOLD, simulation.last_primary_axis
        else:
            source, selected_axis = NONE, None

    entered_hold = source == HOLD and not simulation.hold_active
    exited_hold = source != HOLD and simulation.hold_active
    if entered_hold:
        simulation.hold_active = True
        simulation.hold_start_time_sec = now
        simulation.hold_start_phase = phase
    if exited_hold:
        duration = max(
            0.0,
            now - (
                simulation.hold_start_time_sec
                if simulation.hold_start_time_sec is not None else now
            ),
        )
        simulation.completed_hold_durations_sec.append(duration)
        simulation.phase_hold_durations_sec.setdefault(
            simulation.hold_start_phase, []
        ).append(duration)
        simulation.hold_active = False
        simulation.hold_start_time_sec = None
        simulation.hold_start_phase = "UNLABELED"

    result = _update_strategy(
        simulation.strategy,
        source,
        selected_axis,
        v2_axis,
        primary_axis,
        mediapipe_axis,
    )
    _update_phase_stats(
        simulation.phase_stats,
        phase,
        source,
        selected_axis,
        v2_axis,
        degraded_keep=temporally_valid_degraded,
    )
    phase_diagnostics = _phase_stats_diagnostics(simulation.phase_stats)
    for phase_name, details in phase_diagnostics.items():
        durations = simulation.phase_hold_durations_sec.get(phase_name, [])
        details["hold_duration_p95_sec"] = _percentile(durations, 95.0)

    total_frames = (
        simulation.strategy.primary_frames
        + simulation.strategy.fallback_mediapipe_frames
        + simulation.strategy.hold_frames
        + simulation.strategy.none_frames
    )
    hold_duration_p95 = _percentile(
        simulation.completed_hold_durations_sec, 95.0
    )
    hold_v2_p95 = _percentile(
        simulation.strategy.hold_v2_angles_deg, 95.0
    )
    hold_v2_max = (
        max(simulation.strategy.hold_v2_angles_deg)
        if simulation.strategy.hold_v2_angles_deg else None
    )
    direction_delta_p95 = _percentile(
        simulation.strategy.direction_deltas_deg, 95.0
    )
    direction_delta_max = (
        max(simulation.strategy.direction_deltas_deg)
        if simulation.strategy.direction_deltas_deg else None
    )
    direction_v2_mean = (
        float(np.mean(simulation.strategy.v2_angles_deg))
        if simulation.strategy.v2_angles_deg else None
    )
    direction_v2_p95 = _percentile(
        simulation.strategy.v2_angles_deg, 95.0
    )
    return {
        "strategy": f"STRATEGY_B_PLUS_HOLD_{simulation.timeout_sec:.1f}S",
        "hold_timeout_sec": simulation.timeout_sec,
        "current_source": source,
        "current_direction": selected_axis,
        "primary_frames": simulation.strategy.primary_frames,
        "degraded_keep_frames": simulation.degraded_keep_frames,
        "hold_frames": simulation.strategy.hold_frames,
        "fallback_mediapipe_frames": (
            simulation.strategy.fallback_mediapipe_frames
        ),
        "none_frames": simulation.strategy.none_frames,
        "source_switch_count": simulation.strategy.source_switch_count,
        "state_transition_count": simulation.strategy.state_transition_count,
        "filter_clear_count": simulation.strategy.filter_clear_count,
        "hold_episode_count": (
            len(simulation.completed_hold_durations_sec)
            + int(simulation.hold_active)
        ),
        "hold_duration_p95": hold_duration_p95,
        "hold_duration_p95_sec": hold_duration_p95,
        "hold_vs_v2_angle_p95": hold_v2_p95,
        "hold_vs_v2_angle_p95_deg": hold_v2_p95,
        "hold_vs_v2_angle_max": hold_v2_max,
        "hold_vs_v2_angle_max_deg": hold_v2_max,
        "media_pipe_usage_ratio": (
            simulation.strategy.fallback_mediapipe_frames / total_frames
            if total_frames > 0 else None
        ),
        "direction_delta_p95": direction_delta_p95,
        "direction_delta_p95_deg": direction_delta_p95,
        "direction_delta_max": direction_delta_max,
        "direction_delta_max_deg": direction_delta_max,
        "direction_vs_v2_mean": direction_v2_mean,
        "direction_vs_v2_mean_deg": direction_v2_mean,
        "direction_vs_v2_p95": direction_v2_p95,
        "direction_vs_v2_p95_deg": direction_v2_p95,
        "phase_statistics": phase_diagnostics,
        "shadow_only": True,
    }


def update_selector_fallback_strategy_shadow(
    state: SelectorFallbackStrategyShadowState,
    estimate,
    temporal_diagnostic: dict,
    timestamp_sec: float,
    *,
    v2_2_axis=None,
    gesture: str | None = None,
) -> dict:
    """Evaluate three fallback policies without mutating any live object."""
    now = float(timestamp_sec)
    if not math.isfinite(now):
        raise ValueError("timestamp_sec must be finite")

    primary_axis = _axis_or_none(estimate.primary_direction)
    mediapipe_axis = _axis_or_none(estimate.mediapipe_direction)
    actual_axis = _axis_or_none(getattr(estimate, "direction", None))
    v2_axis = _plain_axis_or_none(v2_2_axis)
    strong = bool(
        estimate.primary_quality_level == "PRIMARY_STRONG"
        and primary_axis is not None
    )
    temporally_valid_degraded = bool(
        estimate.primary_quality_level == "PRIMARY_DEGRADED"
        and temporal_diagnostic.get("degraded_temporal_state")
        == "DEGRADED_TEMPORALLY_VALID"
        and temporal_diagnostic.get("valid") is True
        and primary_axis is not None
    )
    primary_allowed_b = strong or temporally_valid_degraded

    if strong:
        source_a, axis_a = PRIMARY, primary_axis
    elif mediapipe_axis is not None:
        source_a, axis_a = FALLBACK, mediapipe_axis
    else:
        source_a, axis_a = NONE, None

    if primary_allowed_b:
        source_b, axis_b = PRIMARY, primary_axis
    elif mediapipe_axis is not None:
        source_b, axis_b = FALLBACK, mediapipe_axis
    else:
        source_b, axis_b = NONE, None

    if primary_allowed_b:
        source_c, axis_c = PRIMARY, primary_axis
        state.last_primary_axis = primary_axis
        state.last_primary_time_sec = now
        hold_age_sec = 0.0
    else:
        hold_age_sec = (
            now - state.last_primary_time_sec
            if state.last_primary_time_sec is not None else None
        )
        if (
            state.last_primary_axis is not None
            and hold_age_sec is not None
            and 0.0 <= hold_age_sec <= state.hold_timeout_sec
        ):
            source_c, axis_c = HOLD, state.last_primary_axis
        else:
            source_c, axis_c = NONE, None

    previous_c_source = state.strategy_c.previous_source
    previous_c_initialized = state.strategy_c.initialized
    result_a = _update_strategy(
        state.strategy_a, source_a, axis_a, v2_axis,
        primary_axis, mediapipe_axis,
    )
    result_b = _update_strategy(
        state.strategy_b, source_b, axis_b, v2_axis,
        primary_axis, mediapipe_axis,
    )
    result_c = _update_strategy(
        state.strategy_c, source_c, axis_c, v2_axis,
        primary_axis, mediapipe_axis,
    )

    entered_hold = source_c == HOLD and not state.hold_active
    exited_hold = source_c != HOLD and state.hold_active
    if entered_hold:
        state.hold_active = True
        state.hold_start_time_sec = now
        state.hold_start_source = (
            previous_c_source if previous_c_initialized else NONE
        )
        state.hold_start_axis = axis_c
        state.hold_last_v2_angle_deg = _unsigned_angle_deg(axis_c, v2_axis)
        state.hold_count += 1
    elif source_c == HOLD:
        state.hold_last_v2_angle_deg = _unsigned_angle_deg(axis_c, v2_axis)
    if exited_hold:
        duration = max(
            0.0,
            now - (
                state.hold_start_time_sec
                if state.hold_start_time_sec is not None else now
            ),
        )
        state.completed_hold_count += 1
        state.total_completed_hold_duration_sec += duration
        state.max_completed_hold_duration_sec = max(
            state.max_completed_hold_duration_sec, duration
        )
        state.last_completed_hold = {
            "hold_duration_sec": duration,
            "hold_start_source": state.hold_start_source,
            "hold_end_source": source_c,
            "hold_direction_change_deg": 0.0,
            "hold_end_direction_change_deg": _unsigned_angle_deg(
                state.hold_start_axis, axis_c
            ),
            "hold_vs_v2_angle_deg": state.hold_last_v2_angle_deg,
        }
        state.hold_active = False
        state.hold_start_time_sec = None
        state.hold_start_source = None
        state.hold_start_axis = None
        state.hold_last_v2_angle_deg = None

    current_hold_duration = (
        max(
            0.0,
            now - (
                state.hold_start_time_sec
                if state.hold_start_time_sec is not None else now
            ),
        )
        if state.hold_active else None
    )
    current_hold_v2_angle = (
        _unsigned_angle_deg(axis_c, v2_axis) if source_c == HOLD else None
    )
    completed_hold_this_frame = (
        state.last_completed_hold if exited_hold else None
    )
    phase, phase_confidence, phase_evidence = _infer_experiment_phase(
        state, gesture, primary_axis, mediapipe_axis
    )
    _update_phase_stats(
        state.strategy_a_phase_stats, phase, source_a, axis_a, v2_axis
    )
    _update_phase_stats(
        state.strategy_b_phase_stats, phase, source_b, axis_b, v2_axis
    )
    _update_phase_stats(
        state.strategy_c_phase_stats, phase, source_c, axis_c, v2_axis
    )
    b_plus_02 = _update_b_plus_simulation(
        state.strategy_b_plus_02,
        strong=strong,
        temporally_valid_degraded=temporally_valid_degraded,
        primary_axis=primary_axis,
        mediapipe_axis=mediapipe_axis,
        v2_axis=v2_axis,
        now=now,
        phase=phase,
    )
    b_plus_05 = _update_b_plus_simulation(
        state.strategy_b_plus_05,
        strong=strong,
        temporally_valid_degraded=temporally_valid_degraded,
        primary_axis=primary_axis,
        mediapipe_axis=mediapipe_axis,
        v2_axis=v2_axis,
        now=now,
        phase=phase,
    )
    b_plus_10 = _update_b_plus_simulation(
        state.strategy_b_plus_10,
        strong=strong,
        temporally_valid_degraded=temporally_valid_degraded,
        primary_axis=primary_axis,
        mediapipe_axis=mediapipe_axis,
        v2_axis=v2_axis,
        now=now,
        phase=phase,
    )

    fallback_reduction_b = (
        state.strategy_a.fallback_mediapipe_frames
        - state.strategy_b.fallback_mediapipe_frames
    )
    fallback_reduction_c = (
        state.strategy_a.fallback_mediapipe_frames
        - state.strategy_c.fallback_mediapipe_frames
    )
    baseline_fallback_frames = state.strategy_a.fallback_mediapipe_frames
    baseline_switches = state.strategy_a.source_switch_count
    return {
        "policy_definition": {
            "Strategy_A": "STRONG_PRIMARY_ELSE_MEDIAPIPE",
            "Strategy_B": "STRONG_OR_TEMPORALLY_VALID_DEGRADED_ELSE_MEDIAPIPE",
            "Strategy_C": "STRONG_OR_TEMPORALLY_VALID_DEGRADED_ELSE_HOLD_THEN_NONE",
            "counting_convention": (
                "INITIAL_SOURCE_NOT_COUNTED;PRIMARY_TO_HOLD_IS_A_STATE_"
                "TRANSITION_NOT_A_PROVIDER_SWITCH_OR_FILTER_CLEAR"
            ),
        },
        "inputs": {
            "primary_quality": estimate.primary_quality_level,
            "primary_available": primary_axis is not None,
            "mediapipe_available": mediapipe_axis is not None,
            "degraded_temporally_valid": temporally_valid_degraded,
            "v2_2_axis_available": v2_axis is not None,
            "gesture": gesture,
        },
        "frame_directions": {
            "strategy_b_direction": axis_b,
            "strategy_c_direction": axis_c,
            "actual_direction": actual_axis,
            "actual_source": getattr(estimate, "active_source", None),
        },
        "v2_2_comparison": {
            "b_vs_v2_angle_deg": _unsigned_angle_deg(axis_b, v2_axis),
            "c_vs_v2_angle_deg": _unsigned_angle_deg(axis_c, v2_axis),
        },
        "hold_diagnostic": {
            "active": state.hold_active,
            "entered_this_frame": entered_hold,
            "exited_this_frame": exited_hold,
            "hold_duration_sec": (
                current_hold_duration
                if state.hold_active else (
                    completed_hold_this_frame.get("hold_duration_sec")
                    if completed_hold_this_frame is not None else None
                )
            ),
            "hold_duration": (
                current_hold_duration
                if state.hold_active else (
                    completed_hold_this_frame.get("hold_duration_sec")
                    if completed_hold_this_frame is not None else None
                )
            ),
            "hold_start_source": (
                state.hold_start_source
                if state.hold_active else (
                    completed_hold_this_frame.get("hold_start_source")
                    if completed_hold_this_frame is not None else None
                )
            ),
            "hold_end_source": (
                source_c if exited_hold else None
            ),
            "hold_direction_change_deg": (
                _unsigned_angle_deg(state.hold_start_axis, axis_c)
                if state.hold_active else (
                    completed_hold_this_frame.get(
                        "hold_direction_change_deg"
                    ) if completed_hold_this_frame is not None else None
                )
            ),
            "hold_vs_v2_angle_deg": (
                current_hold_v2_angle
                if state.hold_active else (
                    completed_hold_this_frame.get("hold_vs_v2_angle_deg")
                    if completed_hold_this_frame is not None else None
                )
            ),
            "hold_count": state.hold_count,
            "completed_hold_count": state.completed_hold_count,
            "total_completed_hold_duration_sec": (
                state.total_completed_hold_duration_sec
            ),
            "max_completed_hold_duration_sec": (
                state.max_completed_hold_duration_sec
            ),
            "last_completed_hold": state.last_completed_hold,
        },
        "experiment_phase_shadow": {
            "phase": phase,
            "confidence": phase_confidence,
            "evidence": phase_evidence,
            "ground_truth": False,
            "Strategy_A": _phase_stats_diagnostics(
                state.strategy_a_phase_stats
            ),
            "Strategy_B": _phase_stats_diagnostics(
                state.strategy_b_phase_stats
            ),
            "Strategy_C": _phase_stats_diagnostics(
                state.strategy_c_phase_stats
            ),
        },
        "Strategy_A": result_a,
        "Strategy_B": result_b,
        "Strategy_C": {
            **result_c,
            "hold_timeout_sec": state.hold_timeout_sec,
            "hold_age_sec": hold_age_sec,
        },
        "Strategy_B_plus": {
            "hold_0_2s": b_plus_02,
            "hold_0_5s": b_plus_05,
            "hold_1_0s": b_plus_10,
        },
        "comparison": {
            "strategy_b_fallback_frames_avoided_vs_a": fallback_reduction_b,
            "strategy_c_fallback_frames_avoided_vs_a": fallback_reduction_c,
            "strategy_b_fallback_reduction_ratio_vs_a": (
                fallback_reduction_b / baseline_fallback_frames
                if baseline_fallback_frames > 0 else None
            ),
            "strategy_c_fallback_reduction_ratio_vs_a": (
                fallback_reduction_c / baseline_fallback_frames
                if baseline_fallback_frames > 0 else None
            ),
            "strategy_b_source_switch_reduction_vs_a": (
                state.strategy_a.source_switch_count
                - state.strategy_b.source_switch_count
            ),
            "strategy_c_source_switch_reduction_vs_a": (
                state.strategy_a.source_switch_count
                - state.strategy_c.source_switch_count
            ),
            "strategy_b_source_switch_reduction_ratio_vs_a": (
                (
                    state.strategy_a.source_switch_count
                    - state.strategy_b.source_switch_count
                ) / baseline_switches
                if baseline_switches > 0 else None
            ),
            "strategy_c_source_switch_reduction_ratio_vs_a": (
                (
                    state.strategy_a.source_switch_count
                    - state.strategy_c.source_switch_count
                ) / baseline_switches
                if baseline_switches > 0 else None
            ),
            "strategy_b_filter_clear_reduction_vs_a": (
                state.strategy_a.filter_clear_count
                - state.strategy_b.filter_clear_count
            ),
            "strategy_c_filter_clear_reduction_vs_a": (
                state.strategy_a.filter_clear_count
                - state.strategy_c.filter_clear_count
            ),
            "hand_direction_risk_reduction": {
                "strategy_b_avoided_mediapipe_frames": fallback_reduction_b,
                "strategy_c_avoided_mediapipe_frames": fallback_reduction_c,
            },
            "large_angle_hold_risk": result_c["hold_risk"],
        },
        "hand_direction_risk": {
            "Strategy_B": result_b["mediapipe_risk"],
            "Strategy_C": result_c["mediapipe_risk"],
        },
        "shadow_only": True,
    }
