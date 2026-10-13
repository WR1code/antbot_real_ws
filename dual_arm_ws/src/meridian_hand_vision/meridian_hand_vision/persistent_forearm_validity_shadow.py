"""Validity diagnostics for the persistent wrist-geometry forearm axis.

This module is deliberately downstream of V2.1/V2.2.  It observes their
objects, but never mutates them and never supplies a direction to selection,
publication, target generation, or robot control.
"""

from __future__ import annotations

from collections import Counter, deque
from dataclasses import dataclass, field
import math

import numpy as np


@dataclass(frozen=True)
class PersistentForearmValidityConfig:
    """Experimental initial thresholds; none are production-qualified."""

    minimum_consecutive_visible_frames: int = 5
    minimum_valid_updates: int = 8
    minimum_track_confidence: float = 0.70
    minimum_branch_length_m: float = 0.080
    maximum_centerline_rms_m: float = 0.004
    minimum_valid_section_count: int = 4
    maximum_branch_descriptor_angle_deg: float = 5.0
    direction_window_frames: int = 5
    maximum_direction_delta_p95_deg: float = 3.0
    maximum_single_direction_delta_deg: float = 10.0
    minimum_switch_confirmation_frames: int = 5
    maximum_rgb_depth_delta_ms: float = 30.0
    stale_timeout_ms: float = 500.0


@dataclass
class PersistentForearmValidityShadowState:
    config: PersistentForearmValidityConfig = field(
        default_factory=PersistentForearmValidityConfig
    )
    current_track_id: int | None = None
    last_active_track_id: int | None = None
    last_valid_track_id: int | None = None
    direction_history: deque = field(default_factory=deque, repr=False)
    last_sample_stamp_ns: int | None = None
    last_measurement_time_sec: float | None = None
    first_observed_time_sec: float | None = None
    milestone_times_sec: dict[str, float] = field(default_factory=dict)
    condition_pass: Counter = field(default_factory=Counter)
    condition_fail: Counter = field(default_factory=Counter)
    state_counts: Counter = field(default_factory=Counter)
    reject_counts: Counter = field(default_factory=Counter)
    phase_counts: dict[str, Counter] = field(default_factory=dict)
    phase_direction_deltas_deg: dict[str, list[float]] = field(
        default_factory=dict
    )
    total_frames: int = 0
    first_detection_time_sec: float | None = None
    first_valid_time_sec: float | None = None
    valid_segment_start_sec: float | None = None
    longest_valid_duration_sec: float = 0.0
    valid_direction_deltas_deg: list[float] = field(default_factory=list)
    previous_valid_direction: tuple[float, float, float] | None = None
    track_switch_count: int = 0
    track_absent_since_sec: float | None = None
    track_rebuild_durations_ms: list[float] = field(default_factory=list)


def _axis(value):
    if value is None:
        return None
    vector = np.asarray(value, dtype=np.float64)
    if vector.shape != (3,) or not np.all(np.isfinite(vector)):
        return None
    length = float(np.linalg.norm(vector))
    if length <= 1e-12:
        return None
    return vector / length


def _unsigned_angle_deg(first, second) -> float | None:
    first_axis = _axis(first)
    second_axis = _axis(second)
    if first_axis is None or second_axis is None:
        return None
    cosine = abs(float(np.dot(first_axis, second_axis)))
    return float(np.degrees(np.arccos(np.clip(cosine, 0.0, 1.0))))


def _percentile(values, percentile: float) -> float | None:
    return float(np.percentile(values, percentile)) if values else None


def _condition(passed: bool, value, threshold, relation: str) -> dict:
    return {
        "passed": bool(passed),
        "value": value,
        "threshold": threshold,
        "relation": relation,
    }


def _find_active_objects(estimate, tracking, tracker_state):
    active_track_id = getattr(tracking, "active_track_id", None)
    track = (
        getattr(tracker_state, "tracks", {}).get(active_track_id)
        if active_track_id is not None else None
    )
    tracked_branch = next((
        item for item in getattr(tracking, "branches", ())
        if active_track_id is not None
        and getattr(item, "track_id", None) == active_track_id
    ), None)
    branch = None
    if tracked_branch is not None:
        branch = next((
            item for item in getattr(estimate, "branches", ())
            if getattr(item, "branch_id", None)
            == getattr(tracked_branch, "branch_id", None)
        ), None)
    return active_track_id, track, tracked_branch, branch


def _record_milestones(
    state: PersistentForearmValidityShadowState,
    timestamp_sec: float,
    *,
    detection_valid: bool,
    wrist_seed_valid: bool,
    mask_valid: bool,
    branch_exists: bool,
    active_track_exists: bool,
    validity: str,
) -> None:
    evidence = (
        ("detection", detection_valid),
        ("depth_seed", wrist_seed_valid),
        ("mask", mask_valid),
        ("branch", branch_exists),
        ("active_track", active_track_exists),
        ("valid", validity == "VALID"),
    )
    for name, present in evidence:
        if present and name not in state.milestone_times_sec:
            state.milestone_times_sec[name] = timestamp_sec


def _summary(state: PersistentForearmValidityShadowState) -> dict:
    total = max(1, state.total_frames)
    condition_names = sorted(set(state.condition_pass) | set(state.condition_fail))
    condition_rates = {}
    for name in condition_names:
        passed = int(state.condition_pass[name])
        failed = int(state.condition_fail[name])
        evaluated = passed + failed
        condition_rates[name] = {
            "passed": passed,
            "failed": failed,
            "pass_ratio": passed / evaluated if evaluated else None,
            "reject_ratio": failed / evaluated if evaluated else None,
        }
    first_valid_ms = None
    if (
        state.first_detection_time_sec is not None
        and state.first_valid_time_sec is not None
    ):
        first_valid_ms = 1000.0 * (
            state.first_valid_time_sec - state.first_detection_time_sec
        )
    first_valid_from_observation_ms = None
    if (
        state.first_observed_time_sec is not None
        and state.first_valid_time_sec is not None
    ):
        first_valid_from_observation_ms = 1000.0 * (
            state.first_valid_time_sec - state.first_observed_time_sec
        )
    milestones = {}
    if state.first_observed_time_sec is not None:
        milestones = {
            name: 1000.0 * (value - state.first_observed_time_sec)
            for name, value in state.milestone_times_sec.items()
        }
    stage_order = (
        "detection", "depth_seed", "mask", "branch", "active_track", "valid"
    )
    stage_durations = {}
    for first_name, second_name in zip(stage_order[:-1], stage_order[1:]):
        first_time = state.milestone_times_sec.get(first_name)
        second_time = state.milestone_times_sec.get(second_name)
        stage_durations[f"{first_name}_to_{second_name}_ms"] = (
            1000.0 * (second_time - first_time)
            if first_time is not None and second_time is not None else None
        )
    return {
        "total_frames": state.total_frames,
        "state_counts": {
            name: int(state.state_counts[name])
            for name in ("ACQUIRING", "VALID", "STALE", "INVALID")
        },
        "state_ratios": {
            name: state.state_counts[name] / total
            for name in ("ACQUIRING", "VALID", "STALE", "INVALID")
        },
        "direction_availability_ratio": state.state_counts["VALID"] / total,
        "first_valid_establishment_ms": first_valid_ms,
        "first_valid_from_shadow_start_ms": first_valid_from_observation_ms,
        "longest_continuous_valid_ms": 1000.0 * state.longest_valid_duration_sec,
        "valid_direction_delta_p95_deg": _percentile(
            state.valid_direction_deltas_deg, 95.0
        ),
        "valid_direction_delta_max_deg": (
            max(state.valid_direction_deltas_deg)
            if state.valid_direction_deltas_deg else None
        ),
        "track_switch_count": state.track_switch_count,
        "track_rebuild_time_ms": (
            state.track_rebuild_durations_ms[-1]
            if state.track_rebuild_durations_ms else None
        ),
        "track_rebuild_time_history_ms": tuple(state.track_rebuild_durations_ms),
        "condition_rates": condition_rates,
        "reject_reason_counts": dict(state.reject_counts),
        "stage_milestones_from_first_observation_ms": milestones,
        "stage_durations_ms": stage_durations,
        "phase_statistics": {
            phase: {
                "frames": int(counts["frames"]),
                "valid": int(counts["VALID"]),
                "stale": int(counts["STALE"]),
                "acquiring": int(counts["ACQUIRING"]),
                "invalid": int(counts["INVALID"]),
                "valid_ratio": (
                    counts["VALID"] / counts["frames"]
                    if counts["frames"] else None
                ),
                "valid_direction_delta_p95_deg": _percentile(
                    state.phase_direction_deltas_deg.get(phase, []), 95.0
                ),
                "valid_direction_delta_max_deg": (
                    max(state.phase_direction_deltas_deg.get(phase, []))
                    if state.phase_direction_deltas_deg.get(phase) else None
                ),
            }
            for phase, counts in sorted(state.phase_counts.items())
        },
    }


def update_persistent_forearm_validity_shadow(
    state: PersistentForearmValidityShadowState,
    estimate,
    tracking,
    tracker_state,
    wrist_mask_estimate,
    timestamp_sec: float,
    *,
    rgb_stamp_ns: int | None,
    depth_stamp_ns: int | None,
    frame_seq: int | None = None,
    detection_valid: bool = False,
    v1_primary_axis=None,
    actual_selector_axis=None,
    experiment_phase: str = "UNLABELED",
) -> dict:
    """Evaluate current V2.2 evidence without changing any producer state."""
    cfg = state.config
    timestamp_sec = float(timestamp_sec)
    if state.first_observed_time_sec is None:
        state.first_observed_time_sec = timestamp_sec
    if detection_valid and state.first_detection_time_sec is None:
        state.first_detection_time_sec = timestamp_sec
    state.total_frames += 1

    active_track_id, track, tracked_branch, branch = _find_active_objects(
        estimate, tracking, tracker_state
    )
    persistent_axis = (
        getattr(getattr(track, "descriptor", None), "axis", None)
        if track is not None else None
    )
    v2_1_axis = getattr(getattr(estimate, "direction", None), "vector", None)
    overlay_axis = persistent_axis if _axis(persistent_axis) is not None else v2_1_axis
    overlay_source = (
        "V2.2_TRACK" if _axis(persistent_axis) is not None
        else "V2.1_AXIS" if _axis(v2_1_axis) is not None
        else "NONE"
    )

    active_state = getattr(tracking, "active_state", "NO_ACTIVE_TRACK")
    current_branch_associated = bool(
        active_track_id is not None
        and tracked_branch is not None
        and branch is not None
        and getattr(track, "current_branch_id", None)
        == getattr(branch, "branch_id", None)
        and active_state != "ACTIVE_MISSING"
    )

    if active_track_id != state.current_track_id:
        if state.current_track_id is not None and active_track_id is not None:
            state.track_switch_count += 1
        state.current_track_id = active_track_id
        state.direction_history.clear()
    if state.last_active_track_id is not None and active_track_id is None:
        if state.track_absent_since_sec is None:
            state.track_absent_since_sec = timestamp_sec
    elif active_track_id is not None and state.track_absent_since_sec is not None:
        state.track_rebuild_durations_ms.append(
            1000.0 * max(0.0, timestamp_sec - state.track_absent_since_sec)
        )
        state.track_absent_since_sec = None
    state.last_active_track_id = active_track_id

    timestamp_values_valid = bool(
        isinstance(rgb_stamp_ns, int)
        and isinstance(depth_stamp_ns, int)
        and rgb_stamp_ns > 0
        and depth_stamp_ns > 0
    )
    rgb_depth_delta_ms = (
        abs(rgb_stamp_ns - depth_stamp_ns) / 1_000_000.0
        if timestamp_values_valid else None
    )
    sample_stamp_ns = max(rgb_stamp_ns, depth_stamp_ns) if timestamp_values_valid else None
    timestamp_monotonic = bool(
        sample_stamp_ns is not None
        and (
            state.last_sample_stamp_ns is None
            or sample_stamp_ns >= state.last_sample_stamp_ns
        )
    )
    sample_time_consistent = bool(
        timestamp_values_valid
        and timestamp_monotonic
        and rgb_depth_delta_ms <= cfg.maximum_rgb_depth_delta_ms
    )
    if sample_stamp_ns is not None and timestamp_monotonic:
        state.last_sample_stamp_ns = sample_stamp_ns

    current_axis = None
    if branch is not None:
        current_axis = getattr(getattr(branch, "axis", None), "vector", None)
        if current_axis is None:
            current_axis = getattr(
                getattr(getattr(branch, "features", None), "descriptor", None),
                "axis",
                None,
            )
    branch_descriptor_angle = _unsigned_angle_deg(current_axis, persistent_axis)
    if current_branch_associated and _axis(persistent_axis) is not None:
        state.direction_history.append(tuple(float(v) for v in persistent_axis))
        while len(state.direction_history) > cfg.direction_window_frames:
            state.direction_history.popleft()
        state.last_measurement_time_sec = timestamp_sec

    recent_deltas = [
        _unsigned_angle_deg(first, second)
        for first, second in zip(
            tuple(state.direction_history)[:-1],
            tuple(state.direction_history)[1:],
        )
    ]
    recent_deltas = [value for value in recent_deltas if value is not None]
    direction_delta_p95 = _percentile(recent_deltas, 95.0)
    direction_delta_max = max(recent_deltas) if recent_deltas else None

    update_count = int(
        getattr(getattr(track, "feature_ema", None), "update_count", 0)
        if track is not None else 0
    )
    consecutive_hits = int(getattr(track, "consecutive_hits", 0) or 0)
    confidence = float(getattr(track, "confidence_v2", 0.0) or 0.0)
    length_m = getattr(branch, "axis_length_m", None) if branch is not None else None
    rms_m = getattr(branch, "rms_m", None) if branch is not None else None
    valid_sections = int(getattr(branch, "valid_section_count", 0) or 0)
    branch_valid = bool(getattr(branch, "valid", False)) if branch is not None else False
    wrist_seed_valid = bool(
        getattr(wrist_mask_estimate, "wrist_seed_point_m", None) is not None
    )
    mask_valid = bool(
        getattr(wrist_mask_estimate, "connected_point_count", 0) > 0
        and getattr(wrist_mask_estimate, "mask_area_px", 0) > 0
    )

    conditions = {
        "active_track_present": _condition(
            track is not None and _axis(persistent_axis) is not None,
            active_track_id,
            "non-null track with axis",
            "required",
        ),
        "track_currently_visible": _condition(
            active_state != "ACTIVE_MISSING", active_state,
            "not ACTIVE_MISSING", "required",
        ),
        "current_branch_associated": _condition(
            current_branch_associated,
            getattr(tracked_branch, "branch_id", None),
            "active track association", "required",
        ),
        "branch_valid": _condition(
            branch_valid, getattr(branch, "reason", None), "valid branch", "required"
        ),
        "consecutive_visible_frames": _condition(
            consecutive_hits >= cfg.minimum_consecutive_visible_frames,
            consecutive_hits, cfg.minimum_consecutive_visible_frames, ">=",
        ),
        "cumulative_valid_updates": _condition(
            update_count >= cfg.minimum_valid_updates,
            update_count, cfg.minimum_valid_updates, ">=",
        ),
        "track_confidence": _condition(
            confidence >= cfg.minimum_track_confidence,
            confidence, cfg.minimum_track_confidence, ">=",
        ),
        "branch_length": _condition(
            length_m is not None and length_m >= cfg.minimum_branch_length_m,
            length_m, cfg.minimum_branch_length_m, ">=",
        ),
        "centerline_rms": _condition(
            rms_m is not None and rms_m <= cfg.maximum_centerline_rms_m,
            rms_m, cfg.maximum_centerline_rms_m, "<=",
        ),
        "valid_section_count": _condition(
            valid_sections >= cfg.minimum_valid_section_count,
            valid_sections, cfg.minimum_valid_section_count, ">=",
        ),
        "branch_descriptor_axis_angle": _condition(
            branch_descriptor_angle is not None
            and branch_descriptor_angle <= cfg.maximum_branch_descriptor_angle_deg,
            branch_descriptor_angle,
            cfg.maximum_branch_descriptor_angle_deg,
            "<=",
        ),
        "direction_window_ready": _condition(
            len(state.direction_history) >= cfg.direction_window_frames,
            len(state.direction_history), cfg.direction_window_frames, ">=",
        ),
        "direction_delta_p95": _condition(
            direction_delta_p95 is not None
            and direction_delta_p95 <= cfg.maximum_direction_delta_p95_deg,
            direction_delta_p95,
            cfg.maximum_direction_delta_p95_deg,
            "<=",
        ),
        "single_direction_delta": _condition(
            direction_delta_max is not None
            and direction_delta_max <= cfg.maximum_single_direction_delta_deg,
            direction_delta_max,
            cfg.maximum_single_direction_delta_deg,
            "<=",
        ),
        "track_switch_confirmed": _condition(
            state.last_valid_track_id in (None, active_track_id)
            or consecutive_hits >= cfg.minimum_switch_confirmation_frames,
            consecutive_hits,
            cfg.minimum_switch_confirmation_frames,
            ">= after track switch",
        ),
        "sample_time_consistent": _condition(
            sample_time_consistent,
            rgb_depth_delta_ms,
            cfg.maximum_rgb_depth_delta_ms,
            "valid monotonic stamps and delta <=",
        ),
        "wrist_seed_valid": _condition(
            wrist_seed_valid,
            getattr(wrist_mask_estimate, "reason", None),
            "current depth-supported seed", "required",
        ),
        "mask_valid": _condition(
            mask_valid,
            getattr(wrist_mask_estimate, "connected_point_count", 0),
            ">0 connected points", "required",
        ),
    }

    for name, result in conditions.items():
        if result["passed"]:
            state.condition_pass[name] += 1
        else:
            state.condition_fail[name] += 1

    reject_reasons = [name.upper() for name, item in conditions.items() if not item["passed"]]
    all_valid = all(item["passed"] for item in conditions.values())
    direction_age_ms = (
        max(0.0, 1000.0 * (timestamp_sec - state.last_measurement_time_sec))
        if state.last_measurement_time_sec is not None else None
    )
    maturity_conditions = {
        "CONSECUTIVE_VISIBLE_FRAMES",
        "CUMULATIVE_VALID_UPDATES",
        "TRACK_CONFIDENCE",
        "DIRECTION_WINDOW_READY",
        "TRACK_SWITCH_CONFIRMED",
    }
    classification_reasons = set(reject_reasons)
    if not conditions["direction_window_ready"]["passed"]:
        # Delta statistics are undefined during warm-up; they are not an
        # independent hard failure until the configured window is populated.
        classification_reasons.discard("DIRECTION_DELTA_P95")
        classification_reasons.discard("SINGLE_DIRECTION_DELTA")
    if all_valid:
        validity = "VALID"
        shadow_direction = tuple(float(value) for value in persistent_axis)
        state.last_valid_track_id = active_track_id
        if state.first_valid_time_sec is None:
            state.first_valid_time_sec = timestamp_sec
    elif (
        active_state == "ACTIVE_MISSING"
        and _axis(persistent_axis) is not None
        and active_track_id == state.last_valid_track_id
        and direction_age_ms is not None
        and direction_age_ms <= cfg.stale_timeout_ms
    ):
        validity = "STALE"
        shadow_direction = None
        reject_reasons.append("HISTORICAL_DIRECTION_DISPLAY_ONLY")
    elif (
        current_branch_associated
        and _axis(persistent_axis) is not None
        and reject_reasons
        and classification_reasons.issubset(maturity_conditions)
    ):
        validity = "ACQUIRING"
        shadow_direction = None
    elif (
        _axis(v2_1_axis) is not None
        and active_track_id is None
        and active_state == "WAITING_FOR_CONFIRMATION"
    ):
        # V2.1 has current geometry, but no persistent track has completed
        # confirmation.  It remains display-only and is explicitly acquiring.
        validity = "ACQUIRING"
        shadow_direction = None
    else:
        validity = "INVALID"
        shadow_direction = None

    state.state_counts[validity] += 1
    for reason in set(reject_reasons):
        state.reject_counts[reason] += 1
    phase = experiment_phase or "UNLABELED"
    phase_counter = state.phase_counts.setdefault(phase, Counter())
    phase_counter["frames"] += 1
    phase_counter[validity] += 1

    if validity == "VALID":
        if state.valid_segment_start_sec is None:
            state.valid_segment_start_sec = timestamp_sec
        if state.previous_valid_direction is not None:
            delta = _unsigned_angle_deg(
                state.previous_valid_direction, shadow_direction
            )
            if delta is not None:
                state.valid_direction_deltas_deg.append(delta)
                state.phase_direction_deltas_deg.setdefault(phase, []).append(
                    delta
                )
        state.previous_valid_direction = shadow_direction
    else:
        if state.valid_segment_start_sec is not None:
            state.longest_valid_duration_sec = max(
                state.longest_valid_duration_sec,
                timestamp_sec - state.valid_segment_start_sec,
            )
            state.valid_segment_start_sec = None
        state.previous_valid_direction = None

    current_valid_duration_sec = (
        timestamp_sec - state.valid_segment_start_sec
        if state.valid_segment_start_sec is not None else 0.0
    )
    state.longest_valid_duration_sec = max(
        state.longest_valid_duration_sec, current_valid_duration_sec
    )

    _record_milestones(
        state,
        timestamp_sec,
        detection_valid=detection_valid,
        wrist_seed_valid=wrist_seed_valid,
        mask_valid=mask_valid,
        branch_exists=bool(getattr(estimate, "branches", ())),
        active_track_exists=track is not None,
        validity=validity,
    )

    descriptor = getattr(track, "descriptor", None)
    v1_angle = _unsigned_angle_deg(persistent_axis, v1_primary_axis)
    selector_angle = _unsigned_angle_deg(persistent_axis, actual_selector_axis)
    signed_dot_v1 = None
    if _axis(persistent_axis) is not None and _axis(v1_primary_axis) is not None:
        signed_dot_v1 = float(np.dot(_axis(persistent_axis), _axis(v1_primary_axis)))

    return {
        "v2_2_overlay_source": overlay_source,
        "overlay_direction": (
            tuple(float(value) for value in overlay_axis)
            if _axis(overlay_axis) is not None else None
        ),
        "v2_2_track_state": active_state,
        "v2_2_shadow_validity": validity,
        "v2_2_shadow_direction": shadow_direction,
        "v2_2_reject_reasons": tuple(dict.fromkeys(reject_reasons)),
        "v2_2_direction_age_ms": direction_age_ms,
        "v2_2_vs_v1_primary_angle_deg": v1_angle,
        "v2_2_vs_actual_selector_angle_deg": selector_angle,
        "v2_2_track_id": active_track_id,
        "current_branch_associated": current_branch_associated,
        "current_branch_id": getattr(branch, "branch_id", None),
        "frame_seq": frame_seq,
        "conditions": conditions,
        "experimental_thresholds": {
            key: value for key, value in vars(cfg).items()
        },
        "temporal": {
            "consecutive_visible_frames": consecutive_hits,
            "cumulative_valid_updates": update_count,
            "direction_window_size": len(state.direction_history),
            "direction_delta_p95_deg": direction_delta_p95,
            "direction_delta_max_deg": direction_delta_max,
            "rgb_depth_delta_ms": rgb_depth_delta_ms,
            "timestamp_monotonic": timestamp_monotonic,
        },
        "geometry": {
            "branch_length_m": length_m,
            "centerline_rms_m": rms_m,
            "valid_section_count": valid_sections,
            "branch_descriptor_axis_angle_deg": branch_descriptor_angle,
            "track_confidence_v2": confidence,
            "persistent_descriptor": {
                "axis": getattr(descriptor, "axis", None),
                "centroid_m": getattr(descriptor, "centroid_m", None),
                "near_endpoint_m": getattr(descriptor, "near_endpoint_m", None),
                "far_endpoint_m": getattr(descriptor, "far_endpoint_m", None),
            },
            "v1_primary_region_available": False,
            "v1_primary_region_reason": "V1.1 exposes an axis but no comparable branch descriptor",
            "v2_2_v1_signed_dot": signed_dot_v1,
            "axis_sign_relation": (
                "SAME" if signed_dot_v1 is not None and signed_dot_v1 >= 0.0
                else "OPPOSITE" if signed_dot_v1 is not None else "UNAVAILABLE"
            ),
            "coordinate_frame": "camera_optical_frame",
            "angle_definition": "unsigned_axis_angle",
        },
        "pipeline_evidence": {
            "detection_valid": detection_valid,
            "wrist_seed_valid": wrist_seed_valid,
            "wrist_mask_reason": getattr(wrist_mask_estimate, "reason", None),
            "mask_area_px": getattr(wrist_mask_estimate, "mask_area_px", 0),
            "connected_point_count": getattr(
                wrist_mask_estimate, "connected_point_count", 0
            ),
            "v2_1_reason": getattr(estimate, "reason", None),
            "branch_count": len(getattr(estimate, "branches", ())),
            "selected_branch": getattr(estimate, "selected_branch", None),
            "active_track_id": active_track_id,
            "active_track_state": active_state,
            "track_lost_reason": getattr(track, "last_lost_reason", None),
            "track_lost_detail": getattr(track, "last_lost_detail", {}),
            "sample_binding": (
                "SAME_VIEWER_CYCLE" if sample_time_consistent
                else "TIMESTAMP_INCONSISTENT"
            ),
        },
        "statistics": _summary(state),
        "shadow_only": True,
        "eligible_for_robot_target": False,
        "anatomical_ground_truth": False,
    }
