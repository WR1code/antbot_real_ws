from types import SimpleNamespace

import pytest

from meridian_hand_vision.primary_degraded_keep_shadow import (
    PrimaryDegradedKeepShadowState,
    update_primary_degraded_keep_shadow,
)


def _direction(axis):
    return None if axis is None else SimpleNamespace(vector=axis)


def _estimate(
    source,
    *,
    direction=(1.0, 0.0, 0.0),
    primary=(1.0, 0.0, 0.0),
    changed=False,
    rms=0.003,
):
    return SimpleNamespace(
        active_source=source,
        source_changed=changed,
        direction=_direction(direction),
        primary_direction=_direction(primary),
        rms_m=rms,
    )


def _temporal(valid):
    return {
        "degraded_temporal_state": (
            "DEGRADED_TEMPORALLY_VALID" if valid else "DEGRADED_UNSTABLE"
        ),
        "valid": valid,
        "axis_delta_p95_deg": 2.0,
        "length_change_ratio": 0.04,
    }


def test_valid_degraded_fallback_is_kept_as_primary_only_in_shadow():
    state = PrimaryDegradedKeepShadowState()
    estimate = _estimate("FALLBACK_MEDIAPIPE", changed=True)
    result = update_primary_degraded_keep_shadow(
        state, estimate, _temporal(True), None
    )
    assert result["current_actual_source"] == "FALLBACK_MEDIAPIPE"
    assert result["current_shadow_source"] == "PRIMARY_3D_FOREARM"
    assert result["shadow_keep_applied"] is True
    assert result["actual_fallback_frames"] == 1
    assert result["shadow_primary_frames"] == 1
    assert estimate.active_source == "FALLBACK_MEDIAPIPE"


def test_unstable_degraded_remains_fallback_in_shadow():
    state = PrimaryDegradedKeepShadowState()
    result = update_primary_degraded_keep_shadow(
        state,
        _estimate("FALLBACK_MEDIAPIPE", changed=True),
        _temporal(False),
        None,
    )
    assert result["current_shadow_source"] == "FALLBACK_MEDIAPIPE"
    assert result["shadow_keep_applied"] is False
    assert result["shadow_fallback_frames"] == 1


def test_keep_primary_can_save_an_actual_source_switch():
    state = PrimaryDegradedKeepShadowState()
    update_primary_degraded_keep_shadow(
        state, _estimate("PRIMARY_3D_FOREARM"), _temporal(False), None
    )
    result = update_primary_degraded_keep_shadow(
        state,
        _estimate("FALLBACK_MEDIAPIPE", changed=True),
        _temporal(True),
        None,
    )
    assert result["actual_source_switch_count"] == 1
    assert result["shadow_source_switch_count"] == 1
    # The initial INVALID->PRIMARY shadow transition is counted; compare the
    # same initialization convention before interpreting the cumulative delta.
    assert result["saved_switch_count"] == 0
    result = update_primary_degraded_keep_shadow(
        state,
        _estimate("PRIMARY_3D_FOREARM", changed=True),
        _temporal(False),
        None,
    )
    assert result["actual_source_switch_count"] == 2
    assert result["shadow_source_switch_count"] == 1
    assert result["saved_switch_count"] == 1


def test_parallel_filter_produces_direction_without_touching_actual_filter():
    state = PrimaryDegradedKeepShadowState(filter_window_size=3)
    result = None
    for _ in range(3):
        result = update_primary_degraded_keep_shadow(
            state,
            _estimate("FALLBACK_MEDIAPIPE"),
            _temporal(True),
            _direction((1.0, 0.0, 0.0)),
        )
    assert result["shadow_simulated_direction"] == pytest.approx(
        (1.0, 0.0, 0.0)
    )
    assert result["actual_vs_shadow_angle_deg"] == pytest.approx(0.0)


def test_retained_risk_records_v2_angle_and_threshold_metrics():
    state = PrimaryDegradedKeepShadowState()
    result = update_primary_degraded_keep_shadow(
        state,
        _estimate("FALLBACK_MEDIAPIPE", primary=(1.0, 0.0, 0.0)),
        _temporal(True),
        None,
        v2_2_axis=(0.0, 1.0, 0.0),
    )
    risk = result["retained_degraded_risk"]
    assert risk["axis_delta_p95_deg"] == 2.0
    assert risk["rms_m"] == 0.003
    assert risk["length_change_ratio"] == 0.04
    assert risk["v2_2_angle_deg"] == pytest.approx(90.0)
    assert result["risk_summary"]["retained_degraded_count"] == 1


def test_missing_primary_axis_cannot_be_kept():
    state = PrimaryDegradedKeepShadowState()
    result = update_primary_degraded_keep_shadow(
        state,
        _estimate("FALLBACK_MEDIAPIPE", primary=None),
        _temporal(True),
        None,
    )
    assert result["shadow_keep_applied"] is False
    assert result["current_shadow_source"] == "FALLBACK_MEDIAPIPE"
