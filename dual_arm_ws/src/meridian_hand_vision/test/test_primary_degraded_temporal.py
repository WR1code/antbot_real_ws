from types import SimpleNamespace

import pytest

from meridian_hand_vision.primary_degraded_temporal import (
    PrimaryDegradedTemporalShadowState,
    update_primary_degraded_temporal_shadow,
)


def _estimate(
    *,
    axis=(1.0, 0.0, 0.0),
    quality="PRIMARY_DEGRADED",
    reason="INLIER_RATIO_LOW",
    source="FALLBACK_MEDIAPIPE",
    changed=False,
    length=0.120,
    rms=0.003,
):
    direction = None if axis is None else SimpleNamespace(vector=axis)
    return SimpleNamespace(
        primary_direction=direction,
        primary_quality_level=quality,
        reason=reason,
        active_source=source,
        source_changed=changed,
        axis_length_m=length,
        rms_m=rms,
    )


def test_temporally_consistent_degraded_would_keep_primary_in_shadow():
    state = PrimaryDegradedTemporalShadowState()
    result = None
    for axis, length, rms in (
        ((1.0, 0.000, 0.0), 0.120, 0.0030),
        ((1.0, 0.010, 0.0), 0.121, 0.0031),
        ((1.0, 0.015, 0.0), 0.119, 0.0029),
        ((1.0, 0.020, 0.0), 0.122, 0.0032),
        ((1.0, 0.025, 0.0), 0.120, 0.0030),
    ):
        result = update_primary_degraded_temporal_shadow(
            state, _estimate(axis=axis, length=length, rms=rms)
        )
    assert result["degraded_temporal_state"] == "DEGRADED_TEMPORALLY_VALID"
    assert result["shadow_would_keep_primary"] is True
    assert result["valid"] is True
    assert result["degraded_streak"] == 5


def test_four_of_five_degraded_frames_can_be_valid():
    state = PrimaryDegradedTemporalShadowState()
    update_primary_degraded_temporal_shadow(
        state, _estimate(quality="PRIMARY_STRONG", reason="NONE")
    )
    result = None
    for offset in (0.0, 0.01, 0.02, 0.03):
        result = update_primary_degraded_temporal_shadow(
            state, _estimate(axis=(1.0, offset, 0.0))
        )
    assert result["degraded_frame_count"] == 4
    assert result["valid"] is True


def test_source_reset_keeps_shadow_invalid_for_current_window():
    state = PrimaryDegradedTemporalShadowState()
    update_primary_degraded_temporal_shadow(
        state, _estimate(changed=True)
    )
    result = None
    for _ in range(3):
        result = update_primary_degraded_temporal_shadow(state, _estimate())
    assert result["valid"] is False
    assert result["source_change_count"] == 1
    assert "SOURCE_RESET_IN_WINDOW" in result["reason"]


def test_axis_jump_marks_degraded_window_unstable():
    state = PrimaryDegradedTemporalShadowState()
    result = None
    for axis in (
        (1.0, 0.0, 0.0),
        (1.0, 0.01, 0.0),
        (0.0, 1.0, 0.0),
        (1.0, 0.02, 0.0),
        (1.0, 0.03, 0.0),
    ):
        result = update_primary_degraded_temporal_shadow(
            state, _estimate(axis=axis)
        )
    assert result["valid"] is False
    assert result["axis_delta_p95_deg"] > 10.0
    assert "AXIS_DELTA_P95_HIGH" in result["reason"]


def test_valid_shadow_does_not_claim_keep_when_primary_is_active():
    state = PrimaryDegradedTemporalShadowState()
    result = None
    for _ in range(5):
        result = update_primary_degraded_temporal_shadow(
            state, _estimate(source="PRIMARY_3D_FOREARM")
        )
    assert result["valid"] is True
    assert result["shadow_would_keep_primary"] is False


def test_length_and_rms_thresholds_are_strict():
    state = PrimaryDegradedTemporalShadowState()
    result = None
    for length, rms in (
        (0.100, 0.003),
        (0.100, 0.003),
        (0.100, 0.003),
        (0.116, 0.007),
        (0.100, 0.003),
    ):
        result = update_primary_degraded_temporal_shadow(
            state, _estimate(length=length, rms=rms)
        )
    assert result["valid"] is False
    assert result["length_change_ratio"] > 0.15
    assert result["rms_p95"] > 0.006
    assert "LENGTH_CHANGE_HIGH" in result["reason"]
    assert "RMS_P95_HIGH" in result["reason"]
