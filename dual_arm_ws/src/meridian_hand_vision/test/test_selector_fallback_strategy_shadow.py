import math
from types import SimpleNamespace

import pytest

from meridian_hand_vision.selector_fallback_strategy_shadow import (
    SelectorFallbackStrategyShadowState,
    update_selector_fallback_strategy_shadow,
)


def _direction(axis):
    return None if axis is None else SimpleNamespace(vector=axis)


def _estimate(
    quality,
    primary=(1.0, 0.0, 0.0),
    media=(0.0, 1.0, 0.0),
    actual=(0.0, 1.0, 0.0),
):
    return SimpleNamespace(
        primary_quality_level=quality,
        primary_direction=_direction(primary),
        mediapipe_direction=_direction(media),
        direction=_direction(actual),
        active_source="FALLBACK_MEDIAPIPE",
    )


def _temporal(valid=False):
    return {
        "degraded_temporal_state": (
            "DEGRADED_TEMPORALLY_VALID" if valid else "DEGRADED_UNSTABLE"
        ),
        "valid": valid,
    }


def test_strong_primary_is_used_by_all_strategies():
    state = SelectorFallbackStrategyShadowState()
    result = update_selector_fallback_strategy_shadow(
        state, _estimate("PRIMARY_STRONG"), _temporal(), 1.0
    )
    assert result["Strategy_A"]["current_source"] == "PRIMARY_3D_FOREARM"
    assert result["Strategy_B"]["current_source"] == "PRIMARY_3D_FOREARM"
    assert result["Strategy_C"]["current_source"] == "PRIMARY_3D_FOREARM"


def test_temporally_valid_degraded_differs_between_a_and_b():
    state = SelectorFallbackStrategyShadowState()
    estimate = _estimate("PRIMARY_DEGRADED")
    result = update_selector_fallback_strategy_shadow(
        state, estimate, _temporal(True), 1.0
    )
    assert result["Strategy_A"]["current_source"] == "FALLBACK_MEDIAPIPE"
    assert result["Strategy_B"]["current_source"] == "PRIMARY_3D_FOREARM"
    assert result["Strategy_C"]["current_source"] == "PRIMARY_3D_FOREARM"
    assert result["comparison"]["strategy_b_fallback_frames_avoided_vs_a"] == 1
    assert estimate.primary_quality_level == "PRIMARY_DEGRADED"
    assert estimate.active_source == "FALLBACK_MEDIAPIPE"


def test_strategy_c_holds_then_times_out_without_using_mediapipe():
    state = SelectorFallbackStrategyShadowState(hold_timeout_sec=0.5)
    update_selector_fallback_strategy_shadow(
        state, _estimate("PRIMARY_STRONG"), _temporal(), 1.0
    )
    held = update_selector_fallback_strategy_shadow(
        state, _estimate("PRIMARY_REJECTED"), _temporal(), 1.2
    )
    assert held["Strategy_C"]["current_source"] == "HOLD_LAST_PRIMARY_DIRECTION"
    assert held["Strategy_C"]["hold_frames"] == 1
    assert held["Strategy_C"]["filter_clear_count"] == 0

    timed_out = update_selector_fallback_strategy_shadow(
        state, _estimate("PRIMARY_REJECTED"), _temporal(), 1.6
    )
    assert timed_out["Strategy_C"]["current_source"] == "NONE"
    assert timed_out["Strategy_C"]["none_frames"] == 1
    assert timed_out["Strategy_C"]["filter_clear_count"] == 1


def test_source_switch_and_direction_delta_are_counted_independently():
    state = SelectorFallbackStrategyShadowState()
    update_selector_fallback_strategy_shadow(
        state, _estimate("PRIMARY_STRONG"), _temporal(), 1.0
    )
    result = update_selector_fallback_strategy_shadow(
        state, _estimate("PRIMARY_REJECTED"), _temporal(), 1.1
    )
    assert result["Strategy_A"]["source_switch_count"] == 1
    assert result["Strategy_A"]["filter_clear_count"] == 1
    assert result["Strategy_A"]["direction_delta_max_deg"] == pytest.approx(90.0)


def test_hold_risk_uses_v2_axis_without_affecting_hold_result():
    state = SelectorFallbackStrategyShadowState()
    update_selector_fallback_strategy_shadow(
        state,
        _estimate("PRIMARY_STRONG"),
        _temporal(),
        1.0,
        v2_2_axis=(1.0, 0.0, 0.0),
    )
    result = update_selector_fallback_strategy_shadow(
        state,
        _estimate("PRIMARY_REJECTED"),
        _temporal(),
        1.1,
        v2_2_axis=(0.0, 1.0, 0.0),
    )
    risk = result["Strategy_C"]["hold_risk"]
    assert risk["comparable_frames"] == 1
    assert risk["angle_max_deg"] == pytest.approx(90.0)
    assert risk["over_30deg_frames"] == 1
    assert result["shadow_only"] is True


def test_no_primary_history_means_strategy_c_none():
    state = SelectorFallbackStrategyShadowState()
    result = update_selector_fallback_strategy_shadow(
        state,
        _estimate("PRIMARY_REJECTED", primary=None),
        _temporal(),
        1.0,
    )
    assert result["Strategy_A"]["current_source"] == "FALLBACK_MEDIAPIPE"
    assert result["Strategy_C"]["current_source"] == "NONE"
    assert result["Strategy_C"]["hold_frames"] == 0


def test_frame_directions_and_v2_comparison_are_explicit():
    state = SelectorFallbackStrategyShadowState()
    result = update_selector_fallback_strategy_shadow(
        state,
        _estimate("PRIMARY_DEGRADED"),
        _temporal(True),
        1.0,
        v2_2_axis=(1.0, 0.0, 0.0),
    )
    assert result["frame_directions"]["strategy_b_direction"] == pytest.approx(
        (1.0, 0.0, 0.0)
    )
    assert result["frame_directions"]["actual_direction"] == pytest.approx(
        (0.0, 1.0, 0.0)
    )
    assert result["v2_2_comparison"]["b_vs_v2_angle_deg"] == pytest.approx(0.0)


def test_hold_lifecycle_records_start_end_and_direction_change():
    state = SelectorFallbackStrategyShadowState()
    update_selector_fallback_strategy_shadow(
        state, _estimate("PRIMARY_STRONG"), _temporal(), 1.0
    )
    entered = update_selector_fallback_strategy_shadow(
        state, _estimate("PRIMARY_REJECTED"), _temporal(), 1.1
    )
    assert entered["hold_diagnostic"]["entered_this_frame"] is True
    assert entered["hold_diagnostic"]["hold_start_source"] == "PRIMARY_3D_FOREARM"

    exited = update_selector_fallback_strategy_shadow(
        state,
        _estimate(
            "PRIMARY_STRONG",
            primary=(0.0, 1.0, 0.0),
            actual=(0.0, 1.0, 0.0),
        ),
        _temporal(),
        1.3,
    )
    event = exited["hold_diagnostic"]["last_completed_hold"]
    assert exited["hold_diagnostic"]["exited_this_frame"] is True
    assert event["hold_end_source"] == "PRIMARY_3D_FOREARM"
    assert event["hold_duration_sec"] == pytest.approx(0.2)
    assert event["hold_end_direction_change_deg"] == pytest.approx(90.0)


def test_mediapipe_risk_compares_with_primary_and_v2():
    state = SelectorFallbackStrategyShadowState()
    result = update_selector_fallback_strategy_shadow(
        state,
        _estimate("PRIMARY_REJECTED"),
        _temporal(),
        1.0,
        v2_2_axis=(1.0, 0.0, 0.0),
    )
    risk = result["hand_direction_risk"]["Strategy_B"]
    assert risk["usage_frames"] == 1
    assert risk["vs_v2_2_median_deg"] == pytest.approx(90.0)
    assert risk["vs_primary_median_deg"] == pytest.approx(90.0)


def test_experiment_phase_shadow_marks_rotation_and_fist_as_inferred():
    state = SelectorFallbackStrategyShadowState()
    result = None
    for index, degrees in enumerate((0.0, 3.0, 6.0, 9.0)):
        radians = degrees * 3.141592653589793 / 180.0
        result = update_selector_fallback_strategy_shadow(
            state,
            _estimate(
                "PRIMARY_STRONG",
                media=(float(math.cos(radians)),
                       float(math.sin(radians)), 0.0),
            ),
            _temporal(),
            1.0 + index * 0.1,
            gesture="open_palm",
        )
    phase = result["experiment_phase_shadow"]
    assert phase["phase"] == "PALM_ROTATION"
    assert phase["ground_truth"] is False

    fist = update_selector_fallback_strategy_shadow(
        state,
        _estimate("PRIMARY_STRONG"),
        _temporal(),
        2.0,
        gesture="fist",
    )
    assert fist["experiment_phase_shadow"]["phase"] == "FIST"


def test_b_plus_timeout_variants_diverge_without_using_mediapipe():
    state = SelectorFallbackStrategyShadowState()
    update_selector_fallback_strategy_shadow(
        state, _estimate("PRIMARY_STRONG"), _temporal(), 1.0
    )
    result = update_selector_fallback_strategy_shadow(
        state, _estimate("PRIMARY_REJECTED"), _temporal(), 1.3
    )
    variants = result["Strategy_B_plus"]
    assert variants["hold_0_2s"]["current_source"] == "NONE"
    assert variants["hold_0_5s"]["current_source"] == "HOLD_LAST_PRIMARY_DIRECTION"
    assert variants["hold_1_0s"]["current_source"] == "HOLD_LAST_PRIMARY_DIRECTION"
    for variant in variants.values():
        assert variant["fallback_mediapipe_frames"] == 0
        assert variant["media_pipe_usage_ratio"] == 0.0
        assert variant["shadow_only"] is True


def test_b_plus_counts_temporally_valid_degraded_as_primary():
    state = SelectorFallbackStrategyShadowState()
    result = update_selector_fallback_strategy_shadow(
        state,
        _estimate("PRIMARY_DEGRADED"),
        _temporal(True),
        1.0,
        gesture="fist",
    )
    for variant in result["Strategy_B_plus"].values():
        assert variant["primary_frames"] == 1
        assert variant["degraded_keep_frames"] == 1
        assert variant["phase_statistics"]["FIST"]["degraded_keep_frames"] == 1


def test_b_plus_reports_hold_and_direction_v2_risk():
    state = SelectorFallbackStrategyShadowState()
    update_selector_fallback_strategy_shadow(
        state,
        _estimate("PRIMARY_STRONG"),
        _temporal(),
        1.0,
        v2_2_axis=(1.0, 0.0, 0.0),
    )
    result = update_selector_fallback_strategy_shadow(
        state,
        _estimate("PRIMARY_REJECTED"),
        _temporal(),
        1.1,
        v2_2_axis=(0.0, 1.0, 0.0),
    )
    variant = result["Strategy_B_plus"]["hold_0_2s"]
    assert variant["hold_frames"] == 1
    assert variant["hold_vs_v2_angle_max_deg"] == pytest.approx(90.0)
    assert variant["direction_vs_v2_mean_deg"] == pytest.approx(45.0)
    assert variant["direction_vs_v2_p95_deg"] == pytest.approx(85.5)
