from types import SimpleNamespace

import numpy as np
import pytest

from meridian_hand_vision.pulse_region import (
    CameraIntrinsics,
    PulseRegion,
    StableArmDirectionFilter,
    StablePulseFilter,
    deproject_pixel,
    estimate_pulse_region,
    estimate_pulse_region_diagnostic,
    estimate_arm_direction,
    pulse_pixel,
)


def _point(x, y):
    return SimpleNamespace(x=x, y=y, z=0.0)


def _hand():
    points = [_point(0.50, 0.62) for _ in range(21)]
    points[0] = _point(0.50, 0.66)
    points[5] = _point(0.42, 0.50)
    points[9] = _point(0.49, 0.47)
    points[13] = _point(0.56, 0.49)
    points[17] = _point(0.62, 0.53)
    return points


def test_pulse_pixel_is_proximal_and_on_thumb_side():
    target = pulse_pixel(_hand(), 1000, 1000)
    assert target[1] > 660
    assert target[0] < 500


def test_deprojection_uses_pinhole_intrinsics_and_unmirrors():
    intrinsics = CameraIntrinsics(500.0, 500.0, 320.0, 240.0, 640, 480)
    point = deproject_pixel((420, 240), 1.0, intrinsics, 640, 480)
    mirrored = deproject_pixel((219, 240), 1.0, intrinsics, 640, 480, mirrored=True)
    assert np.allclose(point, (0.2, 0.0, 1.0), atol=0.002)
    assert np.allclose(mirrored, point, atol=0.002)


def test_region_uses_registered_depth_and_camera_info():
    depth = np.full((1000, 1000), 0.75, dtype=np.float32)
    intrinsics = CameraIntrinsics(800.0, 800.0, 500.0, 500.0, 1000, 1000)
    region = estimate_pulse_region(_hand(), depth, intrinsics)
    assert region is not None
    assert region.point_m[2] == 0.75
    assert 0.006 <= region.radius_m <= 0.018


def test_region_diagnostic_reports_depth_rejection_and_raw_pixel():
    depth = np.zeros((1000, 1000), dtype=np.float32)
    intrinsics = CameraIntrinsics(800.0, 800.0, 500.0, 500.0, 1000, 1000)
    region, reason, details = estimate_pulse_region_diagnostic(
        _hand(), depth, intrinsics
    )

    assert region is None
    assert reason == "INVALID_DEPTH"
    assert details["raw_pixel_uv"] is not None
    assert details["raw_depth"] is None


def _sample(x=0.1):
    return PulseRegion((10, 10), (x, 0.2, 0.5), 0.01)


@pytest.mark.parametrize("rate_hz", [6, 13, 30])
def test_target_stabilization_has_no_duration_or_fps_gate(rate_hz):
    stable = StablePulseFilter()
    first_ready = None
    for index in range(2 * rate_hz + 1):
        timestamp = index / rate_hz
        result = stable.update(_sample(0.1 + (index % 3 - 1) * 0.0005), timestamp)
        if result is not None and first_ready is None:
            first_ready = timestamp
    assert first_ready is not None
    assert first_ready <= 2.0 / rate_hz + 1e-6
    metrics = stable.diagnostics(first_ready)
    assert metrics["stable_required_ms"] == 0.0
    assert metrics["vision_valid_samples"] >= 3
    assert metrics["spatial_jitter_mm"] < 8.0


def test_single_or_two_missing_frames_preserve_history_without_republishing():
    stable = StablePulseFilter()
    assert stable.update(_sample(), 0.0) is None
    assert stable.update(None, 0.1) is None
    assert stable.update(_sample(), 0.2) is None
    assert stable.update(None, 0.3) is None
    assert stable.update(None, 0.4) is None
    assert stable.update(_sample(), 0.5) is not None
    assert stable.reset_count == 0
    assert stable.diagnostics(0.5)["vision_valid_samples"] == 3


def test_pre_latch_timer_survives_quarter_second_camera_gap():
    stable = StablePulseFilter()
    assert stable.update(_sample(), 0.0) is None
    assert stable.update(_sample(), 0.15) is None
    assert stable.update(_sample(), 0.30) is not None
    before = stable.diagnostics(0.43)
    assert before["target_stable"]
    stable.observe_gap(0.68)
    during = stable.diagnostics(0.68)
    assert during["target_stable"]
    assert during["stable_reset_count"] == 0
    assert stable.update(_sample(), 0.69) is not None
    after = stable.diagnostics(0.69)
    assert after["stable_latched"]
    assert after["stable_reset_count"] == 0


def test_elapsed_time_alone_cannot_pass_without_three_valid_samples():
    stable = StablePulseFilter()
    assert stable.update(_sample(), 0.0) is None
    assert stable.update(_sample(), 0.6) is None
    assert stable.diagnostics(0.6)["vision_valid_samples"] == 2


def test_spatial_jitter_p95_is_the_acquisition_gate():
    stable = StablePulseFilter()
    stable.update(_sample(0.1), 0.0)
    stable.update(_sample(0.1), 0.1)
    assert stable.update(_sample(0.112), 0.2) is None
    metrics = stable.diagnostics(0.2)
    assert metrics["jitter_p95_mm"] > 5.0
    assert stable.reset_count == 0  # Soft variation is not a hard jump.


def test_acquisition_transient_does_not_reset_but_identity_and_timeout_do():
    stable = StablePulseFilter()
    stable.update(_sample(), 0.0, "Left")
    stable.update(_sample(), 0.2, "Left")
    assert stable.update(_sample(0.2), 0.3, "Left") is None
    assert stable.diagnostics(0.3)["last_stable_reset_reason"] == "NONE"
    assert stable.diagnostics(0.3)["vision_valid_samples"] == 3
    stable.update(_sample(0.2), 0.4, "Right")
    assert stable.diagnostics(0.4)["last_stable_reset_reason"] == "HAND_IDENTITY_CHANGED"
    assert stable.update(None, 2.5) is None
    assert stable.diagnostics(2.5)["last_stable_reset_reason"] == "TARGET_HARD_TIMEOUT"
    assert stable.reset_count == 2


def _latched_filter():
    stable = StablePulseFilter()
    for index in range(6):
        result = stable.update(_sample(), index * 0.1, "Left")
    assert result is not None
    assert stable.stable_latched
    return stable


@pytest.mark.parametrize(
    "gap_age,state", [
        (0.4, "TARGET_STABLE"),
        (0.8, "TARGET_TEMPORARILY_STALE"),
        (1.8, "WAIT_TARGET_RECOVERY"),
    ],
)
def test_data_gap_preserves_latch_frozen_target_and_timer(gap_age, state):
    stable = _latched_filter()
    frozen = stable.frozen_region
    stable_since = stable._stable_since
    assert stable.update(None, 0.5 + gap_age) is None
    metrics = stable.diagnostics(0.5 + gap_age)
    assert metrics["target_tracking_state"] == state
    assert metrics["stable_latched"]
    assert metrics["frozen_target"] == frozen.point_m
    assert stable._stable_since == stable_since
    assert stable.reset_count == 0


@pytest.mark.parametrize("gap_age", [0.4, 0.8, 1.8])
def test_same_target_reacquires_immediately_without_old_target_republish(gap_age):
    stable = _latched_filter()
    stable.observe_gap(0.5 + gap_age)
    assert stable.diagnostics(0.5 + gap_age)["stable_latched"]
    result = stable.update(_sample(0.1028), 0.5 + gap_age)
    assert result is not None
    assert result.point_m[0] == pytest.approx(0.1, abs=0.003)
    assert stable.diagnostics(0.5 + gap_age)["reacquire_result"] == "REACQUIRED_SAME_TARGET"
    assert stable.reset_count == 0


def test_8_to_15_mm_is_observed_without_reset_or_publish():
    stable = _latched_filter()
    assert stable.update(_sample(0.111), 1.2) is None
    metrics = stable.diagnostics(1.2)
    assert metrics["stable_latched"]
    assert metrics["reacquire_result"] == "OBSERVE_SPATIAL_DRIFT"
    assert stable.reset_count == 0
    assert stable.update(_sample(0.102), 1.3) is not None
    assert stable.reset_count == 0


def test_more_than_15_mm_is_target_change_and_hard_timeout_is_only_gap_reset():
    stable = _latched_filter()
    assert stable.update(_sample(0.118), 0.6) is None
    assert stable.diagnostics(0.6)["reacquire_result"] == "TARGET_CHANGED"
    assert stable.last_reset_reason == "TARGET_SPATIAL_DRIFT"
    stable = _latched_filter()
    stable.observe_gap(2.51)
    metrics = stable.diagnostics(2.51)
    assert not metrics["stable_latched"]
    assert metrics["target_tracking_state"] == "TARGET_LOST"
    assert stable.last_reset_reason == "TARGET_HARD_TIMEOUT"


def test_early_acquisition_outliers_age_out_without_reentering_view():
    stable = StablePulseFilter()
    assert stable.update(_sample(-0.0368), 0.00, "Left") is None
    assert stable.update(_sample(0.060), 0.10, "Left") is None
    for timestamp, x in ((0.20, 0.014), (0.35, 0.015), (0.50, 0.013)):
        assert stable.update(_sample(x), timestamp, "Left") is None
    result = stable.update(_sample(0.014), 0.72, "Left")
    assert result is not None
    metrics = stable.diagnostics(0.72)
    assert metrics["acquisition_state"] == "TARGET_STABLE"
    assert metrics["window_valid_samples"] >= 3
    assert metrics["jitter_p95_mm"] <= 5.0
    assert metrics["stable_reset_count"] == 0
    assert metrics["window_center_xyz"][0] == pytest.approx(0.014, abs=0.001)


def test_stable_hysteresis_holds_brief_excursion_above_8mm():
    stable = _latched_filter()
    assert stable.update(_sample(0.114), 0.60) is None
    assert stable.diagnostics(0.60)["stable_latched"]
    assert stable.update(_sample(0.114), 0.81) is None
    assert not stable.diagnostics(0.81)["stable_latched"]


def test_arm_direction_uses_registered_3d_palm_to_wrist_axis():
    depth = np.full((1000, 1000), 0.75, dtype=np.float32)
    intrinsics = CameraIntrinsics(800.0, 800.0, 500.0, 500.0, 1000, 1000)
    result = estimate_arm_direction(_hand(), depth, intrinsics)
    assert result is not None
    assert np.linalg.norm(result.vector) == pytest.approx(1.0)
    assert result.vector[1] > 0.9


def test_arm_direction_filter_treats_opposite_vectors_as_same_axis():
    from meridian_hand_vision.pulse_region import ArmDirection

    stable = StableArmDirectionFilter(window_size=3, maximum_angle_deg=5.0)
    assert stable.update(ArmDirection((2.0, 0.0, 0.0))) is None
    assert stable.update(ArmDirection((-1.0, 0.0, 0.0))) is None
    result = stable.update(ArmDirection((1.0, 0.01, 0.0)))
    assert result is not None
    assert result.vector[0] > 0.99
