import json
from datetime import datetime

import pytest

from rebotarm_pulse.precontact_diagnostics import PlanDiagnostics, REASON_CODES
from rebotarm_pulse.precontact_monitor import (
    DirectionStabilityFilter,
    PressureDebouncer,
    RecoverableFreshnessGate,
    TargetStabilityFilter,
    target_gap_state,
)


def _stable_filter():
    tracker = TargetStabilityFilter()
    status = None
    for index in range(10):
        offset = (-0.003, 0.0, 0.003)[index % 3]
        status = tracker.add(index * 0.1, (0.20 + offset, -0.10, 0.40))
    assert status is not None and status.stable
    return tracker, status


def test_target_plus_minus_3_mm_jitter_reaches_ready():
    _tracker, status = _stable_filter()
    assert status.frozen is not None
    assert status.jitter_m <= 0.008


def test_single_12_mm_outlier_does_not_error_or_unfreeze():
    tracker, _status = _stable_filter()
    status = tracker.add(1.01, (0.212, -0.10, 0.40))
    assert not status.hard_jump
    assert not status.hard_drift
    assert status.stable  # 8 mm exit threshold must persist for 200 ms.
    assert status.stable_latched
    assert status.frozen is not None
    assert status.reacquire_result == "OBSERVE_SPATIAL_DRIFT"
    assert status.stable_reset_count == 0
    recovered = tracker.add(1.11, (0.202, -0.10, 0.40))
    assert recovered.stable
    # The real outlier stays in the 0.6 s window; hysteresis keeps the latch
    # while the P95 window converges instead of toggling immediately.
    assert recovered.reacquire_result == "OBSERVE_SPATIAL_DRIFT"


def test_persistent_more_than_15_mm_drift_returns_to_tracking():
    tracker, _status = _stable_filter()
    status = tracker.add(1.0, (0.225, -0.10, 0.40))
    assert status.hard_drift
    assert status.reacquire_result == "TARGET_CHANGED"
    assert status.stable_reset_reason == "TARGET_SPATIAL_DRIFT"
    assert tracker.status().frozen is None


def test_data_gap_keeps_frozen_target_and_stability_timer_until_hard_timeout():
    tracker, status = _stable_filter()
    frozen = status.frozen
    stable_since = tracker.stable_since
    reset_count = tracker.reset_count
    for timestamp in (1.3, 2.3, 2.8):
        during_gap = tracker.status(timestamp)
        assert during_gap.stable
        assert during_gap.stable_latched
        assert during_gap.frozen == frozen
        assert tracker.stable_since == stable_since
        assert tracker.reset_count == reset_count
    assert tracker.status(2.91).stable_latched is False
    assert tracker.last_reset_reason == "TARGET_HARD_TIMEOUT"


def test_reacquire_after_1_8_second_gap_does_not_repeat_half_second_gate():
    tracker, before = _stable_filter()
    stable_since = tracker.stable_since
    tracker.status(2.7)
    after = tracker.add(2.7, (before.frozen[0] + 0.0028, -0.10, 0.40))
    assert after.stable
    assert after.frozen == before.frozen
    assert after.reacquire_result == "REACQUIRED_SAME_TARGET"
    assert after.reacquire_distance_m <= 0.005
    assert tracker.stable_since == stable_since
    assert tracker.reset_count == 0


@pytest.mark.parametrize(
    "age,expected", [
        (0.4, "TARGET_STABLE"),
        (0.8, "TARGET_TEMPORARILY_STALE"),
        (1.8, "WAIT_TARGET_RECOVERY"),
        (2.01, "TARGET_LOST"),
    ],
)
def test_target_gap_state_is_separate_from_spatial_instability(age, expected):
    assert target_gap_state(age) == expected


def test_acquisition_transient_ages_out_of_rolling_window():
    tracker = TargetStabilityFilter()
    tracker.add(0.00, (-0.0368, 0.0811, 0.236))
    tracker.add(0.10, (0.060, 0.060, 0.270))
    for timestamp, x in ((0.20, 0.014), (0.35, 0.015), (0.50, 0.013)):
        assert not tracker.add(timestamp, (x, 0.065, 0.270)).stable
    status = tracker.add(0.72, (0.014, 0.065, 0.270))
    assert status.stable
    assert status.acquisition_state == "TARGET_STABLE"
    assert status.jitter_m <= 0.005
    assert status.sample_count >= 3
    assert status.stable_reset_count == 0


def test_arm_direction_two_degree_jitter_is_stable():
    tracker = DirectionStabilityFilter(stable_deg=3.0)
    for index, degrees in enumerate((-2.0, 0.0, 2.0)):
        radians = __import__("math").radians(degrees)
        status = tracker.add(index * 0.1, (__import__("math").sin(radians), 0.0,
                                           __import__("math").cos(radians)))
    assert status.stable
    assert status.jitter_deg <= 3.0


def test_arm_direction_waits_for_sample_count_even_with_zero_jitter():
    tracker = DirectionStabilityFilter(min_samples=3, stable_deg=3.0)
    first = tracker.add(0.0, (0.0, 0.0, 1.0))
    second = tracker.add(0.1, (0.0, 0.0, 1.0))
    assert first.jitter_deg == second.jitter_deg == 0.0
    assert (first.sample_count, second.sample_count) == (1, 2)
    assert not first.stable and not second.stable
    assert tracker.add(0.2, (0.0, 0.0, 1.0)).stable


def test_single_40_pa_spike_is_filtered():
    pressure = PressureDebouncer(threshold_pa=30.0, hold_sec=0.15, window_size=5)
    pressure.set_baseline(100.0)
    for index in range(5):
        pressure.add(index * 0.01, 100.0)
    assert not pressure.add(0.06, 140.0).abort
    assert pressure.status(0.30).delta_pa == pytest.approx(0.0)


def test_sustained_40_pa_for_more_than_150_ms_aborts():
    pressure = PressureDebouncer(threshold_pa=30.0, hold_sec=0.15, window_size=5)
    pressure.set_baseline(100.0)
    for index in range(5):
        pressure.add(index * 0.01, 100.0)
    status = None
    for index in range(12):
        status = pressure.add(0.10 + index * 0.03, 140.0)
    assert status is not None and status.abort
    assert status.over_threshold_duration_sec >= 0.15


def test_1000_pa_filtered_change_aborts_without_150_ms_hold():
    pressure = PressureDebouncer(
        threshold_pa=30.0, hold_sec=0.15, window_size=5,
        emergency_threshold_pa=1000.0,
    )
    pressure.set_baseline(100.0)
    for index in range(5):
        assert not pressure.add(index * 0.01, 100.0).abort
    assert not pressure.add(0.05, 1100.0).abort
    assert not pressure.add(0.06, 1100.0).abort
    status = pressure.add(0.07, 1100.0)
    assert status.delta_pa == pytest.approx(1000.0)
    assert status.emergency_abort
    assert status.abort
    assert status.over_threshold_duration_sec < 0.15


def test_emergency_change_uses_absolute_pressure_delta():
    pressure = PressureDebouncer(emergency_threshold_pa=1000.0)
    pressure.set_baseline(100.0)
    for index in range(5):
        pressure.add(index * 0.01, 100.0)
    for index in range(3):
        status = pressure.add(0.05 + index * 0.01, -900.0)
    assert status.delta_pa == pytest.approx(-1000.0)
    assert status.emergency_abort


def test_default_100_pa_hold_ignores_observed_31_pa_drift():
    pressure = PressureDebouncer()
    pressure.set_baseline(0.0)
    for index in range(30):
        status = pressure.add(index * 0.01, 31.0)
    assert status.delta_pa == pytest.approx(31.0)
    assert not status.abort


def test_default_100_pa_hold_aborts_sustained_101_pa_change():
    pressure = PressureDebouncer()
    pressure.set_baseline(0.0)
    for index in range(30):
        status = pressure.add(index * 0.01, 101.0)
    assert status.abort
    assert not status.emergency_abort


def test_stale_input_recovers_without_permanent_error():
    gate = RecoverableFreshnessGate(1.5)
    gate.update(1.0)
    assert gate.result(2.6) == "WAIT"
    gate.update(2.61)
    assert gate.result(2.62) == "PASS"


def test_jsonl_has_unique_plan_ids_and_replayable_objects(tmp_path):
    writer = PlanDiagnostics(str(tmp_path), datetime(2026, 9, 13, 9, 42, 15))
    first = writer.new_plan(datetime(2026, 9, 13, 9, 42, 15))
    writer.write("state_change", state="TARGET_TRACKING")
    second = writer.new_plan(datetime(2026, 9, 13, 9, 42, 16))
    writer.write("planning_failure", reason="TARGET_MOVED")
    assert first == "PIPERH-20260913-094215000000-001"
    assert second == "PIPERH-20260913-094216000000-002"
    events = [json.loads(line) for line in writer.path.read_text().splitlines()]
    assert [event["event_type"] for event in events] == ["state_change", "planning_failure"]
    assert events[-1]["reason"] in REASON_CODES
