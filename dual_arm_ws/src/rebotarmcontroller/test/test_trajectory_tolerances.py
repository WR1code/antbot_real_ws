from types import SimpleNamespace

import numpy as np
import pytest

from rebotarmcontroller.ros_actions import (
    duration_seconds,
    path_tolerance_violation,
    resolved_position_tolerances,
    set_duration_seconds,
    tracking_speed_scale,
)


def test_duration_seconds_uses_nanoseconds():
    assert duration_seconds(SimpleNamespace(sec=2, nanosec=500_000_000)) == 2.5


def test_set_duration_seconds_handles_fraction_and_rounding_carry():
    duration = SimpleNamespace(sec=0, nanosec=0)
    set_duration_seconds(duration, 2.25)
    assert (duration.sec, duration.nanosec) == (2, 250_000_000)
    set_duration_seconds(duration, 1.9999999996)
    assert (duration.sec, duration.nanosec) == (2, 0)


def test_requested_tolerance_can_only_tighten_configured_limit():
    specifications = [
        SimpleNamespace(name="joint1", position=0.001),
        SimpleNamespace(name="joint2", position=0.5),
        SimpleNamespace(name="unknown", position=0.0001),
        SimpleNamespace(name="joint3", position=0.0),
    ]
    actual = resolved_position_tolerances(
        specifications,
        ["joint1", "joint2", "joint3"],
        0.005,
    )
    np.testing.assert_allclose(actual, [0.001, 0.005, 0.005])


def test_path_tolerance_violation_identifies_the_failed_joint():
    message = path_tolerance_violation(
        ["joint1", "joint6"],
        np.array([0.0, 0.0321548]),
        np.array([0.0, 0.0005722]),
        np.array([0.03, 0.03]),
    )

    assert message is not None
    assert "joint=joint6" in message
    assert "error=0.031583 rad" in message


def test_path_tolerance_violation_returns_none_inside_limits():
    assert path_tolerance_violation(
        ["joint1"],
        np.array([0.02]),
        np.array([0.0]),
        np.array([0.03]),
    ) is None


def test_tracking_clock_runs_at_full_speed_for_small_error():
    assert tracking_speed_scale(
        np.array([0.01]),
        np.array([0.0]),
        np.array([0.03]),
    ) == 1.0


def test_tracking_clock_stops_before_hard_path_limit():
    assert tracking_speed_scale(
        np.array([0.024]),
        np.array([0.0]),
        np.array([0.03]),
    ) == 0.0


def test_tracking_clock_slows_smoothly_between_thresholds():
    scale = tracking_speed_scale(
        np.array([0.01875]),
        np.array([0.0]),
        np.array([0.03]),
    )
    assert scale == pytest.approx(0.5)
