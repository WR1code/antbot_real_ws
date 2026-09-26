import math

import pytest

from rebotarm_pulse.sensor_axis_alignment import (
    compute_sensor_axis_alignment,
    direction_is_fresh,
)


def _direction(degrees):
    radians = math.radians(degrees)
    return math.cos(radians), math.sin(radians), 0.0


@pytest.mark.parametrize(
    ("target_degrees", "expected_degrees"),
    [(0.0, 0.0), (30.0, 30.0), (-45.0, -45.0)],
)
def test_signed_alignment_angles(target_degrees, expected_degrees):
    result = compute_sensor_axis_alignment(
        (1.0, 0.0, 0.0), _direction(target_degrees), (0.0, 0.0, 1.0),
        sensor_array_axis_configured=True,
    )
    assert result.valid
    assert result.delta_deg == pytest.approx(expected_degrees)


def test_unoriented_arm_axis_selects_smaller_rotation():
    result = compute_sensor_axis_alignment(
        (1.0, 0.0, 0.0), _direction(170.0), (0.0, 0.0, 1.0),
        sensor_array_axis_configured=True,
    )
    assert result.valid
    assert result.delta_deg == pytest.approx(-10.0)
    assert result.target_arm_axis_base == pytest.approx(_direction(-10.0))


@pytest.mark.parametrize(
    ("sensor", "arm", "reason"),
    [
        ((1.0, 0.0, 0.0), (0.0, 0.0, 3.0), "arm direction"),
        ((0.0, 0.0, -2.0), (1.0, 0.0, 0.0), "sensor array"),
        ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), "sensor axis"),
        ((1.0, 0.0, 0.0), (math.nan, 0.0, 0.0), "finite"),
        ((1.0, 0.0, 0.0), (math.inf, 0.0, 0.0), "finite"),
    ],
)
def test_invalid_and_degenerate_vectors(sensor, arm, reason):
    result = compute_sensor_axis_alignment(
        sensor, arm, (0.0, 0.0, 1.0),
        sensor_array_axis_configured=True,
    )
    assert not result.valid
    assert reason in result.reason


def test_inputs_are_normalized_internally():
    result = compute_sensor_axis_alignment(
        (8.0, 0.0, 0.0), (0.0, -5.0, 0.0), (0.0, 0.0, 3.0),
        sensor_array_axis_configured=True,
    )
    assert result.valid
    assert result.delta_deg == pytest.approx(-90.0)
    assert result.current_sensor_axis_base == pytest.approx((1.0, 0.0, 0.0))


def test_unconfigured_installation_and_joint_limit_are_invalid():
    unconfigured = compute_sensor_axis_alignment(
        (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0),
        sensor_array_axis_configured=False,
    )
    assert not unconfigured.valid
    assert "configured=false" in unconfigured.reason
    limited = compute_sensor_axis_alignment(
        (1.0, 0.0, 0.0), _direction(30.0), (0.0, 0.0, 1.0),
        sensor_array_axis_configured=True,
        current_j6_rad=3.0,
        j6_limits_rad=(-3.14, 3.14),
    )
    assert not limited.valid
    assert "outside" in limited.reason


def test_direction_freshness_blocks_stale_or_invalid_time():
    assert direction_is_fresh(9.8, 10.0, 0.5)[0]
    stale = direction_is_fresh(9.0, 10.0, 0.5)
    assert not stale[0]
    assert "stale" in stale[1]
    assert not direction_is_fresh(math.nan, 10.0, 0.5)[0]
