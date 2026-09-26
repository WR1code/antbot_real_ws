"""Verify the simulation-only gripper command adapter."""

import pytest

from rebot_xbox_servo.sim_gripper import (
    integrate_gripper_positions,
    make_gripper_trajectory,
)


def test_builds_open_gripper_trajectory():
    command = make_gripper_trajectory(
        ['gripper_joint1', 'gripper_joint2'],
        [0.045, 0.045],
        1.25,
    )
    assert command.joint_names == ['gripper_joint1', 'gripper_joint2']
    assert list(command.points[0].positions) == [0.045, 0.045]
    assert command.points[0].time_from_start.sec == 1
    assert command.points[0].time_from_start.nanosec == 250_000_000


def test_rejects_unsafe_command_shapes():
    with pytest.raises(ValueError):
        make_gripper_trajectory(['one'], [0.0], 1.0)
    with pytest.raises(ValueError):
        make_gripper_trajectory(['one', 'two'], [0.0, 0.0], 0.0)


def test_trigger_amount_scales_velocity_and_stops_without_drift():
    half_speed = integrate_gripper_positions(
        [0.0, 0.0], 0.0225, 0.1, [0.0, 0.0], [0.045, 0.045]
    )
    full_speed = integrate_gripper_positions(
        [0.0, 0.0], 0.045, 0.1, [0.0, 0.0], [0.045, 0.045]
    )
    stopped = integrate_gripper_positions(
        half_speed, 0.0, 0.1, [0.0, 0.0], [0.045, 0.045]
    )
    assert half_speed == pytest.approx([0.00225, 0.00225])
    assert full_speed == pytest.approx([0.0045, 0.0045])
    assert stopped == pytest.approx(half_speed)


def test_velocity_integration_clamps_to_gripper_limits():
    assert integrate_gripper_positions(
        [0.044, 0.044], 0.045, 1.0, [0.0, 0.0], [0.045, 0.045]
    ) == [0.045, 0.045]
    assert integrate_gripper_positions(
        [0.001, 0.001], -0.045, 1.0, [0.0, 0.0], [0.045, 0.045]
    ) == [0.0, 0.0]
