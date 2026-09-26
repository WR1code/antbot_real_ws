"""Tests for the simulation-only startup trajectory."""

from types import SimpleNamespace

import pytest

from rebot_xbox_servo.arm_initializer import (
    controller_is_active,
    make_startup_trajectory,
)


def test_required_controller_must_explicitly_be_active():
    """Reject configured/inactive and unrelated active controllers."""
    controllers = [
        SimpleNamespace(name='rebotarm_controller', state='inactive'),
        SimpleNamespace(name='other_controller', state='active'),
    ]
    assert controller_is_active(controllers, 'rebotarm_controller') is False
    controllers[0].state = 'active'
    assert controller_is_active(controllers, 'rebotarm_controller') is True


def test_startup_trajectory_moves_six_zero_joints_slowly_to_safe_pose():
    names = [f'joint{index}' for index in range(1, 7)]
    target = [0.0, 1.75, 0.7, -0.7, 0.0, 0.0]
    message = make_startup_trajectory(names, [0.0] * 6, target, 6.0)
    assert message.joint_names == names
    assert list(message.points[0].positions) == [0.0] * 6
    assert list(message.points[0].velocities) == [0.0] * 6
    assert list(message.points[1].positions) == target
    assert list(message.points[1].velocities) == [0.0] * 6
    assert message.points[1].time_from_start.sec == 6


def test_startup_trajectory_rejects_bad_shape_or_fast_duration():
    with pytest.raises(ValueError):
        make_startup_trajectory(['joint1'], [], [0.0], 6.0)
    with pytest.raises(ValueError):
        make_startup_trajectory(['joint1'], [0.0], [1.0], 0.5)
