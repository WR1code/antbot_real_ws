"""Tests for the measured Xbox-to-ANTBot command mapping."""

import os
from copy import deepcopy

from antbot_teleop.mapping_xbox import (
    CONTROL_ARM,
    CONTROL_BASE,
    MappingXbox,
    apply_deadzone,
    planar_command,
    trigger_amount,
)

import pytest

import rclpy
from rclpy.duration import Duration

from sensor_msgs.msg import Joy


class RecordingPublisher:
    """Minimal publisher double that records snapshots."""

    def __init__(self):
        """Create an empty message list."""
        self.messages = []

    def publish(self, message):
        """Record an immutable snapshot of one publication."""
        self.messages.append(deepcopy(message))


def joy(left_x=0.0, left_y=0.0, pressed=()):
    """Construct a neutral measured Generic Xbox report."""
    message = Joy()
    message.axes = [left_x, left_y, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
    message.buttons = [0] * 11
    for index in pressed:
        message.buttons[index] = 1
    return message


def test_left_stick_maps_to_holonomic_body_motion():
    """Map up/down and lateral stick motion into ROS body axes."""
    assert planar_command(
        0.0, -1.0, 0.0, 0.0, 0.3, 0.5
    ) == pytest.approx((0.3, 0.0, 0.0))
    assert planar_command(
        1.0, 0.0, 0.0, 0.0, 0.3, 0.5
    ) == pytest.approx((0.0, -0.3, 0.0))
    assert planar_command(
        -1.0, 0.0, 0.0, 0.0, 0.3, 0.5
    ) == pytest.approx((0.0, 0.3, 0.0))


def test_triggers_rotate_in_requested_directions():
    """Map LT to CCW and RT to CW with cancellation when both are held."""
    assert trigger_amount(1.0, 0.05) == 0.0
    assert trigger_amount(-1.0, 0.05) == 1.0
    assert planar_command(
        0.0, 0.0, 1.0, 0.0, 0.3, 0.5
    )[2] == pytest.approx(0.5)
    assert planar_command(
        0.0, 0.0, 0.0, 1.0, 0.3, 0.5
    )[2] == pytest.approx(-0.5)
    assert planar_command(
        0.0, 0.0, 1.0, 1.0, 0.3, 0.5
    )[2] == pytest.approx(0.0)


def test_deadzone_makes_stick_release_an_immediate_zero():
    """Treat centered stick values as an immediate stop."""
    assert apply_deadzone(0.09, 0.10) == 0.0
    assert apply_deadzone(-0.10, 0.10) == 0.0
    assert apply_deadzone(0.11, 0.10) == pytest.approx(0.11)


def test_node_safety_stop_speed_and_control_switch():
    """Exercise guarded motion, stop, speed tiers and shared ownership."""
    os.environ['ROS_LOG_DIR'] = '/tmp/antbot_mapping_xbox_test_logs'
    os.makedirs(os.environ['ROS_LOG_DIR'], exist_ok=True)
    rclpy.init()
    node = MappingXbox()
    cmd_vel = RecordingPublisher()
    armed = RecordingPublisher()
    target = RecordingPublisher()
    node.cmd_vel_publisher = cmd_vel
    node.armed_publisher = armed
    node.target_publisher = target
    try:
        node._joy_callback(joy())
        node._joy_callback(joy(pressed=[0]))
        assert node.armed is True

        node._joy_callback(joy())
        node._joy_callback(joy(left_y=-1.0))
        assert node.last_command.linear.x == pytest.approx(0.15)
        node._joy_callback(joy())
        assert node.last_command.linear.x == 0.0

        node._joy_callback(joy(pressed=[10]))
        assert node.speed_index == 1
        node._joy_callback(joy())

        node._joy_callback(joy(pressed=[8]))
        assert node.control_target == CONTROL_ARM
        assert node.armed is False
        assert target.messages[-1].data == CONTROL_ARM
        assert cmd_vel.messages[-1].linear.x == 0.0

        node._joy_callback(joy())
        node._joy_callback(joy(pressed=[0]))
        assert node.armed is False
        node._joy_callback(joy())

        node._joy_callback(joy(pressed=[8]))
        assert node.control_target == CONTROL_BASE
        assert node.armed is False
        node._joy_callback(joy())
        node._joy_callback(joy(pressed=[0]))
        assert node.armed is True

        node.last_joy_time = (
            node.get_clock().now() - Duration(seconds=1.0))
        node._timer_callback()
        assert node.armed is False
        assert node.joystick_timed_out is True
        assert armed.messages[-1].data is False
    finally:
        node.destroy_node()
        rclpy.shutdown()
