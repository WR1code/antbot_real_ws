"""Unit tests for operator input limiting without starting ROS nodes."""

import math

from geometry_msgs.msg import Twist

from antbot_h743_bridge.operator_manager import is_gamepad_name, limited_twist


def test_limited_twist_gives_clamped_point_turn_priority():
    message = Twist()
    message.linear.x = 3.0
    message.linear.y = 4.0
    message.angular.z = 2.0
    result = limited_twist(message, 0.10)
    assert result.linear.x == 0.0
    assert result.linear.y == 0.0
    assert result.angular.z == 1.0


def test_limited_twist_rejects_non_finite_input():
    message = Twist()
    message.linear.x = math.nan
    message.linear.y = 0.1
    result = limited_twist(message, 0.10)
    assert result.linear.x == 0.0
    assert result.linear.y == 0.0


def test_gamepad_name_filter_accepts_controller_and_rejects_touchscreen():
    assert is_gamepad_name("Generic X-Box pad")
    assert is_gamepad_name("8BitDo Ultimate 3mode Xbox")
    assert not is_gamepad_name("ILITEK ILITEK-TP Mouse")
    assert not is_gamepad_name("")
