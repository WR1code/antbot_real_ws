"""Unit tests for operator input limiting without starting ROS nodes."""

import math

from geometry_msgs.msg import Twist

from antbot_h743_bridge.operator_manager import limited_twist


def test_limited_twist_clamps_planar_magnitude_and_drops_rotation():
    message = Twist()
    message.linear.x = 3.0
    message.linear.y = 4.0
    message.angular.z = 2.0
    result = limited_twist(message, 0.10)
    assert math.isclose(result.linear.x, 0.06)
    assert math.isclose(result.linear.y, 0.08)
    assert result.angular.z == 0.0


def test_limited_twist_rejects_non_finite_input():
    message = Twist()
    message.linear.x = math.nan
    message.linear.y = 0.1
    result = limited_twist(message, 0.10)
    assert result.linear.x == 0.0
    assert result.linear.y == 0.0
