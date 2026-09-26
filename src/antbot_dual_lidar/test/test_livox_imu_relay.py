import math

from sensor_msgs.msg import Imu

from antbot_dual_lidar.livox_imu_relay import (
    STANDARD_GRAVITY,
    normalize_livox_imu,
)


def test_normalize_livox_imu_assigns_frame_and_si_units():
    message = Imu()
    message.header.frame_id = "livox_frame"
    message.linear_acceleration.x = 0.5
    message.linear_acceleration.z = 1.0
    message.angular_velocity.z = 0.25
    result = normalize_livox_imu(message, "mid360_front_imu")
    assert result.header.frame_id == "mid360_front_imu"
    assert math.isclose(result.linear_acceleration.x, 0.5 * STANDARD_GRAVITY)
    assert math.isclose(result.linear_acceleration.z, STANDARD_GRAVITY)
    assert result.angular_velocity.z == 0.25
    assert result.orientation_covariance[0] == -1.0
