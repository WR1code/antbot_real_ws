import math

from antbot_mapping.lio_odom_adapter import base_pose_from_imu_pose


def test_base_pose_removes_rotated_sensor_offset():
    half = math.sqrt(0.5)
    position, orientation = base_pose_from_imu_pose(
        (1.0, 2.0, 3.0), (0.0, 0.0, half, half), (0.3, 0.2, 0.1)
    )
    assert all(math.isclose(a, b) for a, b in zip(position, (1.2, 1.7, 2.9)))
    assert all(
        math.isclose(a, b)
        for a, b in zip(orientation, (0.0, 0.0, half, half))
    )
