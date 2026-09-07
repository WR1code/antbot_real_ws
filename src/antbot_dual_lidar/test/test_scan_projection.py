import math

import numpy as np
import pytest

from antbot_dual_lidar.scan_projection import project_xyz_to_laserscan


def test_projects_nearest_point_in_horizontal_slice():
    points = np.array([
        [2.0, 0.0, 0.0],
        [1.0, 0.0, 0.1],
        [0.5, 0.0, 0.8],
        [0.0, 3.0, -0.1],
    ])
    angle_min, _angle_max, ranges = project_xyz_to_laserscan(
        points, angle_increment=math.radians(1.0)
    )
    zero = int(math.floor((0.0 - angle_min) / math.radians(1.0)))
    ninety = int(math.floor((math.pi / 2 - angle_min) / math.radians(1.0)))
    assert ranges[zero] == pytest.approx(1.0)
    assert ranges[ninety] == pytest.approx(3.0)
    assert np.isfinite(ranges).sum() == 2


def test_keeps_vertical_information_outside_scan_without_changing_cloud():
    points = np.array([[1.0, 0.0, 0.5], [2.0, 0.0, -0.5]])
    _minimum, _maximum, ranges = project_xyz_to_laserscan(points)
    assert not np.isfinite(ranges).any()
    assert np.array_equal(points[:, 2], [0.5, -0.5])


@pytest.mark.parametrize(
    "translation,yaw,self_point,obstacle_point",
    [
        ((0.322, 0.222, 0.414), 0.0, (0.10, 0.0, -0.10), (0.30, 0.0, -0.10)),
        (
            (-0.322, -0.222, 0.414),
            math.pi,
            (0.10, 0.0, -0.10),
            (0.30, 0.0, -0.10),
        ),
    ],
)
def test_body_filter_uses_each_lidar_mount_pose(
    translation, yaw, self_point, obstacle_point
):
    points = np.array([self_point, obstacle_point])
    angle_min, _angle_max, ranges = project_xyz_to_laserscan(
        points,
        angle_increment=math.radians(1.0),
        sensor_translation=translation,
        sensor_yaw=yaw,
        body_bounds=(-0.48, 0.48, -0.34, 0.34, -0.10, 0.50),
    )
    zero = int(math.floor((0.0 - angle_min) / math.radians(1.0)))
    assert ranges[zero] == pytest.approx(0.30)


def test_rejects_invalid_projection_configuration():
    with pytest.raises(ValueError):
        project_xyz_to_laserscan(np.zeros((2, 2)))
    with pytest.raises(ValueError):
        project_xyz_to_laserscan(
            np.zeros((1, 3)), min_height=1.0, max_height=0.0
        )
    with pytest.raises(ValueError):
        project_xyz_to_laserscan(
            np.zeros((1, 3)), body_bounds=(1.0, 0.0, -1.0, 1.0, -1.0, 1.0)
        )
