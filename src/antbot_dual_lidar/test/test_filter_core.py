import numpy as np
import pytest

from antbot_dual_lidar.filter_core import FilterConfig, filter_xyz, transform_xyz


def config(**changes):
    values = dict(
        min_range=0.15, max_range=20.0, min_height=0.02, max_height=2.0,
        voxel_leaf_size=0.0, body_filter_enabled=True,
        body_min_x=-0.48, body_max_x=0.48,
        body_min_y=-0.34, body_max_y=0.34,
        body_min_z=-0.10, body_max_z=0.50,
    )
    values.update(changes)
    return FilterConfig(**values)


def test_nan_inf_range_height_and_body_filter():
    points = np.array([
        [np.nan, 0.0, 1.0], [np.inf, 0.0, 1.0],
        [0.01, 0.0, 0.03], [21.0, 0.0, 1.0],
        [1.0, 0.0, -0.1], [1.0, 0.0, 2.1],
        [0.2, 0.0, 0.2], [1.0, 0.0, 1.0],
    ])
    result, stats = filter_xyz(points, config())
    assert result.tolist() == [[1.0, 0.0, 1.0]]
    assert stats == {
        "input": 8, "non_finite": 2, "range": 2,
        "height": 2, "body": 1, "voxel": 0, "output": 1,
    }


def test_empty_cloud_and_voxel_filter():
    empty, stats = filter_xyz(np.empty((0, 3)), config(voxel_leaf_size=0.05))
    assert empty.shape == (0, 3)
    assert stats["output"] == 0
    points = np.array([[1.001, 0.001, 1.0], [1.009, 0.009, 1.0], [1.2, 0.0, 1.0]])
    result, stats = filter_xyz(points, config(voxel_leaf_size=0.05))
    assert len(result) == 2
    assert stats["voxel"] == 1


def test_filter_indices_preserve_auxiliary_field_alignment():
    points = np.array([[1.0, 0.0, 1.0], [0.2, 0.0, 0.2], [2.0, 0.0, 1.0]])
    result, _, indices = filter_xyz(points, config(), return_indices=True)
    assert result.tolist() == [[1.0, 0.0, 1.0], [2.0, 0.0, 1.0]]
    assert indices.tolist() == [0, 2]


def test_complete_robot_envelope_is_removed_but_nearby_obstacle_is_kept():
    points = np.array([
        [0.435, 0.0, 0.30],    # front body detail
        [0.0, 0.305, 0.25],    # side body detail / turned wheel envelope
        [0.0, 0.0, 0.48],      # upper self return
        [0.51, 0.0, 0.30],     # obstacle just beyond the safety envelope
    ])
    result, stats = filter_xyz(points, config())
    np.testing.assert_allclose(result, [[0.51, 0.0, 0.30]])
    assert stats["body"] == 3


def test_full_quaternion_transform():
    half = np.sqrt(0.5)
    result = transform_xyz(
        np.array([[1.0, 0.0, 0.0]]),
        (1.0, 2.0, 3.0), (0.0, 0.0, half, half),
    )
    np.testing.assert_allclose(result, [[1.0, 3.0, 3.0]])


@pytest.mark.parametrize(
    "changes",
    [
        {"min_range": -1.0},
        {"min_range": 2.0, "max_range": 1.0},
        {"min_height": 2.0, "max_height": 1.0},
        {"voxel_leaf_size": -0.1},
        {"body_min_x": 1.0, "body_max_x": 0.0},
    ],
)
def test_invalid_parameters(changes):
    with pytest.raises(ValueError):
        config(**changes).validate()
