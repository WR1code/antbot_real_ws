import math

import numpy as np

from antbot_dual_lidar.deskew_core import (
    PoseBuffer,
    deskew_points,
    point_times_ns,
    points_to_world,
    quaternion_slerp,
)


def test_slerp_midpoint_is_stable_and_shortest_path():
    identity = [0.0, 0.0, 0.0, 1.0]
    yaw_180 = [0.0, 0.0, 1.0, 0.0]
    midpoint = quaternion_slerp(identity, yaw_180, 0.5)
    assert np.allclose(np.abs(midpoint), [0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)])
    assert np.allclose(quaternion_slerp(identity, -np.asarray(identity), 0.4), identity)


def test_pose_buffer_interpolates_translation_and_clears_on_reset():
    buffer = PoseBuffer(10.0)
    buffer.add(1_000, [0, 0, 0], [0, 0, 0, 1])
    buffer.add(2_000, [2, 0, 0], [0, 0, 0, 1])
    pose = buffer.interpolate(1_500)
    assert np.allclose(pose.translation, [1, 0, 0])
    assert buffer.interpolate(999) is None
    buffer.add(100, [3, 0, 0], [0, 0, 0, 1])
    assert buffer.reset_count == 1
    assert buffer.bounds == (100, 100)


def test_all_point_time_hypotheses_are_explicit():
    offsets = np.asarray([0, 25_000_000, 100_000_000], dtype=np.uint32)
    assert point_times_ns(1_000_000_000, offsets, "header_plus_offset").tolist() == [
        1_000_000_000, 1_025_000_000, 1_100_000_000
    ]
    assert point_times_ns(1_000_000_000, offsets, "header_minus_offset").tolist() == [
        1_000_000_000, 975_000_000, 900_000_000
    ]
    assert point_times_ns(
        1_000_000_000, offsets, "header_plus_offset_minus_scan_period"
    ).tolist() == [900_000_000, 925_000_000, 1_000_000_000]
    assert point_times_ns(1_000_000_000, offsets, "header_midpoint").tolist() == [
        950_000_000, 975_000_000, 1_050_000_000
    ]


def test_deskew_translation_and_rotation():
    poses = PoseBuffer()
    poses.add(0, [0, 0, 0], [0, 0, 0, 1])
    poses.add(1_000_000_000, [1, 0, 0], [0, 0, 0, 1])
    corrected = deskew_points(
        [[2, 0, 0], [1, 0, 0]],
        [0, 1_000_000_000],
        1_000_000_000,
        poses,
        [0, 0, 0],
        [0, 0, 0, 1],
    )
    assert np.allclose(corrected, [[1, 0, 0], [1, 0, 0]])

    poses = PoseBuffer()
    poses.add(0, [0, 0, 0], [0, 0, 0, 1])
    poses.add(1_000_000_000, [0, 0, 0], [0, 0, 1, 0])
    corrected = deskew_points(
        [[1, 0, 0]],
        [0],
        1_000_000_000,
        poses,
        [0, 0, 0],
        [0, 0, 0, 1],
    )
    assert np.allclose(corrected, [[-1, 0, 0]], atol=1e-8)


def test_deskew_handles_empty_and_missing_pose():
    poses = PoseBuffer()
    assert deskew_points(
        np.empty((0, 3)), [], 0, poses, [0, 0, 0], [0, 0, 0, 1]
    ).shape == (0, 3)
    poses.add(10, [0, 0, 0], [0, 0, 0, 1])
    assert deskew_points(
        [[1, 0, 0]], [9], 10, poses, [0, 0, 0], [0, 0, 0, 1]
    ) is None


def test_vectorized_pose_interpolation_and_world_projection():
    poses = PoseBuffer()
    poses.add(0, [0, 0, 0], [0, 0, 0, 1])
    poses.add(100, [1, 0, 0], [0, 0, 0, 1])
    translations, quaternions = poses.interpolate_many([0, 50, 100])
    assert np.allclose(translations[:, 0], [0, 0.5, 1])
    assert np.allclose(quaternions, [[0, 0, 0, 1]] * 3)
    world = points_to_world(
        [[1, 0, 0]] * 3,
        [0, 50, 100],
        poses,
        [0, 0, 0],
        [0, 0, 0, 1],
    )
    assert np.allclose(world[:, 0], [1, 1.5, 2])
