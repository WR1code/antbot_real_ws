import cv2
import numpy as np

from rebotarm_pulse.apriltag_board_pose import (
    board_tag_centres,
    rotation_vector_to_quaternion,
    solve_board_pose,
    tag_object_corners,
)


def test_a4_board_layout_matches_printed_geometry():
    centres = board_tag_centres()
    assert centres == {
        0: (-0.05, 0.06),
        1: (0.05, 0.06),
        2: (-0.05, -0.06),
        3: (0.05, -0.06),
    }


def test_tag_corners_match_apriltag_ros_detection_order():
    np.testing.assert_allclose(
        tag_object_corners(0.10, 0.20, 0.040),
        np.asarray(
            [
                [0.12, 0.22, 0.0],
                [0.08, 0.22, 0.0],
                [0.08, 0.18, 0.0],
                [0.12, 0.18, 0.0],
            ]
        ),
    )


def test_joint_pose_recovers_synthetic_board_pose():
    camera_matrix = np.asarray(
        [[620.0, 0.0, 320.0], [0.0, 618.0, 240.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    expected_rotation = np.asarray([0.18, -0.11, 0.07], dtype=np.float64)
    expected_translation = np.asarray([0.025, -0.018, 0.52], dtype=np.float64)
    observations = {}
    for tag_id, centre in board_tag_centres().items():
        image_points, _ = cv2.projectPoints(
            tag_object_corners(*centre, 0.040),
            expected_rotation,
            expected_translation,
            camera_matrix,
            np.zeros((5, 1), dtype=np.float64),
        )
        observations[tag_id] = image_points.reshape((4, 2))

    pose = solve_board_pose(observations, camera_matrix)
    assert pose is not None
    np.testing.assert_allclose(pose.rotation_vector, expected_rotation, atol=1.0e-6)
    np.testing.assert_allclose(pose.translation_vector, expected_translation, atol=1.0e-7)
    assert pose.reprojection_rms_px < 1.0e-5
    assert pose.inlier_tag_ids == (0, 1, 2, 3)


def test_pose_requires_two_consistent_tags():
    camera_matrix = np.asarray(
        [[620.0, 0.0, 320.0], [0.0, 620.0, 240.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    one_tag = {0: np.asarray([[10, 10], [20, 10], [20, 20], [10, 20]])}
    assert solve_board_pose(one_tag, camera_matrix) is None


def test_joint_pose_recovers_when_planar_ransac_cannot_initialize(monkeypatch):
    camera_matrix = np.asarray(
        [[620.0, 0.0, 320.0], [0.0, 620.0, 240.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    rotation = np.asarray([0.12, -0.08, 0.05], dtype=np.float64)
    translation = np.asarray([0.01, -0.02, 0.50], dtype=np.float64)
    observations = {}
    for tag_id in (1, 3):
        centre = board_tag_centres()[tag_id]
        image_points, _ = cv2.projectPoints(
            tag_object_corners(*centre, 0.040),
            rotation,
            translation,
            camera_matrix,
            np.zeros((5, 1), dtype=np.float64),
        )
        observations[tag_id] = image_points.reshape((4, 2))

    monkeypatch.setattr(
        cv2,
        "solvePnPRansac",
        lambda *args, **kwargs: (False, None, None, None),
    )
    pose = solve_board_pose(observations, camera_matrix)
    assert pose is not None
    assert pose.inlier_tag_ids == (1, 3)
    np.testing.assert_allclose(pose.translation_vector, translation, atol=1.0e-6)


def test_four_tag_pose_recovers_when_planar_ransac_cannot_initialize(monkeypatch):
    camera_matrix = np.asarray(
        [[620.0, 0.0, 320.0], [0.0, 620.0, 240.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    rotation = np.asarray([-0.10, 0.16, 0.04], dtype=np.float64)
    translation = np.asarray([-0.02, 0.01, 0.55], dtype=np.float64)
    observations = {}
    for tag_id, centre in board_tag_centres().items():
        image_points, _ = cv2.projectPoints(
            tag_object_corners(*centre, 0.040),
            rotation,
            translation,
            camera_matrix,
            np.zeros((5, 1), dtype=np.float64),
        )
        observations[tag_id] = image_points.reshape((4, 2))

    monkeypatch.setattr(
        cv2,
        "solvePnPRansac",
        lambda *args, **kwargs: (False, None, None, None),
    )
    pose = solve_board_pose(observations, camera_matrix)
    assert pose is not None
    assert pose.inlier_tag_ids == (0, 1, 2, 3)
    np.testing.assert_allclose(pose.translation_vector, translation, atol=1.0e-6)


def test_direct_planar_fallback_rejects_inconsistent_tags(monkeypatch):
    camera_matrix = np.asarray(
        [[620.0, 0.0, 320.0], [0.0, 620.0, 240.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    rotation = np.asarray([0.08, -0.05, 0.03], dtype=np.float64)
    translation = np.asarray([0.01, 0.02, 0.50], dtype=np.float64)
    observations = {}
    for tag_id, centre in board_tag_centres().items():
        image_points, _ = cv2.projectPoints(
            tag_object_corners(*centre, 0.040),
            rotation,
            translation,
            camera_matrix,
            np.zeros((5, 1), dtype=np.float64),
        )
        observations[tag_id] = image_points.reshape((4, 2))
    observations[3] += np.asarray([60.0, -45.0])
    monkeypatch.setattr(
        cv2,
        "solvePnPRansac",
        lambda *args, **kwargs: (False, None, None, None),
    )
    assert solve_board_pose(observations, camera_matrix) is None


def test_joint_pose_rejects_one_corrupted_tag():
    camera_matrix = np.asarray(
        [[620.0, 0.0, 320.0], [0.0, 620.0, 240.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    rotation = np.asarray([0.12, 0.08, -0.05], dtype=np.float64)
    translation = np.asarray([-0.02, 0.01, 0.48], dtype=np.float64)
    observations = {}
    for tag_id, centre in board_tag_centres().items():
        image_points, _ = cv2.projectPoints(
            tag_object_corners(*centre, 0.040),
            rotation,
            translation,
            camera_matrix,
            np.zeros((5, 1), dtype=np.float64),
        )
        observations[tag_id] = image_points.reshape((4, 2))
    observations[3] += np.asarray([80.0, -60.0])

    pose = solve_board_pose(observations, camera_matrix)
    assert pose is not None
    assert pose.inlier_tag_ids == (0, 1, 2)
    np.testing.assert_allclose(pose.translation_vector, translation, atol=1.0e-6)


def test_rotation_quaternion_is_normalized():
    quaternion = rotation_vector_to_quaternion(np.asarray([0.3, -0.2, 0.1]))
    assert abs(float(np.linalg.norm(quaternion)) - 1.0) < 1.0e-12
