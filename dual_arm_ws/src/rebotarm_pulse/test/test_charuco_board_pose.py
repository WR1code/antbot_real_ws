import cv2
import numpy as np

from rebotarm_pulse.charuco_board_pose import (
    board_chessboard_corners,
    create_charuco_board,
    solve_charuco_pose,
)


def test_bundled_charuco_geometry_is_5_by_7_at_physical_size():
    board = create_charuco_board()
    corners = board_chessboard_corners(board)
    assert corners.shape == (24, 3)
    np.testing.assert_allclose(corners[0], [0.035, 0.035, 0.0])
    np.testing.assert_allclose(corners[-1], [0.140, 0.210, 0.0])


def test_charuco_pose_recovers_synthetic_camera_transform():
    board = create_charuco_board()
    object_points = board_chessboard_corners(board)
    camera_matrix = np.asarray(
        [[680.0, 0.0, 320.0], [0.0, 675.0, 240.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    expected_rotation = np.asarray([0.16, -0.10, 0.04], dtype=np.float64)
    expected_translation = np.asarray([-0.07, -0.10, 0.58], dtype=np.float64)
    image_points, _ = cv2.projectPoints(
        object_points,
        expected_rotation,
        expected_translation,
        camera_matrix,
        np.zeros((5, 1), dtype=np.float64),
    )
    ids = np.arange(len(object_points), dtype=np.int32).reshape((-1, 1))
    pose = solve_charuco_pose(image_points, ids, camera_matrix, board)
    assert pose is not None
    assert pose.corner_count == 24
    assert pose.reprojection_rms_px < 1.0e-5
    np.testing.assert_allclose(pose.rotation_vector, expected_rotation, atol=1.0e-6)
    np.testing.assert_allclose(pose.translation_vector, expected_translation, atol=1.0e-7)


def test_charuco_pose_rejects_too_few_or_inconsistent_corners():
    board = create_charuco_board()
    matrix = np.asarray(
        [[680.0, 0.0, 320.0], [0.0, 680.0, 240.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    points, _ = cv2.projectPoints(
        board_chessboard_corners(board),
        np.asarray([0.1, -0.1, 0.05]),
        np.asarray([-0.07, -0.10, 0.55]),
        matrix,
        np.zeros((5, 1), dtype=np.float64),
    )
    ids = np.arange(len(points), dtype=np.int32).reshape((-1, 1))
    assert solve_charuco_pose(points[:5], ids[:5], matrix, board) is None
    corrupted = points.copy()
    corrupted[8] += np.asarray([[45.0, -35.0]])
    assert solve_charuco_pose(corrupted, ids, matrix, board) is None
