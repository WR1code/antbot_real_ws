import cv2
import numpy as np
import pytest
from apriltag_msgs.msg import AprilTagDetectionArray
from sensor_msgs.msg import CameraInfo, Image

import rebotarm_pulse.apriltag_board_pose as apriltag_module
import rebotarm_pulse.charuco_board_pose as charuco_module
from rebotarm_pulse.apriltag_board_pose import AprilTagBoardPoseNode
from rebotarm_pulse.charuco_board_pose import (
    CharucoBoardPoseNode,
    board_chessboard_corners,
    create_charuco_board,
    solve_charuco_pose,
)


class _Logger:
    def __init__(self):
        self.errors = []
        self.infos = []

    def error(self, message, **_kwargs):
        self.errors.append(message)

    def info(self, message, **_kwargs):
        self.infos.append(message)

    def warning(self, message, **_kwargs):
        self.errors.append(message)


def _camera_info(projection=None) -> CameraInfo:
    message = CameraInfo()
    message.width = 1280
    message.height = 720
    message.header.frame_id = "camera_color_optical_frame"
    message.k = [900.0, 0.0, 640.0, 0.0, 900.0, 360.0, 0.0, 0.0, 1.0]
    message.p = projection or [
        880.0, 0.0, 638.0, 0.0,
        0.0, 882.0, 358.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
    ]
    return message


def _camera_node(node_type):
    node = object.__new__(node_type)
    node.camera_matrix = None
    node.camera_info_width = None
    node.camera_info_height = None
    node.camera_info_frame_id = None
    node.camera_info_stamp = None
    node.camera_model_logged = False
    logger = _Logger()
    node.get_logger = lambda: logger
    return node, logger


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


@pytest.mark.parametrize("node_type", [CharucoBoardPoseNode, AprilTagBoardPoseNode])
def test_rectified_camera_info_uses_projection_matrix(node_type):
    node, logger = _camera_node(node_type)
    node._camera_info(_camera_info())

    np.testing.assert_allclose(
        node.camera_matrix,
        [[880.0, 0.0, 638.0], [0.0, 882.0, 358.0], [0.0, 0.0, 1.0]],
    )
    assert node.camera_info_width == 1280
    assert node.camera_info_height == 720
    assert node.camera_info_frame_id == "camera_color_optical_frame"
    assert "intrinsic source=CameraInfo.P" in logger.infos[0]


@pytest.mark.parametrize("node_type", [CharucoBoardPoseNode, AprilTagBoardPoseNode])
def test_zero_projection_does_not_fall_back_to_valid_k(node_type):
    node, logger = _camera_node(node_type)
    node._camera_info(_camera_info([0.0] * 12))

    assert node.camera_matrix is None
    assert node.camera_info_width is None
    assert "K fallback is disabled" in logger.errors[0]


@pytest.mark.parametrize("invalid_value", [float("nan"), float("inf")])
@pytest.mark.parametrize("node_type", [CharucoBoardPoseNode, AprilTagBoardPoseNode])
def test_nonfinite_projection_is_rejected(node_type, invalid_value):
    projection = list(_camera_info().p)
    projection[2] = invalid_value
    node, _ = _camera_node(node_type)
    node._camera_info(_camera_info(projection))
    assert node.camera_matrix is None


@pytest.mark.parametrize("node_type", [CharucoBoardPoseNode, AprilTagBoardPoseNode])
def test_invalid_camera_info_clears_previous_calibration(node_type):
    node, _ = _camera_node(node_type)
    node._camera_info(_camera_info())
    assert node.camera_matrix is not None

    node._camera_info(_camera_info([0.0] * 12))

    assert node.camera_matrix is None
    assert node.camera_info_width is None
    assert node.camera_info_height is None
    assert node.camera_info_frame_id is None
    assert node.camera_info_stamp is None


def test_mismatched_image_size_is_rejected_before_charuco_processing(monkeypatch):
    node, logger = _camera_node(CharucoBoardPoseNode)
    node._camera_info(_camera_info())
    image = Image()
    image.width = 640
    image.height = 480
    image.header.frame_id = "camera_color_optical_frame"
    monkeypatch.setattr(
        charuco_module,
        "image_to_gray",
        lambda _message: pytest.fail("mismatched image must not be decoded or passed to PnP"),
    )

    node._image(image)

    assert "does not match CameraInfo size 1280x720" in logger.errors[-1]


def test_mismatched_rectified_frame_id_is_rejected_before_charuco_processing(monkeypatch):
    node, logger = _camera_node(CharucoBoardPoseNode)
    node._camera_info(_camera_info())
    image = Image()
    image.width = 1280
    image.height = 720
    image.header.frame_id = "camera_depth_optical_frame"
    monkeypatch.setattr(
        charuco_module,
        "image_to_gray",
        lambda _message: pytest.fail("wrong-stream image must not be decoded or passed to PnP"),
    )

    node._image(image)

    assert "does not match CameraInfo frame_id" in logger.errors[-1]


def test_apriltag_board_rejects_mismatched_image_metadata_before_pnp(monkeypatch):
    node, logger = _camera_node(AprilTagBoardPoseNode)
    node._camera_info(_camera_info())
    node.latest_image_width = 640
    node.latest_image_height = 480
    node.latest_image_frame_id = "camera_color_optical_frame"
    monkeypatch.setattr(
        apriltag_module,
        "solve_board_pose",
        lambda *_args, **_kwargs: pytest.fail("mismatched image must not reach board PnP"),
    )

    node._detections(AprilTagDetectionArray())

    assert "does not match CameraInfo size 1280x720" in logger.errors[-1]
