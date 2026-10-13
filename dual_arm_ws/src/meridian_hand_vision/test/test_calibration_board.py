import cv2
import numpy as np

from meridian_hand_vision.calibration_board import (
    _lower_right_corner,
    _rectification_maps,
    oriented_board_normal,
    rectify_board_image,
)
from meridian_hand_vision.pulse_region import CameraIntrinsics


def test_board_normal_points_from_camera_toward_board() -> None:
    vector = oriented_board_normal(np.zeros(3), np.asarray([0.1, 0.0, 0.5]))
    np.testing.assert_allclose(vector, (0.0, 0.0, 1.0), atol=1.0e-9)


def test_board_normal_flips_opposite_board_coordinate_convention() -> None:
    rotation, _ = cv2.Rodrigues(np.asarray([np.pi, 0.0, 0.0]))
    rvec, _ = cv2.Rodrigues(rotation)
    vector = oriented_board_normal(rvec, np.asarray([0.0, 0.0, 0.5]))
    np.testing.assert_allclose(vector, (0.0, 0.0, 1.0), atol=1.0e-9)


def test_lower_right_corner_is_selected_in_image_space() -> None:
    corners = np.asarray(
        [[-0.1, -0.1, 0.0], [0.1, -0.1, 0.0],
         [0.1, 0.1, 0.0], [-0.1, 0.1, 0.0]],
        dtype=np.float64,
    )
    matrix = np.asarray(
        [[500.0, 0.0, 320.0], [0.0, 500.0, 240.0], [0.0, 0.0, 1.0]]
    )
    point, pixel = _lower_right_corner(
        corners, np.zeros(3), np.asarray([0.0, 0.0, 1.0]),
        matrix, np.zeros((5, 1)), (480, 640),
    )
    np.testing.assert_allclose(point, (0.1, 0.1, 1.0), atol=1.0e-9)
    assert pixel == (370, 290)


def test_runtime_board_input_is_rectified_with_projection_matrix() -> None:
    image = np.zeros((48, 64, 3), dtype=np.uint8)
    intrinsics = CameraIntrinsics(50.0, 51.0, 31.0, 23.0, 64, 48)
    projection = [
        48.0, 0.0, 30.0, 0.0,
        0.0, 49.0, 22.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
    ]
    result = rectify_board_image(
        image, intrinsics, np.zeros(5), projection, np.eye(3)
    )
    assert result is not None
    rectified, matrix = result
    assert rectified.shape == image.shape
    np.testing.assert_allclose(
        matrix,
        [[48.0, 0.0, 30.0], [0.0, 49.0, 22.0], [0.0, 0.0, 1.0]],
    )


def test_runtime_board_input_rejects_missing_projection_matrix() -> None:
    image = np.zeros((48, 64, 3), dtype=np.uint8)
    intrinsics = CameraIntrinsics(50.0, 51.0, 31.0, 23.0, 64, 48)
    assert rectify_board_image(image, intrinsics, np.zeros(5), ()) is None


def test_runtime_board_rectification_maps_are_cached() -> None:
    _rectification_maps.cache_clear()
    image = np.zeros((48, 64, 3), dtype=np.uint8)
    intrinsics = CameraIntrinsics(50.0, 51.0, 31.0, 23.0, 64, 48)
    projection = [
        48.0, 0.0, 30.0, 0.0,
        0.0, 49.0, 22.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
    ]
    for _ in range(2):
        assert rectify_board_image(
            image, intrinsics, np.zeros(5), projection, np.eye(3)
        ) is not None
    cache = _rectification_maps.cache_info()
    assert cache.misses == 1
    assert cache.hits == 1
