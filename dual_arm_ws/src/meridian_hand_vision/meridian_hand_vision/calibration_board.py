"""Estimate the normal of either calibration board used by this workspace."""

from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
import math

import cv2
import numpy as np

from rebotarm_pulse.charuco_board_pose import (
    board_chessboard_corners,
    create_charuco_board,
    solve_charuco_pose,
)

from .pulse_region import CameraIntrinsics, SurfaceNormal


@dataclass(frozen=True)
class CalibrationBoardNormal:
    normal: SurfaceNormal
    board_type: str
    reprojection_rms_px: float
    lower_right_corner_m: tuple[float, float, float]
    lower_right_pixel: tuple[int, int]


def _camera_matrix(intrinsics: CameraIntrinsics) -> np.ndarray:
    return np.asarray(
        [
            [intrinsics.fx, 0.0, intrinsics.cx],
            [0.0, intrinsics.fy, intrinsics.cy],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def oriented_board_normal(
    rotation_vector: np.ndarray,
    translation_vector: np.ndarray,
) -> tuple[float, float, float] | None:
    """Return board +Z, flipped when necessary to point camera -> board.

    PnP board conventions differ between the ChArUco and AprilTag layouts.
    Orienting the result by its dot product with the board centre removes that
    convention ambiguity and gives the same inward direction expected by the
    Link6 approach code.
    """
    rotation, _ = cv2.Rodrigues(
        np.asarray(rotation_vector, dtype=np.float64).reshape(3, 1)
    )
    normal = rotation[:, 2]
    centre = np.asarray(translation_vector, dtype=np.float64).reshape(3)
    length = float(np.linalg.norm(normal))
    if not math.isfinite(length) or length <= 1.0e-9 or not np.all(np.isfinite(centre)):
        return None
    normal = normal / length
    if float(np.dot(normal, centre)) < 0.0:
        normal = -normal
    return tuple(float(value) for value in normal)


def _charuco_board():
    return create_charuco_board()


def _charuco_object_corners(board) -> np.ndarray:
    return board_chessboard_corners(board)


@lru_cache(maxsize=4)
def _rectification_maps(
    width: int,
    height: int,
    matrix_values: tuple[float, ...],
    distortion_values: tuple[float, ...],
    rotation_values: tuple[float, ...],
    projection_values: tuple[float, ...],
) -> tuple[np.ndarray, np.ndarray]:
    return cv2.initUndistortRectifyMap(
        np.asarray(matrix_values, dtype=np.float64).reshape((3, 3)),
        np.asarray(distortion_values, dtype=np.float64).reshape((-1, 1)),
        np.asarray(rotation_values, dtype=np.float64).reshape((3, 3)),
        np.asarray(projection_values, dtype=np.float64).reshape((3, 3)),
        (width, height),
        cv2.CV_32FC1,
    )


def rectify_board_image(
    image: np.ndarray,
    intrinsics: CameraIntrinsics,
    distortion_coefficients,
    projection_matrix,
    rectification_matrix=(),
) -> tuple[np.ndarray, np.ndarray] | None:
    """Match image_proc's hand-eye input: rectified image, P, zero distortion."""
    projection = np.asarray(projection_matrix, dtype=np.float64)
    if projection.size != 12 or not np.all(np.isfinite(projection)):
        return None
    projection = projection.reshape((3, 4))[:, :3]
    if projection[0, 0] <= 0.0 or projection[1, 1] <= 0.0:
        return None
    rotation = np.asarray(rectification_matrix, dtype=np.float64)
    if rotation.size == 0:
        rotation = np.eye(3, dtype=np.float64)
    elif rotation.size == 9 and np.all(np.isfinite(rotation)):
        rotation = rotation.reshape((3, 3))
    else:
        return None
    distortion = np.asarray(distortion_coefficients, dtype=np.float64).reshape(-1)
    if distortion.size not in (4, 5, 8, 12, 14) or not np.all(np.isfinite(distortion)):
        return None
    matrix = _camera_matrix(intrinsics)
    height, width = image.shape[:2]
    map_x, map_y = _rectification_maps(
        width,
        height,
        tuple(float(value) for value in matrix.reshape(-1)),
        tuple(float(value) for value in distortion),
        tuple(float(value) for value in rotation.reshape(-1)),
        tuple(float(value) for value in projection.reshape(-1)),
    )
    rectified = cv2.remap(
        image, map_x, map_y, interpolation=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
    )
    return rectified, projection


def _solve_and_score(
    object_points: np.ndarray,
    image_points: np.ndarray,
    matrix: np.ndarray,
    distortion: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, float] | None:
    object_points = np.asarray(object_points, dtype=np.float64).reshape((-1, 3))
    image_points = np.asarray(image_points, dtype=np.float64).reshape((-1, 2))
    if len(object_points) < 6 or len(object_points) != len(image_points):
        return None
    success, rotation, translation = cv2.solvePnP(
        object_points,
        image_points,
        matrix,
        distortion,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not success or float(np.asarray(translation).reshape(3)[2]) <= 0.0:
        return None
    if hasattr(cv2, "solvePnPRefineLM"):
        rotation, translation = cv2.solvePnPRefineLM(
            object_points,
            image_points,
            matrix,
            distortion,
            rotation,
            translation,
        )
    projected, _ = cv2.projectPoints(
        object_points,
        rotation,
        translation,
        matrix,
        distortion,
    )
    residuals = projected.reshape((-1, 2)) - image_points
    rms = float(np.sqrt(np.mean(np.sum(residuals * residuals, axis=1))))
    if not math.isfinite(rms) or rms > 3.0:
        return None
    return (
        np.asarray(rotation, dtype=np.float64).reshape(3),
        np.asarray(translation, dtype=np.float64).reshape(3),
        rms,
    )


def _lower_right_corner(
    object_corners: np.ndarray,
    rotation: np.ndarray,
    translation: np.ndarray,
    matrix: np.ndarray,
    distortion: np.ndarray,
    image_shape: tuple[int, int],
) -> tuple[tuple[float, float, float], tuple[int, int]]:
    """Select the physical board corner nearest the image's lower-right corner."""
    object_corners = np.asarray(object_corners, dtype=np.float64).reshape((-1, 3))
    pixels, _ = cv2.projectPoints(
        object_corners, rotation, translation, matrix, distortion
    )
    pixels = pixels.reshape((-1, 2))
    height, width = image_shape
    # Normalize both axes so portrait/landscape resolution does not bias the
    # result.  This is an explicit image-space definition, independent of the
    # board coordinate convention or how the sheet was rotated on the table.
    scores = pixels[:, 0] / max(width, 1) + pixels[:, 1] / max(height, 1)
    index = int(np.argmax(scores))
    rotation_matrix, _ = cv2.Rodrigues(rotation.reshape(3, 1))
    camera_point = rotation_matrix @ object_corners[index] + translation.reshape(3)
    return (
        tuple(float(value) for value in camera_point),
        tuple(int(round(value)) for value in pixels[index]),
    )


def _estimate_charuco(
    gray: np.ndarray,
    matrix: np.ndarray,
    distortion: np.ndarray,
) -> CalibrationBoardNormal | None:
    board = _charuco_board()
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(gray, dictionary)
    if marker_ids is None or len(marker_ids) < 2:
        return None
    count, corners, ids = cv2.aruco.interpolateCornersCharuco(
        marker_corners,
        marker_ids,
        gray,
        board,
        cameraMatrix=matrix,
        distCoeffs=distortion,
    )
    if ids is None or int(count) < 6:
        return None
    ids_flat = np.asarray(ids, dtype=np.int32).reshape(-1)
    pose = solve_charuco_pose(corners, ids, matrix, board)
    if pose is None:
        return None
    rotation = pose.rotation_vector
    translation = pose.translation_vector
    rms = pose.reprojection_rms_px
    vector = oriented_board_normal(rotation, translation)
    if vector is None:
        return None
    corner_m, corner_pixel = _lower_right_corner(
        np.asarray(
            [[0.0, 0.0, 0.0], [0.175, 0.0, 0.0],
             [0.175, 0.245, 0.0], [0.0, 0.245, 0.0]],
            dtype=np.float64,
        ),
        rotation, translation, matrix, distortion, gray.shape[:2],
    )
    return CalibrationBoardNormal(
        SurfaceNormal(vector=vector, rms_m=0.0, point_count=len(ids_flat)),
        "charuco_a4_5x7",
        rms,
        corner_m,
        corner_pixel,
    )


def _april_object_corners(tag_id: int) -> np.ndarray:
    # Printed board layout: 0/1 on top, 2/3 on bottom.  OpenCV returns each
    # marker's corners clockwise from top-left.
    centres = {
        0: (-0.050, -0.060),
        1: (0.050, -0.060),
        2: (-0.050, 0.060),
        3: (0.050, 0.060),
    }
    cx, cy = centres[tag_id]
    half = 0.020
    return np.asarray(
        [
            [cx - half, cy - half, 0.0],
            [cx + half, cy - half, 0.0],
            [cx + half, cy + half, 0.0],
            [cx - half, cy + half, 0.0],
        ],
        dtype=np.float64,
    )


def _estimate_apriltag_board(
    gray: np.ndarray,
    matrix: np.ndarray,
    distortion: np.ndarray,
) -> CalibrationBoardNormal | None:
    if not hasattr(cv2.aruco, "DICT_APRILTAG_36h11"):
        return None
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary)
    if ids is None:
        return None
    observations = {
        int(tag_id): np.asarray(marker, dtype=np.float64).reshape((4, 2))
        for marker, tag_id in zip(corners, np.asarray(ids).reshape(-1))
        if int(tag_id) in (0, 1, 2, 3)
    }
    if len(observations) < 2:
        return None
    object_points = np.concatenate(
        [_april_object_corners(tag_id) for tag_id in sorted(observations)], axis=0
    )
    image_points = np.concatenate(
        [observations[tag_id] for tag_id in sorted(observations)], axis=0
    )
    solved = _solve_and_score(object_points, image_points, matrix, distortion)
    if solved is None:
        return None
    rotation, translation, rms = solved
    vector = oriented_board_normal(rotation, translation)
    if vector is None:
        return None
    corner_m, corner_pixel = _lower_right_corner(
        np.asarray(
            [[-0.070, -0.080, 0.0], [0.070, -0.080, 0.0],
             [0.070, 0.080, 0.0], [-0.070, 0.080, 0.0]],
            dtype=np.float64,
        ),
        rotation, translation, matrix, distortion, gray.shape[:2],
    )
    return CalibrationBoardNormal(
        SurfaceNormal(vector=vector, rms_m=0.0, point_count=len(image_points)),
        "a4_4tag_board",
        rms,
        corner_m,
        corner_pixel,
    )


def estimate_calibration_board_normal(
    bgr_image: np.ndarray,
    intrinsics: CameraIntrinsics,
    mirrored: bool = False,
    distortion_coefficients=(),
    projection_matrix=(),
    rectification_matrix=(),
) -> CalibrationBoardNormal | None:
    """Detect a board through the same rectified PnP model as hand-eye."""
    image = cv2.flip(bgr_image, 1) if mirrored else bgr_image
    rectified_input = rectify_board_image(
        image,
        intrinsics,
        distortion_coefficients,
        projection_matrix,
        rectification_matrix,
    )
    if rectified_input is None:
        return None
    image, matrix = rectified_input
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    distortion = np.zeros((5, 1), dtype=np.float64)
    return _estimate_charuco(gray, matrix, distortion) or _estimate_apriltag_board(
        gray, matrix, distortion
    )
