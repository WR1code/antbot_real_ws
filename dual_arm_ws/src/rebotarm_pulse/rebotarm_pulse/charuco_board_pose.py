"""Detect the bundled A4 ChArUco board and publish its pose as TF."""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster

from .apriltag_board_pose import rotation_vector_to_quaternion


CHARUCO_SQUARES_X = 5
CHARUCO_SQUARES_Y = 7
CHARUCO_SQUARE_LENGTH_M = 0.035
CHARUCO_MARKER_LENGTH_M = 0.026
CHARUCO_DICTIONARY_ID = cv2.aruco.DICT_4X4_50


@dataclass(frozen=True)
class CharucoPose:
    rotation_vector: np.ndarray
    translation_vector: np.ndarray
    reprojection_rms_px: float
    corner_count: int


def create_charuco_board(
    squares_x: int = CHARUCO_SQUARES_X,
    squares_y: int = CHARUCO_SQUARES_Y,
    square_length_m: float = CHARUCO_SQUARE_LENGTH_M,
    marker_length_m: float = CHARUCO_MARKER_LENGTH_M,
):
    """Create a board compatible with both OpenCV 4.6 and newer releases."""
    dictionary = cv2.aruco.getPredefinedDictionary(CHARUCO_DICTIONARY_ID)
    if hasattr(cv2.aruco, "CharucoBoard_create"):
        return cv2.aruco.CharucoBoard_create(
            int(squares_x),
            int(squares_y),
            float(square_length_m),
            float(marker_length_m),
            dictionary,
        )
    return cv2.aruco.CharucoBoard(
        (int(squares_x), int(squares_y)),
        float(square_length_m),
        float(marker_length_m),
        dictionary,
    )


def board_chessboard_corners(board) -> np.ndarray:
    corners = (
        board.getChessboardCorners()
        if hasattr(board, "getChessboardCorners")
        else board.chessboardCorners
    )
    return np.asarray(corners, dtype=np.float64).reshape((-1, 3))


def solve_charuco_pose(
    charuco_corners: np.ndarray,
    charuco_ids: np.ndarray,
    camera_matrix: np.ndarray,
    board,
    minimum_corners: int = 6,
    maximum_reprojection_error_px: float = 3.0,
) -> CharucoPose | None:
    """Solve camera_T_board and reject sparse or inaccurate observations."""
    image_points = np.asarray(charuco_corners, dtype=np.float64).reshape((-1, 2))
    corner_ids = np.asarray(charuco_ids, dtype=np.int32).reshape(-1)
    if len(corner_ids) < int(minimum_corners) or len(image_points) != len(corner_ids):
        return None
    object_corners = board_chessboard_corners(board)
    if np.any(corner_ids < 0) or np.any(corner_ids >= len(object_corners)):
        return None
    matrix = np.asarray(camera_matrix, dtype=np.float64).reshape((3, 3))
    object_points = object_corners[corner_ids]
    success, rotation, translation = cv2.solvePnP(
        object_points,
        image_points,
        matrix,
        np.zeros((5, 1), dtype=np.float64),
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not success or float(np.asarray(translation).reshape(3)[2]) <= 0.0:
        return None
    if hasattr(cv2, "solvePnPRefineLM"):
        rotation, translation = cv2.solvePnPRefineLM(
            object_points,
            image_points,
            matrix,
            np.zeros((5, 1), dtype=np.float64),
            rotation,
            translation,
        )
    projected, _ = cv2.projectPoints(
        object_points,
        rotation,
        translation,
        matrix,
        np.zeros((5, 1), dtype=np.float64),
    )
    residuals = projected.reshape((-1, 2)) - image_points
    rms = float(np.sqrt(np.mean(np.sum(residuals * residuals, axis=1))))
    if not np.isfinite(rms) or rms > float(maximum_reprojection_error_px):
        return None
    return CharucoPose(
        np.asarray(rotation, dtype=np.float64).reshape(3),
        np.asarray(translation, dtype=np.float64).reshape(3),
        rms,
        len(corner_ids),
    )


def image_to_gray(message: Image) -> np.ndarray:
    """Decode common 8-bit ROS image encodings without requiring cv_bridge."""
    encoding = message.encoding.lower()
    channels_by_encoding = {
        "mono8": 1,
        "8uc1": 1,
        "rgb8": 3,
        "bgr8": 3,
        "8uc3": 3,
        "rgba8": 4,
        "bgra8": 4,
        "8uc4": 4,
    }
    channels = channels_by_encoding.get(encoding)
    if channels is None:
        raise ValueError(f"unsupported image encoding '{message.encoding}'")
    row_width = int(message.width) * channels
    if int(message.step) < row_width:
        raise ValueError("image step is smaller than its encoded row width")
    raw = np.frombuffer(message.data, dtype=np.uint8)
    required = int(message.step) * int(message.height)
    if raw.size < required:
        raise ValueError("image data is shorter than height * step")
    rows = raw[:required].reshape((int(message.height), int(message.step)))
    pixels = rows[:, :row_width]
    if channels == 1:
        return pixels.reshape((int(message.height), int(message.width))).copy()
    image = pixels.reshape((int(message.height), int(message.width), channels))
    if encoding in {"rgb8"}:
        conversion = cv2.COLOR_RGB2GRAY
    elif encoding in {"rgba8"}:
        conversion = cv2.COLOR_RGBA2GRAY
    elif channels == 3:
        conversion = cv2.COLOR_BGR2GRAY
    else:
        conversion = cv2.COLOR_BGRA2GRAY
    return cv2.cvtColor(image, conversion)


class CharucoBoardPoseNode(Node):
    def __init__(self) -> None:
        super().__init__("charuco_board_pose")
        self.declare_parameter("squares_x", CHARUCO_SQUARES_X)
        self.declare_parameter("squares_y", CHARUCO_SQUARES_Y)
        self.declare_parameter("square_length_m", CHARUCO_SQUARE_LENGTH_M)
        self.declare_parameter("marker_length_m", CHARUCO_MARKER_LENGTH_M)
        self.declare_parameter("minimum_charuco_corners", 6)
        self.declare_parameter("max_reprojection_error_px", 3.0)
        self.declare_parameter("marker_frame", "marker_frame")

        self.squares_x = int(self.get_parameter("squares_x").value)
        self.squares_y = int(self.get_parameter("squares_y").value)
        self.square_length_m = float(self.get_parameter("square_length_m").value)
        self.marker_length_m = float(self.get_parameter("marker_length_m").value)
        self.minimum_corners = int(self.get_parameter("minimum_charuco_corners").value)
        self.maximum_error_px = float(self.get_parameter("max_reprojection_error_px").value)
        self.marker_frame = str(self.get_parameter("marker_frame").value)
        if (
            self.squares_x < 2
            or self.squares_y < 2
            or self.square_length_m <= 0.0
            or not 0.0 < self.marker_length_m < self.square_length_m
            or self.minimum_corners < 4
        ):
            raise ValueError("invalid ChArUco board geometry or minimum corner count")

        self.board = create_charuco_board(
            self.squares_x,
            self.squares_y,
            self.square_length_m,
            self.marker_length_m,
        )
        self.dictionary = cv2.aruco.getPredefinedDictionary(CHARUCO_DICTIONARY_ID)
        self.camera_matrix: np.ndarray | None = None
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            CameraInfo, "camera_info", self._camera_info, qos_profile_sensor_data
        )
        self.create_subscription(Image, "image", self._image, qos_profile_sensor_data)
        self.get_logger().info(
            "A4 ChArUco board enabled: %dx%d, square %.1f mm, marker %.1f mm, DICT_4X4_50"
            % (
                self.squares_x,
                self.squares_y,
                self.square_length_m * 1000.0,
                self.marker_length_m * 1000.0,
            )
        )

    def _camera_info(self, message: CameraInfo) -> None:
        projection = np.asarray(message.p, dtype=np.float64).reshape((3, 4))
        matrix = projection[:, :3]
        if matrix[0, 0] <= 0.0 or matrix[1, 1] <= 0.0:
            matrix = np.asarray(message.k, dtype=np.float64).reshape((3, 3))
        if matrix[0, 0] > 0.0 and matrix[1, 1] > 0.0:
            self.camera_matrix = matrix.copy()

    def _image(self, message: Image) -> None:
        if self.camera_matrix is None:
            self.get_logger().warning(
                "waiting for a valid rectified CameraInfo", throttle_duration_sec=5.0
            )
            return
        try:
            gray = image_to_gray(message)
        except ValueError as error:
            self.get_logger().error(str(error), throttle_duration_sec=5.0)
            return
        marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary)
        if marker_ids is None or len(marker_ids) < 2:
            self.get_logger().warning(
                "ChArUco board unavailable: fewer than two markers visible",
                throttle_duration_sec=2.0,
            )
            return
        count, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
            marker_corners,
            marker_ids,
            gray,
            self.board,
            cameraMatrix=self.camera_matrix,
            distCoeffs=np.zeros((5, 1), dtype=np.float64),
        )
        if charuco_ids is None or int(count) < self.minimum_corners:
            self.get_logger().warning(
                "ChArUco board unavailable: %d/%d required chessboard corners visible"
                % (int(count), self.minimum_corners),
                throttle_duration_sec=2.0,
            )
            return
        pose = solve_charuco_pose(
            charuco_corners,
            charuco_ids,
            self.camera_matrix,
            self.board,
            self.minimum_corners,
            self.maximum_error_px,
        )
        if pose is None:
            self.get_logger().warning(
                "ChArUco pose rejected by geometric consistency checks",
                throttle_duration_sec=2.0,
            )
            return
        quaternion = rotation_vector_to_quaternion(pose.rotation_vector)
        transform = TransformStamped()
        transform.header = message.header
        transform.child_frame_id = self.marker_frame
        transform.transform.translation.x = float(pose.translation_vector[0])
        transform.transform.translation.y = float(pose.translation_vector[1])
        transform.transform.translation.z = float(pose.translation_vector[2])
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]
        self.broadcaster.sendTransform(transform)
        self.get_logger().info(
            "ChArUco pose: %d corners, reprojection RMS=%.2f px"
            % (pose.corner_count, pose.reprojection_rms_px),
            throttle_duration_sec=3.0,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CharucoBoardPoseNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
