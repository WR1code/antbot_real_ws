"""Estimate one rigid board pose from the corners of several AprilTags."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import cv2
import numpy as np
import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from tf2_ros import TransformBroadcaster


DEFAULT_TAG_IDS = (0, 1, 2, 3)


@dataclass(frozen=True)
class BoardPose:
    rotation_vector: np.ndarray
    translation_vector: np.ndarray
    reprojection_rms_px: float
    inlier_tag_ids: tuple[int, ...]


def board_tag_centres(
    tag_ids: Iterable[int] = DEFAULT_TAG_IDS,
    spacing_x_m: float = 0.100,
    spacing_y_m: float = 0.120,
) -> dict[int, tuple[float, float]]:
    """Return the 2x2 A4 board centres, with +X right and +Y toward page top."""
    ids = tuple(int(tag_id) for tag_id in tag_ids)
    if len(ids) != 4 or len(set(ids)) != 4:
        raise ValueError("the A4 board requires four distinct tag IDs")
    half_x = float(spacing_x_m) / 2.0
    half_y = float(spacing_y_m) / 2.0
    return {
        ids[0]: (-half_x, half_y),
        ids[1]: (half_x, half_y),
        ids[2]: (-half_x, -half_y),
        ids[3]: (half_x, -half_y),
    }


def tag_object_corners(
    centre_x_m: float, centre_y_m: float, tag_size_m: float
) -> np.ndarray:
    """Object corners corresponding to apriltag detection corners p[0]..p[3]."""
    half = float(tag_size_m) / 2.0
    # apriltag_ros copies AprilTag's det->p array verbatim.  For the printed
    # board coordinate system (+X right, +Y up), that array corresponds to
    # top-right, top-left, bottom-left, bottom-right.  Keeping the former
    # left-to-right order mirrors every tag internally; RANSAC then accepts
    # only one column of the four-tag board and produces a biased board pose.
    return np.asarray(
        [
            [centre_x_m + half, centre_y_m + half, 0.0],
            [centre_x_m - half, centre_y_m + half, 0.0],
            [centre_x_m - half, centre_y_m - half, 0.0],
            [centre_x_m + half, centre_y_m - half, 0.0],
        ],
        dtype=np.float64,
    )


def solve_board_pose(
    observations: dict[int, np.ndarray],
    camera_matrix: np.ndarray,
    tag_size_m: float = 0.040,
    spacing_x_m: float = 0.100,
    spacing_y_m: float = 0.120,
    tag_ids: Iterable[int] = DEFAULT_TAG_IDS,
    minimum_visible_tags: int = 2,
    ransac_error_px: float = 3.0,
) -> BoardPose | None:
    """Solve camera_T_board from detected tag corners, rejecting weak solutions."""
    centres = board_tag_centres(tag_ids, spacing_x_m, spacing_y_m)
    object_chunks: list[np.ndarray] = []
    image_chunks: list[np.ndarray] = []
    point_tags: list[int] = []
    for tag_id, centre in centres.items():
        if tag_id not in observations:
            continue
        corners = np.asarray(observations[tag_id], dtype=np.float64).reshape((-1, 2))
        if corners.shape != (4, 2) or not np.all(np.isfinite(corners)):
            continue
        object_chunks.append(tag_object_corners(*centre, tag_size_m))
        image_chunks.append(corners)
        point_tags.extend([tag_id] * 4)

    if len(object_chunks) < int(minimum_visible_tags):
        return None
    object_points = np.concatenate(object_chunks, axis=0)
    image_points = np.concatenate(image_chunks, axis=0)
    matrix = np.asarray(camera_matrix, dtype=np.float64).reshape((3, 3))
    success, rvec, tvec, inliers = cv2.solvePnPRansac(
        object_points,
        image_points,
        matrix,
        np.zeros((5, 1), dtype=np.float64),
        iterationsCount=100,
        reprojectionError=float(ransac_error_px),
        confidence=0.99,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    indices = (
        np.asarray([], dtype=int)
        if not success or inliers is None
        else inliers.reshape(-1).astype(int)
    )
    inlier_tags = tuple(sorted({point_tags[index] for index in indices}))
    if len(inlier_tags) < int(minimum_visible_tags) or len(indices) < 6:
        # OpenCV's RANSAC can fail to initialize on an otherwise valid planar
        # observation, including when all four tags are visible.  Fall back to
        # a direct planar solve for every available tag.  This remains guarded
        # by the all-point reprojection threshold below, so inconsistent input
        # is rejected rather than published.  When RANSAC succeeds, it still
        # retains the ability to discard a corrupted tag.
        success, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            matrix,
            np.zeros((5, 1), dtype=np.float64),
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not success:
            return None
        indices = np.arange(len(object_points), dtype=int)
        inlier_tags = tuple(sorted({point_tags[index] for index in indices}))

    inlier_objects = object_points[indices]
    inlier_images = image_points[indices]
    if hasattr(cv2, "solvePnPRefineLM"):
        rvec, tvec = cv2.solvePnPRefineLM(
            inlier_objects,
            inlier_images,
            matrix,
            np.zeros((5, 1), dtype=np.float64),
            rvec,
            tvec,
        )
    projected, _ = cv2.projectPoints(
        inlier_objects,
        rvec,
        tvec,
        matrix,
        np.zeros((5, 1), dtype=np.float64),
    )
    residuals = projected.reshape((-1, 2)) - inlier_images
    rms = float(np.sqrt(np.mean(np.sum(residuals * residuals, axis=1))))
    if not np.isfinite(rms) or rms > float(ransac_error_px):
        return None
    if float(np.asarray(tvec).reshape(3)[2]) <= 0.0:
        return None
    return BoardPose(
        np.asarray(rvec, dtype=np.float64).reshape(3),
        np.asarray(tvec, dtype=np.float64).reshape(3),
        rms,
        inlier_tags,
    )


def rotation_vector_to_quaternion(rotation_vector: np.ndarray) -> tuple[float, float, float, float]:
    rotation, _ = cv2.Rodrigues(np.asarray(rotation_vector, dtype=np.float64).reshape(3))
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = 2.0 * np.sqrt(trace + 1.0)
        quaternion = (
            (rotation[2, 1] - rotation[1, 2]) / scale,
            (rotation[0, 2] - rotation[2, 0]) / scale,
            (rotation[1, 0] - rotation[0, 1]) / scale,
            0.25 * scale,
        )
    else:
        diagonal = np.diag(rotation)
        index = int(np.argmax(diagonal))
        following = (index + 1) % 3
        remaining = (index + 2) % 3
        scale = 2.0 * np.sqrt(
            max(0.0, 1.0 + rotation[index, index] - rotation[following, following] - rotation[remaining, remaining])
        )
        values = [0.0, 0.0, 0.0]
        values[index] = 0.25 * scale
        values[following] = (rotation[following, index] + rotation[index, following]) / scale
        values[remaining] = (rotation[remaining, index] + rotation[index, remaining]) / scale
        quaternion = (
            values[0],
            values[1],
            values[2],
            (rotation[remaining, following] - rotation[following, remaining]) / scale,
        )
    norm = float(np.linalg.norm(quaternion))
    return tuple(float(value / norm) for value in quaternion)


class AprilTagBoardPoseNode(Node):
    def __init__(self) -> None:
        super().__init__("apriltag_board_pose")
        self.declare_parameter("tag_ids", list(DEFAULT_TAG_IDS))
        self.declare_parameter("tag_size_m", 0.040)
        self.declare_parameter("center_spacing_x_m", 0.100)
        self.declare_parameter("center_spacing_y_m", 0.120)
        self.declare_parameter("minimum_visible_tags", 2)
        self.declare_parameter("max_reprojection_error_px", 3.0)
        self.declare_parameter("marker_frame", "marker_frame")

        self.tag_ids = tuple(int(value) for value in self.get_parameter("tag_ids").value)
        self.tag_size_m = float(self.get_parameter("tag_size_m").value)
        self.spacing_x_m = float(self.get_parameter("center_spacing_x_m").value)
        self.spacing_y_m = float(self.get_parameter("center_spacing_y_m").value)
        self.minimum_visible_tags = int(self.get_parameter("minimum_visible_tags").value)
        self.maximum_error_px = float(self.get_parameter("max_reprojection_error_px").value)
        self.marker_frame = str(self.get_parameter("marker_frame").value)
        board_tag_centres(self.tag_ids, self.spacing_x_m, self.spacing_y_m)
        if self.tag_size_m <= 0.0 or self.minimum_visible_tags < 2:
            raise ValueError("tag_size_m must be positive and minimum_visible_tags must be at least 2")

        self.camera_matrix: np.ndarray | None = None
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(CameraInfo, "camera_info", self._camera_info, qos_profile_sensor_data)
        self.create_subscription(
            AprilTagDetectionArray,
            "detections",
            self._detections,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            "A4 four-tag board enabled: IDs %s, tag %.1f mm, spacing %.1f x %.1f mm, minimum %d tags"
            % (
                list(self.tag_ids),
                self.tag_size_m * 1000.0,
                self.spacing_x_m * 1000.0,
                self.spacing_y_m * 1000.0,
                self.minimum_visible_tags,
            )
        )

    def _camera_info(self, message: CameraInfo) -> None:
        projection = np.asarray(message.p, dtype=np.float64).reshape((3, 4))
        matrix = projection[:, :3]
        if matrix[0, 0] <= 0.0 or matrix[1, 1] <= 0.0:
            matrix = np.asarray(message.k, dtype=np.float64).reshape((3, 3))
        if matrix[0, 0] > 0.0 and matrix[1, 1] > 0.0:
            self.camera_matrix = matrix.copy()

    def _detections(self, message: AprilTagDetectionArray) -> None:
        if self.camera_matrix is None:
            self.get_logger().warning("waiting for a valid rectified CameraInfo", throttle_duration_sec=5.0)
            return
        observations = {
            int(detection.id): np.asarray(
                [[corner.x, corner.y] for corner in detection.corners], dtype=np.float64
            )
            for detection in message.detections
            if int(detection.id) in self.tag_ids
        }
        pose = solve_board_pose(
            observations,
            self.camera_matrix,
            self.tag_size_m,
            self.spacing_x_m,
            self.spacing_y_m,
            self.tag_ids,
            self.minimum_visible_tags,
            self.maximum_error_px,
        )
        if pose is None:
            self.get_logger().warning(
                "board pose unavailable: need at least %d geometrically consistent tags; visible IDs=%s"
                % (self.minimum_visible_tags, sorted(observations)),
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
            "board pose: tags=%s, reprojection RMS=%.2f px"
            % (list(pose.inlier_tag_ids), pose.reprojection_rms_px),
            throttle_duration_sec=3.0,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AprilTagBoardPoseNode()
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
