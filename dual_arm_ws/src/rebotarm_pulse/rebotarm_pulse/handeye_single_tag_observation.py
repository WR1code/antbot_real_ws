"""Adapt real single-tag detection updates to the hand-eye observation stream."""

import json

from apriltag_msgs.msg import AprilTagDetectionArray
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String

from .apriltag_board_pose import rectified_camera_info, tag_object_corners
from .handeye_observation import DetectionIdentity, OBSERVATION_TOPIC, detection_observation


class SingleTagObservationNode(Node):
    """Publish observations only; apriltag_ros still owns the existing TF."""

    def __init__(self):
        super().__init__("handeye_single_tag_observation")
        self.declare_parameter("tag_id", 0)
        self.declare_parameter("tag_size_m", 0.040)
        self.declare_parameter("marker_frame", "marker_frame")
        self.identity = DetectionIdentity()
        self.camera = None
        self.publisher = self.create_publisher(String, OBSERVATION_TOPIC, 10)
        self.create_subscription(CameraInfo, "camera_info", self._camera_info, qos_profile_sensor_data)
        self.create_subscription(
            AprilTagDetectionArray, "detections", self._detections, qos_profile_sensor_data,
        )

    def _camera_info(self, message):
        self.camera = rectified_camera_info(message)

    def _detections(self, message):
        if self.camera is None or message.header.frame_id != self.camera[3]:
            return
        tag_id = int(self.get_parameter("tag_id").value)
        detection = next((item for item in message.detections if int(item.id) == tag_id), None)
        if detection is None:
            return
        corners = np.asarray([[item.x, item.y] for item in detection.corners], dtype=np.float64)
        if corners.shape != (4, 2) or not np.all(np.isfinite(corners)):
            return
        objects = tag_object_corners(0.0, 0.0, float(self.get_parameter("tag_size_m").value))
        success, rvec, tvec = cv2.solvePnP(
            objects, corners, self.camera[0], np.zeros(5), flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not success or tvec.reshape(3)[2] <= 0.0:
            return
        projected, _ = cv2.projectPoints(objects, rvec, tvec, self.camera[0], np.zeros(5))
        rms = float(np.sqrt(np.mean(np.sum((projected.reshape(4, 2) - corners) ** 2, axis=1))))
        observation = detection_observation(
            self.identity, message.header, str(self.get_parameter("marker_frame").value),
            rvec, tvec, rms, corners, image_size=(self.camera[1], self.camera[2]),
        )
        self.publisher.publish(String(data=json.dumps(observation.to_dict())))


def main(args=None):
    rclpy.init(args=args)
    node = SingleTagObservationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
