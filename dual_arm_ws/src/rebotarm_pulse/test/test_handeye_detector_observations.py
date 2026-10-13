"""Successful detector callbacks publish observations alongside unchanged TF."""

import json
from types import SimpleNamespace

from apriltag_msgs.msg import AprilTagDetectionArray, AprilTagDetection
import cv2
import numpy as np
from sensor_msgs.msg import Image

from rebotarm_pulse import apriltag_board_pose as april
from rebotarm_pulse import charuco_board_pose as charuco
from rebotarm_pulse.handeye_observation import DetectionIdentity


def attach_outputs(node):
    transforms, observations = [], []
    node.broadcaster = SimpleNamespace(sendTransform=transforms.append)
    node.observation_identity = DetectionIdentity()
    node.observation_publisher = SimpleNamespace(publish=observations.append)
    node.get_logger = lambda: SimpleNamespace(info=lambda *_args, **_kwargs: None)
    node.marker_frame = "marker_frame"
    node.camera_info_width = 640
    node.camera_info_height = 480
    node.camera_info_frame_id = "camera_color_optical_frame"
    node.camera_matrix = np.asarray([[600., 0, 320], [0, 600, 240], [0, 0, 1]])
    return transforms, observations


def test_charuco_publishes_image_identity_pose_and_corners_without_changing_tf(monkeypatch):
    node = object.__new__(charuco.CharucoBoardPoseNode)
    transforms, observations = attach_outputs(node)
    node.board = charuco.create_charuco_board()
    node.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    node.minimum_corners = 6
    node.maximum_error_px = 3.0
    rvec = np.asarray([0.1, -0.1, 0.05])
    tvec = np.asarray([-0.07, -0.10, 0.6])
    points, _ = cv2.projectPoints(charuco.board_chessboard_corners(node.board), rvec, tvec, node.camera_matrix, np.zeros(5))
    ids = np.arange(len(points), dtype=np.int32).reshape(-1, 1)
    monkeypatch.setattr(charuco, "image_to_gray", lambda _image: np.zeros((480, 640), np.uint8))
    monkeypatch.setattr(cv2.aruco, "detectMarkers", lambda *_args: ([None, None], np.array([[0], [1]]), None))
    monkeypatch.setattr(cv2.aruco, "interpolateCornersCharuco", lambda *_args, **_kwargs: (len(points), points, ids))
    image = Image()
    image.width, image.height = 640, 480
    image.header.frame_id = node.camera_info_frame_id
    image.header.stamp.sec = 1
    node._image(image)
    node._image(image)
    assert len(transforms) == 2 and len(observations) == 2
    first, second = [json.loads(item.data) for item in observations]
    assert first["detection_id"] != second["detection_id"]
    assert first["image_timestamp"] == 1000000000
    assert first["detection_sequence"] == 1
    assert (first["image_width"], first["image_height"]) == (640, 480)
    assert len(first["corners"]) == 24
    np.testing.assert_allclose(first["camera_to_board_pose"]["translation"], tvec, atol=1e-6)
    assert transforms[0].header == image.header
    assert transforms[0].child_frame_id == "marker_frame"


def test_apriltag_board_publishes_observation_from_detection_header():
    node = object.__new__(april.AprilTagBoardPoseNode)
    transforms, observations = attach_outputs(node)
    node.latest_image_width, node.latest_image_height = 640, 480
    node.latest_image_frame_id = node.camera_info_frame_id
    node.tag_ids = (0, 1, 2, 3)
    node.tag_size_m, node.spacing_x_m, node.spacing_y_m = 0.04, 0.1, 0.12
    node.minimum_visible_tags, node.maximum_error_px = 2, 3.0
    message = AprilTagDetectionArray()
    message.header.frame_id = node.camera_info_frame_id
    message.header.stamp.sec = 2
    tvec = np.asarray([0.01, -0.02, 0.5])
    for tag_id, centre in april.board_tag_centres().items():
        points, _ = cv2.projectPoints(april.tag_object_corners(*centre, 0.04), np.asarray([0.1, -0.1, 0.05]), tvec, node.camera_matrix, np.zeros(5))
        detection = AprilTagDetection()
        detection.id = tag_id
        for corner, value in zip(detection.corners, points.reshape(4, 2)):
            corner.x, corner.y = float(value[0]), float(value[1])
        message.detections.append(detection)
    node._detections(message)
    assert len(transforms) == 1 and len(observations) == 1
    saved = json.loads(observations[0].data)
    assert saved["image_timestamp"] == 2000000000
    assert (saved["image_width"], saved["image_height"]) == (640, 480)
    assert len(saved["corners"]) == 16
    assert saved["reprojection_error"] < 1e-5
    np.testing.assert_allclose(saved["camera_to_board_pose"]["translation"], tvec, atol=1e-6)
    assert transforms[0].header == message.header
