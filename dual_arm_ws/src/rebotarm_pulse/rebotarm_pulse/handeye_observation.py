"""Detection identity, frame admission, and robot-time matching for hand-eye."""

from dataclasses import asdict, dataclass, replace
import json
import math
import time
import uuid

import cv2
import numpy as np

from .adaptive_handeye import Transform, rotation_log


OBSERVATION_TOPIC = "/rebotarm_handeye/observations"


@dataclass(frozen=True)
class Observation:
    detection_id: str
    image_timestamp: int
    frame_id: str
    camera_to_board_pose: dict
    reprojection_error: float
    board_area: float
    corners: list
    receive_time: int = 0
    robot_timestamp: int | None = None
    joint_state: dict | None = None
    robot_pose: dict | None = None
    robot_visual_time_offset: float | None = None
    marker_frame: str = "marker_frame"
    detection_sequence: int = 0
    image_width: int = 0
    image_height: int = 0

    def to_dict(self):
        return asdict(self)

    @classmethod
    def from_json(cls, payload, receive_time):
        item = cls(**json.loads(payload))
        if not item.detection_id or item.image_timestamp <= 0 or not item.frame_id:
            raise ValueError("invalid observation identity or timestamp")
        pose = item.camera_to_board_pose
        Transform(pose["rotation"], pose["translation"])
        corners = np.asarray(item.corners, dtype=np.float64)
        if corners.ndim != 2 or corners.shape[1] != 2 or not np.all(np.isfinite(corners)):
            raise ValueError("invalid observation corners")
        if not math.isfinite(item.reprojection_error) or not math.isfinite(item.board_area):
            raise ValueError("non-finite observation measurements")
        return replace(item, receive_time=int(receive_time))


class DetectionIdentity:
    """The UUID separates detector lifetimes; sequence counts real callbacks."""

    def __init__(self):
        self.session = uuid.uuid4().hex
        self.sequence = 0

    def next(self):
        self.sequence += 1
        return f"{self.session}:{self.sequence}"


def detection_observation(identity, header, marker_frame, rvec, tvec, rms, corners,
                          image_size=(0, 0)):
    rotation, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3))
    points = np.asarray(corners, dtype=np.float64).reshape((-1, 2))
    area = float(cv2.contourArea(cv2.convexHull(points.astype(np.float32))))
    return Observation(
        identity.next(), int(header.stamp.sec) * 1000000000 + int(header.stamp.nanosec),
        header.frame_id,
        {"rotation": rotation.tolist(), "translation": np.asarray(tvec).reshape(3).tolist()},
        float(rms), area, points.tolist(), marker_frame=marker_frame,
        detection_sequence=identity.sequence,
        image_width=int(image_size[0]), image_height=int(image_size[1]),
    )


class FrameAdmission:
    """Run-wide identity and timestamp guards, independent of ROS and TF."""

    def __init__(self, interval=0.15, max_age=0.25):
        self.interval_ns = int(interval * 1e9)
        self.max_age_ns = int(max_age * 1e9)
        self.seen = set()
        self.latest_timestamp = 0
        self.accepted_timestamp = 0

    def reason(self, observation, now_ns, window_start_ns):
        if observation.detection_id in self.seen:
            return "duplicate_detection_id"
        self.seen.add(observation.detection_id)
        stamp = observation.image_timestamp
        if stamp <= self.latest_timestamp:
            return "non_increasing_timestamp"
        self.latest_timestamp = stamp
        if stamp < window_start_ns:
            return "before_capture_window"
        age = now_ns - stamp
        if age < 0:
            return "future_timestamp"
        if age > self.max_age_ns:
            return "expired_frame"
        if self.accepted_timestamp and stamp - self.accepted_timestamp < self.interval_ns:
            return "detection_interval"
        return None

    def accept(self, observation):
        self.accepted_timestamp = observation.image_timestamp


def nearest_joint_state(history, stamp_ns, max_offset_sec):
    if not history:
        return None
    state = min(history, key=lambda item: abs(item["timestamp"] - stamp_ns))
    if abs(state["timestamp"] - stamp_ns) > int(max_offset_sec * 1e9):
        return None
    return state


def anchor_drift(before, after, translation_threshold=0.0011, rotation_threshold=0.25):
    translation = float(np.linalg.norm(after.translation - before.translation))
    rotation = math.degrees(float(np.linalg.norm(rotation_log(before.rotation.T @ after.rotation))))
    return {
        "translation_error": translation,
        "rotation_error": rotation,
        "translation_threshold_m": translation_threshold,
        "rotation_threshold_deg": rotation_threshold,
        "exceeded": translation > translation_threshold or rotation > rotation_threshold,
    }


class MotionSafetyError(RuntimeError):
    """Fatal motion errors must not be treated as skippable candidates."""


def confirm_trajectory_stop(handle, result_future, timeout_sec):
    """Only an action terminal result confirms stop, not a cancel response."""
    try:
        handle.cancel_goal_async()
    except Exception:
        # Even a failed cancel can race with a normally completed trajectory.
        pass
    deadline = time.monotonic() + timeout_sec
    while not result_future.done() and time.monotonic() < deadline:
        time.sleep(0.01)
    if result_future.done():
        try:
            result = result_future.result()
            if result is not None and result.status in (4, 5, 6):
                return result
        except Exception:
            pass
    raise MotionSafetyError("MOTION_STOP_UNCONFIRMED: trajectory has no confirmed terminal result")
