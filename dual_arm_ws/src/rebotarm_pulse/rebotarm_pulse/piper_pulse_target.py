"""Observe the vision pulse point in Piper-H's planning frame without motion."""

from __future__ import annotations

import copy
from collections import deque
import math
import json
import os
from pathlib import Path
import threading
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, PointStamped, TransformStamped, Vector3Stamped
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_geometry_msgs import do_transform_point, do_transform_vector3
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener
from visualization_msgs.msg import Marker
import yaml

from .target_pipeline_diagnostics import (
    PipelineCounters,
    PublishGapMonitor,
    TARGET_REASON_CODES,
    TargetPipelineLog,
)


def load_saved_calibration(name: str) -> TransformStamped | None:
    """Load easy_handeye2's saved transform without publishing a duplicate TF child."""
    if not name or "/" in name or ".." in name:
        return None
    path = Path(os.environ.get("EASY_HANDEYE2_CALIBRATIONS_DIRECTORY", str(Path.home() / ".ros2/easy_handeye2/calibrations"))) / f"{name}.calib"
    if not path.is_file():
        return None
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except (OSError, yaml.YAMLError):
        return None
    if not isinstance(document, dict):
        return None
    parameters = document.get("parameters", {})
    values = document.get("transform", {})
    translation = values.get("translation", {})
    rotation = values.get("rotation", {})
    calibration_type = str(parameters.get("calibration_type", ""))
    parent = str(
        parameters.get(
            "robot_effector_frame" if calibration_type == "eye_in_hand"
            else "robot_base_frame",
            "",
        )
    )
    child = str(parameters.get("tracking_base_frame", ""))
    required = (
        translation.get("x"), translation.get("y"), translation.get("z"),
        rotation.get("x"), rotation.get("y"), rotation.get("z"), rotation.get("w"),
    )
    if calibration_type not in ("eye_in_hand", "eye_on_base") or not parent or not child:
        return None
    try:
        numeric = tuple(float(value) for value in required)
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(value) for value in numeric):
        return None
    transform = TransformStamped()
    transform.header.frame_id = parent
    transform.child_frame_id = child
    transform.transform.translation.x = numeric[0]
    transform.transform.translation.y = numeric[1]
    transform.transform.translation.z = numeric[2]
    transform.transform.rotation.x = numeric[3]
    transform.transform.rotation.y = numeric[4]
    transform.transform.rotation.z = numeric[5]
    transform.transform.rotation.w = numeric[6]
    return transform


def quaternion_from_z_axis(vector) -> tuple[float, float, float, float]:
    """Return a deterministic frame whose local +Z follows ``vector``.

    The board detector currently publishes a point and a normal rather than a
    complete board pose.  Keep +X as close as possible to planning-frame +X
    (falling back to +Y near the singular case), then construct a right-handed
    orthonormal frame.  This makes the RViz TF useful for checking both the red
    cross position and the approach direction without implying an arbitrary
    changing roll.
    """
    z = (float(vector.x), float(vector.y), float(vector.z))
    length = math.sqrt(sum(value * value for value in z))
    if not math.isfinite(length) or length <= 1.0e-9:
        raise ValueError("board normal is zero or non-finite")
    z = tuple(value / length for value in z)
    reference = (1.0, 0.0, 0.0) if abs(z[0]) < 0.9 else (0.0, 1.0, 0.0)
    dot = sum(a * b for a, b in zip(reference, z))
    x = tuple(a - dot * b for a, b in zip(reference, z))
    x_length = math.sqrt(sum(value * value for value in x))
    x = tuple(value / x_length for value in x)
    y = (
        z[1] * x[2] - z[2] * x[1],
        z[2] * x[0] - z[0] * x[2],
        z[0] * x[1] - z[1] * x[0],
    )
    # Rotation-matrix columns are the child frame axes expressed in the parent.
    m00, m01, m02 = x[0], y[0], z[0]
    m10, m11, m12 = x[1], y[1], z[1]
    m20, m21, m22 = x[2], y[2], z[2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = (
            (m21 - m12) / scale,
            (m02 - m20) / scale,
            (m10 - m01) / scale,
            0.25 * scale,
        )
    elif m00 > m11 and m00 > m22:
        scale = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        quaternion = (
            0.25 * scale,
            (m01 + m10) / scale,
            (m02 + m20) / scale,
            (m21 - m12) / scale,
        )
    elif m11 > m22:
        scale = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        quaternion = (
            (m01 + m10) / scale,
            0.25 * scale,
            (m12 + m21) / scale,
            (m02 - m20) / scale,
        )
    else:
        scale = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        quaternion = (
            (m02 + m20) / scale,
            (m12 + m21) / scale,
            0.25 * scale,
            (m10 - m01) / scale,
        )
    q_length = math.sqrt(sum(value * value for value in quaternion))
    return tuple(value / q_length for value in quaternion)


def nearest_stamp_ns(stamps, target_ns: int) -> int | None:
    """Return the recorded timestamp nearest to an image acquisition time."""
    candidates = [int(value) for value in stamps if int(value) > 0]
    return min(candidates, key=lambda value: abs(value - target_ns)) if candidates else None


def timing_distribution(values) -> dict:
    """Summarize absolute millisecond deltas with a nearest-rank P95."""
    samples = sorted(abs(float(value)) for value in values if math.isfinite(float(value)))
    if not samples:
        return {"count": 0, "mean_ms": None, "p95_ms": None, "max_ms": None}
    p95_index = max(0, math.ceil(0.95 * len(samples)) - 1)
    return {
        "count": len(samples),
        "mean_ms": sum(samples) / len(samples),
        "p95_ms": samples[p95_index],
        "max_ms": samples[-1],
    }


class PiperPulseTarget(Node):
    def __init__(self) -> None:
        super().__init__("piper_pulse_target")
        self.declare_parameter("target_topic", "/meridian_hand_vision/pulse_point")
        self.declare_parameter(
            "arm_direction_topic", "/meridian_hand_vision/arm_direction"
        )
        self.declare_parameter(
            "surface_normal_topic", "/meridian_hand_vision/surface_normal"
        )
        self.declare_parameter("planning_frame", "piperh_planning_world")
        self.declare_parameter("calibration_name", "piperh_camera_eye_in_hand")
        # The camera driver and easy_handeye2 cannot both own the same optical
        # child frame in one TF tree. Load the saved calibration directly and
        # bypass that duplicate-child ambiguity.
        self.declare_parameter("calibration_bridge_frame", "piperh/Link5")
        self.declare_parameter(
            "board_target_frame", "piperh/calibration_board_lower_right_target"
        )
        self.declare_parameter("pulse_target_frame", "piperh/pulse_region_target")
        self.declare_parameter("tf_timeout_sec", 0.1)
        self.declare_parameter("input_hard_age_sec", 1.5)
        self.declare_parameter("heartbeat_hz", 2.0)
        self.declare_parameter("statistics_period_sec", 5.0)
        self.declare_parameter("joint_state_topic", "/piperh/joint_states")
        self.declare_parameter("timing_debug_topic", "/handeye_target/timing_debug")
        self.declare_parameter("timing_statistics_window", 300)
        self.declare_parameter("timing_motion_velocity_mps", 0.1)
        self.declare_parameter("gap_warning_sec", [0.5, 1.0, 2.0])
        self.declare_parameter("target_range_min_m", [-2.0, -2.0, 0.0])
        self.declare_parameter("target_range_max_m", [2.0, 2.0, 3.0])
        self.declare_parameter("diagnostic_log_directory", "logs/pulse_precontact")
        self.declare_parameter(
            "source_status_topic", "/meridian_hand_vision/pulse_status"
        )
        self.planning_frame = str(self.get_parameter("planning_frame").value)
        self.calibration_bridge_frame = str(
            self.get_parameter("calibration_bridge_frame").value
        )
        self.board_target_frame = str(self.get_parameter("board_target_frame").value)
        self.pulse_target_frame = str(self.get_parameter("pulse_target_frame").value)
        self.tf_timeout_sec = float(self.get_parameter("tf_timeout_sec").value)
        calibration_name = str(self.get_parameter("calibration_name").value)
        self.calibration_name = calibration_name
        self.calibration_transform = load_saved_calibration(calibration_name)
        self._calibration_translation = self._translation_tuple(
            self.calibration_transform
        )
        self._last_transform_warning = 0.0
        self._started_at = time.monotonic()
        self._last_camera_rx = 0.0
        self._last_frame_rx = 0.0
        self._last_detection_rx = 0.0
        self._last_valid_target_rx = 0.0
        self._last_valid_target_stamp_ns = 0
        self._last_target_input_rx = 0.0
        self._target_input_seq = 0
        self._publish_seq = 0
        self._last_tf_age_ms = None
        self._tf_timing_local = threading.local()
        self._timing_lock = threading.Lock()
        timing_window = max(10, int(self.get_parameter("timing_statistics_window").value))
        self._joint_stamp_history: deque[int] = deque(maxlen=max(100, timing_window * 2))
        self._timing_history = {
            "rgb_depth": deque(maxlen=timing_window),
            "rgb_tf": deque(maxlen=timing_window),
            "rgb_joint": deque(maxlen=timing_window),
        }
        self._last_tf_ok = False
        self._last_blocker = "NO_CAMERA_FRAME"
        self._last_source_status: dict = {}
        self._last_source_heartbeat: dict = {}
        self._last_sync_drops_total = 0
        self._last_valid_target: PointStamped | None = None
        self._last_pulse_normal: Vector3Stamped | None = None
        self._last_world_xyz = None
        self._last_board_corner: PointStamped | None = None
        self._last_board_normal: Vector3Stamped | None = None
        self._last_heartbeat = 0.0
        self._last_statistics = time.monotonic()
        self._counters = PipelineCounters()
        self._gap = PublishGapMonitor(
            self.get_parameter("gap_warning_sec").value
        )
        log_directory = os.path.abspath(
            str(self.get_parameter("diagnostic_log_directory").value)
        )
        self._pipeline_log = TargetPipelineLog(log_directory)
        if self.calibration_transform is None:
            self.get_logger().warning(
                f"saved hand-eye calibration is unavailable: {calibration_name}"
            )
        else:
            self.get_logger().info(
                "using saved hand-eye calibration directly: "
                f"{self.calibration_transform.header.frame_id} -> "
                f"{self.calibration_transform.child_frame_id}"
            )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.target_publisher = self.create_publisher(
            PointStamped, "/piperh/pulse/target", 10
        )
        self.marker_publisher = self.create_publisher(
            Marker, "/piperh/pulse/target_marker", 10
        )
        self.arm_direction_publisher = self.create_publisher(
            Vector3Stamped, "/piperh/pulse/arm_direction", 10
        )
        self.surface_normal_publisher = self.create_publisher(
            Vector3Stamped, "/piperh/pulse/surface_normal", 10
        )
        self.board_corner_publisher = self.create_publisher(
            PointStamped, "/piperh/calibration_board/lower_right_corner", 10
        )
        self.board_normal_publisher = self.create_publisher(
            Vector3Stamped, "/piperh/calibration_board/normal", 10
        )
        self.diagnostic_publisher = self.create_publisher(
            DiagnosticArray, "/piperh/pulse/target_diagnostics", 10
        )
        self.timing_debug_publisher = self.create_publisher(
            String, str(self.get_parameter("timing_debug_topic").value), 10
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_state_topic").value),
            self.observe_joint_state,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointStamped,
            str(self.get_parameter("target_topic").value),
            self.observe,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("source_status_topic").value),
            self.observe_source_status,
            10,
        )
        calibration_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            String,
            "/piperh/pulse/calibration_selection",
            self.select_calibration,
            calibration_qos,
        )
        heartbeat_hz = max(0.1, float(self.get_parameter("heartbeat_hz").value))
        self.create_timer(1.0 / heartbeat_hz, self._heartbeat)
        self.create_timer(
            max(1.0, float(self.get_parameter("statistics_period_sec").value)),
            self._statistics,
        )
        self.get_logger().info(
            f"pulse target pipeline JSONL: {self._pipeline_log.path}"
        )
        self.create_subscription(
            Vector3Stamped,
            str(self.get_parameter("arm_direction_topic").value),
            self.observe_arm_direction,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Vector3Stamped,
            str(self.get_parameter("surface_normal_topic").value),
            self.observe_surface_normal,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointStamped,
            "/meridian_hand_vision/calibration_board_corner",
            self.observe_board_corner,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Vector3Stamped,
            "/meridian_hand_vision/calibration_board_normal",
            self.observe_board_normal,
            qos_profile_sensor_data,
        )

    def select_calibration(self, message: String) -> None:
        """Switch the saved transform used for subsequent camera observations."""
        name = message.data.strip()
        if not name or name == self.calibration_name:
            return
        transform = load_saved_calibration(name)
        if transform is None:
            self.get_logger().error(
                f"rejected unavailable or invalid hand-eye calibration: {name}"
            )
            return
        self.calibration_name = name
        self.calibration_transform = transform
        self._calibration_translation = self._translation_tuple(transform)
        self._last_valid_target = None
        self._last_valid_target_rx = 0.0
        self._last_valid_target_stamp_ns = 0
        self._last_target_input_rx = 0.0
        self._last_tf_ok = False
        self._last_blocker = "CALIBRATION_CHANGED"
        self._last_board_corner = None
        self._last_board_normal = None
        self._last_pulse_normal = None
        self.get_logger().info(
            "selected saved hand-eye calibration: "
            f"{name} ({transform.header.frame_id} -> {transform.child_frame_id})"
        )

    @staticmethod
    def _translation_tuple(transform: TransformStamped | None) -> tuple[float, float, float] | None:
        if transform is None:
            return None
        translation = transform.transform.translation
        return (translation.x, translation.y, translation.z)

    def _validated_point_calibration(
        self, calibration: TransformStamped
    ) -> TransformStamped:
        """Reject a hand-eye transform whose persistent translation was mutated."""
        current = self._translation_tuple(calibration)
        expected = getattr(self, "_calibration_translation", current)
        self.get_logger().info(
            "[HANDEYE TARGET] Link5_T_camera translation "
            f"x={current[0]:.9f} y={current[1]:.9f} z={current[2]:.9f}"
        )
        zeroed = all(abs(value) <= 1.0e-12 for value in current)
        changed = expected is not None and any(
            abs(actual - original) > 1.0e-12
            for actual, original in zip(current, expected)
        )
        if zeroed or changed:
            self.get_logger().error(
                "ERROR: calibration transform translation was modified; "
                f"expected={expected} actual={current}"
            )
            raise RuntimeError("calibration transform translation was modified")
        return calibration

    def _lookup_transform(self, target_frame: str, source_frame: str, stamp):
        query_stamp_ns = int(stamp.nanoseconds)
        if query_stamp_ns <= 0:
            raise TransformException(
                "latest TF lookup is forbidden for a moving hand target"
            )
        transform = self.tf_buffer.lookup_transform(
            target_frame, source_frame, stamp,
            timeout=Duration(seconds=self.tf_timeout_sec),
        )
        stamp_ns = (
            int(transform.header.stamp.sec) * 1_000_000_000
            + int(transform.header.stamp.nanosec)
        )
        if not hasattr(self._tf_timing_local, "stamps_ns"):
            self._tf_timing_local.stamps_ns = []
        self._tf_timing_local.query_stamp_ns = query_stamp_ns
        if stamp_ns > 0:
            self._tf_timing_local.stamps_ns.append(stamp_ns)
        self._last_tf_age_ms = (
            max(0.0, (self.get_clock().now().nanoseconds - stamp_ns) / 1_000_000.0)
            if stamp_ns > 0 else 0.0
        )
        self._tf_timing_local.tf_age_ms = self._last_tf_age_ms
        return transform

    def observe_joint_state(self, message: JointState) -> None:
        stamp_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        if stamp_ns > 0:
            with self._timing_lock:
                self._joint_stamp_history.append(stamp_ns)

    def _timing_record(self, context: dict, accepted: bool, reason: str) -> dict:
        rgb_stamp_ns = int(context.get("rgb_stamp_ns") or 0)
        depth_stamp_ns = int(context.get("depth_stamp_ns") or 0)
        camera_info_stamp_ns = int(context.get("camera_info_stamp_ns") or 0)
        tf_query_stamp_ns = int(
            getattr(self._tf_timing_local, "query_stamp_ns", 0) or 0
        )
        tf_stamp_ns = nearest_stamp_ns(
            getattr(self._tf_timing_local, "stamps_ns", []), rgb_stamp_ns
        )
        with self._timing_lock:
            joint_stamp_ns = nearest_stamp_ns(
                tuple(self._joint_stamp_history), rgb_stamp_ns
            )

        def delta_ms(other_ns: int | None) -> float | None:
            return (
                abs(rgb_stamp_ns - other_ns) / 1_000_000.0
                if rgb_stamp_ns > 0 and other_ns is not None and other_ns > 0 else None
            )

        rgb_depth_ms = delta_ms(depth_stamp_ns)
        rgb_tf_ms = delta_ms(tf_stamp_ns)
        rgb_joint_ms = delta_ms(joint_stamp_ns)
        current = {
            "rgb_depth": rgb_depth_ms,
            "rgb_tf": rgb_tf_ms,
            "rgb_joint": rgb_joint_ms,
        }
        velocity = float(self.get_parameter("timing_motion_velocity_mps").value)
        with self._timing_lock:
            for key, value in current.items():
                if value is not None:
                    self._timing_history[key].append(value)
            statistics = {
                key: timing_distribution(tuple(values))
                for key, values in self._timing_history.items()
            }
        for summary in statistics.values():
            for statistic in ("mean", "p95", "max"):
                delta = summary[f"{statistic}_ms"]
                summary[f"estimated_motion_error_{statistic}_mm"] = (
                    velocity * delta if delta is not None else None
                )
        record = {
            "accepted": accepted,
            "reason": reason,
            "rgb_stamp": rgb_stamp_ns / 1e9 if rgb_stamp_ns else None,
            "depth_stamp": depth_stamp_ns / 1e9 if depth_stamp_ns else None,
            "camera_info_stamp": camera_info_stamp_ns / 1e9 if camera_info_stamp_ns else None,
            "tf_query_time": (
                tf_query_stamp_ns / 1e9 if tf_query_stamp_ns > 0 else None
            ),
            "tf_stamp": tf_stamp_ns / 1e9 if tf_stamp_ns else None,
            "joint_state_nearest_stamp": joint_stamp_ns / 1e9 if joint_stamp_ns else None,
            "delta_rgb_depth_ms": rgb_depth_ms,
            "delta_rgb_tf_ms": rgb_tf_ms,
            "delta_rgb_joint_ms": rgb_joint_ms,
            "estimated_motion_error_mm": {
                key: velocity * value if value is not None else None
                for key, value in current.items()
            },
            "assumed_velocity_mps": velocity,
            "statistics": statistics,
        }
        self.timing_debug_publisher.publish(
            String(data=json.dumps(record, ensure_ascii=False, sort_keys=True))
        )
        self._safe_log("timing_debug", **record)
        return record

    def _transform_point(self, message: PointStamped) -> PointStamped:
        transformed = message
        calibration = self.calibration_transform
        if calibration is not None:
            calibration = self._validated_point_calibration(calibration)
            if transformed.header.frame_id != calibration.child_frame_id:
                camera_transform = self._lookup_transform(
                    calibration.child_frame_id,
                    transformed.header.frame_id,
                    Time.from_msg(transformed.header.stamp),
                )
                transformed = do_transform_point(transformed, camera_transform)
                transformed.header.stamp = message.header.stamp
            transformed = do_transform_point(transformed, calibration)
            transformed.header.stamp = message.header.stamp
        elif self.calibration_bridge_frame and (
            transformed.header.frame_id != self.calibration_bridge_frame
        ):
            camera_to_bridge = self._lookup_transform(
                self.calibration_bridge_frame,
                transformed.header.frame_id,
                Time.from_msg(transformed.header.stamp),
            )
            transformed = do_transform_point(transformed, camera_to_bridge)
            # Static TF headers carry stamp zero.  Preserve the camera sample
            # time so the moving Link5 pose is evaluated at image acquisition.
            transformed.header.stamp = message.header.stamp
        if transformed.header.frame_id != self.planning_frame:
            bridge_to_planning = self._lookup_transform(
                self.planning_frame,
                transformed.header.frame_id,
                Time.from_msg(transformed.header.stamp),
            )
            transformed = do_transform_point(transformed, bridge_to_planning)
        return transformed

    def _transform_vector(self, message: Vector3Stamped) -> Vector3Stamped:
        transformed = message
        calibration = self.calibration_transform
        if calibration is not None:
            if transformed.header.frame_id != calibration.child_frame_id:
                camera_transform = self._lookup_transform(
                    calibration.child_frame_id,
                    transformed.header.frame_id,
                    Time.from_msg(transformed.header.stamp),
                )
                transformed = do_transform_vector3(transformed, camera_transform)
                transformed.header.stamp = message.header.stamp
            # Jazzy's do_transform_vector3() zeroes the translation on the
            # TransformStamped passed to it.  Never give it the persistent
            # Link5_T_camera calibration object used by subsequent points.
            transformed = do_transform_vector3(
                transformed, copy.deepcopy(calibration)
            )
            transformed.header.stamp = message.header.stamp
        elif self.calibration_bridge_frame and (
            transformed.header.frame_id != self.calibration_bridge_frame
        ):
            camera_to_bridge = self._lookup_transform(
                self.calibration_bridge_frame,
                transformed.header.frame_id,
                Time.from_msg(transformed.header.stamp),
            )
            transformed = do_transform_vector3(transformed, camera_to_bridge)
            transformed.header.stamp = message.header.stamp
        if transformed.header.frame_id != self.planning_frame:
            bridge_to_planning = self._lookup_transform(
                self.planning_frame,
                transformed.header.frame_id,
                Time.from_msg(transformed.header.stamp),
            )
            transformed = do_transform_vector3(transformed, bridge_to_planning)
        return transformed

    def observe(self, message: PointStamped) -> None:
        received = time.monotonic()
        self._target_input_seq += 1
        input_interval_ms = (
            (received - self._last_target_input_rx) * 1000.0
            if self._last_target_input_rx else None
        )
        self._last_target_input_rx = received
        self._tf_timing_local.query_stamp_ns = 0
        self._tf_timing_local.stamps_ns = []
        self._tf_timing_local.tf_age_ms = None
        context = {
            "frame_seq": self._last_source_status.get("frame_seq", self._target_input_seq),
            "frame_rx_time": self._last_source_status.get("frame_rx_time", received),
            "source_target_rx_time": received,
            "frame_interval_ms": self._last_source_status.get(
                "frame_interval_ms", input_interval_ms
            ),
            "detection_present": self._last_source_status.get("detection_present", True),
            "detection_valid": self._last_source_status.get("detection_valid", True),
            "raw_pixel_uv": self._last_source_status.get("raw_pixel_uv"),
            "raw_depth": self._last_source_status.get("raw_depth", message.point.z),
            "plane_inlier_count": self._last_source_status.get("plane_inlier_count"),
            "plane_residual": self._last_source_status.get("plane_residual"),
            "normal_confidence": self._last_source_status.get("normal_confidence"),
            "depth_valid_ratio": self._last_source_status.get("depth_valid_ratio"),
            "normal_angle": self._last_source_status.get("normal_angle"),
            "rgb_stamp_ns": self._last_source_status.get("rgb_stamp_ns"),
            "depth_stamp_ns": self._last_source_status.get("depth_stamp_ns"),
            "camera_info_stamp_ns": self._last_source_status.get("camera_info_stamp_ns"),
            "delta_rgb_depth_ms": self._last_source_status.get("delta_rgb_depth_ms"),
            "camera_xyz": (message.point.x, message.point.y, message.point.z),
            "surface_point_camera": (
                message.point.x, message.point.y, message.point.z
            ),
            "target_valid_before_tf": False,
            "tf_success": False,
            "tf_age_ms": None,
            "world_xyz": None,
            "publish_target": False,
            "publish_seq": self._publish_seq,
        }
        try:
            if not message.header.frame_id:
                self._drop("EMPTY_FRAME_ID", context)
                return
            source_xyz = (message.point.x, message.point.y, message.point.z)
            if not all(math.isfinite(value) for value in source_xyz):
                self._drop("NONFINITE_TARGET", context)
                return
            stamp_ns = (
                int(message.header.stamp.sec) * 1_000_000_000
                + int(message.header.stamp.nanosec)
            )
            if stamp_ns <= 0:
                self._drop("MISSING_TIMESTAMP", context)
                return
            context["rgb_stamp_ns"] = int(context.get("rgb_stamp_ns") or stamp_ns)
            acquisition_age = (
                self.get_clock().now().nanoseconds - stamp_ns
            ) / 1_000_000_000.0
            if (
                not math.isfinite(acquisition_age) or acquisition_age < -0.1
                or acquisition_age > float(self.get_parameter("input_hard_age_sec").value)
            ):
                self._counters.tf_failed += 1
                context["input_age_ms"] = acquisition_age * 1000.0
                self._drop("TF_STALE", context, event_type="tf_failure")
                return
            context["target_valid_before_tf"] = True
            tf_started = time.monotonic()
            target = self._transform_point(message)
            context["tf_lookup_duration_ms"] = (time.monotonic() - tf_started) * 1000.0
        except TransformException as error:
            self._counters.tf_failed += 1
            context["exception"] = str(error)
            self._timing_record(context, False, "TF_LOOKUP_FAILED")
            self._drop("TF_LOOKUP_FAILED", context, event_type="tf_failure")
            return
        except Exception as error:
            context["exception"] = repr(error)
            self._timing_record(context, False, "INTERNAL_EXCEPTION")
            self._drop("INTERNAL_EXCEPTION", context, event_type="exception")
            self.get_logger().error(f"pulse target pipeline exception: {error}")
            return
        world_xyz = (target.point.x, target.point.y, target.point.z)
        context.update(
            tf_success=True,
            tf_age_ms=getattr(self._tf_timing_local, "tf_age_ms", None),
            world_xyz=world_xyz,
            surface_point_base=world_xyz,
        )
        self.get_logger().info(
            "HAND_SURFACE_POINT_TF "
            f"surface_point_camera={context['surface_point_camera']} "
            f"surface_point_base={world_xyz}"
        )
        self._last_tf_ok = True
        self._counters.tf_success += 1
        if not all(math.isfinite(value) for value in world_xyz):
            self._timing_record(context, False, "NONFINITE_TARGET")
            self._drop("NONFINITE_TARGET", context)
            return
        minimum = tuple(float(value) for value in self.get_parameter("target_range_min_m").value)
        maximum = tuple(float(value) for value in self.get_parameter("target_range_max_m").value)
        if not all(low <= value <= high for value, low, high in zip(world_xyz, minimum, maximum)):
            context["allowed_range"] = {"minimum": minimum, "maximum": maximum}
            self._timing_record(context, False, "TARGET_OUT_OF_RANGE")
            self._drop("TARGET_OUT_OF_RANGE", context)
            return
        try:
            self.target_publisher.publish(target)
        except Exception as error:
            context["exception"] = repr(error)
            self._timing_record(context, False, "INTERNAL_EXCEPTION")
            self._drop("INTERNAL_EXCEPTION", context, event_type="exception")
            self.get_logger().error(f"pulse target publish failed: {error!r}")
            return
        self._publish_seq += 1
        self._counters.targets_published += 1
        self._gap.published(received)
        self._last_valid_target_rx = received
        self._last_valid_target_stamp_ns = stamp_ns
        self._last_valid_target = target
        self._last_world_xyz = world_xyz
        self._last_blocker = "NONE"
        self._publish_pulse_target_tf()
        timing_record = self._timing_record(context, True, "NONE")
        context.update(
            publish_target=True,
            publish_seq=self._publish_seq,
            last_publish_interval_ms=(
                self._gap.last_interval_sec * 1000.0
                if self._gap.last_interval_sec is not None else None
            ),
            last_publish_age_ms=0.0,
            tf_query_time=timing_record["tf_query_time"],
            joint_state_nearest_stamp=timing_record["joint_state_nearest_stamp"],
            delta_rgb_tf_ms=timing_record["delta_rgb_tf_ms"],
            delta_rgb_joint_ms=timing_record["delta_rgb_joint_ms"],
        )
        self._safe_log("target_publish", **context)
        self._publish_marker(received)

    def _drop(self, reason: str, context: dict, event_type: str = "target_drop") -> None:
        if reason not in TARGET_REASON_CODES:
            reason = "INTERNAL_EXCEPTION"
        self._last_blocker = reason
        self._last_tf_ok = bool(context.get("tf_success"))
        self._counters.dropped(reason)
        context.update(
            publish_target=False,
            publish_seq=self._publish_seq,
            last_publish_interval_ms=(
                self._gap.last_interval_sec * 1000.0
                if self._gap.last_interval_sec is not None else None
            ),
            last_publish_age_ms=self._publish_age_ms(time.monotonic()),
            reason=reason,
        )
        self._safe_log(event_type, **context)

    def _safe_log(self, event_type: str, **fields) -> None:
        try:
            self._pipeline_log.write(event_type, **fields)
        except (OSError, TypeError, ValueError) as error:
            self.get_logger().error(f"target pipeline JSONL write failed: {error}")

    def observe_source_status(self, message: String) -> None:
        received = time.monotonic()
        try:
            status = json.loads(message.data)
            if not isinstance(status, dict):
                raise ValueError("pulse status must be a JSON object")
        except (json.JSONDecodeError, ValueError) as error:
            self._last_blocker = "INTERNAL_EXCEPTION"
            self._safe_log("exception", reason="INTERNAL_EXCEPTION", exception=str(error))
            return
        source_heartbeat = status.get("event_type") == "source_heartbeat"
        if source_heartbeat:
            self._last_source_heartbeat = status
            self._last_camera_rx = received if status.get("camera_rx") else self._last_camera_rx
            sync_drops_total = int(status.get("sync_drops_total", self._last_sync_drops_total))
            self._counters.dropped(
                "NO_SYNCED_PAIR", max(0, sync_drops_total - self._last_sync_drops_total)
            )
            self._last_sync_drops_total = sync_drops_total
            if status.get("reason") == "NO_SYNCED_PAIR":
                self._last_blocker = "NO_SYNCED_PAIR"
            self._safe_log(
                "source_heartbeat",
                **{key: value for key, value in status.items() if key != "event_type"},
            )
            return
        self._last_source_status = status
        self._last_frame_rx = received
        self._last_camera_rx = received
        self._counters.frames_received += 1
        detection_present = bool(status.get("detection_present"))
        if detection_present:
            self._last_detection_rx = received
            self._counters.detections_received += 1
        if bool(status.get("detection_3d_valid", status.get("target_valid_before_tf"))):
            self._counters.detections_valid += 1
        raw_reason = str(status.get("reason", "INTERNAL_EXCEPTION"))
        legacy = {
            "stable": "NONE", "stabilizing": "FILTER_STABILIZING",
            "invalid_depth": "INVALID_DEPTH",
            "waiting_for_one_hand": "NO_DETECTION",
            "waiting_for_camera_info": "INVALID_CAMERA_INFO",
        }
        reason = legacy.get(raw_reason, raw_reason)
        if reason not in TARGET_REASON_CODES:
            reason = "INTERNAL_EXCEPTION"
        if reason == "INVALID_DEPTH":
            self._counters.depth_rejected += 1
        frame_event = {
            **status,
            "frame_rx_time": status.get("frame_rx_time", received),
            "source_status_rx_time": received,
            "target_valid_before_tf": bool(status.get("target_valid_before_tf")),
            "tf_success": False,
            "tf_age_ms": None,
            "world_xyz": None,
            "publish_target": False,
            "publish_seq": self._publish_seq,
            "last_publish_interval_ms": (
                self._gap.last_interval_sec * 1000.0
                if self._gap.last_interval_sec is not None else None
            ),
            "last_publish_age_ms": self._publish_age_ms(received),
        }
        if not bool(status.get("publish_source_target")):
            self._last_blocker = reason
            self._counters.dropped(reason)
            drop_event = dict(frame_event)
            drop_event["reason"] = reason
            self._safe_log("target_drop", stage="FILTER", **drop_event)
        self._safe_log("frame_result", blocker=reason, **frame_event)

    def _publish_age_ms(self, now: float) -> float | None:
        if self._gap.last_publish_time is None:
            return None
        return (now - self._gap.last_publish_time) * 1000.0

    def _snapshot(self, now: float | None = None) -> dict:
        now = time.monotonic() if now is None else now
        gap = self._gap.current_gap(now)
        acquisition_age_ms = (
            max(
                0.0,
                (self.get_clock().now().nanoseconds - self._last_valid_target_stamp_ns)
                / 1_000_000.0,
            )
            if self._last_valid_target_stamp_ns else None
        )
        camera_alive = bool(self._last_camera_rx and now - self._last_camera_rx <= 1.0)
        publish_recent = gap is not None and gap <= 0.5
        if not camera_alive:
            effective_blocker = "NO_CAMERA_FRAME"
        elif not self._last_frame_rx or now - self._last_frame_rx > 1.0:
            effective_blocker = "NO_SYNCED_PAIR"
        elif self._last_blocker == "NONE" and not publish_recent:
            # The source reported a publishable frame but no corresponding
            # PointStamped reached/passed the bridge within one heartbeat.
            effective_blocker = "TARGET_MESSAGE_MISSING"
        else:
            effective_blocker = self._last_blocker
        return {
            "calibration_name": self.calibration_name,
            "camera_alive": camera_alive,
            "detection_alive": bool(
                self._last_detection_rx and now - self._last_detection_rx <= 1.0
            ),
            "target_valid": bool(
                acquisition_age_ms is not None
                and acquisition_age_ms <= 1000.0 * float(
                    self.get_parameter("input_hard_age_sec").value)
            ),
            "publish_recent": publish_recent,
            "tf_ok": self._last_tf_ok,
            "last_frame_age_ms": (
                (now - self._last_frame_rx) * 1000.0 if self._last_frame_rx else None
            ),
            "last_camera_rx_age_ms": (
                (now - self._last_camera_rx) * 1000.0 if self._last_camera_rx else None
            ),
            "last_detection_age_ms": (
                (now - self._last_detection_rx) * 1000.0
                if self._last_detection_rx else None
            ),
            # This is acquisition age from the original camera header stamp,
            # not bridge receipt age.  The original PointStamped is never
            # refreshed or re-published as a new visual observation.
            "last_valid_target_age_ms": acquisition_age_ms,
            "last_publish_age_ms": gap * 1000.0 if gap is not None else None,
            "current_publish_gap_ms": gap * 1000.0 if gap is not None else None,
            "max_publish_gap_ms": self._gap.max_gap_sec * 1000.0,
            "last_publish_interval_ms": (
                self._gap.last_interval_sec * 1000.0
                if self._gap.last_interval_sec is not None else None
            ),
            "publish_rate_hz": self._gap.rate_hz(now),
            "last_blocker": effective_blocker,
            "last_valid_target": self._last_world_xyz,
            **self._counters.snapshot(),
        }

    def _heartbeat(self) -> None:
        now = time.monotonic()
        snapshot = self._snapshot(now)
        self._publish_diagnostics(snapshot)
        self._publish_marker(now)
        gap, crossed = self._gap.inspect(now)
        for threshold in crossed:
            self.get_logger().warning(
                "TARGET_PUBLISH_GAP_WARNING "
                f"threshold={threshold * 1000.0:.0f}ms "
                f"last_publish_age={gap * 1000.0:.0f}ms "
                f"last_valid_target={self._last_world_xyz} "
                f"last_blocker={snapshot['last_blocker']}"
            )
            self._safe_log(
                "publish_gap", threshold_ms=threshold * 1000.0,
                last_publish_age_ms=gap * 1000.0,
                last_valid_target=self._last_world_xyz,
                last_blocker=snapshot["last_blocker"],
            )
        self.get_logger().info(
            "[PULSE TARGET] "
            f"camera_rx={'YES' if snapshot['camera_alive'] else 'NO'} "
            f"camera_rate={self._camera_rate_hz():.1f}Hz "
            f"synced_frame={'YES' if snapshot['last_frame_age_ms'] is not None and snapshot['last_frame_age_ms'] <= 1000.0 else 'NO'} "
            f"detection={'YES' if snapshot['detection_alive'] else 'NO'} "
            f"tf={'PASS' if snapshot['tf_ok'] else 'FAIL'} "
            f"target_publish={'YES' if snapshot['publish_recent'] else 'NO'} "
            f"last_publish_age={snapshot['last_publish_age_ms']}ms "
            f"BLOCKER={snapshot['last_blocker']}"
        )
        self._safe_log("heartbeat", **snapshot)

    def _camera_rate_hz(self) -> float:
        if self._last_frame_rx and time.monotonic() - self._last_frame_rx > 0.5:
            return float(self._last_source_heartbeat.get("camera_rate_hz", 0.0))
        interval = self._last_source_status.get("frame_interval_ms")
        try:
            return 1000.0 / float(interval) if float(interval) > 0.0 else 0.0
        except (TypeError, ValueError):
            return 0.0

    def _statistics(self) -> None:
        snapshot = self._snapshot()
        self.get_logger().info(
            "[PULSE TARGET STATS] "
            f"frames={snapshot['frames_received']} "
            f"detections={snapshot['detections_received']} "
            f"valid_3D={snapshot['detections_valid']} "
            f"TF_success={snapshot['tf_success']} "
            f"published={snapshot['targets_published']} "
            f"drop_reason={snapshot['drop_reasons']}"
        )
        self._safe_log("statistics", **snapshot)

    def _publish_diagnostics(self, snapshot: dict) -> None:
        status = DiagnosticStatus()
        status.name = "Piper-H pulse target pipeline"
        status.hardware_id = "piperh_vision"
        status.level = DiagnosticStatus.OK if snapshot["target_valid"] else DiagnosticStatus.WARN
        status.message = "OK" if snapshot["target_valid"] else snapshot["last_blocker"]
        keys = (
            "camera_alive", "detection_alive", "target_valid", "tf_ok",
            "last_frame_age_ms", "last_detection_age_ms",
            "last_valid_target_age_ms", "last_publish_age_ms", "publish_rate_hz",
            "max_publish_gap_ms", "last_blocker", "targets_published",
        )
        status.values = [KeyValue(key=key, value=str(snapshot.get(key))) for key in keys]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.diagnostic_publisher.publish(array)

    def _publish_marker(self, now: float) -> None:
        if self._last_valid_target is None or self._gap.last_publish_time is None:
            return
        if self._last_valid_target_stamp_ns:
            age = max(
                0.0,
                (self.get_clock().now().nanoseconds - self._last_valid_target_stamp_ns)
                / 1_000_000_000.0,
            )
        else:
            age = now - self._gap.last_publish_time
        marker = Marker()
        # Do not alias and mutate the stored target header: its camera
        # acquisition stamp is immutable evidence. Only this visualization
        # marker gets a current render stamp.
        marker.header.frame_id = self._last_valid_target.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "piper_pulse_target"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = self._last_valid_target.point
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.018
        if age <= 0.5:
            marker.color.r, marker.color.g, marker.color.b = 0.1, 1.0, 0.1
        elif age <= 1.5:
            marker.color.r, marker.color.g, marker.color.b = 1.0, 0.85, 0.0
        else:
            marker.color.r, marker.color.g, marker.color.b = 1.0, 0.05, 0.05
        marker.color.a = 0.95
        self.marker_publisher.publish(marker)
        label = Marker()
        label.header = marker.header
        label.ns = "piper_pulse_target_age"
        label.id = 1
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position = self._last_valid_target.point
        label.pose.position.z += 0.025
        label.pose.orientation.w = 1.0
        label.scale.z = 0.014
        label.color.r, label.color.g, label.color.b, label.color.a = (
            marker.color.r, marker.color.g, marker.color.b, 1.0
        )
        label.text = f"Target age: {age * 1000.0:.0f} ms" + (
            " STALE" if age > 0.5 else ""
        )
        self.marker_publisher.publish(label)

    def observe_arm_direction(self, message: Vector3Stamped) -> None:
        if not message.header.frame_id:
            self._safe_log("target_drop", stage="ARM_DIRECTION", reason="EMPTY_FRAME_ID")
            return
        try:
            direction = self._transform_vector(message)
        except TransformException as error:
            self._safe_log(
                "tf_failure", stage="ARM_DIRECTION", reason="TF_LOOKUP_FAILED",
                exception=str(error),
            )
            return
        except Exception as error:
            self._safe_log(
                "exception", stage="ARM_DIRECTION", reason="INTERNAL_EXCEPTION",
                exception=repr(error),
            )
            return
        self.arm_direction_publisher.publish(direction)

    def observe_surface_normal(self, message: Vector3Stamped) -> None:
        if not message.header.frame_id:
            self._safe_log("target_drop", stage="SURFACE_NORMAL", reason="EMPTY_FRAME_ID")
            return
        try:
            normal = self._transform_vector(message)
        except TransformException as error:
            self._safe_log(
                "tf_failure", stage="SURFACE_NORMAL", reason="TF_LOOKUP_FAILED",
                exception=str(error),
            )
            return
        except Exception as error:
            self._safe_log(
                "exception", stage="SURFACE_NORMAL", reason="INTERNAL_EXCEPTION",
                exception=repr(error),
            )
            return
        self.surface_normal_publisher.publish(normal)
        self._last_pulse_normal = normal
        camera_normal = (message.vector.x, message.vector.y, message.vector.z)
        base_normal = (normal.vector.x, normal.vector.y, normal.vector.z)
        self.get_logger().info(
            "HAND_SURFACE_TF "
            f"surface_normal_camera={camera_normal} "
            f"surface_normal_base={base_normal} "
            f"normal_angle={self._last_source_status.get('normal_angle')}deg"
        )
        self._publish_pulse_target_tf()
        if self._last_valid_target is not None:
            vector = normal.vector
            length = math.sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z)
            if length > 1e-6:
                nx, ny, nz = vector.x / length, vector.y / length, vector.z / length
                target = self._last_valid_target.point
                marker = Marker()
                marker.header = normal.header
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "piper_surface_normal"
                marker.id = 2
                marker.type = Marker.ARROW
                marker.action = Marker.ADD
                marker.points = [
                    Point(x=target.x - 0.12 * nx, y=target.y - 0.12 * ny, z=target.z - 0.12 * nz),
                    Point(x=target.x, y=target.y, z=target.z),
                ]
                marker.scale.x, marker.scale.y, marker.scale.z = 0.006, 0.012, 0.018
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.1, 0.5, 1.0, 0.95
                marker.lifetime.nanosec = 500_000_000
                self.marker_publisher.publish(marker)

    def _publish_pulse_target_tf(self) -> None:
        """Broadcast the final 3D pulse-region center used by the planner.

        Local +Z follows the detected skin normal.  Requiring matching source
        timestamps prevents an axis from one camera frame being attached to a
        target point from another frame.
        """
        target = self._last_valid_target
        normal = self._last_pulse_normal
        if target is None or normal is None or not self.pulse_target_frame:
            return
        target_stamp = (
            int(target.header.stamp.sec) * 1_000_000_000
            + int(target.header.stamp.nanosec)
        )
        normal_stamp = (
            int(normal.header.stamp.sec) * 1_000_000_000
            + int(normal.header.stamp.nanosec)
        )
        if (
            target_stamp != normal_stamp
            or target.header.frame_id != normal.header.frame_id
            or target.header.frame_id != self.planning_frame
        ):
            return
        try:
            quaternion = quaternion_from_z_axis(normal.vector)
        except ValueError as error:
            self._safe_log(
                "target_drop", stage="PULSE_TARGET_TF", reason="INVALID_SURFACE_NORMAL",
                exception=str(error),
            )
            return
        transform = TransformStamped()
        transform.header = target.header
        transform.child_frame_id = self.pulse_target_frame
        transform.transform.translation.x = target.point.x
        transform.transform.translation.y = target.point.y
        transform.transform.translation.z = target.point.z
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]
        self.tf_broadcaster.sendTransform(transform)

    def observe_board_corner(self, message: PointStamped) -> None:
        """Transform the image-defined board corner with the selected hand-eye result."""
        if not message.header.frame_id:
            self._safe_log(
                "target_drop", stage="BOARD_CORNER", reason="EMPTY_FRAME_ID"
            )
            return
        try:
            corner = self._transform_point(message)
        except TransformException as error:
            self._safe_log(
                "tf_failure", stage="BOARD_CORNER", reason="TF_LOOKUP_FAILED",
                exception=str(error),
            )
            return
        except Exception as error:
            self._safe_log(
                "exception", stage="BOARD_CORNER", reason="INTERNAL_EXCEPTION",
                exception=repr(error),
            )
            return
        xyz = (corner.point.x, corner.point.y, corner.point.z)
        if not all(math.isfinite(value) for value in xyz):
            self._safe_log(
                "target_drop", stage="BOARD_CORNER", reason="NONFINITE_TARGET"
            )
            return
        # Pair the corner with the normal from the same camera frame even when
        # the final dynamic TF lookup returns its own header timestamp.
        corner.header.stamp = message.header.stamp
        self.board_corner_publisher.publish(corner)
        self._last_board_corner = corner
        self._publish_board_target_tf()
        self._safe_log(
            "board_corner_publish", calibration_name=self.calibration_name,
            camera_xyz=(message.point.x, message.point.y, message.point.z),
            world_xyz=xyz,
        )

    def observe_board_normal(self, message: Vector3Stamped) -> None:
        if not message.header.frame_id:
            self._safe_log(
                "target_drop", stage="BOARD_NORMAL", reason="EMPTY_FRAME_ID"
            )
            return
        try:
            normal = self._transform_vector(message)
        except TransformException as error:
            self._safe_log(
                "tf_failure", stage="BOARD_NORMAL", reason="TF_LOOKUP_FAILED",
                exception=str(error),
            )
            return
        except Exception as error:
            self._safe_log(
                "exception", stage="BOARD_NORMAL", reason="INTERNAL_EXCEPTION",
                exception=repr(error),
            )
            return
        normal.header.stamp = message.header.stamp
        self.board_normal_publisher.publish(normal)
        self._last_board_normal = normal
        self._publish_board_target_tf()

    def _publish_board_target_tf(self) -> None:
        """Broadcast the exact transformed red-cross target for RViz TF display."""
        corner = self._last_board_corner
        normal = self._last_board_normal
        if corner is None or normal is None or not self.board_target_frame:
            return
        corner_stamp = (
            int(corner.header.stamp.sec) * 1_000_000_000
            + int(corner.header.stamp.nanosec)
        )
        normal_stamp = (
            int(normal.header.stamp.sec) * 1_000_000_000
            + int(normal.header.stamp.nanosec)
        )
        if (
            corner_stamp != normal_stamp
            or corner.header.frame_id != normal.header.frame_id
            or corner.header.frame_id != self.planning_frame
        ):
            return
        try:
            quaternion = quaternion_from_z_axis(normal.vector)
        except ValueError as error:
            self._safe_log(
                "target_drop", stage="BOARD_TARGET_TF", reason="INVALID_BOARD_NORMAL",
                exception=str(error),
            )
            return
        transform = TransformStamped()
        transform.header = corner.header
        transform.child_frame_id = self.board_target_frame
        transform.transform.translation.x = corner.point.x
        transform.transform.translation.y = corner.point.y
        transform.transform.translation.z = corner.point.z
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]
        self.tf_broadcaster.sendTransform(transform)


def main() -> None:
    rclpy.init()
    node = PiperPulseTarget()
    # Target and arm-direction callbacks perform timestamped TF lookups.  A
    # SingleThreadedExecutor cannot receive /tf while either lookup is waiting,
    # so a brief miss makes the TF buffer fall progressively farther behind the
    # camera stream.  TransformListener uses its own reentrant callback group;
    # multiple executor threads let that group refill the buffer concurrently.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
