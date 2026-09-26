"""Observe the vision pulse point in Piper-H's planning frame without motion."""

from __future__ import annotations

import math
import json
import os
from pathlib import Path
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PointStamped, TransformStamped, Vector3Stamped
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import String
from tf2_geometry_msgs import do_transform_point, do_transform_vector3
from tf2_ros import Buffer, TransformException, TransformListener
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


class PiperPulseTarget(Node):
    def __init__(self) -> None:
        super().__init__("piper_pulse_target")
        self.declare_parameter("target_topic", "/meridian_hand_vision/pulse_point")
        self.declare_parameter(
            "arm_direction_topic", "/meridian_hand_vision/arm_direction"
        )
        self.declare_parameter("planning_frame", "piperh_planning_world")
        self.declare_parameter("calibration_name", "piperh_camera_eye_in_hand")
        # The camera driver and easy_handeye2 cannot both own the same optical
        # child frame in one TF tree. Load the saved calibration directly and
        # bypass that duplicate-child ambiguity.
        self.declare_parameter("calibration_bridge_frame", "piperh/Link5")
        self.declare_parameter("tf_timeout_sec", 0.1)
        self.declare_parameter("input_hard_age_sec", 1.5)
        self.declare_parameter("heartbeat_hz", 2.0)
        self.declare_parameter("statistics_period_sec", 5.0)
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
        self.tf_timeout_sec = float(self.get_parameter("tf_timeout_sec").value)
        calibration_name = str(self.get_parameter("calibration_name").value)
        self.calibration_transform = load_saved_calibration(calibration_name)
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
        self._last_tf_ok = False
        self._last_blocker = "NO_CAMERA_FRAME"
        self._last_source_status: dict = {}
        self._last_source_heartbeat: dict = {}
        self._last_sync_drops_total = 0
        self._last_valid_target: PointStamped | None = None
        self._last_world_xyz = None
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
        self.target_publisher = self.create_publisher(
            PointStamped, "/piperh/pulse/target", 10
        )
        self.marker_publisher = self.create_publisher(
            Marker, "/piperh/pulse/target_marker", 10
        )
        self.arm_direction_publisher = self.create_publisher(
            Vector3Stamped, "/piperh/pulse/arm_direction", 10
        )
        self.diagnostic_publisher = self.create_publisher(
            DiagnosticArray, "/piperh/pulse/target_diagnostics", 10
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

    def _lookup_transform(self, target_frame: str, source_frame: str, stamp):
        transform = self.tf_buffer.lookup_transform(
            target_frame, source_frame, stamp,
            timeout=Duration(seconds=self.tf_timeout_sec),
        )
        stamp_ns = (
            int(transform.header.stamp.sec) * 1_000_000_000
            + int(transform.header.stamp.nanosec)
        )
        self._last_tf_age_ms = (
            max(0.0, (self.get_clock().now().nanoseconds - stamp_ns) / 1_000_000.0)
            if stamp_ns > 0 else 0.0
        )
        return transform

    def _transform_point(self, message: PointStamped) -> PointStamped:
        transformed = message
        calibration = self.calibration_transform
        if calibration is not None:
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
            transformed = do_transform_vector3(transformed, calibration)
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
            "camera_xyz": (message.point.x, message.point.y, message.point.z),
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
            self._drop("TF_LOOKUP_FAILED", context, event_type="tf_failure")
            return
        except Exception as error:
            context["exception"] = repr(error)
            self._drop("INTERNAL_EXCEPTION", context, event_type="exception")
            self.get_logger().error(f"pulse target pipeline exception: {error}")
            return
        world_xyz = (target.point.x, target.point.y, target.point.z)
        context.update(tf_success=True, tf_age_ms=self._last_tf_age_ms, world_xyz=world_xyz)
        self._last_tf_ok = True
        self._counters.tf_success += 1
        if not all(math.isfinite(value) for value in world_xyz):
            self._drop("NONFINITE_TARGET", context)
            return
        minimum = tuple(float(value) for value in self.get_parameter("target_range_min_m").value)
        maximum = tuple(float(value) for value in self.get_parameter("target_range_max_m").value)
        if not all(low <= value <= high for value, low, high in zip(world_xyz, minimum, maximum)):
            context["allowed_range"] = {"minimum": minimum, "maximum": maximum}
            self._drop("TARGET_OUT_OF_RANGE", context)
            return
        try:
            self.target_publisher.publish(target)
        except Exception as error:
            context["exception"] = repr(error)
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
        context.update(
            publish_target=True,
            publish_seq=self._publish_seq,
            last_publish_interval_ms=(
                self._gap.last_interval_sec * 1000.0
                if self._gap.last_interval_sec is not None else None
            ),
            last_publish_age_ms=0.0,
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


def main() -> None:
    rclpy.init()
    node = PiperPulseTarget()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
