"""Bounded Piper-H pulse approach, with opt-in non-human dummy testing."""

from __future__ import annotations

from collections import deque
from datetime import datetime
from enum import Enum
import json
import math
import os
import sys
import time

from geometry_msgs.msg import (
    Point,
    PointStamped,
    Pose,
    PoseStamped,
    Quaternion,
    Vector3Stamped,
)
from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetCartesianPath, GetStateValidity
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from sensor_msgs.msg import FluidPressure
from tf2_geometry_msgs import do_transform_point, do_transform_vector3
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from rebotarm_moveit_demos.demo_common import MoveItDemoBase

from .approach_geometry import (
    horizontal_vertical_displacement,
    inside_workspace,
    norm,
    rotate_vector,
    subtract,
    translation_within_axis_limits,
)
from .pressure_safety import local_pressure_baseline
from .pressure_serial_owner import serial_device, serial_owner_pids
from .precontact_diagnostics import PlanDiagnostics
from .precontact_monitor import (
    DirectionStabilityFilter,
    PressureDebouncer,
    TargetStabilityFilter,
    target_gap_state,
)
from .sensor_axis_alignment import (
    SensorAxisAlignment,
    compute_sensor_axis_alignment,
    direction_is_fresh,
)
from .staged_precontact import (
    ApproachCandidate,
    approach_candidates,
    first_valid_candidate,
    make_staged_precontact,
)


class PlanningState(Enum):
    IDLE = "IDLE"
    WAITING_FOR_INPUT = "WAITING_FOR_INPUT"
    TARGET_TRACKING = "TARGET_TRACKING"
    TARGET_STABILIZING = "TARGET_STABILIZING"
    TARGET_STABLE = "TARGET_STABLE"
    TARGET_TEMPORARILY_STALE = "TARGET_TEMPORARILY_STALE"
    WAIT_TARGET_RECOVERY = "WAIT_TARGET_RECOVERY"
    ARM_AXIS_ALIGN = "ARM_AXIS_ALIGN"
    PRECONTACT_POSITIONING = "PRECONTACT_POSITIONING"
    PRECONTACT_READY = "PRECONTACT_READY"
    APPROACH_PLANNED = "APPROACH_PLANNED"
    PRECONTACT_EXECUTING = "PRECONTACT_EXECUTING"
    PRECONTACT_REACHED = "PRECONTACT_REACHED"
    DUMMY_TARGET_REACHED = "DUMMY_TARGET_REACHED"
    DUMMY_PRESSURE_STOPPED = "DUMMY_PRESSURE_STOPPED"
    ERROR = "ERROR"


class PiperPulseAlign(MoveItDemoBase):
    """Plan collision-checked lateral positioning followed by pre-contact approach."""

    def __init__(self) -> None:
        super().__init__("piper_pulse_align")
        self._cartesian = self.node.create_client(
            GetCartesianPath, "/compute_cartesian_path"
        )
        self._model_parameters = self.node.create_client(
            GetParameters, "/move_group/get_parameters"
        )
        self._state_validity = self.node.create_client(
            GetStateValidity, "/check_state_validity"
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self.node)
        self._latest_target: PointStamped | None = None
        self._target_received = 0.0
        self._last_target_acquisition_ns = 0
        self._target_frame = ""
        self._latest_arm_direction: Vector3Stamped | None = None
        self._arm_direction_received = 0.0
        self._arm_direction_frame = ""
        self._xbox_locked = False
        self._xbox_known = False
        self._selected_robot = ""
        self._serial_connected = False
        self._zero_calibrated = False
        self._pressure_pa: dict[str, float] = {}
        self._pressure_received: dict[str, float] = {}
        self._pressure_window = {
            channel: deque(maxlen=500) for channel in ("s1", "s2", "s3")
        }
        self._local_pressure_baseline: dict[str, float] | None = None
        self._planning_state = PlanningState.IDLE
        self._previous_state = PlanningState.IDLE
        self._state_changed_at = time.monotonic()
        self._primary_blocker = "WAIT_TARGET"
        self._blocking_reasons: list[str] = ["WAIT_TARGET"]
        self._gates: dict[str, dict] = {}
        self._context: dict = {}
        self._candidate_results: list[dict] = []
        self._attempt_invalidated = False
        self._target_lost_reset = False
        self._retry_count = 0
        self._last_heartbeat = 0.0
        self._target_filter = TargetStabilityFilter(
            window_sec=float(self._param("target_filter_window_sec")),
            min_samples=int(self._param("target_stable_min_samples")),
            stable_radius_m=float(self._param("target_stable_radius_m")),
            stable_duration_sec=float(self._param("target_stable_duration_sec")),
            soft_drift_m=float(self._param("target_soft_drift_m")),
            hard_drift_m=float(self._param("target_hard_drift_m")),
            hard_jump_m=float(self._param("target_jump_reject_m")),
            hard_drift_hold_sec=float(self._param("target_hard_drift_hold_sec")),
            ema_alpha=float(self._param("target_ema_alpha")),
            inlier_fraction=float(self._param("target_stable_inlier_fraction")),
            hard_timeout_sec=float(self._param("target_hard_timeout_sec")),
        )
        self._direction_filter = DirectionStabilityFilter(
            window_sec=float(self._param("arm_direction_filter_window_sec")),
            min_samples=int(self._param("arm_direction_stable_min_samples")),
            stable_deg=float(self._param("arm_direction_stable_deg")),
        )
        self._pressure_filters = {
            channel: PressureDebouncer(
                threshold_pa=float(self._param("pressure_precontact_delta_pa")),
                hold_sec=float(self._param("pressure_abort_hold_sec")),
                window_size=int(self._param("pressure_filter_window_samples")),
                emergency_threshold_pa=float(self._param("pressure_emergency_delta_pa")),
            ) for channel in ("s1", "s2", "s3")
        }
        log_directory = os.path.abspath(str(self._param("diagnostic_log_directory")))
        self._diagnostics = PlanDiagnostics(log_directory)
        self.node.create_subscription(
            PointStamped, str(self._param("target_topic")), self._target_cb,
            qos_profile_sensor_data,
        )
        self.node.create_subscription(
            Vector3Stamped, str(self._param("arm_direction_topic")),
            self._arm_direction_cb, qos_profile_sensor_data,
        )
        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.node.create_subscription(
            Bool, "/rebot_xbox/armed", self._xbox_cb, status_qos
        )
        self.node.create_subscription(
            String, "/dual_arm/selected", self._selected_cb, status_qos
        )
        self.node.create_subscription(
            Bool, "/piperh/pulse/serial_connected", self._serial_cb, status_qos
        )
        self.node.create_subscription(
            Bool, "/piperh/pulse/zero_calibrated", self._zero_cb, status_qos
        )
        for channel in ("s1", "s2", "s3"):
            self.node.create_subscription(
                FluidPressure, f"/piperh/pulse/zeroed/{channel}/pressure",
                lambda message, name=channel: self._pressure_cb(name, message),
                qos_profile_sensor_data,
            )
        self._precontact_publisher = self.node.create_publisher(
            PointStamped, "/piperh/pulse/precontact", 10
        )
        marker_qos = QoSProfile(depth=1)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._axis_marker_publisher = self.node.create_publisher(
            MarkerArray, "/piperh/pulse/axis_alignment_markers", marker_qos
        )
        self._staged_marker_publisher = self.node.create_publisher(
            MarkerArray, "/piperh/pulse/staged_precontact_markers", marker_qos
        )
        self._diagnostic_publisher = self.node.create_publisher(
            DiagnosticArray, "/piperh/pulse/precontact_diagnostics", 10
        )
        self.node.create_service(
            Trigger, "/piperh/pulse/dump_precontact_diagnostics", self._dump_diagnostics
        )
        heartbeat_hz = max(0.1, float(self._param("heartbeat_hz")))
        self._heartbeat_period = 1.0 / heartbeat_hz
        self.node.create_timer(self._heartbeat_period, self._heartbeat)
        self.node.get_logger().info(
            f"Piper-H pre-contact JSONL: {self._diagnostics.path}; "
            f"execute_motion={str(bool(self._param('execute_motion'))).lower()}"
        )

    def _transition(self, state: PlanningState, reason: str = "") -> None:
        previous = self._planning_state
        if previous == state:
            return
        self._previous_state = previous
        self._planning_state = state
        self._state_changed_at = time.monotonic()
        self.node.get_logger().info(
            f"[STATE] plan_id={self._diagnostics.plan_id} "
            f"{previous.value} -> {state.value} reason={reason or 'NONE'}"
        )
        self._emit("state_change", previous_state=previous.value, state=state.value,
                   reason=reason or "NONE", snapshot=self.build_diagnostic_snapshot())

    def _target_cb(self, message: PointStamped) -> None:
        if message.header.frame_id:
            acquisition_stamp_ns = (
                int(message.header.stamp.sec) * 1_000_000_000
                + int(message.header.stamp.nanosec)
            )
            acquisition_age = (
                (self.node.get_clock().now().nanoseconds - acquisition_stamp_ns)
                / 1_000_000_000.0
            )
            if (
                acquisition_stamp_ns <= 0
                or not math.isfinite(acquisition_age)
                or acquisition_age < -0.1
                or acquisition_age > float(self._param("target_hard_timeout_sec"))
            ):
                self._set_blocker(
                    "TARGET_STALE",
                    f"original acquisition age={acquisition_age * 1000.0:.0f}ms",
                )
                return
            if acquisition_stamp_ns <= self._last_target_acquisition_ns:
                self._set_blocker("TARGET_STALE", "out-of-order acquisition timestamp")
                return
            if self._target_frame and self._target_frame != message.header.frame_id:
                self._target_filter.reset("NEW_TARGET")
            now = time.monotonic()
            previous_age = now - self._target_received if self._target_received else None
            candidate_received = now - max(0.0, acquisition_age)
            try:
                status = self._target_filter.add(
                    candidate_received,
                    (float(message.point.x), float(message.point.y), float(message.point.z)),
                )
            except ValueError as error:
                self._set_blocker("TARGET_STALE", str(error))
                return
            self._target_frame = message.header.frame_id
            self._latest_target = message
            self._target_received = candidate_received
            self._last_target_acquisition_ns = acquisition_stamp_ns
            self._target_lost_reset = False
            if status.reacquire_result == "REACQUIRED_SAME_TARGET" and (
                previous_age is not None
                and previous_age > float(self._param("target_dropout_grace_sec"))
            ):
                self.node.get_logger().info(
                    "TARGET REACQUIRED "
                    f"gap={previous_age * 1000.0:.0f}ms "
                    f"distance_to_frozen={status.reacquire_distance_m * 1000.0:.1f}mm "
                    "result=REACQUIRED_SAME_TARGET stable_reset=NO"
                )
                self._emit(
                    "target_reacquired", gap_ms=previous_age * 1000.0,
                    reacquire_distance_mm=status.reacquire_distance_m * 1000.0,
                    reacquire_result="REACQUIRED_SAME_TARGET", stable_reset=False,
                )
            if status.hard_jump:
                if self._planning_state in (
                    PlanningState.ARM_AXIS_ALIGN,
                    PlanningState.PRECONTACT_POSITIONING,
                    PlanningState.PRECONTACT_READY,
                    PlanningState.PRECONTACT_EXECUTING,
                ):
                    self._attempt_invalidated = True
                elif self._diagnostics.plan_id:
                    self._new_plan("TARGET_HARD_JUMP")
                    self._transition(PlanningState.TARGET_TRACKING, "TARGET_HARD_JUMP")
                self._primary_blocker = "TARGET_HARD_JUMP"
                self._blocking_reasons = ["TARGET_HARD_JUMP"]
                self._emit("gate_snapshot", reason="TARGET_HARD_JUMP",
                           snapshot=self.build_diagnostic_snapshot())
            elif status.hard_drift:
                planning_active = self._planning_state in (
                    PlanningState.ARM_AXIS_ALIGN,
                    PlanningState.PRECONTACT_POSITIONING,
                    PlanningState.PRECONTACT_READY,
                    PlanningState.PRECONTACT_EXECUTING,
                )
                self._attempt_invalidated = planning_active
                if status.frozen is not None:
                    self._target_filter.restart_after_drift()
                if not planning_active and self._diagnostics.plan_id:
                    self._new_plan("TARGET_MOVED")
                self._transition(PlanningState.TARGET_TRACKING, "TARGET_MOVED")
                self._set_blocker("TARGET_MOVED", "filtered drift persisted above hard limit")
            elif status.reacquire_result == "OBSERVE_SPATIAL_DRIFT":
                self._emit(
                    "target_spatial_hysteresis",
                    jitter_p95_mm=status.jitter_m * 1000.0,
                    target_motion_mm=status.reacquire_distance_m * 1000.0,
                    stable_latched=status.stable_latched,
                    action="KEEP_STABLE_DURING_200MS_HYSTERESIS",
                )

    def _arm_direction_cb(self, message: Vector3Stamped) -> None:
        if message.header.frame_id:
            self._latest_arm_direction = message
            self._arm_direction_received = time.monotonic()
            self._arm_direction_frame = message.header.frame_id
            try:
                self._direction_filter.add(
                    self._arm_direction_received,
                    (float(message.vector.x), float(message.vector.y), float(message.vector.z)),
                )
            except ValueError:
                self._primary_blocker = "ARM_DIRECTION_UNSTABLE"

    def _xbox_cb(self, message: Bool) -> None:
        self._xbox_known = True
        self._xbox_locked = not bool(message.data)

    def _selected_cb(self, message: String) -> None:
        self._selected_robot = message.data.strip().lower()

    def _serial_cb(self, message: Bool) -> None:
        self._serial_connected = bool(message.data)

    def _zero_cb(self, message: Bool) -> None:
        self._zero_calibrated = bool(message.data)

    def _pressure_cb(self, channel: str, message: FluidPressure) -> None:
        pressure = float(message.fluid_pressure)
        received = time.monotonic()
        self._pressure_pa[channel] = pressure
        self._pressure_received[channel] = received
        self._pressure_window[channel].append((received, pressure))
        status = self._pressure_filters[channel].add(received, pressure)
        if status.abort:
            self._emit("pressure_event", channel=channel.upper(),
                       reason="PRESSURE_EMERGENCY_ABORT" if status.emergency_abort
                       else "PRESSURE_DELTA_ABORT",
                       pressure=self._pressure_snapshot(received))

    def _new_plan(self, reason: str) -> None:
        old_plan = self._diagnostics.plan_id
        plan_id = self._diagnostics.new_plan()
        self._candidate_results = []
        self._attempt_invalidated = False
        self._local_pressure_baseline = None
        for pressure_filter in self._pressure_filters.values():
            pressure_filter.set_baseline(None)
        self._context = {
            "motion_sent": False,
            "execute_motion": bool(self._param("execute_motion")),
        }
        self.node.get_logger().info(
            f"[PLAN] plan_id={plan_id} started reason={reason} previous_plan_id={old_plan or 'NONE'}"
        )
        self._emit("plan_started", reason=reason, previous_plan_id=old_plan or None)

    def _emit(self, event_type: str, **fields) -> None:
        try:
            self._diagnostics.write(event_type, **fields)
        except (OSError, TypeError, ValueError) as error:
            self.node.get_logger().error(f"diagnostic JSONL write failed: {error}")

    def _set_blocker(self, reason: str, details: str = "") -> None:
        changed = reason != self._primary_blocker
        self._primary_blocker = reason
        self._blocking_reasons = [reason + (f": {details}" if details else "")]
        if changed:
            self.node.get_logger().warn(
                f"[PRIMARY_BLOCKER] plan_id={self._diagnostics.plan_id} "
                f"PRIMARY_BLOCKER={reason} {details}"
            )
            self._emit("gate_snapshot", reason=reason,
                       snapshot=self.build_diagnostic_snapshot())

    def _gate(self, name: str, result: str, actual=None, threshold=None, detail: str = "") -> None:
        self._gates[name] = {
            "result": result, "actual": actual, "threshold": threshold, "detail": detail,
        }

    def _pressure_snapshot(self, now: float | None = None) -> dict:
        now = time.monotonic() if now is None else now
        result = {}
        for channel in ("s1", "s2", "s3"):
            status = self._pressure_filters[channel].status(now)
            result[channel.upper()] = {
                "raw_pa": status.raw_pa,
                "filtered_pa": status.filtered_pa,
                "baseline_pa": status.baseline_pa,
                "delta_pa": status.delta_pa,
                "age_ms": (
                    (now - self._pressure_received[channel]) * 1000.0
                    if channel in self._pressure_received else None
                ),
                "over_threshold_duration_ms": status.over_threshold_duration_sec * 1000.0,
                "abort": status.abort,
                "emergency_abort": status.emergency_abort,
            }
        deltas = [abs(value["delta_pa"]) for value in result.values()
                  if value["delta_pa"] is not None]
        result["max_delta_pa"] = max(deltas, default=None)
        return result

    def _pressure_graph_snapshot(self) -> dict:
        prefix = "/piperh/pulse"
        counts = {
            f"{kind}/{channel}/pressure": self.node.count_publishers(
                f"{prefix}/{kind}/{channel}/pressure"
            )
            for kind in ("raw", "zeroed") for channel in ("s1", "s2", "s3")
        }
        counts.update({
            f"raw/{channel}/temperature": self.node.count_publishers(
                f"{prefix}/raw/{channel}/temperature"
            )
            for channel in ("s1", "s2", "s3")
        })
        status_count = self.node.count_publishers(f"{prefix}/serial_connected")
        zero_count = self.node.count_publishers(f"{prefix}/zero_calibrated")
        device = serial_device(str(self._param("pressure_serial_device")))
        owners = serial_owner_pids(device)
        return {
            "pressure_status_publisher_count": status_count,
            "pressure_zero_status_publisher_count": zero_count,
            "pressure_data_publisher_count": max(counts.values(), default=0),
            "pressure_data_publisher_counts": counts,
            "pressure_source_unique": (
                status_count == 1 and zero_count == 1
                and all(count == 1 for count in counts.values())
            ),
            "serial_device": device,
            "serial_owner_pid": owners[0] if len(owners) == 1 else None,
            "serial_owner_pids": owners,
        }

    def build_diagnostic_snapshot(self) -> dict:
        now = time.monotonic()
        target = self._target_filter.status(now)
        target_age_sec = (
            now - self._target_received if self._latest_target is not None else None
        )
        gap_state = target_gap_state(
            target_age_sec,
            float(self._param("target_dropout_grace_sec")),
            float(self._param("target_soft_timeout_sec")),
            float(self._param("target_hard_timeout_sec")),
        )
        direction = self._direction_filter.status()
        pressure = self._pressure_snapshot(now)
        pressure_graph = self._pressure_graph_snapshot()
        joints = {
            f"j{index}": self._latest_joint_positions.get(name)
            for index, name in enumerate(self.joint_names, start=1)
        }
        snapshot = {
            "timestamp": datetime.now().astimezone().isoformat(timespec="milliseconds"),
            "plan_id": self._diagnostics.plan_id,
            "state": self._planning_state.value,
            "previous_state": self._previous_state.value,
            "state_duration_ms": (now - self._state_changed_at) * 1000.0,
            "blocking_reasons": list(self._blocking_reasons),
            "primary_blocker": self._primary_blocker,
            "target_received": self._latest_target is not None,
            "target_age_ms": target_age_sec * 1000.0 if target_age_sec is not None else None,
            "last_valid_target_age_ms": (
                target_age_sec * 1000.0 if target_age_sec is not None else None
            ),
            "dropout_grace_ms": float(self._param("target_dropout_grace_sec")) * 1000.0,
            "target_gap_state": gap_state,
            "temporary_stale": gap_state in (
                "TARGET_TEMPORARILY_STALE", "WAIT_TARGET_RECOVERY"
            ),
            "stable_latched": target.stable_latched,
            "frozen_target": target.frozen,
            "reacquire_distance_mm": (
                target.reacquire_distance_m * 1000.0
                if target.reacquire_distance_m is not None else None
            ),
            "reacquire_result": target.reacquire_result,
            "stable_reset_count": target.stable_reset_count,
            "stable_reset_reason": target.stable_reset_reason,
            "target_sample_count": target.sample_count,
            "target_raw_xyz": target.raw,
            "target_filtered_xyz": target.filtered,
            "target_frozen_xyz": target.frozen,
            "target_jitter_mm": target.jitter_m * 1000.0,
            "target_drift_mm": target.drift_m * 1000.0,
            "target_stable_duration_ms": target.stable_duration_sec * 1000.0,
            "target_stable": target.stable,
            "acquisition_state": target.acquisition_state,
            "window_oldest_age_ms": (
                target.window_oldest_age_sec * 1000.0
                if target.window_oldest_age_sec is not None else None
            ),
            "window_valid_samples": target.sample_count,
            "window_center_xyz": target.window_center,
            "jitter_p95_mm": target.jitter_m * 1000.0,
            "oldest_sample_distance_mm": target.oldest_sample_distance_m * 1000.0,
            "target_motion_mm": target.drift_m * 1000.0,
            "arm_direction_received": self._latest_arm_direction is not None,
            "arm_direction_age_ms": ((now - self._arm_direction_received) * 1000.0
                                     if self._latest_arm_direction is not None else None),
            "arm_direction_raw": direction.raw,
            "arm_direction_filtered": direction.filtered,
            "arm_direction_jitter_deg": direction.jitter_deg,
            "arm_direction_sample_count": direction.sample_count,
            "arm_direction_min_samples": int(self._param("arm_direction_stable_min_samples")),
            "arm_direction_window_ms": float(self._param("arm_direction_filter_window_sec")) * 1000.0,
            "arm_direction_required_duration_ms": 0.0,
            "selected_arm": self._selected_robot,
            "xbox_locked": self._xbox_locked if self._xbox_known else None,
            "joint_feedback_age_ms": ((now - self._latest_joint_state_time) * 1000.0
                                      if self._latest_joint_state_time else None),
            "joint_positions_rad": joints,
            "planning_frame": str(self._param("planning_frame")),
            "target_source_frame": self._target_frame,
            "arm_direction_source_frame": self._arm_direction_frame,
            "Link6_frame": str(self._param("tool_frame")),
            "pressure": pressure,
            "pressure_delta_pa": pressure.get("max_delta_pa"),
            "serial_connected": self._serial_connected,
            "pressure_zero_reported": self._zero_calibrated,
            "local_pressure_baseline_established": self._local_pressure_baseline is not None,
            "local_pressure_baseline_pa": self._local_pressure_baseline,
            "last_pressure_age_ms": max(
                (pressure[channel]["age_ms"] for channel in ("S1", "S2", "S3")
                 if pressure[channel]["age_ms"] is not None), default=None
            ),
            **pressure_graph,
            "gates": dict(self._gates),
            "candidate_results": list(self._candidate_results),
            **self._context,
        }
        return snapshot

    def _publish_diagnostics(self, snapshot: dict) -> None:
        status = DiagnosticStatus()
        status.name = "Piper-H pre-contact planner"
        status.hardware_id = "piperh"
        ready = snapshot["state"] in (
            PlanningState.APPROACH_PLANNED.value,
            PlanningState.PRECONTACT_EXECUTING.value,
            PlanningState.PRECONTACT_REACHED.value,
        )
        status.level = DiagnosticStatus.OK if ready else DiagnosticStatus.WARN
        status.message = "READY" if ready else self._primary_blocker
        keys = (
            "state", "primary_blocker", "target_age_ms", "target_jitter_mm",
            "target_drift_mm", "clearance_mm", "lateral_distance_mm",
            "pressure_delta_pa", "candidate_angle_deg", "cartesian_fraction",
            "pressure_status_publisher_count", "pressure_data_publisher_count",
            "pressure_data_publisher_counts", "serial_device", "serial_owner_pid",
            "serial_connected", "last_pressure_age_ms",
            "local_pressure_baseline_established", "local_pressure_baseline_pa",
            "arm_direction_jitter_deg", "arm_direction_sample_count",
            "arm_direction_min_samples", "arm_direction_window_ms",
            "arm_direction_required_duration_ms",
            "acquisition_state", "window_oldest_age_ms", "window_valid_samples",
            "window_center_xyz", "jitter_p95_mm", "oldest_sample_distance_mm",
            "target_motion_mm",
        )
        values = {"ready": ready, **snapshot}
        status.values = [KeyValue(key=key, value=str(values.get(key)))
                         for key in ("ready",) + keys]
        message = DiagnosticArray()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.status = [status]
        self._diagnostic_publisher.publish(message)

    def _heartbeat(self) -> None:
        now = time.monotonic()
        if now - self._last_heartbeat < self._heartbeat_period * 0.95:
            return
        self._last_heartbeat = now
        if self._latest_target is not None:
            age = now - self._target_received
            gap_state = target_gap_state(
                age,
                float(self._param("target_dropout_grace_sec")),
                float(self._param("target_soft_timeout_sec")),
                float(self._param("target_hard_timeout_sec")),
            )
            if gap_state == "TARGET_LOST" and not self._target_lost_reset:
                self._target_filter.reset("TARGET_HARD_TIMEOUT")
                self._target_lost_reset = True
                if (
                    bool(self._param("dummy_contact_mode"))
                    and self._planning_state == PlanningState.PRECONTACT_EXECUTING
                ):
                    # The accepted trajectory already contains the frozen goal.
                    # Camera occlusion during dummy execution is diagnostic only.
                    self._emit("dummy_target_dropout", reason="FROZEN_GOAL_CONTINUES")
                else:
                    self._attempt_invalidated = True
                    self._transition(PlanningState.TARGET_TRACKING, "TARGET_LOST")
                    self._set_blocker("TARGET_LOST", "target hard timeout; frozen target cleared")
        snapshot = self.build_diagnostic_snapshot()
        self._publish_diagnostics(snapshot)
        self._emit("heartbeat", snapshot=snapshot)
        self.node.get_logger().info(
            "[PULSE PRECONTACT] "
            f"plan_id={self._diagnostics.plan_id or 'NONE'} state={snapshot['state']} "
            f"VISION={'PASS' if snapshot['target_stable'] else 'WAIT'} "
            f"jitter={snapshot['target_jitter_mm']:.1f}/"
            f"{float(self._param('target_stable_radius_m')) * 1000.0:.1f}mm "
            f"motion={snapshot['target_motion_mm']:.1f}mm "
            f"TARGET_AGE={snapshot['target_gap_state']} {snapshot['target_age_ms']}ms "
            f"target_gap_state={snapshot['target_gap_state']} "
            f"stable_latched={snapshot['stable_latched']} "
            f"clearance={snapshot.get('clearance_mm')}mm "
            f"pressure_delta={snapshot['pressure'].get('max_delta_pa')}Pa "
            f"blocker={self._primary_blocker}"
        )
        gates = snapshot.get("gates", {})
        pressure_pass = all(
            gates.get(name, {}).get("result") == "PASS"
            for name in ("pressure_source_unique", "pressure_connected",
                         "pressure_zeroed", "pressure_fresh")
        )
        target_age_pass = snapshot["target_gap_state"] == "TARGET_STABLE"
        self.node.get_logger().info(
            "[PULSE GATES] "
            f"VISION: {'PASS' if snapshot['target_stable'] else 'WAIT'} "
            f"jitter={snapshot['target_jitter_mm']:.1f}/"
            f"{float(self._param('target_stable_radius_m')) * 1000.0:.1f}mm; "
            f"TARGET AGE: {'PASS' if target_age_pass else 'WAIT'} "
            f"{snapshot['target_age_ms']}ms; "
            f"PRESSURE: {'PASS' if pressure_pass else 'FAIL'} "
            f"connected={snapshot['serial_connected']}; "
            f"IK: {gates.get('ik_hover', {}).get('result', 'NOT_CHECKED')}"
        )

    def _spin_inputs(self, timeout_sec: float) -> None:
        """Service a burst of queued sensor callbacks without FPS starvation.

        The planner subscribes to target, arm direction, three pressure streams,
        joint states, ownership and status topics. Processing only one callback
        before each expensive diagnostic graph snapshot can starve the target
        queue even when its publisher is running at 20-30 Hz. Drain the ready
        queue after the first bounded wait so the 0.6 s spatial window contains
        the real samples that have already arrived.
        """
        rclpy.spin_once(self.node, timeout_sec=max(0.0, timeout_sec))
        for _ in range(32):
            rclpy.spin_once(self.node, timeout_sec=0.0)

    def _wait_for_target_recovery(self) -> bool:
        """Do not create a new geometry/IK/Cartesian plan from stale vision."""
        deadline = time.monotonic() + float(self._param("target_timeout_sec"))
        previous_state = self._planning_state
        while rclpy.ok() and time.monotonic() < deadline:
            now = time.monotonic()
            age = now - self._target_received if self._latest_target is not None else None
            gap_state = target_gap_state(
                age,
                float(self._param("target_dropout_grace_sec")),
                float(self._param("target_soft_timeout_sec")),
                float(self._param("target_hard_timeout_sec")),
            )
            target = self._target_filter.status(now)
            if gap_state == "TARGET_LOST":
                self._target_filter.reset("TARGET_HARD_TIMEOUT")
                self._target_lost_reset = True
                self._attempt_invalidated = True
                self._transition(PlanningState.TARGET_TRACKING, "TARGET_LOST")
                self._set_blocker("TARGET_LOST")
                return False
            if self._attempt_invalidated:
                self._set_blocker("TARGET_MOVED")
                return False
            if gap_state == "TARGET_STABLE" and target.stable:
                if self._planning_state in (
                    PlanningState.TARGET_TEMPORARILY_STALE,
                    PlanningState.WAIT_TARGET_RECOVERY,
                ):
                    self._transition(previous_state, "REACQUIRED_SAME_TARGET")
                if self._primary_blocker in (
                    "TARGET_TEMPORARILY_STALE", "WAIT_TARGET_RECOVERY", "WAIT_TARGET"
                ):
                    self._set_blocker("NONE")
                return True
            if gap_state == "TARGET_TEMPORARILY_STALE":
                self._transition(PlanningState.TARGET_TEMPORARILY_STALE, gap_state)
                self._set_blocker(gap_state)
            elif gap_state == "WAIT_TARGET_RECOVERY":
                self._transition(PlanningState.WAIT_TARGET_RECOVERY, gap_state)
                self._set_blocker(gap_state)
            elif not target.stable:
                self._set_blocker("TARGET_STABILIZING")
            else:
                self._set_blocker("WAIT_TARGET")
            self._spin_inputs(0.05)
        self._set_blocker("WAIT_TARGET")
        return False

    def _dump_diagnostics(self, _request, response):
        snapshot = self.build_diagnostic_snapshot()
        self._emit("gate_snapshot", requested_dump=True, snapshot=snapshot)
        self.node.get_logger().info(
            "[DIAGNOSTIC_DUMP] " + json.dumps(snapshot, ensure_ascii=False, default=str)
        )
        response.success = True
        response.message = f"saved snapshot to {self._diagnostics.path}"
        return response

    def _pressure_safe(self) -> bool:
        maximum_age = float(self._param("maximum_pressure_age_sec"))
        now = time.monotonic()
        return self._serial_connected and self._zero_calibrated and self._pressure_graph_snapshot()[
            "pressure_source_unique"
        ] and all(
            channel in self._pressure_pa
            and now - self._pressure_received[channel] <= maximum_age
            and not self._pressure_filters[channel].status(now).abort
            for channel in ("s1", "s2", "s3")
        )

    def _capture_local_pressure_baseline(self) -> bool:
        window_sec = float(self._param("local_baseline_window_sec"))
        minimum_samples = int(self._param("local_baseline_min_samples"))
        maximum_std = float(self._param("local_baseline_max_std_pa"))
        deadline = time.monotonic() + max(window_sec + 0.5, 1.0)
        while rclpy.ok() and time.monotonic() < deadline:
            now = time.monotonic()
            recent_counts = [
                sum(now - stamp <= window_sec for stamp, _value in samples)
                for samples in self._pressure_window.values()
            ]
            if min(recent_counts, default=0) >= minimum_samples:
                break
            self._spin_inputs(0.02)
        now = time.monotonic()
        baseline = {}
        for channel, samples in self._pressure_window.items():
            try:
                baseline[channel] = local_pressure_baseline(
                    list(samples), now, window_sec, minimum_samples, maximum_std
                )
            except ValueError as error:
                self._gate("local_pressure_baseline", "FAIL", str(error),
                           "stable non-contact window")
                self.node.get_logger().error(
                    f"{channel} local pressure baseline failed: {error}"
                )
                return False
        self._local_pressure_baseline = baseline
        for channel, value in baseline.items():
            self._pressure_filters[channel].set_baseline(value)
        self._gate("local_pressure_baseline", "PASS", baseline,
                   "stable non-contact window")
        self._gate("pressure_stable", "PASS", 0.0,
                   {"delta_pa": float(self._param("pressure_precontact_delta_pa")),
                    "hold_ms": float(self._param("pressure_abort_hold_sec")) * 1000.0,
                    "emergency_delta_pa": float(self._param("pressure_emergency_delta_pa"))},
                   "local baseline established; pressure delta now evaluated")
        self.node.get_logger().info(
            "local non-contact baselines (Pa): "
            + ", ".join(f"{name}={value:.1f}" for name, value in baseline.items())
        )
        return True

    def run(self) -> bool:
        execute_motion = bool(self._param("execute_motion"))
        dummy_mode = bool(self._param("dummy_contact_mode"))
        goal_standoff_m = 0.0 if dummy_mode else float(self._param("standoff_m"))
        horizontal_limit_m = min(float(self._param("maximum_horizontal_translation_m")), 0.300)
        vertical_limit_m = min(float(self._param("maximum_vertical_translation_m")), 0.200)
        self._new_plan("USER_REQUEST" if self._retry_count == 0 else "TARGET_REPLANNING")
        if not self._cartesian.wait_for_service(timeout_sec=10.0):
            self.node.get_logger().error("MoveIt /compute_cartesian_path is unavailable")
            self._set_blocker("INTERNAL_ERROR", "/compute_cartesian_path unavailable")
            self._summary(False, "INTERNAL_ERROR")
            return False
        if not self.wait_for_ik_service():
            self._set_blocker("INTERNAL_ERROR", "/compute_ik unavailable")
            self._summary(False, "INTERNAL_ERROR")
            return False
        if execute_motion and not self.wait_for_execute_server():
            self._set_blocker("EXECUTE_ACTION_UNAVAILABLE", "/execute_trajectory unavailable")
            self._summary(False, "EXECUTE_ACTION_UNAVAILABLE")
            return False
        if not self._state_validity.wait_for_service(timeout_sec=10.0):
            self.node.get_logger().error("MoveIt /check_state_validity is unavailable")
            self._set_blocker("INTERNAL_ERROR", "/check_state_validity unavailable")
            self._summary(False, "INTERNAL_ERROR")
            return False
        if not self._tool_collision_model_loaded():
            self._set_blocker("TOOL_ENVELOPE_COLLISION", "collision model unavailable")
            self._summary(False, "TOOL_ENVELOPE_COLLISION")
            return False
        if not self._wait_for_preflight():
            self._summary(False, self._primary_blocker)
            return False
        if not self._wait_for_target_recovery():
            self._summary(False, self._primary_blocker)
            return False
        current = self.fresh_current_joint_values()
        if current is None:
            self._set_blocker("JOINT_FEEDBACK_STALE")
            self._summary(False, "JOINT_FEEDBACK_STALE")
            return False
        target = self._frozen_target_in_planning_frame()
        transform = self._current_tool_transform()
        if target is None or transform is None:
            self._set_blocker("TF_LOOKUP_FAILED")
            self._summary(False, "TF_LOOKUP_FAILED")
            return False
        frozen_target = (target.point.x, target.point.y, target.point.z)
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        current_link_xyz = (translation.x, translation.y, translation.z)
        current_quaternion = (rotation.x, rotation.y, rotation.z, rotation.w)
        if not self._wait_for_target_recovery():
            self._summary(False, self._primary_blocker)
            return False
        try:
            plan = make_staged_precontact(
                current_link_xyz,
                current_quaternion,
                frozen_target,
                goal_standoff_m,
                float(self._param("tip_offset_m")),
                horizontal_limit_m,
                vertical_limit_m,
                float(self._param("lateral_deadband_m")),
                allow_zero_standoff=dummy_mode,
            )
        except ValueError as error:
            reason = "POSITION_CORRECTION_TOO_LARGE"
            self._set_blocker(reason, str(error))
            self.node.get_logger().error(f"{reason}: {error}")
            self._summary(False, reason)
            return False
        if dummy_mode and plan.clearance_m < 0.0:
            self._set_blocker("TARGET_ALREADY_PASSED", "probe tip is beyond the visual target")
            self._summary(False, "TARGET_ALREADY_PASSED")
            return False
        if not self._capture_local_pressure_baseline():
            self._set_blocker("PRESSURE_UNSTABLE")
            self._summary(False, "PRESSURE_UNSTABLE")
            return False
        if not self._wait_for_target_recovery():
            self._summary(False, self._primary_blocker)
            return False
        self._transition(PlanningState.ARM_AXIS_ALIGN, "INPUT_GATES_PASS")
        self._run_axis_alignment_dry_run(
            current_quaternion, current[-1], target
        )
        arm_direction, arm_reason = self._arm_direction_in_planning_frame()
        if arm_direction is None:
            self._set_blocker("ARM_DIRECTION_STALE", arm_reason)
            self._transition(PlanningState.WAITING_FOR_INPUT, "ARM_DIRECTION_STALE")
            self._summary(False, "ARM_DIRECTION_STALE")
            return False
        self._record_geometry(
            plan, current_link_xyz, current_quaternion, frozen_target, arm_direction
        )
        sensor_axis = rotate_vector(
            current_quaternion,
            tuple(float(value) for value in self._param("sensor_array_axis_local")),
        )
        limits = tuple(float(value) for value in self._param("workspace_limits"))
        correction_points = (plan.hover_link, plan.nominal_precontact_link)
        if not all(
            translation_within_axis_limits(
                point,
                current_link_xyz,
                horizontal_limit_m,
                vertical_limit_m,
            )
            for point in correction_points
        ):
            self._set_blocker("POSITION_CORRECTION_TOO_LARGE")
            self._summary(False, "POSITION_CORRECTION_TOO_LARGE")
            return False
        if not all(inside_workspace(point, limits) for point in (
            plan.hover_link, plan.hover_tip,
            plan.nominal_precontact_link, plan.nominal_precontact_tip,
        )):
            self._set_blocker("HOVER_OUT_OF_WORKSPACE")
            self.node.get_logger().error("HOVER_OUT_OF_WORKSPACE: Piper-H pre-contact pose is outside workspace limits")
            self._summary(False, "HOVER_OUT_OF_WORKSPACE")
            return False
        self._gate(
            "workspace", "PASS",
            [plan.hover_link, plan.nominal_precontact_link],
            limits,
        )
        if not self._wait_for_target_recovery():
            self._summary(False, self._primary_blocker)
            return False
        self._transition(PlanningState.PRECONTACT_POSITIONING, "GEOMETRY_PASS")
        direct_precontact = (
            plan.clearance_m < goal_standoff_m
            or norm(plan.lateral_offset) <= float(self._param("lateral_deadband_m"))
        )
        hover_joints = current
        pre_hover_waypoints = ()
        if direct_precontact:
            self._gate("ik_hover", "NOT_APPLICABLE", None, "direct Cartesian path")
            self._gate("collision_hover", "NOT_APPLICABLE", None, "direct Cartesian path")
        else:
            hover_pose = self._pose(plan.hover_link, plan.orientation)
            hover_joints = self._valid_ik(
                hover_pose, current, "current-clearance lateral waypoint"
            )
            if hover_joints is None:
                reason = self._context.get("last_ik_reason", "HOVER_IK_FAILED")
                reason = "HOVER_COLLISION" if reason == "COLLISION" else "HOVER_IK_FAILED"
                self._set_blocker(reason)
                self._transition(PlanningState.ERROR)
                self._summary(False, reason)
                return False
            pre_hover_waypoints = (hover_pose.pose,)
            self._gate("ik_hover", "PASS", hover_joints, "valid joint-limit solution")
            self._gate("collision_hover", "PASS", "collision-free", "required")
        self._transition(
            PlanningState.PRECONTACT_READY,
            "DIRECT_PATH_READY" if direct_precontact else "HOVER_VALID",
        )
        candidates = approach_candidates(
            plan, arm_direction,
            float(self._param("maximum_approach_deviation_deg")),
            float(self._param("maximum_approach_tangent_shift_m")),
        )
        if dummy_mode:
            # Only the nominal candidate ends exactly at the visual target.
            candidates = candidates[:1]
        selected_trajectory = None

        def validate(candidate: ApproachCandidate) -> bool:
            nonlocal selected_trajectory
            result = {
                "angle_offset_deg": candidate.angle_deg,
                "approach_direction": candidate.direction,
                "tangential_shift_mm": candidate.tangent_shift_m * 1000.0,
                "candidate_tip_xyz": candidate.tip,
                "candidate_Link6_xyz": candidate.link,
                "workspace": "NOT_CHECKED", "ik": "NOT_CHECKED",
                "joint_limit": "NOT_CHECKED", "robot_collision": "NOT_CHECKED",
                "tool_envelope_collision": "NOT_CHECKED",
                "cartesian": {}, "result": "FAIL", "reason": "INTERNAL_ERROR",
            }
            if not self._wait_for_target_recovery():
                result.update(reason=self._primary_blocker)
                self._candidate_result(result)
                return False
            if not translation_within_axis_limits(
                candidate.link,
                current_link_xyz,
                horizontal_limit_m,
                vertical_limit_m,
            ):
                result.update(reason="POSITION_CORRECTION_TOO_LARGE", workspace="FAIL")
                self._candidate_result(result)
                return False
            if not inside_workspace(candidate.link, limits) or not inside_workspace(
                candidate.tip, limits
            ):
                result.update(reason="PRECONTACT_OUT_OF_WORKSPACE", workspace="FAIL")
                self._candidate_result(result)
                return False
            result["workspace"] = "PASS"
            pose = self._pose(candidate.link, plan.orientation)
            goal_joints = self._valid_ik(
                pose, hover_joints,
                f"approach candidate {candidate.angle_deg:.1f} deg",
                warn_only=True,
            )
            if goal_joints is None:
                collision = self._context.get("last_ik_reason") == "COLLISION"
                result.update(
                    reason="PRECONTACT_COLLISION" if collision else "PRECONTACT_IK_FAILED",
                    ik="PASS" if collision else "FAIL",
                    joint_limit="PASS" if collision else "NOT_CHECKED",
                    robot_collision="FAIL" if collision else "NOT_CHECKED",
                    tool_envelope_collision=("FAIL" if collision and self._context.get("last_collision_has_tool")
                                             else "PASS" if collision else "NOT_CHECKED"),
                    ik_diagnostic=self._context.get("last_ik_diagnostic"),
                )
                self._candidate_result(result)
                self.node.get_logger().warn(
                    f"approach candidate rejected: angle={candidate.angle_deg:.1f} deg; "
                    "IK/collision validity failed"
                )
                return False
            result.update(ik="PASS", joint_limit="PASS", robot_collision="PASS",
                          tool_envelope_collision="PASS", ik_solution_rad=goal_joints,
                          ik_diagnostic=self._context.get("last_ik_diagnostic"))
            hover_cartesian = {}
            approach_cartesian = {}
            if pre_hover_waypoints:
                self._plan_cartesian(current, pre_hover_waypoints, warn_only=True)
                hover_cartesian = dict(self._context.get("last_cartesian", {}))
                self._plan_cartesian(hover_joints, (pose.pose,), warn_only=True)
                approach_cartesian = dict(self._context.get("last_cartesian", {}))
            trajectory = self._plan_cartesian(
                current, pre_hover_waypoints + (pose.pose,), warn_only=True
            )
            combined_cartesian = dict(self._context.get("last_cartesian", {}))
            result["cartesian"] = {
                "current_to_hover_fraction": hover_cartesian.get("fraction"),
                "hover_to_precontact_fraction": approach_cartesian.get("fraction"),
                "combined_fraction": combined_cartesian.get("fraction", 0.0),
                "eef_step_m": float(self._param("cartesian_step_m")),
                "jump_threshold": float(self._param("jump_threshold")),
                "waypoint_count": len(pre_hover_waypoints) + 1,
                "failed_segment": (
                    "DIRECT_TO_PRECONTACT" if not pre_hover_waypoints and combined_cartesian.get("fraction", 0.0) < 0.999
                    else "CURRENT_TO_HOVER" if pre_hover_waypoints and hover_cartesian.get("fraction", 0.0) < 0.999
                    else "HOVER_TO_PRECONTACT" if approach_cartesian.get("fraction", 0.0) < 0.999
                    else "COMBINED" if combined_cartesian.get("fraction", 0.0) < 0.999 else None
                ),
            }
            if trajectory is None:
                result.update(reason="CARTESIAN_PATH_INCOMPLETE")
                self._candidate_result(result)
                self.node.get_logger().warn(
                    f"approach candidate rejected: angle={candidate.angle_deg:.1f} deg; "
                    "collision-free Cartesian path unavailable"
                )
                return False
            selected_trajectory = trajectory
            result.update(result="PASS", reason="NONE")
            self._candidate_result(result)
            return True

        selected = first_valid_candidate(candidates, validate)
        if selected is None or selected_trajectory is None:
            if self._primary_blocker in (
                "TARGET_LOST", "TARGET_MOVED", "TARGET_TEMPORARILY_STALE",
                "WAIT_TARGET_RECOVERY", "WAIT_TARGET",
            ):
                self._summary(False, self._primary_blocker)
                return False
            self.node.get_logger().error(
                "all nominal and small-deviation approach candidates failed IK/collision checks"
            )
            self._set_blocker("NO_VALID_APPROACH_CANDIDATE")
            self._transition(PlanningState.ERROR)
            self._summary(False, "NO_VALID_APPROACH_CANDIDATE")
            return False
        for candidate in candidates[len(self._candidate_results):]:
            self._candidate_result({
                "angle_offset_deg": candidate.angle_deg,
                "approach_direction": candidate.direction,
                "tangential_shift_mm": candidate.tangent_shift_m * 1000.0,
                "candidate_tip_xyz": candidate.tip,
                "candidate_Link6_xyz": candidate.link,
                "workspace": "NOT_CHECKED", "ik": "NOT_CHECKED",
                "joint_limit": "NOT_CHECKED", "robot_collision": "NOT_CHECKED",
                "tool_envelope_collision": "NOT_CHECKED", "cartesian": {},
                "result": "NOT_ATTEMPTED", "reason": "EARLIER_CANDIDATE_SELECTED",
            })
        self._gate("ik_precontact", "PASS", "selected candidate", "valid solution")
        self._gate("collision_precontact", "PASS", "collision-free", "required")
        self._gate("cartesian_path", "PASS", 1.0, 1.0)
        precontact = PointStamped()
        precontact.header = target.header
        precontact.point = Point(x=selected.tip[0], y=selected.tip[1], z=selected.tip[2])
        if not self._wait_for_target_recovery():
            self._summary(False, self._primary_blocker)
            return False
        if not dummy_mode:
            self._precontact_publisher.publish(precontact)
        self.node.get_logger().info(
            ("DUMMY_TARGET_POSITIONING: " if dummy_mode else "PRECONTACT_POSITIONING: ") +
            f"current_tip={plan.current_tip}, target={plan.target}, "
            f"lateral_offset={plan.lateral_offset}, "
            f"height/approach_offset={plan.clearance_m:.4f} m, "
            f"arm_axis={arm_direction}, sensor_array_axis={sensor_axis}, "
            f"nominal_approach={plan.nominal_approach}, "
            f"planned_goal={selected.tip}, IK_valid=true, collision_valid=true"
        )
        self.node.get_logger().info(
            f"approach candidate selected: deviation={selected.angle_deg:.1f} deg, "
            f"tangent_shift={selected.tangent_shift_m * 1000.0:.1f} mm; "
            "bounded correction="
            f"XY {self._context['horizontal_position_correction_mm']:.1f} mm / "
            f"Z {self._context['vertical_position_correction_mm']:.1f} mm"
        )
        self._publish_staged_markers(plan, selected, target.header)
        if not self._pre_execution_safe(frozen_target):
            self._transition(PlanningState.TARGET_TRACKING, self._primary_blocker)
            self._summary(False, self._primary_blocker)
            if (
                self._primary_blocker in (
                    "TARGET_MOVED", "TARGET_HARD_JUMP", "TARGET_LOST"
                )
                and self._retry_count < int(self._param("maximum_replan_attempts"))
            ):
                self._retry_count += 1
                return self.run()
            return False
        self.preview_trajectory(selected_trajectory, current)
        self._context["candidate_angle_deg"] = selected.angle_deg
        self._context["cartesian_fraction"] = 1.0
        self._context["precontact_tip_xyz"] = selected.tip
        self._context["goal_tip_xyz"] = selected.tip
        self._primary_blocker = "NONE"
        self._blocking_reasons = []
        self._transition(PlanningState.APPROACH_PLANNED, "SAFE_PLAN_COMPLETE")
        if not execute_motion:
            self.node.get_logger().info(
                "pulse approach plan complete; execute_motion=false, so no trajectory was sent"
            )
            self._summary(True, "NONE")
            return True

        # preview_trajectory intentionally creates a cancellation window. Inputs may
        # change during it, so every live hard gate is checked again immediately
        # before the ExecuteTrajectory goal is submitted.
        if not self._pre_execution_safe(frozen_target):
            self._transition(PlanningState.TARGET_TRACKING, self._primary_blocker)
            self._summary(False, self._primary_blocker)
            return False
        self._context["motion_sent"] = True
        self._transition(PlanningState.PRECONTACT_EXECUTING, "ALL_HARD_GATES_PASS")
        self.node.get_logger().warn(
            "DUMMY-ONLY: executing collision-checked path no farther than the visual target; "
            "pressure anomaly cancels motion; no force control is available"
            if dummy_mode else
            "executing collision-checked pre-contact path with live pressure cancellation"
        )
        execution_timeout = (
            min(float(self._param("dummy_execution_timeout_sec")), 90.0)
            if dummy_mode else float(self._param("result_timeout"))
        )
        if not self._execute_trajectory_with_pressure_monitor(
            selected_trajectory, execution_timeout
        ):
            if dummy_mode and self._context.get("pressure_stop_confirmed"):
                self._transition(PlanningState.DUMMY_PRESSURE_STOPPED, self._primary_blocker)
                self._summary(True, self._primary_blocker)
                return True
            if self._primary_blocker == "NONE":
                self._set_blocker("TRAJECTORY_EXECUTION_FAILED")
            self._transition(PlanningState.ERROR, self._primary_blocker)
            self._summary(False, self._primary_blocker)
            return False
        self._primary_blocker = "NONE"
        self._blocking_reasons = []
        self._transition(
            PlanningState.DUMMY_TARGET_REACHED if dummy_mode else PlanningState.PRECONTACT_REACHED,
            "EXECUTION_SUCCESS",
        )
        self.node.get_logger().info(
            "DUMMY_TARGET_REACHED: planned visual endpoint reached without pressure trigger"
            if dummy_mode else
            "PRECONTACT_REACHED: Piper-H stopped at the configured standoff"
        )
        self._summary(True, "NONE")
        return True

    def _record_geometry(
        self, plan, current_link_xyz, current_quaternion, target, arm_direction
    ) -> None:
        target_minus_tip = subtract(target, plan.current_tip)
        lateral_distance = norm(plan.lateral_offset)
        horizontal_correction, vertical_correction = horizontal_vertical_displacement(
            plan.nominal_precontact_link, current_link_xyz
        )
        tool_z = plan.nominal_approach
        direction = arm_direction
        angle = None
        if direction is not None:
            cosine = max(-1.0, min(1.0, sum(a * b for a, b in zip(tool_z, direction))))
            angle = math.degrees(math.acos(cosine))
        self._context.update({
            "Link6_position_xyz": current_link_xyz,
            "Link6_orientation_xyzw": current_quaternion,
            "Link6_local_plus_z_planning_frame": tool_z,
            "nominal_approach_xyz": plan.nominal_approach,
            "current_tip_xyz": plan.current_tip,
            "target_xyz": target,
            "target_minus_tip_xyz": target_minus_tip,
            "clearance_mm": plan.clearance_m * 1000.0,
            "required_standoff_mm": (
                0.0 if bool(self._param("dummy_contact_mode"))
                else float(self._param("standoff_m")) * 1000.0
            ),
            "lateral_offset_xyz": plan.lateral_offset,
            "lateral_distance_mm": lateral_distance * 1000.0,
            "total_position_correction_mm": plan.correction_m * 1000.0,
            "horizontal_position_correction_mm": horizontal_correction * 1000.0,
            "vertical_position_correction_mm": vertical_correction * 1000.0,
            "max_horizontal_position_correction_mm": float(
                self._param("maximum_horizontal_translation_m")
            ) * 1000.0,
            "max_vertical_position_correction_mm": float(
                self._param("maximum_vertical_translation_m")
            ) * 1000.0,
            "hover_tip_xyz": plan.hover_tip,
            "precontact_tip_xyz": plan.nominal_precontact_tip,
            "approach_distance_mm": plan.approach_distance_m * 1000.0,
            "tool_arm_angle_deg": angle,
            "target_axis_dot_m": sum(a * b for a, b in zip(target_minus_tip, tool_z)),
        })
        self._gate(
            "clearance", "PASS",
            plan.clearance_m * 1000.0,
            "visual target" if bool(self._param("dummy_contact_mode")) else
            f"goal standoff {float(self._param('standoff_m')) * 1000.0:.1f} mm",
            "direct Cartesian path" if plan.clearance_m < (
                0.0 if bool(self._param("dummy_contact_mode"))
                else float(self._param("standoff_m"))
            ) else "current-clearance hover then goal",
        )
        self.node.get_logger().info(
            "[GEOMETRY] "
            f"plan_id={self._diagnostics.plan_id} clearance={plan.clearance_m * 1000.0:.1f}mm "
            f"lateral={lateral_distance * 1000.0:.1f}mm "
            f"tool+Z={tool_z} arm_direction={direction} tool/arm_angle={angle}deg "
            f"dot(target-current_tip,nominal_approach)={self._context['target_axis_dot_m']:.4f}m"
        )
        self._emit("gate_snapshot", snapshot=self.build_diagnostic_snapshot())

    def _candidate_result(self, result: dict) -> None:
        self._candidate_results.append(result)
        self._emit("candidate_result", state=self._planning_state.value, **result)
        cartesian = result.get("cartesian", {})
        self.node.get_logger().info(
            f"[CANDIDATE] plan_id={self._diagnostics.plan_id} "
            f"angle={result['angle_offset_deg']:+.1f}deg RESULT={result['result']} "
            f"REASON={result['reason']} IK={result['ik']} collision={result['robot_collision']} "
            f"fractions={cartesian.get('current_to_hover_fraction')}/"
            f"{cartesian.get('hover_to_precontact_fraction')}/"
            f"{cartesian.get('combined_fraction')}"
        )

    def _summary(self, success: bool, reason: str) -> None:
        snapshot = self.build_diagnostic_snapshot()
        execute_motion = bool(self._context.get("execute_motion", False))
        motion_sent = bool(self._context.get("motion_sent", False))
        result = (
            self._planning_state.value
            if success and self._planning_state in (
                PlanningState.PRECONTACT_REACHED,
                PlanningState.DUMMY_TARGET_REACHED,
                PlanningState.DUMMY_PRESSURE_STOPPED,
            )
            else "APPROACH_PLANNED" if success else "FAILED"
        )
        event = {
            "result": result, "reason": reason, "motion_sent": motion_sent,
            "execute_motion": execute_motion, "snapshot": snapshot,
        }
        if motion_sent:
            event_type = "execution_success" if success else "execution_failure"
        elif execute_motion and not success:
            event_type = "execution_blocked"
        else:
            event_type = "planning_success" if success else "planning_failure"
        self._emit(event_type, **event)
        candidate_summary = ", ".join(
            f"{item['angle_offset_deg']:+.0f}deg {item['result']} {item['reason']}"
            for item in self._candidate_results
        ) or "not attempted"
        self.node.get_logger().info(
            "[SUMMARY] PIPER-H PRECONTACT SUMMARY "
            f"plan_id={self._diagnostics.plan_id} result={result} primary_reason={reason} "
            f"clearance={snapshot.get('clearance_mm')}mm lateral={snapshot.get('lateral_distance_mm')}mm "
            f"target_jitter={snapshot.get('target_jitter_mm')}mm "
            f"candidates=[{candidate_summary}] "
            f"motion_sent={'YES' if motion_sent else 'NO'} "
            f"execute_motion={str(execute_motion).lower()}"
        )

    def _frozen_target_in_planning_frame(self) -> PointStamped | None:
        frozen = self._target_filter.status().frozen
        source = self._latest_target
        if frozen is None or source is None:
            return None
        target = PointStamped()
        target.header = source.header
        target.point = Point(x=frozen[0], y=frozen[1], z=frozen[2])
        return self._transform_target(target)

    def _pose(self, position, orientation) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = str(self._param("planning_frame"))
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose = Pose(
            position=Point(x=position[0], y=position[1], z=position[2]),
            orientation=Quaternion(
                x=orientation[0], y=orientation[1],
                z=orientation[2], w=orientation[3],
            ),
        )
        return pose

    def _valid_ik(self, pose, seed, label: str, warn_only: bool = False):
        diagnostic = {
            "label": label,
            "planning_frame": pose.header.frame_id,
            "Link6_target_position": (
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
            ),
            "Link6_target_quaternion": (
                pose.pose.orientation.x, pose.pose.orientation.y,
                pose.pose.orientation.z, pose.pose.orientation.w,
            ),
            "pulse_tip_target": self._context.get("precontact_tip_xyz"),
            "seed_joint_values_rad": list(seed),
            "timeout_ms": float(self._param("ik_timeout_sec")) * 1000.0,
            "attempts": 1,
        }
        self._context["last_ik_diagnostic"] = diagnostic
        self._context["last_collision_has_tool"] = False
        joints = self.compute_ik_joint_target(
            pose, seed, str(self._param("ik_link")),
            float(self._param("ik_timeout_sec")), True, label,
            warn_only=warn_only,
        )
        if joints is None:
            self._context["last_ik_reason"] = "IK_FAILED"
            diagnostic.update(result="FAIL", reason="IK_FAILED")
            return None
        request = GetStateValidity.Request()
        request.robot_state = self._joint_state(joints, is_diff=False)
        request.group_name = self.group_name
        future = self._state_validity.call_async(request)
        if not self.wait(future, float(self._param("ik_timeout_sec"))):
            self._context["last_ik_reason"] = "IK_FAILED"
            diagnostic.update(result="FAIL", reason="STATE_VALIDITY_TIMEOUT")
            self.node.get_logger().warn(f"state-validity timeout for {label}")
            return None
        response = future.result()
        if response is None or not response.valid:
            contacts = len(response.contacts) if response is not None else 0
            pairs = []
            if response is not None:
                for contact in response.contacts:
                    first = getattr(contact, "contact_body_1", "")
                    second = getattr(contact, "contact_body_2", "")
                    pairs.append(f"{first} <-> {second}" if first or second else "unavailable")
            if not pairs:
                pairs = ["unavailable"]
            has_tool = any("pulse_tool_envelope" in pair for pair in pairs)
            self._context["last_ik_reason"] = "COLLISION"
            self._context["last_collision_has_tool"] = has_tool
            diagnostic.update(result="FAIL", reason="COLLISION", collision_pairs=pairs)
            self.node.get_logger().warn(
                f"collision validity failed for {label}; contacts={contacts}; pairs={pairs}"
            )
            return None
        self._context["last_ik_reason"] = "NONE"
        diagnostic.update(result="PASS", ik_solution_rad=list(joints), collision_pairs=[])
        self.node.get_logger().info(f"IK valid and collision-free: {label}")
        return joints

    def _publish_staged_markers(self, plan, selected, header) -> None:
        markers = []

        def sphere(marker_id, name, point, color):
            marker = Marker()
            marker.header = header
            marker.ns = "piper_pulse_staged"
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = Point(x=point[0], y=point[1], z=point[2])
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.014
            marker.color.r, marker.color.g, marker.color.b = color
            marker.color.a = 0.95
            marker.frame_locked = True
            markers.append(marker)
            label = Marker()
            label.header = header
            label.ns = "piper_pulse_staged_labels"
            label.id = marker_id
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position = Point(x=point[0], y=point[1], z=point[2] + 0.018)
            label.pose.orientation.w = 1.0
            label.scale.z = 0.014
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = name
            label.frame_locked = True
            markers.append(label)

        sphere(0, "current pulse_tip_center", plan.current_tip, (0.2, 0.7, 1.0))
        sphere(1, "target", plan.target, (1.0, 0.2, 0.2))
        sphere(2, "dummy target endpoint" if bool(self._param("dummy_contact_mode"))
               else "pre-contact", selected.tip, (0.2, 1.0, 0.3))
        arrow = Marker()
        arrow.header = header
        arrow.ns = "piper_pulse_staged"
        arrow.id = 10
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.points = [
            Point(x=plan.hover_tip[0], y=plan.hover_tip[1], z=plan.hover_tip[2]),
            Point(
                x=plan.hover_tip[0] + 0.06 * plan.nominal_approach[0],
                y=plan.hover_tip[1] + 0.06 * plan.nominal_approach[1],
                z=plan.hover_tip[2] + 0.06 * plan.nominal_approach[2],
            ),
        ]
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.004, 0.008, 0.012
        arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 0.9, 0.3, 1.0, 0.95
        arrow.frame_locked = True
        markers.append(arrow)
        path = Marker()
        path.header = header
        path.ns = "piper_pulse_staged"
        path.id = 11
        path.type = Marker.LINE_STRIP
        path.action = Marker.ADD
        path_points = [plan.current_tip]
        goal_standoff_m = (
            0.0 if bool(self._param("dummy_contact_mode"))
            else float(self._param("standoff_m"))
        )
        if plan.clearance_m >= goal_standoff_m:
            path_points.append(plan.hover_tip)
        path_points.append(selected.tip)
        path.points = [Point(x=x, y=y, z=z) for x, y, z in path_points]
        path.scale.x = 0.004
        path.color.r, path.color.g, path.color.b, path.color.a = 1.0, 0.75, 0.1, 1.0
        path.frame_locked = True
        markers.append(path)
        self._staged_marker_publisher.publish(MarkerArray(markers=markers))

    def _wait_for_preflight(self) -> bool:
        deadline = time.monotonic() + float(self._param("target_timeout_sec"))
        self._transition(PlanningState.WAITING_FOR_INPUT, "PREFLIGHT_STARTED")
        while rclpy.ok() and time.monotonic() < deadline:
            self._spin_inputs(0.05)
            now = time.monotonic()
            target = self._target_filter.status(now)
            target_age = (
                now - self._target_received if self._latest_target is not None
                else math.inf
            )
            target_grace = float(self._param("target_dropout_grace_sec"))
            target_soft_timeout = float(self._param("target_soft_timeout_sec"))
            target_hard_timeout = float(self._param("target_hard_timeout_sec"))
            gap_state = target_gap_state(
                target_age if self._latest_target is not None else None,
                target_grace, target_soft_timeout, target_hard_timeout,
            )
            target_fresh = gap_state == "TARGET_STABLE"
            target_lost = gap_state == "TARGET_LOST"
            if target_lost and not self._target_lost_reset:
                self._target_filter.reset("TARGET_HARD_TIMEOUT")
                self._target_lost_reset = True
                self._transition(PlanningState.TARGET_TRACKING, "TARGET_LOST")
                self._emit(
                    "target_lost", reason="TARGET_LOST",
                    last_target_age_ms=target_age * 1000.0,
                    soft_timeout_ms=target_soft_timeout * 1000.0,
                    hard_timeout_ms=target_hard_timeout * 1000.0,
                )
                target = self._target_filter.status(now)
            direction = self._direction_filter.status()
            pressure_graph = self._pressure_graph_snapshot()
            pressure_unique = pressure_graph["pressure_source_unique"]
            direction_fresh = (
                self._latest_arm_direction is not None
                and now - self._arm_direction_received
                <= float(self._param("arm_direction_max_age_sec"))
            )
            self._gate("arm_selected", "PASS" if self._selected_robot == "piperh" else "WAIT",
                       self._selected_robot, "piperh")
            self._gate("xbox_locked", "PASS" if self._xbox_known and self._xbox_locked else "WAIT",
                       self._xbox_locked if self._xbox_known else None, True)
            self._gate("joint_feedback_fresh", "PASS" if self._latest_joint_state_time and now - self._latest_joint_state_time <= 1.0 else "WAIT",
                       (now - self._latest_joint_state_time) * 1000.0 if self._latest_joint_state_time else None, "<=1000 ms")
            self._gate("target_received", "PASS" if self._latest_target else "WAIT")
            self._gate("target_fresh", "PASS" if target_fresh else "WAIT",
                       target_age * 1000.0 if self._latest_target else None,
                       f"grace<={target_grace * 1000.0:.0f} ms; "
                       f"soft<={target_soft_timeout * 1000.0:.0f} ms; "
                       f"hard<={target_hard_timeout * 1000.0:.0f} ms")
            self._gate("target_stable", "PASS" if target.stable else "WAIT",
                       {"jitter_mm": target.jitter_m * 1000.0,
                        "motion_mm": target.drift_m * 1000.0,
                        "samples": target.sample_count,
                        "acquisition_state": target.acquisition_state},
                       {"jitter_mm": float(self._param("target_stable_radius_m")) * 1000.0,
                        "hold_jitter_mm": float(self._param("target_soft_drift_m")) * 1000.0,
                        "changed_mm": float(self._param("target_hard_drift_m")) * 1000.0,
                        "min_samples": int(self._param("target_stable_min_samples")),
                        "duration_ms": 0.0})
            drift_result = "FAIL" if target.hard_drift else "WAIT" if target.drift_m > float(self._param("target_soft_drift_m")) else "PASS"
            self._gate("target_drift", drift_result, target.drift_m * 1000.0,
                       {"soft_mm": float(self._param("target_soft_drift_m")) * 1000.0,
                        "hard_mm": float(self._param("target_hard_drift_m")) * 1000.0})
            self._gate("arm_direction_received", "PASS" if self._latest_arm_direction else "WAIT")
            self._gate("arm_direction_fresh", "PASS" if direction_fresh else "WAIT",
                       (now - self._arm_direction_received) * 1000.0 if self._latest_arm_direction else None,
                       f"<={float(self._param('arm_direction_max_age_sec')) * 1000.0:.0f} ms")
            self._gate("arm_direction_stable", "PASS" if direction.stable else "WAIT",
                       {"jitter_deg": direction.jitter_deg,
                        "samples": direction.sample_count},
                       {"jitter_deg": float(self._param("arm_direction_stable_deg")),
                        "min_samples": int(self._param("arm_direction_stable_min_samples")),
                        "window_ms": float(self._param("arm_direction_filter_window_sec")) * 1000.0,
                        "required_duration_ms": 0.0})
            self._gate("pressure_source_unique", "PASS" if pressure_unique else "WAIT",
                       pressure_graph, "exactly 1 publisher on every pressure/status topic")
            self._gate("pressure_connected", "PASS" if self._serial_connected and pressure_unique else "WAIT",
                       self._serial_connected, True,
                       "requires unique pressure/status publishers")
            self._gate("pressure_zeroed", "PASS" if self._zero_calibrated and self._serial_connected and pressure_unique else "WAIT",
                       self._zero_calibrated, True,
                       "device zero flag; local pressure baseline is a separate later gate")
            pressure_fresh = all(channel in self._pressure_received and now - self._pressure_received[channel] <= float(self._param("maximum_pressure_age_sec")) for channel in ("s1", "s2", "s3"))
            self._gate("pressure_fresh", "PASS" if pressure_fresh and pressure_unique else "WAIT",
                       self._pressure_snapshot(now), f"<={float(self._param('maximum_pressure_age_sec')) * 1000.0:.0f} ms")
            self._gate("local_pressure_baseline", "PASS" if self._local_pressure_baseline is not None else "NOT_CHECKED",
                       self._local_pressure_baseline, "non-contact local baseline captured after preflight")
            self._gate("pressure_stable", "NOT_CHECKED" if self._local_pressure_baseline is None else
                       "FAIL" if any(self._pressure_filters[c].status(now).abort for c in ("s1", "s2", "s3")) else "PASS",
                       self._pressure_snapshot(now).get("max_delta_pa"),
                       {"delta_pa": float(self._param("pressure_precontact_delta_pa")),
                        "hold_ms": float(self._param("pressure_abort_hold_sec")) * 1000.0,
                        "emergency_delta_pa": float(self._param("pressure_emergency_delta_pa"))},
                       "not evaluated before local baseline" if self._local_pressure_baseline is None else "")
            for name in ("clearance", "workspace", "ik_hover", "collision_hover",
                         "ik_precontact", "collision_precontact", "cartesian_path"):
                self._gates.setdefault(name, {"result": "NOT_CHECKED", "actual": None,
                                              "threshold": None, "detail": ""})
            if not self._latest_target:
                self._set_blocker("WAIT_TARGET")
            elif target_lost:
                self._set_blocker(
                    "TARGET_LOST",
                    f"age={target_age * 1000.0:.0f}ms; cleared frozen target/history",
                )
            elif gap_state == "TARGET_TEMPORARILY_STALE":
                self._transition(PlanningState.TARGET_TEMPORARILY_STALE, gap_state)
                self._set_blocker(
                    "TARGET_TEMPORARILY_STALE",
                    f"temporary dropout age={target_age * 1000.0:.0f}ms; "
                    "frozen target/history retained; new descent planning blocked",
                )
            elif gap_state == "WAIT_TARGET_RECOVERY":
                self._transition(PlanningState.WAIT_TARGET_RECOVERY, gap_state)
                self._set_blocker(
                    "WAIT_TARGET_RECOVERY",
                    f"age={target_age * 1000.0:.0f}ms; frozen target/history retained; "
                    "waiting for same-target reacquisition",
                )
            elif not target.stable:
                tracking_state = (
                    PlanningState.TARGET_TRACKING
                    if target.sample_count < int(self._param("target_stable_min_samples"))
                    else PlanningState.TARGET_STABILIZING
                )
                self._transition(tracking_state, "TARGET_STABILIZING")
                self._set_blocker(
                    "TARGET_STABILIZING",
                    f"jitter={target.jitter_m * 1000.0:.1f}/"
                    f"{float(self._param('target_stable_radius_m')) * 1000.0:.1f}mm "
                    f"motion={target.drift_m * 1000.0:.1f}mm "
                    f"samples={target.sample_count}/"
                    f"{int(self._param('target_stable_min_samples'))} "
                    f"acquisition={target.acquisition_state}; no FPS/duration gate",
                )
            elif not self._latest_arm_direction:
                self._set_blocker("WAIT_ARM_DIRECTION")
            elif not direction_fresh:
                self._set_blocker("ARM_DIRECTION_STALE")
            elif not direction.stable:
                self._set_blocker("ARM_DIRECTION_UNSTABLE",
                                  f"jitter={direction.jitter_deg:.2f}/"
                                  f"{float(self._param('arm_direction_stable_deg')):.2f}deg "
                                  f"samples={direction.sample_count}/"
                                  f"{int(self._param('arm_direction_stable_min_samples'))}; "
                                  "no duration gate")
            elif self._selected_robot != "piperh":
                self._set_blocker("WRONG_ARM_SELECTED")
            elif not self._xbox_known or not self._xbox_locked:
                self._set_blocker("XBOX_NOT_LOCKED")
            elif not pressure_unique:
                self._set_blocker(
                    "PRESSURE_DUPLICATE_PUBLISHERS",
                    f"status={pressure_graph['pressure_status_publisher_count']} "
                    f"data={pressure_graph['pressure_data_publisher_counts']}",
                )
            elif not self._serial_connected:
                self._set_blocker("PRESSURE_NOT_CONNECTED")
            elif not self._zero_calibrated:
                self._set_blocker("PRESSURE_NOT_ZEROED")
            elif not pressure_fresh:
                self._set_blocker("PRESSURE_STALE")
            elif not self._pressure_safe():
                self._set_blocker("PRESSURE_DELTA_ABORT")
            else:
                self._primary_blocker = "NONE"
                self._blocking_reasons = []
                self._transition(PlanningState.TARGET_STABLE, "TARGET_STABLE")
                self._emit("gate_snapshot", snapshot=self.build_diagnostic_snapshot())
                return True
            if (
                target_fresh and target.stable and direction_fresh and direction.stable
                and self._xbox_known and self._xbox_locked
                and self._selected_robot == "piperh" and self._serial_connected
                and self._zero_calibrated and pressure_unique and self._pressure_safe()
            ):
                return True
        self.node.get_logger().warn(
            f"preflight wait timed out; PRIMARY_BLOCKER={self._primary_blocker}"
        )
        return False

    def _tool_collision_model_loaded(self) -> bool:
        if not self._model_parameters.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error("MoveIt model parameters are unavailable")
            return False
        request = GetParameters.Request(names=["robot_description"])
        future = self._model_parameters.call_async(request)
        if not self.wait(future, 5.0):
            self.node.get_logger().error("timed out checking Piper-H collision model")
            return False
        response = future.result()
        description = (
            response.values[0].string_value
            if response is not None and response.values else ""
        )
        if "pulse_tool_envelope" not in description:
            self.node.get_logger().error(
                "active MoveIt model lacks pulse_tool_envelope; "
                "restart the dual-arm launch to load the installed probe collision body"
            )
            return False
        return True

    def _target_in_planning_frame(self) -> PointStamped | None:
        target = self._latest_target
        if target is None:
            return None
        return self._transform_target(target)

    def _transform_target(self, target: PointStamped) -> PointStamped | None:
        planning_frame = str(self._param("planning_frame"))
        if target.header.frame_id == planning_frame:
            return target
        try:
            transform = self._tf_buffer.lookup_transform(
                planning_frame, target.header.frame_id,
                Time.from_msg(target.header.stamp),
                timeout=Duration(seconds=float(self._param("tf_timeout_sec"))),
            )
            transformed = do_transform_point(target, transform)
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            self._context["target_tf"] = {
                "source": target.header.frame_id, "target": planning_frame,
                "translation": (translation.x, translation.y, translation.z),
                "quaternion": (rotation.x, rotation.y, rotation.z, rotation.w),
                "requested_time": {
                    "sec": int(target.header.stamp.sec),
                    "nanosec": int(target.header.stamp.nanosec),
                },
            }
            return transformed
        except TransformException as error:
            detail = {
                "source": target.header.frame_id, "target": planning_frame,
                "requested_time": {
                    "sec": int(target.header.stamp.sec),
                    "nanosec": int(target.header.stamp.nanosec),
                }, "error": str(error), "latest_available_time": "unavailable",
            }
            self._emit("tf_error", reason="TF_LOOKUP_FAILED", **detail)
            self.node.get_logger().error(f"TF_LOOKUP_FAILED target transform: {detail}")
            return None

    def _arm_direction_in_planning_frame(self) -> tuple[tuple[float, float, float] | None, str]:
        message = self._latest_arm_direction
        if message is None:
            return None, "arm_direction has not been received"
        fresh, reason = direction_is_fresh(
            self._arm_direction_received, time.monotonic(),
            float(self._param("arm_direction_max_age_sec")),
        )
        if not fresh:
            return None, reason
        stamp_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        if stamp_ns <= 0:
            return None, "arm_direction timestamp is missing"
        ros_age = (
            self.node.get_clock().now().nanoseconds - stamp_ns
        ) / 1_000_000_000.0
        if (
            not math.isfinite(ros_age) or ros_age < -0.1
            or ros_age > float(self._param("arm_direction_max_age_sec"))
        ):
            return None, f"arm_direction header timestamp is stale (age={ros_age:.3f}s)"
        planning_frame = str(self._param("planning_frame"))
        filtered = self._direction_filter.status().filtered
        if filtered is None:
            return None, "arm_direction filter has no valid samples"
        if message.header.frame_id == planning_frame:
            return filtered, "arm_direction is ready"
        try:
            transform = self._tf_buffer.lookup_transform(
                planning_frame, message.header.frame_id,
                Time.from_msg(message.header.stamp),
                timeout=Duration(seconds=float(self._param("tf_timeout_sec"))),
            )
        except TransformException as error:
            self._emit(
                "tf_error", reason="TF_LOOKUP_FAILED",
                source=message.header.frame_id, target=planning_frame,
                requested_time={"sec": int(message.header.stamp.sec),
                                "nanosec": int(message.header.stamp.nanosec)},
                latest_available_time="unavailable", error=str(error),
            )
            return None, f"arm_direction TF unavailable: {error}"
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        self._context["arm_direction_tf"] = {
            "source": message.header.frame_id, "target": planning_frame,
            "translation": (translation.x, translation.y, translation.z),
            "quaternion": (rotation.x, rotation.y, rotation.z, rotation.w),
            "requested_time": {"sec": int(message.header.stamp.sec),
                               "nanosec": int(message.header.stamp.nanosec)},
        }
        filtered_message = Vector3Stamped()
        filtered_message.header = message.header
        filtered_message.vector.x, filtered_message.vector.y, filtered_message.vector.z = filtered
        try:
            transformed_filtered = do_transform_vector3(filtered_message, transform)
        except TransformException as error:
            return None, f"arm_direction filtered TF unavailable: {error}"
        vector = transformed_filtered.vector
        return (vector.x, vector.y, vector.z), "arm_direction is ready"

    def _run_axis_alignment_dry_run(
        self,
        link6_quaternion: tuple[float, float, float, float],
        current_j6_rad: float,
        target: PointStamped,
    ) -> None:
        arm_direction, input_reason = self._arm_direction_in_planning_frame()
        if arm_direction is None:
            self.node.get_logger().warn(f"sensor-axis dry-run invalid: {input_reason}")
            self._publish_axis_markers(target, None)
            return
        try:
            local_sensor = tuple(
                float(value) for value in self._param("sensor_array_axis_local")
            )
            local_j6 = tuple(float(value) for value in self._param("j6_axis_local"))
            sensor_base = rotate_vector(link6_quaternion, local_sensor)
            j6_base = rotate_vector(link6_quaternion, local_j6)
        except (TypeError, ValueError) as error:
            self.node.get_logger().warn(
                f"sensor-axis dry-run invalid: local axis configuration: {error}"
            )
            self._publish_axis_markers(target, None, arm_direction=arm_direction)
            return
        try:
            result = compute_sensor_axis_alignment(
                sensor_base, arm_direction, j6_base,
                sensor_array_axis_configured=bool(
                    self._param("sensor_array_axis_configured")
                ),
                current_j6_rad=current_j6_rad,
                j6_limits_rad=tuple(
                    float(value) for value in self._param("j6_limits_rad")
                ),
            )
        except (TypeError, ValueError) as error:
            self.node.get_logger().warn(
                f"sensor-axis dry-run invalid: parameter validation: {error}"
            )
            self._publish_axis_markers(target, None, arm_direction=arm_direction)
            return
        if bool(self._param("enable_auto_sensor_axis_alignment")):
            self.node.get_logger().warn(
                "enable_auto_sensor_axis_alignment was requested, but this build is "
                "dry-run only; no J6 rotation command will be generated"
            )
        details = (
            f"valid={result.valid}, reason={result.reason}, "
            f"delta={result.delta_rad:.6f} rad ({result.delta_deg:.3f} deg), "
            f"sensor={result.current_sensor_axis_base}, "
            f"arm={result.target_arm_axis_base}, J6={result.j6_axis_base}, "
            f"sensor_projected={result.projected_sensor_axis}, "
            f"arm_projected={result.projected_arm_axis}"
        )
        (self.node.get_logger().info if result.valid else
         self.node.get_logger().warn)(f"sensor-axis dry-run: {details}")
        self._publish_axis_markers(target, result)

    def _publish_axis_markers(
        self,
        target: PointStamped,
        result: SensorAxisAlignment | None,
        arm_direction: tuple[float, float, float] | None = None,
    ) -> None:
        origin = target.point
        clear = Marker()
        clear.header = target.header
        clear.action = Marker.DELETEALL
        markers = [clear]

        def arrow(marker_id: int, name: str, vector, rgb) -> None:
            if vector is None:
                return
            marker = Marker()
            marker.header = target.header
            marker.ns = f"piper_pulse_axis_{name}"
            marker.id = marker_id
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.points = [
                Point(x=origin.x, y=origin.y, z=origin.z),
                Point(
                    x=origin.x + 0.08 * vector[0],
                    y=origin.y + 0.08 * vector[1],
                    z=origin.z + 0.08 * vector[2],
                ),
            ]
            marker.scale.x, marker.scale.y, marker.scale.z = 0.004, 0.008, 0.012
            marker.color.r, marker.color.g, marker.color.b = rgb
            marker.color.a = 0.95
            marker.frame_locked = True
            markers.append(marker)

        sensor = result.current_sensor_axis_base if result is not None else None
        arm = (
            result.target_arm_axis_base if result is not None
            else arm_direction
        )
        j6 = result.j6_axis_base if result is not None else None
        if result is None or bool(self._param("sensor_array_axis_configured")):
            arrow(0, "sensor", sensor, (0.1, 0.8, 1.0))
        arrow(1, "arm", arm, (0.2, 1.0, 0.2))
        arrow(2, "j6", j6, (1.0, 0.3, 0.9))
        text_marker = Marker()
        text_marker.header = target.header
        text_marker.ns = "piper_pulse_axis_alignment"
        text_marker.id = 3
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position = Point(
            x=origin.x, y=origin.y, z=origin.z + 0.04
        )
        text_marker.pose.orientation.w = 1.0
        text_marker.scale.z = 0.018
        text_marker.color.r = text_marker.color.g = text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        text_marker.text = (
            f"J6 dry-run: {result.delta_deg:+.2f} deg"
            if result is not None and result.valid
            else f"J6 dry-run invalid: {result.reason if result else 'missing arm_direction'}"
        )
        text_marker.frame_locked = True
        markers.append(text_marker)
        self._axis_marker_publisher.publish(MarkerArray(markers=markers))

    def _current_tool_transform(self):
        target_frame = str(self._param("planning_frame"))
        source_frame = str(self._param("tool_frame"))
        deadline = time.monotonic() + float(self._param("tf_timeout_sec"))
        last_error = None
        while rclpy.ok():
            try:
                # A blocking lookup on this node starves its own TF subscription.
                # Let the executor receive both /tf and /tf_static between tries.
                transform = self._tf_buffer.lookup_transform(
                    target_frame, source_frame, Time(), timeout=Duration(seconds=0.0),
                )
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                self._context["Link6_tf"] = {
                    "source": source_frame,
                    "target": target_frame,
                    "translation": (translation.x, translation.y, translation.z),
                    "quaternion": (rotation.x, rotation.y, rotation.z, rotation.w),
                    "tf_age_ms": "latest_static_or_dynamic",
                }
                return transform
            except TransformException as error:
                last_error = error
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            self._spin_inputs(min(0.05, remaining))
        detail = {
            "source": source_frame,
            "target": target_frame,
            "requested_time": "latest", "latest_available_time": "unavailable",
            "error": str(last_error) if last_error is not None else "ROS shutdown",
        }
        self._emit("tf_error", reason="TF_LOOKUP_FAILED", **detail)
        self.node.get_logger().error(f"TF_LOOKUP_FAILED current Link6: {detail}")
        return None

    def _pre_execution_safe(self, frozen_target) -> bool:
        pressure_graph = self._pressure_graph_snapshot()
        if not pressure_graph["pressure_source_unique"]:
            self._set_blocker("PRESSURE_DUPLICATE_PUBLISHERS")
            return False
        if not self._xbox_known or not self._xbox_locked:
            self._set_blocker("XBOX_NOT_LOCKED")
            self.node.get_logger().error("Xbox control is no longer LOCKED")
            return False
        if self._selected_robot != "piperh":
            self._set_blocker("ARM_NOT_SELECTED", self._selected_robot or "unknown")
            return False
        if not self._serial_connected:
            self._set_blocker("PRESSURE_DISCONNECTED")
            return False
        if not self._zero_calibrated:
            self._set_blocker("PRESSURE_NOT_ZEROED")
            return False
        if self._local_pressure_baseline is None:
            self._set_blocker("PRESSURE_BASELINE_MISSING")
            return False
        now = time.monotonic()
        joints_complete = all(
            name in self._latest_joint_positions for name in self.joint_names
        )
        joint_age = (
            now - self._latest_joint_state_time
            if self._latest_joint_state_time else math.inf
        )
        self._gate(
            "joint_feedback_fresh", "PASS" if joints_complete and joint_age <= 1.0 else "FAIL",
            joint_age * 1000.0 if math.isfinite(joint_age) else None, "<=1000 ms",
        )
        if not joints_complete or joint_age > 1.0:
            self._set_blocker("JOINT_FEEDBACK_STALE")
            return False
        pressure_deadline = time.monotonic() + float(self._param("target_timeout_sec"))
        while not self._pressure_safe():
            if not self._pressure_graph_snapshot()["pressure_source_unique"]:
                self._set_blocker("PRESSURE_DUPLICATE_PUBLISHERS")
                return False
            if (
                self._latest_target is not None
                and time.monotonic() - self._target_received
                > float(self._param("target_hard_timeout_sec"))
            ):
                self._target_filter.reset("TARGET_HARD_TIMEOUT")
                self._target_lost_reset = True
                self._attempt_invalidated = True
                self._set_blocker("TARGET_LOST", "target expired while waiting for pressure")
                return False
            pressure = self._pressure_snapshot()
            if any(pressure[channel]["abort"] for channel in ("S1", "S2", "S3")):
                self._set_blocker("PRESSURE_DELTA_ABORT")
                return False
            self._set_blocker("PRESSURE_STALE")
            if time.monotonic() >= pressure_deadline:
                return False
            self._spin_inputs(0.05)
        target_status = self._target_filter.status(time.monotonic())
        if self._attempt_invalidated or target_status.frozen is None:
            if (
                self._target_lost_reset
                or target_status.stable_reset_reason == "TARGET_HARD_TIMEOUT"
            ):
                self._target_lost_reset = True
                self._attempt_invalidated = True
                self._set_blocker("TARGET_LOST", "target history was cleared")
                return False
            self._set_blocker("TARGET_MOVED", "stable target was invalidated during planning")
            return False
        recovery_deadline = time.monotonic() + float(self._param("target_timeout_sec"))
        observed_soft_drift_at = None
        while rclpy.ok() and time.monotonic() < recovery_deadline:
            now = time.monotonic()
            target_age = (
                now - self._target_received if self._latest_target is not None
                else math.inf
            )
            if self._latest_target is None:
                self._set_blocker("WAIT_TARGET")
                self._spin_inputs(0.05)
                continue
            if target_age > float(self._param("target_hard_timeout_sec")):
                self._target_filter.reset("TARGET_HARD_TIMEOUT")
                self._target_lost_reset = True
                self._attempt_invalidated = True
                self._set_blocker(
                    "TARGET_LOST",
                    f"age={target_age * 1000.0:.0f}ms; cleared frozen target/history",
                )
                return False
            if target_age > float(self._param("target_soft_timeout_sec")):
                self._set_blocker(
                    "WAIT_TARGET_RECOVERY",
                    f"temporary dropout age={target_age * 1000.0:.0f}ms; "
                    "new descent planning blocked",
                )
                self._spin_inputs(0.05)
                continue
            if target_age > float(self._param("target_dropout_grace_sec")):
                self._set_blocker(
                    "TARGET_TEMPORARILY_STALE",
                    f"temporary dropout age={target_age * 1000.0:.0f}ms; "
                    "new descent planning blocked",
                )
                self._spin_inputs(0.05)
                continue
            status = self._target_filter.status(now)
            if not status.stable:
                self._set_blocker(
                    "TARGET_TEMPORARILY_STALE",
                    f"spatial reacquisition={status.reacquire_result}",
                )
                self._spin_inputs(0.05)
                continue
            drift = status.drift_m
            if status.hard_drift:
                self._set_blocker(
                    "TARGET_MOVED",
                    f"filtered drift={drift * 1000.0:.1f}mm persisted above "
                    f"{float(self._param('target_hard_drift_m')) * 1000.0:.1f}mm",
                )
                return False
            if drift > float(self._param("target_soft_drift_m")):
                if observed_soft_drift_at is None:
                    observed_soft_drift_at = now
                self._set_blocker(
                    "TARGET_MOVED",
                    f"observing soft drift={drift * 1000.0:.1f}mm",
                )
                if now - observed_soft_drift_at < float(self._param("target_soft_drift_observe_sec")):
                    self._spin_inputs(0.05)
                    continue
                if drift <= float(self._param("target_hard_drift_m")):
                    break
            else:
                break
        else:
            self._set_blocker("WAIT_TARGET")
            return False
        self._primary_blocker = "NONE"
        self._blocking_reasons = []
        return True

    def _execution_safety_blocker(self, now: float) -> str | None:
        """Return the live interlock that requires cancelling active motion."""
        if bool(self._param("dummy_contact_mode")):
            if self._attempt_invalidated:
                return "TARGET_MOVED"
        if not self._xbox_known or not self._xbox_locked:
            return "XBOX_NOT_LOCKED"
        if self._selected_robot != "piperh":
            return "ARM_NOT_SELECTED"
        if not self._serial_connected:
            return "PRESSURE_DISCONNECTED"
        if not self._zero_calibrated:
            return "PRESSURE_NOT_ZEROED"
        if self._local_pressure_baseline is None:
            return "PRESSURE_BASELINE_MISSING"
        maximum_age = float(self._param("maximum_pressure_age_sec"))
        if any(
            channel not in self._pressure_received
            or now - self._pressure_received[channel] > maximum_age
            for channel in ("s1", "s2", "s3")
        ):
            return "PRESSURE_STALE"
        if any(
            self._pressure_filters[channel].status(now).emergency_abort
            for channel in ("s1", "s2", "s3")
        ):
            return "PRESSURE_EMERGENCY_ABORT"
        if any(
            self._pressure_filters[channel].status(now).abort
            for channel in ("s1", "s2", "s3")
        ):
            return "PRESSURE_DELTA_ABORT"
        return None

    def _execute_trajectory_with_pressure_monitor(
        self, trajectory, timeout_sec: float
    ) -> bool:
        """Execute while pressure and operator interlocks remain continuously valid."""
        send_future = self._execute.send_goal_async(
            ExecuteTrajectory.Goal(trajectory=trajectory)
        )
        if not self.wait(send_future, 5.0):
            self.node.get_logger().error("Timed out sending ExecuteTrajectory goal")
            return False
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.node.get_logger().error("ExecuteTrajectory rejected goal")
            return False

        result_future = goal_handle.get_result_async()
        deadline = time.monotonic() + timeout_sec
        next_graph_check = 0.0
        while rclpy.ok() and not result_future.done():
            now = time.monotonic()
            blocker = self._execution_safety_blocker(now)
            if blocker is None and now >= next_graph_check:
                next_graph_check = now + 0.25
                if not self._pressure_graph_snapshot()["pressure_source_unique"]:
                    blocker = "PRESSURE_DUPLICATE_PUBLISHERS"
            if blocker is not None:
                self._set_blocker(
                    blocker, "active trajectory cancelled by live safety monitor"
                )
                self.node.get_logger().error(
                    f"{blocker}: cancelling active pre-contact trajectory"
                )
                cancel_future = goal_handle.cancel_goal_async()
                self.wait(cancel_future, 2.0)
                self.wait(result_future, 2.0)
                if blocker in ("PRESSURE_EMERGENCY_ABORT", "PRESSURE_DELTA_ABORT"):
                    self._context["pressure_stop_confirmed"] = (
                        result_future.done()
                        and result_future.result() is not None
                        and result_future.result().status == GoalStatus.STATUS_CANCELED
                    )
                return False
            if now >= deadline:
                self._set_blocker("TRAJECTORY_EXECUTION_FAILED", "execution timeout")
                cancel_future = goal_handle.cancel_goal_async()
                self.wait(cancel_future, 2.0)
                self.wait(result_future, 2.0)
                return False
            rclpy.spin_once(self.node, timeout_sec=min(0.02, deadline - now))

        action_result = result_future.result() if result_future.done() else None
        if action_result is None:
            self.node.get_logger().error("ExecuteTrajectory returned an empty result")
            return False
        result = action_result.result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.node.get_logger().error(
                f"ExecuteTrajectory failed with code {result.error_code.val}: "
                f"{result.error_code.message}"
            )
            return False
        return True

    def _plan_cartesian(
        self, start: list[float], waypoints: tuple[Pose, ...], warn_only: bool = False
    ):
        if not self._wait_for_target_recovery():
            return None
        request = GetCartesianPath.Request()
        request.header.frame_id = str(self._param("planning_frame"))
        request.header.stamp = self.node.get_clock().now().to_msg()
        request.start_state = self._joint_state(start, is_diff=False)
        request.group_name = self.group_name
        request.link_name = str(self._param("ik_link"))
        request.waypoints = list(waypoints)
        request.max_step = float(self._param("cartesian_step_m"))
        request.jump_threshold = float(self._param("jump_threshold"))
        request.revolute_jump_threshold = float(self._param("revolute_jump_threshold_rad"))
        request.avoid_collisions = True
        dummy_mode = bool(self._param("dummy_contact_mode"))
        request.max_velocity_scaling_factor = min(
            float(self._param("velocity_scaling")), 0.05
        ) if dummy_mode else float(self._param("velocity_scaling"))
        request.max_acceleration_scaling_factor = min(
            float(self._param("acceleration_scaling")), 0.05
        ) if dummy_mode else float(self._param("acceleration_scaling"))
        request.cartesian_speed_limited_link = request.link_name
        request.max_cartesian_speed = min(
            float(self._param("maximum_cartesian_speed_m_s")),
            float(self._param("dummy_max_cartesian_speed_m_s")), 0.005,
        ) if dummy_mode else float(self._param("maximum_cartesian_speed_m_s"))
        future = self._cartesian.call_async(request)
        timeout = float(self._param("result_timeout"))
        if not self.wait(future, timeout):
            self._context["last_cartesian"] = {
                "fraction": 0.0, "error_code": "TIMEOUT",
                "eef_step_m": request.max_step, "jump_threshold": request.jump_threshold,
                "waypoint_count": len(request.waypoints),
                "last_valid_waypoint": "unavailable", "last_valid_tip_xyz": "unavailable",
            }
            self.node.get_logger().error("Piper-H Cartesian planning timed out")
            return None
        response = future.result()
        code = response.error_code.val if response is not None else "empty"
        fraction = float(response.fraction) if response is not None else 0.0
        self._context["last_cartesian"] = {
            "fraction": fraction, "error_code": code,
            "eef_step_m": request.max_step, "jump_threshold": request.jump_threshold,
            "revolute_jump_threshold_rad": request.revolute_jump_threshold,
            "waypoint_count": len(request.waypoints),
            "last_valid_waypoint": "unavailable", "last_valid_tip_xyz": "unavailable",
        }
        if (
            response is None or response.error_code.val != MoveItErrorCodes.SUCCESS
            or response.fraction < 0.999
            or len(response.solution.joint_trajectory.points) < 2
        ):
            (self.node.get_logger().warn if warn_only else self.node.get_logger().error)(
                f"full collision-free Cartesian path unavailable: "
                f"code={code} fraction={fraction:.3f}"
            )
            return None
        return response.solution


def main() -> None:
    rclpy.init()
    operation = None
    ok = False
    try:
        operation = PiperPulseAlign()
        ok = operation.run()
    except Exception as error:
        if operation is not None:
            operation.node.get_logger().error(str(error))
            operation._set_blocker("INTERNAL_ERROR", str(error))
            operation._emit("exception", reason="INTERNAL_ERROR", exception=str(error),
                            snapshot=operation.build_diagnostic_snapshot())
            operation._transition(PlanningState.ERROR, "INTERNAL_ERROR")
            operation._summary(False, "INTERNAL_ERROR")
        else:
            print(f"piper_pulse_align: {error}", file=sys.stderr)
    finally:
        if operation is not None:
            operation.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
