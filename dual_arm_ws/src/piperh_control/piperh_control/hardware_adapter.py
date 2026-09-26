"""Safe ROS 2 trajectory and feedback adapter for the official Piper CAN node."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from datetime import datetime
import gc
import json
import math
import os
import threading
import time
import uuid
from pathlib import Path

import can
import numpy as np

from control_msgs.action import FollowJointTrajectory
from ament_index_python.packages import get_package_share_directory
from piper_msgs.srv import Enable
from piper_msgs.msg import PiperStatusMsg, PiperTeachJointState
from rebotarm_msgs.msg import ArmStatus
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectory
from piper.teach_can_grouper import JOINT_IDS, TeachCanGrouper

from .gravity_compensator import (
    PiperHGravityCompensator,
    limit_torque,
    rotation_from_rpy,
)
from .mit_backend import OfficialMitBackend, read_firmware
from .playback_tracking import TrackingLog, bounded_trajectory_step, prepare_trajectory
from .command_authority import AuthoritativeWriter, AuthorityToken, CommandAuthority


ARM_JOINTS = tuple(f"joint{index}" for index in range(1, 7))
NORMAL_FEEDBACK_SOURCE = "direct_socketcan_0x2A5_0x2A7"
LOWER = (-2.618, 0.0, -2.96706, -2.356195, -1.56207, -3.14)
UPPER = (2.618, 3.14, 0.0, 2.356195, 1.56207, 3.14)


@dataclass(frozen=True)
class JointFeedback:
    joint_positions: tuple[float, ...]
    feedback_timestamp: float
    feedback_age: float
    valid: bool
    source: str


def bounded_arm(names: Sequence[str], positions: Sequence[float]) -> list[float] | None:
    """Order and validate one Piper-H arm command."""
    if len(names) != len(positions) or len(set(names)) != len(names):
        return None
    values = dict(zip(names, positions))
    if not all(name in values for name in ARM_JOINTS):
        return None
    ordered = [float(values[name]) for name in ARM_JOINTS]
    if not all(math.isfinite(value) for value in ordered):
        return None
    if any(value < low or value > high for value, low, high in zip(ordered, LOWER, UPPER)):
        return None
    return ordered


def command_gate_reason(feedback_fresh: bool, motors_enabled: bool) -> str | None:
    """Return the fail-safe reason preventing a hardware command, if any."""
    if not feedback_fresh:
        return "stale CAN feedback"
    if not motors_enabled:
        return "motors are disabled"
    return None


class HardwareAdapter(Node):
    """Expose FollowJointTrajectory while keeping CAN details behind one gate."""

    def __init__(self) -> None:
        super().__init__("piperh_hardware_adapter")
        self.declare_parameter("raw_feedback_topic", "/piperh/driver_joint_states")
        self.declare_parameter(
            "grouped_feedback_topic", "/piperh/teach_joint_states_raw"
        )
        self.declare_parameter(
            "recording_feedback_topic", "/piperh/normal_joint_feedback"
        )
        self.declare_parameter("driver_command_topic", "/piperh/driver_joint_command")
        self.declare_parameter("driver_authority_topic", "/piperh/command_authority")
        self.declare_parameter("command_lease_seconds", 0.75)
        self.declare_parameter("driver_enable_service", "/piperh/enable_srv")
        self.declare_parameter("motor_enable_service", "/piperh/motor/set_enabled")
        self.declare_parameter("stream_command_topic", "/piperh/servo_joint_trajectory")
        self.declare_parameter("feedback_topic", "/joint_states")
        self.declare_parameter("arm_status_topic", "/piperh/arm_status")
        self.declare_parameter(
            "trajectory_action", "/arm_controller/follow_joint_trajectory"
        )
        self.declare_parameter("feedback_timeout_sec", 0.75)
        self.declare_parameter("motor_enable_latch_timeout_sec", 3.0)
        self.declare_parameter("feedback_publish_rate_hz", 100.0)
        self.declare_parameter("command_rate_hz", 50.0)
        self.declare_parameter("driver_speed_percent", 25)
        self.declare_parameter("playback_rate_hz", 100.0)
        self.declare_parameter("playback_max_velocity_rad_s", 0.40)
        self.declare_parameter("playback_max_acceleration_rad_s2", 1.0)
        self.declare_parameter("playback_feedback_timeout_sec", 0.20)
        self.declare_parameter("playback_joint_tolerance_rad", 0.02)
        self.declare_parameter("playback_stable_cycles", 10)
        self.declare_parameter("playback_settle_timeout_sec", 5.0)
        self.declare_parameter("tracking_warn_threshold_rad", 0.05)
        self.declare_parameter("tracking_abort_threshold_rad", 0.20)
        self.declare_parameter("tracking_abort_duration_sec", 0.5)
        self.declare_parameter("playback_tracking_dir", str(Path(os.environ.get("REBOTARM_WORKSPACE", ".")) / "log/piperh_tracking"))
        self.declare_parameter("can_port", "can0")
        self.declare_parameter("gravity_require_selected_robot", False)
        self.declare_parameter("gravity_real_torque_enabled", False)
        self.declare_parameter("gravity_mit_support_confirmed", False)
        self.declare_parameter("gravity_base_mount_confirmed", False)
        self.declare_parameter("gravity_expected_firmware", "")
        self.declare_parameter("gravity_mount_roll", 0.0)
        self.declare_parameter("gravity_mount_pitch", 0.0)
        self.declare_parameter("gravity_mount_yaw", 0.0)
        self.declare_parameter("gravity_payload_mass_kg", 0.0)
        self.declare_parameter("gravity_payload_com_xyz_m", [0.0, 0.0, 0.0])
        self.declare_parameter("gravity_torque_scale", 0.10)
        self.declare_parameter("gravity_kp", [0.0] * 6)
        self.declare_parameter("gravity_kd", [0.8] * 6)
        self.declare_parameter("gravity_max_torque_nm", [0.0] * 6)
        self.declare_parameter("gravity_rate_hz", 50.0)
        self.declare_parameter("gravity_max_velocity_rad_s", 0.6)
        self.declare_parameter("gravity_max_duration_sec", 0.0)

        self._group = ReentrantCallbackGroup()
        # FollowJointTrajectory executes a blocking fixed-period loop in the
        # general callback group. Keep the authoritative grouped-CAN callback
        # isolated so service/status traffic cannot serialize it behind that
        # loop.
        self._feedback_group = MutuallyExclusiveCallbackGroup()
        self._lock = threading.Lock()
        self._arm = [0.0] * 6
        self._motors_enabled = False
        self._xbox_armed = None
        self._selected_robot = None
        self._vendor_status = None
        self._last_vendor_status = 0.0
        self._gravity_active = False
        self._gravity_transition = False
        self._gravity_stop_event = threading.Event()
        self._gravity_thread = None
        self._gravity_sdk = None
        self._gravity_fault = ""
        self._gravity_phase = "IDLE"
        self._action_active = False
        self._command_authority = CommandAuthority()
        self._servo_authority: AuthorityToken | None = None
        self._action_authority: AuthorityToken | None = None
        self._gravity_authority: AuthorityToken | None = None
        self._authority_lease = float(
            self.get_parameter("command_lease_seconds").value)
        if not 0.1 <= self._authority_lease <= 2.0:
            raise ValueError("command_lease_seconds must be within [0.1, 2.0]")
        self._driver_authority_id = f"piperh_adapter:{uuid.uuid4().hex}"
        self._mode_lock = threading.Lock()
        self._last_feedback = 0.0
        self._last_joint_stamp = 0.0
        self._last_driver_feedback = 0.0
        self._last_driver_joint_stamp = 0.0
        self._driver_feedback_count = 0
        self._grouped_feedback_count = 0
        self._last_grouped_feedback = 0.0
        self._last_grouped_joint_stamp = 0.0
        self._direct_can_feedback_count = 0
        self._last_feedback_publish = 0.0
        self._timeout = float(self.get_parameter("feedback_timeout_sec").value)
        self._motor_enable_latch_timeout = float(
            self.get_parameter("motor_enable_latch_timeout_sec").value
        )
        feedback_publish_rate = float(
            self.get_parameter("feedback_publish_rate_hz").value
        )
        self._feedback_publish_period = 1.0 / feedback_publish_rate
        self._rate = float(self.get_parameter("command_rate_hz").value)
        self._driver_speed = int(self.get_parameter("driver_speed_percent").value)
        self._playback_rate = float(self.get_parameter("playback_rate_hz").value)
        self._playback_velocity = float(self.get_parameter("playback_max_velocity_rad_s").value)
        self._playback_acceleration = float(self.get_parameter("playback_max_acceleration_rad_s2").value)
        self._playback_feedback_timeout = float(self.get_parameter("playback_feedback_timeout_sec").value)
        self._playback_tolerance = float(self.get_parameter("playback_joint_tolerance_rad").value)
        self._playback_stable_cycles = int(self.get_parameter("playback_stable_cycles").value)
        self._playback_settle_timeout = float(self.get_parameter("playback_settle_timeout_sec").value)
        self._tracking_warn = float(self.get_parameter("tracking_warn_threshold_rad").value)
        self._tracking_abort = float(self.get_parameter("tracking_abort_threshold_rad").value)
        self._tracking_abort_duration = float(self.get_parameter("tracking_abort_duration_sec").value)
        if min(
            self._timeout,
            self._motor_enable_latch_timeout,
            self._rate,
            feedback_publish_rate,
        ) <= 0.0:
            raise ValueError("adapter timing parameters must be positive")
        if self._motor_enable_latch_timeout <= self._timeout:
            raise ValueError(
                "motor_enable_latch_timeout_sec must exceed feedback_timeout_sec"
            )
        if not 1 <= self._driver_speed <= 100:
            raise ValueError("driver_speed_percent must be in [1, 100]")
        if (not all(math.isfinite(x) and x > 0 for x in (
            self._playback_rate, self._playback_velocity, self._playback_acceleration,
            self._playback_feedback_timeout, self._playback_tolerance,
            self._playback_settle_timeout, self._tracking_warn, self._tracking_abort,
            self._tracking_abort_duration)) or self._playback_stable_cycles < 1
            or self._tracking_abort <= self._tracking_warn):
            raise ValueError("invalid playback timing or tracking thresholds")

        self._command_pub = self.create_publisher(
            JointState, str(self.get_parameter("driver_command_topic").value), 10
        )
        self._driver_authority_pub = self.create_publisher(
            String, str(self.get_parameter("driver_authority_topic").value), 10
        )
        self._driver_authority_timer = self.create_timer(
            self._authority_lease / 3.0, self._publish_driver_authority
        )
        self._feedback_pub = self.create_publisher(
            JointState, str(self.get_parameter("feedback_topic").value), 10
        )
        self._recording_feedback_pub = self.create_publisher(
            PiperTeachJointState,
            str(self.get_parameter("recording_feedback_topic").value),
            qos_profile_sensor_data,
        )
        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._status_pub = self.create_publisher(
            ArmStatus, str(self.get_parameter("arm_status_topic").value), status_qos
        )
        self._status_timer = self.create_timer(0.2, self._publish_status)
        self._driver_enable_client = self.create_client(
            Enable,
            str(self.get_parameter("driver_enable_service").value),
            callback_group=self._group,
        )
        self._motor_enable_service = self.create_service(
            SetBool,
            str(self.get_parameter("motor_enable_service").value),
            self._set_motor_enabled,
            callback_group=self._group,
        )
        self.create_service(
            Trigger, "/piperh/gravity_compensation/start",
            self._start_gravity_compensation, callback_group=self._group,
        )
        self.create_service(
            Trigger, "/piperh/gravity_compensation/stop",
            self._stop_gravity_compensation, callback_group=self._group,
        )
        self.create_service(
            Trigger, "/piperh/control_state/dump",
            self._dump_control_state, callback_group=self._group,
        )
        xbox_qos = QoSProfile(depth=1)
        xbox_qos.reliability = ReliabilityPolicy.RELIABLE
        xbox_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            Bool, "/piperh/xbox/armed", self._xbox_status,
            xbox_qos, callback_group=self._group,
        )
        self.create_subscription(
            String, "/dual_arm/selected", self._selected_robot_status,
            xbox_qos, callback_group=self._group,
        )
        self.create_subscription(
            PiperStatusMsg, "/piperh/driver_arm_status", self._vendor_status_callback,
            qos_profile_sensor_data, callback_group=self._group,
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("raw_feedback_topic").value),
            self._driver_feedback_diagnostic,
            qos_profile_sensor_data,
            callback_group=self._group,
        )
        self.create_subscription(
            PiperTeachJointState,
            str(self.get_parameter("grouped_feedback_topic").value),
            self._grouped_feedback_diagnostic,
            qos_profile_sensor_data,
            callback_group=self._feedback_group,
        )
        self.create_subscription(
            JointTrajectory,
            str(self.get_parameter("stream_command_topic").value),
            self._stream,
            10,
            callback_group=self._group,
        )
        self._arm_action = ActionServer(
            self,
            FollowJointTrajectory,
            str(self.get_parameter("trajectory_action").value),
            execute_callback=self._execute_arm,
            goal_callback=self._arm_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self._group,
        )
        self.get_logger().warning(
            "CAN output is gated by the official driver's /enable_srv; auto-enable is off."
        )
        self._publish_driver_authority()
        self._direct_can_grouper = TeachCanGrouper()
        self._direct_can_stop = threading.Event()
        self._direct_can_bus = can.Bus(
            interface="socketcan",
            channel=str(self.get_parameter("can_port").value),
            can_filters=[
                {"can_id": value, "can_mask": 0x7FF, "extended": False}
                for value in JOINT_IDS
            ],
        )
        self._direct_can_thread = threading.Thread(
            target=self._direct_can_feedback_loop,
            name="piperh-direct-feedback",
            daemon=True,
        )
        self._direct_can_thread.start()

    def _set_motor_enabled(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        """Expose the vendor motor gate through a standard, namespaced service."""
        if request.data:
            with self._lock:
                if self._gravity_active or self._gravity_transition:
                    response.success = False
                    response.message = "stop gravity compensation before enabling motors"
                    return response
        else:
            self._stop_gravity_loop()
        if not self._driver_enable_client.wait_for_service(timeout_sec=0.0):
            response.success = False
            response.message = "Piper vendor enable service is not ready"
            return response

        vendor_request = Enable.Request()
        vendor_request.enable_request = bool(request.data)
        future = self._driver_enable_client.call_async(vendor_request)
        completed = threading.Event()
        future.add_done_callback(lambda _: completed.set())
        if not completed.wait(timeout=6.5):
            future.cancel()
            response.success = False
            response.message = "Timed out waiting for Piper motor status"
            return response
        try:
            vendor_response = future.result()
        except Exception as error:  # pragma: no cover - middleware failure path
            response.success = False
            response.message = f"Piper enable service failed: {error}"
            return response

        response.success = bool(vendor_response.enable_response)
        if response.success:
            with self._lock:
                self._motors_enabled = bool(request.data)
            self._publish_status()
        action = "enabled" if request.data else "disabled"
        response.message = (
            f"Piper motors {action}"
            if response.success
            else f"Piper motors were not confirmed {action}"
        )
        return response

    def _driver_feedback_diagnostic(self, message: JointState) -> None:
        """Track the SDK-derived ROS layer without using it as the safety clock."""
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if not math.isfinite(stamp) or stamp <= 0.0:
            return
        with self._lock:
            if stamp <= self._last_driver_joint_stamp:
                return
            self._last_driver_joint_stamp = stamp
            self._last_driver_feedback = time.monotonic()
            self._driver_feedback_count += 1

    def _grouped_feedback_diagnostic(self, message: PiperTeachJointState) -> None:
        """Validate and relay the sole high-rate normal recording stream."""
        if len(message.name) != len(message.position):
            return
        values = dict(zip(message.name, message.position))
        if not all(name in values for name in ARM_JOINTS):
            return
        if not all(math.isfinite(float(values[name])) for name in ARM_JOINTS):
            return
        stamp = (
            message.sample_timestamp.sec + message.sample_timestamp.nanosec * 1e-9
        )
        now = time.monotonic()
        if (
            not message.timing_valid
            or not math.isfinite(stamp) or stamp <= 0.0
        ):
            return
        with self._lock:
            if stamp <= self._last_grouped_joint_stamp:
                return
            self._last_grouped_joint_stamp = stamp
            self._last_grouped_feedback = now
            self._grouped_feedback_count += 1
        self._recording_feedback_pub.publish(message)

    def _direct_can_feedback_loop(self) -> None:
        """Receive real joint frames without depending on ROS executor scheduling."""
        last_warning = 0.0
        while not self._direct_can_stop.is_set():
            try:
                frame = self._direct_can_bus.recv(timeout=0.2)
            except can.CanError as error:
                if self._direct_can_stop.is_set():
                    break
                now = time.monotonic()
                if now - last_warning >= 1.0:
                    self.get_logger().error(
                        f"direct joint feedback receive failed: {error}"
                    )
                    last_warning = now
                continue
            if frame is None or frame.is_extended_id:
                continue
            sample = self._direct_can_grouper.add(
                frame.arbitration_id, frame.timestamp, bytes(frame.data)
            )
            if sample is None or not sample.timing_valid:
                continue
            now = time.monotonic()
            publish = False
            with self._lock:
                if sample.timestamp <= self._last_joint_stamp:
                    continue
                self._last_joint_stamp = sample.timestamp
                self._last_feedback = now
                self._direct_can_feedback_count += 1
                self._arm = list(sample.positions)
                if now - self._last_feedback_publish >= self._feedback_publish_period:
                    self._last_feedback_publish = now
                    arm = list(self._arm)
                    publish = True
            if publish:
                output = JointState()
                output.header.stamp.sec = int(sample.timestamp)
                output.header.stamp.nanosec = int(
                    (sample.timestamp - int(sample.timestamp)) * 1e9
                )
                output.name = list(ARM_JOINTS)
                output.position = arm
                self._feedback_pub.publish(output)

    def get_joint_feedback(self) -> JointFeedback:
        """Return the last normal 0x2A5/0x2A6/0x2A7 joint cycle."""
        now = time.monotonic()
        with self._lock:
            freshness_stamp = self._last_feedback
            sample_stamp = self._last_joint_stamp
            positions = tuple(self._arm)
            valid = bool(freshness_stamp) and bool(sample_stamp)
        age = now - freshness_stamp if freshness_stamp else math.inf
        return JointFeedback(positions, sample_stamp, age,
                             valid and age <= self._playback_feedback_timeout,
                             NORMAL_FEEDBACK_SOURCE)

    def _refresh_after_scheduling_stall(self, stale: JointFeedback) -> JointFeedback:
        """Let the CAN receiver publish one genuinely new sample after a process stall."""
        if stale.source != NORMAL_FEEDBACK_SOURCE:
            return stale
        stale_stamp = stale.feedback_timestamp
        deadline = time.monotonic() + 0.020
        latest = stale
        while time.monotonic() < deadline:
            # recv() normally produces a complete sample every 5 ms. Sleeping
            # releases the GIL so the dedicated SocketCAN thread can run first
            # after an OS/RMW scheduling pause.
            time.sleep(0.001)
            latest = self.get_joint_feedback()
            if latest.valid and latest.feedback_timestamp != stale_stamp:
                self.get_logger().warning(
                    "PLAYBACK stale reading recovered after a new direct CAN sample"
                )
                return latest
        return latest

    def _set_gravity_phase(self, phase: str, detail: str = "") -> None:
        with self._lock:
            self._gravity_phase = phase
        suffix = f": {detail}" if detail else ""
        logger = self.get_logger()
        log = getattr(logger, "warning", logger.error)
        log(f"[PiperH Gravity Compensation] state={phase}{suffix}")
        self._publish_status()

    def _dump_control_state(self, _request, response):
        """Return a read-only snapshot.  This callback never transmits CAN."""
        now = time.monotonic()
        with self._lock:
            vendor = self._vendor_status
            document = {
                "control_mode": getattr(vendor, "ctrl_mode", None),
                "enabled": self._motors_enabled,
                "teach_status": getattr(vendor, "teach_status", None),
                "arm_status": getattr(vendor, "arm_status", None),
                "error_code": getattr(vendor, "err_code", None),
                "selected_robot": self._selected_robot,
                "xbox_armed": self._xbox_armed,
                "feedback_age_sec": now - self._last_feedback,
                "joint_feedback_source": NORMAL_FEEDBACK_SOURCE,
                "joint_feedback_stamp": self._last_joint_stamp,
                "grouped_feedback_count": self._grouped_feedback_count,
                "grouped_feedback_age_sec": (
                    now - self._last_grouped_feedback
                    if self._last_grouped_feedback else None
                ),
                "grouped_joint_feedback_stamp": self._last_grouped_joint_stamp,
                "direct_can_feedback_count": self._direct_can_feedback_count,
                "direct_can_dropped_cycles": self._direct_can_grouper.dropped_cycles,
                "direct_can_wide_span_cycles": self._direct_can_grouper.wide_span_cycles,
                "driver_feedback_age_sec": (
                    now - self._last_driver_feedback
                    if self._last_driver_feedback else None
                ),
                "driver_joint_feedback_stamp": self._last_driver_joint_stamp,
                "driver_feedback_count": self._driver_feedback_count,
                "gravity_phase": self._gravity_phase,
                "gravity_real_torque_enabled": bool(
                    self.get_parameter("gravity_real_torque_enabled").value
                ),
                "gravity_mit_support_confirmed": bool(
                    self.get_parameter("gravity_mit_support_confirmed").value
                ),
                "gravity_base_mount_confirmed": bool(
                    self.get_parameter("gravity_base_mount_confirmed").value
                ),
                "expected_firmware": str(
                    self.get_parameter("gravity_expected_firmware").value
                ),
            }
        response.success = True
        response.message = json.dumps(document, ensure_ascii=False, sort_keys=True)
        self.get_logger().warning(f"[PIPERH CONTROL STATE] {response.message}")
        return response

    def _publish_status(self) -> None:
        """Expose the generic safety state consumed by the shared teach UI."""
        with self._lock:
            now = time.monotonic()
            feedback_age = math.inf
            gravity_active = self._gravity_active
            gravity_transition = self._gravity_transition
            gravity_phase = self._gravity_phase
            gravity_fault = self._gravity_fault
            feedback_age = now - self._last_feedback
            feedback_fresh = feedback_age <= self._timeout
            # A stale sample immediately closes the command gate through
            # control_loop_active/_publish_driver.  Keep the operator's enable
            # acknowledgement across a short scheduling/CAN gap, however, so
            # opening another motion file does not require an unrelated second
            # enable click.  A sustained disconnect still invalidates it.
            latch_timeout = getattr(
                self, "_motor_enable_latch_timeout", self._timeout
            )
            if feedback_age > latch_timeout:
                self._motors_enabled = False
            enabled = self._motors_enabled
        message = ArmStatus()
        message.header.stamp = self.get_clock().now().to_msg()
        message.mode = "PIPER_CAN"
        message.enabled = enabled
        message.control_loop_active = enabled and feedback_fresh and not gravity_transition
        message.state_machine = (
            gravity_phase if gravity_transition else
            "GRAVITY_COMPENSATION_ACTIVE" if gravity_active else
            "IDLE" if enabled else "DISABLED"
        )
        message.joint_names = list(ARM_JOINTS)
        message.per_joint_status_code = [0] * len(ARM_JOINTS)
        message.error_codes = (
            (["STALE_FEEDBACK"] if not feedback_fresh else []) +
            # A gravity-loop failure is returned by the start/stop service and
            # remains in the teach UI message.  Once the adapter has safely
            # returned to IDLE it is history, not a live arm fault; reporting
            # it here would make the teach preflight block every retry.
            ([gravity_fault] if gravity_fault and
             (gravity_active or gravity_transition) else [])
        )
        self._status_pub.publish(message)

    def _feedback_fresh(self) -> bool:
        with self._lock:
            return time.monotonic() - self._last_feedback <= self._timeout

    def _publish_driver_authority(self) -> None:
        message = String()
        message.data = json.dumps({
            "owner": self._driver_authority_id,
            "lease_seconds": self._authority_lease,
        })
        self._driver_authority_pub.publish(message)

    def _publish_driver(self, arm: Sequence[float], authority: AuthorityToken | None) -> bool:
        with self._lock:
            if not self._command_authority.validate(authority):
                self.get_logger().error(
                    "motion command suppressed: no live Piper command authority"
                )
                return False
            feedback_fresh = time.monotonic() - self._last_feedback <= self._timeout
            enabled = self._motors_enabled
            gravity_busy = self._gravity_active or self._gravity_transition
            if gravity_busy:
                self.get_logger().warning("position command suppressed during gravity compensation")
                return False
            reason = command_gate_reason(feedback_fresh, enabled)
            if reason is not None:
                self.get_logger().error(f"{reason}; command suppressed")
                return False
            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = self._driver_authority_id
            message.name = list(ARM_JOINTS)
            message.position = [float(value) for value in arm]
            # The upstream driver uses velocity[6] as one global 1..100 speed
            # percentage, even when gripper_exist is false.
            message.velocity = [0.0] * 6 + [float(self._driver_speed)]
            message.effort = [0.0] * 6
            self._command_pub.publish(message)
            return True

    def _stream(self, message: JointTrajectory) -> None:
        if not message.points:
            return
        arm = bounded_arm(message.joint_names, message.points[-1].positions)
        if arm is None:
            self.get_logger().error("invalid or out-of-range Servo command rejected")
            return
        with self._lock:
            authority = self._servo_authority
            if authority is not None:
                self._command_authority.renew(
                    authority, lease_seconds=self._authority_lease)
        self._publish_driver(arm, authority)

    def _arm_goal(self, goal) -> GoalResponse:
        with self._lock:
            if self._gravity_active or self._gravity_transition or self._action_active:
                return GoalResponse.REJECT
            trajectory = goal.trajectory
            valid = bool(trajectory.points) and all(
                bounded_arm(trajectory.joint_names, point.positions) is not None
                for point in trajectory.points
            )
            if valid:
                authority = self._command_authority.acquire(
                    "action", lease_seconds=self._authority_lease)
                if authority is None:
                    return GoalResponse.REJECT
                self._action_authority = authority
                self._action_active = True
            return GoalResponse.ACCEPT if valid else GoalResponse.REJECT

    @staticmethod
    def _duration(point) -> float:
        return point.time_from_start.sec + point.time_from_start.nanosec * 1e-9

    def _execute_arm(self, goal_handle):
        with self._lock:
            self._action_active = True
            authority = self._action_authority
        try:
            return self._execute_arm_locked(goal_handle)
        finally:
            with self._lock:
                self._action_active = False
                self._command_authority.release(authority)
                if self._action_authority == authority:
                    self._action_authority = None

    def _execute_arm_locked(self, goal_handle):
        result = FollowJointTrajectory.Result()
        trajectory = goal_handle.request.trajectory
        try:
            ordered = [bounded_arm(trajectory.joint_names, p.positions) for p in trajectory.points]
            prepared = prepare_trajectory(
                [self._duration(p) for p in trajectory.points], ordered,
                velocity_limit=self._playback_velocity,
                acceleration_limit=self._playback_acceleration,
                rate_hz=self._playback_rate, lower=LOWER, upper=UPPER,
            )
        except (ValueError, TypeError) as error:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = f"INVALID_TRAJECTORY: {error}"
            return result
        feedback = self.get_joint_feedback()
        if not feedback.valid:
            feedback = self._refresh_after_scheduling_stall(feedback)
        if not feedback.valid or feedback.source != NORMAL_FEEDBACK_SOURCE:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
            result.error_string = f"FEEDBACK_STALE: source={feedback.source} age={feedback.feedback_age:.3f}s"
            return result
        path = Path(str(self.get_parameter("playback_tracking_dir").value)) / (
            f"playback_tracking_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}.csv")
        try:
            tracking = TrackingLog(path)
        except OSError as error:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = f"TRACKING_LOG_FAILED: {error}"
            return result
        self.get_logger().info(
            f"PLAYBACK START motion={trajectory.header.frame_id or '(action goal)'} "
            f"raw_duration={prepared.times[-1]:.3f}s processed_duration={prepared.duration:.3f}s "
            f"raw_samples={len(prepared.times)} interpolation_frequency={self._playback_rate:.1f}Hz "
            f"velocity_limit={self._playback_velocity:.3f}rad/s "
            f"acceleration_limit={self._playback_acceleration:.3f}rad/s2 "
            f"joint_tolerance={self._playback_tolerance:.3f}rad "
            f"feedback_source={feedback.source} feedback_age={feedback.feedback_age:.3f}s "
            f"vel_all={self._driver_speed} vel_all_mode=fixed "
            f"tracking_csv={path}"
        )
        period = 1.0 / self._playback_rate
        start = time.monotonic()
        next_tick = start
        last_wall_tick = start
        elapsed = 0.0
        last_report = start - 1.0
        abort_since = None
        stable = 0
        last_feedback_stamp = feedback.feedback_timestamp
        status = "ABORTED"
        final_target = prepared.sample(prepared.duration)
        last_actual = feedback.joint_positions
        last_errors = tuple(0.0 for _ in range(6))
        gc_was_enabled = gc.isenabled()
        gc.collect()
        if gc_was_enabled:
            # Cyclic GC stops every Python thread in this process. Refcounting
            # remains active, and the fixed-loop objects do not require cyclic
            # collection. Restore GC as soon as this replay finishes.
            gc.disable()
        try:
            while rclpy.ok():
                now = time.monotonic()
                if now < next_tick:
                    time.sleep(min(next_tick - now, period))
                    continue
                next_tick = max(next_tick + period, now + period * 0.1)
                elapsed += bounded_trajectory_step(now - last_wall_tick, period)
                last_wall_tick = now
                settling = elapsed >= prepared.duration
                target = final_target if settling else prepared.sample(elapsed)
                if goal_handle.is_cancel_requested:
                    status = "CANCELED"
                    goal_handle.canceled()
                    return result
                feedback = self.get_joint_feedback()
                if not feedback.valid:
                    feedback = self._refresh_after_scheduling_stall(feedback)
                if not feedback.valid or feedback.source != NORMAL_FEEDBACK_SOURCE:
                    status = "FEEDBACK_STALE"
                    result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                    result.error_string = f"FEEDBACK_STALE: source={feedback.source} age={feedback.feedback_age:.3f}s"
                    goal_handle.abort()
                    return result
                with self._lock:
                    authority = self._action_authority
                    if authority is not None:
                        self._command_authority.renew(
                            authority, lease_seconds=self._authority_lease)
                if (bounded_arm(ARM_JOINTS, target) is None or
                        not self._publish_driver(target, authority)):
                    status = "ABORTED"
                    result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                    result.error_string = "COMMAND_SUPPRESSED: feedback, enable, or mode gate changed"
                    goal_handle.abort()
                    return result
                last_actual = feedback.joint_positions
                last_errors = tracking.add(elapsed, target, last_actual, feedback.feedback_age,
                                           "SETTLE" if settling else "TRACK")
                max_error = max(abs(x) for x in last_errors)
                worst_joint = 1 + max(range(6), key=lambda j: abs(last_errors[j]))
                action_feedback = FollowJointTrajectory.Feedback()
                action_feedback.joint_names = list(ARM_JOINTS)
                action_feedback.desired.positions = list(target)
                action_feedback.actual.positions = list(last_actual)
                action_feedback.error.positions = list(last_errors)
                action_feedback.desired.time_from_start.sec = int(elapsed)
                action_feedback.desired.time_from_start.nanosec = int((elapsed % 1.0) * 1e9)
                goal_handle.publish_feedback(action_feedback)
                if now - last_report >= 1.0:
                    self.get_logger().info(
                        f"PLAYBACK t={elapsed:.2f}/{prepared.duration:.2f}s "
                        f"max_error={max_error:.3f}rad joint=J{worst_joint} "
                        f"feedback_age={feedback.feedback_age:.3f}s phase={'SETTLE' if settling else 'TRACK'}")
                    last_report = now
                if max_error >= self._tracking_warn and not settling and now - last_report < period:
                    self.get_logger().warning(f"PLAYBACK tracking warning J{worst_joint} error={max_error:.3f}rad")
                if max_error >= self._tracking_abort and not settling:
                    abort_since = now if abort_since is None else abort_since
                    if now - abort_since >= self._tracking_abort_duration:
                        status = "ABORTED"
                        result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                        result.error_string = f"TRACKING_ABORT: J{worst_joint} error={max_error:.3f}rad"
                        goal_handle.abort()
                        return result
                else:
                    abort_since = None
                if settling:
                    if feedback.feedback_timestamp != last_feedback_stamp:
                        stable = stable + 1 if max_error < self._playback_tolerance else 0
                    if stable >= self._playback_stable_cycles:
                        status = "SUCCESS"
                        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                        goal_handle.succeed()
                        return result
                    if elapsed - prepared.duration >= self._playback_settle_timeout:
                        status = "TRACKING_TIMEOUT"
                        result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                        result.error_string = (
                            f"TRACKING_TIMEOUT: final_target={list(final_target)} "
                            f"final_actual={list(last_actual)} final_error={list(last_errors)}")
                        goal_handle.abort()
                        return result
                last_feedback_stamp = feedback.feedback_timestamp
            status = "ABORTED"
            result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
            result.error_string = "ROS_SHUTDOWN"
            goal_handle.abort()
            return result
        finally:
            if gc_was_enabled:
                gc.enable()
            tracking.close()
            self.get_logger().info(f"PLAYBACK RESULT status={status} csv={path}\n{tracking.summary()}")

    def _xbox_status(self, message: Bool) -> None:
        with self._lock:
            self._xbox_armed = bool(message.data)
            self._sync_servo_authority_locked()

    def _vendor_status_callback(self, message: PiperStatusMsg) -> None:
        with self._lock:
            self._vendor_status = message
            self._last_vendor_status = time.monotonic()

    def _selected_robot_status(self, message: String) -> None:
        with self._lock:
            self._selected_robot = message.data
            self._sync_servo_authority_locked()

    def _sync_servo_authority_locked(self) -> None:
        """Treat explicit Piper selection + Xbox arm as Servo acquisition."""
        should_own = self._selected_robot == "piperh" and self._xbox_armed is True
        if not should_own:
            self._command_authority.release(self._servo_authority)
            self._servo_authority = None
            return
        if self._servo_authority is not None and self._command_authority.renew(
            self._servo_authority, lease_seconds=self._authority_lease
        ):
            return
        self._servo_authority = self._command_authority.acquire(
            "servo", lease_seconds=self._authority_lease
        )

    def _gravity_parameters(self):
        if not bool(self.get_parameter("gravity_real_torque_enabled").value):
            raise RuntimeError("real torque output is disabled; use the dry-run executable")
        if not bool(self.get_parameter("gravity_mit_support_confirmed").value):
            raise RuntimeError("Piper-H MIT support is not field-confirmed")
        if not bool(self.get_parameter("gravity_base_mount_confirmed").value):
            raise RuntimeError("Piper-H base mount orientation is not confirmed")
        expected_firmware = str(self.get_parameter("gravity_expected_firmware").value).strip()
        if not expected_firmware:
            raise ValueError("gravity_expected_firmware must be set after field calibration")
        from .mit_backend import firmware_profile
        firmware_profile(expected_firmware)
        arrays = {}
        arrays["gravity_mount_rpy"] = np.asarray([
            self.get_parameter("gravity_mount_roll").value,
            self.get_parameter("gravity_mount_pitch").value,
            self.get_parameter("gravity_mount_yaw").value,
        ], dtype=float)
        if not np.all(np.isfinite(arrays["gravity_mount_rpy"])):
            raise ValueError("gravity mount orientation must be finite")
        payload_mass = float(self.get_parameter("gravity_payload_mass_kg").value)
        payload_com = np.asarray(
            self.get_parameter("gravity_payload_com_xyz_m").value, dtype=float
        )
        if (
            not math.isfinite(payload_mass)
            or not 0.0 <= payload_mass <= 3.0
            or payload_com.shape != (3,)
            or not np.all(np.isfinite(payload_com))
            or np.any(np.abs(payload_com) > 0.5)
        ):
            raise ValueError("invalid gravity payload mass or center of mass")
        arrays["gravity_payload_mass_kg"] = payload_mass
        arrays["gravity_payload_com_xyz_m"] = payload_com
        scale = float(self.get_parameter("gravity_torque_scale").value)
        if not math.isfinite(scale) or not 0.0 < scale <= 0.10:
            raise ValueError("first-stage gravity_torque_scale must be within (0, 0.10]")
        arrays["gravity_torque_scale"] = scale
        for name in ("gravity_kp", "gravity_kd", "gravity_max_torque_nm"):
            value = np.asarray(self.get_parameter(name).value, dtype=float)
            if value.shape != (6,) or not np.all(np.isfinite(value)):
                raise ValueError(f"{name} must contain six finite values")
            arrays[name] = value
        if np.any(arrays["gravity_kp"] < 0) or np.any(arrays["gravity_kp"] > 20):
            raise ValueError("gravity_kp must be within 0..20")
        if np.any(arrays["gravity_kd"] < 0) or np.any(arrays["gravity_kd"] > 2):
            raise ValueError("gravity_kd must be within 0..2")
        if np.any(arrays["gravity_max_torque_nm"] <= 0):
            raise ValueError("gravity_max_torque_nm is unconfigured; do not guess limits")
        rate = float(self.get_parameter("gravity_rate_hz").value)
        if not 20 <= rate <= 100:
            raise ValueError("gravity_rate_hz must be within 20..100")
        max_velocity = float(
            self.get_parameter("gravity_max_velocity_rad_s").value
        )
        if not 0.1 <= max_velocity <= 3.0:
            raise ValueError("gravity_max_velocity_rad_s must be within 0.1..3.0")
        arrays["gravity_max_velocity_rad_s"] = max_velocity
        max_duration = float(self.get_parameter("gravity_max_duration_sec").value)
        if not 0.0 <= max_duration <= 3600.0:
            raise ValueError("gravity_max_duration_sec must be within 0..3600")
        return arrays, rate, max_duration

    def _start_gravity_compensation(self, _request, response):
        with self._mode_lock:
            with self._lock:
                now = time.monotonic()
                reason = command_gate_reason(
                    now - self._last_feedback <= self._timeout,
                    self._motors_enabled,
                )
                if self._gravity_active or self._gravity_transition:
                    reason = "gravity compensation is already active"
                elif self._action_active:
                    reason = "trajectory is active"
                elif self._xbox_armed is not False:
                    reason = "Xbox must report LOCKED"
                elif (
                    self.get_parameter("gravity_require_selected_robot").value
                    and self._selected_robot != "piperh"
                ):
                    reason = "Piper-H is not the selected robot"
                elif now - self._last_vendor_status > self._timeout:
                    reason = "Piper vendor status is missing or stale"
                elif (
                    self._vendor_status is None
                    or self._vendor_status.ctrl_mode not in (0, 1)
                    or self._vendor_status.teach_status not in (0, 2)
                    or self._vendor_status.arm_status != 0
                    or self._vendor_status.err_code != 0
                ):
                    reason = "Piper vendor is not in a healthy idle/CAN mode"
                elif bounded_arm(ARM_JOINTS, self._arm) is None:
                    reason = "joint feedback is invalid or outside limits"
                if reason:
                    self.get_logger().warning(
                        "[PiperH Gravity Compensation] start rejected: "
                        f"reason={reason}; control_mode="
                        f"{getattr(self._vendor_status, 'ctrl_mode', 'UNKNOWN')}; "
                        f"teach_status={getattr(self._vendor_status, 'teach_status', 'UNKNOWN')}; "
                        f"enabled={self._motors_enabled}; selected={self._selected_robot}; "
                        f"xbox_armed={self._xbox_armed}; "
                        f"feedback_age_sec={now - self._last_feedback:.3f}"
                    )
                    response.success = False
                    response.message = reason
                    return response
                self._gravity_authority = self._command_authority.acquire(
                    "gravity", now=now, lease_seconds=self._authority_lease)
                if self._gravity_authority is None:
                    response.success = False
                    response.message = "another Piper command owner is active"
                    return response
                self._gravity_transition = True
                self._gravity_fault = ""
                self._gravity_phase = "GRAVITY_PRECHECK"
                self.get_logger().warning(
                    "[PiperH Gravity Compensation] button/service request accepted; "
                    f"control_mode={getattr(self._vendor_status, 'ctrl_mode', None)}, "
                    f"teach_status={getattr(self._vendor_status, 'teach_status', None)}, "
                    f"enabled={self._motors_enabled}, selected={self._selected_robot}, "
                    f"xbox_armed={self._xbox_armed}"
                )
            sdk = None
            try:
                arrays, rate, max_duration = self._gravity_parameters()
                can_port = str(self.get_parameter("can_port").value)
                expected_firmware = str(
                    self.get_parameter("gravity_expected_firmware").value
                ).strip()
                actual_firmware = read_firmware(can_port)
                if actual_firmware != expected_firmware:
                    raise RuntimeError(
                        f"Piper firmware changed: expected {expected_firmware}, "
                        f"got {actual_firmware}; configured driver profile is invalid"
                    )
                urdf_path = Path(
                    get_package_share_directory("piper_h_description")
                ) / "urdf/piper_h_description.urdf"
                model = PiperHGravityCompensator(
                    str(urdf_path),
                    rotation_from_rpy(*arrays["gravity_mount_rpy"]),
                    arrays["gravity_payload_mass_kg"],
                    arrays["gravity_payload_com_xyz_m"],
                )
                sdk = OfficialMitBackend(can_port, actual_firmware)
                sdk.connect()
                initial_motor_state = sdk.wait_motor_state(timeout=1.5)
                if bounded_arm(ARM_JOINTS, initial_motor_state[0]) is None:
                    raise RuntimeError("initial Piper motor positions are outside limits")
            except Exception as error:
                with self._lock:
                    self._gravity_transition = False
                    self._gravity_phase = "ERROR"
                    self._gravity_fault = str(error)
                    self._command_authority.release(self._gravity_authority)
                    self._gravity_authority = None
                self.get_logger().error(
                    "[PiperH Gravity Compensation] start setup failed before "
                    f"any MIT command: {error}"
                )
                if sdk is not None:
                    try:
                        sdk.disconnect()
                    except Exception:
                        pass
                response.success = False
                response.message = str(error)
                return response
            with self._lock:
                self._gravity_active = True
                self._gravity_sdk = sdk
            self._gravity_stop_event.clear()
            ready = threading.Event()
            self._gravity_thread = threading.Thread(
                target=self._gravity_loop,
                args=(sdk, model, arrays, rate, max_duration, initial_motor_state, ready),
                name="piperh-gravity-compensation", daemon=True,
            )
            self._gravity_thread.start()
            ready.wait(timeout=3.0)
            if not ready.is_set():
                self._stop_gravity_loop_locked()
            with self._lock:
                response.success = self._gravity_active and not self._gravity_transition
                response.message = (
                    "Piper-H gravity compensation active" if response.success else
                    self._gravity_fault or "gravity compensation did not start"
                )
            return response

    def _stop_gravity_compensation(self, _request, response):
        with self._mode_lock:
            with self._lock:
                active = self._gravity_active or self._gravity_transition
            if not active:
                response.success = False
                response.message = "gravity compensation is not active"
                return response
            self._stop_gravity_loop_locked()
            with self._lock:
                response.success = not self._gravity_active
                response.message = self._gravity_fault or "Piper-H gravity compensation stopped"
            return response

    def _stop_gravity_loop(self) -> None:
        with self._mode_lock:
            self._stop_gravity_loop_locked()

    def _stop_gravity_loop_locked(self) -> None:
        self._gravity_stop_event.set()
        thread = self._gravity_thread
        if thread and thread.is_alive():
            thread.join(timeout=2.0)
        if thread and thread.is_alive():
            with self._lock:
                self._gravity_fault = "gravity loop did not stop within 2 seconds"
        self._gravity_thread = None

    def _gravity_loop(
        self, sdk, model, arrays, rate, max_duration, initial_motor_state, ready
    ) -> None:
        interval = 1.0 / rate
        started = time.monotonic()
        last_q = np.asarray(initial_motor_state[0], dtype=float)
        last_motor_stamp = float(initial_motor_state[3])
        last_motor_progress = time.monotonic()
        mit_entered = False
        torque_limited = False
        mit_writer = AuthoritativeWriter(
            self._command_authority, lambda command: sdk.send_joint(*command)
        )
        try:
            while not self._gravity_stop_event.is_set():
                tick = time.monotonic()
                if max_duration > 0.0 and tick - started >= max_duration:
                    raise RuntimeError("gravity field-trial duration reached")
                with self._lock:
                    if not self._command_authority.renew(
                        self._gravity_authority,
                        now=tick,
                        lease_seconds=self._authority_lease,
                    ):
                        raise RuntimeError("gravity command authority was lost")
                    fresh = tick - self._last_feedback <= self._timeout
                    enabled = self._motors_enabled
                    xbox_locked = self._xbox_armed is False
                    selected = (
                        not self.get_parameter("gravity_require_selected_robot").value
                        or self._selected_robot == "piperh"
                    )
                    vendor_safe = (
                        tick - self._last_vendor_status <= self._timeout and
                        self._vendor_status is not None and
                        self._vendor_status.ctrl_mode in (0, 1) and
                        self._vendor_status.teach_status in (0, 2) and
                        self._vendor_status.arm_status == 0 and
                        self._vendor_status.err_code == 0
                    )
                if not fresh or not enabled or not xbox_locked or not vendor_safe or not selected:
                    raise RuntimeError(
                        "feedback, motor enable, Xbox LOCKED, selection or vendor status was lost"
                    )
                if not mit_entered:
                    initial_torque = limit_torque(
                        model.torque(last_q, np.zeros(6)),
                        arrays["gravity_torque_scale"],
                        arrays["gravity_max_torque_nm"],
                    )
                    # Prime every motor reference while still in MOVE_J so the
                    # transition cannot leave any joint without a valid frame.
                    for joint in range(6):
                        if not mit_writer.write(
                            self._gravity_authority,
                            (
                                joint + 1,
                                float(last_q[joint]),
                                float(arrays["gravity_kp"][joint]),
                                float(arrays["gravity_kd"][joint]),
                                float(initial_torque[joint]),
                            ),
                        ):
                            raise RuntimeError(
                                "gravity command authority was lost"
                            )
                    sdk.enter_mit()
                    mit_entered = True
                positions, velocities, efforts, motor_stamp = sdk.read_motor_state()
                q = np.asarray(positions, dtype=float)
                velocity = np.asarray(velocities, dtype=float)
                effort = np.asarray(efforts, dtype=float)
                if not np.all(np.isfinite(velocity)) or not np.all(np.isfinite(effort)):
                    raise RuntimeError("invalid Piper-H MIT motor feedback")
                if last_motor_stamp is None or motor_stamp > last_motor_stamp:
                    last_motor_stamp = motor_stamp
                    last_motor_progress = tick
                elif tick - last_motor_progress > self._timeout:
                    raise RuntimeError("Piper-H MIT motor feedback is stale")
                if bounded_arm(ARM_JOINTS, q) is None:
                    raise RuntimeError("joint feedback is outside Piper-H limits")
                max_velocity = float(
                    arrays.get("gravity_max_velocity_rad_s", 0.6)
                )
                peak_joint = int(np.argmax(np.abs(velocity)))
                if abs(velocity[peak_joint]) > max_velocity:
                    raise RuntimeError(
                        f"joint {peak_joint + 1} velocity "
                        f"{abs(velocity[peak_joint]):.3f} rad/s exceeded "
                        f"{max_velocity:.3f} rad/s gravity-mode limit"
                    )
                last_q = q
                raw_torque = model.torque(q, velocity)
                torque = limit_torque(
                    raw_torque,
                    arrays["gravity_torque_scale"],
                    arrays["gravity_max_torque_nm"],
                )
                now_limited = not np.allclose(
                    arrays["gravity_torque_scale"] * raw_torque, torque
                )
                if now_limited and not torque_limited:
                    self.get_logger().warning(
                        "gravity torque reached a configured limit; "
                        "continuing with bounded torque"
                    )
                torque_limited = now_limited
                for joint in range(6):
                    if not mit_writer.write(
                        self._gravity_authority,
                        (
                            joint + 1,
                            float(q[joint]),
                            float(arrays["gravity_kp"][joint]),
                            float(arrays["gravity_kd"][joint]),
                            float(torque[joint]),
                        ),
                    ):
                        raise RuntimeError(
                            "gravity command authority was lost"
                        )
                with self._lock:
                    self._gravity_transition = False
                    self._gravity_phase = "GRAVITY_COMPENSATION_ACTIVE"
                ready.set()
                self._gravity_stop_event.wait(max(0.0, interval - (time.monotonic() - tick)))
        except Exception as error:
            with self._lock:
                self._gravity_fault = str(error)
            self.get_logger().error(f"Piper-H gravity compensation stopped: {error}")
        finally:
            ready.set()
            try:
                if mit_entered and last_q is not None:
                    sdk.hold(last_q)
            except Exception as error:
                with self._lock:
                    self._gravity_fault = f"position hold failed: {error}"
            finally:
                with self._lock:
                    self._gravity_active = False
                    self._gravity_transition = False
                    self._gravity_sdk = None
                    self._command_authority.release(self._gravity_authority)
                    self._gravity_authority = None
                try:
                    sdk.disconnect()
                except Exception:
                    pass
                self._publish_status()

    def destroy_node(self) -> bool:
        self._command_authority.revoke("adapter shutdown")
        self._servo_authority = None
        self._action_authority = None
        self._gravity_authority = None
        self._stop_gravity_loop()
        self._direct_can_stop.set()
        if hasattr(self, "_direct_can_bus"):
            self._direct_can_bus.shutdown()
        if hasattr(self, "_direct_can_thread"):
            self._direct_can_thread.join(timeout=1.0)
        self._arm_action.destroy()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HardwareAdapter()
    # One worker is occupied by the blocking trajectory loop. A second is
    # reserved in practice for grouped-CAN feedback, leaving one for status,
    # services, and SDK-layer diagnostics.
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
