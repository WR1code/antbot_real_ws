"""One-run, zero-history Link5 eye-in-hand calibration for Piper-H.

The node deliberately does not load an existing calibration.  It bootstraps from
small collision-checked joint motions around the operator supplied anchor, then
uses the provisional result to expand camera poses over a board-centred cap.
"""

from __future__ import annotations

from collections import Counter, deque
from dataclasses import asdict, replace
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import sys
import threading
import time

from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetMotionPlan, GetPositionFK
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .handeye_observation import (
    FrameAdmission, MotionSafetyError, Observation, OBSERVATION_TOPIC,
    anchor_drift, confirm_trajectory_stop, nearest_joint_state,
)
from .handeye_arrival import HandeyeArrivalWindow, arrival_timeout_message
from .handeye_quality import QualityConfig, evaluate_observation_quality, summarize_quality

from .adaptive_handeye import (
    HemispherePose,
    PosePair,
    Transform,
    aggregate_pose_pairs,
    atomic_write_yaml,
    board_poses,
    calibration_document,
    look_at_roll_deg,
    plan_adaptive_increment,
    pose_dispersion,
    quaternion_to_rotation,
    rotation_observability,
    rotation_log,
    rotation_to_quaternion,
    solve_handeye,
    solver_consensus,
    split_bootstrap_train_holdout,
)


JOINT_NAMES = tuple(f"joint{index}" for index in range(1, 7))
LOWER = np.asarray((-2.6179938, 0.0, -2.9670597, -2.3561944, -1.5620696, -2.0943951))
UPPER = np.asarray((2.6179938, 3.1415926, 0.0, 2.3561944, 1.5620696, 2.0943951))


def calibration_joint_margins(
    margin_fraction: float,
    locked_j6_margin_rad: float,
) -> np.ndarray:
    """Return J1-J5 fractional margins and a fixed margin for locked J6."""
    fraction = float(margin_fraction)
    j6_margin = float(locked_j6_margin_rad)
    if not math.isfinite(fraction) or not 0.0 < fraction < 0.5:
        raise ValueError("joint_margin_fraction must be finite and within (0, 0.5)")
    if (
        not math.isfinite(j6_margin)
        or j6_margin <= 0.0
        or 2.0 * j6_margin >= UPPER[5] - LOWER[5]
    ):
        raise ValueError("locked_j6_margin_rad must be finite, positive, and below half range")
    margins = fraction * (UPPER - LOWER)
    margins[5] = j6_margin
    return margins


def values_within_calibration_margins(values, margins) -> bool:
    """Check one six-joint pose against the calibration safety envelope."""
    joints = np.asarray(values, dtype=np.float64)
    margin = np.asarray(margins, dtype=np.float64)
    return bool(
        joints.shape == (6,)
        and margin.shape == (6,)
        and np.all(np.isfinite(joints))
        and np.all(np.isfinite(margin))
        and np.all(joints >= LOWER + margin)
        and np.all(joints <= UPPER - margin)
    )


def _transform(message) -> Transform:
    value = message.transform if hasattr(message, "transform") else message
    rotation = value.rotation
    translation = value.translation
    return Transform(
        quaternion_to_rotation(rotation.x, rotation.y, rotation.z, rotation.w),
        (translation.x, translation.y, translation.z),
    )


def _pose(transform: Transform) -> PoseStamped:
    output = PoseStamped()
    output.pose.position.x, output.pose.position.y, output.pose.position.z = transform.translation
    x, y, z, w = rotation_to_quaternion(transform.rotation)
    output.pose.orientation.x = x
    output.pose.orientation.y = y
    output.pose.orientation.z = z
    output.pose.orientation.w = w
    return output


class PiperHAutoHandeye(Node):
    """Guarded automatic Link5 calibration orchestrator."""

    def __init__(self) -> None:
        super().__init__("piperh_auto_handeye")
        defaults = {
            "calibration_name": "piperh_camera_eye_in_hand",
            "calibration_workspace_mode": "local",
            "robot_base_frame": "piperh/base_link",
            "robot_effector_frame": "piperh/Link5",
            "moveit_base_frame": "piperh/base_link",
            "moveit_link_name": "Link5",
            "tracking_base_frame": "camera_color_optical_frame",
            "tracking_marker_frame": "marker_frame",
            "joint_state_topic": "/piperh/joint_states",
            "camera_info_topic": "/camera/color/camera_info",
            "xbox_armed_topic": "/piperh/xbox/armed",
            "moveit_namespace": "/piperh",
            "trajectory_action": "/piperh/arm_controller/follow_joint_trajectory",
            "motion_enabled": False,
            "record_only": False,
            "sample_frames": 15,
            "sample_period_sec": 0.06,
            "min_detection_interval": 0.15,
            "max_detection_age": 0.25,
            "max_future_timestamp_wait_sec": 0.5,
            "robot_tf_wait_sec": 0.1,
            "robot_visual_sync_threshold": 0.05,
            "joint_history_size": 1000,
            "joint_candidate_pool_size": 768,
            "j4_zero_exclusion_deg": 0.30,
            "observation_queue_size": 100,
            "vision_frame_timeout": 10.0,
            "anchor_translation_threshold": 0.0011,
            "anchor_rotation_threshold": 0.25,
            "anchor_goal_tolerance_rad": 0.0005,
            "motion_stop_timeout": 1.5,
            "quality_min_board_area": 0.01,
            "quality_max_reprojection_error": 3.0,
            "quality_min_corner_distribution": 0.005,
            "quality_reprojection_epsilon": 0.1,
            "quality_area_weight": 0.4,
            "quality_reprojection_weight": 0.03,
            "quality_distribution_weight": 0.3,
            "settle_sec": 0.8,
            "joint_arrival_tolerance_deg": 0.11,
            "stable_duration": 1.0,
            "stable_position_variation_deg": 0.01,
            "arrival_timeout_sec": 10.0,
            "velocity_scaling": 0.08,
            "bootstrap_goal_tolerance_rad": 0.0005,
            "bootstrap_velocity_scaling": 0.4,
            "bootstrap_acceleration_scaling": 0.25,
            "bootstrap_convergence_retries": 1,
            "planning_time_sec": 4.0,
            "service_timeout_sec": 12.0,
            "trajectory_timeout_sec": 35.0,
            "bootstrap_max_samples": 18,
            "bootstrap_holdout_samples": 3,
            "bootstrap_min_sv_ratio": 0.06,
            "bootstrap_min_maximum_rotation_deg": 12.0,
            "bootstrap_min_translation_span_m": 0.025,
            "bootstrap_max_solver_translation_m": 0.040,
            "bootstrap_max_solver_rotation_deg": 5.0,
            "bootstrap_max_holdout_translation_m": 0.035,
            "bootstrap_max_holdout_rotation_deg": 5.0,
            "camera_mount_max_radius_m": 0.25,
            "joint_margin_fraction": 0.08,
            "locked_j6_margin_rad": 0.10,
            "round_calibration_samples": 8,
            "selection_samples": 10,
            "test_samples": 20,
            "minimum_valid_test_samples": 16,
            "test_max_translation_p95_m": 0.030,
            "test_max_rotation_p95_deg": 4.0,
            "maximum_calibration_samples": 80,
            "incremental_samples": 8,
            "convergence_rounds": 2,
            "convergence_improvement_fraction": 0.05,
            "convergence_translation_delta_m": 0.0015,
            "convergence_rotation_delta_deg": 0.25,
            "charuco_board_center_xyz": [0.0875, 0.1225, 0.0],
            "board_outer_corners_xyz": [
                0.0, 0.0, 0.0, 0.175, 0.0, 0.0,
                0.175, 0.245, 0.0, 0.0, 0.245, 0.0,
            ],
            "fov_margin_px": 20.0,
            "minimum_projected_board_area_px2": 12000.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        for name in (
            "min_detection_interval", "max_detection_age", "max_future_timestamp_wait_sec",
            "robot_tf_wait_sec", "robot_visual_sync_threshold",
            "vision_frame_timeout", "anchor_translation_threshold",
            "anchor_rotation_threshold", "anchor_goal_tolerance_rad", "motion_stop_timeout",
            "joint_arrival_tolerance_deg", "stable_duration",
            "stable_position_variation_deg", "arrival_timeout_sec",
            "bootstrap_goal_tolerance_rad", "bootstrap_velocity_scaling",
            "bootstrap_acceleration_scaling",
        ):
            value = float(self.get_parameter(name).value)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        for name in ("bootstrap_velocity_scaling", "bootstrap_acceleration_scaling"):
            if float(self.get_parameter(name).value) > 1.0:
                raise ValueError(f"{name} must not exceed 1.0")
        if int(self.get_parameter("bootstrap_convergence_retries").value) < 0:
            raise ValueError("bootstrap_convergence_retries must not be negative")
        calibration_joint_margins(
            float(self.get_parameter("joint_margin_fraction").value),
            float(self.get_parameter("locked_j6_margin_rad").value),
        )
        for name in ("joint_history_size", "observation_queue_size", "joint_candidate_pool_size"):
            if int(self.get_parameter(name).value) < 1:
                raise ValueError(f"{name} must be positive")
        j4_zero_exclusion = float(self.get_parameter("j4_zero_exclusion_deg").value)
        if not math.isfinite(j4_zero_exclusion) or not 0.0 <= j4_zero_exclusion <= 5.0:
            raise ValueError("j4_zero_exclusion_deg must be finite and within 0..5 degrees")
        self._quality_config = QualityConfig(**{
            name: float(self.get_parameter("quality_" + name).value)
            for name in QualityConfig.__dataclass_fields__
        })
        self._quality_results = []
        self._record_only = bool(self.get_parameter("record_only").value)
        self._workspace_mode = str(
            self.get_parameter("calibration_workspace_mode").value
        ).strip().lower()
        if self._workspace_mode not in ("global", "local"):
            raise ValueError("calibration_workspace_mode must be 'global' or 'local'")
        self._group = ReentrantCallbackGroup()
        self._base = str(self.get_parameter("robot_base_frame").value).lstrip("/")
        self._link5 = str(self.get_parameter("robot_effector_frame").value).lstrip("/")
        self._camera = str(self.get_parameter("tracking_base_frame").value).lstrip("/")
        self._marker = str(self.get_parameter("tracking_marker_frame").value).lstrip("/")
        self._moveit_base = str(self.get_parameter("moveit_base_frame").value).lstrip("/")
        self._moveit_link = str(self.get_parameter("moveit_link_name").value).lstrip("/")
        if not self._link5.endswith("Link5"):
            raise ValueError("Piper-H automatic eye-in-hand calibration requires Link5")
        self._lock = threading.Lock()
        self._stop_lock = threading.Lock()
        self._joint_history = deque(maxlen=int(self.get_parameter("joint_history_size").value))
        self._observations = deque(maxlen=int(self.get_parameter("observation_queue_size").value))
        self._frame_admission = FrameAdmission(
            float(self.get_parameter("min_detection_interval").value),
            float(self.get_parameter("max_detection_age").value),
        )
        self._active_result = None
        self._pending_goal = None
        self._anchor_robot_pose = None
        self._anchor_visual_pose = None
        self._arrival_condition = threading.Condition()
        self._arrival_feedback = deque(maxlen=1000)
        self._arrival_latest = None
        self._arrival_last_stamp = 0
        self._positions: tuple[float, ...] | None = None
        self._velocities: tuple[float, ...] | None = None
        self._joint_stamp = 0.0
        self._xbox_known = False
        self._xbox_armed = True
        self._camera_info_hash: str | None = None
        self._camera_info_changed = False
        self._camera_info_document: dict | None = None
        self._stopping = threading.Event()
        self._active_goal = None
        self._worker: threading.Thread | None = None
        self._sample_count = 0
        self._required_count = 1
        self._run_dir: Path | None = None
        self._base_T_target: Transform | None = None

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._status = self.create_publisher(String, "/rebotarm/handeye/auto_status", qos)
        self.create_subscription(
            String, OBSERVATION_TOPIC, self._observation, 10, callback_group=self._group,
        )
        self.create_subscription(
            JointState, str(self.get_parameter("joint_state_topic").value),
            self._joint_state, qos_profile_sensor_data, callback_group=self._group,
        )
        self.create_subscription(
            CameraInfo, str(self.get_parameter("camera_info_topic").value),
            self._camera_info, qos_profile_sensor_data, callback_group=self._group,
        )
        self.create_subscription(
            Bool, str(self.get_parameter("xbox_armed_topic").value),
            self._armed, qos, callback_group=self._group,
        )
        self._buffer = Buffer(cache_time=Duration(seconds=20.0))
        self._listener = TransformListener(self._buffer, self, spin_thread=False)
        prefix = str(self.get_parameter("moveit_namespace").value).rstrip("/")
        self._ik = None  # Link5 candidates are generated by FK in joint space.
        if self._record_only:
            self._fk = self._planner = self._trajectory = None
        else:
            self._fk = self.create_client(
                GetPositionFK, f"{prefix}/compute_fk", callback_group=self._group
            )
            self._planner = self.create_client(
                GetMotionPlan, f"{prefix}/plan_kinematic_path", callback_group=self._group
            )
            self._trajectory = ActionClient(
                self, FollowJointTrajectory,
                str(self.get_parameter("trajectory_action").value), callback_group=self._group,
            )
        self.create_service(
            Trigger, "/rebotarm/handeye/cancel", self._cancel,
            callback_group=self._group,
        )
        self.create_timer(0.5, self._start_once, callback_group=self._group)
        self.get_logger().info(
            f"startup mode={'record_only' if self._record_only else 'automatic'}; "
            f"motion_enabled={self.get_parameter('motion_enabled').value}"
        )
        self._publish("precheck", "等待仅记录预检" if self._record_only else "等待自动标定预检")

    def destroy_node(self):
        self._stopping.set()
        if self._record_only and self._worker is not None:
            self._worker.join(timeout=5.0)
            if self._worker.is_alive():
                self.get_logger().error("record-only session finalization did not finish within 5 seconds")
        try:
            self._confirm_motion_stopped()
        except MotionSafetyError as error:
            self.get_logger().error(str(error))
        return super().destroy_node()

    def _cancel(self, _request, response):
        self._stopping.set()
        try:
            self._confirm_motion_stopped()
            response.success = True
            response.message = "record-only stop requested" if self._record_only else "trajectory stop confirmed"
        except MotionSafetyError as error:
            response.success = False
            response.message = str(error)
        return response

    def _confirm_motion_stopped(self):
        with self._stop_lock:
            timeout = float(self.get_parameter("motion_stop_timeout").value)
            with self._lock:
                pending = self._pending_goal
            if pending is not None:
                deadline = time.monotonic() + timeout
                while not pending.done() and time.monotonic() < deadline:
                    time.sleep(0.01)
                if not pending.done():
                    raise MotionSafetyError("MOTION_STOP_UNCONFIRMED: goal acceptance unresolved")
                try:
                    handle = pending.result()
                    if handle is None:
                        raise MotionSafetyError("MOTION_STOP_UNCONFIRMED: missing goal response")
                    if handle.accepted:
                        with self._lock:
                            self._active_goal = handle
                            if self._active_result is None:
                                self._active_result = handle.get_result_async()
                    with self._lock:
                        self._pending_goal = None
                except Exception as error:
                    raise MotionSafetyError(f"MOTION_STOP_UNCONFIRMED: {error}") from error
            with self._lock:
                handle, result = self._active_goal, self._active_result
            if handle is None:
                return
            if result is None:
                result = handle.get_result_async()
            confirm_trajectory_stop(handle, result, timeout)
            with self._lock:
                if self._active_goal is handle:
                    self._active_goal = None
                    self._active_result = None

    def _goal_response(self, future):
        # Goal acceptance can arrive after our timeout. Keep the late handle
        # and request cancellation even if the worker has already aborted.
        try:
            handle = future.result()
            if handle is None or not handle.accepted:
                return
            with self._lock:
                if self._pending_goal is not future:
                    # The worker already adopted/finished this goal. A delayed
                    # callback must not resurrect a cleared terminal handle.
                    return
                self._active_goal = handle
                if self._active_result is None:
                    self._active_result = handle.get_result_async()
            if self._stopping.is_set():
                self._cancel_active_goal()
        except Exception as error:
            self.get_logger().error(f"MOTION_STOP_UNCONFIRMED: goal response: {error}")

    def _observation(self, message):
        try:
            item = Observation.from_json(message.data, self.get_clock().now().nanoseconds)
        except (ValueError, TypeError, KeyError) as error:
            self.get_logger().warning(f"invalid hand-eye observation: {error}")
            return
        if item.frame_id.lstrip("/") != self._camera or item.marker_frame != self._marker:
            return
        with self._lock:
            self._observations.append(item)

    def _cancel_active_goal(self) -> None:
        with self._lock:
            handle = self._active_goal
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception as error:
                self.get_logger().warning(f"trajectory cancellation failed: {error}")

    def _publish(self, state: str, message: str, **extra) -> None:
        document = {
            "state": state,
            "message": message,
            "calibration_type": "eye_in_hand",
            "sampled_poses": self._sample_count,
            "required_poses": self._required_count,
            "mode": "piperh_link5_adaptive",
            **extra,
        }
        output = String()
        output.data = json.dumps(document, ensure_ascii=False)
        self._status.publish(output)

    def _joint_state(self, message: JointState) -> None:
        lookup = {name.split("/")[-1]: index for index, name in enumerate(message.name)}
        if any(name not in lookup for name in JOINT_NAMES):
            return
        if len(message.position) != len(message.name):
            return
        values = tuple(float(message.position[lookup[name]]) for name in JOINT_NAMES)
        if not all(math.isfinite(value) for value in values):
            return
        with self._lock:
            self._positions = values
            self._velocities = (
                tuple(float(message.velocity[lookup[name]]) for name in JOINT_NAMES)
                if len(message.velocity) == len(message.name) else None
            )
            self._joint_stamp = time.monotonic()
            stamp = int(message.header.stamp.sec) * 1000000000 + int(message.header.stamp.nanosec)
            if stamp > 0:
                self._joint_history.append({
                    "timestamp": stamp, "names": list(JOINT_NAMES),
                    "positions": list(values),
                    "velocities": list(self._velocities) if self._velocities is not None else None,
                })

        if not self._record_only:
            # Use the same real CAN-derived feedback and units as calibration targets/TF.
            self._arrival_joint_state(message)

    def _camera_info(self, message: CameraInfo) -> None:
        document = {
            "width": int(message.width),
            "height": int(message.height),
            "distortion_model": message.distortion_model,
            "d": [float(value) for value in message.d],
            "k": [float(value) for value in message.k],
            "r": [float(value) for value in message.r],
            "p": [float(value) for value in message.p],
            "binning_x": int(message.binning_x),
            "binning_y": int(message.binning_y),
            "roi": {
                "x_offset": int(message.roi.x_offset),
                "y_offset": int(message.roi.y_offset),
                "height": int(message.roi.height),
                "width": int(message.roi.width),
                "do_rectify": bool(message.roi.do_rectify),
            },
        }
        digest = hashlib.sha256(
            json.dumps(document, sort_keys=True, separators=(",", ":")).encode()
        ).hexdigest()
        projection = np.asarray(message.p, dtype=np.float64).reshape((3, 4))
        valid = (
            message.width > 0 and message.height > 0
            and np.all(np.isfinite(projection))
            and projection[0, 0] > 0.0 and projection[1, 1] > 0.0
            and abs(projection[2, 2]) > 1.0e-9
        )
        if not valid:
            return
        with self._lock:
            if self._camera_info_hash is not None and digest != self._camera_info_hash:
                self._camera_info_changed = True
            else:
                self._camera_info_hash = digest
                self._camera_info_document = document

    def _armed(self, message: Bool) -> None:
        with self._lock:
            self._xbox_known = True
            self._xbox_armed = bool(message.data)
            active = self._active_goal is not None
        if message.data and active:
            self._stopping.set()
            self._cancel_active_goal()

    def _motion_guard(self) -> None:
        with self._lock:
            safe = self._xbox_known and not self._xbox_armed
            camera_changed = self._camera_info_changed
        if self._stopping.is_set():
            raise MotionSafetyError("calibration canceled")
        if not safe:
            self._stopping.set()
            self._cancel_active_goal()
            raise MotionSafetyError("Xbox is not LOCKED; automatic motion aborted")
        if camera_changed:
            self._stopping.set()
            self._cancel_active_goal()
            raise MotionSafetyError("CameraInfo changed during the run; automatic motion aborted")

    def _start_once(self) -> None:
        if self._worker is not None:
            return
        self._worker = threading.Thread(target=self._run_guarded, daemon=True)
        self._worker.start()

    def _run_guarded(self) -> None:
        try:
            if self._record_only:
                self._run_record_only()
            else:
                self._run()
        except Exception as error:
            self._stopping.set()
            try:
                self._confirm_motion_stopped()
            except MotionSafetyError as stop_error:
                error = stop_error
            mode = "record-only" if self._record_only else "automatic hand-eye"
            self.get_logger().error(f"{mode} failed: {error}")
            if self._run_dir is not None:
                try:
                    atomic_write_yaml(self._run_dir / "failure_report.yaml", {
                        "result": "FAIL",
                        "reason": str(error),
                        "sampled_poses": self._sample_count,
                        "utc": datetime.now(timezone.utc).isoformat(),
                    })
                except Exception as report_error:
                    self.get_logger().error(f"could not write failure report: {report_error}")
            self._publish("failed", str(error), error_code=str(error).split(":", 1)[0])

    def _wait_future(self, future, label: str, timeout: float | None = None):
        event = threading.Event()
        future.add_done_callback(lambda _future: event.set())
        limit = float(timeout or self.get_parameter("service_timeout_sec").value)
        if not event.wait(limit):
            future.cancel()
            raise TimeoutError(f"{label} timeout")
        result = future.result()
        if result is None:
            raise RuntimeError(f"{label} returned no result")
        return result

    def _current_positions(self) -> tuple[float, ...]:
        with self._lock:
            positions, stamp = self._positions, self._joint_stamp
        if positions is None or time.monotonic() - stamp > 0.75:
            raise RuntimeError("Piper-H joint feedback is missing or stale")
        return positions

    def _precheck(self) -> tuple[float, ...]:
        if not bool(self.get_parameter("motion_enabled").value):
            raise RuntimeError("automatic motion is disabled; start from the guarded RViz button")
        deadline = time.monotonic() + 12.0
        while time.monotonic() < deadline and not self._stopping.is_set():
            with self._lock:
                safe = self._xbox_known and not self._xbox_armed
            if (
                safe and self._positions is not None
                and self._camera_info_hash is not None
                and self._fk.service_is_ready() and self._planner.service_is_ready()
                and self._trajectory.server_is_ready()
                and self._can_lookup(self._base, self._link5)
                and self._can_lookup(self._camera, self._marker)
            ):
                break
            time.sleep(0.1)
        else:
            raise RuntimeError(
                "预检失败：需要 Xbox LOCKED、新鲜关节反馈、MoveIt/控制器以及稳定标定板 TF"
            )
        # A historical Link5->camera edge would make this run non-zero-history.
        if self._can_lookup(self._link5, self._camera):
            raise RuntimeError("检测到已有 Link5→Camera TF；停止旧标定 TF 发布后重试")
        anchor = self._current_positions()
        j6_margin = float(self.get_parameter("locked_j6_margin_rad").value)
        if not LOWER[5] + j6_margin <= anchor[5] <= UPPER[5] - j6_margin:
            raise RuntimeError(
                "anchor J6 violates official Piper-H calibration margin: "
                f"value={anchor[5]:+.6f} rad, allowed="
                f"[{LOWER[5] + j6_margin:+.6f}, {UPPER[5] - j6_margin:+.6f}] rad"
            )
        return anchor

    def _can_lookup(self, parent: str, child: str) -> bool:
        return self._buffer.can_transform(parent, child, rclpy.time.Time(), timeout=Duration(seconds=0.05))

    def _lookup(self, parent: str, child: str) -> Transform:
        try:
            return _transform(self._buffer.lookup_transform(
                parent, child, rclpy.time.Time(), timeout=Duration(seconds=0.25)
            ))
        except TransformException as error:
            raise RuntimeError(f"TF {parent}→{child} unavailable: {error}") from error

    def _capture(self, label: str) -> PosePair:
        audit = {
            "label": label, "observations": [], "rejected_reasons": Counter(), "quality": [],
            "timestamp_wait": {"waited_frames": 0, "max_initial_future_sec": 0.0},
        }
        try:
            return self._capture_window(label, audit)
        finally:
            summary = summarize_quality(audit["quality"])
            self._quality_results.extend(audit["quality"])
            self.get_logger().info(f"{label}: quality_summary={summary}")
            if self._run_dir is not None:
                atomic_write_yaml(self._run_dir / f"capture_{label}.yaml", {
                    "label": label, "accepted": summary["valid"],
                    "required": int(self.get_parameter("sample_frames").value),
                    "rejected_reasons": dict(audit["rejected_reasons"]),
                    "timestamp_wait": audit["timestamp_wait"],
                    "observations": audit["observations"],
                    "quality_summary": summary,
                })
                atomic_write_yaml(self._run_dir / "quality_summary.yaml", {
                    "quality_summary": summarize_quality(self._quality_results),
                })

    def _capture_window(self, label: str, audit: dict) -> PosePair:
        self._motion_guard()
        if self._stopping.wait(float(self.get_parameter("settle_sec").value)):
            self._motion_guard()
        window_start = self.get_clock().now().nanoseconds
        with self._lock:
            self._observations.clear()
        frames: list[PosePair] = []
        raw = audit["observations"]
        rejected = audit["rejected_reasons"]
        required = int(self.get_parameter("sample_frames").value)
        deadline = time.monotonic() + float(self.get_parameter("vision_frame_timeout").value)
        while len(frames) < required and time.monotonic() < deadline:
            self._motion_guard()
            with self._lock:
                observation = self._observations.popleft() if self._observations else None
            if observation is None:
                self._stopping.wait(0.01)
                continue
            timestamp_ready, initial_future_sec = self._wait_for_observation_timestamp(
                observation, deadline,
            )
            if initial_future_sec > 0.0:
                timing = audit["timestamp_wait"]
                timing["waited_frames"] += 1
                timing["max_initial_future_sec"] = max(
                    timing["max_initial_future_sec"], initial_future_sec,
                )
            if not timestamp_ready:
                rejected["future_timestamp"] += 1
                continue
            reason = self._frame_admission.reason(
                observation, self.get_clock().now().nanoseconds, window_start,
            )
            if reason is not None:
                rejected[reason] += 1
                continue
            # A bounded timestamp wait also gives joint feedback and TF time to
            # reach the image stamp. Snapshot history only after that wait.
            with self._lock:
                history = list(self._joint_history)
            record, pair, quality, reason = self._synchronize_observation(observation, history)
            if reason is not None:
                rejected[reason] += 1
                continue
            audit["quality"].append(quality)
            raw.append(record)
            if not quality.valid:
                rejected["low_quality"] += 1
                continue
            self._frame_admission.accept(observation)
            frames.append(pair)
        if len(frames) < required:
            raise RuntimeError(
                f"VISION_FRAME_TIMEOUT: {label}: {len(frames)}/{required} unique frames; "
                f"rejected={dict(rejected)}; timestamp_wait={audit['timestamp_wait']}"
            )
        pair = aggregate_pose_pairs(frames)
        self._sample_count += 1
        if self._run_dir is not None:
            document = {
                "label": label,
                "frame_count": len(frames),
                "camera_info_sha256": self._camera_info_hash,
                "base_T_link5": self._transform_dict(pair.base_T_link5),
                "camera_T_target": self._transform_dict(pair.camera_T_target),
                "raw": raw,
                "rejected_reasons": dict(rejected),
                "quality_summary": summarize_quality(audit["quality"]),
            }
            atomic_write_yaml(self._run_dir / f"{self._sample_count:03d}_{label}.yaml", document)
        return pair

    def _wait_for_observation_timestamp(self, observation, capture_deadline):
        """Wait for a slightly future camera stamp without weakening time-sync gates."""
        now_ns = self.get_clock().now().nanoseconds
        initial_future_sec = max(0.0, (observation.image_timestamp - now_ns) / 1e9)
        if initial_future_sec <= 0.0:
            return True, 0.0
        limit = float(self.get_parameter("max_future_timestamp_wait_sec").value)
        if initial_future_sec > limit:
            return False, initial_future_sec
        wait_deadline = min(capture_deadline, time.monotonic() + limit)
        while self.get_clock().now().nanoseconds < observation.image_timestamp:
            self._motion_guard()
            remaining = wait_deadline - time.monotonic()
            if remaining <= 0.0:
                return False, initial_future_sec
            ahead = (observation.image_timestamp - self.get_clock().now().nanoseconds) / 1e9
            self._stopping.wait(min(max(ahead, 0.001), 0.05, remaining))
        return True, initial_future_sec

    def _synchronize_observation(self, observation, history):
        """Shared capture/record-only path; never queries a latest visual TF."""
        joint = nearest_joint_state(
            history, observation.image_timestamp,
            float(self.get_parameter("robot_visual_sync_threshold").value),
        )
        if joint is None:
            return None, None, None, "joint_timestamp_mismatch"
        velocities = joint["velocities"]
        if velocities is not None and (
            not all(math.isfinite(value) for value in velocities)
            or max(abs(value) for value in velocities) > 0.015
        ):
            return None, None, None, "robot_not_settled"
        if max(abs(a - b) for a, b in zip(joint["positions"], self._current_positions())) > 0.003:
            return None, None, None, "robot_not_settled"
        try:
            message = self._buffer.lookup_transform(
                self._base, self._link5, rclpy.time.Time(nanoseconds=observation.image_timestamp),
                timeout=Duration(seconds=float(self.get_parameter("robot_tf_wait_sec").value)),
            )
        except TransformException:
            return None, None, None, "robot_tf_timestamp_unavailable"
        robot = _transform(message)
        camera = observation.camera_to_board_pose
        pair = PosePair(robot, Transform(camera["rotation"], camera["translation"]))
        quality = evaluate_observation_quality(observation, self._quality_config)
        record = replace(
            observation, robot_timestamp=joint["timestamp"], joint_state=joint,
            robot_pose=self._transform_dict(robot),
            robot_visual_time_offset=(joint["timestamp"] - observation.image_timestamp) / 1e9,
        ).to_dict()
        record.update(valid=quality.valid, reason=quality.reasons, vision_quality=quality.to_dict())
        return record, pair, quality, None

    def _run_record_only(self):
        """Observe manual, stationary poses indefinitely; no planning or solving."""
        self._publish("precheck", "仅记录：等待 CameraInfo、joint 和 TF；不会驱动机械臂")
        deadline = time.monotonic() + float(self.get_parameter("service_timeout_sec").value)
        while not self._stopping.is_set():
            with self._lock:
                ready = self._camera_info_document is not None and bool(self._joint_history)
            if ready and self._can_lookup(self._base, self._link5) and self._can_lookup(self._camera, self._marker):
                self._current_positions()
                break
            if time.monotonic() > deadline:
                raise RuntimeError("RECORD_PRECHECK_TIMEOUT: need CameraInfo, stamped joint feedback and robot/board TF")
            self._stopping.wait(0.05)
        if self._stopping.is_set():
            return
        root = Path(os.environ.get("ANTBOT_HAND_EYE_RUNS", str(Path.home() / ".ros" / "handeye_runs")))
        timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
        self._run_dir = root / f"record_session_{timestamp}"
        (self._run_dir / "observations").mkdir(parents=True, exist_ok=False)
        atomic_write_yaml(self._run_dir / "camera_info.yaml", self._camera_info_document)
        self.get_logger().info(f"record_session: {self._run_dir}")
        self._publish("anchor", "仅记录：保持初始姿态静止，等待首个合格 Anchor 观测", record_session=str(self._run_dir))
        window_start = self.get_clock().now().nanoseconds
        rejected, quality_results = Counter(), []
        count, anchor_recorded = 0, False

        def save_summary(state):
            atomic_write_yaml(self._run_dir / "session.yaml", {
                "mode": "record_only", "state": state, "record_session": str(self._run_dir),
                "base_frame": self._base, "link_frame": self._link5,
                "camera_frame": self._camera, "target_frame": self._marker,
                "observations": count, "anchor_recorded": anchor_recorded,
                "quality_summary": summarize_quality(quality_results),
                "rejected_reasons": dict(rejected),
                "camera_info_sha256": self._camera_info_hash,
                "motion_enabled": False, "calibration_written": False,
                "quality_parameters": asdict(self._quality_config),
                "min_detection_interval": float(self.get_parameter("min_detection_interval").value),
                "max_detection_age": float(self.get_parameter("max_detection_age").value),
                "robot_visual_sync_threshold": float(self.get_parameter("robot_visual_sync_threshold").value),
                "updated_utc": datetime.now(timezone.utc).isoformat(),
            })

        save_summary("waiting_anchor")
        try:
            while not self._stopping.is_set():
                with self._lock:
                    changed = self._camera_info_changed
                    observation = self._observations.popleft() if self._observations else None
                    history = list(self._joint_history)
                if changed:
                    raise RuntimeError("RECORD_CAMERA_INFO_CHANGED: stop and restart recording")
                if observation is None:
                    self._stopping.wait(0.01)
                    continue
                reason = self._frame_admission.reason(observation, self.get_clock().now().nanoseconds, window_start)
                record, pair, quality = None, None, None
                if reason is None:
                    record, pair, quality, reason = self._synchronize_observation(observation, history)
                if reason is not None:
                    rejected[reason] += 1
                    # Keep rejected observations too; never fill unmatched robot fields with latest state.
                    record = observation.to_dict()
                    record.update(valid=False, reason=[reason], vision_quality=None)
                else:
                    quality_results.append(quality)
                    if quality.valid:
                        self._frame_admission.accept(observation)
                    else:
                        rejected["low_quality"] += 1
                count += 1
                atomic_write_yaml(self._run_dir / "observations" / f"{count:06d}.yaml", record)
                if quality is not None and quality.valid and not anchor_recorded:
                    self._anchor_robot_pose, self._anchor_visual_pose = pair.base_T_link5, pair.camera_T_target
                    atomic_write_yaml(self._run_dir / "anchor.yaml", record)
                    anchor_recorded = True
                save_summary("recording" if anchor_recorded else "waiting_anchor")
                if count == 1 or count % 15 == 0:
                    self._publish("recording" if anchor_recorded else "anchor",
                                  f"仅记录：已保存 {count} 条观测；人工移动后请停稳",
                                  record_session=str(self._run_dir), quality_summary=summarize_quality(quality_results))
        except Exception:
            save_summary("failed")
            raise
        else:
            save_summary("stopped")
            self._publish("record_stopped", "仅记录已停止，没有求解或保存外参", record_session=str(self._run_dir))

    def _check_anchor_robot(self, label, after):
        if self._anchor_robot_pose is None:
            return
        drift = anchor_drift(
            self._anchor_robot_pose, after,
            float(self.get_parameter("anchor_translation_threshold").value),
            float(self.get_parameter("anchor_rotation_threshold").value),
        )
        if self._run_dir is not None:
            atomic_write_yaml(self._run_dir / f"anchor_robot_{label}.yaml", {
                "T_before": self._transform_dict(self._anchor_robot_pose),
                "T_after": self._transform_dict(after), **drift,
            })
        if drift["exceeded"]:
            self._stopping.set()
            self._confirm_motion_stopped()
            raise MotionSafetyError(f"ANCHOR_DRIFT_ERROR: {label}: {drift}")

    def _return_anchor(self, anchor, label):
        # Planning remains tighter than the real-feedback and Cartesian Anchor
        # gates, while allowing MoveIt's normal endpoint approximation.
        self._execute(
            anchor, label,
            goal_tolerance=float(self.get_parameter("anchor_goal_tolerance_rad").value),
        )
        # Capture waits for settling and admits only new observations with
        # synchronized joint feedback and timestamped robot TF. A latest TF
        # immediately after action success can still describe the moving arm.
        pair = self._capture(f"anchor_check_{label}")
        self._check_anchor_robot(label + "_arrival", pair.base_T_link5)
        if self._anchor_visual_pose is not None:
            drift = pose_dispersion([self._anchor_visual_pose, pair.camera_T_target])
            if drift["translation_max_m"] > 0.020 or drift["rotation_max_deg"] > 3.0:
                self._stopping.set()
                raise MotionSafetyError(f"ANCHOR_DRIFT_ERROR: visual chain: {drift}")
        return pair

    @staticmethod
    def _transform_dict(value: Transform) -> dict:
        return {"rotation": value.rotation.tolist(), "translation": value.translation.tolist()}

    def _robot_state(self, positions) -> RobotState:
        return RobotState(
            is_diff=False,
            joint_state=JointState(name=list(JOINT_NAMES), position=list(positions)),
        )

    def _plan(
        self, start, goal, goal_tolerance=0.008,
        velocity_scaling=None, acceleration_scaling=None,
    ):
        request = GetMotionPlan.Request()
        motion = request.motion_plan_request
        motion.group_name = "arm"
        motion.pipeline_id = "ompl"
        motion.planner_id = "RRTConnect"
        motion.allowed_planning_time = float(self.get_parameter("planning_time_sec").value)
        motion.num_planning_attempts = 4
        default_scaling = float(self.get_parameter("velocity_scaling").value)
        motion.max_velocity_scaling_factor = (
            default_scaling if velocity_scaling is None else float(velocity_scaling)
        )
        motion.max_acceleration_scaling_factor = (
            default_scaling if acceleration_scaling is None else float(acceleration_scaling)
        )
        motion.start_state = self._robot_state(start)
        motion.path_constraints = Constraints(joint_constraints=[JointConstraint(
            joint_name="joint6", position=float(goal[5]), tolerance_above=0.001,
            tolerance_below=0.001, weight=1.0,
        )])
        motion.goal_constraints = [Constraints(joint_constraints=[
            JointConstraint(
                joint_name=name, position=float(value), tolerance_above=goal_tolerance,
                tolerance_below=goal_tolerance, weight=1.0,
            ) for name, value in zip(JOINT_NAMES, goal)
        ])]
        response = self._wait_future(
            self._planner.call_async(request), "MoveIt planning"
        ).motion_plan_response
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"MoveIt rejected path ({response.error_code.val})")
        return response.trajectory.joint_trajectory

    def _execute(
        self, goal, label: str, goal_tolerance=0.008,
        velocity_scaling=None, acceleration_scaling=None,
        convergence_retries=0, command_target=None,
        enforce_j4_zero_exclusion=False,
    ) -> None:
        if self._record_only:
            raise MotionSafetyError("record-only mode forbids trajectory execution")
        self._motion_guard()
        with self._lock:
            if self._active_goal is not None or self._pending_goal is not None:
                raise MotionSafetyError("MOTION_STOP_UNCONFIRMED: previous goal still tracked")
        current = self._current_positions()
        values = np.asarray(goal, dtype=np.float64)
        if values.shape != (6,) or not np.all(np.isfinite(values)):
            raise MotionSafetyError(f"{label}: invalid joint goal")
        if np.any(values < LOWER) or np.any(values > UPPER):
            raise MotionSafetyError(f"{label}: goal exceeds joint limits")
        plan_options = {"goal_tolerance": goal_tolerance}
        if velocity_scaling is not None:
            plan_options["velocity_scaling"] = velocity_scaling
        if acceleration_scaling is not None:
            plan_options["acceleration_scaling"] = acceleration_scaling
        trajectory = self._plan(current, goal, **plan_options)
        if set(trajectory.joint_names) != set(JOINT_NAMES) or not trajectory.points:
            raise MotionSafetyError(f"{label}: invalid planned trajectory")
        indices = [trajectory.joint_names.index(name) for name in JOINT_NAMES]
        for point in trajectory.points:
            if len(point.positions) != len(JOINT_NAMES):
                raise MotionSafetyError(f"{label}: incomplete trajectory point")
            values = np.asarray([point.positions[index] for index in indices])
            if (
                not np.all(np.isfinite(values)) or np.any(values < LOWER)
                or np.any(values > UPPER) or abs(values[5] - goal[5]) > 0.001
            ):
                raise MotionSafetyError(f"{label}: trajectory violates joint limits or J6 lock")
        endpoint = np.asarray([trajectory.points[-1].positions[index] for index in indices])
        endpoint_error = endpoint - np.asarray(goal)
        worst_endpoint = int(np.argmax(np.abs(endpoint_error)))
        if abs(endpoint_error[worst_endpoint]) > goal_tolerance + 1e-9:
            raise MotionSafetyError(
                f"{label}: planned endpoint exceeds goal tolerance: "
                f"J{worst_endpoint + 1} error={endpoint_error[worst_endpoint]:+.9f} rad "
                f"({math.degrees(endpoint_error[worst_endpoint]):+.6f} deg), "
                f"tolerance={goal_tolerance:.9f} rad"
            )
        if enforce_j4_zero_exclusion and PiperHAutoHandeye._j4_in_zero_exclusion(
            self, endpoint
        ):
            exclusion = float(self.get_parameter("j4_zero_exclusion_deg").value)
            raise RuntimeError(
                f"{label}: planned endpoint J4={math.degrees(endpoint[3]):+.6f} deg "
                f"is inside the configured zero exclusion zone (+/-{exclusion:.6f} deg)"
            )
        self._motion_guard()
        action_goal = FollowJointTrajectory.Goal()
        action_goal.trajectory = trajectory
        # Keep MoveIt's planning tolerance independent from controller
        # convergence. The controller continues driving to the formal
        # calibration threshold; action success still cannot replace the
        # strict CAN-feedback / one-second arrival window below.
        controller_tolerance = math.radians(float(
            self.get_parameter("joint_arrival_tolerance_deg").value
        ))
        action_goal.goal_tolerance = [
            JointTolerance(name=name, position=controller_tolerance)
            for name in JOINT_NAMES
        ]
        with self._stop_lock:
            self._motion_guard()
            with self._lock:
                self._pending_goal = self._trajectory.send_goal_async(action_goal)
                pending = self._pending_goal
            pending.add_done_callback(self._goal_response)
        deadline = time.monotonic() + 5.0
        try:
            while not pending.done():
                self._motion_guard()
                if time.monotonic() >= deadline:
                    raise MotionSafetyError(f"{label}: goal acceptance timeout")
                time.sleep(0.01)
            with self._stop_lock:
                handle = pending.result()
                with self._lock:
                    if handle is not None and handle.accepted:
                        self._active_goal = handle
                        if self._active_result is None:
                            self._active_result = handle.get_result_async()
                        result_future = self._active_result
                    self._pending_goal = None
        except Exception:
            self._stopping.set()
            self._confirm_motion_stopped()
            raise
        if handle is None:
            raise MotionSafetyError(f"{label}: goal acceptance returned no handle")
        if not handle.accepted:
            raise RuntimeError(f"controller rejected {label}")
        completed = threading.Event()
        result_future.add_done_callback(lambda _future: completed.set())
        deadline = time.monotonic() + float(
            self.get_parameter("trajectory_timeout_sec").value
        )
        try:
            self._motion_guard()
            while not completed.wait(0.1):
                self._motion_guard()
                self._current_positions()
                if time.monotonic() >= deadline:
                    raise MotionSafetyError(f"{label}: trajectory timeout")
            wrapped = result_future.result()
            if wrapped is None or wrapped.status not in (4, 5, 6):
                raise MotionSafetyError(f"{label}: no terminal trajectory result")
            result = wrapped.result
        except Exception:
            self._stopping.set()
            self._confirm_motion_stopped()
            raise
        else:
            with self._lock:
                if self._active_goal is handle:
                    self._active_goal = None
                    self._active_result = None
        if wrapped.status != 4 or result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self._stopping.set()
            raise MotionSafetyError(f"{label} controller error {result.error_code}: {result.error_string}")
        # Action success alone is not calibration arrival success.  Compare
        # against the final point actually sent to the controller, not the
        # pre-planning request (which MoveIt may satisfy within its tolerance).
        arrival_target = endpoint.tolist()
        if command_target is None:
            self._wait_for_arrival(arrival_target, label)
            return
        arrived = self._wait_for_arrival(
            arrival_target, label, command_target=command_target,
            planned_final=arrival_target, abort_on_timeout=convergence_retries <= 0,
        )
        if arrived:
            return
        retry_index = int(self.get_parameter("bootstrap_convergence_retries").value) - int(
            convergence_retries
        ) + 1
        retry_label = f"{label.split('_convergence_')[0]}_convergence_{retry_index:02d}"
        self.get_logger().warning(
            f"{label}: strict arrival gate not reached; executing small convergence "
            f"trajectory {retry_label} to the previously commanded final point"
        )
        self._execute(
            arrival_target, retry_label, goal_tolerance=goal_tolerance,
            velocity_scaling=velocity_scaling,
            acceleration_scaling=acceleration_scaling,
            convergence_retries=convergence_retries - 1,
            command_target=command_target,
            enforce_j4_zero_exclusion=enforce_j4_zero_exclusion,
        )

    def _j4_in_zero_exclusion(self, joints) -> bool:
        exclusion = math.radians(float(
            self.get_parameter("j4_zero_exclusion_deg").value
        ))
        return exclusion > 0.0 and abs(float(joints[3])) < exclusion

    def _arrival_joint_state(self, message: JointState) -> None:
        names = [name.split("/")[-1] for name in message.name]
        if len(message.position) != len(names) or any(names.count(name) != 1 for name in JOINT_NAMES):
            return
        positions = tuple(float(message.position[names.index(name)]) for name in JOINT_NAMES)
        if not all(math.isfinite(value) for value in positions):
            return
        stamp = message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        if stamp <= 0 or not -0.1 <= age <= 0.75:
            return
        received = time.monotonic()
        with self._arrival_condition:
            if stamp <= self._arrival_last_stamp:
                return
            self._arrival_last_stamp = stamp
            self._arrival_latest = (positions, stamp, received)
            self._arrival_feedback.append(self._arrival_latest)
            self._arrival_condition.notify_all()

    def _wait_for_arrival(
        self, target, label: str, command_target=None, planned_final=None,
        abort_on_timeout=True,
    ) -> bool:
        if self._record_only:
            raise MotionSafetyError("record-only mode forbids automatic arrival checks")
        window = HandeyeArrivalWindow(
            target, float(self.get_parameter("joint_arrival_tolerance_deg").value),
            float(self.get_parameter("stable_duration").value),
            float(self.get_parameter("stable_position_variation_deg").value),
        )
        started = time.monotonic()
        start_stamp = self.get_clock().now().nanoseconds
        deadline = started + float(self.get_parameter("arrival_timeout_sec").value)
        with self._arrival_condition:
            # Never reuse pre-completion feedback as a stationary arrival window.
            self._arrival_feedback.clear()
        self._publish("arrival", f"{label}: 等待真实关节反馈到位并稳定")
        while time.monotonic() < deadline:
            self._motion_guard()
            with self._arrival_condition:
                samples = list(self._arrival_feedback)
                self._arrival_feedback.clear()
                if not samples:
                    self._arrival_condition.wait(min(0.05, max(0.0, deadline - time.monotonic())))
            for positions, stamp, received in samples:
                now = time.monotonic()
                if (now >= deadline or received < started or stamp <= start_stamp
                        or now - received > 0.75
                        or (self.get_clock().now().nanoseconds - stamp) / 1e9 > 0.75):
                    continue
                if window.add(positions, stamp / 1e9, received):
                    self._motion_guard()
                    self.get_logger().info(
                        f"{label}: arrival success; residual_deg={list(window.residual_deg)}; "
                        f"stable_duration={window.duration:.3f}s"
                    )
                    return True
        self._motion_guard()
        with self._arrival_condition:
            latest = self._arrival_latest
        positions = latest[0] if latest is not None else None
        age = time.monotonic() - latest[2] if latest is not None else math.inf
        message = arrival_timeout_message(
            label, target, positions, age,
            command_target=command_target, planned_final=planned_final,
        )
        self.get_logger().error(message)
        if abort_on_timeout:
            self._stopping.set()
            raise MotionSafetyError(message)
        return False

    def _bootstrap_goals(self, anchor):
        degrees = (
            (8, 0, 0, 0, 0), (-8, 0, 0, 0, 0),
            (0, 7, -5, 0, 0), (0, -7, 5, 0, 0),
            (0, 0, 7, 7, 0), (0, 0, -7, -7, 0),
            (0, 0, 0, 10, 0), (0, 0, 0, -10, 0),
            (0, 0, 0, 0, 10), (0, 0, 0, 0, -10),
            (6, 6, -4, 7, 4), (-6, -6, 4, -7, -4),
            (7, -5, 6, -8, 5), (-7, 5, -6, 8, -5),
            (10, 5, -8, 10, -7), (-10, -5, 8, -10, 7),
            (5, 10, -10, -6, 9), (-5, -10, 10, 6, -9),
        )
        anchor = np.asarray(anchor)
        margin = calibration_joint_margins(
            float(self.get_parameter("joint_margin_fraction").value),
            float(self.get_parameter("locked_j6_margin_rad").value),
        )
        for values in degrees:
            candidate = anchor.copy()
            candidate[:5] += np.radians(values)
            candidate[5] = anchor[5]
            if values_within_calibration_margins(candidate, margin):
                yield tuple(float(value) for value in candidate)

    def _fk_for_link5(self, joints) -> Transform | None:
        request = GetPositionFK.Request()
        request.header.frame_id = self._moveit_base
        request.fk_link_names = [self._moveit_link]
        request.robot_state = self._robot_state(joints)
        response = self._wait_future(self._fk.call_async(request), "Link5 FK", 4.0)
        if response.error_code.val != MoveItErrorCodes.SUCCESS or not response.pose_stamped:
            return None
        pose = response.pose_stamped[0].pose
        return Transform(
            quaternion_to_rotation(
                pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w,
            ),
            (pose.position.x, pose.position.y, pose.position.z),
        )

    def _generate_joint_candidates(
        self, anchor, link5_T_camera: Transform, base_T_target: Transform,
        centre_offset,
    ) -> list[HemispherePose]:
        """Generate reachable J1-J5 samples, then classify them in board geometry."""
        centre = base_T_target.point(centre_offset)
        normal = base_T_target.rotation[:, 2].copy()
        anchor_link5 = self._fk_for_link5(anchor)
        if anchor_link5 is None:
            raise RuntimeError("MoveIt FK failed for the known Anchor pose")
        anchor_camera = anchor_link5 @ link5_T_camera
        if float((anchor_camera.translation - centre) @ normal) < 0.0:
            normal = -normal
        axis1 = base_T_target.rotation[:, 0]
        axis1 = axis1 - float(axis1 @ normal) * normal
        axis1 /= np.linalg.norm(axis1)
        axis2 = np.cross(normal, axis1)
        amplitudes = np.radians((32.0, 25.0, 25.0, 35.0, 30.0))
        margin = calibration_joint_margins(
            float(self.get_parameter("joint_margin_fraction").value),
            float(self.get_parameter("locked_j6_margin_rad").value),
        )
        count = int(self.get_parameter("joint_candidate_pool_size").value)
        rng = np.random.default_rng(20261003)
        output: list[HemispherePose] = []
        rejection = Counter()
        # Preserve the global-mode candidate stream exactly.  Local mode keeps
        # drawing from that same deterministic stream until it has the same
        # number of admissible candidates, rather than shrinking the sample
        # budget after applying the workspace constraints.
        attempts = 0
        while attempts < count or (
            self._workspace_mode == "local" and len(output) < count
        ):
            attempts += 1
            if self._workspace_mode == "local":
                self._motion_guard()
            goal = np.asarray(anchor, dtype=np.float64).copy()
            goal[:5] += rng.uniform(-1.0, 1.0, 5) * amplitudes
            goal[5] = anchor[5]
            if not values_within_calibration_margins(goal, margin):
                rejection["joint_margin"] += 1
                continue
            if self._j4_in_zero_exclusion(goal):
                rejection["j4_zero_exclusion"] += 1
                continue
            base_T_link5 = self._fk_for_link5(goal)
            if base_T_link5 is None:
                rejection["fk"] += 1
                continue
            base_T_camera = base_T_link5 @ link5_T_camera
            if not self._candidate_in_fov(base_T_camera):
                rejection["predicted_fov_or_projected_area"] += 1
                continue
            if not self._candidate_in_workspace(base_T_camera):
                rejection["workspace_constraints"] += 1
                continue
            vector = base_T_camera.translation - centre
            radius = float(np.linalg.norm(vector))
            if radius <= 1.0e-9:
                rejection["zero_radius"] += 1
                continue
            theta = math.degrees(math.acos(np.clip(float(vector @ normal) / radius, -1.0, 1.0)))
            if theta > 45.0:
                rejection["outside_45_deg"] += 1
                continue
            phi = math.degrees(math.atan2(float(vector @ axis2), float(vector @ axis1))) % 360.0
            roll = look_at_roll_deg(base_T_camera, centre, base_T_target.rotation[:, 0])
            output.append(HemispherePose(
                radius, theta, phi, roll, base_T_camera,
                tuple(float(value) for value in goal),
            ))
        if self._run_dir is not None:
            atomic_write_yaml(self._run_dir / "joint_candidate_pool.yaml", {
                "requested": count,
                "attempted": attempts,
                "accepted": len(output),
                "rejected": dict(rejection),
            })
        return output

    @staticmethod
    def _observed_workspace_metrics(camera_T_target: Transform) -> tuple[float, float]:
        """Return measured board tilt and camera-frame target Z."""
        # The board normal is sign-ambiguous for tilt; use the acute angle to
        # the optical Z axis, matching the offline diagnostic definition.
        cosine = float(np.clip(abs(camera_T_target.rotation[2, 2]), 0.0, 1.0))
        tilt_deg = math.degrees(math.acos(cosine))
        return tilt_deg, float(camera_T_target.translation[2])

    def _candidate_workspace_metrics(self, base_T_camera: Transform) -> tuple[float, float]:
        """Return predicted board tilt and camera-frame target Z."""
        if self._base_T_target is None:
            return math.inf, math.inf
        return self._observed_workspace_metrics(
            base_T_camera.inverse() @ self._base_T_target
        )

    @classmethod
    def _observed_in_local_workspace(cls, camera_T_target: Transform) -> bool:
        tilt_deg, z_m = cls._observed_workspace_metrics(camera_T_target)
        return tilt_deg < 25.0 and z_m < 0.48

    def _candidate_in_local_workspace(self, base_T_camera: Transform) -> bool:
        tilt_deg, z_m = self._candidate_workspace_metrics(base_T_camera)
        return tilt_deg < 25.0 and z_m < 0.48

    def _candidate_in_workspace(self, base_T_camera: Transform) -> bool:
        if self._workspace_mode == "global":
            return True
        return self._candidate_in_local_workspace(base_T_camera)

    def _candidate_in_fov(self, base_T_camera: Transform) -> bool:
        if self._base_T_target is None or self._camera_info_document is None:
            return False
        corners = np.asarray(
            self.get_parameter("board_outer_corners_xyz").value,
            dtype=np.float64,
        ).reshape((-1, 3))
        camera_T_target = base_T_camera.inverse() @ self._base_T_target
        camera_points = np.asarray([
            camera_T_target.point(corner) for corner in corners
        ])
        if np.any(camera_points[:, 2] <= 0.05):
            return False
        homogeneous = np.column_stack((camera_points, np.ones(len(camera_points))))
        projection = np.asarray(
            self._camera_info_document["p"], dtype=np.float64
        ).reshape((3, 4))
        pixels_h = (projection @ homogeneous.T).T
        pixels = pixels_h[:, :2] / pixels_h[:, 2, None]
        margin = float(self.get_parameter("fov_margin_px").value)
        width = float(self._camera_info_document["width"])
        height = float(self._camera_info_document["height"])
        if (
            np.any(pixels[:, 0] < margin) or np.any(pixels[:, 0] > width - margin)
            or np.any(pixels[:, 1] < margin) or np.any(pixels[:, 1] > height - margin)
        ):
            return False
        area = 0.5 * abs(float(
            np.dot(pixels[:, 0], np.roll(pixels[:, 1], -1))
            - np.dot(pixels[:, 1], np.roll(pixels[:, 0], -1))
        ))
        return area >= float(self.get_parameter("minimum_projected_board_area_px2").value)

    def _refresh_joint_candidates(
        self, candidates, link5_T_camera: Transform, base_T_target: Transform,
        centre_offset, anchor,
    ) -> tuple[list[HemispherePose], Counter]:
        """Recompute candidate camera geometry with the frozen final model."""
        centre = base_T_target.point(centre_offset)
        normal = base_T_target.rotation[:, 2].copy()
        anchor_link5 = self._fk_for_link5(anchor)
        if anchor_link5 is None:
            raise RuntimeError("MoveIt FK failed while refreshing final Test candidates")
        anchor_camera = anchor_link5 @ link5_T_camera
        if float((anchor_camera.translation - centre) @ normal) < 0.0:
            normal = -normal
        axis1 = base_T_target.rotation[:, 0]
        axis1 = axis1 - float(axis1 @ normal) * normal
        axis1 /= np.linalg.norm(axis1)
        axis2 = np.cross(normal, axis1)

        output: list[HemispherePose] = []
        rejection = Counter()
        for candidate in candidates:
            if candidate.joint_goal is None:
                rejection["missing_joint_goal"] += 1
                continue
            base_T_link5 = self._fk_for_link5(candidate.joint_goal)
            if base_T_link5 is None:
                rejection["fk"] += 1
                continue
            base_T_camera = base_T_link5 @ link5_T_camera
            if not self._candidate_in_fov(base_T_camera):
                rejection["predicted_fov_or_projected_area"] += 1
                continue
            if not self._candidate_in_workspace(base_T_camera):
                rejection["workspace_constraints"] += 1
                continue
            vector = base_T_camera.translation - centre
            radius = float(np.linalg.norm(vector))
            if radius <= 1.0e-9:
                rejection["zero_radius"] += 1
                continue
            theta = math.degrees(math.acos(np.clip(
                float(vector @ normal) / radius, -1.0, 1.0,
            )))
            if theta > 45.0:
                rejection["outside_45_deg"] += 1
                continue
            phi = math.degrees(math.atan2(
                float(vector @ axis2), float(vector @ axis1),
            )) % 360.0
            roll = look_at_roll_deg(
                base_T_camera, centre, base_T_target.rotation[:, 0],
            )
            output.append(replace(
                candidate, radius_m=radius, theta_deg=theta, phi_deg=phi,
                roll_deg=roll, base_T_camera=base_T_camera,
            ))
        return output, rejection

    @staticmethod
    def _candidate_key(candidate) -> tuple:
        return (
            candidate.radius_m, candidate.theta_deg,
            round(candidate.phi_deg, 3), candidate.roll_deg,
        )

    def _stratified_candidates(self, candidates, count: int, excluded: set):
        """Cover strata first, then fill from every remaining distinct candidate.

        Reachability and FOV filtering can leave many nominal radius/theta/roll
        buckets empty.  The first pass preserves the intended stratification;
        the max-min fallback prevents sparse buckets from hiding otherwise safe,
        unused candidates from Selection or the independent Test set.
        """
        radii = sorted({round(item.radius_m, 6) for item in candidates})
        buckets = {}
        for item in candidates:
            radius = min(range(len(radii)), key=lambda i: abs(radii[i] - item.radius_m))
            theta = 0 if item.theta_deg <= 20.0 else 1 if item.theta_deg < 40.0 else 2
            roll = 0 if item.roll_deg < 0.0 else 1 if item.roll_deg == 0.0 else 2
            buckets.setdefault((radius, theta, roll), []).append(item)
        output = []
        local = set()
        attempts = max(count * 8, 64)
        for index in range(attempts):
            group = (
                index % max(len(radii), 1),
                (index + index // 3) % 3,
                (2 * index + index // 3) % 3,
            )
            target_phi = (360.0 * index / max(count, 1)) % 360.0
            choices = sorted(
                buckets.get(group, []),
                key=lambda item: abs(((item.phi_deg - target_phi + 180.0) % 360.0) - 180.0),
            )
            selected = next((
                item for item in choices
                if self._candidate_key(item) not in excluded
                and self._candidate_key(item) not in local
            ), None)
            if selected is None:
                continue
            output.append(selected)
            local.add(self._candidate_key(selected))
            if len(output) >= count:
                break

        if len(output) < count:
            remaining = []
            remaining_keys = set()
            for item in candidates:
                key = self._candidate_key(item)
                if key in excluded or key in local or key in remaining_keys:
                    continue
                remaining.append(item)
                remaining_keys.add(key)

            radius_values = [item.radius_m for item in candidates]
            radius_scale = max(
                max(radius_values, default=0.0) - min(radius_values, default=0.0),
                0.05,
            )
            reference_keys = set(excluded) | local

            def diversity_score(item):
                key = self._candidate_key(item)
                if not reference_keys:
                    return 0.0

                def distance(other):
                    radius = abs(key[0] - other[0]) / radius_scale
                    theta = abs(key[1] - other[1]) / 45.0
                    phi = abs(((key[2] - other[2] + 180.0) % 360.0) - 180.0) / 180.0
                    roll = abs(key[3] - other[3]) / 180.0
                    return math.sqrt(radius * radius + theta * theta + phi * phi + roll * roll)

                return min(distance(other) for other in reference_keys)

            while remaining and len(output) < count:
                selected = max(
                    remaining,
                    key=lambda item: (diversity_score(item), self._candidate_key(item)),
                )
                remaining.remove(selected)
                key = self._candidate_key(selected)
                output.append(selected)
                local.add(key)
                reference_keys.add(key)
        return output

    def _collect_hemisphere(
        self, candidates, link5_T_camera, anchor, count: int, prefix: str,
        excluded: set | None = None, fixed_attempts: bool = False,
        attempt_log: list[dict] | None = None, preserve_order: bool = False,
    ) -> list[PosePair]:
        output: list[PosePair] = []
        quality_start = len(self._quality_results)
        if not candidates:
            return output
        # The stride avoids consuming one ring/radius/roll block first.
        stride = max(1, len(candidates) // max(count * 4, 1))
        ordered = list(candidates) if preserve_order else [
            item for offset in range(stride) for item in candidates[offset::stride]
        ]
        seen = set()
        attempted = 0
        enforce_local_workspace = self._workspace_mode == "local"
        for index, candidate in enumerate(ordered):
            if (attempted if fixed_attempts else len(output)) >= count or self._stopping.is_set():
                break
            key = self._candidate_key(candidate)
            if key in seen or (excluded is not None and key in excluded):
                continue
            seen.add(key)
            attempted += 1
            record = {
                "id": f"{prefix}_{attempted:03d}",
                "radius_m": candidate.radius_m,
                "theta_deg": candidate.theta_deg,
                "phi_deg": candidate.phi_deg,
                "roll_deg": candidate.roll_deg,
                "ik": False,
                "executed": False,
                "detected_and_accepted": False,
            }
            if fixed_attempts and excluded is not None:
                excluded.add(key)
            solution = candidate.joint_goal
            if solution is None:
                record["failure"] = "missing_joint_goal"
                if attempt_log is not None:
                    attempt_log.append(record)
                continue
            # Candidate generation used the seed extrinsic. Recompute the
            # camera pose with the current provisional/final extrinsic before
            # every motion so FOV and workspace gates never use stale geometry.
            base_T_link5 = self._fk_for_link5(solution)
            if base_T_link5 is None:
                record["failure"] = "fk"
                if attempt_log is not None:
                    attempt_log.append(record)
                continue
            predicted = base_T_link5 @ link5_T_camera
            record["ik"] = True
            if not self._candidate_in_fov(predicted):
                record["failure"] = "predicted_fov_or_projected_area"
                if attempt_log is not None:
                    attempt_log.append(record)
                continue
            if enforce_local_workspace and not self._candidate_in_local_workspace(predicted):
                record["failure"] = "workspace_constraints"
                if attempt_log is not None:
                    attempt_log.append(record)
                continue
            try:
                self._publish("moving", f"{prefix}：前往候选 {len(output) + 1}/{count}")
                self._execute(
                    solution, f"{prefix}_{index:03d}",
                    enforce_j4_zero_exclusion=True,
                )
                record["executed"] = True
                pair = self._capture(f"{prefix}_{index:03d}")
                if enforce_local_workspace:
                    tilt_deg, z_m = self._observed_workspace_metrics(pair.camera_T_target)
                    record["measured_tilt_deg"] = tilt_deg
                    record["measured_depth_m"] = z_m
                    if not self._observed_in_local_workspace(pair.camera_T_target):
                        record["failure"] = "measured_workspace_constraints"
                        self.get_logger().warning(
                            f"skip {prefix} candidate {index}: measured workspace "
                            f"tilt={tilt_deg:.3f}deg depth={z_m:.3f}m"
                        )
                        if attempt_log is not None:
                            attempt_log.append(record)
                        continue
                output.append(pair)
                record["detected_and_accepted"] = True
                if excluded is not None:
                    excluded.add(key)
            except MotionSafetyError:
                raise
            except Exception as error:
                record["failure"] = str(error)
                self.get_logger().warning(f"skip {prefix} candidate {index}: {error}")
            if attempt_log is not None:
                attempt_log.append(record)
        summary = summarize_quality(self._quality_results[quality_start:])
        self.get_logger().info(f"{prefix}: quality_summary={summary}")
        if self._run_dir is not None:
            atomic_write_yaml(self._run_dir / f"quality_summary_{prefix}.yaml", {
                "quality_summary": summary,
            })
        return output

    def _run(self) -> None:
        if self._record_only:
            raise MotionSafetyError("record-only mode cannot enter calibration workflow")
        self._publish("precheck", "检查 Link5、控制权、MoveIt、相机和标定板")
        anchor = self._precheck()
        timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        root = Path(os.environ.get("ANTBOT_HAND_EYE_RUNS", str(Path.home() / ".ros" / "handeye_runs")))
        self._run_dir = root / f"piperh_link5_{timestamp}"
        self._run_dir.mkdir(parents=True, exist_ok=False)
        atomic_write_yaml(self._run_dir / "camera_info.yaml", self._camera_info_document)
        atomic_write_yaml(self._run_dir / "manifest.yaml", {
            "started_utc": datetime.now(timezone.utc).isoformat(),
            "mode": "piperh_link5_zero_history_adaptive",
            "base_frame": self._base,
            "link_frame": self._link5,
            "camera_frame": self._camera,
            "target_frame": self._marker,
            "camera_info_sha256": self._camera_info_hash,
            "historical_calibration_loaded": False,
            "j6_policy": "locked_to_anchor",
            "workspace_mode": self._workspace_mode,
            "workspace_constraints": {
                "tilt_max_deg": 25,
                "z_max": 0.48,
            },
        })
        if self._workspace_mode == "local":
            self.get_logger().info(
                "Local workspace calibration: tilt <25deg; z <0.48m"
            )
        else:
            self.get_logger().info("Global workspace calibration")
        self._required_count = int(self.get_parameter("bootstrap_max_samples").value) + 40
        self._publish("anchor", "Anchor 已锁定；开始多帧基准采样")
        bootstrap_quality_start = len(self._quality_results)
        self._wait_for_arrival(anchor, "anchor_initial")
        anchor_pair = self._capture("anchor_initial")
        if (
            self._workspace_mode == "local"
            and not self._observed_in_local_workspace(anchor_pair.camera_T_target)
        ):
            tilt_deg, z_m = self._observed_workspace_metrics(anchor_pair.camera_T_target)
            raise RuntimeError(
                "initial Anchor lies outside local workspace: "
                f"tilt={tilt_deg:.3f}deg depth={z_m:.3f}m"
            )
        self._anchor_robot_pose = anchor_pair.base_T_link5
        self._anchor_visual_pose = anchor_pair.camera_T_target

        bootstrap = [anchor_pair]
        self._publish("bootstrap", "零历史外参 Bootstrap：J1–J5 小范围探索，J6 锁定")
        for index, goal in enumerate(self._bootstrap_goals(anchor), 1):
            if len(bootstrap) >= int(self.get_parameter("bootstrap_max_samples").value):
                break
            try:
                self._execute(
                    goal, f"bootstrap_{index:02d}",
                    goal_tolerance=float(self.get_parameter(
                        "bootstrap_goal_tolerance_rad"
                    ).value),
                    velocity_scaling=float(self.get_parameter(
                        "bootstrap_velocity_scaling"
                    ).value),
                    acceleration_scaling=float(self.get_parameter(
                        "bootstrap_acceleration_scaling"
                    ).value),
                    convergence_retries=int(self.get_parameter(
                        "bootstrap_convergence_retries"
                    ).value),
                    command_target=goal,
                )
                pair = self._capture(f"bootstrap_{index:02d}")
                if (
                    self._workspace_mode == "local"
                    and not self._observed_in_local_workspace(pair.camera_T_target)
                ):
                    tilt_deg, z_m = self._observed_workspace_metrics(pair.camera_T_target)
                    self.get_logger().warning(
                        f"skip bootstrap candidate {index}: measured workspace "
                        f"tilt={tilt_deg:.3f}deg depth={z_m:.3f}m"
                    )
                else:
                    bootstrap.append(pair)
                self._return_anchor(anchor, f"bootstrap_return_{index:02d}")
            except MotionSafetyError:
                raise
            except Exception as error:
                self.get_logger().warning(f"bootstrap candidate {index} skipped: {error}")
                try:
                    self._return_anchor(anchor, "bootstrap_recovery")
                except Exception as recovery_error:
                    raise RuntimeError(f"Bootstrap recovery to Anchor failed: {recovery_error}") from error
            if len(bootstrap) >= 12:
                metrics = rotation_observability(bootstrap)
                positions = np.asarray([
                    item.base_T_link5.translation for item in bootstrap
                ])
                span = float(np.max(np.linalg.norm(
                    positions[:, None, :] - positions[None, :, :], axis=2
                )))
                if (
                    metrics["sv_ratio"] >= float(self.get_parameter(
                        "bootstrap_min_sv_ratio"
                    ).value)
                    and metrics["maximum_relative_deg"] >= float(self.get_parameter(
                        "bootstrap_min_maximum_rotation_deg"
                    ).value)
                    and span >= float(self.get_parameter(
                        "bootstrap_min_translation_span_m"
                    ).value)
                ):
                    break
        summary = summarize_quality(self._quality_results[bootstrap_quality_start:])
        self.get_logger().info(f"bootstrap: quality_summary={summary}")
        atomic_write_yaml(self._run_dir / "quality_summary_bootstrap.yaml", {
            "quality_summary": summary,
        })
        holdout_count = int(self.get_parameter("bootstrap_holdout_samples").value)
        if len(bootstrap) < max(10, holdout_count + 6):
            raise RuntimeError(f"Bootstrap only collected {len(bootstrap)} valid poses")
        try:
            train, holdout, bootstrap_split = split_bootstrap_train_holdout(
                bootstrap,
                holdout_count,
                float(self.get_parameter("bootstrap_min_sv_ratio").value),
                float(self.get_parameter("bootstrap_min_maximum_rotation_deg").value),
                float(self.get_parameter("bootstrap_min_translation_span_m").value),
            )
        except ValueError as error:
            raise RuntimeError(f"Bootstrap train/holdout split failed: {error}") from error
        atomic_write_yaml(self._run_dir / "bootstrap_split.yaml", bootstrap_split)
        observability = rotation_observability(train)
        if observability["sv_ratio"] < float(self.get_parameter("bootstrap_min_sv_ratio").value):
            raise RuntimeError(f"Bootstrap rotation observability is weak: {observability}")
        if observability["maximum_relative_deg"] < float(
            self.get_parameter("bootstrap_min_maximum_rotation_deg").value
        ):
            raise RuntimeError(f"Bootstrap rotation baseline is too small: {observability}")
        link_positions = np.asarray([
            item.base_T_link5.translation for item in train
        ])
        translation_span = float(np.max(np.linalg.norm(
            link_positions[:, None, :] - link_positions[None, :, :], axis=2
        )))
        if translation_span < float(
            self.get_parameter("bootstrap_min_translation_span_m").value
        ):
            raise RuntimeError(
                f"Bootstrap translation baseline is too small: {translation_span:.4f} m"
            )
        seeds = solve_handeye(train, holdout)
        if len(seeds) < 2:
            raise RuntimeError("fewer than two hand-eye solvers produced a valid seed")
        seed = seeds[0]
        if np.linalg.norm(seed.link5_T_camera.translation) > float(
            self.get_parameter("camera_mount_max_radius_m").value
        ):
            raise RuntimeError("seed camera translation lies outside the physical Link5 mount envelope")
        if (
            seed.holdout_translation_p95_m > float(self.get_parameter("bootstrap_max_holdout_translation_m").value)
            or seed.holdout_rotation_p95_deg > float(self.get_parameter("bootstrap_max_holdout_rotation_deg").value)
        ):
            raise RuntimeError("Bootstrap holdout rejected the provisional hand-eye result")
        consensus = solver_consensus(seeds[: min(3, len(seeds))])
        if (
            consensus["translation_max_m"] > float(self.get_parameter(
                "bootstrap_max_solver_translation_m"
            ).value)
            or consensus["rotation_max_deg"] > float(self.get_parameter(
                "bootstrap_max_solver_rotation_deg"
            ).value)
        ):
            raise RuntimeError(f"Bootstrap solver consensus rejected the seed: {consensus}")
        self._publish("seed_solved", f"临时外参通过：{seed.solver}", observability=observability, consensus=consensus)
        link5_T_camera = seed.link5_T_camera
        base_T_target = aggregate_pose_pairs([
            PosePair(item.base_T_link5 @ link5_T_camera @ item.camera_T_target, Transform.identity())
            for item in train
        ]).base_T_link5
        self._base_T_target = base_T_target
        centre_offset = tuple(float(value) for value in self.get_parameter("charuco_board_center_xyz").value)
        joint_pool = self._generate_joint_candidates(
            anchor, link5_T_camera, base_T_target, centre_offset,
        )
        if len(joint_pool) < 24:
            raise RuntimeError(
                f"joint-space candidate pool produced only {len(joint_pool)} valid poses"
            )

        calibration = list(train)
        used_candidates: set = set()
        for theta in (25.0, 35.0, 45.0):
            self._publish("hemisphere", f"生成并筛选 {theta:.0f}° 半球候选")
            pool = [item for item in joint_pool if item.theta_deg <= theta]
            round_attempts: list[dict] = []
            new_cal = self._collect_hemisphere(
                pool, link5_T_camera, anchor,
                int(self.get_parameter("round_calibration_samples").value),
                f"cal_{int(theta)}",
                used_candidates,
                attempt_log=round_attempts,
            )
            if len(new_cal) < 4:
                failures = Counter(
                    item.get("failure", "accepted") for item in round_attempts
                )
                raise RuntimeError(
                    f"{theta:.0f}° round produced only {len(new_cal)} calibration samples; "
                    f"candidate_results={dict(failures)}"
                )
            calibration.extend(new_cal)
            updated = solve_handeye(calibration, holdout)
            if not updated:
                raise RuntimeError(f"no solver succeeded after {theta:.0f}° round")
            link5_T_camera = updated[0].link5_T_camera
            base_T_target = aggregate_pose_pairs([
                PosePair(item.base_T_link5 @ link5_T_camera @ item.camera_T_target, Transform.identity())
                for item in calibration
            ]).base_T_link5
            self._base_T_target = base_T_target
            current_anchor = self._return_anchor(anchor, f"anchor_return_{int(theta)}")
            anchor_drift = pose_dispersion([
                anchor_pair.camera_T_target, current_anchor.camera_T_target,
            ])
            if anchor_drift["translation_max_m"] > 0.020 or anchor_drift["rotation_max_deg"] > 3.0:
                raise RuntimeError(f"Anchor chain drift exceeded limit: {anchor_drift}")

        # Freeze the training set before acquiring a disjoint selection set.
        # Selection ranks solvers only; it is never folded back into calibration.
        full_pool = joint_pool
        selection_count = int(self.get_parameter("selection_samples").value)
        selection_pool = self._stratified_candidates(
            full_pool, selection_count * 3, used_candidates
        )
        selection = self._collect_hemisphere(
            selection_pool, link5_T_camera, anchor, selection_count, "selection",
            used_candidates, preserve_order=True,
        )
        if len(selection) < 6:
            raise RuntimeError(f"Selection valid {len(selection)}, required at least 6")
        solved = solve_handeye(calibration, selection)
        if not solved:
            raise RuntimeError("final solver stage failed")
        convergence_history = []
        previous = None
        stable_rounds = 0
        maximum_samples = int(self.get_parameter("maximum_calibration_samples").value)
        required_stable = int(self.get_parameter("convergence_rounds").value)
        while True:
            final = solved[0]
            self._base_T_target = aggregate_pose_pairs([
                PosePair(
                    item.base_T_link5 @ final.link5_T_camera @ item.camera_T_target,
                    Transform.identity(),
                )
                for item in calibration
            ]).base_T_link5
            current = {
                "calibration_samples": len(calibration),
                "solver": final.solver,
                "selection_translation_p95_m": final.holdout_translation_p95_m,
                "selection_rotation_p95_deg": final.holdout_rotation_p95_deg,
            }
            if previous is not None:
                old = previous["candidate"]
                translation_delta = float(np.linalg.norm(
                    final.link5_T_camera.translation - old.link5_T_camera.translation
                ))
                rotation_delta = math.degrees(float(np.linalg.norm(rotation_log(
                    old.link5_T_camera.rotation.T @ final.link5_T_camera.rotation
                ))))
                translation_change = abs(
                    final.holdout_translation_p95_m - old.holdout_translation_p95_m
                ) / max(old.holdout_translation_p95_m, 1.0e-9)
                rotation_change = abs(
                    final.holdout_rotation_p95_deg - old.holdout_rotation_p95_deg
                ) / max(old.holdout_rotation_p95_deg, 1.0e-9)
                current.update({
                    "translation_change_fraction": translation_change,
                    "rotation_change_fraction": rotation_change,
                    "solution_translation_delta_m": translation_delta,
                    "solution_rotation_delta_deg": rotation_delta,
                })
                stable = (
                    translation_change <= float(self.get_parameter(
                        "convergence_improvement_fraction"
                    ).value)
                    and rotation_change <= float(self.get_parameter(
                        "convergence_improvement_fraction"
                    ).value)
                    and translation_delta <= float(self.get_parameter(
                        "convergence_translation_delta_m"
                    ).value)
                    and rotation_delta <= float(self.get_parameter(
                        "convergence_rotation_delta_deg"
                    ).value)
                )
                stable_rounds = stable_rounds + 1 if stable else 0
            convergence_history.append(current)
            if stable_rounds >= required_stable:
                break
            if len(calibration) >= maximum_samples:
                raise RuntimeError(
                    f"calibration did not converge by {len(calibration)} samples"
                )
            previous = {"candidate": final}
            increment_count, required_increment = plan_adaptive_increment(
                len(calibration),
                maximum_samples,
                int(self.get_parameter("incremental_samples").value),
            )
            increment = self._collect_hemisphere(
                full_pool, final.link5_T_camera, anchor,
                increment_count,
                f"increment_{len(convergence_history):02d}", used_candidates,
            )
            if len(increment) < required_increment:
                raise RuntimeError(
                    "adaptive increment produced only "
                    f"{len(increment)}/{required_increment} required valid samples "
                    f"({increment_count} requested)"
                )
            calibration.extend(increment)
            incremental_anchor = self._return_anchor(
                anchor, f"anchor_increment_{len(convergence_history):02d}"
            )
            incremental_drift = pose_dispersion([
                anchor_pair.camera_T_target, incremental_anchor.camera_T_target,
            ])
            if (
                incremental_drift["translation_max_m"] > 0.020
                or incremental_drift["rotation_max_deg"] > 3.0
            ):
                raise RuntimeError(
                    f"incremental Anchor chain drift exceeded limit: {incremental_drift}"
                )
            solved = solve_handeye(calibration, selection)
            if not solved:
                raise RuntimeError("solver failed during adaptive increment")

        link5_T_camera = final.link5_T_camera
        # Test observations are acquired only after solver and thresholds freeze.
        final_base_T_target = aggregate_pose_pairs([
            PosePair(
                item.base_T_link5 @ link5_T_camera @ item.camera_T_target,
                Transform.identity(),
            )
            for item in calibration
        ]).base_T_link5
        self._base_T_target = final_base_T_target
        remaining_test_pool = [
            item for item in full_pool
            if self._candidate_key(item) not in used_candidates
        ]
        test_pool, test_prefilter_rejections = self._refresh_joint_candidates(
            remaining_test_pool, link5_T_camera, final_base_T_target,
            centre_offset, anchor,
        )
        test_count = int(self.get_parameter("test_samples").value)
        registered_test = self._stratified_candidates(
            test_pool, test_count, set()
        )
        if len(registered_test) != test_count:
            raise RuntimeError(
                f"could only pre-register {len(registered_test)}/{test_count} Test poses"
            )
        atomic_write_yaml(self._run_dir / "test_registration.yaml", {
            "registered_before_execution": True,
            "prefilter_input": len(remaining_test_pool),
            "prefilter_valid": len(test_pool),
            "prefilter_rejections": dict(test_prefilter_rejections),
            "primary": [
                {
                    "id": f"test_{index:03d}",
                    "radius_m": item.radius_m,
                    "theta_deg": item.theta_deg,
                    "phi_deg": item.phi_deg,
                    "roll_deg": item.roll_deg,
                    "base_T_camera": self._transform_dict(item.base_T_camera),
                }
                for index, item in enumerate(registered_test, 1)
            ],
            "replacement_policy": "none",
        })
        test_attempts: list[dict] = []
        test = self._collect_hemisphere(
            registered_test, link5_T_camera, anchor, test_count, "test", None,
            fixed_attempts=True, attempt_log=test_attempts, preserve_order=True,
        )
        atomic_write_yaml(self._run_dir / "test_results.yaml", {
            "planned": test_count,
            "valid": len(test),
            "attempts": test_attempts,
        })
        minimum_test = int(self.get_parameter("minimum_valid_test_samples").value)
        if len(test) < minimum_test:
            raise RuntimeError(f"Final Test valid {len(test)}, required {minimum_test}")
        test_metrics = pose_dispersion(board_poses(test, link5_T_camera))
        if (
            test_metrics["translation_p95_m"] > float(self.get_parameter("test_max_translation_p95_m").value)
            or test_metrics["rotation_p95_deg"] > float(self.get_parameter("test_max_rotation_p95_deg").value)
        ):
            raise RuntimeError(f"Final Test accuracy gate failed: {test_metrics}")
        final_anchor = self._return_anchor(anchor, "final_anchor_return")
        final_anchor_drift = pose_dispersion([
            anchor_pair.camera_T_target, final_anchor.camera_T_target,
        ])
        if (
            final_anchor_drift["translation_max_m"] > 0.020
            or final_anchor_drift["rotation_max_deg"] > 3.0
        ):
            raise RuntimeError(
                f"final Anchor chain drift exceeded limit: {final_anchor_drift}"
            )

        name = str(self.get_parameter("calibration_name").value)
        document = calibration_document(
            name, self._base, self._link5, self._camera, self._marker, link5_T_camera
        )
        document["metadata"] = {
            "workspace_mode": self._workspace_mode,
            "workspace_constraints": {
                "tilt_max_deg": 25,
                "z_max": 0.48,
            },
        }
        calibration_root = Path(os.environ.get(
            "EASY_HANDEYE2_CALIBRATIONS_DIRECTORY",
            str(Path.home() / ".ros2" / "easy_handeye2" / "calibrations"),
        ))
        destination = calibration_root / f"{name}.calib"
        if destination.exists():
            shutil.copy2(destination, self._run_dir / "previous_official.calib")
        atomic_write_yaml(destination, document)
        calibration_sha256 = hashlib.sha256(destination.read_bytes()).hexdigest()
        run_stamp = self._run_dir.name.rsplit("_", 1)[-1]
        archive_name = f"{name}__pass_{run_stamp}"
        archive_document = dict(document)
        archive_document["parameters"] = dict(document["parameters"])
        archive_document["parameters"]["name"] = archive_name
        archive_document["metadata"] = dict(document["metadata"])
        archive_document["metadata"]["source_run"] = str(self._run_dir)
        archive_document["metadata"]["result"] = "PASS"
        archive_document["metadata"]["test_metrics"] = test_metrics
        archive_destination = calibration_root / f"{archive_name}.calib"
        atomic_write_yaml(archive_destination, archive_document)
        archive_sha256 = hashlib.sha256(archive_destination.read_bytes()).hexdigest()
        report = {
            "result": "PASS",
            "solver": final.solver,
            "calibration_samples": len(calibration),
            "selection_samples": len(selection),
            "test_planned": test_count,
            "test_valid": len(test),
            "test_attempts": test_attempts,
            "test_metrics": test_metrics,
            "final_anchor_chain_drift": final_anchor_drift,
            "bootstrap_observability": observability,
            "convergence_history": convergence_history,
            "solver_candidates": [asdict(item) | {"link5_T_camera": self._transform_dict(item.link5_T_camera)} for item in solved],
            "output": str(destination),
            "output_sha256": calibration_sha256,
            "archive_output": str(archive_destination),
            "archive_output_sha256": archive_sha256,
            "camera_info_sha256": self._camera_info_hash,
            "workspace_mode": self._workspace_mode,
            "workspace_constraints": {
                "tilt_max_deg": 25,
                "z_max": 0.48,
            },
        }
        atomic_write_yaml(self._run_dir / "final_report.yaml", report)
        self._publish(
            "saved", f"Link5 自动手眼标定通过并已原子保存：{destination}",
            test_metrics=test_metrics, solver=final.solver,
        )


def _normalize_ros_arguments(args):
    """Accept legacy bare -p shorthand; standard ROS arguments stay unchanged.

    Without --ros-args, rclpy treats -p as application arguments and silently
    leaves record_only at its default. Insert the delimiter before the first
    bare parameter flag; malformed options still fail in the ROS parser.
    """
    arguments = list(sys.argv if args is None else args)
    if "--ros-args" not in arguments:
        for index, item in enumerate(arguments):
            if item in ("-p", "--param", "--params-file"):
                arguments.insert(index, "--ros-args")
                break
    return arguments


def main(args=None) -> None:
    rclpy.init(args=_normalize_ros_arguments(args))
    node = PiperHAutoHandeye()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
