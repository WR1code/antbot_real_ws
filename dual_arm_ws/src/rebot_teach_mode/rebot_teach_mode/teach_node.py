"""ROS 2 services for guarded gravity-compensation recording and replay."""

from __future__ import annotations

import json
import math
import os
import threading
import time
from datetime import datetime
from dataclasses import replace
from pathlib import Path
from typing import Sequence

from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    DisplayRobotState,
    DisplayTrajectory,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    ObjectColor,
    RobotState,
    RobotTrajectory,
)
from rebot_teach_msgs.msg import DrawingPath
from rebot_teach_msgs.srv import (
    CheckShapeReachability,
    ConfigureShapeMarker,
    ConfigureTrace,
    CopyActionGroup,
    CreateShapeAction,
    DeleteActionGroup,
    ListActionGroups,
    ListActionSequences,
    PreviewActionGroup,
    PreviewActionSequence,
    ReplayActionGroup,
    ReplayActionSequence,
    RenameActionGroup,
    SaveActionGroup,
    SaveActionSequence,
    SelectActionGroup,
    StartActionGroup,
)
from rebotarm_msgs.msg import ArmStatus
from piper_msgs.msg import PiperTeachJointState
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
    MarkerArray,
)
from moveit_msgs.srv import GetCartesianPath, GetMotionPlan, GetPositionIK

from .action_library import ActionGroupLibrary, validate_action_name
from .forward_kinematics import SerialChainFK
from .piperh_handeye_presets import ensure_piperh_handeye_presets
from .shape_actions import (
    SHAPE_NAMES_ZH,
    SUPPORTED_DRAWINGS,
    SUPPORTED_SHAPES,
    drawing_strokes,
    multiply_quaternions,
    normalize_quaternion,
    parent_pose_for_child_pose,
    pen_tcp_path,
    retime_joint_path,
    sample_cartesian_path,
    shape_outline,
    transform_points,
)
from .trajectory import (
    RecordedPoint,
    RecordedTrajectory,
    TrajectoryRecorder,
    build_preview_loop,
    build_playback_plan,
    concatenate_trajectories_with_transitions,
    load_trajectory,
    prepare_rviz_preview,
    sample_trajectory,
    save_trajectory,
    validate_joint_limits,
)


PREFLIGHT_REACHABLE = "reachable"
PREFLIGHT_NEAR_LIMIT = "near_limit"
PREFLIGHT_COLLISION = "collision"
PREFLIGHT_NO_IK = "no_ik"
PREFLIGHT_PRIORITY = {
    PREFLIGHT_REACHABLE: 0,
    PREFLIGHT_NEAR_LIMIT: 1,
    PREFLIGHT_COLLISION: 2,
    PREFLIGHT_NO_IK: 3,
}


class TeachModeNode(Node):
    """Own the explicit record/replay state without bypassing driver arbitration."""

    def __init__(self) -> None:
        super().__init__("rebot_teach_mode")
        self._group = ReentrantCallbackGroup()
        self._lock = threading.RLock()
        self._declare_parameters()

        self._namespace = str(self.get_parameter("arm_namespace").value).strip("/")
        moveit_namespace = str(
            self.get_parameter("moveit_namespace").value
        ).strip("/")
        self._moveit_prefix = f"/{moveit_namespace}" if moveit_namespace else ""
        self._allow_hardware = bool(self.get_parameter("allow_hardware").value)
        self._preview_only = bool(self.get_parameter("preview_only").value)
        if self._preview_only and self._allow_hardware:
            raise ValueError("preview_only cannot be combined with allow_hardware=true")
        self._joint_names = tuple(
            str(name) for name in self.get_parameter("joint_names").value
        )
        self._trajectory_path = Path(
            str(self.get_parameter("trajectory_path").value)
        ).expanduser()
        self._robot_model = str(self.get_parameter("robot_model").value).strip()
        self._action_library = ActionGroupLibrary(
            str(self.get_parameter("action_library_dir").value),
            self._joint_names,
            robot_model=self._robot_model,
        )
        if self._robot_model == "piperh":
            ensure_piperh_handeye_presets(self._action_library)
        self._feedback_timeout = float(
            self.get_parameter("feedback_timeout_sec").value
        )
        self._status_timeout = float(self.get_parameter("status_timeout_sec").value)
        self._driver_service_timeout = float(
            self.get_parameter("driver_service_timeout_sec").value
        )
        self._preview_frame_period = float(
            self.get_parameter("preview_frame_period_sec").value
        )
        self._preview_maximum_idle = float(
            self.get_parameter("preview_maximum_idle_sec").value
        )
        self._preview_idle_delta = float(
            self.get_parameter("preview_idle_position_delta_rad").value
        )
        self._require_xbox_locked = bool(
            self.get_parameter("require_xbox_locked").value
        )
        self._supports_gravity_compensation = bool(
            self.get_parameter("supports_gravity_compensation").value
        )
        self._teach_drag_mode = str(
            self.get_parameter("teach_drag_mode").value
        ).strip()
        if self._teach_drag_mode not in {"gravity_compensation", "passive_disabled"}:
            raise ValueError(
                "teach_drag_mode must be gravity_compensation or passive_disabled"
            )
        self._maximum_tracking_error = float(
            self.get_parameter("maximum_tracking_error_rad").value
        )
        self._tracking_error_limit = int(
            self.get_parameter("tracking_error_consecutive_samples").value
        )
        self._lower_limits = tuple(
            float(value) for value in self.get_parameter("joint_lower_limits").value
        )
        self._upper_limits = tuple(
            float(value) for value in self.get_parameter("joint_upper_limits").value
        )
        self._joint_limit_margin = float(
            self.get_parameter("joint_limit_margin_rad").value
        )
        if len(self._joint_names) != 6 or len(set(self._joint_names)) != 6:
            raise ValueError("joint_names must contain six unique arm joints")
        if (
            self._feedback_timeout <= 0.0
            or self._status_timeout <= 0.0
            or self._driver_service_timeout <= 0.0
            or self._preview_frame_period <= 0.0
            or self._preview_maximum_idle <= 0.0
            or self._preview_idle_delta < 0.0
        ):
            raise ValueError("feedback, status, and driver service timeouts must be positive")

        self._recorder = TrajectoryRecorder(
            self._joint_names,
            sample_period_sec=float(self.get_parameter("sample_period_sec").value),
            minimum_position_delta=(
                0.0 if self._robot_model == "piperh" else float(
                    self.get_parameter("minimum_position_delta_rad").value
                )
            ),
            maximum_duration_sec=float(
                self.get_parameter("maximum_recording_duration_sec").value
            ),
            maximum_samples=int(self.get_parameter("maximum_samples").value),
        )
        self._trajectory: RecordedTrajectory | None = None
        self._selected_action_name = ""
        self._pending_action_name = ""
        self._pending_action_description = ""
        self._pending_action_overwrite = False
        self._state = "LOCKED" if not self._allow_hardware else "IDLE"
        self._message = (
            "allow_hardware=false; record and replay are disabled"
            if not self._allow_hardware
            else "waiting for fresh robot feedback"
        )
        self._latest_positions: tuple[float, ...] | None = None
        self._latest_joint_time = 0.0
        self._latest_joint_feedback_stamp = 0.0
        self._latest_teach_positions: tuple[float, ...] | None = None
        self._latest_teach_sample_stamp = 0.0
        self._latest_teach_arrival_time = 0.0
        self._latest_teach_metadata: dict = {}
        self._latest_teach_dropped_cycles = 0
        self._latest_teach_wide_span_cycles = 0
        self._recording_dropped_gaps = 0
        self._recording_timed_out = False
        self._recording_source = ""
        self._recording_passive_frames = 0
        self._recording_passive_first_time = 0.0
        self._recording_passive_last_time = 0.0
        self._recording_sample_ticks = 0
        self._recording_duplicate_feedback = 0
        self._recording_gap_counts = {"30ms": 0, "50ms": 0, "100ms": 0}
        self._recording_timestamp_backwards = 0
        self._recording_timing_invalid = False
        self._recording_dropped_cycles_start = 0
        self._recording_wide_span_cycles_start = 0
        self._latest_arm_status: ArmStatus | None = None
        self._latest_status_time = 0.0
        self._xbox_armed: bool | None = None
        self._active_goal = None
        self._active_replay_speed_scale = float(
            self.get_parameter("replay_speed_scale").value
        )
        self._replay_progress = 0.0
        self._replay_plan_duration_sec = 0.0
        self._replay_started_time = 0.0
        self._replay_generation = 0
        self._last_replay_progress_publish = 0.0
        self._replay_sequence_name = ""
        self._replay_action_index = 0
        self._replay_total_actions = 0
        self._tracking_error_count = 0
        self._cancel_requested = False
        self._preview_active = False
        self._preview_paused = False
        self._preview_sequence_name = ""
        self._preview_generation = 0
        self._preview_progress_offset = 0.0
        self._preview_duration_sec = 0.0
        self._rviz_preview_trajectory: RecordedTrajectory | None = None
        self._sequence_preview_cache_key = None
        self._sequence_preview_cache: RecordedTrajectory | None = None
        self._preview_started_time = 0.0
        self._preview_hold_pending = False
        self._active_sequence_name = ""
        self._sequence_queue: list[str] = []
        self._sequence_transitions: list[RecordedTrajectory | None] = []
        self._pending_transition: RecordedTrajectory | None = None
        self._sequence_action_index = 0
        self._sequence_total_actions = 0
        self._watchdog_suppressed_until = 0.0
        if self._robot_model == "piperh":
            self._preview_link_names = (
                "base_link",
                "Link1",
                "Link2",
                "Link3",
                "Link4",
                "Link5",
                "Link6",
            )
        else:
            self._preview_link_names = (
                "base_link",
                "link1",
                "link2",
                "link3",
                "link4",
                "link5",
                "link6",
                "gripper_link",
                "gripper_left",
                "gripper_right",
            )
        self._tcp_base_frame = str(self.get_parameter("tcp_base_frame").value)
        self._tcp_link_name = str(self.get_parameter("tcp_link_name").value)
        visualization_prefix = str(
            self.get_parameter("visualization_frame_prefix").value
        ).strip("/")
        self._visualization_base_frame = (
            f"{visualization_prefix}/{self._tcp_base_frame.lstrip('/')}"
            if visualization_prefix
            else self._tcp_base_frame
        )
        self._visualization_tcp_link_name = (
            f"{visualization_prefix}/{self._tcp_link_name.lstrip('/')}"
            if visualization_prefix
            else self._tcp_link_name
        )
        robot_description = str(self.get_parameter("robot_description").value)
        self._tcp_fk = SerialChainFK(
            robot_description, self._tcp_base_frame, self._tcp_link_name
        ) if robot_description else None
        self._tcp_trace_points: list[tuple[float, float, float]] = []
        self._tcp_trace_times: list[float] = []
        self._tcp_trace_duration = 0.0
        self._tcp_trace_progress = 0.0
        self._last_preview_trace_publish_time = 0.0
        self._tcp_trace_line_width_m = float(
            self.get_parameter("tcp_trace_line_width_m").value
        )
        if (
            not math.isfinite(self._tcp_trace_line_width_m)
            or self._tcp_trace_line_width_m < 0.0005
            or self._tcp_trace_line_width_m > 0.03
        ):
            raise ValueError(
                "tcp_trace_line_width_m must be within 0.0005 to 0.03 metres"
            )
        shape_center = tuple(
            float(value) for value in self.get_parameter("shape_center").value
        )
        if len(shape_center) != 3 or not all(math.isfinite(value) for value in shape_center):
            raise ValueError("shape_center must contain three finite values")
        self._shape_marker_shape = "rectangle"
        self._shape_marker_source = ""
        self._shape_marker_visible = True
        self._shape_preflight = None
        self._shape_preflight_running = False
        self._shape_marker_width = float(self.get_parameter("shape_width").value)
        self._shape_marker_height = float(self.get_parameter("shape_height").value)
        self._shape_pen_length_m = float(
            self.get_parameter("shape_pen_length_m").value
        )
        self._shape_pen_mount_offset_m = float(
            self.get_parameter("shape_pen_mount_offset_m").value
        )
        self._shape_pen_lift_m = float(self.get_parameter("shape_pen_lift_m").value)
        self._validate_pen_geometry(
            self._shape_pen_length_m, self._shape_pen_lift_m
        )
        if (
            not math.isfinite(self._shape_pen_mount_offset_m)
            or self._shape_pen_mount_offset_m < 0.0
            or self._shape_pen_mount_offset_m > 0.30
        ):
            raise ValueError("pen mount offset must be within 0.0 to 0.30 metres")
        self._shape_marker_pose = Pose(
            position=Point(x=shape_center[0], y=shape_center[1], z=shape_center[2]),
            orientation=Quaternion(w=1.0),
        )
        self._shape_marker_server = InteractiveMarkerServer(
            self, f"/{self._namespace}/teach/shape_marker"
        )
        self._refresh_shape_marker_locked()

        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._status_publisher = self.create_publisher(
            String, f"/{self._namespace}/teach/status", state_qos
        )
        preview_qos = QoSProfile(depth=1)
        preview_qos.reliability = ReliabilityPolicy.RELIABLE
        preview_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._display_trajectory_publisher = self.create_publisher(
            DisplayTrajectory,
            f"/{self._namespace}/display_planned_path",
            preview_qos,
        )
        self._display_robot_state_publisher = self.create_publisher(
            DisplayRobotState,
            f"/{self._namespace}/display_robot_state",
            QoSProfile(depth=1),
        )
        self._tcp_trace_publisher = self.create_publisher(
            MarkerArray, f"/{self._namespace}/teach/tcp_trajectory", preview_qos
        )
        self._drawing_path_publisher = self.create_publisher(
            DrawingPath, f"/{self._namespace}/teach/drawing_path", preview_qos
        )
        self._publish_drawing_path()
        self._publish_tcp_trace(0.0)
        self.create_subscription(
            JointState,
            f"/{self._namespace}/joint_states",
            self._joint_state_callback,
            qos_profile_sensor_data,
            callback_group=self._group,
        )
        if self._robot_model == "piperh":
            teach_feedback_qos = QoSProfile(
                depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(
                PiperTeachJointState,
                f"/{self._namespace}/normal_joint_feedback",
                self._teach_joint_state_callback,
                teach_feedback_qos,
                callback_group=self._group,
            )
        self.create_subscription(
            ArmStatus,
            f"/{self._namespace}/arm_status",
            self._arm_status_callback,
            state_qos,
            callback_group=self._group,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("xbox_armed_topic").value),
            self._xbox_armed_callback,
            state_qos,
            callback_group=self._group,
        )

        self._gravity_start = self.create_client(
            Trigger,
            f"/{self._namespace}/gravity_compensation/start",
            callback_group=self._group,
        )
        self._gravity_stop = self.create_client(
            Trigger,
            f"/{self._namespace}/gravity_compensation/stop",
            callback_group=self._group,
        )
        trajectory_action = str(
            self.get_parameter("trajectory_action").value
        ).strip() or f"/{self._namespace}/follow_joint_trajectory"
        self._trajectory_action = ActionClient(
            self,
            FollowJointTrajectory,
            trajectory_action,
            callback_group=self._group,
        )
        self._ik_client = self.create_client(
            GetPositionIK, f"{self._moveit_prefix}/compute_ik", callback_group=self._group
        )
        self._cartesian_path_client = self.create_client(
            GetCartesianPath,
            f"{self._moveit_prefix}/compute_cartesian_path",
            callback_group=self._group,
        )
        self._motion_plan_client = self.create_client(
            GetMotionPlan,
            f"{self._moveit_prefix}/plan_kinematic_path",
            callback_group=self._group,
        )
        for name, callback in (
            ("gravity_mode/start", self._start_gravity_mode),
            ("gravity_mode/stop", self._stop_gravity_mode),
            ("start_recording", self._start_recording),
            ("stop_recording", self._stop_recording),
            ("replay", self._replay),
            ("cancel", self._cancel),
            ("pause_preview", self._pause_preview),
            ("clear_action_selection", self._clear_action_selection),
            ("reload", self._reload),
            ("reset", self._reset),
        ):
            self.create_service(
                Trigger,
                f"/{self._namespace}/teach/{name}",
                callback,
                callback_group=self._group,
            )
        self.create_service(
            CreateShapeAction,
            f"/{self._namespace}/teach/create_shape_action",
            self._create_shape_action,
            callback_group=self._group,
        )
        self.create_service(
            ConfigureShapeMarker,
            f"/{self._namespace}/teach/configure_shape_marker",
            self._configure_shape_marker,
            callback_group=self._group,
        )
        self.create_service(
            CheckShapeReachability,
            f"/{self._namespace}/teach/check_shape_reachability",
            self._check_shape_reachability,
            callback_group=self._group,
        )
        self.create_service(
            ConfigureTrace,
            f"/{self._namespace}/teach/configure_trace",
            self._configure_trace,
            callback_group=self._group,
        )
        self.create_service(
            StartActionGroup,
            f"/{self._namespace}/teach/start_action_group",
            self._start_action_group,
            callback_group=self._group,
        )
        self.create_service(
            SaveActionGroup,
            f"/{self._namespace}/teach/save_action_group",
            self._save_action_group,
            callback_group=self._group,
        )
        self.create_service(
            SelectActionGroup,
            f"/{self._namespace}/teach/select_action_group",
            self._select_action_group,
            callback_group=self._group,
        )
        self.create_service(
            ListActionGroups,
            f"/{self._namespace}/teach/list_action_groups",
            self._list_action_groups,
            callback_group=self._group,
        )
        self.create_service(
            ReplayActionGroup,
            f"/{self._namespace}/teach/replay_action_group",
            self._replay_action_group,
            callback_group=self._group,
        )
        self.create_service(
            PreviewActionGroup,
            f"/{self._namespace}/teach/preview_action_group",
            self._preview_action_group,
            callback_group=self._group,
        )
        self.create_service(
            PreviewActionSequence,
            f"/{self._namespace}/teach/preview_action_sequence",
            self._preview_action_sequence,
            callback_group=self._group,
        )
        self.create_service(
            RenameActionGroup,
            f"/{self._namespace}/teach/rename_action_group",
            self._rename_action_group,
            callback_group=self._group,
        )
        self.create_service(
            DeleteActionGroup,
            f"/{self._namespace}/teach/delete_action_group",
            self._delete_action_group,
            callback_group=self._group,
        )
        self.create_service(
            CopyActionGroup,
            f"/{self._namespace}/teach/copy_action_group",
            self._copy_action_group,
            callback_group=self._group,
        )
        self.create_service(
            ListActionSequences,
            f"/{self._namespace}/teach/list_action_sequences",
            self._list_action_sequences,
            callback_group=self._group,
        )
        self.create_service(
            SaveActionSequence,
            f"/{self._namespace}/teach/save_action_sequence",
            self._save_action_sequence,
            callback_group=self._group,
        )
        self.create_service(
            ReplayActionSequence,
            f"/{self._namespace}/teach/replay_action_sequence",
            self._replay_action_sequence,
            callback_group=self._group,
        )
        self.create_timer(0.1, self._watchdog, callback_group=self._group)
        self.create_timer(0.5, self._publish_status, callback_group=self._group)
        self.create_timer(
            self._preview_frame_period,
            self._advance_rviz_preview,
            callback_group=self._group,
        )
        self._try_load_existing()
        self._publish_status()

    def _declare_parameters(self) -> None:
        self.declare_parameter("arm_namespace", "rebotarm")
        self.declare_parameter("moveit_namespace", "")
        self.declare_parameter("xbox_armed_topic", "/rebot_xbox/armed")
        self.declare_parameter("allow_hardware", False)
        self.declare_parameter("preview_only", False)
        self.declare_parameter(
            "joint_names",
            ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        )
        self.declare_parameter(
            "trajectory_path",
            str(Path(os.environ.get("ROBOT_MOTION_ROOT", "motions")) / "rs/teach/latest.motion.json"),
        )
        self.declare_parameter(
            "action_library_dir",
            str(Path(os.environ.get("ROBOT_MOTION_ROOT", "motions")) / "rs/action_groups"),
        )
        self.declare_parameter("robot_model", "rebotarm_rs")
        self.declare_parameter("trajectory_action", "")
        self.declare_parameter("sample_period_sec", 0.02)
        self.declare_parameter("minimum_position_delta_rad", 0.0005)
        self.declare_parameter("minimum_recording_duration_sec", 0.5)
        self.declare_parameter("minimum_recording_points", 5)
        self.declare_parameter("maximum_recording_duration_sec", 120.0)
        self.declare_parameter("maximum_samples", 6000)
        self.declare_parameter("feedback_timeout_sec", 0.30)
        self.declare_parameter("teach_drag_mode", "passive_disabled")
        self.declare_parameter("status_timeout_sec", 0.50)
        self.declare_parameter("driver_service_timeout_sec", 5.0)
        self.declare_parameter("preview_frame_period_sec", 0.02)
        self.declare_parameter("preview_maximum_idle_sec", 0.20)
        self.declare_parameter("preview_idle_position_delta_rad", 0.002)
        self.declare_parameter("require_xbox_locked", True)
        self.declare_parameter("supports_gravity_compensation", True)
        self.declare_parameter("endpoint_tolerance_rad", 0.08)
        self.declare_parameter("replay_speed_scale", 1.0)
        self.declare_parameter("maximum_joint_velocity_rad_s", 0.30)
        self.declare_parameter("reverse_return", True)
        self.declare_parameter("start_dwell_sec", 0.5)
        self.declare_parameter("sequence_transition_direct_tolerance_rad", 0.01)
        self.declare_parameter("sequence_transition_planning_time_sec", 5.0)
        self.declare_parameter("sequence_transition_service_timeout_sec", 15.0)
        self.declare_parameter("sequence_transition_velocity_scaling_factor", 0.1)
        self.declare_parameter("maximum_tracking_error_rad", 0.25)
        self.declare_parameter("tracking_error_consecutive_samples", 5)
        self.declare_parameter(
            "joint_lower_limits", [-2.8, 0.0, 0.0, -1.57, -1.57, -3.14]
        )
        self.declare_parameter(
            "joint_upper_limits", [2.8, 3.14, 3.14, 1.57, 1.57, 3.14]
        )
        self.declare_parameter("joint_limit_margin_rad", 0.0)
        self.declare_parameter("robot_description", "")
        self.declare_parameter("tcp_base_frame", "base_link")
        self.declare_parameter("tcp_link_name", "gripper_tcp")
        self.declare_parameter("visualization_frame_prefix", "")
        self.declare_parameter("shape_center", [0.28, 0.0, 0.12])
        self.declare_parameter("shape_width", 0.06)
        self.declare_parameter("shape_height", 0.08)
        self.declare_parameter("shape_tcp_rpy", [0.0, 1.57, 0.0])
        self.declare_parameter(
            "shape_tcp_yaw_offsets", [0.0, math.pi, -math.pi]
        )
        self.declare_parameter("shape_cartesian_step_m", 0.003)
        self.declare_parameter("shape_jump_threshold", 0.0)
        self.declare_parameter("shape_planning_time_sec", 5.0)
        self.declare_parameter("shape_service_timeout_sec", 15.0)
        self.declare_parameter("shape_maximum_joint_velocity_rad_s", 0.15)
        self.declare_parameter("shape_maximum_start_drift_rad", 0.02)
        self.declare_parameter("shape_marker_scale_m", 0.09)
        self.declare_parameter("shape_pen_length_m", 0.0)
        self.declare_parameter("shape_pen_mount_offset_m", 0.0)
        self.declare_parameter("shape_pen_lift_m", 0.012)
        self.declare_parameter("shape_preflight_maximum_points", 60)
        self.declare_parameter("shape_preflight_joint_warning_margin_rad", 0.15)
        self.declare_parameter("shape_preflight_ik_timeout_sec", 0.08)
        self.declare_parameter("shape_preflight_suggestion_step_m", 0.02)
        self.declare_parameter("shape_preflight_suggestion_steps", 3)
        self.declare_parameter("image_maximum_strokes", 24)
        self.declare_parameter("image_maximum_points", 480)
        self.declare_parameter("image_minimum_contour_length_px", 24.0)
        self.declare_parameter("image_simplify_epsilon_px", 2.0)
        self.declare_parameter("tcp_trace_line_width_m", 0.004)

    def _now(self) -> float:
        return time.monotonic()

    @staticmethod
    def _validate_pen_geometry(pen_length: float, pen_lift: float) -> None:
        if (
            not math.isfinite(pen_length)
            or pen_length < 0.0
            or pen_length > 0.50
        ):
            raise ValueError("pen length must be within 0.0 to 0.50 metres")
        if not math.isfinite(pen_lift) or pen_lift < 0.001 or pen_lift > 0.10:
            raise ValueError("pen lift must be within 0.001 to 0.10 metres")

    def _pen_tcp_to_tip_m(self) -> float:
        return self._shape_pen_mount_offset_m + self._shape_pen_length_m

    def _joint_state_callback(self, message: JointState) -> None:
        try:
            if len(message.name) != len(message.position):
                raise ValueError("joint name/position length mismatch")
            if len(message.name) != len(set(message.name)):
                raise ValueError("duplicate joint names")
            lookup = dict(zip(message.name, message.position))
            positions = tuple(float(lookup[name]) for name in self._joint_names)
            if not all(math.isfinite(value) for value in positions):
                raise ValueError("non-finite joint feedback")
            feedback_stamp = 0.0
            if self._robot_model == "piperh":
                feedback_stamp = (
                    float(message.header.stamp.sec)
                    + float(message.header.stamp.nanosec) * 1e-9
                )
                if not math.isfinite(feedback_stamp) or feedback_stamp <= 0.0:
                    raise ValueError("missing or invalid Piper-H joint frame timestamp")
        except (KeyError, ValueError) as error:
            self.get_logger().warning(f"rejecting joint feedback: {error}")
            return
        with self._lock:
            # Sample the monotonic clock only after acquiring the state lock.
            # A joint callback can queue while a slow DM mode-switch service
            # holds this lock; taking the timestamp before the lock would let
            # that older sample arrive after Recorder.start() and appear to
            # move backwards in time.
            now = self._now()
            if self._robot_model == "piperh":
                if feedback_stamp <= self._latest_joint_feedback_stamp:
                    if self._state == "RECORDING" and self._recording_source == "normal":
                        self._recording_duplicate_feedback += 1
                    return
                self._latest_joint_feedback_stamp = feedback_stamp
            self._latest_positions = positions
            self._latest_joint_time = now
            piper_recording = (
                getattr(self, "_robot_model", "") == "piperh"
                and getattr(self, "_recording_source", "") == "normal"
            )
            if self._state == "RECORDING" and self._robot_model != "piperh":
                try:
                    self._recorder.add(now, positions)
                except Exception as error:
                    self._enter_fault(
                        f"recording failed: {error}", stop_gravity=not piper_recording
                    )
            if not self._preview_active and self._preview_hold_pending:
                self._publish_preview_pose(positions)

    @staticmethod
    def _message_time(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _teach_joint_state_callback(self, message: PiperTeachJointState) -> None:
        """Record one complete 2A5/2A6/2A7 cycle on its CAN sample time."""
        try:
            if tuple(message.name) != self._joint_names or len(message.position) != 6:
                raise ValueError("expected ordered joint1..joint6 teach feedback")
            positions = tuple(float(value) for value in message.position)
            stamps = tuple(self._message_time(value) for value in (
                message.timestamp_j12, message.timestamp_j34, message.timestamp_j56))
            sample_stamp = self._message_time(message.sample_timestamp)
            span = float(message.intra_cycle_span)
            arrival = float(message.callback_monotonic_time)
            if not all(math.isfinite(value) for value in positions + stamps + (sample_stamp, span, arrival,)):
                raise ValueError("non-finite teach feedback")
            if min(stamps) <= 0 or sample_stamp != max(stamps) or span < 0:
                raise ValueError("invalid grouped CAN timestamps")
            metadata = {
                "timestamp_j12": stamps[0], "timestamp_j34": stamps[1],
                "timestamp_j56": stamps[2], "callback_monotonic_time": arrival,
                "intra_cycle_span": span, "timing_valid": bool(message.timing_valid),
            }
        except ValueError as error:
            self.get_logger().warning(f"rejecting grouped teach feedback: {error}")
            return
        with self._lock:
            previous_stamp = self._latest_teach_sample_stamp
            self._latest_teach_positions = positions
            self._latest_teach_sample_stamp = sample_stamp
            self._latest_teach_arrival_time = self._now()
            self._latest_teach_metadata = metadata
            self._latest_teach_dropped_cycles = int(message.dropped_cycles)
            self._latest_teach_wide_span_cycles = int(message.wide_span_cycles)
            if self._state != "RECORDING" or self._recording_source != "normal":
                return
            if previous_stamp and sample_stamp <= previous_stamp:
                self._recording_timestamp_backwards += 1
                self._recording_timing_invalid = True
                return
            if previous_stamp:
                gap = sample_stamp - previous_stamp
                for threshold, name in ((.030, "30ms"), (.050, "50ms"), (.100, "100ms")):
                    if gap > threshold:
                        self._recording_gap_counts[name] += 1
                if gap > 1.0:
                    self._recording_timing_invalid = True
            if not message.timing_valid:
                self._recording_timing_invalid = True
            try:
                self._piper_recording_preflight()
                if self._recorder.add(sample_stamp, positions, force=True,
                                      feedback_timestamp=sample_stamp,
                                      sample_metadata=metadata):
                    self._recording_passive_frames += 1
                    self._recording_passive_last_time = sample_stamp
            except Exception as error:
                self._enter_fault(f"recording failed: {error}", stop_gravity=False)

    def _piper_recording_preflight(self) -> tuple[float, ...]:
        """Validate normal feedback for the selected Piper-H drag mode."""
        if self._robot_model != "piperh":
            raise RuntimeError("passive recording is only valid for Piper-H")
        now = self._now()
        if not self._allow_hardware:
            raise PermissionError("allow_hardware=false")
        if self._latest_arm_status is None or now - self._latest_status_time > self._status_timeout:
            raise RuntimeError("arm status is missing or stale")
        if self._latest_arm_status.error_codes:
            raise RuntimeError("arm reports errors")
        if self._teach_drag_mode == "passive_disabled":
            if self._latest_arm_status.state_machine != "DISABLED" or self._latest_arm_status.enabled:
                raise RuntimeError("Piper-H motors must be disabled for passive recording")
        elif (
            self._latest_arm_status.state_machine != "GRAVITY_COMPENSATION_ACTIVE"
            or not self._latest_arm_status.enabled
        ):
            raise RuntimeError("Piper-H gravity compensation is not active")
        if self._require_xbox_locked and self._xbox_armed is not False:
            raise RuntimeError("Xbox lock state is unknown or ARMED")
        if (
            self._latest_teach_positions is None
            or self._now() - self._latest_teach_arrival_time > self._feedback_timeout
        ):
            raise RuntimeError("grouped Piper-H teach feedback is missing or stale")
        return self._latest_teach_positions

    def _arm_status_callback(self, message: ArmStatus) -> None:
        with self._lock:
            self._latest_arm_status = message
            self._latest_status_time = self._now()
            if message.error_codes and self._state in {"RECORDING", "REPLAYING"}:
                self._enter_fault(
                    "arm reported errors: " + ", ".join(message.error_codes),
                    stop_gravity=(
                        self._state == "RECORDING"
                        and self._teach_drag_mode == "gravity_compensation"
                    ),
                    cancel_goal=self._state == "REPLAYING",
                )

    def _xbox_armed_callback(self, message: Bool) -> None:
        with self._lock:
            self._xbox_armed = bool(message.data)
            if self._xbox_armed and self._state in {"RECORDING", "REPLAYING"}:
                self._enter_fault(
                    "Xbox Servo became ARMED during teach mode",
                    stop_gravity=(
                        self._state == "RECORDING"
                        and self._teach_drag_mode == "gravity_compensation"
                    ),
                    cancel_goal=self._state == "REPLAYING",
                )

    def _preflight(
        self,
        *,
        expected_driver_states: set[str],
        require_enabled: bool = True,
    ) -> tuple[float, ...]:
        now = self._now()
        if not self._allow_hardware:
            raise PermissionError("allow_hardware=false")
        if (
            self._latest_positions is None
            or now - self._latest_joint_time > self._feedback_timeout
        ):
            raise RuntimeError("joint feedback is missing or stale")
        if (
            self._latest_arm_status is None
            or now - self._latest_status_time > self._status_timeout
        ):
            raise RuntimeError("arm status is missing or stale")
        if self._latest_arm_status.error_codes:
            raise RuntimeError("arm reports errors")
        violations = self._joint_limit_violations(self._latest_positions)
        if violations:
            detail = "; ".join(violations)
            if self._robot_model == "piperh":
                raise RuntimeError(
                    "current Piper-H feedback is outside MoveIt joint limits: "
                    f"{detail}; disable the motors and use the manufacturer-approved "
                    "recovery procedure to return every joint inside its limit before planning"
                )
            raise RuntimeError(
                f"current robot feedback is outside configured joint limits: {detail}"
            )
        if require_enabled and not self._latest_arm_status.enabled:
            raise RuntimeError("arm is not enabled")
        if self._latest_arm_status.state_machine not in expected_driver_states:
            raise RuntimeError(
                "driver state must be "
                + "/".join(sorted(expected_driver_states))
                + f", got {self._latest_arm_status.state_machine}"
            )
        if self._require_xbox_locked and self._xbox_armed is not False:
            raise RuntimeError("Xbox lock state is unknown or ARMED")
        return self._latest_positions

    def _joint_limit_violations(
        self, positions: Sequence[float]
    ) -> tuple[str, ...]:
        violations = []
        for name, value, lower, upper in zip(
            self._joint_names, positions, self._lower_limits, self._upper_limits
        ):
            guarded_lower = lower + self._joint_limit_margin
            guarded_upper = upper - self._joint_limit_margin
            if not guarded_lower <= value <= guarded_upper:
                violations.append(
                    f"{name}={value:.6f} rad, expected "
                    f"[{guarded_lower:.6f}, {guarded_upper:.6f}]"
                )
        return tuple(violations)

    def _call_trigger(self, client, label: str, timeout_sec: float | None = None):
        timeout_sec = (
            self._driver_service_timeout
            if timeout_sec is None
            else float(timeout_sec)
        )
        if not client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError(f"{label} service is unavailable")
        future = client.call_async(Trigger.Request())
        event = threading.Event()
        future.add_done_callback(lambda _future: event.set())
        if not event.wait(timeout_sec):
            raise TimeoutError(f"{label} service timed out")
        result = future.result()
        if result is None or not result.success:
            detail = result.message if result is not None else "no response"
            raise RuntimeError(f"{label} failed: {detail}")
        return result

    def _call_trigger_while_state_unlocked(self, client, label: str):
        """Wait for a driver service without starving feedback/executor callbacks."""
        self._lock.release()
        try:
            return self._call_trigger(client, label)
        finally:
            self._lock.acquire()

    def _wait_for_preflight_while_state_unlocked(
        self, expected_driver_states: set[str], timeout_sec: float = 2.0,
        require_enabled: bool = True,
    ) -> tuple[float, ...]:
        """Let status callbacks run before checking a completed mode switch."""
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        last_error: Exception | None = None
        self._lock.release()
        try:
            while True:
                with self._lock:
                    try:
                        return self._preflight(
                            expected_driver_states=expected_driver_states,
                            require_enabled=require_enabled,
                        )
                    except Exception as error:
                        last_error = error
                if time.monotonic() >= deadline:
                    raise RuntimeError(
                        f"driver transition completed without fresh safe feedback: {last_error}"
                    ) from last_error
                time.sleep(0.02)
        finally:
            self._lock.acquire()

    def _send_trajectory_goal_while_state_unlocked(self, goal):
        """Send an action goal without starving its executor response callback."""
        self._lock.release()
        try:
            if not self._trajectory_action.wait_for_server(timeout_sec=2.0):
                raise RuntimeError("follow_joint_trajectory action is unavailable")
            future = self._trajectory_action.send_goal_async(
                goal, feedback_callback=self._trajectory_feedback
            )
            event = threading.Event()
            future.add_done_callback(lambda _future: event.set())
            if not event.wait(3.0):
                raise TimeoutError("trajectory goal response timed out")
            goal_handle = future.result()
            if goal_handle is None or not goal_handle.accepted:
                raise RuntimeError("trajectory goal was rejected")
            return goal_handle
        finally:
            self._lock.acquire()

    def _start_recording(self, _request, response):
        with self._lock:
            try:
                name = datetime.now().strftime("示教动作_%Y%m%d_%H%M%S_%f")
                self._begin_recording_locked(
                    name,
                    "由兼容 Trigger/Xbox 接口自动命名的拖动示教动作",
                    overwrite=False,
                )
                response.success = True
                response.message = self._message
            except Exception as error:
                response.success = False
                response.message = str(error)
                self._message = response.message
            self._publish_status()
            return response

    def _start_gravity_mode(self, _request, response):
        """Start compensation without recording, serialized with teach actions."""
        with self._lock:
            previous_state = self._state
            try:
                if not self._supports_gravity_compensation:
                    raise RuntimeError("gravity compensation is unavailable for this model")
                if self._state not in {"IDLE", "READY"}:
                    raise RuntimeError(f"cannot start gravity mode from {self._state}")
                self._preflight(expected_driver_states={"IDLE"})
                self._state = "GRAVITY_PRECHECK" if self._robot_model == "piperh" else "STARTING"
                self._message = "checking normal feedback and gravity compensation safety"
                self._publish_status()
                self._call_trigger_while_state_unlocked(
                    self._gravity_start, "gravity compensation start"
                )
                if self._robot_model == "piperh":
                    self._wait_for_preflight_while_state_unlocked(
                        {"GRAVITY_COMPENSATION_ACTIVE"}
                    )
                    self._state = previous_state
                    self._message = "gravity compensation active; normal feedback remains recording source"
                else:
                    self._wait_for_preflight_while_state_unlocked({"GRAVITY_COMP"})
                    self._state = previous_state
                    self._message = "standalone gravity compensation active"
                response.success = True
            except Exception as error:
                self._state = previous_state
                self._message = (
                    f"gravity compensation unavailable: {error}; "
                    "disable the arm and use teach_drag_mode=passive_disabled"
                )
                response.success = False
            response.message = self._message
            self._publish_status()
            return response

    def _stop_gravity_mode(self, _request, response):
        """A standalone stop cannot interrupt an active drag recording."""
        with self._lock:
            previous_state = self._state
            try:
                if not self._supports_gravity_compensation:
                    raise RuntimeError("gravity compensation is unavailable for this model")
                if self._state not in {"IDLE", "READY"}:
                    raise RuntimeError(f"cannot stop gravity mode from {self._state}")
                if (
                    self._latest_arm_status is None
                    or self._latest_arm_status.state_machine != "GRAVITY_COMPENSATION_ACTIVE"
                ):
                    raise RuntimeError("driver gravity compensation is not active")
                self._state = "STOPPING"
                self._message = "stopping standalone gravity compensation"
                self._publish_status()
                self._call_trigger_while_state_unlocked(
                    self._gravity_stop, "gravity compensation stop"
                )
                self._wait_for_preflight_while_state_unlocked({"IDLE"})
                self._state = "READY" if self._trajectory else "IDLE"
                self._message = "standalone gravity compensation stopped"
                response.success = True
            except Exception as error:
                self._state = previous_state
                self._message = f"gravity compensation stop failed: {error}"
                response.success = False
            response.message = self._message
            self._publish_status()
            return response

    def _start_action_group(self, request, response):
        with self._lock:
            try:
                self._begin_recording_locked(
                    request.name,
                    request.description,
                    overwrite=bool(request.overwrite),
                )
                response.success = True
                response.message = self._message
            except Exception as error:
                response.success = False
                response.message = str(error)
                self._message = response.message
            self._publish_status()
            return response

    def _begin_recording_locked(
        self, name: str, description: str, *, overwrite: bool
    ) -> None:
        gravity_supported = getattr(self, "_supports_gravity_compensation", True)
        allowed_states = {"IDLE", "READY"}
        if self._state not in allowed_states:
            raise RuntimeError(f"cannot record from state {self._state}")
        normalized = validate_action_name(name)
        target = self._action_library.path_for(normalized)
        if target.exists() and not overwrite:
            raise FileExistsError(
                f"action group {normalized!r} already exists; set overwrite=true"
            )
        if self._robot_model == "piperh":
            if self._teach_drag_mode == "gravity_compensation":
                self._preflight(expected_driver_states={"IDLE", "GRAVITY_COMPENSATION_ACTIVE"})
                if self._latest_arm_status.state_machine == "IDLE":
                    self._state = "GRAVITY_PRECHECK"
                    self._message = "starting gravity compensation from normal feedback"
                    self._publish_status()
                    try:
                        self._call_trigger_while_state_unlocked(
                            self._gravity_start, "gravity compensation start"
                        )
                        self._wait_for_preflight_while_state_unlocked(
                            {"GRAVITY_COMPENSATION_ACTIVE"}
                        )
                    except Exception as error:
                        self._state = "READY" if self._trajectory else "IDLE"
                        raise RuntimeError(
                            f"gravity compensation precheck failed: {error}; disable the arm "
                            "and select teach_drag_mode=passive_disabled"
                        ) from error
            positions = self._piper_recording_preflight()
            recording_source = "normal"
        elif gravity_supported:
            positions = self._preflight(
                expected_driver_states={"IDLE", "GRAVITY_COMP"}
            )
        else:
            positions = self._preflight(
                expected_driver_states={"DISABLED"}, require_enabled=False
            )
            if self._latest_arm_status.enabled:
                raise RuntimeError(
                    "disable Piper-H motors before passive action recording"
                )
        if (
            gravity_supported
            and self._robot_model != "piperh"
            and self._latest_arm_status.state_machine == "IDLE"
        ):
            self._state = "STARTING"
            self._message = "switching driver to gravity compensation; please wait"
            self._publish_status()
            try:
                self._call_trigger_while_state_unlocked(
                    self._gravity_start, "gravity compensation start"
                )
                positions = self._wait_for_preflight_while_state_unlocked(
                    {"GRAVITY_COMP"}
                )
            except Exception as error:
                self._clear_pending_action()
                driver_idle = (
                    self._latest_arm_status is not None
                    and self._latest_arm_status.state_machine == "IDLE"
                    and self._now() - self._latest_status_time <= self._status_timeout
                )
                self._state = ("READY" if self._trajectory else "IDLE") if driver_idle else "FAULT"
                self._message = f"recording start failed: {error}"
                if not driver_idle and self._gravity_stop.service_is_ready():
                    self._gravity_stop.call_async(Trigger.Request())
                raise RuntimeError(self._message) from error
            # The slow driver service owns the hardware lock while switching
            # modes, so feedback callbacks queued during that interval need a
            # brief chance to catch up before the recording watchdog runs.
            self._watchdog_suppressed_until = self._now() + max(
                1.0, self._feedback_timeout * 3.0, self._status_timeout * 2.0
            )
        self._recording_source = recording_source if self._robot_model == "piperh" else ""
        if self._robot_model == "piperh":
            self._recorder.start(
                self._latest_teach_sample_stamp, positions,
                feedback_timestamp=self._latest_teach_sample_stamp,
                sample_metadata=self._latest_teach_metadata,
            )
            self._recording_passive_frames = 1
            self._recording_passive_first_time = self._latest_teach_sample_stamp
            self._recording_passive_last_time = self._latest_teach_sample_stamp
            self._recording_sample_ticks = 0
            self._recording_duplicate_feedback = 0
            self._recording_dropped_gaps = 0
            self._recording_timed_out = False
            self._recording_gap_counts = {"30ms": 0, "50ms": 0, "100ms": 0}
            self._recording_timestamp_backwards = 0
            self._recording_timing_invalid = not self._latest_teach_metadata.get("timing_valid", True)
            self._recording_dropped_cycles_start = self._latest_teach_dropped_cycles
            self._recording_wide_span_cycles_start = self._latest_teach_wide_span_cycles
        else:
            self._recorder.start(self._now(), positions)
        self._pending_action_name = normalized
        self._pending_action_description = str(description).strip()
        self._pending_action_overwrite = overwrite
        self._state = "RECORDING"
        self._message = (
            f"recording Piper-H normal feedback ({self._teach_drag_mode}) "
            f"as action group {normalized!r}; call stop to save"
            if self._robot_model == "piperh"
            else f"recording action group {normalized!r}; release X or call stop"
        )

    def _stop_recording(self, _request, response):
        with self._lock:
            gravity_supported = getattr(
                self, "_supports_gravity_compensation", True
            )
            if self._state != "RECORDING":
                if self._state == "FAULT":
                    response.success = False
                    response.message = f"already in FAULT: {self._message}"
                    # Preserve the first fault: it is the actionable cause.
                    self._publish_status()
                    return response
                response.success = False
                response.message = f"cannot stop recording from state {self._state}"
                self._publish_status()
                return response

            gravity_stopped = False
            try:
                recording_source = getattr(self, "_recording_source", "")
                if self._robot_model == "piperh":
                    positions = self._piper_recording_preflight()
                    velocities = efforts = None
                elif gravity_supported:
                    positions = self._preflight(
                        expected_driver_states={"GRAVITY_COMP"}
                    )
                    velocities = efforts = None
                else:
                    positions = self._preflight(
                        expected_driver_states={"DISABLED"}, require_enabled=False
                    )
                    velocities = efforts = None
                self._state = "STOPPING"
                self._message = (
                    f"saving Piper-H normal-feedback recording ({self._teach_drag_mode})"
                    if self._robot_model == "piperh"
                    else "stopping gravity compensation and saving; please wait"
                    if gravity_supported
                    else "saving passive Piper-H recording; please wait"
                )
                self._publish_status()
                if gravity_supported and (
                    self._robot_model != "piperh"
                    or self._teach_drag_mode == "gravity_compensation"
                ):
                    self._call_trigger_while_state_unlocked(
                        self._gravity_stop, "gravity compensation stop"
                    )
                    positions = self._wait_for_preflight_while_state_unlocked(
                        {"IDLE"}
                    )
                    gravity_stopped = True
                finish_kwargs = (
                    {
                        "feedback_timestamp": self._latest_teach_sample_stamp,
                        "sample_metadata": {
                            **self._latest_teach_metadata,
                            "timing_valid": not self._recording_timing_invalid,
                        },
                    }
                    if self._robot_model == "piperh" else {}
                )
                trajectory = self._recorder.finish(
                    self._latest_teach_sample_stamp if self._robot_model == "piperh" else self._now(),
                    positions,
                    velocities=velocities,
                    efforts=efforts,
                    minimum_points=int(
                        self.get_parameter("minimum_recording_points").value
                    ),
                    minimum_duration_sec=float(
                        self.get_parameter("minimum_recording_duration_sec").value
                    ),
                    **finish_kwargs,
                )
                if self._robot_model == "piperh":
                    trajectory = replace(
                        trajectory,
                        timing_valid=not self._recording_timing_invalid,
                        joint_limits_valid=not any(
                            self._joint_limit_violations(point.positions)
                            for point in trajectory.points
                        ),
                    )
                else:
                    self._validate_trajectory(trajectory)
                save_trajectory(self._trajectory_path, trajectory)
                info = self._action_library.save(
                    self._pending_action_name,
                    self._pending_action_description,
                    trajectory,
                    overwrite=self._pending_action_overwrite,
                )
                self._trajectory = trajectory
                self._selected_action_name = info.name
                self._clear_pending_action()
                self._state = "READY"
                self._publish_recorded_preview(trajectory)
                passive_duration = max(
                    0.0,
                    self._recording_passive_last_time - self._recording_passive_first_time,
                ) if recording_source == "normal" else 0.0
                passive_hz = (
                    (self._recording_passive_frames - 1) / passive_duration
                    if passive_duration > 0.0 and self._recording_passive_frames > 1
                    else 0.0
                )
                saved_hz = (
                    (len(trajectory.points) - 1) / trajectory.duration
                    if trajectory.duration > 0.0 else 0.0
                )
                self._message = (
                    f"saved action group {info.name!r}: {len(trajectory.points)} points, "
                    f"duration={trajectory.duration:.2f}s, "
                    f"grouped CAN feedback={passive_hz:.1f} Hz, "
                    f"saved={saved_hz:.1f} Hz, "
                    f"dropped cycles={max(0, self._latest_teach_dropped_cycles - self._recording_dropped_cycles_start)}, "
                    f"wide spans={max(0, self._latest_teach_wide_span_cycles - self._recording_wide_span_cycles_start)}, "
                    f"gaps>30/50/100ms={self._recording_gap_counts['30ms']}/"
                    f"{self._recording_gap_counts['50ms']}/{self._recording_gap_counts['100ms']}, "
                    f"timestamp backwards={self._recording_timestamp_backwards}, "
                    f"timing_valid={str(not self._recording_timing_invalid).lower()}, "
                    f"dropped gaps={self._recording_dropped_gaps}, "
                    f"timeout={str(self._recording_timed_out).lower()}"
                )
                response.success = True
                response.message = self._message
            except Exception as error:
                self._recorder.discard()
                self._clear_pending_action()
                if self._robot_model == "piperh" and (
                    self._latest_arm_status is not None
                    and self._latest_arm_status.state_machine == "DISABLED"
                    and not self._latest_arm_status.enabled
                ):
                    self._state = "READY" if self._trajectory else "IDLE"
                    self._message = f"passive recording was not saved: {error}"
                    self.get_logger().warning(f"[TEACH MODE] {self._message}")
                elif gravity_stopped:
                    # The driver has completed the stop transition. A short
                    # recording, invalid sample set, or save error should be retryable and
                    # must not strand the RViz controls in FAULT.
                    self._state = "READY" if self._trajectory else "IDLE"
                    self._message = f"recording was not saved: {error}"
                    self.get_logger().warning(
                        f"[TEACH MODE] {self._message}"
                    )
                else:
                    self._state = "FAULT"
                    self._message = f"recording stop failed: {error}"
                    self.get_logger().error(
                        f"[TEACH MODE] FAULT: {self._message}"
                    )
                    if self._robot_model != "piperh" and self._gravity_stop.service_is_ready():
                        self._gravity_stop.call_async(Trigger.Request())
                response.success = False
                response.message = self._message
            self._publish_status()
            return response

    def _clear_pending_action(self) -> None:
        self._pending_action_name = ""
        self._pending_action_description = ""
        self._pending_action_overwrite = False

    def _save_action_group(self, request, response):
        with self._lock:
            try:
                if self._state not in {"LOCKED", "IDLE", "READY"}:
                    raise RuntimeError(
                        f"cannot save an action group from state {self._state}"
                    )
                if self._trajectory is None:
                    raise RuntimeError("no validated teaching trajectory is loaded")
                self._validate_trajectory(self._trajectory)
                info = self._action_library.save(
                    request.name,
                    request.description,
                    self._trajectory,
                    overwrite=bool(request.overwrite),
                )
                self._selected_action_name = info.name
                self._publish_recorded_preview(self._trajectory)
                response.success = True
                response.message = f"saved action group {info.name!r}"
                response.path = str(info.path)
                self._message = response.message
            except Exception as error:
                response.success = False
                response.message = str(error)
                response.path = ""
                self._message = response.message
            self._publish_status()
            return response

    def _select_action_group(self, request, response):
        with self._lock:
            try:
                if self._state not in {"LOCKED", "IDLE", "READY"}:
                    raise RuntimeError(
                        f"cannot select an action group from state {self._state}"
                    )
                info, trajectory = self._action_library.load(request.name)
                self._validate_trajectory(trajectory)
                self._trajectory = trajectory
                self._selected_action_name = info.name
                self._state = "READY" if self._allow_hardware else "LOCKED"
                self._publish_recorded_preview(trajectory)
                self._message = f"selected action group {info.name!r}"
                response.success = True
                response.message = self._message
                response.path = str(info.path)
            except Exception as error:
                response.success = False
                response.message = str(error)
                response.path = ""
                self._message = response.message
            self._publish_status()
            return response

    def _clear_action_selection(self, _request, response):
        """Enter new-action mode and remove any old RViz trajectory display."""
        with self._lock:
            try:
                if self._state not in {"LOCKED", "IDLE", "READY"}:
                    raise RuntimeError(
                        f"cannot clear the action selection from state {self._state}"
                    )
                positions = self._latest_positions
                if positions is None and self._trajectory is not None:
                    positions = self._trajectory.points[0].positions
                if positions is not None:
                    self._publish_preview_pose(positions)
                else:
                    self._preview_active = False
                    self._preview_paused = False
                    self._preview_sequence_name = ""
                    self._rviz_preview_trajectory = None
                    self._tcp_trace_points = []
                    self._tcp_trace_times = []
                    self._tcp_trace_duration = 0.0
                    self._display_trajectory_publisher.publish(DisplayTrajectory())
                    self._publish_tcp_trace(0.0)
                self._clear_sequence_locked()
                self._selected_action_name = ""
                self._trajectory = None
                self._state = "IDLE" if self._allow_hardware else "LOCKED"
                self._message = (
                    "cleared the selected action and RViz preview; hardware unchanged"
                )
                response.success = True
                response.message = self._message
            except Exception as error:
                response.success = False
                response.message = str(error)
                self._message = response.message
            self._publish_status()
            return response

    def _list_action_groups(self, _request, response):
        with self._lock:
            groups, warnings = self._action_library.list_groups()
            response.success = True
            response.message = (
                f"found {len(groups)} action groups"
                if not warnings
                else f"found {len(groups)} action groups; " + "; ".join(warnings)
            )
            response.names = [item.name for item in groups]
            response.descriptions = [item.description for item in groups]
            response.paths = [str(item.path) for item in groups]
            response.metadata_json = [
                json.dumps(item.shape_parameters, ensure_ascii=False)
                if item.shape_parameters is not None
                else "{}"
                for item in groups
            ]
            response.selected_name = self._selected_action_name
            return response

    def _rename_action_group(self, request, response):
        with self._lock:
            try:
                if self._state not in {"LOCKED", "IDLE", "READY"}:
                    raise RuntimeError(
                        f"cannot rename an action group from state {self._state}"
                    )
                old_name = validate_action_name(request.old_name)
                info = self._action_library.rename(old_name, request.new_name)
                if self._selected_action_name == old_name:
                    self._selected_action_name = info.name
                response.success = True
                response.message = f"renamed action group {old_name!r} to {info.name!r}"
                response.path = str(info.path)
                self._message = response.message
            except Exception as error:
                response.success = False
                response.message = str(error)
                response.path = ""
                self._message = response.message
            self._publish_status()
            return response

    def _delete_action_group(self, request, response):
        with self._lock:
            try:
                if self._state not in {"LOCKED", "IDLE", "READY"}:
                    raise RuntimeError(
                        f"cannot delete an action group from state {self._state}"
                    )
                name = validate_action_name(request.name)
                info = self._action_library.delete(name)
                if self._selected_action_name == name:
                    positions = self._latest_positions
                    if positions is None and self._trajectory is not None:
                        positions = self._trajectory.points[0].positions
                    if positions is not None:
                        self._publish_preview_pose(positions)
                    self._selected_action_name = ""
                    self._trajectory = None
                    self._state = "IDLE" if self._allow_hardware else "LOCKED"
                response.success = True
                response.message = f"deleted action group {name!r}"
                response.path = str(info.path)
                self._message = response.message
            except Exception as error:
                response.success = False
                response.message = str(error)
                response.path = ""
                self._message = response.message
            self._publish_status()
            return response

    def _copy_action_group(self, request, response):
        with self._lock:
            try:
                if self._state not in {"LOCKED", "IDLE", "READY"}:
                    raise RuntimeError(
                        f"cannot copy an action group from state {self._state}"
                    )
                source_name = validate_action_name(request.source_name)
                _, trajectory = self._action_library.load(source_name)
                self._validate_trajectory(trajectory)
                info = self._action_library.copy(source_name, request.new_name)
                self._trajectory = trajectory
                self._selected_action_name = info.name
                self._publish_recorded_preview(trajectory)
                if self._allow_hardware:
                    self._state = "READY"
                response.success = True
                response.message = (
                    f"copied action group {source_name!r} to {info.name!r}"
                )
                response.path = str(info.path)
                self._message = response.message
            except Exception as error:
                response.success = False
                response.message = str(error)
                response.path = ""
                self._message = response.message
            self._publish_status()
            return response

    def _list_action_sequences(self, _request, response):
        with self._lock:
            sequences, warnings = self._action_library.list_sequences()
            response.success = True
            response.message = (
                f"found {len(sequences)} action sequences"
                if not warnings
                else f"found {len(sequences)} action sequences; " + "; ".join(warnings)
            )
            response.names = [item.name for item in sequences]
            response.action_lists_json = [
                json.dumps(item.action_names, ensure_ascii=False)
                for item in sequences
            ]
            return response

    def _save_action_sequence(self, request, response):
        with self._lock:
            try:
                if self._state not in {"LOCKED", "IDLE", "READY"}:
                    raise RuntimeError(
                        f"cannot save an action sequence from state {self._state}"
                    )
                info = self._action_library.save_sequence(
                    request.name,
                    request.action_names,
                    overwrite=bool(request.overwrite),
                )
                response.success = True
                response.message = (
                    f"saved action sequence {info.name!r} with "
                    f"{len(info.action_names)} actions"
                )
                response.path = str(info.path)
                self._message = response.message
            except Exception as error:
                response.success = False
                response.message = str(error)
                response.path = ""
                self._message = response.message
            self._publish_status()
            return response

    @staticmethod
    def _quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        return Quaternion(
            x=sr * cp * cy - cr * sp * sy,
            y=cr * sp * cy + sr * cp * sy,
            z=cr * cp * sy - sr * sp * cy,
            w=cr * cp * cy + sr * sp * sy,
        )

    @staticmethod
    def _quaternion_values(quaternion: Quaternion) -> tuple[float, float, float, float]:
        return normalize_quaternion((
            quaternion.x, quaternion.y, quaternion.z, quaternion.w
        ))

    @staticmethod
    def _copy_pose(pose: Pose) -> Pose:
        return Pose(
            position=Point(x=pose.position.x, y=pose.position.y, z=pose.position.z),
            orientation=Quaternion(
                x=pose.orientation.x,
                y=pose.orientation.y,
                z=pose.orientation.z,
                w=pose.orientation.w,
            ),
        )

    def _shape_signature(self):
        pose = self._shape_marker_pose
        # RViz may round-trip an InteractiveMarker pose with harmless floating
        # point noise.  A sub-micrometre change must not invalidate a completed
        # reachability check.
        stable = lambda value: round(float(value), 7)
        return (
            self._shape_marker_shape,
            self._shape_marker_source,
            stable(self._shape_marker_width),
            stable(self._shape_marker_height),
            stable(self._shape_pen_length_m),
            stable(self._shape_pen_lift_m),
            stable(pose.position.x), stable(pose.position.y),
            stable(pose.position.z), stable(pose.orientation.x),
            stable(pose.orientation.y), stable(pose.orientation.z),
            stable(pose.orientation.w),
        )

    def _shape_preflight_markers(self) -> list[Marker]:
        result = self._shape_preflight
        if not result or result.get("signature") != self._shape_signature():
            return []
        points = result["local_tip_points"]
        statuses = result["statuses"]
        colors = {
            PREFLIGHT_REACHABLE: (0.12, 0.82, 0.30),
            PREFLIGHT_NEAR_LIMIT: (1.0, 0.78, 0.05),
            PREFLIGHT_COLLISION: (0.78, 0.12, 0.92),
            PREFLIGHT_NO_IK: (0.95, 0.04, 0.04),
        }
        # Put an identical line layer on each side of the drawing plane.  This
        # prevents the plane fill, the total-motion trace, or back-face depth
        # testing from hiding the result at oblique camera angles.
        # Keep the two faces close enough to read as one stroke.  A large
        # separation produces a visibly doubled/distorted drawing in side view.
        visual_offset = max(0.00025, self._tcp_trace_line_width_m * 0.10)
        markers = {}
        for side_index, side in enumerate((-1.0, 1.0)):
            for status_index, status in enumerate(colors):
                for lifted_index, lifted in enumerate((False, True)):
                    marker = Marker()
                    marker.type = Marker.LINE_LIST
                    marker.action = Marker.ADD
                    marker.pose.orientation.w = 1.0
                    marker.scale.x = (
                        max(0.001, self._tcp_trace_line_width_m * 0.35)
                        if lifted else
                        max(0.003, self._tcp_trace_line_width_m * 1.25)
                    )
                    marker.color.r, marker.color.g, marker.color.b = colors[status]
                    marker.color.a = 0.38 if lifted else 1.0
                    marker.id = (
                        side_index * len(colors) * 2 + status_index * 2 + lifted_index
                    )
                    markers[(side, status, lifted)] = marker
        for index, (first, second) in enumerate(zip(points, points[1:])):
            status = max(
                (statuses[index], statuses[index + 1]),
                key=lambda value: PREFLIGHT_PRIORITY[value],
            )
            lifted = max(first[2], second[2]) > 1e-5
            # Lift/transfer segments are thin, translucent and dashed.  This
            # keeps them available for safety diagnosis without obscuring the
            # actual pen-down content.
            if lifted and index % 2:
                continue
            for side in (-1.0, 1.0):
                markers[(side, status, lifted)].points.extend((
                    Point(
                        x=first[0], y=first[1],
                        z=first[2] + side * visual_offset,
                    ),
                    Point(
                        x=second[0], y=second[1],
                        z=second[2] + side * visual_offset,
                    ),
                ))
        return [marker for marker in markers.values() if marker.points]

    def _shape_pose(
        self, position, yaw_offset: float, shape_frame_orientation: Quaternion
    ) -> Pose:
        roll, pitch, yaw = (
            float(value) for value in self.get_parameter("shape_tcp_rpy").value
        )
        local_orientation = self._quaternion_from_rpy(roll, pitch, yaw + yaw_offset)
        orientation = multiply_quaternions(
            self._quaternion_values(shape_frame_orientation),
            self._quaternion_values(local_orientation),
        )
        return Pose(
            position=Point(x=position[0], y=position[1], z=position[2]),
            orientation=Quaternion(
                x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3]
            ),
        )

    def _shape_pose_at_current_tcp(self) -> Pose | None:
        """Align the first pen-up waypoint with the live TCP pose."""
        if self._tcp_fk is None or self._latest_positions is None:
            return None
        joints = dict(zip(self._joint_names, self._latest_positions))
        tcp_position, tcp_orientation = self._tcp_fk.pose(joints)
        local_strokes = self._local_drawing_strokes()
        first_local_tcp = pen_tcp_path(
            local_strokes, self._pen_tcp_to_tip_m(), self._shape_pen_lift_m
        )[0]
        roll, pitch, yaw = (
            float(value) for value in self.get_parameter("shape_tcp_rpy").value
        )
        local_orientation = self._quaternion_values(
            self._quaternion_from_rpy(roll, pitch, yaw)
        )
        position, orientation = parent_pose_for_child_pose(
            first_local_tcp, local_orientation, tcp_position, tcp_orientation
        )
        return Pose(
            position=Point(x=position[0], y=position[1], z=position[2]),
            orientation=Quaternion(
                x=orientation[0], y=orientation[1],
                z=orientation[2], w=orientation[3],
            ),
        )

    def _local_drawing_strokes(
        self, drawing: str | None = None, source: str | None = None
    ) -> tuple[tuple[tuple[float, float, float], ...], ...]:
        return drawing_strokes(
            self._shape_marker_shape if drawing is None else drawing,
            self._shape_marker_source if source is None else source,
            self._shape_marker_width,
            self._shape_marker_height,
            image_maximum_strokes=int(
                self.get_parameter("image_maximum_strokes").value
            ),
            image_maximum_points=int(
                self.get_parameter("image_maximum_points").value
            ),
            image_minimum_contour_length_px=float(
                self.get_parameter("image_minimum_contour_length_px").value
            ),
            image_simplify_epsilon_px=float(
                self.get_parameter("image_simplify_epsilon_px").value
            ),
        )

    def _shape_marker_message(self) -> InteractiveMarker:
        marker = InteractiveMarker()
        marker.header.frame_id = self._visualization_base_frame
        marker.name = "drawing_shape_pose"
        marker.description = f"绘图位置：{SHAPE_NAMES_ZH[self._shape_marker_shape]}"
        marker.scale = float(self.get_parameter("shape_marker_scale_m").value)
        marker.pose = self._copy_pose(self._shape_marker_pose)

        preview_control = InteractiveMarkerControl()
        preview_control.name = "shape_outline"
        preview_control.always_visible = True
        preview_control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D
        local_strokes = self._local_drawing_strokes()
        if self._shape_marker_shape in SUPPORTED_SHAPES:
            local_outline = local_strokes[0]
            fill_marker = Marker()
            fill_marker.type = Marker.TRIANGLE_LIST
            fill_marker.action = Marker.ADD
            fill_marker.color.r = 1.0
            fill_marker.color.g = 0.58
            fill_marker.color.b = 0.08
            fill_marker.color.a = 0.12
            for first, second in zip(local_outline, local_outline[1:]):
                fill_marker.points.extend((
                    Point(x=0.0, y=0.0, z=0.0),
                    Point(x=first[0], y=first[1], z=first[2]),
                    Point(x=second[0], y=second[1], z=second[2]),
                    Point(x=0.0, y=0.0, z=0.0),
                    Point(x=second[0], y=second[1], z=second[2]),
                    Point(x=first[0], y=first[1], z=first[2]),
                ))
            preview_control.markers.append(fill_marker)
        preflight_markers = self._shape_preflight_markers()
        # Before preflight, the requested pen-down drawing is cyan. Render it just above and just
        # below the mathematical plane so it remains visible from both sides
        # and does not z-fight with the translucent filled shape.
        if preflight_markers:
            preview_control.markers.extend(preflight_markers)
        else:
            visual_offset = max(0.00025, self._tcp_trace_line_width_m * 0.10)
            for side in (-1.0, 1.0):
                outline_marker = Marker()
                outline_marker.type = Marker.LINE_LIST
                outline_marker.action = Marker.ADD
                outline_marker.scale.x = self._tcp_trace_line_width_m
                outline_marker.color.r = 0.05
                outline_marker.color.g = 0.72
                outline_marker.color.b = 1.0
                outline_marker.color.a = 1.0
                for stroke in local_strokes:
                    for first, second in zip(stroke, stroke[1:]):
                        outline_marker.points.extend((
                            Point(
                                x=first[0], y=first[1],
                                z=first[2] + side * visual_offset,
                            ),
                            Point(
                                x=second[0], y=second[1],
                                z=second[2] + side * visual_offset,
                            ),
                        ))
                preview_control.markers.append(outline_marker)
        center_marker = Marker()
        center_marker.type = Marker.SPHERE
        center_marker.action = Marker.ADD
        center_marker.scale.x = 0.012
        center_marker.scale.y = 0.012
        center_marker.scale.z = 0.012
        center_marker.color.r = 1.0
        center_marker.color.g = 0.84
        center_marker.color.b = 0.10
        center_marker.color.a = 1.0
        preview_control.markers.append(center_marker)
        normal_marker = Marker()
        normal_marker.type = Marker.ARROW
        normal_marker.action = Marker.ADD
        normal_marker.scale.x = 0.008
        normal_marker.scale.y = 0.015
        normal_marker.scale.z = 0.020
        normal_marker.color.r = 1.0
        normal_marker.color.g = 0.25
        normal_marker.color.b = 0.20
        normal_marker.color.a = 1.0
        normal_marker.points = [
            Point(x=0.0, y=0.0, z=0.0),
            Point(
                x=0.0,
                y=0.0,
                z=max(0.06, self._shape_pen_length_m + self._shape_pen_lift_m),
            ),
        ]
        preview_control.markers.append(normal_marker)
        if self._shape_pen_length_m > 1e-6:
            pen_marker = Marker()
            pen_marker.type = Marker.CYLINDER
            pen_marker.action = Marker.ADD
            # The marker origin is the physical tip. Only render the exposed
            # segment back to the gripper front; TCP is farther on local +Z.
            pen_marker.pose.position.z = self._shape_pen_length_m * 0.5
            pen_marker.pose.orientation.w = 1.0
            pen_marker.scale.x = 0.008
            pen_marker.scale.y = 0.008
            pen_marker.scale.z = self._shape_pen_length_m
            pen_marker.color.r = 0.70
            pen_marker.color.g = 0.32
            pen_marker.color.b = 0.95
            pen_marker.color.a = 0.92
            preview_control.markers.append(pen_marker)
        tcp_marker = Marker()
        tcp_marker.type = Marker.SPHERE
        tcp_marker.action = Marker.ADD
        tcp_marker.pose.position.z = self._pen_tcp_to_tip_m()
        tcp_marker.pose.orientation.w = 1.0
        tcp_marker.scale.x = 0.014
        tcp_marker.scale.y = 0.014
        tcp_marker.scale.z = 0.014
        tcp_marker.color.r = 0.82
        tcp_marker.color.g = 0.40
        tcp_marker.color.b = 1.0
        tcp_marker.color.a = 1.0
        preview_control.markers.append(tcp_marker)
        mount_marker = Marker()
        mount_marker.type = Marker.SPHERE
        mount_marker.action = Marker.ADD
        mount_marker.pose.position.z = self._shape_pen_length_m
        mount_marker.pose.orientation.w = 1.0
        mount_marker.scale.x = 0.012
        mount_marker.scale.y = 0.012
        mount_marker.scale.z = 0.012
        mount_marker.color.r = 0.20
        mount_marker.color.g = 0.85
        mount_marker.color.b = 1.0
        mount_marker.color.a = 1.0
        preview_control.markers.append(mount_marker)
        tip_marker = Marker()
        tip_marker.type = Marker.SPHERE
        tip_marker.action = Marker.ADD
        tip_marker.pose.orientation.w = 1.0
        tip_marker.scale.x = 0.009
        tip_marker.scale.y = 0.009
        tip_marker.scale.z = 0.009
        tip_marker.color.r = 1.0
        tip_marker.color.g = 0.86
        tip_marker.color.b = 0.05
        tip_marker.color.a = 1.0
        preview_control.markers.append(tip_marker)
        name_marker = Marker()
        name_marker.type = Marker.TEXT_VIEW_FACING
        name_marker.action = Marker.ADD
        name_marker.pose.position.z = 0.075
        name_marker.scale.z = 0.025
        name_marker.color.r = 1.0
        name_marker.color.g = 0.84
        name_marker.color.b = 0.10
        name_marker.color.a = 1.0
        name_marker.text = (
            f"当前绘图：{SHAPE_NAMES_ZH[self._shape_marker_shape]}  "
            f"{self._shape_marker_width:.3f} x {self._shape_marker_height:.3f} m  "
            f"L={self._shape_pen_length_m * 1000.0:.1f} mm  "
            f"抬笔={self._shape_pen_lift_m * 1000.0:.1f} mm"
        )
        if preflight_markers:
            counts = self._shape_preflight["counts"]
            name_marker.text += (
                f"\n预检：绿 {counts[PREFLIGHT_REACHABLE]} / "
                f"黄 {counts[PREFLIGHT_NEAR_LIMIT]} / "
                f"紫 {counts[PREFLIGHT_COLLISION]} / "
                f"红 {counts[PREFLIGHT_NO_IK]}"
            )
        preview_control.markers.append(name_marker)
        marker.controls.append(preview_control)

        root_half = math.sqrt(0.5)
        axes = (
            ("x", Quaternion(x=root_half, w=root_half)),
            ("y", Quaternion(z=root_half, w=root_half)),
            ("z", Quaternion(y=root_half, w=root_half)),
        )
        for axis, orientation in axes:
            rotate = InteractiveMarkerControl()
            rotate.name = f"rotate_{axis}"
            rotate.orientation = orientation
            rotate.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            marker.controls.append(rotate)
            move = InteractiveMarkerControl()
            move.name = f"move_{axis}"
            move.orientation = orientation
            move.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            marker.controls.append(move)
        return marker

    def _publish_drawing_path(
        self,
        local_strokes: Sequence[Sequence[Sequence[float]]] | None = None,
        marker_pose: Pose | None = None,
    ) -> None:
        """Publish the concise, stroke-aware Cartesian source path."""
        if not hasattr(self, "_drawing_path_publisher"):
            return
        message = DrawingPath()
        message.header.frame_id = self._visualization_base_frame
        message.header.stamp = self.get_clock().now().to_msg()
        message.drawing_type = self._shape_marker_shape
        message.source = self._shape_marker_source
        message.pen_length_m = self._shape_pen_length_m
        message.pen_lift_m = self._shape_pen_lift_m
        if not self._shape_marker_visible or self._shape_marker_shape == "freehand" or (
            self._shape_marker_shape == "image" and not self._shape_marker_source
        ):
            self._drawing_path_publisher.publish(message)
            return
        strokes = self._local_drawing_strokes() if local_strokes is None else local_strokes
        pose = self._shape_marker_pose if marker_pose is None else marker_pose
        translation = (pose.position.x, pose.position.y, pose.position.z)
        orientation = self._quaternion_values(pose.orientation)
        for stroke in strokes:
            message.stroke_start_indices.append(len(message.points))
            for point in transform_points(stroke, translation, orientation):
                message.points.append(Point(x=point[0], y=point[1], z=point[2]))
        self._drawing_path_publisher.publish(message)

    def _shape_size_handle_message(self, axis: str) -> InteractiveMarker:
        is_width = axis == "width"
        local_position = (
            (self._shape_marker_width * 0.5, 0.0, 0.0)
            if is_width
            else (0.0, self._shape_marker_height * 0.5, 0.0)
        )
        world_position = transform_points(
            (local_position,),
            (
                self._shape_marker_pose.position.x,
                self._shape_marker_pose.position.y,
                self._shape_marker_pose.position.z,
            ),
            self._quaternion_values(self._shape_marker_pose.orientation),
        )[0]
        marker = InteractiveMarker()
        marker.header.frame_id = self._visualization_base_frame
        marker.name = f"drawing_shape_{axis}_handle"
        marker.description = "拖动调整宽度 W" if is_width else "拖动调整高度 H"
        marker.scale = 0.055
        marker.pose = Pose(
            position=Point(
                x=world_position[0], y=world_position[1], z=world_position[2]
            ),
            orientation=self._copy_pose(self._shape_marker_pose).orientation,
        )
        control = InteractiveMarkerControl()
        control.name = f"resize_{axis}"
        control.always_visible = True
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation = (
            Quaternion(w=1.0)
            if is_width
            else Quaternion(z=math.sqrt(0.5), w=math.sqrt(0.5))
        )
        handle = Marker()
        handle.type = Marker.CUBE
        handle.action = Marker.ADD
        handle.scale.x = 0.018
        handle.scale.y = 0.018
        handle.scale.z = 0.018
        handle.color.r = 1.0
        handle.color.g = 0.84
        handle.color.b = 0.10
        handle.color.a = 1.0
        control.markers.append(handle)
        marker.controls.append(control)
        return marker

    def _refresh_shape_marker_locked(self) -> None:
        for name in (
            "drawing_shape_pose",
            "drawing_shape_width_handle",
            "drawing_shape_height_handle",
        ):
            self._shape_marker_server.erase(name)
        if self._shape_marker_visible:
            self._shape_marker_server.insert(
                self._shape_marker_message(),
                feedback_callback=self._shape_marker_feedback,
            )
            self._shape_marker_server.insert(
                self._shape_size_handle_message("width"),
                feedback_callback=self._shape_width_feedback,
            )
            self._shape_marker_server.insert(
                self._shape_size_handle_message("height"),
                feedback_callback=self._shape_height_feedback,
            )
        self._shape_marker_server.applyChanges()

    def _hide_shape_visualization_locked(self) -> None:
        """Remove the single-action canvas before sequence preview or replay."""
        self._shape_marker_visible = False
        self._refresh_shape_marker_locked()
        self._publish_drawing_path()

    def _shape_size_feedback(
        self, feedback: InteractiveMarkerFeedback, axis: str
    ) -> None:
        if feedback.event_type != InteractiveMarkerFeedback.POSE_UPDATE:
            return
        with self._lock:
            if self._state == "SHAPE_PLANNING" or self._shape_preflight_running:
                self._refresh_shape_marker_locked()
                return
            center = self._shape_marker_pose.position
            orientation = self._quaternion_values(
                self._shape_marker_pose.orientation
            )
            unit = (1.0, 0.0, 0.0) if axis == "width" else (0.0, 1.0, 0.0)
            direction = transform_points(
                (unit,), (0.0, 0.0, 0.0), orientation
            )[0]
            delta = (
                feedback.pose.position.x - center.x,
                feedback.pose.position.y - center.y,
                feedback.pose.position.z - center.z,
            )
            extent = abs(sum(a * b for a, b in zip(delta, direction)))
            size = min(0.50, max(0.01, extent * 2.0))
            if axis == "width":
                self._shape_marker_width = size
            else:
                self._shape_marker_height = size
            self._message = (
                f"drawing shape {axis} adjusted to {size:.3f} m in RViz; "
                "hardware unchanged"
            )
            self._refresh_shape_marker_locked()
            self._publish_drawing_path()
            self._publish_status()

    def _shape_width_feedback(self, feedback: InteractiveMarkerFeedback) -> None:
        self._shape_size_feedback(feedback, "width")

    def _shape_height_feedback(self, feedback: InteractiveMarkerFeedback) -> None:
        self._shape_size_feedback(feedback, "height")

    def _shape_marker_feedback(self, feedback: InteractiveMarkerFeedback) -> None:
        if feedback.event_type != InteractiveMarkerFeedback.POSE_UPDATE:
            return
        with self._lock:
            if self._state == "SHAPE_PLANNING" or self._shape_preflight_running:
                self._shape_marker_server.setPose(
                    "drawing_shape_pose", self._copy_pose(self._shape_marker_pose)
                )
                self._shape_marker_server.applyChanges()
                return
            try:
                orientation = normalize_quaternion((
                    feedback.pose.orientation.x,
                    feedback.pose.orientation.y,
                    feedback.pose.orientation.z,
                    feedback.pose.orientation.w,
                ))
                position = (
                    float(feedback.pose.position.x),
                    float(feedback.pose.position.y),
                    float(feedback.pose.position.z),
                )
                if not all(math.isfinite(value) for value in position):
                    raise ValueError("shape marker position is not finite")
                self._shape_marker_pose = Pose(
                    position=Point(x=position[0], y=position[1], z=position[2]),
                    orientation=Quaternion(
                        x=orientation[0], y=orientation[1],
                        z=orientation[2], w=orientation[3],
                    ),
                )
                self._message = "drawing shape pose adjusted in RViz; hardware unchanged"
            except ValueError as error:
                self.get_logger().warning(f"rejecting shape marker feedback: {error}")
                self._shape_marker_server.setPose(
                    "drawing_shape_pose", self._copy_pose(self._shape_marker_pose)
                )
                self._shape_marker_server.applyChanges()
                return
            self._publish_drawing_path()
            self._publish_status()

    def _configure_shape_marker(self, request, response):
        with self._lock:
            try:
                if self._state == "SHAPE_PLANNING" or self._shape_preflight_running:
                    raise RuntimeError(
                        "shape marker is frozen while MoveIt is checking or planning"
                    )
                shape = str(request.shape).strip().lower()
                if shape == "freehand":
                    self._shape_marker_visible = False
                    self._refresh_shape_marker_locked()
                    self._shape_marker_shape = "freehand"
                    self._shape_marker_source = ""
                    self._publish_drawing_path()
                    response.success = True
                    response.message = (
                        "freehand recording selected; no preset shape marker; "
                        "hardware unchanged"
                    )
                    self._message = response.message
                    self._publish_status()
                    return response
                if shape not in SUPPORTED_DRAWINGS:
                    raise ValueError(
                        f"unsupported drawing {shape!r}; expected {', '.join(SUPPORTED_DRAWINGS)}"
                    )
                source = str(request.source).strip()
                # Image mode is allowed to remain empty while the file picker is
                # open. It simply has no marker until a readable image is selected.
                if shape != "image" or source:
                    self._local_drawing_strokes(shape, source)
                self._shape_marker_shape = shape
                self._shape_marker_source = source
                self._shape_marker_visible = bool(request.visible) and not (
                    shape == "image" and not source
                )
                if request.reset_pose:
                    self._shape_marker_width = float(
                        self.get_parameter("shape_width").value
                    )
                    self._shape_marker_height = float(
                        self.get_parameter("shape_height").value
                    )
                    self._shape_pen_length_m = float(
                        self.get_parameter("shape_pen_length_m").value
                    )
                    self._shape_pen_mount_offset_m = float(
                        self.get_parameter("shape_pen_mount_offset_m").value
                    )
                    self._shape_pen_lift_m = float(
                        self.get_parameter("shape_pen_lift_m").value
                    )
                    current_pose = self._shape_pose_at_current_tcp()
                    if current_pose is not None:
                        self._shape_marker_pose = current_pose
                    else:
                        center = tuple(
                            float(value)
                            for value in self.get_parameter("shape_center").value
                        )
                        self._shape_marker_pose = Pose(
                            position=Point(x=center[0], y=center[1], z=center[2]),
                            orientation=Quaternion(w=1.0),
                        )
                elif request.set_pose:
                    values = (
                        float(request.x), float(request.y), float(request.z),
                        float(request.roll), float(request.pitch), float(request.yaw),
                    )
                    if not all(math.isfinite(value) for value in values):
                        raise ValueError("shape pose values must be finite")
                    self._shape_marker_pose = Pose(
                        position=Point(x=values[0], y=values[1], z=values[2]),
                        orientation=self._quaternion_from_rpy(
                            values[3], values[4], values[5]
                        ),
                    )
                if request.set_size:
                    width = float(request.width)
                    height = float(request.height)
                    if (
                        not math.isfinite(width)
                        or not math.isfinite(height)
                        or width < 0.01
                        or height < 0.01
                        or width > 0.50
                        or height > 0.50
                    ):
                        raise ValueError(
                            "shape width and height must be within 0.01 to 0.50 metres"
                        )
                    self._shape_marker_width = width
                    self._shape_marker_height = height
                if request.set_pen:
                    pen_length = float(request.pen_length)
                    pen_lift = float(request.pen_lift)
                    self._validate_pen_geometry(pen_length, pen_lift)
                    self._shape_pen_length_m = pen_length
                    self._shape_pen_lift_m = pen_lift
                self._refresh_shape_marker_locked()
                self._publish_drawing_path()
                self._publish_tcp_trace(self._tcp_trace_progress)
                response.success = True
                response.message = (
                    f"shape marker set to {shape}; "
                    f"{'visible' if self._shape_marker_visible else 'hidden'}; hardware unchanged"
                )
                self._message = response.message
            except Exception as error:
                response.success = False
                response.message = str(error)
                self._message = response.message
            self._publish_status()
            return response

    def _configure_trace(self, request, response):
        """Read or update the RViz-only TCP trajectory line width."""
        with self._lock:
            try:
                if request.set_line_width:
                    line_width = float(request.line_width_m)
                    if (
                        not math.isfinite(line_width)
                        or line_width < 0.0005
                        or line_width > 0.03
                    ):
                        raise ValueError(
                            "trace line width must be within 0.0005 to 0.03 metres"
                        )
                    self._tcp_trace_line_width_m = line_width
                    self._refresh_shape_marker_locked()
                    self._publish_tcp_trace(self._tcp_trace_progress)
                    self._message = (
                        f"TCP trace line width set to {line_width * 1000.0:.1f} mm; "
                        "hardware unchanged"
                    )
                response.success = True
                response.message = self._message if request.set_line_width else (
                    f"TCP trace line width is "
                    f"{self._tcp_trace_line_width_m * 1000.0:.1f} mm"
                )
            except Exception as error:
                response.success = False
                response.message = str(error)
                self._message = response.message
            response.line_width_m = self._tcp_trace_line_width_m
            self._publish_status()
            return response

    def _call_service_while_state_unlocked(
        self, client, request, label: str, timeout_sec: float
    ):
        """Call a MoveIt service without starving its executor callback."""
        self._lock.release()
        try:
            if not client.wait_for_service(timeout_sec=2.0):
                raise RuntimeError(f"{label} service is unavailable")
            future = client.call_async(request)
            event = threading.Event()
            future.add_done_callback(lambda _future: event.set())
            if not event.wait(timeout_sec):
                raise TimeoutError(f"{label} timed out")
            result = future.result()
            if result is None:
                raise RuntimeError(f"{label} returned no response")
            return result
        finally:
            self._lock.acquire()

    def _robot_state(self, positions: Sequence[float]) -> RobotState:
        state = RobotState(
            is_diff=False,
            joint_state=JointState(
                name=list(self._joint_names), position=list(positions)
            ),
        )
        virtual_pen = self._virtual_pen_attached_object()
        if virtual_pen is not None:
            state.attached_collision_objects.append(virtual_pen)
        return state

    def _compute_shape_start_ik(
        self,
        position,
        seed: tuple[float, ...],
        timeout_sec: float,
        shape_frame_orientation: Quaternion,
    ) -> tuple[tuple[float, ...], float]:
        best: tuple[float, ...] | None = None
        best_yaw = 0.0
        best_cost = float("inf")
        error_codes: list[int] = []
        for yaw_offset in self.get_parameter("shape_tcp_yaw_offsets").value:
            request = GetPositionIK.Request()
            request.ik_request.group_name = "arm"
            request.ik_request.robot_state = self._robot_state(seed)
            request.ik_request.avoid_collisions = True
            request.ik_request.ik_link_name = self._tcp_link_name
            request.ik_request.pose_stamped = PoseStamped(
                pose=self._shape_pose(
                    position, float(yaw_offset), shape_frame_orientation
                )
            )
            request.ik_request.pose_stamped.header.frame_id = self._tcp_base_frame
            request.ik_request.timeout.sec = int(min(timeout_sec, 5.0))
            response = self._call_service_while_state_unlocked(
                self._ik_client, request, "shape IK", timeout_sec
            )
            if response.error_code.val != MoveItErrorCodes.SUCCESS:
                error_codes.append(int(response.error_code.val))
                continue
            lookup = dict(zip(
                response.solution.joint_state.name,
                response.solution.joint_state.position,
            ))
            try:
                candidate = tuple(float(lookup[name]) for name in self._joint_names)
            except KeyError:
                continue
            cost = sum(abs(value - start) for value, start in zip(candidate, seed))
            if cost < best_cost:
                best, best_yaw, best_cost = candidate, float(yaw_offset), cost
        if best is None:
            point_text = ", ".join(f"{float(value):.3f}" for value in position)
            codes = sorted(set(error_codes))
            raise RuntimeError(
                "MoveIt could not find IK for the first pen-up TCP point "
                f"[{point_text}] m (error codes {codes}); check drawing pose, "
                "pen length, lift distance, and TCP orientation"
            )
        return best, best_yaw

    def _preflight_ik(
        self,
        position,
        seed: tuple[float, ...],
        yaw_offset: float,
        shape_frame_orientation: Quaternion,
        avoid_collisions: bool,
        ik_timeout: float,
    ):
        request = GetPositionIK.Request()
        request.ik_request.group_name = "arm"
        request.ik_request.robot_state = self._robot_state(seed)
        request.ik_request.avoid_collisions = avoid_collisions
        request.ik_request.ik_link_name = self._tcp_link_name
        request.ik_request.pose_stamped = PoseStamped(
            pose=self._shape_pose(position, yaw_offset, shape_frame_orientation)
        )
        request.ik_request.pose_stamped.header.frame_id = self._tcp_base_frame
        request.ik_request.timeout.sec = int(ik_timeout)
        request.ik_request.timeout.nanosec = int(
            (ik_timeout - int(ik_timeout)) * 1_000_000_000
        )
        response = self._call_service_while_state_unlocked(
            self._ik_client,
            request,
            "shape reachability IK",
            max(1.0, ik_timeout * 4.0),
        )
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            return None, int(response.error_code.val)
        lookup = dict(zip(
            response.solution.joint_state.name,
            response.solution.joint_state.position,
        ))
        try:
            candidate = tuple(float(lookup[name]) for name in self._joint_names)
        except KeyError:
            return None, int(response.error_code.val)
        return candidate, int(response.error_code.val)

    def _preflight_yaw(
        self, first_position, seed, orientation, ik_timeout, *, collisions=True
    ):
        best = None
        best_yaw = 0.0
        best_cost = float("inf")
        for raw_yaw in self.get_parameter("shape_tcp_yaw_offsets").value:
            yaw = float(raw_yaw)
            candidate, _ = self._preflight_ik(
                first_position, seed, yaw, orientation, collisions, ik_timeout
            )
            if candidate is None:
                continue
            cost = sum(abs(value - start) for value, start in zip(candidate, seed))
            if cost < best_cost:
                best, best_yaw, best_cost = candidate, yaw, cost
        return best, best_yaw

    def _preflight_samples(
        self,
        world_points,
        seed: tuple[float, ...],
        orientation: Quaternion,
        ik_timeout: float,
        warning_margin: float,
    ):
        first_solution, yaw_offset = self._preflight_yaw(
            world_points[0], seed, orientation, ik_timeout, collisions=True
        )
        if first_solution is None:
            first_solution, yaw_offset = self._preflight_yaw(
                world_points[0], seed, orientation, ik_timeout, collisions=False
            )
        current_seed = seed if first_solution is None else first_solution
        statuses = []
        joint_rows = []
        for point in world_points:
            candidate, _ = self._preflight_ik(
                point, current_seed, yaw_offset, orientation, True, ik_timeout
            )
            if candidate is not None:
                margin = min(
                    min(value - lower, upper - value)
                    for value, lower, upper in zip(
                        candidate, self._lower_limits, self._upper_limits
                    )
                )
                status = (
                    PREFLIGHT_NEAR_LIMIT
                    if margin <= warning_margin else PREFLIGHT_REACHABLE
                )
                current_seed = candidate
                joint_rows.append(candidate)
            else:
                geometric, _ = self._preflight_ik(
                    point, current_seed, yaw_offset, orientation, False, ik_timeout
                )
                if geometric is None:
                    status = PREFLIGHT_NO_IK
                    joint_rows.append(None)
                else:
                    status = PREFLIGHT_COLLISION
                    current_seed = geometric
                    joint_rows.append(geometric)
            statuses.append(status)
        return tuple(statuses), tuple(joint_rows), yaw_offset

    @staticmethod
    def _coarse_points(points, maximum=10):
        if len(points) <= maximum:
            return tuple(points)
        return tuple(
            points[round(index * (len(points) - 1) / (maximum - 1))]
            for index in range(maximum)
        )

    def _preflight_candidate_reachable(
        self, world_points, seed, orientation, ik_timeout
    ) -> bool:
        first, yaw_offset = self._preflight_yaw(
            world_points[0], seed, orientation, ik_timeout, collisions=True
        )
        if first is None:
            return False
        current_seed = first
        for point in world_points[1:]:
            candidate, _ = self._preflight_ik(
                point, current_seed, yaw_offset, orientation, True, ik_timeout
            )
            if candidate is None:
                return False
            current_seed = candidate
        return True

    def _suggest_shape_pose(
        self, local_tcp_points, marker_pose, seed, ik_timeout, warning_margin
    ):
        step = float(self.get_parameter("shape_preflight_suggestion_step_m").value)
        steps = int(self.get_parameter("shape_preflight_suggestion_steps").value)
        if not math.isfinite(step) or step <= 0.0 or steps < 1 or steps > 10:
            return None, 0.0
        orientation_values = self._quaternion_values(marker_pose.orientation)
        directions = (
            (0.0, 0.0, 1.0), (0.0, 0.0, -1.0),
            (1.0, 0.0, 0.0), (-1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0), (0.0, -1.0, 0.0),
        )
        local_samples = self._coarse_points(local_tcp_points)
        for step_index in range(1, steps + 1):
            distance = step * step_index
            for direction in directions:
                local_offset = tuple(distance * value for value in direction)
                world_offset = transform_points(
                    (local_offset,), (0.0, 0.0, 0.0), orientation_values
                )[0]
                translation = (
                    marker_pose.position.x + world_offset[0],
                    marker_pose.position.y + world_offset[1],
                    marker_pose.position.z + world_offset[2],
                )
                world_samples = transform_points(
                    local_samples, translation, orientation_values
                )
                if self._preflight_candidate_reachable(
                    world_samples, seed, marker_pose.orientation, ik_timeout
                ):
                    suggested = self._copy_pose(marker_pose)
                    suggested.position.x = translation[0]
                    suggested.position.y = translation[1]
                    suggested.position.z = translation[2]
                    return suggested, distance
        return None, 0.0

    def _check_shape_reachability(self, _request, response):
        """Classify the current pen-tip path without commanding any motion."""
        with self._lock:
            try:
                if self._state == "SHAPE_PLANNING":
                    raise RuntimeError("shape planning is already running")
                if self._shape_preflight_running:
                    raise RuntimeError("shape reachability check is already running")
                self._shape_preflight_running = True
                if self._shape_marker_shape == "freehand":
                    raise RuntimeError("freehand mode has no preset path to check")
                if self._latest_positions is None or (
                    self._now() - self._latest_joint_time > self._feedback_timeout
                ):
                    raise RuntimeError("fresh robot joint feedback is required")
                signature = self._shape_signature()
                local_strokes = self._local_drawing_strokes()
                raw_tcp_path = pen_tcp_path(
                    local_strokes,
                    self._pen_tcp_to_tip_m(),
                    self._shape_pen_lift_m,
                )
                maximum_points = int(
                    self.get_parameter("shape_preflight_maximum_points").value
                )
                local_tcp_points = sample_cartesian_path(
                    raw_tcp_path,
                    float(self.get_parameter("shape_cartesian_step_m").value),
                    maximum_points,
                )
                marker_pose = self._copy_pose(self._shape_marker_pose)
                translation = (
                    marker_pose.position.x,
                    marker_pose.position.y,
                    marker_pose.position.z,
                )
                orientation_values = self._quaternion_values(marker_pose.orientation)
                world_points = transform_points(
                    local_tcp_points, translation, orientation_values
                )
                ik_timeout = float(
                    self.get_parameter("shape_preflight_ik_timeout_sec").value
                )
                warning_margin = float(self.get_parameter(
                    "shape_preflight_joint_warning_margin_rad"
                ).value)
                if (
                    not math.isfinite(ik_timeout) or ik_timeout <= 0.0 or
                    not math.isfinite(warning_margin) or warning_margin < 0.0
                ):
                    raise ValueError("invalid shape preflight parameters")
                statuses, _, yaw_offset = self._preflight_samples(
                    world_points, self._latest_positions, marker_pose.orientation,
                    ik_timeout, warning_margin,
                )
                if signature != self._shape_signature():
                    raise RuntimeError("shape changed during reachability check; run it again")
                counts = {
                    status: statuses.count(status) for status in PREFLIGHT_PRIORITY
                }
                invalid = counts[PREFLIGHT_COLLISION] + counts[PREFLIGHT_NO_IK]
                suggested_pose = None
                suggestion_distance = 0.0
                if invalid:
                    suggested_pose, suggestion_distance = self._suggest_shape_pose(
                        local_tcp_points, marker_pose, self._latest_positions,
                        ik_timeout, warning_margin,
                    )
                tcp_to_tip = self._pen_tcp_to_tip_m()
                local_tip_points = tuple(
                    (point[0], point[1], point[2] - tcp_to_tip)
                    for point in local_tcp_points
                )
                self._shape_preflight = {
                    "signature": signature,
                    "local_tip_points": local_tip_points,
                    "statuses": statuses,
                    "counts": counts,
                    "yaw_offset": yaw_offset,
                    "suggested_pose": suggested_pose,
                    "suggestion_distance_m": suggestion_distance,
                }
                self._refresh_shape_marker_locked()
                response.success = True
                response.feasible = invalid == 0
                response.sampled_points = len(statuses)
                response.reachable_points = counts[PREFLIGHT_REACHABLE]
                response.near_limit_points = counts[PREFLIGHT_NEAR_LIMIT]
                response.collision_points = counts[PREFLIGHT_COLLISION]
                response.no_ik_points = counts[PREFLIGHT_NO_IK]
                response.has_suggestion = suggested_pose is not None
                if suggested_pose is not None:
                    response.suggested_pose = suggested_pose
                    response.suggestion_distance_m = suggestion_distance
                response.message = (
                    f"checked {len(statuses)} pen-tip samples: "
                    f"reachable {counts[PREFLIGHT_REACHABLE]}, "
                    f"near limit {counts[PREFLIGHT_NEAR_LIMIT]}, "
                    f"collision {counts[PREFLIGHT_COLLISION]}, "
                    f"no IK {counts[PREFLIGHT_NO_IK]}"
                )
                self._message = response.message
            except Exception as error:
                response.success = False
                response.message = str(error)
                self._message = response.message
            finally:
                self._shape_preflight_running = False
            self._publish_status()
            return response

    def _plan_joint_segment(
        self,
        start: Sequence[float],
        goal: Sequence[float],
        timeout_sec: float,
        *,
        label: str = "shape approach planning",
        planning_time_sec: float | None = None,
        velocity_scaling_factor: float = 0.1,
    ) -> JointTrajectory:
        scaling = float(velocity_scaling_factor)
        if not 0.0 < scaling <= 1.0:
            raise ValueError("planning velocity scaling factor must be in (0, 1]")
        request = GetMotionPlan.Request()
        motion = request.motion_plan_request
        motion.group_name = "arm"
        motion.pipeline_id = "ompl"
        motion.planner_id = "RRTConnect"
        motion.allowed_planning_time = float(
            self.get_parameter("shape_planning_time_sec").value
            if planning_time_sec is None
            else planning_time_sec
        )
        motion.num_planning_attempts = 5
        motion.max_velocity_scaling_factor = scaling
        motion.max_acceleration_scaling_factor = scaling
        motion.start_state = self._robot_state(start)
        motion.goal_constraints = [Constraints(
            joint_constraints=[
                JointConstraint(
                    joint_name=name,
                    position=float(value),
                    tolerance_above=0.01,
                    tolerance_below=0.01,
                    weight=1.0,
                )
                for name, value in zip(self._joint_names, goal)
            ]
        )]
        response = self._call_service_while_state_unlocked(
            self._motion_plan_client, request, label, timeout_sec
        ).motion_plan_response
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                f"MoveIt {label} failed: {response.error_code.val} "
                f"{response.error_code.message or 'planner returned no detailed message'}. "
                "Collision check rejected the path; possible causes are a forbidden-zone "
                "collision, start/goal already in collision, joint limits, or no IK solution. "
                "Inspect the red forbidden-zone geometry and the start/goal states in RViz."
            )
        return response.trajectory.joint_trajectory

    def _plan_shape_outline(
        self, start: Sequence[float], poses: Sequence[Pose], timeout_sec: float
    ) -> JointTrajectory:
        request = GetCartesianPath.Request()
        request.header.frame_id = self._tcp_base_frame
        request.start_state = self._robot_state(start)
        request.group_name = "arm"
        request.link_name = self._tcp_link_name
        request.waypoints = list(poses)
        request.max_step = float(self.get_parameter("shape_cartesian_step_m").value)
        request.jump_threshold = float(
            self.get_parameter("shape_jump_threshold").value
        )
        request.avoid_collisions = True
        request.max_velocity_scaling_factor = 0.1
        request.max_acceleration_scaling_factor = 0.1
        response = self._call_service_while_state_unlocked(
            self._cartesian_path_client, request, "Cartesian shape planning", timeout_sec
        )
        if response.error_code.val != MoveItErrorCodes.SUCCESS or response.fraction < 0.999:
            raise RuntimeError(
                f"MoveIt completed only {response.fraction * 100.0:.1f}% of the "
                f"collision-free shape path (code {response.error_code.val}). "
                "The rejected segment may intersect a forbidden zone, exceed a joint limit, "
                "or have no IK solution; inspect the red collision segment in RViz."
            )
        return response.solution.joint_trajectory

    def _trajectory_rows(self, trajectory: JointTrajectory) -> list[tuple[float, ...]]:
        lookup = {name: index for index, name in enumerate(trajectory.joint_names)}
        if any(name not in lookup for name in self._joint_names):
            raise RuntimeError("MoveIt shape path omitted an arm joint")
        return [
            tuple(float(point.positions[lookup[name]]) for name in self._joint_names)
            for point in trajectory.points
        ]

    def _recorded_from_joint_trajectory(
        self, trajectory: JointTrajectory, label: str
    ) -> RecordedTrajectory:
        lookup = {name: index for index, name in enumerate(trajectory.joint_names)}
        if any(name not in lookup for name in self._joint_names):
            raise RuntimeError(f"MoveIt {label} omitted an arm joint")
        if len(trajectory.points) < 2:
            raise RuntimeError(f"MoveIt {label} returned fewer than two points")
        raw_times = [
            float(point.time_from_start.sec)
            + float(point.time_from_start.nanosec) / 1_000_000_000.0
            for point in trajectory.points
        ]
        first_time = raw_times[0]
        points = tuple(
            RecordedPoint(
                raw_time - first_time,
                tuple(float(point.positions[lookup[name]]) for name in self._joint_names),
            )
            for point, raw_time in zip(trajectory.points, raw_times)
        )
        if not all(
            math.isfinite(point.time_from_start)
            and point.time_from_start >= 0.0
            and all(math.isfinite(value) for value in point.positions)
            for point in points
        ):
            raise RuntimeError(f"MoveIt {label} returned non-finite trajectory data")
        if any(
            second.time_from_start < first.time_from_start
            for first, second in zip(points, points[1:])
        ):
            raise RuntimeError(f"MoveIt {label} returned non-monotonic timing")
        result = RecordedTrajectory(
            joint_names=self._joint_names,
            points=points,
            created_utc="",
        )
        self._validate_trajectory(result)
        return result

    def _plan_sequence_transitions(
        self,
        action_names: Sequence[str],
        trajectories: Sequence[RecordedTrajectory],
    ) -> list[RecordedTrajectory | None]:
        """Pre-plan every disconnected action boundary before hardware moves."""
        if len(action_names) != len(trajectories):
            raise ValueError("action names and trajectories must have equal length")
        tolerance = float(
            self.get_parameter("sequence_transition_direct_tolerance_rad").value
        )
        planning_time = float(
            self.get_parameter("sequence_transition_planning_time_sec").value
        )
        timeout = float(
            self.get_parameter("sequence_transition_service_timeout_sec").value
        )
        scaling = float(
            self.get_parameter("sequence_transition_velocity_scaling_factor").value
        )
        transitions: list[RecordedTrajectory | None] = []
        for index, (first, second) in enumerate(
            zip(trajectories, trajectories[1:])
        ):
            if self._cancel_requested:
                raise RuntimeError("action sequence planning was canceled")
            start = first.points[-1].positions
            goal = second.points[0].positions
            if max(abs(a - b) for a, b in zip(start, goal)) <= tolerance:
                transitions.append(None)
                continue
            boundary = f"{action_names[index]!r} -> {action_names[index + 1]!r}"
            self._message = (
                f"planning collision-checked sequence transition {boundary}; "
                "hardware unchanged"
            )
            self._publish_status()
            planned = self._plan_joint_segment(
                start,
                goal,
                timeout,
                label=f"sequence transition {boundary}",
                planning_time_sec=planning_time,
                velocity_scaling_factor=scaling,
            )
            if self._cancel_requested:
                raise RuntimeError("action sequence planning was canceled")
            transition = self._recorded_from_joint_trajectory(
                planned, f"sequence transition {boundary}"
            )
            if max(
                abs(a - b)
                for a, b in zip(start, transition.points[0].positions)
            ) > tolerance:
                raise RuntimeError(
                    f"MoveIt sequence transition {boundary} does not start at "
                    "the preceding action endpoint"
                )
            if max(
                abs(a - b)
                for a, b in zip(transition.points[-1].positions, goal)
            ) > tolerance:
                raise RuntimeError(
                    f"MoveIt sequence transition {boundary} does not reach "
                    "the following action start"
                )
            transitions.append(transition)
        return transitions

    def _plan_sequence_entry(
        self,
        action_name: str,
        trajectory: RecordedTrajectory,
        current: Sequence[float],
    ) -> RecordedTrajectory | None:
        """Plan a collision-checked path from the live pose to a sequence start."""
        tolerance = float(
            self.get_parameter("sequence_transition_direct_tolerance_rad").value
        )
        goal = trajectory.points[0].positions
        start = tuple(float(value) for value in current)
        if max(abs(a - b) for a, b in zip(start, goal)) <= tolerance:
            return None
        planning_time = float(
            self.get_parameter("sequence_transition_planning_time_sec").value
        )
        timeout = float(
            self.get_parameter("sequence_transition_service_timeout_sec").value
        )
        scaling = float(
            self.get_parameter("sequence_transition_velocity_scaling_factor").value
        )
        boundary = f"current pose -> {action_name!r}"
        self._message = (
            f"planning collision-checked sequence entry {boundary}; "
            "hardware unchanged"
        )
        self._publish_status()
        planned = self._plan_joint_segment(
            start,
            goal,
            timeout,
            label=f"sequence entry {boundary}",
            planning_time_sec=planning_time,
            velocity_scaling_factor=scaling,
        )
        entry = self._recorded_from_joint_trajectory(
            planned, f"sequence entry {boundary}"
        )
        if max(
            abs(a - b) for a, b in zip(start, entry.points[0].positions)
        ) > tolerance:
            raise RuntimeError(
                f"MoveIt sequence entry {boundary} does not start at the live pose"
            )
        if max(
            abs(a - b) for a, b in zip(entry.points[-1].positions, goal)
        ) > tolerance:
            raise RuntimeError(
                f"MoveIt sequence entry {boundary} does not reach the first action"
            )
        return entry

    def _plan_sequence_entry_guarded(
        self,
        action_name: str,
        trajectory: RecordedTrajectory,
        current: Sequence[float],
    ) -> RecordedTrajectory | None:
        """Expose first-action planning as a cancellable, hardware-idle state."""
        previous_state = self._state
        self._cancel_requested = False
        self._state = "SEQUENCE_PLANNING"
        try:
            entry = self._plan_sequence_entry(action_name, trajectory, current)
            if self._cancel_requested:
                raise RuntimeError("action sequence planning was canceled")
            return entry
        except Exception as error:
            if self._cancel_requested:
                self._cancel_requested = False
                raise RuntimeError(
                    "action sequence planning was canceled"
                ) from error
            raise
        finally:
            if self._state == "SEQUENCE_PLANNING":
                self._state = previous_state

    def _plan_sequence_transitions_guarded(
        self,
        action_names: Sequence[str],
        trajectories: Sequence[RecordedTrajectory],
    ) -> list[RecordedTrajectory | None]:
        """Expose transition planning as a cancellable, hardware-idle state."""
        previous_state = self._state
        self._cancel_requested = False
        self._state = "SEQUENCE_PLANNING"
        try:
            transitions = self._plan_sequence_transitions(
                action_names, trajectories
            )
            if self._cancel_requested:
                raise RuntimeError("action sequence planning was canceled")
            return transitions
        except Exception as error:
            if self._cancel_requested:
                self._cancel_requested = False
                raise RuntimeError(
                    "action sequence planning was canceled"
                ) from error
            raise
        finally:
            if self._state == "SEQUENCE_PLANNING":
                self._state = previous_state

    def _create_shape_action(self, request, response):
        """Plan a collision-checked drawing path and store it as a normal teach action."""
        with self._lock:
            previous_state = self._state
            try:
                if self._state not in {"LOCKED", "IDLE", "READY"}:
                    raise RuntimeError(f"cannot create a shape from state {self._state}")
                shape = str(request.shape).strip().lower()
                if shape not in SUPPORTED_DRAWINGS:
                    raise ValueError(
                        f"unsupported drawing {shape!r}; expected {', '.join(SUPPORTED_DRAWINGS)}"
                    )
                source = str(request.source).strip()
                if self._latest_positions is None:
                    raise RuntimeError("fresh robot joint feedback is required")
                if self._now() - self._latest_joint_time > self._feedback_timeout:
                    raise RuntimeError("robot joint feedback is stale")
                if (
                    self._allow_hardware
                    and self._require_xbox_locked
                    and self._xbox_armed is not False
                ):
                    raise RuntimeError("Xbox must remain LOCKED while planning a shape")
                current = self._latest_positions
                local_strokes = self._local_drawing_strokes(shape, source)
                self._shape_marker_shape = shape
                self._shape_marker_source = source
                self._refresh_shape_marker_locked()
                self._publish_drawing_path(local_strokes)
                marker_pose = self._copy_pose(self._shape_marker_pose)
                self._state = "SHAPE_PLANNING"
                self._message = (
                    f"planning built-in {shape} action with MoveIt; hardware unchanged"
                )
                self._publish_status()
                timeout = float(self.get_parameter("shape_service_timeout_sec").value)
                local_tcp_path = pen_tcp_path(
                    local_strokes,
                    self._pen_tcp_to_tip_m(),
                    self._shape_pen_lift_m,
                )
                tcp_path = transform_points(
                    local_tcp_path,
                    (
                        marker_pose.position.x,
                        marker_pose.position.y,
                        marker_pose.position.z,
                    ),
                    self._quaternion_values(marker_pose.orientation),
                )
                first_point = tcp_path[0]
                first_joints, yaw_offset = self._compute_shape_start_ik(
                    first_point, current, timeout, marker_pose.orientation
                )
                approach = self._plan_joint_segment(current, first_joints, timeout)
                poses = [
                    self._shape_pose(point, yaw_offset, marker_pose.orientation)
                    for point in tcp_path[1:]
                ]
                drawing = self._plan_shape_outline(first_joints, poses, timeout)
                drawing_rows = self._trajectory_rows(drawing)
                if not drawing_rows:
                    raise RuntimeError("MoveIt returned an empty Cartesian shape path")
                return_path = self._plan_joint_segment(drawing_rows[-1], current, timeout)
                rows = (
                    self._trajectory_rows(approach)
                    + drawing_rows
                    + self._trajectory_rows(return_path)
                )
                if self._latest_positions is None or max(
                    abs(value - start)
                    for value, start in zip(self._latest_positions, current)
                ) > float(self.get_parameter("shape_maximum_start_drift_rad").value):
                    raise RuntimeError(
                        "robot moved while the shape was being planned; generate it again"
                    )
                if (
                    self._allow_hardware
                    and self._require_xbox_locked
                    and self._xbox_armed is not False
                ):
                    raise RuntimeError("Xbox left LOCKED while planning the shape")
                trajectory = retime_joint_path(
                    self._joint_names,
                    rows,
                    float(self.get_parameter(
                        "shape_maximum_joint_velocity_rad_s"
                    ).value),
                )
                self._validate_trajectory(trajectory)
                if shape == "text":
                    safe_source = source.replace("/", "_").replace("\\", "_")[:32]
                    name = f"字符包_{safe_source}"
                    description = (
                        f"单线字符绘图动作：{source}；含 {len(local_strokes)} 笔；"
                        "由 MoveIt 在当前姿态生成"
                    )
                elif shape == "image":
                    name = f"简笔画_{Path(source).stem[:36]}"
                    description = (
                        f"图片简笔画动作：{Path(source).name}；含 {len(local_strokes)} 条轮廓；"
                        "由 MoveIt 在当前姿态生成"
                    )
                else:
                    name = f"形状包_{SHAPE_NAMES_ZH[shape]}"
                    description = (
                        f"内置绘图形状动作：{SHAPE_NAMES_ZH[shape]}；由 MoveIt 在当前姿态生成"
                    )
                target_name = str(request.target_name).strip()
                if target_name:
                    name = validate_action_name(target_name)
                shape_parameters = {
                    "kind": "shape",
                    "shape": shape,
                    "source": source,
                    "pose": {
                        "position": [
                            marker_pose.position.x,
                            marker_pose.position.y,
                            marker_pose.position.z,
                        ],
                        "orientation": list(
                            self._quaternion_values(marker_pose.orientation)
                        ),
                    },
                    "width": self._shape_marker_width,
                    "height": self._shape_marker_height,
                    "pen_length_m": self._shape_pen_length_m,
                    "pen_lift_m": self._shape_pen_lift_m,
                }
                info = self._action_library.save(
                    name,
                    description,
                    trajectory,
                    overwrite=bool(request.overwrite),
                    shape_parameters=shape_parameters,
                )
                self._trajectory = trajectory
                self._selected_action_name = info.name
                self._state = "READY" if self._allow_hardware else "LOCKED"
                self._publish_recorded_preview(trajectory)
                response.success = True
                response.message = (
                    f"created shape action {info.name!r} with "
                    f"{len(trajectory.points)} collision-checked points; hardware unchanged"
                )
                response.name = info.name
                response.path = str(info.path)
                self._message = response.message
            except Exception as error:
                if self._state == "SHAPE_PLANNING":
                    self._state = previous_state
                response.success = False
                response.message = str(error)
                response.name = ""
                response.path = ""
                self._message = response.message
            self._publish_status()
            return response

    def _preview_action_group(self, request, response):
        """Preview a complete action or seek its RViz-only animation loop."""
        with self._lock:
            try:
                if self._state not in {"LOCKED", "IDLE", "READY"}:
                    raise RuntimeError(
                        f"cannot preview from state {self._state}"
                    )
                info, trajectory = self._action_library.load(request.name)
                self._validate_trajectory(trajectory)
                self._trajectory = trajectory
                self._selected_action_name = info.name
                if self._allow_hardware:
                    self._state = "READY"
                progress = float(request.progress)
                if progress < 0.0:
                    self._publish_recorded_preview(trajectory)
                    detail = "complete animation"
                else:
                    self._publish_recorded_preview(
                        trajectory, start_progress=progress
                    )
                    detail = (
                        f"animation continued from {progress * 100.0:.1f}%"
                    )
                self._message = (
                    f"previewed action group {info.name!r}: {detail}; hardware unchanged"
                )
                response.success = True
                response.message = self._message
            except Exception as error:
                response.success = False
                response.message = str(error)
                self._message = response.message
            self._publish_status()
            return response

    def _preview_action_sequence(self, request, response):
        """Preview every action in a named sequence without commanding hardware."""
        with self._lock:
            try:
                if self._state not in {"LOCKED", "IDLE", "READY"}:
                    raise RuntimeError(
                        f"cannot preview an action sequence from state {self._state}"
                    )
                requested_names = [
                    str(item) for item in getattr(request, "action_names", ())
                ]
                if requested_names:
                    action_names = tuple(
                        validate_action_name(item) for item in requested_names
                    )
                    sequence_name = str(request.name).strip() or "current editor"
                    display_indices = [int(item) for item in request.display_indices]
                    if display_indices and len(display_indices) != len(action_names):
                        raise ValueError(
                            "display_indices must match the requested action count"
                        )
                    if not display_indices:
                        display_indices = list(range(1, len(action_names) + 1))
                else:
                    sequence = self._action_library.load_sequence(request.name)
                    action_names = sequence.action_names
                    sequence_name = sequence.name
                    display_indices = list(range(1, len(action_names) + 1))
                trajectories = []
                infos = []
                for action_name in action_names:
                    info, trajectory = self._action_library.load(action_name)
                    self._validate_trajectory(trajectory)
                    infos.append(info)
                    trajectories.append(trajectory)
                if not trajectories:
                    raise ValueError("at least one action is required for preview")
                self._hide_shape_visualization_locked()
                self._trajectory = trajectories[0]
                self._selected_action_name = infos[0].name
                if self._allow_hardware:
                    self._state = "READY"
                progress = float(request.progress)
                if bool(request.overlay):
                    self._publish_sequence_overlay(
                        infos, trajectories, display_indices
                    )
                    detail = f"overlay of {len(trajectories)} selected actions"
                else:
                    cache_key = (tuple(action_names), tuple(trajectories))
                    combined = (
                        self._sequence_preview_cache
                        if progress >= 0.0
                        and self._sequence_preview_cache_key == cache_key
                        else None
                    )
                    if combined is None:
                        transitions = self._plan_sequence_transitions_guarded(
                            action_names, trajectories
                        )
                        combined = concatenate_trajectories_with_transitions(
                            trajectories,
                            transitions,
                            endpoint_tolerance=float(
                                self.get_parameter("endpoint_tolerance_rad").value
                            ),
                            maximum_joint_velocity=float(
                                self.get_parameter(
                                    "maximum_joint_velocity_rad_s"
                                ).value
                            ),
                        )
                        self._sequence_preview_cache_key = cache_key
                        self._sequence_preview_cache = combined
                    self._publish_recorded_preview(
                        combined,
                        start_progress=0.0 if progress < 0.0 else progress,
                    )
                    detail = (
                        "complete animation"
                        if progress < 0.0
                        else f"animation continued from {progress * 100.0:.1f}%"
                    )
                self._preview_sequence_name = sequence_name
                self._message = (
                    f"previewing action sequence {sequence_name!r}: {detail}; "
                    "hardware unchanged"
                )
                response.success = True
                response.message = self._message
            except Exception as error:
                response.success = False
                response.message = str(error)
                self._message = response.message
            self._publish_status()
            return response

    def _validate_trajectory(self, trajectory: RecordedTrajectory) -> None:
        if trajectory.joint_names != self._joint_names:
            raise ValueError("trajectory joint names do not match configured arm")
        if not trajectory.timing_valid:
            raise ValueError("trajectory timing is invalid and cannot be replayed")
        if not trajectory.joint_limits_valid:
            raise ValueError("trajectory contains positions outside configured joint limits")
        validate_joint_limits(
            trajectory,
            self._lower_limits,
            self._upper_limits,
            self._joint_limit_margin,
        )

    def _replay(self, _request, response):
        with self._lock:
            try:
                self._clear_sequence_locked()
                self._begin_replay_locked()
                response.success = True
                response.message = self._message
            except Exception as error:
                response.success = False
                response.message = str(error)
                self._message = response.message
            self._publish_status()
            return response

    def _replay_action_group(self, request, response):
        with self._lock:
            try:
                if self._state not in {"IDLE", "READY"}:
                    raise RuntimeError(f"cannot replay from state {self._state}")
                info, trajectory = self._action_library.load(request.name)
                self._validate_trajectory(trajectory)
                self._trajectory = trajectory
                self._selected_action_name = info.name
                self._clear_sequence_locked()
                self._begin_replay_locked(float(request.speed_scale))
                response.success = True
                response.message = self._message
            except Exception as error:
                response.success = False
                response.message = str(error)
                self._message = response.message
            self._publish_status()
            return response

    def _replay_action_sequence(self, request, response):
        with self._lock:
            try:
                if self._state not in {"IDLE", "READY"}:
                    raise RuntimeError(
                        f"cannot replay an action sequence from state {self._state}"
                    )
                requested_names = [str(item) for item in request.action_names]
                if requested_names:
                    action_names = tuple(
                        validate_action_name(item) for item in requested_names
                    )
                    sequence_name = str(request.name).strip() or "current editor"
                else:
                    sequence = self._action_library.load_sequence(request.name)
                    action_names = sequence.action_names
                    sequence_name = sequence.name
                if not action_names:
                    raise ValueError("at least one action is required for replay")
                infos = []
                trajectories = []
                for action_name in action_names:
                    info, trajectory = self._action_library.load(action_name)
                    self._validate_trajectory(trajectory)
                    infos.append(info)
                    trajectories.append(trajectory)
                current = self._preflight(expected_driver_states={"IDLE"})
                entry_transition = self._plan_sequence_entry_guarded(
                    action_names[0], trajectories[0], current
                )
                transitions = self._plan_sequence_transitions_guarded(
                    action_names, trajectories
                )
                self._hide_shape_visualization_locked()
                self._trajectory = trajectories[0]
                self._selected_action_name = infos[0].name
                self._active_sequence_name = sequence_name
                self._sequence_queue = list(action_names[1:])
                self._sequence_transitions = transitions
                self._pending_transition = entry_transition
                self._sequence_action_index = 1
                self._sequence_total_actions = len(action_names)
                try:
                    self._begin_replay_locked(float(request.speed_scale))
                except Exception:
                    self._clear_sequence_locked()
                    raise
                response.success = True
                response.message = self._message
            except Exception as error:
                response.success = False
                response.message = str(error)
                self._message = response.message
            self._publish_status()
            return response

    def _clear_sequence_locked(self) -> None:
        self._active_sequence_name = ""
        self._sequence_queue = []
        self._sequence_transitions = []
        self._pending_transition = None
        self._sequence_action_index = 0
        self._sequence_total_actions = 0

    def _resolve_replay_speed_scale(self, requested: float) -> float:
        speed_scale = float(requested)
        if not math.isfinite(speed_scale):
            raise ValueError("replay speed scale must be finite")
        if speed_scale == 0.0:
            speed_scale = float(self.get_parameter("replay_speed_scale").value)
        if not 0.1 <= speed_scale <= 2.0:
            raise ValueError("replay speed scale must be between 0.1x and 2.0x")
        return speed_scale

    def _begin_replay_locked(self, speed_scale: float | None = None) -> None:
        if self._state not in {"IDLE", "READY"}:
            raise RuntimeError(f"cannot replay from state {self._state}")
        if speed_scale is not None:
            self._active_replay_speed_scale = self._resolve_replay_speed_scale(
                speed_scale
            )
        elif not self._active_sequence_name:
            self._active_replay_speed_scale = self._resolve_replay_speed_scale(0.0)
        current = self._preflight(expected_driver_states={"IDLE"})
        if self._trajectory is None:
            raise RuntimeError("no validated teaching trajectory is loaded")
        self._validate_trajectory(self._trajectory)
        transition = self._pending_transition
        is_sequence_boundary = bool(
            self._active_sequence_name and self._sequence_action_index > 1
        )
        plan = build_playback_plan(
            self._trajectory,
            current,
            transition_trajectory=transition,
            endpoint_tolerance=float(
                self.get_parameter("endpoint_tolerance_rad").value
            ),
            speed_scale=self._active_replay_speed_scale,
            maximum_joint_velocity=float(
                self.get_parameter("maximum_joint_velocity_rad_s").value
            ),
            reverse_return=(
                bool(self.get_parameter("reverse_return").value)
                and not is_sequence_boundary
            ),
            start_dwell_sec=float(self.get_parameter("start_dwell_sec").value),
        )
        self._pending_transition = None
        if not self._active_sequence_name:
            self._sequence_action_index = 1
            self._sequence_total_actions = 1
        self._replay_sequence_name = self._active_sequence_name
        self._replay_action_index = self._sequence_action_index
        self._replay_total_actions = self._sequence_total_actions
        self._replay_progress = 0.0
        self._replay_plan_duration_sec = plan.duration
        motion_file = (
            str(self._action_library.path_for(self._selected_action_name))
            if self._selected_action_name else "(unsaved trajectory)"
        )
        self.get_logger().info(
            f"PLAYBACK PLAN motion={motion_file} raw_duration={self._trajectory.duration:.3f}s "
            f"planned_duration={plan.duration:.3f}s raw_samples={len(self._trajectory.points)} "
            f"velocity_limit={float(self.get_parameter('maximum_joint_velocity_rad_s').value):.3f}rad/s")
        self._replay_generation += 1
        self._last_replay_progress_publish = 0.0
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.header.frame_id = motion_file
        goal.trajectory.joint_names = list(self._joint_names)
        for item in plan.points:
            point = JointTrajectoryPoint()
            point.positions = list(item.positions)
            seconds = int(item.time_from_start)
            nanoseconds = int(
                round((item.time_from_start - seconds) * 1_000_000_000)
            )
            if nanoseconds >= 1_000_000_000:
                seconds += 1
                nanoseconds -= 1_000_000_000
            point.time_from_start.sec = seconds
            point.time_from_start.nanosec = nanoseconds
            goal.trajectory.points.append(point)
        # A direct cyan RViz teaching preview must not keep looping over the
        # actual robot while hardware replay is running.
        self._publish_preview_pose(current)
        self._publish_replay_preview(goal.trajectory, current)
        self._cancel_requested = False
        self._state = "REPLAY_STARTING"
        self._message = "sending trajectory goal to driver; please wait"
        self._publish_status()
        try:
            goal_handle = self._send_trajectory_goal_while_state_unlocked(goal)
        except Exception:
            self._state = "READY"
            raise
        self._active_goal = goal_handle
        self._replay_started_time = self._now()
        self._tracking_error_count = 0
        if self._cancel_requested:
            self._state = "CANCELLING"
            goal_handle.cancel_goal_async()
            self._message = "replay cancellation requested while goal was starting"
            goal_handle.get_result_async().add_done_callback(self._trajectory_result)
            self._publish_status()
            return
        self._state = "REPLAYING"
        return_mode = (
            "collision-checked transition then forward"
            if transition is not None
            else (
                "reverse-to-start then forward"
                if plan.includes_reverse_return
                else "forward"
            )
        )
        group = self._selected_action_name or "draft"
        sequence_detail = (
            f" in sequence {self._active_sequence_name!r}"
            if self._active_sequence_name
            else ""
        )
        self._message = (
            f"replaying {group!r}{sequence_detail} ({return_mode}), "
            f"speed={self._active_replay_speed_scale:.1f}x, "
            f"duration={plan.duration:.2f}s"
        )
        goal_handle.get_result_async().add_done_callback(self._trajectory_result)

    def _publish_replay_preview(
        self,
        trajectory: JointTrajectory,
        start_positions: tuple[float, ...],
        *,
        update_tcp_trace: bool = True,
    ) -> None:
        message = DisplayTrajectory()
        message.trajectory_start = RobotState(
            is_diff=False,
            joint_state=JointState(
                name=list(self._joint_names),
                position=list(start_positions),
            ),
        )
        message.trajectory = [RobotTrajectory(joint_trajectory=trajectory)]
        self._display_trajectory_publisher.publish(message)
        if update_tcp_trace:
            self._set_tcp_trace_from_joint_trajectory(trajectory)
            self._publish_tcp_trace(0.0)
        self.get_logger().info(
            "published action-group path to /display_planned_path for RViz preview"
        )

    def _publish_recorded_preview(
        self,
        trajectory: RecordedTrajectory,
        *,
        start_progress: float = 0.0,
    ) -> None:
        """Publish a timed, seekable teaching loop without commanding hardware."""
        self._preview_sequence_name = ""
        timed_preview = prepare_rviz_preview(
            trajectory,
            frame_period_sec=self._preview_frame_period,
            maximum_idle_sec=self._preview_maximum_idle,
            idle_position_delta=self._preview_idle_delta,
        )
        preview = build_preview_loop(timed_preview, start_progress)
        self._set_tcp_trace_from_recorded(timed_preview)
        self._publish_tcp_trace(start_progress)
        # MotionPlanning's internal player can remain paused while the panel
        # timer advances. Keep its planned-path robot at the current hardware
        # pose and animate RViz's dedicated RobotState display ourselves so the
        # visible model and progress use the same clock.
        hold_positions = self._latest_positions or preview.points[0].positions
        self._publish_static_planned_pose(hold_positions)
        self._rviz_preview_trajectory = timed_preview
        self._preview_started_time = self._now()
        self._preview_paused = False
        self._preview_hold_pending = False
        self._publish_display_robot_state(preview.points[0].positions, highlighted=True)
        self._preview_active = True
        self._preview_generation += 1
        self._preview_progress_offset = float(start_progress)
        self._preview_duration_sec = timed_preview.duration
        self._last_preview_trace_publish_time = 0.0

    def _publish_static_planned_pose(self, positions: tuple[float, ...]) -> None:
        """Interrupt MotionPlanning's player without owning preview timing."""
        joint_trajectory = JointTrajectory()
        joint_trajectory.joint_names = list(self._joint_names)
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        point.time_from_start.nanosec = 50_000_000
        joint_trajectory.points.append(point)
        self._publish_replay_preview(
            joint_trajectory, positions, update_tcp_trace=False
        )

    def _set_tcp_trace_from_recorded(self, trajectory: RecordedTrajectory) -> None:
        self._set_tcp_trace(
            [point.positions for point in trajectory.points],
            [point.time_from_start for point in trajectory.points],
        )

    def _tcp_points_from_recorded(
        self,
        trajectory: RecordedTrajectory,
        tcp_to_tip_m: float | None = None,
    ) -> list[tuple[float, float, float]]:
        if self._tcp_fk is None:
            return []
        tip_offset = self._pen_tcp_to_tip_m() if tcp_to_tip_m is None else tcp_to_tip_m
        return [
            self._tcp_fk.position(
                dict(zip(self._joint_names, point.positions)),
                local_point=(tip_offset, 0.0, 0.0),
            )
            for point in trajectory.points
        ]

    def _publish_sequence_overlay(
        self, infos, trajectories, display_indices: Sequence[int]
    ) -> None:
        """Show selected sequence paths together with stable numbered labels."""
        self._preview_active = False
        self._preview_paused = False
        self._preview_generation += 1
        self._tcp_trace_points = []
        self._tcp_trace_times = []
        self._tcp_trace_duration = 0.0
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers = [clear]
        palette = (
            (0.10, 0.72, 1.00),
            (1.00, 0.58, 0.08),
            (0.20, 0.85, 0.35),
            (0.78, 0.35, 0.95),
            (1.00, 0.25, 0.38),
            (0.95, 0.82, 0.12),
        )
        for item, (info, trajectory, display_index) in enumerate(
            zip(infos, trajectories, display_indices)
        ):
            stored_pen_length = (
                info.shape_parameters.get("pen_length_m")
                if info.shape_parameters is not None
                else None
            )
            tcp_to_tip_m = (
                self._shape_pen_mount_offset_m + float(stored_pen_length)
                if stored_pen_length is not None
                else self._pen_tcp_to_tip_m()
            )
            points = self._tcp_points_from_recorded(trajectory, tcp_to_tip_m)
            if not points:
                continue
            red, green, blue = palette[item % len(palette)]
            line = Marker()
            line.header.frame_id = self._visualization_base_frame
            line.header.stamp = self.get_clock().now().to_msg()
            line.ns = "rebot_sequence_overlay"
            line.id = item * 2
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.pose.orientation.w = 1.0
            line.scale.x = self._tcp_trace_line_width_m
            line.color.r, line.color.g, line.color.b, line.color.a = (
                red, green, blue, 1.0
            )
            line.points = [Point(x=x, y=y, z=z) for x, y, z in points]
            markers.append(line)

            label = Marker()
            label.header = line.header
            label.ns = line.ns
            label.id = item * 2 + 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = sum(point[0] for point in points) / len(points)
            label.pose.position.y = sum(point[1] for point in points) / len(points)
            label.pose.position.z = max(point[2] for point in points) + 0.035
            label.pose.orientation.w = 1.0
            label.scale.z = 0.026
            label.color.r, label.color.g, label.color.b, label.color.a = (
                red, green, blue, 1.0
            )
            label.text = f"#{display_index}  {info.name}"
            markers.append(label)
        self._tcp_trace_publisher.publish(MarkerArray(markers=markers))
        hold = self._latest_positions or trajectories[0].points[0].positions
        self._publish_static_planned_pose(hold)

    def _set_tcp_trace_from_joint_trajectory(self, trajectory: JointTrajectory) -> None:
        lookup = {name: index for index, name in enumerate(trajectory.joint_names)}
        if self._tcp_fk is None or any(name not in lookup for name in self._joint_names):
            self._tcp_trace_points = []
            self._tcp_trace_times = []
            self._tcp_trace_duration = 0.0
            return
        rows = [
            tuple(float(point.positions[lookup[name]]) for name in self._joint_names)
            for point in trajectory.points
        ]
        times = [
            float(point.time_from_start.sec)
            + float(point.time_from_start.nanosec) / 1_000_000_000.0
            for point in trajectory.points
        ]
        self._set_tcp_trace(rows, times)

    def _set_tcp_trace(
        self, rows: Sequence[Sequence[float]], times: Sequence[float]
    ) -> None:
        if self._tcp_fk is None or len(rows) != len(times):
            self._tcp_trace_points = []
            self._tcp_trace_times = []
            self._tcp_trace_duration = 0.0
            return
        self._tcp_trace_points = [
            self._tcp_fk.position(
                dict(zip(self._joint_names, positions)),
                local_point=(self._pen_tcp_to_tip_m(), 0.0, 0.0),
            )
            for positions in rows
        ]
        self._tcp_trace_times = [float(value) for value in times]
        self._tcp_trace_duration = self._tcp_trace_times[-1] if times else 0.0

    def _trace_marker(self, marker_id: int, points, *, completed: bool) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._visualization_base_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "rebot_tcp_trajectory"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self._tcp_trace_line_width_m
        if completed:
            marker.color.r, marker.color.g, marker.color.b = 0.12, 0.78, 0.35
        else:
            marker.color.r, marker.color.g, marker.color.b = 1.0, 0.58, 0.08
        marker.color.a = 1.0
        marker.points = [Point(x=x, y=y, z=z) for x, y, z in points]
        return marker

    def _virtual_pen_markers(self) -> list[Marker]:
        """Return a pen rigidly attached to the live gripper_tcp TF frame."""
        exposed_length = self._shape_pen_length_m
        mount_offset = self._shape_pen_mount_offset_m
        tcp_to_tip = self._pen_tcp_to_tip_m()
        lift = self._shape_pen_lift_m
        root_half = math.sqrt(0.5)

        cylinder = Marker()
        cylinder.header.frame_id = self._visualization_tcp_link_name
        cylinder.header.stamp = self.get_clock().now().to_msg()
        cylinder.ns = "rebot_virtual_pen"
        cylinder.id = 0
        cylinder.type = Marker.CYLINDER
        cylinder.action = Marker.ADD if exposed_length > 1e-6 else Marker.DELETE
        cylinder.pose.position.x = mount_offset + exposed_length * 0.5
        # RViz cylinders use local Z. Rotate Z onto gripper_tcp local +X.
        cylinder.pose.orientation.y = root_half
        cylinder.pose.orientation.w = root_half
        cylinder.scale.x = 0.010
        cylinder.scale.y = 0.010
        cylinder.scale.z = max(exposed_length, 1e-6)
        cylinder.color.r = 0.70
        cylinder.color.g = 0.32
        cylinder.color.b = 0.95
        cylinder.color.a = 1.0

        tcp = Marker()
        tcp.header = cylinder.header
        tcp.ns = cylinder.ns
        tcp.id = 1
        tcp.type = Marker.SPHERE
        tcp.action = Marker.ADD
        tcp.pose.orientation.w = 1.0
        tcp.scale.x = tcp.scale.y = tcp.scale.z = 0.014
        tcp.color.r, tcp.color.g, tcp.color.b, tcp.color.a = 0.82, 0.40, 1.0, 1.0

        tip = Marker()
        tip.header = cylinder.header
        tip.ns = cylinder.ns
        tip.id = 2
        tip.type = Marker.SPHERE
        tip.action = Marker.ADD
        tip.pose.position.x = tcp_to_tip
        tip.pose.orientation.w = 1.0
        tip.scale.x = tip.scale.y = tip.scale.z = 0.010
        tip.color.r, tip.color.g, tip.color.b, tip.color.a = 1.0, 0.86, 0.05, 1.0

        mount = Marker()
        mount.header = cylinder.header
        mount.ns = cylinder.ns
        mount.id = 4
        mount.type = Marker.SPHERE
        mount.action = Marker.ADD
        mount.pose.position.x = mount_offset
        mount.pose.orientation.w = 1.0
        mount.scale.x = mount.scale.y = mount.scale.z = 0.012
        mount.color.r, mount.color.g, mount.color.b, mount.color.a = 0.20, 0.85, 1.0, 1.0

        armward = Marker()
        armward.header = cylinder.header
        armward.ns = cylinder.ns
        armward.id = 3
        armward.type = Marker.ARROW
        armward.action = Marker.ADD
        armward.scale.x, armward.scale.y, armward.scale.z = 0.007, 0.014, 0.020
        armward.color.r, armward.color.g, armward.color.b, armward.color.a = (
            1.0, 0.16, 0.12, 1.0
        )
        armward.points = [
            Point(x=tcp_to_tip, y=0.0, z=0.0),
            Point(x=tcp_to_tip - max(lift, 0.04), y=0.0, z=0.0),
        ]
        return [cylinder, tcp, tip, armward, mount]

    def _publish_tcp_trace(self, progress: float) -> None:
        """Split the TCP trace into completed green and upcoming orange lines."""
        if not hasattr(self, "_tcp_trace_publisher"):
            return
        points = getattr(self, "_tcp_trace_points", [])
        times = getattr(self, "_tcp_trace_times", [])
        normalized = min(1.0, max(0.0, float(progress)))
        self._tcp_trace_progress = normalized
        completed = []
        upcoming = list(points)
        duration = getattr(self, "_tcp_trace_duration", 0.0)
        if points and times and duration > 0.0:
            target = normalized * duration
            if normalized >= 1.0:
                completed, upcoming = list(points), []
            elif normalized > 0.0:
                index = next(
                    (item for item, stamp in enumerate(times) if stamp >= target),
                    len(times) - 1,
                )
                if index == 0:
                    boundary = points[0]
                else:
                    first_time, second_time = times[index - 1], times[index]
                    ratio = 0.0 if second_time <= first_time else (
                        target - first_time
                    ) / (second_time - first_time)
                    boundary = tuple(
                        a + (b - a) * ratio
                        for a, b in zip(points[index - 1], points[index])
                    )
                completed = list(points[:index]) + [boundary]
                upcoming = [boundary] + list(points[index:])
        clear = Marker()
        clear.action = Marker.DELETEALL
        self._tcp_trace_publisher.publish(MarkerArray(markers=[
            clear,
            self._trace_marker(0, completed, completed=True),
            self._trace_marker(1, upcoming, completed=False),
            *self._virtual_pen_markers(),
        ]))

    def _virtual_pen_attached_object(self) -> AttachedCollisionObject | None:
        """Create the same pen for the animated DisplayRobotState preview."""
        exposed_length = self._shape_pen_length_m
        if exposed_length <= 1e-6:
            return None
        mount_offset = self._shape_pen_mount_offset_m
        tcp_to_tip = self._pen_tcp_to_tip_m()
        root_half = math.sqrt(0.5)
        body = SolidPrimitive()
        body.type = SolidPrimitive.CYLINDER
        body.dimensions = [exposed_length, 0.005]
        tip = SolidPrimitive()
        tip.type = SolidPrimitive.SPHERE
        tip.dimensions = [0.005]
        collision = CollisionObject()
        collision.header.frame_id = self._tcp_link_name
        collision.id = "virtual_pen_model"
        collision.operation = CollisionObject.ADD
        collision.primitives = [body, tip]
        collision.primitive_poses = [
            Pose(
                position=Point(x=mount_offset + exposed_length * 0.5),
                orientation=Quaternion(y=root_half, w=root_half),
            ),
            Pose(position=Point(x=tcp_to_tip), orientation=Quaternion(w=1.0)),
        ]
        return AttachedCollisionObject(
            link_name=self._tcp_link_name,
            object=collision,
            touch_links=[
                self._tcp_link_name,
                "gripper_link",
                "gripper_left",
                "gripper_right",
            ],
        )

    def _publish_display_robot_state(
        self, positions: tuple[float, ...], *, highlighted: bool = True
    ) -> None:
        state = RobotState(
            is_diff=False,
            joint_state=JointState(
                name=list(self._joint_names),
                position=list(positions),
            ),
        )
        virtual_pen = self._virtual_pen_attached_object()
        if virtual_pen is not None:
            state.attached_collision_objects.append(virtual_pen)
        message = DisplayRobotState()
        message.state = state
        if highlighted:
            for link_name in self._preview_link_names:
                color = ObjectColor()
                color.id = link_name
                color.color.r = 0.0
                color.color.g = 0.72
                color.color.b = 1.0
                color.color.a = 1.0
                message.highlight_links.append(color)
        if virtual_pen is not None:
            pen_color = ObjectColor()
            pen_color.id = "virtual_pen_model"
            pen_color.color.r = 0.70
            pen_color.color.g = 0.32
            pen_color.color.b = 0.95
            pen_color.color.a = 1.0
            message.highlight_links.append(pen_color)
        self._display_robot_state_publisher.publish(message)

    def _advance_rviz_preview(self) -> None:
        with self._lock:
            trajectory = self._rviz_preview_trajectory
            if (
                not self._preview_active
                or self._preview_paused
                or trajectory is None
                or self._preview_duration_sec <= 0.0
            ):
                return
            now = self._now()
            elapsed = max(0.0, now - self._preview_started_time)
            progress = (
                self._preview_progress_offset
                + elapsed / self._preview_duration_sec
            ) % 1.0
            self._publish_display_robot_state(
                sample_trajectory(trajectory, progress)
            )
            # Keep the RViz-only trace synchronized to the same preview clock.
            # Twenty marker updates per second are smooth while avoiding a
            # potentially large MarkerArray publish on every robot frame.
            last_publish = getattr(self, "_last_preview_trace_publish_time", 0.0)
            if (
                now - last_publish >= 0.05
                or progress < getattr(self, "_tcp_trace_progress", 0.0)
            ):
                self._publish_tcp_trace(progress)
                self._last_preview_trace_publish_time = now

    def _publish_preview_pose(self, positions: tuple[float, ...]) -> None:
        self._publish_static_planned_pose(positions)
        self._publish_display_robot_state(positions, highlighted=False)
        self._preview_active = False
        self._preview_paused = False
        self._preview_sequence_name = ""
        self._preview_progress_offset = 0.0
        self._preview_duration_sec = 0.0
        self._rviz_preview_trajectory = None
        self._preview_started_time = 0.0
        self._preview_hold_pending = False
        self._tcp_trace_points = []
        self._tcp_trace_times = []
        self._tcp_trace_duration = 0.0
        self._publish_tcp_trace(0.0)

    def _current_preview_progress_locked(self) -> float:
        if not self._preview_active or self._preview_duration_sec <= 0.0:
            return self._preview_progress_offset
        if self._preview_paused:
            return self._preview_progress_offset
        elapsed = max(0.0, self._now() - self._preview_started_time)
        return (
            self._preview_progress_offset
            + elapsed / self._preview_duration_sec
        ) % 1.0

    def _pause_preview(self, _request, response):
        with self._lock:
            if not self._preview_active or self._rviz_preview_trajectory is None:
                response.success = False
                response.message = "no active RViz preview to pause"
                return response
            if self._preview_paused:
                self._preview_started_time = self._now()
                self._preview_paused = False
                self._preview_generation += 1
                self._last_preview_trace_publish_time = 0.0
                self._message = "RViz teaching preview resumed; hardware unchanged"
            else:
                self._preview_progress_offset = self._current_preview_progress_locked()
                self._preview_paused = True
                self._preview_started_time = 0.0
                self._preview_generation += 1
                self._publish_display_robot_state(
                    sample_trajectory(
                        self._rviz_preview_trajectory,
                        self._preview_progress_offset,
                    ),
                    highlighted=True,
                )
                self._publish_tcp_trace(self._preview_progress_offset)
                self._message = "RViz teaching preview paused; hardware unchanged"
            response.success = True
            response.message = self._message
            self._publish_status()
            return response

    def _trajectory_feedback(self, feedback_message) -> None:
        feedback = feedback_message.feedback
        desired = list(feedback.desired.positions)
        actual = list(feedback.actual.positions)
        if len(desired) != len(self._joint_names) or len(actual) != len(self._joint_names):
            return
        error = max(abs(a - b) for a, b in zip(desired, actual))
        with self._lock:
            if self._state != "REPLAYING":
                return
            now = self._now()
            desired_duration = feedback.desired.time_from_start
            desired_time = (
                float(desired_duration.sec)
                + float(desired_duration.nanosec) / 1_000_000_000.0
            )
            if desired_time <= 0.0 and self._replay_started_time > 0.0:
                # Older rebotarmcontroller versions published positions but
                # left time_from_start at zero. Keep the UI moving until the
                # controller-provided trajectory clock becomes available.
                desired_time = max(0.0, now - self._replay_started_time)
            if self._replay_plan_duration_sec > 0.0:
                self._replay_progress = max(
                    self._replay_progress,
                    min(
                        1.0,
                        max(0.0, desired_time / self._replay_plan_duration_sec),
                    ),
                )
            self._tracking_error_count = (
                self._tracking_error_count + 1
                if error > self._maximum_tracking_error
                else 0
            )
            if self._tracking_error_count >= self._tracking_error_limit:
                self._enter_fault(
                    f"tracking error {error:.3f} rad exceeded limit",
                    cancel_goal=True,
                )
            if now - self._last_replay_progress_publish >= 0.1:
                self._last_replay_progress_publish = now
                self._publish_tcp_trace(self._replay_progress)
                self._publish_status()

    def _trajectory_result(self, future) -> None:
        with self._lock:
            self._active_goal = None
            try:
                wrapped = future.result()
                result = wrapped.result
                if self._state == "FAULT":
                    self._clear_sequence_locked()
                elif self._cancel_requested:
                    self._state = "READY" if self._trajectory else "IDLE"
                    self._message = "replay canceled"
                    self._clear_sequence_locked()
                elif result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
                    self._replay_progress = 1.0
                    self._publish_tcp_trace(1.0)
                    if self._active_sequence_name and self._sequence_queue:
                        next_name = self._sequence_queue.pop(0)
                        try:
                            if not self._sequence_transitions:
                                raise RuntimeError(
                                    "sequence transition plan is missing"
                                )
                            self._pending_transition = (
                                self._sequence_transitions.pop(0)
                            )
                            info, trajectory = self._action_library.load(next_name)
                            self._validate_trajectory(trajectory)
                            self._trajectory = trajectory
                            self._selected_action_name = info.name
                            self._sequence_action_index += 1
                            self._state = "READY"
                            self._begin_replay_locked()
                            self._publish_status()
                            return
                        except Exception as error:
                            sequence_name = self._active_sequence_name
                            self._clear_sequence_locked()
                            # A disconnected action endpoint is a safe planning
                            # rejection, not a driver fault. Keep the arm in
                            # READY so the operator can select another action.
                            self._state = (
                                "READY" if isinstance(error, ValueError) else "FAULT"
                            )
                            self._message = (
                                f"sequence {sequence_name!r} stopped before "
                                f"{next_name!r}: {error}"
                            )
                    else:
                        sequence_name = self._active_sequence_name
                        self._clear_sequence_locked()
                        self._state = "READY"
                        self._message = (
                            f"action sequence {sequence_name!r} complete"
                            if sequence_name
                            else "replay complete"
                        )
                else:
                    self._clear_sequence_locked()
                    self._state = "FAULT"
                    self._message = (
                        f"replay failed: code={result.error_code} "
                        f"{result.error_string}"
                    )
            except Exception as error:
                self._clear_sequence_locked()
                self._state = "FAULT"
                self._message = f"replay result failed: {error}"
            self._publish_status()

    def _cancel(self, _request, response):
        with self._lock:
            if self._state == "RECORDING":
                self._recorder.discard()
                self._clear_pending_action()
                if self._gravity_stop.service_is_ready():
                    self._gravity_stop.call_async(Trigger.Request())
                self._state = "READY" if self._trajectory else "IDLE"
                self._message = "recording canceled; gravity stop requested"
                response.success = True
            elif self._state == "SEQUENCE_PLANNING":
                self._cancel_requested = True
                self._clear_sequence_locked()
                self._state = (
                    "LOCKED"
                    if not self._allow_hardware
                    else ("READY" if self._trajectory else "IDLE")
                )
                self._message = (
                    "action sequence transition planning canceled; hardware unchanged"
                )
                response.success = True
            elif self._state in {"REPLAYING", "CANCELLING"} and self._active_goal:
                self._clear_sequence_locked()
                self._cancel_requested = True
                self._state = "CANCELLING"
                self._active_goal.cancel_goal_async()
                self._message = "replay cancellation requested"
                response.success = True
                if self._latest_positions is not None:
                    self._publish_preview_pose(self._latest_positions)
            elif self._state in {"REPLAY_STARTING", "CANCELLING"}:
                self._cancel_requested = True
                self._state = "CANCELLING"
                self._message = "replay cancellation queued while goal is starting"
                response.success = True
            elif self._preview_active:
                positions = self._latest_positions
                if positions is None and self._trajectory is not None:
                    positions = self._trajectory.points[0].positions
                if positions is None:
                    response.success = False
                    self._message = "cannot stop RViz preview without a known pose"
                else:
                    self._publish_preview_pose(positions)
                    self._message = "RViz animation stopped; hardware unchanged"
                    response.success = True
            else:
                response.success = False
                self._message = f"nothing to cancel in state {self._state}"
            response.message = self._message
            self._publish_status()
            return response

    def _reload(self, _request, response):
        with self._lock:
            try:
                if self._state in {
                    "STARTING",
                    "RECORDING",
                    "STOPPING",
                    "SEQUENCE_PLANNING",
                    "REPLAYING",
                    "CANCELLING",
                }:
                    raise RuntimeError(f"cannot reload from state {self._state}")
                trajectory = load_trajectory(self._trajectory_path)
                self._validate_trajectory(trajectory)
                self._trajectory = trajectory
                self._selected_action_name = ""
                self._state = "READY" if self._allow_hardware else "LOCKED"
                self._publish_recorded_preview(trajectory)
                self._message = f"loaded {len(trajectory.points)} teaching points"
                response.success = True
                response.message = self._message
            except Exception as error:
                response.success = False
                response.message = str(error)
                self._message = response.message
            self._publish_status()
            return response

    def _reset(self, _request, response):
        with self._lock:
            if self._state != "FAULT":
                response.success = False
                response.message = f"reset is only valid in FAULT, got {self._state}"
                return response
            try:
                self._preflight(expected_driver_states={"IDLE"})
                self._state = "READY" if self._trajectory else "IDLE"
                # A failed result can arrive after the controller has already
                # reported 100% desired-time progress.  Once the operator has
                # explicitly reset the fault, clear that stale progress so the
                # UI cannot relabel the failed run as successfully completed.
                self._replay_progress = 0.0
                self._replay_plan_duration_sec = 0.0
                self._replay_started_time = 0.0
                self._replay_sequence_name = ""
                self._replay_action_index = 0
                self._replay_total_actions = 0
                self._message = "fault reset after fresh safe feedback"
                response.success = True
            except Exception as error:
                response.success = False
                self._message = f"reset rejected: {error}"
            response.message = self._message
            self._publish_status()
            return response

    def _try_load_existing(self) -> None:
        if not self._trajectory_path.is_file():
            return
        try:
            trajectory = load_trajectory(self._trajectory_path)
            self._validate_trajectory(trajectory)
            self._trajectory = trajectory
            self._selected_action_name = ""
            if self._allow_hardware:
                self._state = "READY"
            # Startup recovery makes the last recording available for replay,
            # but must not look like an operator-requested preview.  Publishing
            # one static pose also interrupts a loop left in an RViz instance
            # that outlived a teach-node restart.
            self._publish_preview_pose(trajectory.points[0].positions)
            self._preview_hold_pending = True
            self._message = (
                f"loaded {len(trajectory.points)} teaching points; "
                "RViz preview is stopped"
            )
        except Exception as error:
            # A stale/invalid replay candidate must not lock out a new raw
            # recording. Replay still validates the selected trajectory
            # independently before any hardware command is accepted.
            self._trajectory = None
            self._selected_action_name = ""
            self._state = "IDLE" if self._allow_hardware else "LOCKED"
            self._message = (
                f"existing trajectory unavailable for replay: {error}; "
                "new recording remains available"
            )

    def _enter_fault(
        self,
        message: str,
        *,
        stop_gravity: bool = False,
        cancel_goal: bool = False,
    ) -> None:
        was_recording = self._state == "RECORDING"
        self._state = "FAULT"
        self._message = message
        self._clear_sequence_locked()
        if was_recording:
            self._recorder.discard()
            self._clear_pending_action()
        if stop_gravity:
            if self._gravity_stop.service_is_ready():
                self._gravity_stop.call_async(Trigger.Request())
        if cancel_goal and self._active_goal is not None and not self._cancel_requested:
            self._cancel_requested = True
            self._active_goal.cancel_goal_async()
        self.get_logger().error(f"[TEACH MODE] FAULT: {message}")
        self._publish_status()

    def _watchdog(self) -> None:
        with self._lock:
            now = self._now()
            if now < self._watchdog_suppressed_until:
                return
            if self._state == "RECORDING" and self._robot_model == "piperh" and (
                self._latest_teach_positions is None
                or now - self._latest_teach_arrival_time > self._feedback_timeout
            ):
                self._recording_timed_out = True
                self._enter_fault(
                    "normal Piper-H grouped feedback stale while recording",
                    stop_gravity=self._teach_drag_mode == "gravity_compensation",
                )
            elif self._state == "RECORDING" and self._robot_model == "piperh" and (
                self._latest_arm_status is None
                or now - self._latest_status_time > self._status_timeout
                or (
                    self._teach_drag_mode == "passive_disabled"
                    and (self._latest_arm_status.state_machine != "DISABLED"
                         or self._latest_arm_status.enabled)
                )
                or (
                    self._teach_drag_mode == "gravity_compensation"
                    and (self._latest_arm_status.state_machine != "GRAVITY_COMPENSATION_ACTIVE"
                         or not self._latest_arm_status.enabled)
                )
            ):
                self._enter_fault(
                    f"Piper-H left {self._teach_drag_mode} recording mode",
                    stop_gravity=self._teach_drag_mode == "gravity_compensation",
                )
            elif self._state == "RECORDING" and (
                self._latest_positions is None
                or now - self._latest_joint_time > self._feedback_timeout
            ):
                piper_recording = self._robot_model == "piperh"
                if piper_recording:
                    self._recording_timed_out = True
                self._enter_fault(
                    "joint feedback stale while recording",
                    stop_gravity=(
                        not piper_recording
                        or self._teach_drag_mode == "gravity_compensation"
                    ),
                )
            elif self._state in {"REPLAYING", "CANCELLING"} and (
                self._latest_positions is None
                or now - self._latest_joint_time > self._feedback_timeout
            ):
                self._enter_fault("joint feedback stale while replaying", cancel_goal=True)

    def _publish_status(self) -> None:
        with self._lock:
            trajectory = self._trajectory
            preflight = self._shape_preflight
            preflight_current = bool(
                preflight and preflight.get("signature") == self._shape_signature()
            )
            preflight_status = {"available": preflight_current}
            if preflight_current:
                counts = preflight["counts"]
                suggestion = preflight.get("suggested_pose")
                preflight_status.update({
                    "sampled_points": len(preflight["statuses"]),
                    "reachable_points": counts[PREFLIGHT_REACHABLE],
                    "near_limit_points": counts[PREFLIGHT_NEAR_LIMIT],
                    "collision_points": counts[PREFLIGHT_COLLISION],
                    "no_ik_points": counts[PREFLIGHT_NO_IK],
                    "feasible": (
                        counts[PREFLIGHT_COLLISION] == 0 and
                        counts[PREFLIGHT_NO_IK] == 0
                    ),
                    "has_suggestion": suggestion is not None,
                    "suggestion_distance_m": preflight.get(
                        "suggestion_distance_m", 0.0
                    ),
                    "suggested_position": (
                        [suggestion.position.x, suggestion.position.y,
                         suggestion.position.z]
                        if suggestion is not None else []
                    ),
                })
            document = {
                "state": self._state,
                "message": self._message,
                "allow_hardware": self._allow_hardware,
                "preview_only": self._preview_only,
                "robot_model": self._robot_model,
                "recording_samples": self._recorder.sample_count,
                "teach_drag_mode": self._teach_drag_mode,
                "normal_feedback_ready": (
                    self._latest_teach_positions is not None
                    and self._now() - self._latest_teach_arrival_time <= self._feedback_timeout
                ),
                "passive_recording_ready": (
                    self._robot_model == "piperh"
                    and self._latest_arm_status is not None
                    and self._latest_arm_status.state_machine == "DISABLED"
                    and not self._latest_arm_status.enabled
                    and not self._latest_arm_status.error_codes
                    and self._now() - self._latest_status_time <= self._status_timeout
                    and self._latest_positions is not None
                    and self._now() - self._latest_joint_time <= self._feedback_timeout
                    and not self._joint_limit_violations(self._latest_positions)
                    and (not self._require_xbox_locked or self._xbox_armed is False)
                ),
                "recording_source": self._recording_source,
                "recording_passive_frames": self._recording_passive_frames,
                "recording_dropped_gaps": self._recording_dropped_gaps,
                "recording_timed_out": self._recording_timed_out,
                "has_trajectory": trajectory is not None,
                "trajectory_points": len(trajectory.points) if trajectory else 0,
                "trajectory_duration_sec": trajectory.duration if trajectory else 0.0,
                "trajectory_path": str(self._trajectory_path),
                "selected_action_name": self._selected_action_name,
                "pending_action_name": self._pending_action_name,
                "action_library_dir": str(self._action_library.directory),
                "preview_active": self._preview_active,
                "preview_paused": self._preview_paused,
                "preview_generation": self._preview_generation,
                "preview_progress_offset": self._preview_progress_offset,
                "preview_duration_sec": self._preview_duration_sec,
                "preview_sequence_name": self._preview_sequence_name,
                "active_sequence_name": self._active_sequence_name,
                "sequence_remaining_actions": list(self._sequence_queue),
                "active_replay_speed_scale": self._active_replay_speed_scale,
                "replay_active": self._state
                in {"REPLAY_STARTING", "REPLAYING", "CANCELLING"},
                "replay_generation": self._replay_generation,
                "replay_progress": self._replay_progress,
                "replay_plan_duration_sec": self._replay_plan_duration_sec,
                "replay_action_name": self._selected_action_name,
                "replay_sequence_name": self._replay_sequence_name,
                "replay_action_index": self._replay_action_index,
                "replay_total_actions": self._replay_total_actions,
                "replay_target_positions": (
                    list(trajectory.points[-1].positions)
                    if trajectory is not None
                    and self._state
                    in {"REPLAY_STARTING", "REPLAYING", "CANCELLING"}
                    else []
                ),
                "shape_marker_visible": self._shape_marker_visible,
                "shape_marker_shape": self._shape_marker_shape,
                "shape_marker_source": self._shape_marker_source,
                "shape_marker_width": self._shape_marker_width,
                "shape_marker_height": self._shape_marker_height,
                "shape_pen_length_m": self._shape_pen_length_m,
                "shape_pen_mount_offset_m": self._shape_pen_mount_offset_m,
                "shape_pen_lift_m": self._shape_pen_lift_m,
                "shape_reachability": preflight_status,
                "tcp_trace_line_width_m": self._tcp_trace_line_width_m,
                "visualization_base_frame": self._visualization_base_frame,
                "visualization_tcp_link_name": self._visualization_tcp_link_name,
                "shape_marker_pose": {
                    "position": [
                        self._shape_marker_pose.position.x,
                        self._shape_marker_pose.position.y,
                        self._shape_marker_pose.position.z,
                    ],
                    "orientation": [
                        self._shape_marker_pose.orientation.x,
                        self._shape_marker_pose.orientation.y,
                        self._shape_marker_pose.orientation.z,
                        self._shape_marker_pose.orientation.w,
                    ],
                },
            }
            self._status_publisher.publish(
                String(data=json.dumps(document, ensure_ascii=False))
            )

    def request_safe_shutdown(self) -> list:
        futures = []
        with self._lock:
            if (
                self._state in {"STARTING", "RECORDING", "STOPPING"}
                and self._gravity_stop.service_is_ready()
            ):
                futures.append(self._gravity_stop.call_async(Trigger.Request()))
            if self._active_goal is not None:
                futures.append(self._active_goal.cancel_goal_async())
        return futures


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TeachModeNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        futures = node.request_safe_shutdown()
        deadline = time.monotonic() + 1.0
        while (
            rclpy.ok()
            and any(not future.done() for future in futures)
            and time.monotonic() < deadline
        ):
            executor.spin_once(timeout_sec=0.05)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
