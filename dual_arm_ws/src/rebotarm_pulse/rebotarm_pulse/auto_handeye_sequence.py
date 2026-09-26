"""Coordinate a guarded teach sequence with easy_handeye2 sampling services."""

from __future__ import annotations

from collections import deque
import json
import math
import time

from easy_handeye2_msgs.srv import (
    ComputeCalibration,
    RemoveSample,
    SaveCalibration,
    TakeSample,
)
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage


def _quaternion_normalized(transform) -> tuple[float, float, float, float]:
    rotation = transform.rotation
    values = (float(rotation.x), float(rotation.y), float(rotation.z), float(rotation.w))
    norm = math.sqrt(sum(value * value for value in values))
    if not math.isfinite(norm) or norm <= 1.0e-12:
        raise ValueError("invalid zero-length quaternion")
    return tuple(value / norm for value in values)


def _quaternion_multiply(left, right) -> tuple[float, float, float, float]:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def _rotate_vector(quaternion, vector) -> tuple[float, float, float]:
    x, y, z, w = quaternion
    vx, vy, vz = vector
    # Expanded q * [v, 0] * conjugate(q).
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + y * tz - z * ty,
        vy + w * ty + z * tx - x * tz,
        vz + w * tz + x * ty - y * tx,
    )


def _compose_pose(first_translation, first_q, second):
    second_q = _quaternion_normalized(second)
    rotated = _rotate_vector(
        first_q,
        (
            float(second.translation.x),
            float(second.translation.y),
            float(second.translation.z),
        ),
    )
    translation = (
        first_translation[0] + rotated[0],
        first_translation[1] + rotated[1],
        first_translation[2] + rotated[2],
    )
    return translation, _quaternion_multiply(first_q, second_q)


def _compose_transforms(first, second):
    return _compose_pose(
        (
            float(first.translation.x),
            float(first.translation.y),
            float(first.translation.z),
        ),
        _quaternion_normalized(first),
        second,
    )


def calibration_residuals(samples, calibration_transform) -> dict[str, float]:
    """Measure how stationary the marker is after applying an eye-in-hand result."""
    poses = []
    for sample in samples:
        robot_camera_t, robot_camera_q = _compose_transforms(
            sample.robot, calibration_transform
        )

        poses.append(_compose_pose(robot_camera_t, robot_camera_q, sample.tracking))
    if len(poses) < 3:
        raise ValueError("at least three samples are required for a residual check")

    translations = [pose[0] for pose in poses]
    mean_translation = tuple(
        sum(value[axis] for value in translations) / len(translations)
        for axis in range(3)
    )
    translation_errors = [
        math.sqrt(sum((value[axis] - mean_translation[axis]) ** 2 for axis in range(3)))
        for value in translations
    ]

    reference = poses[0][1]
    aligned = []
    for _, quaternion in poses:
        if sum(a * b for a, b in zip(reference, quaternion)) < 0.0:
            quaternion = tuple(-value for value in quaternion)
        aligned.append(quaternion)
    mean_quaternion = tuple(
        sum(value[axis] for value in aligned) / len(aligned) for axis in range(4)
    )
    norm = math.sqrt(sum(value * value for value in mean_quaternion))
    mean_quaternion = tuple(value / norm for value in mean_quaternion)
    rotation_errors = [
        math.degrees(
            2.0
            * math.acos(
                min(1.0, max(-1.0, abs(sum(a * b for a, b in zip(mean_quaternion, value)))))
            )
        )
        for value in aligned
    ]
    return {
        "translation_rms_m": math.sqrt(
            sum(value * value for value in translation_errors) / len(translation_errors)
        ),
        "translation_max_m": max(translation_errors),
        "rotation_rms_deg": math.sqrt(
            sum(value * value for value in rotation_errors) / len(rotation_errors)
        ),
        "rotation_max_deg": max(rotation_errors),
    }


class AutoHandeyeSequence(Node):
    """Sample stationary sequence targets, then compute and save calibration."""

    def __init__(self) -> None:
        super().__init__("auto_handeye_sequence")
        self.declare_parameter("sequence_name", "自动手眼标定_DM_12姿态")
        self.declare_parameter("action_name_prefix", "手眼标定姿态_")
        self.declare_parameter("calibration_type", "eye_on_base")
        self.declare_parameter("teach_status_topic", "/rebotarm/teach/status")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("teach_cancel_service", "/rebotarm/teach/cancel")
        self.declare_parameter("tracking_marker_frame", "marker_frame")
        self.declare_parameter("minimum_samples", 12)
        self.declare_parameter("settle_sec", 1.0)
        self.declare_parameter("target_tolerance_rad", 0.015)
        self.declare_parameter("feedback_timeout_sec", 0.75)
        self.declare_parameter("service_timeout_sec", 2.0)
        self.declare_parameter("startup_timeout_sec", 8.0)
        self.declare_parameter("marker_freshness_sec", 0.35)
        self.declare_parameter("marker_window_sec", 1.0)
        self.declare_parameter("marker_min_frames", 5)
        self.declare_parameter("marker_max_translation_jitter_m", 0.006)
        self.declare_parameter("marker_max_rotation_jitter_deg", 2.0)
        self.declare_parameter("tf_retry_timeout_sec", 5.0)
        self.declare_parameter("max_translation_rms_m", 0.015)
        self.declare_parameter("max_translation_error_m", 0.030)
        self.declare_parameter("max_rotation_rms_deg", 2.0)
        self.declare_parameter("max_rotation_error_deg", 4.0)

        self._sequence_name = str(self.get_parameter("sequence_name").value)
        self._action_name_prefix = str(
            self.get_parameter("action_name_prefix").value
        )
        self._calibration_type = str(
            self.get_parameter("calibration_type").value
        )
        self._minimum_samples = int(self.get_parameter("minimum_samples").value)
        self._settle_sec = float(self.get_parameter("settle_sec").value)
        self._target_tolerance = float(
            self.get_parameter("target_tolerance_rad").value
        )
        self._feedback_timeout = float(
            self.get_parameter("feedback_timeout_sec").value
        )
        self._service_timeout = float(
            self.get_parameter("service_timeout_sec").value
        )
        self._startup_timeout = float(
            self.get_parameter("startup_timeout_sec").value
        )
        self._tracking_marker_frame = str(
            self.get_parameter("tracking_marker_frame").value
        ).lstrip("/")
        self._marker_freshness = float(
            self.get_parameter("marker_freshness_sec").value
        )
        self._marker_window = float(self.get_parameter("marker_window_sec").value)
        self._marker_min_frames = int(self.get_parameter("marker_min_frames").value)
        self._marker_max_translation_jitter = float(
            self.get_parameter("marker_max_translation_jitter_m").value
        )
        self._marker_max_rotation_jitter = math.radians(
            float(self.get_parameter("marker_max_rotation_jitter_deg").value)
        )
        self._tf_retry_timeout = float(
            self.get_parameter("tf_retry_timeout_sec").value
        )
        self._max_translation_rms = float(
            self.get_parameter("max_translation_rms_m").value
        )
        self._max_translation_error = float(
            self.get_parameter("max_translation_error_m").value
        )
        self._max_rotation_rms = float(
            self.get_parameter("max_rotation_rms_deg").value
        )
        self._max_rotation_error = float(
            self.get_parameter("max_rotation_error_deg").value
        )
        if (
            not self._sequence_name
            or not self._action_name_prefix
            or self._minimum_samples < 3
        ):
            raise ValueError(
                "sequence_name/action_name_prefix must be set and minimum_samples must be >= 3"
            )
        if self._calibration_type not in {"eye_on_base", "eye_in_hand"}:
            raise ValueError("calibration_type must be eye_on_base or eye_in_hand")
        if not all(
            math.isfinite(value) and value > 0.0
            for value in (
                self._settle_sec,
                self._target_tolerance,
                self._feedback_timeout,
                self._service_timeout,
                self._startup_timeout,
                self._marker_freshness,
                self._marker_window,
                self._marker_max_translation_jitter,
                self._marker_max_rotation_jitter,
                self._tf_retry_timeout,
                self._max_translation_rms,
                self._max_translation_error,
                self._max_rotation_rms,
                self._max_rotation_error,
            )
        ) or self._marker_min_frames < 3:
            raise ValueError("automatic hand-eye timing and tolerances must be positive")

        self._joint_names = tuple(f"joint{index}" for index in range(1, 7))
        self._positions: tuple[float, ...] | None = None
        self._joint_stamp = 0.0
        self._target: tuple[float, ...] | None = None
        self._target_index = 0
        self._total_actions = 0
        self._stable_since = 0.0
        self._sampled_indices: set[int] = set()
        self._skipped_indices: set[int] = set()
        self._baseline_samples: int | None = None
        self._last_sample_count = 0
        self._phase = "idle"
        self._phase_started = 0.0
        self._replay_active = False
        self._state_message = ""
        self._pending_kind = ""
        self._pending_started = 0.0
        self._pending_future = None
        self._pending_pose_index = 0
        self._latest_samples = []
        self._marker_observations = deque(maxlen=max(20, self._marker_min_frames * 4))
        self._last_marker_stamp_ns = -1
        self._tf_retry_deadline = 0.0

        qos = rclpy.qos.QoSProfile(depth=1)
        qos.reliability = rclpy.qos.ReliabilityPolicy.RELIABLE
        qos.durability = rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            String,
            str(self.get_parameter("teach_status_topic").value),
            self._teach_status_callback,
            qos,
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_state_topic").value),
            self._joint_state_callback,
            rclpy.qos.qos_profile_sensor_data,
        )
        self.create_subscription(
            TFMessage,
            "/tf",
            self._tf_callback,
            rclpy.qos.qos_profile_sensor_data,
        )
        self._status_publisher = self.create_publisher(
            String, "/rebotarm/handeye/auto_status", qos
        )
        self._current_client = self.create_client(
            TakeSample, "/easy_handeye2/calibration/get_current_transforms"
        )
        self._sample_list_client = self.create_client(
            TakeSample, "/easy_handeye2/calibration/get_sample_list"
        )
        self._sample_client = self.create_client(
            TakeSample, "/easy_handeye2/calibration/take_sample"
        )
        self._remove_sample_client = self.create_client(
            RemoveSample, "/easy_handeye2/calibration/remove_sample"
        )
        self._compute_client = self.create_client(
            ComputeCalibration, "/easy_handeye2/calibration/compute_calibration"
        )
        self._save_client = self.create_client(
            SaveCalibration, "/easy_handeye2/calibration/save_calibration"
        )
        self._cancel_client = self.create_client(
            Trigger, str(self.get_parameter("teach_cancel_service").value)
        )
        self.create_timer(0.05, self._advance)
        self._publish("idle", "等待指定标定动作组开始回放")

    @staticmethod
    def _sample_count(response) -> int:
        samples = getattr(getattr(response, "samples", None), "samples", ())
        return len(samples)

    def _publish(self, state: str, message: str, **extra) -> None:
        document = {
            "state": state,
            "message": message,
            "sequence_name": self._sequence_name,
            "calibration_type": self._calibration_type,
            "sampled_poses": len(self._sampled_indices),
            "skipped_poses": sorted(self._skipped_indices),
            "required_poses": self._minimum_samples,
            "action_index": self._target_index,
            "total_actions": self._total_actions,
            **extra,
        }
        output = String()
        output.data = json.dumps(document, ensure_ascii=False)
        self._status_publisher.publish(output)

    def _joint_state_callback(self, message: JointState) -> None:
        lookup = {name: index for index, name in enumerate(message.name)}
        if any(name not in lookup for name in self._joint_names):
            return
        positions = tuple(float(message.position[lookup[name]]) for name in self._joint_names)
        if not all(math.isfinite(value) for value in positions):
            return
        self._positions = positions
        self._joint_stamp = time.monotonic()

    def _tf_callback(self, message: TFMessage) -> None:
        now = time.monotonic()
        for stamped in message.transforms:
            if stamped.child_frame_id.lstrip("/") != self._tracking_marker_frame:
                continue
            stamp_ns = int(stamped.header.stamp.sec) * 1_000_000_000 + int(
                stamped.header.stamp.nanosec
            )
            if stamp_ns <= self._last_marker_stamp_ns:
                continue
            transform = stamped.transform
            try:
                quaternion = _quaternion_normalized(transform)
            except ValueError:
                continue
            translation = (
                float(transform.translation.x),
                float(transform.translation.y),
                float(transform.translation.z),
            )
            if not all(math.isfinite(value) for value in translation):
                continue
            self._last_marker_stamp_ns = stamp_ns
            self._marker_observations.append((now, translation, quaternion))

    def _marker_window_ready(self, now: float) -> tuple[bool, dict[str, float]]:
        observations = [
            item
            for item in self._marker_observations
            if item[0] >= self._stable_since and now - item[0] <= self._marker_window
        ]
        metrics = {"marker_frames": len(observations)}
        if len(observations) < self._marker_min_frames:
            return False, metrics
        if now - observations[-1][0] > self._marker_freshness:
            return False, metrics
        reference_t = observations[-1][1]
        reference_q = observations[-1][2]
        translation_jitter = max(
            math.sqrt(sum((value[1][axis] - reference_t[axis]) ** 2 for axis in range(3)))
            for value in observations
        )
        rotation_jitter = max(
            2.0
            * math.acos(
                min(1.0, max(-1.0, abs(sum(a * b for a, b in zip(value[2], reference_q)))))
            )
            for value in observations
        )
        metrics.update(
            marker_translation_jitter_mm=translation_jitter * 1000.0,
            marker_rotation_jitter_deg=math.degrees(rotation_jitter),
        )
        return (
            translation_jitter <= self._marker_max_translation_jitter
            and rotation_jitter <= self._marker_max_rotation_jitter,
            metrics,
        )

    def _teach_status_callback(self, message: String) -> None:
        try:
            document = json.loads(message.data)
        except (TypeError, ValueError):
            return
        replay_sequence = str(document.get("replay_sequence_name", ""))
        if replay_sequence != self._sequence_name:
            return
        self._replay_active = bool(document.get("replay_active", False))
        self._state_message = str(document.get("message", ""))
        index = int(document.get("replay_action_index", 0))
        total = int(document.get("replay_total_actions", 0))
        action_name = str(document.get("replay_action_name", ""))
        raw_target = document.get("replay_target_positions", [])
        target = None
        if isinstance(raw_target, list) and len(raw_target) == len(self._joint_names):
            candidate = tuple(float(value) for value in raw_target)
            if all(math.isfinite(value) for value in candidate):
                target = candidate

        if self._replay_active and self._phase == "idle":
            self._total_actions = total
            if total < self._minimum_samples:
                self._fail(
                    f"标定动作组只有 {total} 个姿态，至少需要 {self._minimum_samples} 个"
                )
                return
            self._phase = "starting"
            self._phase_started = time.monotonic()
            self._publish("starting", "标定动作组已启动，正在确认采样服务")
        if self._replay_active and index > 0:
            expected = f"{self._action_name_prefix}{index:02d}"
            if action_name != expected:
                self._fail(
                    f"自动标定动作组内容或顺序已改变：第 {index} 项应为 {expected}"
                )
                return
        if self._phase in {"starting", "sampling"} and index != self._target_index:
            previous_index = self._target_index
            self._target_index = index
            self._target = target
            self._stable_since = 0.0
            self._tf_retry_deadline = 0.0
            if self._pending_kind == "current" and previous_index > 0:
                if self._pending_future is not None:
                    self._pending_future.cancel()
                self._pending_kind = ""
                self._pending_future = None
                self._skip_pose(
                    previous_index,
                    f"姿态 {previous_index} 的 TF 检查尚未完成，已跳过并继续",
                )
            self._publish("moving", f"正在前往标定姿态 {index}/{total}")
        elif target is not None:
            self._target = target

    def _services_ready(self) -> bool:
        return all(
            client.service_is_ready()
            for client in (
                self._current_client,
                self._sample_list_client,
                self._sample_client,
                self._remove_sample_client,
                self._compute_client,
                self._save_client,
            )
        )

    def _start_call(self, kind: str, client, request) -> None:
        self._pending_kind = kind
        self._pending_started = time.monotonic()
        self._pending_future = client.call_async(request)

    def _cancel_replay(self) -> None:
        if self._cancel_client.service_is_ready():
            self._cancel_client.call_async(Trigger.Request())

    def _skip_pose(self, pose_index: int, message: str) -> None:
        if pose_index > 0:
            self._skipped_indices.add(pose_index)
        self._pending_pose_index = 0
        self.get_logger().warning(message)
        self._publish("skipped", message)

    def _fail(self, message: str) -> None:
        if self._phase in {"failed", "saved"}:
            return
        self._phase = "failed"
        self._pending_kind = ""
        self._pending_future = None
        if self._replay_active:
            self._cancel_replay()
        self.get_logger().error(message)
        self._publish("failed", message)

    def _handle_pending(self) -> None:
        future = self._pending_future
        if future is None:
            return
        if not future.done():
            if time.monotonic() - self._pending_started > self._service_timeout:
                if self._pending_kind == "current":
                    pose_index = self._pending_pose_index
                    future.cancel()
                    self._pending_kind = ""
                    self._pending_future = None
                    self._skip_pose(
                        pose_index,
                        f"姿态 {pose_index} 的 TF 检查超时，已跳过并继续",
                    )
                    return
                self._fail(f"{self._pending_kind} 服务响应超时")
            return
        kind = self._pending_kind
        self._pending_kind = ""
        self._pending_future = None
        try:
            response = future.result()
        except Exception as error:  # rclpy propagates service transport errors here
            self._fail(f"{kind} 服务调用失败：{error}")
            return
        if kind == "sample_list":
            self._baseline_samples = self._sample_count(response)
            self._last_sample_count = self._baseline_samples
            if self._baseline_samples != 0:
                self._fail(
                    "自动流程要求空样本集；当前 easy_handeye2 已有 "
                    f"{self._baseline_samples} 个样本，请停止并重新启动手眼标定"
                )
                return
            self._phase = "sampling"
            self._publish("sampling", "采样服务就绪，等待第一个姿态停稳")
        elif kind == "current":
            if self._target_index != self._pending_pose_index:
                self._skip_pose(
                    self._pending_pose_index,
                    "TF 检查完成前动作组已离开当前姿态，已跳过并继续",
                )
                return
            if self._sample_count(response) != 1:
                now = time.monotonic()
                if now < self._tf_retry_deadline:
                    self._stable_since = now
                    self._marker_observations.clear()
                    self._pending_pose_index = 0
                    self._publish(
                        "waiting_marker",
                        "TF 瞬时不可用，正在等待新的 AprilTag 稳定帧后重试",
                    )
                    return
                self._skip_pose(
                    self._pending_pose_index,
                    f"姿态 {self._pending_pose_index} 未取得有效 AprilTag/机器人 TF，已跳过并继续",
                )
                return
            self._start_call("sample", self._sample_client, TakeSample.Request())
        elif kind == "sample":
            if self._target_index != self._pending_pose_index:
                count = self._sample_count(response)
                if count > self._last_sample_count:
                    request = RemoveSample.Request()
                    request.sample_index = count - 1
                    self._start_call("discard_sample", self._remove_sample_client, request)
                else:
                    self._skip_pose(
                        self._pending_pose_index,
                        "写入采样时动作组已离开当前姿态，已跳过并继续",
                    )
                return
            count = self._sample_count(response)
            if count <= self._last_sample_count:
                # A transient TF lookup can make take_sample return the
                # unchanged sample list.  This is not fatal when the other
                # poses already provide enough observations, including when
                # it happens on the final action after replay has stopped.
                self._last_sample_count = count
                self._latest_samples = list(
                    getattr(getattr(response, "samples", None), "samples", ())
                )
                self._skip_pose(
                    self._pending_pose_index,
                    f"姿态 {self._pending_pose_index} 采样未写入 easy_handeye2，已跳过并继续",
                )
                return
            self._last_sample_count = count
            self._latest_samples = list(
                getattr(getattr(response, "samples", None), "samples", ())
            )
            self._sampled_indices.add(self._pending_pose_index)
            self._stable_since = 0.0
            self._tf_retry_deadline = 0.0
            self._publish(
                "sampled",
                f"已采样姿态 {self._pending_pose_index}/{self._total_actions}",
                easy_handeye_sample_count=count,
            )
            self._pending_pose_index = 0
        elif kind == "discard_sample":
            self._last_sample_count = self._sample_count(response)
            self._latest_samples = list(
                getattr(getattr(response, "samples", None), "samples", ())
            )
            self._skip_pose(
                self._pending_pose_index,
                "动作组移动期间产生的样本已丢弃，自动流程继续",
            )
        elif kind == "compute":
            if not bool(getattr(response, "valid", False)):
                self._fail("easy_handeye2 未能计算出有效标定结果，未保存")
                return
            try:
                metrics = calibration_residuals(
                    self._latest_samples, response.calibration.transform
                )
            except (TypeError, ValueError, ZeroDivisionError) as error:
                self._fail(f"无法验证标定残差，未保存：{error}")
                return
            self._publish("validating", "正在验证标定残差", **metrics)
            if (
                metrics["translation_rms_m"] > self._max_translation_rms
                or metrics["translation_max_m"] > self._max_translation_error
                or metrics["rotation_rms_deg"] > self._max_rotation_rms
                or metrics["rotation_max_deg"] > self._max_rotation_error
            ):
                self._fail(
                    "标定残差超限，拒绝保存：平移 RMS "
                    f"{metrics['translation_rms_m'] * 1000.0:.1f} mm，旋转 RMS "
                    f"{metrics['rotation_rms_deg']:.2f}°"
                )
                return
            self._phase = "saving"
            self._publish("saving", "标定质量检查通过，正在保存", **metrics)
            self._start_call("save", self._save_client, SaveCalibration.Request())
        elif kind == "save":
            if not bool(getattr(response, "success", False)):
                self._fail("easy_handeye2 标定结果保存失败")
                return
            filepath = str(getattr(getattr(response, "filepath", None), "data", ""))
            self._phase = "saved"
            self.get_logger().info(f"automatic hand-eye calibration saved to {filepath}")
            self._publish("saved", "自动手眼标定已计算并保存", filepath=filepath)

    def _advance(self) -> None:
        if self._pending_future is not None:
            self._handle_pending()
            return
        if self._phase in {"idle", "failed", "saved", "computing", "saving"}:
            return
        if self._phase == "starting":
            if not self._services_ready():
                if time.monotonic() - self._phase_started > self._startup_timeout:
                    self._fail(
                        "标定服务未就绪；请检查 gripper_tcp 拼写以及 AprilTag 是否持续可见"
                    )
                return
            self._start_call("sample_list", self._sample_list_client, TakeSample.Request())
            return
        if self._phase != "sampling":
            return
        if not self._replay_active:
            if len(self._sampled_indices) < self._minimum_samples:
                self._fail(
                    "动作组已结束，但有效采样不足："
                    f"{len(self._sampled_indices)}/{self._minimum_samples}"
                )
                return
            self._phase = "computing"
            self._publish("computing", "全部姿态采样完成，正在计算标定")
            self._start_call(
                "compute", self._compute_client, ComputeCalibration.Request()
            )
            return
        if self._target_index <= 0 or self._target_index in self._sampled_indices:
            return
        now = time.monotonic()
        if self._positions is None or now - self._joint_stamp > self._feedback_timeout:
            self._stable_since = 0.0
            return
        if self._target is None:
            self._stable_since = 0.0
            return
        error = max(abs(a - b) for a, b in zip(self._positions, self._target))
        if error > self._target_tolerance:
            self._stable_since = 0.0
            return
        if self._stable_since <= 0.0:
            self._stable_since = now
            return
        if now - self._stable_since < self._settle_sec:
            return
        marker_ready, marker_metrics = self._marker_window_ready(now)
        if not marker_ready:
            self._publish(
                "waiting_marker",
                f"姿态 {self._target_index} 已停稳，等待 AprilTag 连续稳定",
                **marker_metrics,
            )
            return
        self._publish(
            "checking_tf",
            f"姿态 {self._target_index} 的 AprilTag 已连续稳定，正在检查 TF",
            **marker_metrics,
        )
        self._pending_pose_index = self._target_index
        if self._tf_retry_deadline <= 0.0:
            self._tf_retry_deadline = now + self._tf_retry_timeout
        self._start_call("current", self._current_client, TakeSample.Request())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AutoHandeyeSequence()
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
