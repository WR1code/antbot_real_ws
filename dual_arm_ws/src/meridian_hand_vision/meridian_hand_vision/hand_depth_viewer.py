from __future__ import annotations

import argparse
import hashlib
import os
import json
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point, PointStamped, Vector3, Vector3Stamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from visualization_msgs.msg import Marker

from . import axis_overlay as axis_overlay_module
from .arm_contour import estimate_arm_contours, estimate_wrist_geometry_mask
from .axis_overlay import (
    draw_candidate_forearm_axis_overlay,
    draw_debug_force_forearm_arrow,
    draw_forearm_diagnostic_overlay,
    draw_forearm_direction_overlay,
    select_forearm_direction_overlay,
    select_vision_forearm_axis,
    unsigned_axis_angle_deg,
)
from .branch_candidate_shadow import (
    BranchCandidateShadowState,
    update_branch_candidate_shadow,
)
from .branch_local_recovery_shadow import (
    BranchLocalRecoveryShadowState,
    update_branch_local_recovery_shadow,
)
from .calibration_board import estimate_calibration_board_normal
from .gesture_classifier import StableGestureFilter, classify_gesture
from .hand_landmarker import HandLandmarker, draw_hand_landmarks
from .image_conversion import (
    color_message_to_bgr,
    colorize_depth,
    depth_message_to_meters,
)
from .invalid_input_shadow import (
    InvalidInputShadowState,
    update_invalid_input_shadow,
)
from .pulse_region import (
    CameraIntrinsics,
    ForearmAxisSelectorState,
    StableArmDirectionFilter,
    StablePulseFilter,
    StableSurfaceNormalFilter,
    WristGeometryBranchTrackerState,
    estimate_forearm_axis_wrist_geometry,
    estimate_pulse_region_diagnostic,
    estimate_surface_patch_diagnostic,
    select_forearm_axis,
    update_wrist_geometry_branch_tracks,
)
from .primary_degraded_temporal import (
    PrimaryDegradedTemporalShadowState,
    update_primary_degraded_temporal_shadow,
)
from .primary_degraded_keep_shadow import (
    PrimaryDegradedKeepShadowState,
    update_primary_degraded_keep_shadow,
)
from .persistent_forearm_validity_shadow import (
    PersistentForearmValidityShadowState,
    update_persistent_forearm_validity_shadow,
)
from .rgbd_sync import validate_rgb_depth_stamps
from .selector_fallback_strategy_shadow import (
    SelectorFallbackStrategyShadowState,
    update_selector_fallback_strategy_shadow,
)
from .wrist_input_robustness import (
    WristTemporalShadowState,
    update_wrist_input_robustness_shadow,
)
from .wrist_component_recovery_shadow import (
    add_branch_comparison,
    estimate_wrist_component_recovery_shadow,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="显示 Orbbec 彩色图、对齐深度、MediaPipe 手部点和手臂轮廓。"
    )
    parser.add_argument("--color-topic", default="/camera/color/image_raw")
    parser.add_argument("--depth-topic", default="/camera/depth/image_raw")
    parser.add_argument("--camera-info-topic", default="/camera/color/camera_info")
    parser.add_argument(
        "--model",
        type=Path,
        default=Path(os.environ.get("MERIDIAN_HAND_MODEL", "hand_landmarker.task")),
    )
    parser.add_argument("--num-hands", type=int, default=2, choices=(1, 2))
    parser.add_argument(
        "--max-rgb-depth-time-diff-ms", "--sync-ms",
        dest="max_rgb_depth_time_diff_ms", type=float, default=30.0,
        help="RGB/Depth 采集时间最大差值；超限帧直接拒绝（--sync-ms 为兼容别名）",
    )
    parser.add_argument("--max-depth", type=float, default=3.0)
    parser.add_argument("--depth-tolerance", type=float, default=0.18)
    parser.add_argument("--display-scale", type=float, default=0.55)
    parser.add_argument(
        "--no-window", action="store_true",
        help="仅发布标注图像供 RViz 使用，不打开独立 OpenCV 窗口",
    )
    parser.add_argument(
        "--debug-force-forearm-arrow",
        action="store_true",
        help="仅调试：在图像中心绘制固定100px蓝色箭头",
    )
    parser.add_argument(
        "--enable-wrist-component-recovery-shadow",
        action="store_true",
        help=(
            "运行腕部component恢复旁路及完整branch对比；默认关闭以避免"
            "额外几何拟合改变实时采样频率"
        ),
    )
    parser.add_argument(
        "--mirror", action="store_true",
        help="启用水平镜像（默认显示摄像头原始方向）",
    )
    # Keep the old flag accepted for scripts written before the default was
    # corrected. It is now redundant because the default is non-mirrored.
    parser.add_argument("--no-mirror", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--gesture-topic", default="/demo/gesture")
    parser.add_argument("--gesture-hold-frames", type=int, default=12)
    parser.add_argument(
        "--annotated-topic", default="/meridian_hand_vision/annotated_image",
        help="发布带关键点/轮廓标注的 RGBD 拼接图像话题",
    )
    parser.add_argument("--pulse-topic", default="/meridian_hand_vision/pulse_point")
    parser.add_argument("--pulse-status-topic", default="/meridian_hand_vision/pulse_status")
    parser.add_argument("--pulse-marker-topic", default="/meridian_hand_vision/pulse_marker")
    parser.add_argument(
        "--arm-direction-topic",
        default="/meridian_hand_vision/arm_direction",
    )
    parser.add_argument(
        "--surface-normal-topic",
        default="/meridian_hand_vision/surface_normal",
    )
    parser.add_argument(
        "--calibration-board-corner-topic",
        default="/meridian_hand_vision/calibration_board_corner",
    )
    parser.add_argument(
        "--calibration-board-normal-topic",
        default="/meridian_hand_vision/calibration_board_normal",
    )
    parser.add_argument("--target-filter-window-sec", type=float, default=0.6)
    parser.add_argument("--target-stable-duration-sec", type=float, default=0.0,
                        help="deprecated compatibility option; not a stability gate")
    parser.add_argument("--target-stable-min-samples", type=int, default=3)
    parser.add_argument("--target-stable-radius-m", type=float, default=0.005)
    parser.add_argument("--target-stable-hold-radius-m", type=float, default=0.008)
    parser.add_argument("--target-unstable-hold-sec", type=float, default=0.2)
    parser.add_argument("--vision-dropout-grace-sec", type=float, default=0.5)
    parser.add_argument("--target-soft-timeout-sec", type=float, default=1.5)
    parser.add_argument("--target-hard-timeout-sec", type=float, default=2.0)
    parser.add_argument("--target-hard-jump-m", type=float, default=0.015)
    # This independent filter only stabilizes the approximate forearm axis;
    # it is not used to decide whether a pulse target may be published.
    parser.add_argument("--arm-direction-stable-frames", type=int, default=12)
    parser.add_argument("--surface-normal-stable-frames", type=int, default=8)
    parser.add_argument("--surface-normal-stable-deg", type=float, default=5.0)
    parser.add_argument(
        "--surface-normal-source",
        choices=("local_skin",),
        default="local_skin",
        help="手部 approach 法向固定来自目标像素附近的局部皮肤深度平面",
    )
    return parser.parse_args()


def _source_identity(path_value) -> dict:
    path = Path(path_value).resolve()
    try:
        content = path.read_bytes()
        stat = path.stat()
        return {
            "path": str(path),
            "mtime_ns": int(stat.st_mtime_ns),
            "sha256": hashlib.sha256(content).hexdigest(),
        }
    except OSError as error:
        return {"path": str(path), "error": type(error).__name__}


def _stamp_ns(message: Image) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


class RgbDepthSubscriber(Node):
    def __init__(
        self,
        color_topic: str,
        depth_topic: str,
        camera_info_topic: str,
        max_rgb_depth_time_diff_ms: float,
        gesture_topic: str,
        annotated_topic: str,
        pulse_topic: str,
        pulse_status_topic: str,
        pulse_marker_topic: str,
        arm_direction_topic: str,
        surface_normal_topic: str,
        calibration_board_corner_topic: str,
        calibration_board_normal_topic: str,
    ) -> None:
        super().__init__("meridian_hand_depth_viewer")
        self.color_message: Image | None = None
        self.depth_message: Image | None = None
        self.camera_info: CameraInfo | None = None
        self.last_color_rx = 0.0
        self.last_depth_rx = 0.0
        self.color_interval_ms: float | None = None
        self.sync_drops_total = 0
        self.maximum_delta_ns = int(max_rgb_depth_time_diff_ms * 1_000_000)
        self.last_rgb_depth_delta_ms: float | None = None
        self.last_sync_reject_reason: str | None = None
        # Vision inference is intentionally lossy: once processing falls behind,
        # an old RGB-D frame has no value and only increases control/display
        # latency. Keep exactly the newest sample for each stream.
        latest_sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(
            Image, color_topic, self._on_color, latest_sensor_qos
        )
        self.create_subscription(
            Image, depth_topic, self._on_depth, latest_sensor_qos
        )
        self.create_subscription(
            CameraInfo, camera_info_topic, self._on_camera_info,
            qos_profile_sensor_data,
        )
        self.gesture_publisher = self.create_publisher(String, gesture_topic, 10)
        self.annotated_image_publisher = self.create_publisher(
            Image, annotated_topic, latest_sensor_qos
        )
        self.pulse_publisher = self.create_publisher(PointStamped, pulse_topic, 10)
        self.pulse_status_publisher = self.create_publisher(String, pulse_status_topic, 10)
        self.pulse_marker_publisher = self.create_publisher(Marker, pulse_marker_topic, 10)
        self.arm_direction_publisher = self.create_publisher(
            Vector3Stamped, arm_direction_topic, 10
        )
        self.surface_normal_publisher = self.create_publisher(
            Vector3Stamped, surface_normal_topic, 10
        )
        self.calibration_board_corner_publisher = self.create_publisher(
            PointStamped, calibration_board_corner_topic, 10
        )
        self.calibration_board_normal_publisher = self.create_publisher(
            Vector3Stamped, calibration_board_normal_topic, 10
        )

    def publish_gesture(self, gesture: str) -> None:
        self.gesture_publisher.publish(String(data=gesture))

    def publish_annotated_image(self, image: np.ndarray, header) -> None:
        if image.ndim != 3 or image.shape[2] != 3 or not image.flags.c_contiguous:
            image = np.ascontiguousarray(image)
        message = Image()
        message.header = header
        message.height, message.width = image.shape[:2]
        message.encoding = "bgr8"
        message.is_bigendian = False
        message.step = int(image.shape[1] * 3)
        message.data = image.tobytes()
        self.annotated_image_publisher.publish(message)

    def _on_color(self, message: Image) -> None:
        received = time.monotonic()
        self.color_interval_ms = (
            (received - self.last_color_rx) * 1000.0 if self.last_color_rx else None
        )
        self.last_color_rx = received
        self.color_message = message

    def _on_depth(self, message: Image) -> None:
        self.last_depth_rx = time.monotonic()
        self.depth_message = message

    def _on_camera_info(self, message: CameraInfo) -> None:
        self.camera_info = message

    def publish_pulse(self, region, header, **diagnostics) -> None:
        message = PointStamped()
        message.header.stamp = header.stamp
        message.header.frame_id = (
            self.camera_info.header.frame_id
            if self.camera_info is not None and self.camera_info.header.frame_id
            else header.frame_id
        )
        message.point = Point(
            x=region.point_m[0], y=region.point_m[1], z=region.point_m[2]
        )
        # Publish the structured frame record first so the downstream target
        # bridge can correlate the following PointStamped with frame_seq.
        self.publish_pulse_status(
            True,
            "NONE",
            frame_id=message.header.frame_id,
            pixel=region.pixel,
            point=region.point_m,
            radius_m=region.radius_m,
            publish_source_target=True,
            **diagnostics,
        )
        self.pulse_publisher.publish(message)

        marker = Marker()
        marker.header = message.header
        marker.ns = "pulse_region"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = message.point
        marker.pose.orientation.w = 1.0
        diameter = region.radius_m * 2.0
        marker.scale.x = diameter
        marker.scale.y = diameter
        marker.scale.z = max(0.004, diameter * 0.35)
        marker.color.r = 1.0
        marker.color.g = 0.25
        marker.color.b = 0.05
        marker.color.a = 0.90
        # Raw camera-space visualization must disappear when the vision source
        # stops.  It is not the transformed Piper-H planning target.
        marker.lifetime.nanosec = 500_000_000
        self.pulse_marker_publisher.publish(marker)

    def publish_arm_direction(self, direction, header) -> None:
        message = Vector3Stamped()
        message.header.stamp = header.stamp
        message.header.frame_id = (
            self.camera_info.header.frame_id
            if self.camera_info is not None and self.camera_info.header.frame_id
            else header.frame_id
        )
        message.vector = Vector3(
            x=direction.vector[0],
            y=direction.vector[1],
            z=direction.vector[2],
        )
        self.arm_direction_publisher.publish(message)

    def publish_surface_normal(self, normal, header) -> None:
        message = Vector3Stamped()
        message.header.stamp = header.stamp
        message.header.frame_id = (
            self.camera_info.header.frame_id
            if self.camera_info is not None and self.camera_info.header.frame_id
            else header.frame_id
        )
        message.vector = Vector3(
            x=normal.vector[0], y=normal.vector[1], z=normal.vector[2]
        )
        self.surface_normal_publisher.publish(message)

    def publish_calibration_board_target(self, board, header) -> None:
        frame_id = (
            self.camera_info.header.frame_id
            if self.camera_info is not None and self.camera_info.header.frame_id
            else header.frame_id
        )
        point = PointStamped()
        point.header.stamp = header.stamp
        point.header.frame_id = frame_id
        point.point = Point(
            x=board.lower_right_corner_m[0],
            y=board.lower_right_corner_m[1],
            z=board.lower_right_corner_m[2],
        )
        normal = Vector3Stamped()
        normal.header = point.header
        normal.vector = Vector3(
            x=board.normal.vector[0],
            y=board.normal.vector[1],
            z=board.normal.vector[2],
        )
        self.calibration_board_corner_publisher.publish(point)
        self.calibration_board_normal_publisher.publish(normal)

    def publish_pulse_status(self, stable: bool, reason: str, **values) -> None:
        payload = {"stable": stable, "reason": reason, **values}
        self.pulse_status_publisher.publish(
            String(data=json.dumps(payload, ensure_ascii=False, sort_keys=True))
        )

    def take_pair(self) -> tuple[Image, Image] | None:
        if self.color_message is None or self.depth_message is None:
            return None
        color_stamp = _stamp_ns(self.color_message)
        depth_stamp = _stamp_ns(self.depth_message)
        valid, delta_ms, reason, stale = validate_rgb_depth_stamps(
            color_stamp, depth_stamp, self.maximum_delta_ns
        )
        self.last_rgb_depth_delta_ms = delta_ms
        if not valid:
            self.sync_drops_total += 1
            self.last_sync_reject_reason = reason
            if stale == "rgb":
                self.color_message = None
            elif stale == "depth":
                self.depth_message = None
            else:
                self.color_message = None
                self.depth_message = None
            return None
        pair = self.color_message, self.depth_message
        self.last_sync_reject_reason = None
        self.color_message = None
        self.depth_message = None
        return pair


def _title(panel: np.ndarray, text: str) -> None:
    cv2.rectangle(panel, (0, 0), (panel.shape[1], 38), (0, 0, 0), -1)
    cv2.putText(
        panel, text, (12, 27), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
        (255, 255, 255), 2, cv2.LINE_AA,
    )


def main() -> None:
    args = parse_args()
    runtime_source_identity = {
        "hand_depth_viewer": _source_identity(__file__),
        "arm_contour": _source_identity(
            estimate_wrist_geometry_mask.__code__.co_filename
        ),
        "pulse_region": _source_identity(
            estimate_forearm_axis_wrist_geometry.__code__.co_filename
        ),
        "component_recovery_shadow": _source_identity(
            estimate_wrist_component_recovery_shadow.__code__.co_filename
        ),
    }
    model_path = args.model.expanduser().resolve()
    if not model_path.is_file():
        raise FileNotFoundError(f"找不到 MediaPipe 模型：{model_path}")
    if args.display_scale <= 0.0:
        raise ValueError("--display-scale 必须大于 0。")
    if args.max_rgb_depth_time_diff_ms <= 0.0:
        raise ValueError("--max-rgb-depth-time-diff-ms 必须大于 0。")

    rclpy.init()
    node = RgbDepthSubscriber(
        args.color_topic, args.depth_topic, args.camera_info_topic,
        args.max_rgb_depth_time_diff_ms,
        args.gesture_topic, args.annotated_topic, args.pulse_topic,
        args.pulse_status_topic, args.pulse_marker_topic,
        args.arm_direction_topic, args.surface_normal_topic,
        args.calibration_board_corner_topic, args.calibration_board_normal_topic,
    )
    gesture_filter = StableGestureFilter(args.gesture_hold_frames)
    pulse_filter = StablePulseFilter(
        window_sec=args.target_filter_window_sec,
        stable_duration_sec=args.target_stable_duration_sec,
        min_samples=args.target_stable_min_samples,
        stable_radius_m=args.target_stable_radius_m,
        stable_hold_radius_m=args.target_stable_hold_radius_m,
        unstable_hold_sec=args.target_unstable_hold_sec,
        dropout_grace_sec=args.vision_dropout_grace_sec,
        soft_timeout_sec=args.target_soft_timeout_sec,
        hard_timeout_sec=args.target_hard_timeout_sec,
        hard_jump_m=args.target_hard_jump_m,
    )
    arm_direction_filter = StableArmDirectionFilter(args.arm_direction_stable_frames)
    arm_direction_selector = ForearmAxisSelectorState()
    primary_degraded_temporal_state = PrimaryDegradedTemporalShadowState()
    primary_degraded_keep_state = PrimaryDegradedKeepShadowState(
        filter_window_size=args.arm_direction_stable_frames,
        maximum_angle_deg=arm_direction_filter.maximum_angle_deg,
    )
    selector_fallback_strategy_shadow_state = (
        SelectorFallbackStrategyShadowState()
    )
    persistent_forearm_validity_shadow_state = (
        PersistentForearmValidityShadowState()
    )
    wrist_geometry_branch_tracker = WristGeometryBranchTrackerState()
    wrist_input_robustness_state = WristTemporalShadowState()
    branch_candidate_shadow_state = BranchCandidateShadowState()
    branch_local_recovery_shadow_state = BranchLocalRecoveryShadowState()
    invalid_input_shadow_state = InvalidInputShadowState()
    surface_normal_filter = StableSurfaceNormalFilter(
        args.surface_normal_stable_frames, args.surface_normal_stable_deg
    )
    start_time = time.monotonic()
    last_notice = 0.0
    last_source_heartbeat = 0.0
    last_stable_status_log = 0.0
    last_forearm_overlay_log = 0.0
    encoding_notice: tuple[str, str] | None = None
    frame_count = 0
    last_frame_received = 0.0
    print(f"彩色 Topic：{args.color_topic}")
    print(f"深度 Topic：{args.depth_topic}")
    print(f"相机参数 Topic：{args.camera_info_topic}")
    print(f"稳定手势 Topic：{args.gesture_topic}")
    print(f"标注图像 Topic：{args.annotated_topic}")
    if args.no_window:
        print("无窗口模式：标注图像仅发布到 RViz；按 Ctrl+C 退出。")
    else:
        print("按 Q、Esc 或 Ctrl+C 退出。")

    try:
        with HandLandmarker(model_path, args.num_hands) as detector:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.02)
                pair = node.take_pair()
                if pair is None:
                    now = time.monotonic()
                    pulse_filter.observe_gap(now)
                    gap_report_threshold = max(
                        0.2,
                        2.0 * node.color_interval_ms / 1000.0
                        if node.color_interval_ms else 0.2,
                    )
                    if (
                        now - last_frame_received >= gap_report_threshold
                        and now - last_source_heartbeat >= 0.5
                    ):
                        camera_rx = bool(node.last_color_rx and now - node.last_color_rx <= 1.0)
                        node.publish_pulse_status(
                            False,
                            "NO_SYNCED_PAIR" if camera_rx else "NO_CAMERA_FRAME",
                            event_type="source_heartbeat",
                            camera_rx=camera_rx,
                            depth_rx=bool(node.last_depth_rx and now - node.last_depth_rx <= 1.0),
                            camera_rate_hz=(
                                1000.0 / node.color_interval_ms
                                if node.color_interval_ms else 0.0
                            ),
                            sync_drops_total=node.sync_drops_total,
                            max_rgb_depth_time_diff_ms=(
                                node.maximum_delta_ns / 1_000_000.0
                            ),
                            delta_rgb_depth_ms=node.last_rgb_depth_delta_ms,
                            sync_reject_reason=node.last_sync_reject_reason,
                            last_color_age_ms=(
                                (now - node.last_color_rx) * 1000.0
                                if node.last_color_rx else None
                            ),
                            last_depth_age_ms=(
                                (now - node.last_depth_rx) * 1000.0
                                if node.last_depth_rx else None
                            ),
                            publish_source_target=False,
                            **pulse_filter.diagnostics(now),
                        )
                        gap = pulse_filter.diagnostics(now)
                        if gap["stable_latched"]:
                            node.get_logger().info(
                                "TARGET GAP "
                                f"age={gap['last_valid_target_age_ms']:.0f}ms "
                                f"grace={gap['dropout_grace_ms']:.0f}ms "
                                f"missing_frames_est={gap['missing_frames_est']} "
                                f"stable_latched=YES action={gap['gap_action']}"
                            )
                        last_source_heartbeat = now
                    if now - last_notice >= 5.0:
                        print("等待同步的彩色图和深度图……")
                        last_notice = now
                    continue

                color_message, depth_message = pair
                rgb_stamp_ns = _stamp_ns(color_message)
                depth_stamp_ns = _stamp_ns(depth_message)
                camera_info_stamp_ns = (
                    _stamp_ns(node.camera_info) if node.camera_info is not None else 0
                )
                delta_rgb_depth_ms = abs(rgb_stamp_ns - depth_stamp_ns) / 1_000_000.0
                frame_received = time.monotonic()
                frame_interval_ms = (
                    (frame_received - last_frame_received) * 1000.0
                    if last_frame_received else None
                )
                last_frame_received = frame_received
                last_source_heartbeat = frame_received
                current_encodings = (color_message.encoding, depth_message.encoding)
                if current_encodings != encoding_notice:
                    print(
                        f"图像编码：color={color_message.encoding}, "
                        f"depth={depth_message.encoding}"
                    )
                    encoding_notice = current_encodings

                frame = color_message_to_bgr(color_message)
                depth_m = depth_message_to_meters(depth_message)
                if depth_m.shape != frame.shape[:2]:
                    node.get_logger().warning(
                        "深度和彩色分辨率不一致；请确认 depth_registration:=true。"
                    )
                    depth_m = cv2.resize(
                        depth_m, (frame.shape[1], frame.shape[0]),
                        interpolation=cv2.INTER_NEAREST,
                    )

                if args.mirror and not args.no_mirror:
                    frame = cv2.flip(frame, 1)
                    depth_m = cv2.flip(depth_m, 1)

                timestamp_ms = int((time.monotonic() - start_time) * 1000)
                result = detector.detect(frame, timestamp_ms)
                gestures = [
                    classify_gesture(landmarks)
                    for landmarks in result.hand_landmarks
                ]
                gesture = gestures[0] if len(gestures) == 1 else "unknown"
                stable_gesture = gesture_filter.update(gesture)
                if stable_gesture is not None:
                    node.publish_gesture(stable_gesture)
                    node.get_logger().info(f"stable gesture: {stable_gesture}")
                mask, observations = estimate_arm_contours(
                    frame, depth_m, result.hand_landmarks, args.depth_tolerance
                )

                candidate = None
                pulse_reason = "NO_DETECTION"
                pulse_details = {
                    "raw_pixel_uv": None, "raw_depth": None, "camera_xyz": None,
                }
                intrinsics = (
                    CameraIntrinsics.from_camera_info(node.camera_info)
                    if node.camera_info is not None else None
                )
                if intrinsics is None:
                    pulse_reason = "INVALID_CAMERA_INFO"
                elif len(result.hand_landmarks) == 1:
                    candidate, pulse_reason, estimate_details = estimate_pulse_region_diagnostic(
                        result.hand_landmarks[0], depth_m, intrinsics,
                        mirrored=args.mirror and not args.no_mirror,
                    )
                    pulse_details.update(estimate_details)
                    if candidate is not None:
                        pulse_reason = "FILTER_STABILIZING"
                elif len(result.hand_landmarks) > 1:
                    pulse_reason = "MULTIPLE_DETECTIONS"
                hand_identity = None
                hand_confidence = None
                if len(result.hand_landmarks) == 1 and result.handedness and result.handedness[0]:
                    handedness = result.handedness[0][0]
                    hand_confidence = float(handedness.score or 0.0)
                    if float(handedness.score or 0.0) >= 0.8:
                        hand_identity = handedness.category_name or None
                arm_direction_estimate = select_forearm_axis(
                    (
                        result.hand_landmarks[0]
                        if len(result.hand_landmarks) == 1 else None
                    ),
                    mask,
                    depth_m,
                    intrinsics,
                    arm_direction_selector,
                    mirrored=args.mirror and not args.no_mirror,
                )
                primary_degraded_temporal_diagnostics = (
                    update_primary_degraded_temporal_shadow(
                        primary_degraded_temporal_state,
                        arm_direction_estimate,
                    )
                )
                wrist_geometry_estimate = estimate_forearm_axis_wrist_geometry(
                    (
                        result.hand_landmarks[0]
                        if len(result.hand_landmarks) == 1 else None
                    ),
                    mask,
                    depth_m,
                    intrinsics,
                    mirrored=args.mirror and not args.no_mirror,
                )
                wrist_geometry_diagnostics = wrist_geometry_estimate.diagnostics()
                wrist_geometry_mask_estimate = estimate_wrist_geometry_mask(
                    (
                        result.hand_landmarks[0][0]
                        if len(result.hand_landmarks) == 1 else None
                    ),
                    frame,
                    depth_m,
                    node.camera_info,
                    mirrored=args.mirror and not args.no_mirror,
                )
                previous_active_track = wrist_geometry_branch_tracker.tracks.get(
                    wrist_geometry_branch_tracker.active_track_id
                )
                previous_active_descriptor = (
                    previous_active_track.descriptor
                    if previous_active_track is not None else None
                )
                wrist_geometry_v2_1_estimate = estimate_forearm_axis_wrist_geometry(
                    (
                        result.hand_landmarks[0]
                        if len(result.hand_landmarks) == 1 else None
                    ),
                    wrist_geometry_mask_estimate.mask,
                    depth_m,
                    intrinsics,
                    mirrored=args.mirror and not args.no_mirror,
                    previous_active_descriptor=previous_active_descriptor,
                )
                branch_parent_topology_diagnostics = (
                    wrist_geometry_v2_1_estimate.parent_topology_shadow
                    or {
                        "candidate_count": 0,
                        "parent_component_count": 0,
                        "branches": [],
                        "classification_distribution": {
                            "FOREARM_LIKE": 0,
                            "PALM_LIKE": 0,
                            "MIXED_STRUCTURE": 0,
                        },
                        "previous_active_available": (
                            previous_active_descriptor is not None
                        ),
                        "shadow_only": True,
                    }
                )
                parent_split_shadow_diagnostics = (
                    wrist_geometry_v2_1_estimate.parent_split_shadow
                    or {
                        "triggered_parent_count": 0,
                        "parents": [],
                        "classification_distribution": {
                            "FOREARM_RECOVERABLE": 0,
                            "PALM_ONLY": 0,
                            "MIXED_UNRESOLVED": 0,
                            "INSUFFICIENT": 0,
                        },
                        "previous_active_available": (
                            previous_active_descriptor is not None
                        ),
                        "shadow_only": True,
                    }
                )
                wrist_geometry_v2_2_tracking = update_wrist_geometry_branch_tracks(
                    wrist_geometry_v2_1_estimate,
                    wrist_geometry_branch_tracker,
                    timestamp_sec=frame_received,
                )
                # Run the recovery experiment only after the production V2.2
                # tracker has consumed the unmodified production estimate.
                # No recovery object is passed back into that chain.
                wrist_component_runtime_environment = {
                    "rgb_shape": list(frame.shape),
                    "depth_shape": list(depth_m.shape),
                    "rgb_encoding": str(color_message.encoding),
                    "depth_encoding": str(depth_message.encoding),
                    "mirrored": bool(args.mirror and not args.no_mirror),
                    "depth_tolerance_m": float(args.depth_tolerance),
                    "wrist_physical_roi_radius_m": 0.25,
                    "color_topic": str(args.color_topic),
                    "depth_topic": str(args.depth_topic),
                    "camera_info_topic": str(args.camera_info_topic),
                    "source_identity": runtime_source_identity,
                }
                if args.enable_wrist_component_recovery_shadow:
                    wrist_component_recovery_started = time.perf_counter()
                    wrist_component_recovery_estimate = (
                        estimate_wrist_component_recovery_shadow(
                            (
                                result.hand_landmarks[0][0]
                                if len(result.hand_landmarks) == 1 else None
                            ),
                            frame,
                            depth_m,
                            node.camera_info,
                            wrist_geometry_mask_estimate,
                            mirrored=args.mirror and not args.no_mirror,
                        )
                    )
                    wrist_component_recovery_axis_estimate = (
                        estimate_forearm_axis_wrist_geometry(
                            (
                                result.hand_landmarks[0]
                                if len(result.hand_landmarks) == 1 else None
                            ),
                            wrist_component_recovery_estimate.mask,
                            depth_m,
                            intrinsics,
                            mirrored=args.mirror and not args.no_mirror,
                            wrist_search_radius_px=32,
                            previous_active_descriptor=previous_active_descriptor,
                        )
                    )
                    wrist_component_recovery_diagnostics = add_branch_comparison(
                        wrist_component_recovery_estimate.diagnostics(),
                        wrist_geometry_v2_1_estimate,
                        wrist_component_recovery_axis_estimate,
                        (
                            previous_active_descriptor.axis
                            if previous_active_descriptor is not None else None
                        ),
                    )
                    wrist_component_recovery_diagnostics["total_elapsed_ms"] = (
                        (time.perf_counter() - wrist_component_recovery_started)
                        * 1000.0
                    )
                    wrist_component_recovery_diagnostics["enabled"] = True
                else:
                    wrist_component_recovery_diagnostics = {
                        "shadow_only": True,
                        "enabled": False,
                        "reason": "DISABLED_TO_PRESERVE_RUNTIME_CADENCE",
                    }
                wrist_component_recovery_diagnostics["runtime_environment"] = (
                    wrist_component_runtime_environment
                )
                branch_candidate_shadow_diagnostics = (
                    update_branch_candidate_shadow(
                        branch_candidate_shadow_state,
                        wrist_geometry_v2_1_estimate,
                        wrist_geometry_v2_2_tracking,
                        wrist_geometry_mask_estimate,
                    )
                )
                branch_local_recovery_diagnostics = (
                    update_branch_local_recovery_shadow(
                        branch_local_recovery_shadow_state,
                        wrist_geometry_v2_1_estimate,
                        wrist_geometry_v2_2_tracking,
                    )
                )
                wrist_input_robustness_diagnostics = (
                    update_wrist_input_robustness_shadow(
                        wrist_input_robustness_state,
                        (
                            result.hand_landmarks[0][0]
                            if len(result.hand_landmarks) == 1 else None
                        ),
                        depth_m,
                        intrinsics,
                        wrist_geometry_mask_estimate,
                        wrist_geometry_v2_1_estimate,
                        wrist_geometry_v2_2_tracking,
                        frame_received,
                        mirrored=args.mirror and not args.no_mirror,
                    )
                )
                invalid_input_shadow_diagnostics = update_invalid_input_shadow(
                    invalid_input_shadow_state,
                    result.hand_landmarks,
                    hand_confidence,
                    depth_m,
                    wrist_geometry_mask_estimate,
                    wrist_geometry_v2_1_estimate,
                    wrist_geometry_v2_2_tracking,
                    wrist_input_robustness_diagnostics,
                    branch_candidate_shadow_diagnostics,
                    frame_index=frame_count + 1,
                )
                mediapipe_palm_axis = (
                    arm_direction_estimate.mediapipe_direction.vector
                    if arm_direction_estimate.mediapipe_direction is not None
                    else None
                )
                v2_1_axis = (
                    wrist_geometry_v2_1_estimate.direction.vector
                    if wrist_geometry_v2_1_estimate.direction is not None
                    else None
                )
                v1_1_overlay_primary_axis = (
                    arm_direction_estimate.primary_direction.vector
                    if arm_direction_estimate.primary_direction is not None
                    else None
                )
                v2_2_track_id = wrist_geometry_v2_2_tracking.active_track_id
                v2_2_track = (
                    wrist_geometry_branch_tracker.tracks.get(v2_2_track_id)
                    if v2_2_track_id is not None else None
                )
                v2_2_track_axis = (
                    v2_2_track.descriptor.axis
                    if v2_2_track is not None else None
                )
                v2_1_axis_norm = (
                    float(np.linalg.norm(v2_1_axis))
                    if v2_1_axis is not None else None
                )
                v2_1_overlay_status = (
                    "OK"
                    if v2_1_axis is not None
                    else wrist_geometry_v2_1_estimate.reason
                )
                v2_2_track_confidence = (
                    float(v2_2_track.confidence_v2)
                    if v2_2_track is not None else None
                )
                v2_2_track_confirmed = bool(
                    v2_2_track is not None
                    and v2_2_track.consecutive_hits
                    >= wrist_geometry_branch_tracker.minimum_confirmation_frames
                )
                if v2_2_track is not None and v2_2_track_axis is not None:
                    if wrist_geometry_v2_2_tracking.active_state == "ACTIVE_MISSING":
                        v2_2_overlay_status = "ACTIVE_MISSING"
                    elif (v2_2_track_confidence or 0.0) < 0.35:
                        # Display-only threshold; it does not gate tracking.
                        v2_2_overlay_status = "LOW_CONFIDENCE"
                    else:
                        v2_2_overlay_status = "OK"
                elif wrist_geometry_v2_2_tracking.candidate_track_id is not None:
                    v2_2_overlay_status = "NOT_CONFIRMED"
                else:
                    v2_2_overlay_status = "NO_TRACK"
                v2_2_associated_branch = next(
                    (
                        branch for branch in wrist_geometry_v2_2_tracking.branches
                        if v2_2_track_id is not None
                        and branch.track_id == v2_2_track_id
                    ),
                    None,
                )
                v2_2_association_axis = (
                    v2_2_associated_branch.features.descriptor.axis
                    if v2_2_associated_branch is not None
                    and v2_2_associated_branch.features is not None
                    else None
                )
                axis_overlay_angles = {
                    "mediapipe_v2_1": unsigned_axis_angle_deg(
                        mediapipe_palm_axis, v2_1_axis
                    ),
                    "mediapipe_v2_2": unsigned_axis_angle_deg(
                        mediapipe_palm_axis, v2_2_track_axis
                    ),
                    "v2_1_v2_2": unsigned_axis_angle_deg(
                        v2_1_axis, v2_2_track_axis
                    ),
                }
                axis_overlay_diagnostics = {
                    "mediapipe_palm_axis": mediapipe_palm_axis,
                    "v2_1_axis": v2_1_axis,
                    "v2_1_axis_norm": v2_1_axis_norm,
                    "v2_1_status": v2_1_overlay_status,
                    "v2_2_track_axis": v2_2_track_axis,
                    "v2_2_track_id": v2_2_track_id,
                    "v2_2_track_confidence": v2_2_track_confidence,
                    "v2_2_track_confirmed": v2_2_track_confirmed,
                    "v2_2_status": v2_2_overlay_status,
                    "v2_2_track_state": wrist_geometry_v2_2_tracking.active_state,
                    "unsigned_angles_deg": axis_overlay_angles,
                }
                old_mask_boolean = mask > 0
                new_mask_boolean = wrist_geometry_mask_estimate.mask > 0
                mask_intersection = int(np.count_nonzero(
                    old_mask_boolean & new_mask_boolean
                ))
                mask_union = int(np.count_nonzero(
                    old_mask_boolean | new_mask_boolean
                ))
                mask_symmetric_difference = int(np.count_nonzero(
                    old_mask_boolean ^ new_mask_boolean
                ))
                old_v2_diagnostics = wrist_geometry_estimate.diagnostics()
                new_v2_diagnostics = wrist_geometry_v2_1_estimate.diagnostics()
                old_new_axis_angle = None
                if (
                    wrist_geometry_estimate.direction is not None
                    and wrist_geometry_v2_1_estimate.direction is not None
                ):
                    old_axis = np.asarray(
                        wrist_geometry_estimate.direction.vector, dtype=np.float64
                    )
                    new_axis = np.asarray(
                        wrist_geometry_v2_1_estimate.direction.vector,
                        dtype=np.float64,
                    )
                    cosine = abs(float(np.dot(old_axis, new_axis))) / (
                        float(np.linalg.norm(old_axis))
                        * float(np.linalg.norm(new_axis))
                    )
                    old_new_axis_angle = float(np.degrees(
                        np.arccos(np.clip(cosine, 0.0, 1.0))
                    ))
                wrist_geometry_mask_diagnostics = (
                    wrist_geometry_mask_estimate.diagnostics()
                )
                wrist_geometry_mask_diagnostics.update({
                    "old_mask_area_px": int(np.count_nonzero(old_mask_boolean)),
                    "new_mask_area_px": int(np.count_nonzero(new_mask_boolean)),
                    "intersection_px": mask_intersection,
                    "union_px": mask_union,
                    "symmetric_difference_px": mask_symmetric_difference,
                    "iou": (
                        float(mask_intersection / mask_union)
                        if mask_union > 0 else None
                    ),
                    # Multiple post-wrist branches are only a geometry proxy;
                    # with wrist-only input they cannot identify a palm as
                    # anatomical ground truth.
                    "possible_palm_region_from_multiple_branches": (
                        wrist_geometry_v2_1_estimate.branch_count > 1
                    ),
                })
                wrist_geometry_v2_1_diagnostics = {
                    "mask": wrist_geometry_mask_diagnostics,
                    "old_mask_v2": old_v2_diagnostics,
                    "new_mask_v2": new_v2_diagnostics,
                    "comparison": {
                        "axis_unsigned_angle_deg": old_new_axis_angle,
                        "rms_delta_m": (
                            wrist_geometry_v2_1_estimate.rms_m
                            - wrist_geometry_estimate.rms_m
                            if wrist_geometry_v2_1_estimate.rms_m is not None
                            and wrist_geometry_estimate.rms_m is not None
                            else None
                        ),
                        "branch_count_delta": (
                            wrist_geometry_v2_1_estimate.branch_count
                            - wrist_geometry_estimate.branch_count
                        ),
                        "old_quality": wrist_geometry_estimate.quality,
                        "new_quality": wrist_geometry_v2_1_estimate.quality,
                    },
                }
                v1_primary_axis = arm_direction_estimate.primary_direction
                wrist_geometry_diagnostics["v1_1_axis"] = (
                    v1_primary_axis.vector if v1_primary_axis is not None else None
                )
                if (
                    wrist_geometry_estimate.direction is not None
                    and v1_primary_axis is not None
                ):
                    v2_vector = np.asarray(
                        wrist_geometry_estimate.direction.vector, dtype=np.float64
                    )
                    v1_vector = np.asarray(v1_primary_axis.vector, dtype=np.float64)
                    cosine = abs(float(np.dot(v2_vector, v1_vector))) / (
                        float(np.linalg.norm(v2_vector))
                        * float(np.linalg.norm(v1_vector))
                    )
                    wrist_geometry_diagnostics["v1_v2_unsigned_angle"] = (
                        float(np.degrees(np.arccos(np.clip(cosine, 0.0, 1.0))))
                    )
                else:
                    wrist_geometry_diagnostics["v1_v2_unsigned_angle"] = None
                arm_direction_filter_history_before = len(
                    arm_direction_filter._samples
                )
                arm_direction_filter_update_called = (
                    arm_direction_estimate.direction is not None
                )
                arm_direction_filter_input_empty = (
                    arm_direction_estimate.direction is None
                )
                arm_direction_filter_clear_called = bool(
                    arm_direction_estimate.source_changed
                )
                if arm_direction_estimate.source_changed:
                    arm_direction_filter.clear()
                stable_direction = (
                    arm_direction_filter.update(arm_direction_estimate.direction)
                    if arm_direction_estimate.direction is not None
                    else None
                )
                stable_direction_axis = (
                    stable_direction.vector
                    if stable_direction is not None else None
                )
                arm_direction_filter_history_size = len(
                    arm_direction_filter._samples
                )
                arm_direction_filter_window_size = int(
                    arm_direction_filter._samples.maxlen
                )
                arm_direction_filter_history_full = (
                    arm_direction_filter_history_size
                    >= arm_direction_filter_window_size
                )
                if arm_direction_filter_clear_called:
                    arm_direction_filter_clear_reason = (
                        "SOURCE_CHANGED_TO_INVALID"
                        if arm_direction_estimate.active_source == "INVALID"
                        else "SOURCE_CHANGED"
                    )
                else:
                    arm_direction_filter_clear_reason = "NONE"
                (
                    forearm_direction_overlay_axis,
                    forearm_direction_overlay_source,
                    forearm_direction_overlay_reason,
                ) = select_forearm_direction_overlay(stable_direction_axis)
                robot_direction_available = stable_direction is not None
                if robot_direction_available:
                    arm_direction_filter_state = "STABLE"
                elif arm_direction_estimate.direction is None:
                    arm_direction_filter_state = "NO_INPUT"
                else:
                    arm_direction_filter_state = "STABILIZING"
                diagnostic_candidate_axes = {
                    "V2.2": (
                        [float(value) for value in v2_2_association_axis]
                        if v2_2_association_axis is not None else None
                    ),
                    "V2.1": (
                        [float(value) for value in v2_1_axis]
                        if v2_1_axis is not None else None
                    ),
                    "V1.1": (
                        [float(value) for value in v1_1_overlay_primary_axis]
                        if v1_1_overlay_primary_axis is not None else None
                    ),
                }
                candidate_direction_axis = (
                    arm_direction_estimate.direction.vector
                    if arm_direction_estimate.direction is not None else None
                )
                primary_degraded_keep_diagnostics = (
                    update_primary_degraded_keep_shadow(
                        primary_degraded_keep_state,
                        arm_direction_estimate,
                        primary_degraded_temporal_diagnostics,
                        stable_direction,
                        v2_2_axis=v2_2_track_axis,
                    )
                )
                selector_fallback_strategy_diagnostics = (
                    update_selector_fallback_strategy_shadow(
                        selector_fallback_strategy_shadow_state,
                        arm_direction_estimate,
                        primary_degraded_temporal_diagnostics,
                        frame_received,
                        v2_2_axis=v2_2_track_axis,
                        gesture=gesture,
                    )
                )
                (
                    vision_forearm_axis,
                    vision_forearm_source,
                ) = select_vision_forearm_axis(v2_2_track_axis, v2_1_axis)
                b_plus_overlay_variants = (
                    selector_fallback_strategy_diagnostics.get(
                        "Strategy_B_plus", {}
                    )
                )
                experiment_phase = (
                    selector_fallback_strategy_diagnostics.get(
                        "experiment_phase_shadow", {}
                    ).get("phase", "UNLABELED")
                )
                persistent_forearm_validity_diagnostics = (
                    update_persistent_forearm_validity_shadow(
                        persistent_forearm_validity_shadow_state,
                        wrist_geometry_v2_1_estimate,
                        wrist_geometry_v2_2_tracking,
                        wrist_geometry_branch_tracker,
                        wrist_geometry_mask_estimate,
                        frame_received,
                        rgb_stamp_ns=rgb_stamp_ns,
                        depth_stamp_ns=depth_stamp_ns,
                        frame_seq=frame_count + 1,
                        detection_valid=len(result.hand_landmarks) == 1,
                        v1_primary_axis=v1_1_overlay_primary_axis,
                        actual_selector_axis=candidate_direction_axis,
                        experiment_phase=experiment_phase,
                    )
                )
                normal_candidate = None
                surface_patch = None
                surface_details = {
                    "plane_inlier_count": 0,
                    "plane_residual": None,
                    "normal_confidence": 0.0,
                    "depth_valid_ratio": 0.0,
                    "normal_angle": None,
                }
                # Board observations remain on their dedicated calibration
                # topics only; they never feed the hand target or approach.
                board_normal = None
                if intrinsics is not None:
                    board_normal = estimate_calibration_board_normal(
                        frame,
                        intrinsics,
                        mirrored=args.mirror and not args.no_mirror,
                        distortion_coefficients=node.camera_info.d,
                        projection_matrix=node.camera_info.p,
                        rectification_matrix=node.camera_info.r,
                    )
                    if board_normal is not None:
                        node.publish_calibration_board_target(
                            board_normal, color_message.header
                        )
                if candidate is not None and intrinsics is not None:
                    normal_radius_px = int(np.clip(
                        candidate.radius_m * intrinsics.fx / max(candidate.point_m[2], 1e-6),
                        8, 40,
                    ))
                    surface_patch, surface_reason, surface_details = (
                        estimate_surface_patch_diagnostic(
                            depth_m, candidate.pixel, intrinsics,
                            mirrored=args.mirror and not args.no_mirror,
                            radius_px=normal_radius_px,
                            valid_mask=mask,
                            reference_depth_m=candidate.point_m[2],
                        )
                    )
                    if surface_patch is None:
                        candidate = None
                        pulse_reason = surface_reason
                    else:
                        # Preserve the wrist/pulse pixel and radius definition;
                        # replace only its 3D point with the fitted skin plane.
                        candidate = type(candidate)(
                            pixel=candidate.pixel,
                            point_m=surface_patch.point_m,
                            radius_m=candidate.radius_m,
                        )
                        pulse_details["camera_xyz"] = surface_patch.point_m
                        pulse_details["raw_depth"] = surface_patch.point_m[2]
                        normal_candidate = surface_patch.normal
                stable_normal = surface_normal_filter.update(normal_candidate)
                stable_region = pulse_filter.update(candidate, frame_received, hand_identity)
                filter_diagnostics = pulse_filter.diagnostics(frame_received)
                if (
                    filter_diagnostics["target_stable"]
                    and frame_received - last_stable_status_log >= 0.5
                ):
                    node.get_logger().info(
                        "TARGET STABLE "
                        f"jitter={filter_diagnostics['target_jitter_mm']:.1f}/"
                        f"{args.target_stable_radius_m * 1000.0:.1f}mm "
                        f"motion={filter_diagnostics['target_motion_mm']:.1f}mm "
                        f"age={filter_diagnostics['target_age_ms']:.0f}ms "
                        f"acquisition={filter_diagnostics['acquisition_state']}"
                    )
                    last_stable_status_log = frame_received
                if filter_diagnostics["reacquire_result"] == "REACQUIRED_SAME_TARGET":
                    node.get_logger().info(
                        "TARGET REACQUIRED "
                        f"gap={pulse_filter.last_gap_sec * 1000.0:.0f}ms "
                        f"distance_to_frozen={filter_diagnostics['reacquire_distance_mm']:.1f}mm "
                        "result=REACQUIRED_SAME_TARGET stable_reset=NO"
                    )
                cycle_diagnostics = {
                    "frame_seq": frame_count + 1,
                    "frame_rx_time": frame_received,
                    "frame_interval_ms": frame_interval_ms,
                    "frame_stamp_sec": int(color_message.header.stamp.sec),
                    "frame_stamp_nanosec": int(color_message.header.stamp.nanosec),
                    "rgb_stamp": rgb_stamp_ns / 1_000_000_000.0,
                    "depth_stamp": depth_stamp_ns / 1_000_000_000.0,
                    "camera_info_stamp": (
                        camera_info_stamp_ns / 1_000_000_000.0
                        if camera_info_stamp_ns > 0 else None
                    ),
                    "rgb_stamp_ns": rgb_stamp_ns,
                    "depth_stamp_ns": depth_stamp_ns,
                    "camera_info_stamp_ns": (
                        camera_info_stamp_ns if camera_info_stamp_ns > 0 else None
                    ),
                    "delta_rgb_depth_ms": delta_rgb_depth_ms,
                    "max_rgb_depth_time_diff_ms": (
                        node.maximum_delta_ns / 1_000_000.0
                    ),
                    "detection_present": bool(result.hand_landmarks),
                    "detection_valid": len(result.hand_landmarks) == 1,
                    "detection_3d_valid": candidate is not None,
                    "detections_count": len(result.hand_landmarks),
                    "target_valid_before_tf": (
                        stable_region is not None and stable_normal is not None
                    ),
                    "hand_identity": hand_identity,
                    "arm_direction_diagnostics": (
                        arm_direction_estimate.diagnostics()
                    ),
                    "primary_degraded_temporal_shadow": (
                        primary_degraded_temporal_diagnostics
                    ),
                    "primary_degraded_keep_shadow": (
                        primary_degraded_keep_diagnostics
                    ),
                    "selector_fallback_strategy_shadow": (
                        selector_fallback_strategy_diagnostics
                    ),
                    "wrist_geometry_diagnostics": wrist_geometry_diagnostics,
                    "wrist_geometry_v2_1_diagnostics": (
                        wrist_geometry_v2_1_diagnostics
                    ),
                    "wrist_geometry_v2_2_diagnostics": (
                        wrist_geometry_v2_2_tracking.diagnostics()
                    ),
                    "v2_2_persistent_validity_shadow": (
                        persistent_forearm_validity_diagnostics
                    ),
                    "wrist_component_recovery_shadow": (
                        wrist_component_recovery_diagnostics
                    ),
                    "axis_overlay_diagnostics": axis_overlay_diagnostics,
                    "wrist_input_robustness_shadow": (
                        wrist_input_robustness_diagnostics
                    ),
                    "invalid_input_shadow": invalid_input_shadow_diagnostics,
                    "branch_candidate_shadow": (
                        branch_candidate_shadow_diagnostics
                    ),
                    "branch_local_recovery_shadow": (
                        branch_local_recovery_diagnostics
                    ),
                    "branch_parent_topology_shadow": (
                        branch_parent_topology_diagnostics
                    ),
                    "parent_split_shadow": parent_split_shadow_diagnostics,
                    "arm_direction_source_switch_count": (
                        arm_direction_selector.source_switch_count
                    ),
                    "arm_direction_primary_success_streak": (
                        arm_direction_selector.primary_success_streak
                    ),
                    "arm_direction_primary_failure_streak": (
                        arm_direction_selector.primary_failure_streak
                    ),
                    "arm_direction_filter_cleared": (
                        arm_direction_estimate.source_changed
                    ),
                    "surface_normal_valid": stable_normal is not None,
                    "surface_normal_camera": (
                        stable_normal.vector if stable_normal is not None else None
                    ),
                    "surface_normal_source": args.surface_normal_source,
                    "calibration_board_type": (
                        board_normal.board_type if board_normal is not None else None
                    ),
                    "calibration_board_reprojection_rms_px": (
                        board_normal.reprojection_rms_px
                        if board_normal is not None else None
                    ),
                    "surface_fit_rms_mm": (
                        stable_normal.rms_m * 1000.0
                        if stable_normal is not None else None
                    ),
                    "surface_fit_point_count": (
                        stable_normal.point_count if stable_normal is not None else 0
                    ),
                    **surface_details,
                    **filter_diagnostics,
                    **pulse_details,
                }
                # Publish the point and normal as one quality-gated pair. A
                # stale normal must never be combined with a newly sampled
                # point after a failed fit.
                if stable_region is not None and stable_normal is not None:
                    node.publish_pulse(
                        stable_region, color_message.header, **cycle_diagnostics
                    )
                    if stable_direction is not None:
                        node.publish_arm_direction(stable_direction, color_message.header)
                    node.publish_surface_normal(stable_normal, color_message.header)
                    node.get_logger().info(
                        "HAND_SURFACE "
                        f"surface_point_camera={stable_region.point_m} "
                        f"surface_normal_camera={stable_normal.vector} "
                        f"normal_angle={surface_details['normal_angle']:.2f}deg "
                        f"plane_inlier_count={surface_details['plane_inlier_count']} "
                        f"plane_residual={surface_details['plane_residual']:.6f}m "
                        f"normal_confidence={surface_details['normal_confidence']:.3f} "
                        f"depth_valid_ratio={surface_details['depth_valid_ratio']:.3f}"
                    )
                    pulse_reason = "stable"
                else:
                    node.publish_pulse_status(
                        False, pulse_reason, publish_source_target=False,
                        **cycle_diagnostics,
                    )

                rgb_panel = frame.copy()
                draw_hand_landmarks(rgb_panel, result)
                cv2.putText(
                    rgb_panel,
                    f"Gesture: {gesture}",
                    (12, 66),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                for observation in observations:
                    cv2.drawContours(
                        rgb_panel, [observation.contour], -1, (255, 0, 255), 3,
                        cv2.LINE_AA,
                    )
                    depth_text = (
                        f"{observation.wrist_depth_m:.3f} m"
                        if observation.wrist_depth_m is not None else "depth N/A"
                    )
                    cv2.putText(
                        rgb_panel, depth_text,
                        (observation.wrist[0] + 10, observation.wrist[1] + 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2,
                        cv2.LINE_AA,
                    )

                forearm_overlay_draw_diagnostics = {}
                candidate_overlay_draw_diagnostics = {}
                # Keep the existing runtime diagnostics, but draw their probe
                # pass off-screen. The visible RGB panel is rendered exactly
                # once at the end with all four direction semantics.
                overlay_probe_panel = np.zeros_like(rgb_panel)
                candidate_overlay_draw_success = (
                    draw_candidate_forearm_axis_overlay(
                        overlay_probe_panel,
                        wrist_geometry_mask_estimate.wrist_seed_point_m,
                        intrinsics,
                        candidate_direction_axis,
                        arm_direction_estimate.active_source,
                        mirrored=args.mirror and not args.no_mirror,
                        diagnostics=candidate_overlay_draw_diagnostics,
                    )
                )
                forearm_overlay_draw_success = draw_forearm_direction_overlay(
                    overlay_probe_panel,
                    wrist_geometry_mask_estimate.wrist_seed_point_m,
                    intrinsics,
                    forearm_direction_overlay_axis,
                    forearm_direction_overlay_source,
                    forearm_direction_overlay_reason,
                    mirrored=args.mirror and not args.no_mirror,
                    diagnostics=forearm_overlay_draw_diagnostics,
                    robot_direction_available=robot_direction_available,
                    filter_state=arm_direction_filter_state,
                    active_source=arm_direction_estimate.active_source,
                    diagnostic_candidate_axes=diagnostic_candidate_axes,
                )
                force_arrow_diagnostics = {"enabled": False}
                force_arrow_draw_success = False
                if args.debug_force_forearm_arrow:
                    force_arrow_draw_success = draw_debug_force_forearm_arrow(
                        rgb_panel, diagnostics=force_arrow_diagnostics
                    )
                if frame_received - last_forearm_overlay_log >= 0.5:
                    v2_2_axis_available = (
                        unsigned_axis_angle_deg(
                            v2_2_association_axis, v2_2_association_axis
                        ) is not None
                    )
                    v2_1_axis_available = (
                        unsigned_axis_angle_deg(v2_1_axis, v2_1_axis) is not None
                    )
                    v1_1_primary_axis_available = (
                        unsigned_axis_angle_deg(
                            v1_1_overlay_primary_axis,
                            v1_1_overlay_primary_axis,
                        ) is not None
                    )
                    selected_axis_vector = (
                        [float(value) for value in forearm_direction_overlay_axis]
                        if forearm_direction_overlay_axis is not None else None
                    )
                    selected_axis_norm = (
                        float(np.linalg.norm(forearm_direction_overlay_axis))
                        if forearm_direction_overlay_axis is not None else None
                    )
                    loaded_module_path = str(
                        Path(axis_overlay_module.__file__).resolve()
                    )
                    loaded_expected_module = (
                        Path(axis_overlay_module.__file__).resolve().parent
                        == Path(__file__).resolve().parent
                    )
                    expected_semantics_version = (
                        "ROBOT_STABLE_CANDIDATE_RAW_V2"
                    )
                    loaded_semantics_version = getattr(
                        axis_overlay_module,
                        "FOREARM_DIRECTION_OVERLAY_SEMANTICS",
                        None,
                    )
                    loaded_expected_semantics = (
                        loaded_semantics_version == expected_semantics_version
                    )
                    stable_direction_available = (
                        unsigned_axis_angle_deg(
                            stable_direction_axis, stable_direction_axis
                        ) is not None
                    )
                    estimate_direction_available = (
                        arm_direction_estimate.direction is not None
                    )
                    primary_axis_available = (
                        arm_direction_estimate.primary_direction is not None
                    )
                    if (
                        not loaded_expected_module
                        or not loaded_expected_semantics
                    ):
                        runtime_diagnosis = "D_RUNNING_OLD_VERSION"
                    elif not stable_direction_available and not (
                        estimate_direction_available
                    ):
                        runtime_diagnosis = "A_STABLE_DIRECTION_ABSENT"
                    elif not stable_direction_available:
                        runtime_diagnosis = "B_FILTER_NOT_STABLE"
                    elif not forearm_overlay_draw_success:
                        runtime_diagnosis = "C_STABLE_DIRECTION_NOT_DRAWN"
                    else:
                        runtime_diagnosis = "DRAW_OK"
                    stable_direction_runtime = {
                        "stable_direction_available": (
                            stable_direction_available
                        ),
                        "stable_direction_vector": (
                            [float(value) for value in stable_direction_axis]
                            if stable_direction_axis is not None else None
                        ),
                        "filter_state": arm_direction_filter_state,
                        "filter_history_size": (
                            arm_direction_filter_history_size
                        ),
                        "filter_ready": stable_direction_available,
                    }
                    arm_direction_estimate_runtime = {
                        "estimate_direction_available": (
                            estimate_direction_available
                        ),
                        "primary_axis_available": primary_axis_available,
                        "active_source": arm_direction_estimate.active_source,
                        "source_changed": bool(
                            arm_direction_estimate.source_changed
                        ),
                    }
                    arm_direction_filter_runtime = {
                        "update_called": arm_direction_filter_update_called,
                        "update_input_empty": arm_direction_filter_input_empty,
                        "history_size_before": (
                            arm_direction_filter_history_before
                        ),
                        "history_size_after": (
                            arm_direction_filter_history_size
                        ),
                        "required_history_size": (
                            arm_direction_filter_window_size
                        ),
                        "history_full": arm_direction_filter_history_full,
                        "filter_ready": stable_direction_available,
                        "clear_called": arm_direction_filter_clear_called,
                        "clear_reason": arm_direction_filter_clear_reason,
                        "invalid_active_source": (
                            arm_direction_estimate.active_source == "INVALID"
                        ),
                        "invalid_caused_clear": bool(
                            arm_direction_filter_clear_called
                            and arm_direction_estimate.active_source == "INVALID"
                        ),
                    }
                    overlay_draw_runtime = {
                        "stable_direction_available": (
                            stable_direction_available
                        ),
                        "draw_attempt": True,
                        "draw_success": forearm_overlay_draw_success,
                        "failure_reason": (
                            forearm_overlay_draw_diagnostics.get(
                                "draw_failure_reason"
                            )
                        ),
                    }
                    candidate_overlay_runtime = {
                        "estimate_direction_available": (
                            estimate_direction_available
                        ),
                        "candidate_direction_vector": (
                            [float(value) for value in candidate_direction_axis]
                            if candidate_direction_axis is not None else None
                        ),
                        "source": arm_direction_estimate.active_source,
                        "draw_attempt": True,
                        "draw_success": candidate_overlay_draw_success,
                        "failure_reason": (
                            candidate_overlay_draw_diagnostics.get(
                                "draw_failure_reason"
                            )
                        ),
                    }
                    overlay_runtime_diagnostics = {
                        "overlay_forearm_axis_attempt": True,
                        "stable_direction": stable_direction_runtime,
                        "arm_direction_estimate": (
                            arm_direction_estimate_runtime
                        ),
                        "stable_arm_direction_filter": (
                            arm_direction_filter_runtime
                        ),
                        "overlay": overlay_draw_runtime,
                        "candidate_overlay": candidate_overlay_runtime,
                        "v2_2_axis_available": v2_2_axis_available,
                        "v2_1_axis_available": v2_1_axis_available,
                        "v1_1_primary_axis_available": (
                            v1_1_primary_axis_available
                        ),
                        "robot_direction_available": robot_direction_available,
                        "filter_state": arm_direction_filter_state,
                        "active_source": arm_direction_estimate.active_source,
                        "diagnostic_candidate_axes": diagnostic_candidate_axes,
                        "v2_1_axis_source_status": (
                            "OK" if v2_1_axis_available
                            else "NO_V2_1_AXIS_SOURCE"
                        ),
                        "selected_axis_source": forearm_direction_overlay_source,
                        "selected_axis_vector": selected_axis_vector,
                        "selected_axis_norm": selected_axis_norm,
                        "wrist_pixel": forearm_overlay_draw_diagnostics.get(
                            "start_pixel"
                        ),
                        "projected_endpoint_pixel": (
                            forearm_overlay_draw_diagnostics.get("end_pixel")
                        ),
                        "start_pixel": forearm_overlay_draw_diagnostics.get(
                            "start_pixel"
                        ),
                        "end_pixel": forearm_overlay_draw_diagnostics.get(
                            "end_pixel"
                        ),
                        "projection_valid": forearm_overlay_draw_diagnostics.get(
                            "projection_valid", False
                        ),
                        "projection_warning": (
                            forearm_overlay_draw_diagnostics.get(
                                "projection_warning"
                            )
                        ),
                        "draw_success": forearm_overlay_draw_success,
                        "draw_failure_reason": (
                            forearm_overlay_draw_diagnostics.get(
                                "draw_failure_reason"
                            )
                        ),
                        "module_path": loaded_module_path,
                        "module_matches_viewer_package": loaded_expected_module,
                        "overlay_semantics_version": loaded_semantics_version,
                        "expected_overlay_semantics_version": (
                            expected_semantics_version
                        ),
                        "overlay_semantics_version_matches": (
                            loaded_expected_semantics
                        ),
                        "runtime_diagnosis": runtime_diagnosis,
                        "draw_order": (
                            "DIAGNOSTIC_PASS_BEFORE_BOARD_PULSE_AND_TITLE;"
                            "FINAL_REDRAW_AFTER_ALL_RGB_OVERLAYS"
                        ),
                        "color_bgr": forearm_overlay_draw_diagnostics.get(
                            "color_bgr"
                        ),
                        "line_width_px": forearm_overlay_draw_diagnostics.get(
                            "line_width_px"
                        ),
                        "arrow_length_m": forearm_overlay_draw_diagnostics.get(
                            "arrow_length_m"
                        ),
                        "alpha": 1.0,
                        "transform": "NOT_REQUIRED_CAMERA_FRAME",
                        "debug_force_forearm_arrow": force_arrow_diagnostics,
                        "debug_force_draw_success": force_arrow_draw_success,
                    }
                    node.get_logger().info(
                        "FOREARM_DIRECTION_OVERLAY "
                        + json.dumps(overlay_runtime_diagnostics, sort_keys=True)
                    )
                    last_forearm_overlay_log = frame_received

                if board_normal is not None:
                    cv2.drawMarker(
                        rgb_panel,
                        board_normal.lower_right_pixel,
                        (255, 255, 0),
                        cv2.MARKER_CROSS,
                        22,
                        3,
                        cv2.LINE_AA,
                    )
                    cv2.putText(
                        rgb_panel,
                        f"{board_normal.board_type} board target",
                        (
                            max(8, board_normal.lower_right_pixel[0] - 100),
                            max(25, board_normal.lower_right_pixel[1] - 16),
                        ),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.48,
                        (255, 255, 0),
                        2,
                        cv2.LINE_AA,
                    )

                displayed_region = stable_region or candidate
                if displayed_region is not None:
                    pulse_color = (
                        (0, 255, 0) if stable_region is not None
                        else (0, 180, 255)
                    )
                    cv2.circle(
                        rgb_panel, displayed_region.pixel, 13, pulse_color, 3,
                        cv2.LINE_AA,
                    )
                    cv2.putText(
                        rgb_panel,
                        "CAMERA PULSE READY" if stable_region is not None
                        else "CAMERA PULSE STABILIZING",
                        (max(8, displayed_region.pixel[0] - 85),
                         max(25, displayed_region.pixel[1] - 18)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, pulse_color, 2,
                        cv2.LINE_AA,
                    )

                depth_panel = colorize_depth(depth_m, args.max_depth)
                contour_panel = cv2.bitwise_and(frame, frame, mask=mask)
                cv2.drawContours(
                    contour_panel, [item.contour for item in observations],
                    -1, (255, 0, 255), 3, cv2.LINE_AA,
                )
                _title(rgb_panel, "RGB + hand landmarks + arm contour")
                _title(depth_panel, f"Registered depth (0-{args.max_depth:.1f} m)")
                _title(contour_panel, "Estimated arm contour")
                # Draw all independent direction semantics as the final RGB
                # layer. This is display-only and cannot feed any estimator,
                # selector, filter, publisher, or robot-control path.
                draw_forearm_diagnostic_overlay(
                    rgb_panel,
                    wrist_geometry_mask_estimate.wrist_seed_point_m,
                    intrinsics,
                    robot_axis=stable_direction_axis,
                    vision_axis=vision_forearm_axis,
                    vision_source=vision_forearm_source,
                    selector_axis=candidate_direction_axis,
                    selector_source=arm_direction_estimate.active_source,
                    b_plus_variants=b_plus_overlay_variants,
                    mirrored=args.mirror and not args.no_mirror,
                )
                if args.debug_force_forearm_arrow:
                    draw_debug_force_forearm_arrow(rgb_panel)

                # The RViz hand-vision tab intentionally shows only the left
                # RGB/landmark/contour panel. Depth remains an internal input
                # for contour estimation and is not rendered in this view.
                node.publish_annotated_image(rgb_panel, color_message.header)
                frame_count += 1
                elapsed = max(time.monotonic() - start_time, 1e-6)
                if not args.no_window:
                    # The optional standalone viewer follows the same layout
                    # as RViz. Depth remains an internal estimation input.
                    display_panel = rgb_panel.copy()
                    cv2.putText(
                        display_panel, f"FPS {frame_count / elapsed:.1f}",
                        (12, display_panel.shape[0] - 14), cv2.FONT_HERSHEY_SIMPLEX,
                        0.65, (0, 255, 0), 2, cv2.LINE_AA,
                    )
                    if args.display_scale != 1.0:
                        display_panel = cv2.resize(
                            display_panel, None, fx=args.display_scale,
                            fy=args.display_scale, interpolation=cv2.INTER_AREA,
                        )
                    cv2.imshow("Meridian Hand Depth Viewer", display_panel)
                    if cv2.waitKey(1) & 0xFF in (ord("q"), ord("Q"), 27):
                        break
    except KeyboardInterrupt:
        pass
    except Exception as error:
        # Never let a detector/conversion failure look like a silent target
        # drought to the downstream bridge.
        node.publish_pulse_status(
            False, "INTERNAL_EXCEPTION", publish_source_target=False,
            exception=repr(error),
        )
        node.get_logger().error(f"hand vision pipeline exception: {error!r}")
    finally:
        if not args.no_window:
            cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
