from __future__ import annotations

import argparse
import os
import json
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point, PointStamped, Vector3, Vector3Stamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from visualization_msgs.msg import Marker

from .arm_contour import estimate_arm_contours
from .gesture_classifier import StableGestureFilter, classify_gesture
from .hand_landmarker import HandLandmarker, draw_hand_landmarks
from .image_conversion import (
    color_message_to_bgr,
    colorize_depth,
    depth_message_to_meters,
)
from .pulse_region import (
    CameraIntrinsics,
    StableArmDirectionFilter,
    StablePulseFilter,
    estimate_arm_direction,
    estimate_pulse_region_diagnostic,
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
    parser.add_argument("--sync-ms", type=float, default=80.0)
    parser.add_argument("--max-depth", type=float, default=3.0)
    parser.add_argument("--depth-tolerance", type=float, default=0.18)
    parser.add_argument("--display-scale", type=float, default=0.55)
    parser.add_argument(
        "--no-window", action="store_true",
        help="仅发布标注图像供 RViz 使用，不打开独立 OpenCV 窗口",
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
    return parser.parse_args()


def _stamp_ns(message: Image) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


class RgbDepthSubscriber(Node):
    def __init__(
        self,
        color_topic: str,
        depth_topic: str,
        camera_info_topic: str,
        sync_ms: float,
        gesture_topic: str,
        annotated_topic: str,
        pulse_topic: str,
        pulse_status_topic: str,
        pulse_marker_topic: str,
        arm_direction_topic: str,
    ) -> None:
        super().__init__("meridian_hand_depth_viewer")
        self.color_message: Image | None = None
        self.depth_message: Image | None = None
        self.camera_info: CameraInfo | None = None
        self.last_color_rx = 0.0
        self.last_depth_rx = 0.0
        self.color_interval_ms: float | None = None
        self.sync_drops_total = 0
        self.maximum_delta_ns = int(sync_ms * 1_000_000)
        self.create_subscription(
            Image, color_topic, self._on_color, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, depth_topic, self._on_depth, qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo, camera_info_topic, self._on_camera_info,
            qos_profile_sensor_data,
        )
        self.gesture_publisher = self.create_publisher(String, gesture_topic, 10)
        self.annotated_image_publisher = self.create_publisher(Image, annotated_topic, 10)
        self.pulse_publisher = self.create_publisher(PointStamped, pulse_topic, 10)
        self.pulse_status_publisher = self.create_publisher(String, pulse_status_topic, 10)
        self.pulse_marker_publisher = self.create_publisher(Marker, pulse_marker_topic, 10)
        self.arm_direction_publisher = self.create_publisher(
            Vector3Stamped, arm_direction_topic, 10
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
        if color_stamp and depth_stamp:
            delta = color_stamp - depth_stamp
            if abs(delta) > self.maximum_delta_ns:
                self.sync_drops_total += 1
                if delta < 0:
                    self.color_message = None
                else:
                    self.depth_message = None
                return None
        pair = self.color_message, self.depth_message
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
    model_path = args.model.expanduser().resolve()
    if not model_path.is_file():
        raise FileNotFoundError(f"找不到 MediaPipe 模型：{model_path}")
    if args.display_scale <= 0.0:
        raise ValueError("--display-scale 必须大于 0。")

    rclpy.init()
    node = RgbDepthSubscriber(
        args.color_topic, args.depth_topic, args.camera_info_topic, args.sync_ms,
        args.gesture_topic, args.annotated_topic, args.pulse_topic,
        args.pulse_status_topic, args.pulse_marker_topic,
        args.arm_direction_topic,
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
    start_time = time.monotonic()
    last_notice = 0.0
    last_source_heartbeat = 0.0
    last_stable_status_log = 0.0
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
                if len(result.hand_landmarks) == 1 and result.handedness and result.handedness[0]:
                    handedness = result.handedness[0][0]
                    if float(handedness.score or 0.0) >= 0.8:
                        hand_identity = handedness.category_name or None
                direction_candidate = (
                    estimate_arm_direction(
                        result.hand_landmarks[0], depth_m, intrinsics,
                        mirrored=args.mirror and not args.no_mirror,
                    )
                    if intrinsics is not None and len(result.hand_landmarks) == 1
                    else None
                )
                stable_direction = arm_direction_filter.update(direction_candidate)
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
                    "detection_present": bool(result.hand_landmarks),
                    "detection_valid": len(result.hand_landmarks) == 1,
                    "detection_3d_valid": candidate is not None,
                    "detections_count": len(result.hand_landmarks),
                    "target_valid_before_tf": stable_region is not None,
                    "hand_identity": hand_identity,
                    **filter_diagnostics,
                    **pulse_details,
                }
                if stable_region is not None:
                    node.publish_pulse(
                        stable_region, color_message.header, **cycle_diagnostics
                    )
                    if stable_direction is not None:
                        node.publish_arm_direction(stable_direction, color_message.header)
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
                displayed_region = stable_region or candidate
                if displayed_region is not None:
                    pulse_color = (0, 255, 0) if stable_region is not None else (0, 180, 255)
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
