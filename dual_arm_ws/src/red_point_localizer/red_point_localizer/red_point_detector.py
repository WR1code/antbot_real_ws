#!/usr/bin/env python3
"""Detect a color target and project its aligned depth into camera 3-D."""

import csv
import math
import os
from typing import Optional

import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class RedPointDetector(Node):
    """Publish target pixels, aligned-depth camera points, and debug imagery."""

    HSV_RANGES: dict[str, tuple[tuple[int, int, int], ...]] = {
        'red': (
            (0, 100, 70),
            (10, 255, 255),
            (160, 100, 70),
            (179, 255, 255),
        ),
        'green': ((40, 80, 60), (85, 255, 255)),
        'cyan': ((85, 80, 60), (105, 255, 255)),
    }
    DISPLAY_COLORS: dict[str, tuple[int, int, int]] = {
        'red': (0, 0, 255),
        'green': (0, 255, 0),
        'cyan': (255, 255, 0),
    }

    def __init__(self) -> None:
        super().__init__('red_point_detector')

        self.image_topic = str(
            self.declare_parameter('image_topic', '/camera/color/image_raw').value
        )
        self.depth_topic = str(
            self.declare_parameter('depth_topic', '/camera/depth/image_raw').value
        )
        self.camera_info_topic = str(
            self.declare_parameter(
                'camera_info_topic', '/camera/color/camera_info'
            ).value
        )
        debug_topic = str(
            self.declare_parameter('debug_topic', '/red_point/debug_image').value
        )
        pixel_topic = str(
            self.declare_parameter('pixel_topic', '/red_point/pixel').value
        )
        camera_point_topic = str(
            self.declare_parameter(
                'camera_point_topic', '/red_point/camera_point'
            ).value
        )
        self.min_area = float(self.declare_parameter('min_area', 100.0).value)
        self.depth_window_size = max(
            1, int(self.declare_parameter('depth_window_size', 7).value)
        )
        if self.depth_window_size % 2 == 0:
            self.depth_window_size += 1
            self.get_logger().warning(
                f'depth_window_size must be odd; using {self.depth_window_size}'
            )
        self.min_valid_depth_count = max(
            1, int(self.declare_parameter('min_valid_depth_count', 5).value)
        )
        self.min_depth_m = float(
            self.declare_parameter('min_depth_m', 0.10).value
        )
        self.max_depth_m = float(
            self.declare_parameter('max_depth_m', 2.50).value
        )
        self.sync_slop_sec = float(
            self.declare_parameter('sync_slop_sec', 0.08).value
        )
        self.depth_scale_16u = float(
            self.declare_parameter('depth_scale_16u', 0.001).value
        )
        self.sample_csv_path = str(
            self.declare_parameter('sample_csv_path', '').value
        ).strip()
        self.debug_image_path = str(
            self.declare_parameter('debug_image_path', '').value
        ).strip()

        requested_color_mode = str(
            self.declare_parameter('color_mode', 'red').value
        ).strip().lower()
        if requested_color_mode not in self.HSV_RANGES:
            self.get_logger().error(
                f'Invalid color_mode "{requested_color_mode}"; expected red, green, '
                'or cyan. Falling back to red.'
            )
            self.color_mode = 'red'
        else:
            self.color_mode = requested_color_mode

        self.bridge = CvBridge()
        self.camera_info: Optional[CameraInfo] = None
        self.frame_count = 0
        self.depth_metadata_logged = False
        self.last_log_ns: dict[str, int] = {}
        self.last_debug_save_ns = 0

        self.debug_publisher = self.create_publisher(Image, debug_topic, 10)
        self.pixel_publisher = self.create_publisher(PointStamped, pixel_topic, 10)
        self.camera_point_publisher = self.create_publisher(
            PointStamped, camera_point_topic, 10
        )
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            qos_profile_sensor_data,
        )
        self.color_subscriber = Subscriber(
            self, Image, self.image_topic, qos_profile=qos_profile_sensor_data
        )
        self.depth_subscriber = Subscriber(
            self, Image, self.depth_topic, qos_profile=qos_profile_sensor_data
        )
        self.synchronizer = ApproximateTimeSynchronizer(
            [self.color_subscriber, self.depth_subscriber],
            queue_size=20,
            slop=self.sync_slop_sec,
        )
        self.synchronizer.registerCallback(self.synchronized_callback)
        self._initialize_csv()

        self.get_logger().info(
            f'RGBD detector: color={self.image_topic}; depth={self.depth_topic}; '
            f'camera_info={self.camera_info_topic}; debug={debug_topic}; '
            f'pixel={pixel_topic}; camera_point={camera_point_topic}; '
            f'color_mode={self.color_mode}; min_area={self.min_area:.1f}; '
            f'slop={self.sync_slop_sec:.3f}s'
        )

    def _throttled_log(self, level: str, key: str, message: str) -> None:
        """Log at most once every five seconds for a recurring condition."""
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_log_ns.get(key, -5_000_000_000) < 5_000_000_000:
            return
        self.last_log_ns[key] = now_ns
        getattr(self.get_logger(), level)(message)

    def _initialize_csv(self) -> None:
        if not self.sample_csv_path:
            return
        try:
            directory = os.path.dirname(self.sample_csv_path)
            if directory:
                os.makedirs(directory, exist_ok=True)
            with open(self.sample_csv_path, 'w', newline='', encoding='utf-8') as csv_file:
                csv.writer(csv_file).writerow([
                    'timestamp', 'u', 'v', 'x_m', 'y_m', 'z_m',
                    'valid_depth_count', 'color_depth_delta_ms',
                ])
        except OSError as error:
            self.get_logger().error(f'Cannot initialize sample CSV: {error}')
            self.sample_csv_path = ''

    def camera_info_callback(self, message: CameraInfo) -> None:
        """Cache the latest valid color CameraInfo."""
        valid = (
            message.width > 0
            and message.height > 0
            and len(message.k) >= 6
            and message.k[0] > 0.0
            and message.k[4] > 0.0
            and bool(message.header.frame_id)
        )
        if not valid:
            self._throttled_log(
                'error', 'invalid_camera_info',
                'Invalid CameraInfo: require positive size/fx/fy and non-empty frame_id.'
            )
            return
        self.camera_info = message

    @staticmethod
    def _stamp_seconds(message: Image) -> float:
        stamp = message.header.stamp
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _create_color_mask(self, hsv_image: np.ndarray) -> np.ndarray:
        ranges = self.HSV_RANGES[self.color_mode]
        mask = cv2.inRange(
            hsv_image,
            np.array(ranges[0], dtype=np.uint8),
            np.array(ranges[1], dtype=np.uint8),
        )
        if self.color_mode == 'red':
            second_mask = cv2.inRange(
                hsv_image,
                np.array(ranges[2], dtype=np.uint8),
                np.array(ranges[3], dtype=np.uint8),
            )
            mask = cv2.bitwise_or(mask, second_mask)
        return mask

    def _detect_target(
        self, image: np.ndarray
    ) -> Optional[tuple[np.ndarray, int, int, float]]:
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = self._create_color_mask(hsv_image)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        contour = max(contours, key=cv2.contourArea) if contours else None
        if contour is None:
            return None
        area = float(cv2.contourArea(contour))
        moments = cv2.moments(contour)
        if area < self.min_area or moments['m00'] == 0.0:
            return None
        u = int(round(moments['m10'] / moments['m00']))
        v = int(round(moments['m01'] / moments['m00']))
        return contour, u, v, area

    def _extract_depth(
        self, depth_image: np.ndarray, encoding: str, u: int, v: int
    ) -> tuple[Optional[float], int]:
        half = self.depth_window_size // 2
        y0 = max(0, v - half)
        y1 = min(depth_image.shape[0], v + half + 1)
        x0 = max(0, u - half)
        x1 = min(depth_image.shape[1], u + half + 1)
        raw_values = np.asarray(depth_image[y0:y1, x0:x1]).reshape(-1)
        finite_raw = raw_values[np.isfinite(raw_values) & (raw_values > 0)]
        raw_median = float(np.median(finite_raw)) if finite_raw.size else math.nan

        normalized_encoding = encoding.strip().lower()
        if normalized_encoding in ('16uc1', 'mono16'):
            values_m = raw_values.astype(np.float64) * self.depth_scale_16u
            converted_median = raw_median * self.depth_scale_16u
        elif normalized_encoding == '32fc1':
            values_m = raw_values.astype(np.float64)
            converted_median = raw_median
        else:
            self._throttled_log(
                'error', 'unsupported_depth_encoding',
                f'Unsupported depth encoding "{encoding}"; no unit is assumed.'
            )
            return None, 0

        if not self.depth_metadata_logged:
            self.get_logger().info(
                f'Depth metadata: encoding={encoding}; raw_median={raw_median:.6g}; '
                f'converted_median_m={converted_median:.6g}; '
                f'depth_scale_16u={self.depth_scale_16u:.9g}'
            )
            self.depth_metadata_logged = True

        valid = values_m[
            np.isfinite(values_m)
            & (values_m > 0.0)
            & (values_m >= self.min_depth_m)
            & (values_m <= self.max_depth_m)
        ]
        if valid.size < self.min_valid_depth_count:
            if finite_raw.size and (
                not math.isfinite(converted_median)
                or converted_median < self.min_depth_m
                or converted_median > self.max_depth_m
            ):
                self._throttled_log(
                    'error', 'possible_depth_scale_error',
                    f'Depth conversion is outside [{self.min_depth_m:.3f}, '
                    f'{self.max_depth_m:.3f}] m (raw median={raw_median:.6g}, '
                    f'converted={converted_median:.6g} m); check depth units/scale.'
                )
            return None, int(valid.size)
        return float(np.median(valid)), int(valid.size)

    def _draw_detection(
        self, image: np.ndarray, contour: np.ndarray, u: int, v: int
    ) -> None:
        display_color = self.DISPLAY_COLORS[self.color_mode]
        x, y, width, height = cv2.boundingRect(contour)
        cv2.drawContours(image, [contour], -1, display_color, 2)
        cv2.rectangle(
            image, (x, y), (x + width - 1, y + height - 1), display_color, 3
        )
        cv2.drawMarker(image, (u, v), (255, 255, 255), cv2.MARKER_CROSS, 24, 2)
        cv2.circle(image, (u, v), 7, (0, 255, 255), 2)

    @staticmethod
    def _draw_lines(image: np.ndarray, lines: list[str]) -> None:
        for index, line in enumerate(lines):
            cv2.putText(
                image, line, (16, 30 + index * 26), cv2.FONT_HERSHEY_SIMPLEX,
                0.65, (0, 0, 0), 4, cv2.LINE_AA
            )
            color = (0, 0, 255) if 'INVALID' in line or 'WAITING' in line else (255, 255, 255)
            cv2.putText(
                image, line, (16, 30 + index * 26), cv2.FONT_HERSHEY_SIMPLEX,
                0.65, color, 2, cv2.LINE_AA
            )

    def _publish_pixel(self, color_message: Image, u: int, v: int) -> None:
        point = PointStamped()
        point.header = color_message.header
        point.point.x = float(u)
        point.point.y = float(v)
        point.point.z = 0.0
        self.pixel_publisher.publish(point)

    def _publish_camera_point(
        self, color_message: Image, u: int, v: int, z_m: float
    ) -> tuple[float, float, float]:
        assert self.camera_info is not None
        fx = float(self.camera_info.k[0])
        fy = float(self.camera_info.k[4])
        cx = float(self.camera_info.k[2])
        cy = float(self.camera_info.k[5])
        x_m = (float(u) - cx) * z_m / fx
        y_m = (float(v) - cy) * z_m / fy
        point = PointStamped()
        point.header.stamp = color_message.header.stamp
        point.header.frame_id = self.camera_info.header.frame_id
        point.point.x = x_m
        point.point.y = y_m
        point.point.z = z_m
        self.camera_point_publisher.publish(point)
        return x_m, y_m, z_m

    def _append_csv(
        self, color_message: Image, u: int, v: int,
        xyz: tuple[float, float, float], valid_count: int, delta_ms: float
    ) -> None:
        if not self.sample_csv_path:
            return
        stamp = self._stamp_seconds(color_message)
        try:
            with open(self.sample_csv_path, 'a', newline='', encoding='utf-8') as csv_file:
                csv.writer(csv_file).writerow([
                    f'{stamp:.9f}', u, v, f'{xyz[0]:.9f}', f'{xyz[1]:.9f}',
                    f'{xyz[2]:.9f}', valid_count, f'{delta_ms:.6f}',
                ])
        except OSError as error:
            self._throttled_log('error', 'csv_write', f'Cannot write sample CSV: {error}')

    def _publish_debug(self, image: np.ndarray, color_message: Image) -> None:
        debug_message = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        debug_message.header = color_message.header
        self.debug_publisher.publish(debug_message)
        if self.debug_image_path:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self.last_debug_save_ns >= 1_000_000_000:
                try:
                    directory = os.path.dirname(self.debug_image_path)
                    if directory:
                        os.makedirs(directory, exist_ok=True)
                    if not cv2.imwrite(self.debug_image_path, image):
                        raise OSError('cv2.imwrite returned false')
                    self.last_debug_save_ns = now_ns
                except OSError as error:
                    self._throttled_log(
                        'error', 'debug_image_write',
                        f'Cannot save debug image: {error}'
                    )

    def synchronized_callback(
        self, color_message: Image, depth_message: Image
    ) -> None:
        """Process one approximately synchronized color/depth pair safely."""
        delta_ms = abs(
            self._stamp_seconds(color_message) - self._stamp_seconds(depth_message)
        ) * 1000.0
        try:
            bgr_image = self.bridge.imgmsg_to_cv2(
                color_message, desired_encoding='bgr8'
            )
            depth_image = self.bridge.imgmsg_to_cv2(
                depth_message, desired_encoding='passthrough'
            )
        except CvBridgeError as error:
            self._throttled_log('error', 'image_conversion', f'Image conversion failed: {error}')
            return

        try:
            annotated = bgr_image.copy()
            detection = self._detect_target(bgr_image)
            lines = [f'color_mode={self.color_mode}', f'delta={delta_ms:.3f} ms']
            if detection is None:
                lines.append(f'NO {self.color_mode.upper()} POINT')
                self._draw_lines(annotated, lines)
                self._publish_debug(annotated, color_message)
                return

            contour, u, v, area = detection
            self._draw_detection(annotated, contour, u, v)
            self._publish_pixel(color_message, u, v)
            lines.extend([f'u={u} v={v} area={area:.1f}'])

            if delta_ms > self.sync_slop_sec * 1000.0 + 1e-6:
                self._throttled_log(
                    'error', 'sync_delta',
                    f'Color/depth delta {delta_ms:.3f} ms exceeds '
                    f'{self.sync_slop_sec * 1000.0:.3f} ms.'
                )
                lines.append('DEPTH INVALID')
                self._draw_lines(annotated, lines)
                self._publish_debug(annotated, color_message)
                return

            if bgr_image.shape[:2] != depth_image.shape[:2]:
                self._throttled_log(
                    'error', 'color_depth_size',
                    f'Color {bgr_image.shape[1]}x{bgr_image.shape[0]} and depth '
                    f'{depth_image.shape[1]}x{depth_image.shape[0]} sizes differ; '
                    'aligned depth is required and no resizing is performed.'
                )
                lines.append('DEPTH INVALID')
                self._draw_lines(annotated, lines)
                self._publish_debug(annotated, color_message)
                return

            depth_m, valid_count = self._extract_depth(
                depth_image, depth_message.encoding, u, v
            )
            lines.append(f'valid_depth={valid_count}')
            if depth_m is None:
                lines.append('DEPTH INVALID')
                self._draw_lines(annotated, lines)
                self._publish_debug(annotated, color_message)
                return

            lines.append(f'Z={depth_m:.3f} m')
            if self.camera_info is None:
                lines.append('WAITING CAMERA INFO')
                self._draw_lines(annotated, lines)
                self._publish_debug(annotated, color_message)
                return

            if (
                self.camera_info.width != color_message.width
                or self.camera_info.height != color_message.height
            ):
                self._throttled_log(
                    'error', 'camera_info_size',
                    f'CameraInfo {self.camera_info.width}x{self.camera_info.height} '
                    f'does not match color {color_message.width}x{color_message.height}.'
                )
                lines.append('CAMERA INFO SIZE INVALID')
                self._draw_lines(annotated, lines)
                self._publish_debug(annotated, color_message)
                return

            xyz = self._publish_camera_point(color_message, u, v, depth_m)
            lines.append(f'X={xyz[0]:.3f} Y={xyz[1]:.3f} Z={xyz[2]:.3f} m')
            self._append_csv(color_message, u, v, xyz, valid_count, delta_ms)
            self._draw_lines(annotated, lines)
            self._publish_debug(annotated, color_message)

            self.frame_count += 1
            if self.frame_count % 30 == 0:
                self.get_logger().info(
                    f'{self.color_mode} u={u} v={v} area={area:.1f} '
                    f'valid_depth={valid_count} delta_ms={delta_ms:.3f} '
                    f'XYZ=({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f}) m'
                )
        except Exception as error:  # A malformed frame must not stop the node.
            self._throttled_log(
                'error', 'frame_processing', f'Failed to process RGBD frame: {error}'
            )


def main(args: Optional[list[str]] = None) -> None:
    """Run the detector until shutdown."""
    rclpy.init(args=args)
    node = RedPointDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
