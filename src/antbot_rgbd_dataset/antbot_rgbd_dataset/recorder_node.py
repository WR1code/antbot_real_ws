"""ROS 2 node for exact-stamp RGB-D keyframe recording."""

from __future__ import annotations

from collections import Counter, deque
import json
from pathlib import Path
import time
import traceback

from cv_bridge import CvBridge
import cv2
import numpy as np
import psutil
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMessage
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .core import (
    DatasetWriter,
    ExactStampSynchronizer,
    KeyframePolicy,
    atomic_png,
    atomic_yaml,
    camera_info_equal,
    canonical_camera_info,
    depth_statistics,
    laplacian_variance,
    select_keyframe,
    sharpness_distribution,
    stamp_ns,
    transform_matrix,
    utc_timestamp,
)


class RgbdKeyframeRecorder(Node):
    STATES = ("IDLE", "RECORDING", "STOPPING", "COMPLETED", "ERROR")

    def __init__(self) -> None:
        super().__init__("rgbd_keyframe_recorder")
        self._declare_parameters()
        self._load_parameters()
        self._bridge = CvBridge()
        self._sync = ExactStampSynchronizer(self.sync_cache_size)
        self._queue: deque[tuple[Image, Image, CameraInfo, CameraInfo]] = deque()
        self._tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._writer: DatasetWriter | None = None
        self._state = "IDLE"
        self._error = ""
        self._manual_next = False
        self._color_info: dict | None = None
        self._depth_info: dict | None = None
        self._rgb_encoding = ""
        self._depth_encoding = ""
        self._alpha_removed = False
        self._created_at = ""
        self._previous_pose: np.ndarray | None = None
        self._previous_timestamp_ns: int | None = None
        self._sharpness_samples: list[float] = []
        self._path = PathMessage()
        self._path.header.frame_id = self.fixed_frame
        self._counters = Counter()
        self._selection_counts = Counter()
        self._bundle_processing_ms: list[float] = []
        self._keyframe_write_ms: list[float] = []
        self._peak_queue = 0
        self._peak_rss_bytes = psutil.Process().memory_info().rss
        self._trajectory_length_m = 0.0

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=self.subscription_depth,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Image, self.rgb_topic, lambda msg: self._receive("rgb", msg), sensor_qos)
        self.create_subscription(Image, self.depth_topic, lambda msg: self._receive("depth", msg), sensor_qos)
        self.create_subscription(
            CameraInfo, self.color_info_topic, lambda msg: self._receive("color_info", msg), sensor_qos
        )
        self.create_subscription(
            CameraInfo, self.depth_info_topic, lambda msg: self._receive("depth_info", msg), sensor_qos
        )
        self._status_publisher = self.create_publisher(String, self.status_topic, 10)
        self._path_publisher = self.create_publisher(PathMessage, self.path_topic, 10)
        self._marker_publisher = self.create_publisher(MarkerArray, self.marker_topic, 10)
        self.create_service(Trigger, self.start_service, self._start_service)
        self.create_service(Trigger, self.stop_service, self._stop_service)
        self.create_service(Trigger, self.capture_service, self._capture_service)
        self.create_service(Trigger, self.status_service, self._status_service)
        self.create_timer(0.01, self._process_one)
        self.create_timer(1.0, self._publish_status)
        if self.auto_start:
            self._start()

    def _declare_parameters(self) -> None:
        defaults = {
            "rgb_topic": "/antbot/camera/color/image_raw",
            "depth_topic": "/antbot/camera/depth/image_raw",
            "color_info_topic": "/antbot/camera/color/camera_info",
            "depth_info_topic": "/antbot/camera/depth/camera_info",
            "fixed_frame": "odom",
            "camera_frame": "camera_color_optical_frame",
            "output_root": "artifacts/rgbd_datasets",
            "dataset_name": "home_rgbd_01",
            "auto_start": True,
            "overwrite_existing": False,
            "translation_threshold_m": 0.15,
            "rotation_threshold_deg": 8.0,
            "minimum_interval_sec": 0.30,
            "maximum_interval_sec": 2.0,
            "stationary_translation_epsilon_m": 0.005,
            "stationary_rotation_epsilon_deg": 0.2,
            "enable_blur_filter": True,
            "minimum_laplacian_variance": 20.0,
            "minimum_valid_depth_ratio": 0.30,
            "minimum_depth_m": 0.10,
            "maximum_depth_m": 20.0,
            "save_float32_npy": True,
            "save_uint16_png": True,
            "sync_cache_size": 30,
            "processing_queue_size": 8,
            "subscription_depth": 10,
            "tf_timeout_sec": 0.20,
            "depth_semantics": "distance_to_image_plane",
            "status_topic": "/antbot/rgbd_dataset/status",
            "path_topic": "/antbot/rgbd_dataset/keyframe_path",
            "marker_topic": "/antbot/rgbd_dataset/keyframe_markers",
            "start_service": "/antbot/rgbd_dataset/start",
            "stop_service": "/antbot/rgbd_dataset/stop",
            "capture_service": "/antbot/rgbd_dataset/capture",
            "status_service": "/antbot/rgbd_dataset/get_status",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _load_parameters(self) -> None:
        for name in (
            "rgb_topic",
            "depth_topic",
            "color_info_topic",
            "depth_info_topic",
            "fixed_frame",
            "camera_frame",
            "output_root",
            "dataset_name",
            "status_topic",
            "path_topic",
            "marker_topic",
            "start_service",
            "stop_service",
            "capture_service",
            "status_service",
            "depth_semantics",
        ):
            setattr(self, name, str(self.get_parameter(name).value))
        for name in ("auto_start", "overwrite_existing", "save_float32_npy", "save_uint16_png"):
            setattr(self, name, bool(self.get_parameter(name).value))
        for name in ("sync_cache_size", "processing_queue_size", "subscription_depth"):
            setattr(self, name, int(self.get_parameter(name).value))
        for name in ("minimum_depth_m", "maximum_depth_m", "tf_timeout_sec"):
            setattr(self, name, float(self.get_parameter(name).value))
        self.policy = KeyframePolicy(
            translation_threshold_m=float(self.get_parameter("translation_threshold_m").value),
            rotation_threshold_deg=float(self.get_parameter("rotation_threshold_deg").value),
            minimum_interval_sec=float(self.get_parameter("minimum_interval_sec").value),
            maximum_interval_sec=float(self.get_parameter("maximum_interval_sec").value),
            stationary_translation_epsilon_m=float(
                self.get_parameter("stationary_translation_epsilon_m").value
            ),
            stationary_rotation_epsilon_deg=float(
                self.get_parameter("stationary_rotation_epsilon_deg").value
            ),
            enable_blur_filter=bool(self.get_parameter("enable_blur_filter").value),
            minimum_laplacian_variance=float(
                self.get_parameter("minimum_laplacian_variance").value
            ),
            minimum_valid_depth_ratio=float(
                self.get_parameter("minimum_valid_depth_ratio").value
            ),
        )

    def _receive(self, stream: str, message: Image | CameraInfo) -> None:
        if self._state not in ("RECORDING", "STOPPING"):
            return
        bundle = self._sync.add(stream, message)
        if bundle is None:
            return
        self._counters["complete_bundles"] += 1
        if self._state != "RECORDING":
            return
        if len(self._queue) >= self.processing_queue_size:
            self._counters["queue_overload_drops"] += 1
            return
        self._queue.append(bundle)
        self._peak_queue = max(self._peak_queue, len(self._queue))

    def _process_one(self) -> None:
        if self._queue:
            bundle = self._queue.popleft()
            started = time.perf_counter()
            try:
                self._process_bundle(*bundle)
            except Exception as exc:
                self._fail(f"bundle processing failed: {exc}", exc)
            finally:
                self._bundle_processing_ms.append((time.perf_counter() - started) * 1000.0)
                self._peak_rss_bytes = max(self._peak_rss_bytes, psutil.Process().memory_info().rss)
        elif self._state == "STOPPING":
            self._finalize()

    def _process_bundle(
        self, rgb_msg: Image, depth_msg: Image, color_msg: CameraInfo, depth_info_msg: CameraInfo
    ) -> None:
        timestamps = {stamp_ns(message) for message in (rgb_msg, depth_msg, color_msg, depth_info_msg)}
        if len(timestamps) != 1:
            self._counters["sync_failures"] += 1
            return
        timestamp_ns = timestamps.pop()
        color_info = canonical_camera_info(color_msg)
        depth_info = canonical_camera_info(depth_info_msg)
        self._check_camera_info(color_info, depth_info)
        rgb = self._rgb_array(rgb_msg)
        depth_m = self._depth_array(depth_msg)
        if rgb.shape[:2] != depth_m.shape or rgb.shape[1] != color_info["width"] or rgb.shape[0] != color_info["height"]:
            raise ValueError(
                f"image/CameraInfo dimensions changed: rgb={rgb.shape}, depth={depth_m.shape}, "
                f"info={color_info['width']}x{color_info['height']}"
            )
        sharpness = laplacian_variance(rgb)
        self._sharpness_samples.append(sharpness)
        stats = depth_statistics(depth_m, self.minimum_depth_m, self.maximum_depth_m)
        transform = self._lookup_transform(timestamp_ns)
        if transform is None:
            return
        translation = np.array(
            [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ],
            dtype=np.float64,
        )
        quaternion = np.array(
            [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ],
            dtype=np.float64,
        )
        matrix = transform_matrix(translation, quaternion)
        selection = select_keyframe(
            self.policy,
            timestamp_ns,
            matrix,
            sharpness,
            stats.valid_ratio,
            self._previous_timestamp_ns,
            self._previous_pose,
            self._manual_next,
        )
        if not selection.selected:
            self._counters[f"rejected_{selection.rejection.lower()}"] += 1
            return
        self._manual_next = False
        pose = self._pose_record(timestamp_ns, translation, quaternion, matrix)
        camera = {
            "frame_index": len(self._writer.rows),
            "timestamp_ns": timestamp_ns,
            "color_camera_info": color_info,
            "depth_camera_info": depth_info,
        }
        write_started = time.perf_counter()
        try:
            row = self._writer.write_frame(
                timestamp_ns, rgb, depth_m, pose, camera, stats, sharpness, selection
            )
        except Exception:
            self._counters["disk_write_failures"] += 1
            raise
        self._keyframe_write_ms.append((time.perf_counter() - write_started) * 1000.0)
        self._trajectory_length_m += selection.translation_m
        self._previous_pose = matrix
        self._previous_timestamp_ns = timestamp_ns
        self._counters["keyframes"] += 1
        for reason in selection.reasons:
            self._selection_counts[reason] += 1
        self._publish_pose(row, translation, quaternion, rgb, depth_m)

    def _check_camera_info(self, color: dict, depth: dict) -> None:
        comparable_color = {**color, "frame_id": ""}
        comparable_depth = {**depth, "frame_id": ""}
        if not camera_info_equal(comparable_color, comparable_depth):
            raise ValueError("color and depth CameraInfo intrinsics differ")
        if color["frame_id"] != self.camera_frame:
            raise ValueError(
                f"color CameraInfo frame is {color['frame_id']!r}, expected {self.camera_frame!r}"
            )
        if self._color_info is None:
            self._color_info = color
            self._depth_info = depth
            self._write_intrinsics()
        elif not camera_info_equal(self._color_info, color) or not camera_info_equal(
            self._depth_info, depth
        ):
            raise ValueError("CameraInfo changed during recording")

    def _rgb_array(self, message: Image) -> np.ndarray:
        array = self._bridge.imgmsg_to_cv2(message, desired_encoding="passthrough")
        encoding = message.encoding.lower()
        if not self._rgb_encoding:
            self._rgb_encoding = message.encoding
        elif self._rgb_encoding != message.encoding:
            raise ValueError("RGB encoding changed during recording")
        if encoding == "rgb8":
            return np.asarray(array)
        if encoding == "bgr8":
            return cv2.cvtColor(array, cv2.COLOR_BGR2RGB)
        if encoding == "rgba8":
            self._alpha_removed = True
            return cv2.cvtColor(array, cv2.COLOR_RGBA2RGB)
        if encoding == "bgra8":
            self._alpha_removed = True
            return cv2.cvtColor(array, cv2.COLOR_BGRA2RGB)
        if encoding in ("mono8", "8uc1"):
            return cv2.cvtColor(array, cv2.COLOR_GRAY2RGB)
        raise ValueError(f"unsupported RGB encoding: {message.encoding}")

    def _depth_array(self, message: Image) -> np.ndarray:
        array = np.asarray(self._bridge.imgmsg_to_cv2(message, desired_encoding="passthrough"))
        if not self._depth_encoding:
            self._depth_encoding = message.encoding
        elif self._depth_encoding != message.encoding:
            raise ValueError("depth encoding changed during recording")
        if message.encoding.upper() == "32FC1":
            return array.astype(np.float32, copy=False)
        if message.encoding.upper() in ("16UC1", "MONO16"):
            return array.astype(np.float32) * 0.001
        raise ValueError(f"unsupported depth encoding: {message.encoding}")

    def _lookup_transform(self, timestamp_ns: int):
        self._counters["tf_queries"] += 1
        try:
            transform = self._tf_buffer.lookup_transform(
                self.fixed_frame,
                self.camera_frame,
                Time(nanoseconds=timestamp_ns),
                timeout=Duration(seconds=self.tf_timeout_sec),
            )
            self._counters["tf_successes"] += 1
            return transform
        except TransformException as exc:
            text = str(exc).lower()
            if "future" in text:
                self._counters["tf_future_extrapolation"] += 1
            elif "past" in text:
                self._counters["tf_past_extrapolation"] += 1
            else:
                self._counters["tf_other_failures"] += 1
            self.get_logger().warning(f"TF query failed at {timestamp_ns}: {exc}")
            return None

    def _pose_record(
        self, timestamp_ns: int, translation: np.ndarray, quaternion: np.ndarray, matrix: np.ndarray
    ) -> dict:
        return {
            "frame_index": len(self._writer.rows),
            "timestamp_ns": timestamp_ns,
            "fixed_frame": self.fixed_frame,
            "camera_frame": self.camera_frame,
            "transform_definition": f"p_{self.fixed_frame} = T_{self.fixed_frame}_camera * p_camera",
            "translation_m": dict(zip(("x", "y", "z"), map(float, translation))),
            "quaternion_xyzw": dict(zip(("x", "y", "z", "w"), map(float, quaternion))),
            f"T_{self.fixed_frame}_camera": matrix.tolist(),
            "T_odom_camera": matrix.tolist() if self.fixed_frame == "odom" else None,
        }

    def _write_intrinsics(self) -> None:
        k = self._color_info["K"]
        atomic_yaml(
            self._writer.root / "intrinsics.yaml",
            {
                "image_width": self._color_info["width"],
                "image_height": self._color_info["height"],
                "fx": k[0],
                "fy": k[4],
                "cx": k[2],
                "cy": k[5],
                "K": k,
                "P": self._color_info["P"],
                "distortion_model": self._color_info["distortion_model"],
                "D": self._color_info["D"],
                "depth_unit": "meter",
                "depth_semantics": self.depth_semantics,
                "camera_frame": self.camera_frame,
            },
        )

    def _publish_pose(
        self, row: dict, translation: np.ndarray, quaternion: np.ndarray, rgb: np.ndarray, depth: np.ndarray
    ) -> None:
        pose = PoseStamped()
        pose.header.frame_id = self.fixed_frame
        pose.header.stamp.sec = int(row["timestamp_sec"])
        pose.header.stamp.nanosec = int(row["timestamp_nanosec"])
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, translation)
        (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ) = map(float, quaternion)
        self._path.header = pose.header
        self._path.poses.append(pose)
        self._path_publisher.publish(self._path)
        marker = Marker()
        marker.header = pose.header
        marker.ns = "rgbd_keyframes"
        marker.id = int(row["frame_index"])
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.scale.x = marker.scale.y = marker.scale.z = 0.06
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.1, 0.8, 1.0, 1.0
        self._marker_publisher.publish(MarkerArray(markers=[marker]))
        if row["frame_index"] == 0:
            atomic_png(self._writer.root / "previews/first_rgb.png", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        atomic_png(self._writer.root / "previews/latest_rgb.png", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        finite = np.isfinite(depth) & (depth >= self.minimum_depth_m) & (depth <= self.maximum_depth_m)
        preview = np.zeros(depth.shape, dtype=np.uint8)
        if np.any(finite):
            preview[finite] = np.clip(
                255.0 * (depth[finite] - self.minimum_depth_m)
                / (self.maximum_depth_m - self.minimum_depth_m),
                0,
                255,
            ).astype(np.uint8)
        atomic_png(self._writer.root / "previews/latest_depth.png", cv2.applyColorMap(preview, cv2.COLORMAP_TURBO))

    def _start_service(self, _request, response):
        if self._state == "RECORDING":
            response.success, response.message = False, "already recording"
            return response
        response.success, response.message = self._start()
        return response

    def _stop_service(self, _request, response):
        if self._state != "RECORDING":
            response.success, response.message = False, f"cannot stop from {self._state}"
        else:
            self._state = "STOPPING"
            response.success, response.message = True, "stop requested; draining bounded queue"
        return response

    def _capture_service(self, _request, response):
        if self._state != "RECORDING":
            response.success, response.message = False, f"cannot capture from {self._state}"
        else:
            self._manual_next = True
            response.success, response.message = True, "next quality-valid bundle will be captured"
        return response

    def _status_service(self, _request, response):
        response.success = self._state != "ERROR"
        response.message = json.dumps(self._status(), ensure_ascii=False)
        return response

    def _start(self) -> tuple[bool, str]:
        try:
            self._writer = DatasetWriter(
                Path(self.output_root),
                self.dataset_name,
                self.save_float32_npy,
                self.save_uint16_png,
                self.overwrite_existing,
            )
        except Exception as exc:
            self._fail(f"failed to create dataset: {exc}", exc)
            return False, self._error
        self._created_at = utc_timestamp()
        self._state = "RECORDING"
        self.get_logger().info(f"recording exact-stamp RGB-D dataset at {self._writer.root}")
        return True, str(self._writer.root)

    def _finalize(self) -> None:
        try:
            self._sync.flush_unmatched()
            self._writer.finalize(self._metadata())
            self._state = "COMPLETED"
            self.get_logger().info(
                f"dataset completed: {self._writer.root} ({len(self._writer.rows)} keyframes)"
            )
        except Exception as exc:
            self._fail(f"failed to finalize dataset: {exc}", exc)

    def _metadata(self) -> dict:
        rows = self._writer.rows
        k = self._color_info["K"] if self._color_info else [None] * 9
        return {
            "dataset_name": self.dataset_name,
            "created_at": self._created_at,
            "completed_at": utc_timestamp(),
            "coordinate_frame": self.fixed_frame,
            "camera_frame": self.camera_frame,
            "transform_definition": {
                f"p_{self.fixed_frame}": f"T_{self.fixed_frame}_camera_times_p_camera"
            },
            "rgb_topic": self.rgb_topic,
            "depth_topic": self.depth_topic,
            "camera_info_topic": self.color_info_topic,
            "depth_camera_info_topic": self.depth_info_topic,
            "rgb_encoding": self._rgb_encoding,
            "rgb_alpha_removed": self._alpha_removed,
            "depth_encoding": self._depth_encoding,
            "depth_unit": "meter",
            "depth_semantics": self.depth_semantics,
            "depth_npy": {
                "dtype": "float32",
                "unit": "meter",
                "invalid_values": [0, "NaN", "Inf"],
            },
            "depth_png": {
                "enabled": self.save_uint16_png,
                "dtype": "uint16",
                "unit": "millimeter",
                "scale_from_meter": 1000,
                "invalid_value": 0,
            },
            "image_width": self._color_info["width"] if self._color_info else None,
            "image_height": self._color_info["height"] if self._color_info else None,
            "intrinsics": {"fx": k[0], "fy": k[4], "cx": k[2], "cy": k[5]},
            "camera_info_constant": True if self._color_info else None,
            "camera_mount": {
                "parent_frame": "base_link",
                "translation_m": {"x": 0.520, "y": 0.000, "z": 0.550},
                "rotation_rpy_deg": {"roll": 0.0, "pitch": 5.0, "yaw": 0.0},
            },
            "keyframe_policy": {
                **self.policy.__dict__,
                "minimum_depth_m": self.minimum_depth_m,
                "maximum_depth_m": self.maximum_depth_m,
                "blur_threshold_status": "CALIBRATED_FROM_LIT_ISAAC_RGB_DISTRIBUTION",
            },
            "sharpness_distribution": sharpness_distribution(self._sharpness_samples),
            "frame_count": len(rows),
            "start_timestamp_ns": rows[0]["timestamp_ns"] if rows else None,
            "end_timestamp_ns": rows[-1]["timestamp_ns"] if rows else None,
            "trajectory_length_m": self._trajectory_length_m,
            "counters": self._reported_counters(),
            "selection_counts": dict(self._selection_counts),
            "performance": self._performance(),
            "disk_usage_bytes": self._disk_usage(),
        }

    def _reported_counters(self) -> dict:
        counters = dict(self._counters)
        counters["sync_failures"] = counters.get("sync_failures", 0) + self._sync.sync_drop_count
        counters["timestamp_regressions"] = self._sync.timestamp_regression_count
        counters["duplicate_input_timestamps"] = self._sync.duplicate_input_count
        return counters

    def _performance(self) -> dict:
        return {
            "average_bundle_processing_ms": self._average(self._bundle_processing_ms),
            "maximum_bundle_processing_ms": max(self._bundle_processing_ms, default=0.0),
            "average_keyframe_write_ms": self._average(self._keyframe_write_ms),
            "maximum_keyframe_write_ms": max(self._keyframe_write_ms, default=0.0),
            "maximum_queue_length": self._peak_queue,
            "peak_rss_bytes": self._peak_rss_bytes,
        }

    @staticmethod
    def _average(values: list[float]) -> float:
        return sum(values) / len(values) if values else 0.0

    def _disk_usage(self) -> int:
        if not self._writer or not self._writer.root.exists():
            return 0
        return sum(path.stat().st_size for path in self._writer.root.rglob("*") if path.is_file())

    def _status(self) -> dict:
        return {
            "state": self._state,
            "dataset": str(self._writer.root) if self._writer else None,
            "queue_length": len(self._queue),
            "keyframes": self._counters["keyframes"],
            "complete_bundles": self._counters["complete_bundles"],
            "error": self._error,
            "counters": self._reported_counters(),
        }

    def _publish_status(self) -> None:
        message = String()
        message.data = json.dumps(self._status(), ensure_ascii=False)
        self._status_publisher.publish(message)

    def _fail(self, message: str, exception: Exception | None = None) -> None:
        self._state = "ERROR"
        self._error = message
        if exception:
            self.get_logger().error(f"{message}\n{traceback.format_exc()}")
        else:
            self.get_logger().error(message)
        if self._writer:
            try:
                metadata = self._metadata()
                metadata["recording_error"] = message
                self._writer.finalize(metadata, complete=False)
            except Exception as finalize_error:
                self.get_logger().error(f"error checkpoint also failed: {finalize_error}")

    def destroy_node(self) -> bool:
        if self._state in ("RECORDING", "STOPPING") and self._writer:
            self._queue.clear()
            try:
                self._finalize()
            except Exception:
                pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RgbdKeyframeRecorder()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            executor.shutdown()
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        finally:
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
