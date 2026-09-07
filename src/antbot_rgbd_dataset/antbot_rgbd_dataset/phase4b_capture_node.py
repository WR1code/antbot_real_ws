"""Real-robot-first Phase 4B approximate-sync RGB-D keyframe capture node."""

from __future__ import annotations

from collections import Counter
import json
import traceback

from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PoseStamped
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data,
)
from rclpy.time import Time
from nav_msgs.msg import Odometry, Path as NavPath
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import ColorRGBA, String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .configuration import validate_capture_parameters
from .core import laplacian_variance, stamp_ns, transform_matrix
from .depth import color_to_rgb, depth_to_meters, filter_depth
from .keyframes import KeyframeConfig, decide_keyframe
from .live_preview import (
    LivePreviewAccumulator, project_rgbd_filtered, save_preview_archive,
    voxel_reduce,
)
from .models import CameraIntrinsics, RGBDFrame
from .phase4b_writer import Phase4BDatasetWriter
from .pointcloud2 import colored_cloud_message
from .synchronization import ApproximateRGBDSynchronizer, NearestPoseBuffer
from .transforms import PoseQualityConfig, check_pose_quality, query_pose_at_timestamp


class Phase4BCaptureNode(Node):
    def __init__(self):
        super().__init__("phase4b_capture_node")
        self._declare_parameters()
        self.parameters = self._read_parameters()
        validate_capture_parameters(self.parameters)
        self.bridge = CvBridge()
        self.sync = ApproximateRGBDSynchronizer(
            self.parameters["max_rgb_depth_delta_ms"],
            self.parameters["sync_queue_size"],
        )
        self.pose_buffer = NearestPoseBuffer(
            self.parameters["max_pose_delta_ms"],
            self.parameters["pose_queue_size"],
        )
        self.tf_buffer = Buffer(cache_time=Duration(seconds=self.parameters["tf_cache_s"]))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.camera_info: CameraInfo | None = None
        self.reference_intrinsics: CameraIntrinsics | None = None
        self.counters = Counter()
        self.previous_valid_pose = None
        self.previous_valid_pose_timestamp_ns = None
        self.previous_keyframe_pose = None
        self.previous_keyframe_timestamp_ns = None
        self.first_frame = None
        self.finalized = False
        self.last_preview_timestamp_ns = None
        self.latest_preview_snapshot = None
        self.camera_path = NavPath()
        self.camera_path.header.frame_id = self.parameters["world_frame"]
        self.preview = LivePreviewAccumulator(
            self.parameters["preview_voxel_size_m"],
            self.parameters["coverage_voxel_size_m"],
            self.parameters["coverage_growth_window_s"],
        )
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.live_cloud_publisher = self.create_publisher(
            PointCloud2, "/antbot/rgbd/live_cloud", qos_profile_sensor_data
        )
        self.accumulated_cloud_publisher = self.create_publisher(
            PointCloud2, "/antbot/rgbd/accumulated_cloud", latched_qos
        )
        self.camera_path_publisher = self.create_publisher(
            NavPath, "/antbot/rgbd/camera_path", latched_qos
        )
        self.coverage_publisher = self.create_publisher(
            MarkerArray, "/antbot/rgbd/coverage_markers", latched_qos
        )
        self.coverage_marker_array_publisher = self.create_publisher(
            MarkerArray, "/antbot/rgbd/coverage_marker_array", latched_qos
        )
        self.status_publisher = self.create_publisher(
            String, "/antbot/rgbd/status", latched_qos
        )
        self.writer = Phase4BDatasetWriter(
            self.parameters["output_root"], self.parameters["dataset_name"],
            self.parameters["overwrite_existing"],
            self.parameters["minimum_free_space_gb"],
        )
        self.keyframe_config = KeyframeConfig(
            self.parameters["translation_threshold_m"],
            self.parameters["rotation_threshold_deg"],
            self.parameters["min_keyframe_interval_s"],
            self.parameters["max_keyframe_interval_s"],
        )
        self.pose_quality_config = PoseQualityConfig(
            self.parameters["max_translation_jump_m"],
            self.parameters["max_rotation_jump_deg"],
            self.parameters["max_linear_speed_mps"],
            self.parameters["max_angular_speed_deg_s"],
        )
        self.create_subscription(
            Image, self.parameters["color_topic"], self._color_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, self.parameters["depth_topic"], self._depth_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo, self.parameters["camera_info_topic"],
            self._camera_info_callback, qos_profile_sensor_data
        )
        if self.parameters["pose_mode"] == "topic":
            if self.parameters["pose_message_type"] == "odometry":
                self.create_subscription(
                    Odometry, self.parameters["pose_topic"],
                    self._odometry_callback, qos_profile_sensor_data
                )
            elif self.parameters["pose_message_type"] == "pose_stamped":
                self.create_subscription(
                    PoseStamped, self.parameters["pose_topic"],
                    self._pose_stamped_callback, qos_profile_sensor_data
                )
            else:
                raise ValueError("pose_message_type must be odometry or pose_stamped")
        self.get_logger().info(
            f"Phase 4B capture ready at {self.writer.root}; "
            f"pose_mode={self.parameters['pose_mode']}, use_sim_time="
            f"{self.get_parameter('use_sim_time').value}"
        )

    def _declare_parameters(self):
        defaults = {
            "color_topic": "/camera/color/image_raw",
            "depth_topic": "/camera/depth/image_raw",
            "camera_info_topic": "/camera/color/camera_info",
            "pose_mode": "tf",
            "pose_source": "odometry",
            "pose_topic": "/camera/pose",
            "pose_message_type": "odometry",
            "pose_topic_child_frame": "camera_color_optical_frame",
            "world_frame": "odom",
            "camera_frame": "camera_color_optical_frame",
            "output_root": "artifacts/rgbd_datasets",
            "dataset_name": "real_rgbd_phase4b",
            "overwrite_existing": False,
            "minimum_free_space_gb": 2.0,
            "robot_id": "antbot",
            "sensor_id": "front_rgbd",
            "depth_encoding_mode": "auto",
            "depth_scale_override": 0.0,
            "depth_aligned_to_color": True,
            "require_rectified_color": True,
            "depth_min": 0.10,
            "depth_max": 8.0,
            "minimum_valid_depth_ratio": 0.30,
            "max_rgb_depth_delta_ms": 30.0,
            "max_pose_delta_ms": 50.0,
            "sync_queue_size": 30,
            "pose_queue_size": 100,
            "tf_timeout_s": 0.10,
            "tf_cache_s": 30.0,
            "translation_threshold_m": 0.08,
            "rotation_threshold_deg": 7.0,
            "min_keyframe_interval_s": 0.10,
            "max_keyframe_interval_s": 1.0,
            "max_translation_jump_m": 1.0,
            "max_rotation_jump_deg": 60.0,
            "max_linear_speed_mps": 3.0,
            "max_angular_speed_deg_s": 180.0,
            "live_preview_enabled": True,
            "preview_voxel_size_m": 0.05,
            "preview_edge_filter_absolute_m": 0.08,
            "preview_edge_filter_relative": 0.03,
            "preview_min_observations": 2,
            "coverage_voxel_size_m": 0.20,
            "preview_pixel_stride": 4,
            "preview_max_rate_hz": 5.0,
            "coverage_growth_window_s": 10.0,
            "target_keyframes": 150,
            "minimum_direction_keyframes": 5,
            "stop_growth_percent": 1.0,
            "preview_output_path": "",
            "preview_output_frame": "",
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)

    def _read_parameters(self):
        return {name: self.get_parameter(name).value for name in (
            "color_topic", "depth_topic", "camera_info_topic", "pose_mode",
            "pose_source", "pose_topic", "pose_message_type", "pose_topic_child_frame",
            "world_frame", "camera_frame", "output_root", "dataset_name",
            "overwrite_existing", "minimum_free_space_gb", "robot_id", "sensor_id",
            "depth_encoding_mode", "depth_scale_override", "depth_aligned_to_color",
            "require_rectified_color",
            "depth_min", "depth_max", "minimum_valid_depth_ratio",
            "max_rgb_depth_delta_ms", "max_pose_delta_ms", "sync_queue_size",
            "pose_queue_size", "tf_timeout_s", "tf_cache_s",
            "translation_threshold_m", "rotation_threshold_deg",
            "min_keyframe_interval_s", "max_keyframe_interval_s",
            "max_translation_jump_m", "max_rotation_jump_deg",
            "max_linear_speed_mps", "max_angular_speed_deg_s",
            "live_preview_enabled", "preview_voxel_size_m",
            "preview_edge_filter_absolute_m", "preview_edge_filter_relative",
            "preview_min_observations",
            "coverage_voxel_size_m", "preview_pixel_stride",
            "preview_max_rate_hz", "coverage_growth_window_s",
            "target_keyframes", "minimum_direction_keyframes",
            "stop_growth_percent", "preview_output_path", "preview_output_frame",
        )}

    def _preview_due(self, timestamp_ns):
        rate = float(self.parameters["preview_max_rate_hz"])
        if rate <= 0:
            return True
        minimum_delta = int(1e9 / rate)
        return (
            self.last_preview_timestamp_ns is None
            or timestamp_ns - self.last_preview_timestamp_ns >= minimum_delta
        )

    def _preview_points(self, frame):
        points, colors = project_rgbd_filtered(
            frame, int(self.parameters["preview_pixel_stride"]),
            float(self.parameters["depth_min"]), float(self.parameters["depth_max"]),
            float(self.parameters["preview_edge_filter_absolute_m"]),
            float(self.parameters["preview_edge_filter_relative"]),
        )
        return voxel_reduce(points, colors, float(self.parameters["preview_voxel_size_m"]))

    def _append_path(self, frame, stamp):
        pose = PoseStamped()
        pose.header.frame_id = self.parameters["world_frame"]
        pose.header.stamp = stamp
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(
            float, frame.T_world_camera[:3, 3]
        )
        quaternion = frame.metadata["quaternion_xyzw"]
        pose.pose.orientation.x = float(quaternion["x"])
        pose.pose.orientation.y = float(quaternion["y"])
        pose.pose.orientation.z = float(quaternion["z"])
        pose.pose.orientation.w = float(quaternion["w"])
        self.camera_path.header.stamp = stamp
        self.camera_path.poses.append(pose)
        self.camera_path_publisher.publish(self.camera_path)

    def _scan_state(self, snapshot, valid_depth_ratio):
        saved = int(self.counters["frames_saved"])
        target = int(self.parameters["target_keyframes"])
        minimum_directions = int(self.parameters["minimum_direction_keyframes"])
        directions_ready = all(
            value >= minimum_directions for value in snapshot.direction_counts.values()
        )
        if (
            saved >= target
            and directions_ready
            and snapshot.recent_growth_percent <= self.parameters["stop_growth_percent"]
        ):
            return "RECOMMEND_STOP", "green", "coverage growth is low; stop is recommended"
        if (
            saved < max(1, target // 2)
            or valid_depth_ratio < self.parameters["minimum_valid_depth_ratio"]
            or not any(snapshot.direction_counts.values())
        ):
            return "INSUFFICIENT", "red", f"continue scanning toward {snapshot.weak_direction}"
        return "USABLE", "yellow", f"continue scanning toward {snapshot.weak_direction}"

    def _publish_status_and_markers(
        self, frame, stamp, snapshot, depth_stats, rgb_depth_delta_ms, pose_delta_ms
    ):
        state, color_name, hint = self._scan_state(snapshot, depth_stats["valid_ratio"])
        saved = int(self.counters["frames_saved"])
        payload = {
            "progress_basis": "relative_coverage_growth_unknown_scene",
            "keyframes": {"saved": saved, "target": int(self.parameters["target_keyframes"])},
            "valid_depth_percent": round(100.0 * depth_stats["valid_ratio"], 2),
            "rgb_depth_delta_ms": round(float(rgb_depth_delta_ms), 3),
            "pose_delta_ms": round(float(pose_delta_ms), 3),
            "occupied_voxels": snapshot.occupied_voxels,
            "weak_voxels": snapshot.weak_voxels,
            "bounds_m": [round(value, 2) for value in snapshot.bounds_m],
            "growth_window_s": float(self.parameters["coverage_growth_window_s"]),
            "recent_growth_percent": round(snapshot.recent_growth_percent, 2),
            "recent_direction": snapshot.direction,
            "direction_counts": snapshot.direction_counts,
            "weak_direction": snapshot.weak_direction,
            "state": state,
            "color": color_name,
            "hint": hint,
        }
        self.status_publisher.publish(
            String(data=json.dumps(payload, ensure_ascii=False, separators=(",", ":")))
        )

        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        centers, counts = self.preview.coverage_arrays()
        for marker_id, (namespace, mask, color) in enumerate((
            ("weak_coverage", counts <= 2, (1.0, 0.15, 0.05, 0.12)),
            ("covered", counts > 2, (0.1, 0.85, 0.25, 0.06)),
        )):
            marker = Marker()
            marker.header.frame_id = self.parameters["world_frame"]
            marker.header.stamp = stamp
            marker.ns = namespace
            marker.id = marker_id
            marker.type = Marker.CUBE_LIST
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            scale = float(self.parameters["coverage_voxel_size_m"]) * 0.92
            marker.scale.x = marker.scale.y = marker.scale.z = scale
            marker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])
            marker.points = [Point(x=float(x), y=float(y), z=float(z)) for x, y, z in centers[mask]]
            markers.markers.append(marker)

        panel = Marker()
        panel.header.frame_id = self.parameters["world_frame"]
        panel.header.stamp = stamp
        panel.ns = "scan_status"
        panel.id = 2
        panel.type = Marker.TEXT_VIEW_FACING
        panel.action = Marker.ADD
        panel.pose.position.x, panel.pose.position.y, panel.pose.position.z = map(
            float, frame.T_world_camera[:3, 3]
        )
        panel.pose.position.z += 1.0
        panel.pose.orientation.w = 1.0
        panel.scale.z = 0.18
        panel.color = {
            "red": ColorRGBA(r=1.0, g=0.1, b=0.1, a=1.0),
            "yellow": ColorRGBA(r=1.0, g=0.8, b=0.05, a=1.0),
            "green": ColorRGBA(r=0.1, g=1.0, b=0.2, a=1.0),
        }[color_name]
        bx, by, bz = snapshot.bounds_m
        growth_window = float(self.parameters["coverage_growth_window_s"])
        panel.text = (
            f"Keyframes {saved}/{self.parameters['target_keyframes']} | "
            f"Valid depth {100.0 * depth_stats['valid_ratio']:.1f}%\n"
            f"Bounds {bx:.1f} x {by:.1f} x {bz:.1f} m | "
            f"New {snapshot.recent_growth_percent:.1f}%/{growth_window:g}s\n"
            f"{state}: scan {snapshot.weak_direction}"
        )
        markers.markers.append(panel)
        self.coverage_publisher.publish(markers)
        self.coverage_marker_array_publisher.publish(markers)

    def _camera_info_callback(self, message: CameraInfo):
        self.camera_info = message

    def _color_callback(self, message: Image):
        pair = self.sync.add_color(stamp_ns(message), message)
        if pair:
            self._process_pair(pair)

    def _depth_callback(self, message: Image):
        pair = self.sync.add_depth(stamp_ns(message), message)
        if pair:
            self._process_pair(pair)

    def _odometry_callback(self, message: Odometry):
        if message.header.frame_id != self.parameters["world_frame"]:
            self.counters["dropped_pose_frame_mismatch"] += 1
            return
        child = message.child_frame_id or self.parameters["pose_topic_child_frame"]
        if child != self.parameters["camera_frame"]:
            self.counters["dropped_pose_not_camera_frame"] += 1
            self.get_logger().warning(
                f"rejecting Odometry child_frame_id={child!r}; expected camera frame "
                f"{self.parameters['camera_frame']!r}. Do not use base_link pose as camera pose."
            )
            return
        self._buffer_pose(message.header.stamp, message.pose.pose)

    def _pose_stamped_callback(self, message: PoseStamped):
        if (
            message.header.frame_id != self.parameters["world_frame"]
            or self.parameters["pose_topic_child_frame"] != self.parameters["camera_frame"]
        ):
            self.counters["dropped_pose_frame_mismatch"] += 1
            return
        self._buffer_pose(message.header.stamp, message.pose)

    def _buffer_pose(self, stamp, pose):
        timestamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        translation = np.array([pose.position.x, pose.position.y, pose.position.z])
        quaternion = np.array([
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        ])
        self.pose_buffer.add(
            timestamp_ns, (transform_matrix(translation, quaternion), quaternion)
        )

    def _pose_at_sensor_time(self, timestamp_ns: int):
        if self.parameters["pose_mode"] == "topic":
            def query_topic(sensor_stamp):
                nearest = self.pose_buffer.nearest(sensor_stamp)
                if nearest is None:
                    return None
                (matrix, quaternion), pose_timestamp_ns = nearest
                return matrix, quaternion, pose_timestamp_ns, self.parameters["pose_source"]
            result, _ = query_pose_at_timestamp(
                query_topic, timestamp_ns, self.parameters["max_pose_delta_ms"]
            )
            return result
        self.counters["tf_queries"] += 1
        def query_tf(sensor_stamp):
            transform = self.tf_buffer.lookup_transform(
                self.parameters["world_frame"], self.parameters["camera_frame"],
                Time(nanoseconds=sensor_stamp),
                timeout=Duration(seconds=self.parameters["tf_timeout_s"]),
            )
            value = transform.transform
            translation = np.array([
                value.translation.x, value.translation.y, value.translation.z
            ])
            quaternion = np.array([
                value.rotation.x, value.rotation.y, value.rotation.z, value.rotation.w
            ])
            return (
                transform_matrix(translation, quaternion), quaternion,
                stamp_ns(transform), self.parameters["pose_source"]
            )
        result, error = query_pose_at_timestamp(
            query_tf, timestamp_ns, self.parameters["max_pose_delta_ms"]
        )
        if error:
            self.counters["tf_query_failures"] += 1
            self.get_logger().warning(f"TF query at image stamp {timestamp_ns} failed: {error}")
        return result

    def _intrinsics(self) -> CameraIntrinsics:
        if self.camera_info is None:
            raise ValueError("CameraInfo has not been received")
        message = self.camera_info
        intrinsics = CameraIntrinsics(
            int(message.width), int(message.height), float(message.k[0]),
            float(message.k[4]), float(message.k[2]), float(message.k[5]),
            str(message.distortion_model), tuple(float(v) for v in message.d),
            str(message.header.frame_id), stamp_ns(message),
            bool(message.roi.do_rectify) or not any(abs(float(v)) > 1e-12 for v in message.d),
        )
        intrinsics.validate()
        if self.parameters["require_rectified_color"] and not intrinsics.rectified:
            raise ValueError(
                "color image is not declared rectified; rectify it upstream or "
                "explicitly set require_rectified_color=false for capture-only use"
            )
        if self.reference_intrinsics is None:
            self.reference_intrinsics = intrinsics
        else:
            comparable = (
                "width", "height", "fx", "fy", "cx", "cy",
                "distortion_model", "distortion_coefficients", "frame_id", "rectified"
            )
            if any(
                getattr(intrinsics, field) != getattr(self.reference_intrinsics, field)
                for field in comparable
            ):
                raise ValueError("CameraInfo calibration changed during capture")
        return intrinsics

    def _process_pair(self, pair):
        try:
            if self.camera_info is None:
                self.counters["dropped_missing_camera_info"] += 1
                return
            color_raw = self.bridge.imgmsg_to_cv2(pair.color, desired_encoding="passthrough")
            depth_raw = self.bridge.imgmsg_to_cv2(pair.depth, desired_encoding="passthrough")
            color = color_to_rgb(color_raw, pair.color.encoding)
            override = float(self.parameters["depth_scale_override"])
            depth_m = depth_to_meters(
                depth_raw, pair.depth.encoding, self.parameters["depth_encoding_mode"],
                override if override > 0 else None,
            )
            filtered, depth_stats = filter_depth(
                depth_m, self.parameters["depth_min"], self.parameters["depth_max"]
            )
            if depth_stats["valid_ratio"] < self.parameters["minimum_valid_depth_ratio"]:
                self.counters["dropped_invalid_depth"] += 1
                return
            pose_result = self._pose_at_sensor_time(pair.color_timestamp_ns)
            if pose_result is None:
                self.counters["dropped_pose_timeout"] += 1
                return
            matrix, quaternion, pose_timestamp_ns, pose_source = pose_result
            accepted, reason, pose_metrics = check_pose_quality(
                self.pose_quality_config, self.previous_valid_pose_timestamp_ns,
                self.previous_valid_pose, pair.color_timestamp_ns, matrix
            )
            if not accepted:
                self.counters["dropped_invalid_transform"] += 1
                self.counters[f"dropped_pose_{reason.lower()}"] += 1
                return
            intrinsics = self._intrinsics()
            frame = RGBDFrame(
                timestamp_ns=pair.color_timestamp_ns,
                frame_index=len(self.writer.writer.rows),
                color=color, depth_m=depth_m.astype(np.float32, copy=False),
                camera_intrinsics=intrinsics, T_world_camera=matrix,
                pose_source=pose_source,
                color_frame_id=str(pair.color.header.frame_id),
                depth_frame_id=str(pair.depth.header.frame_id),
                camera_frame_id=self.parameters["camera_frame"],
                world_frame_id=self.parameters["world_frame"],
                color_timestamp_ns=pair.color_timestamp_ns,
                depth_timestamp_ns=pair.depth_timestamp_ns,
                pose_timestamp_ns=pose_timestamp_ns,
                depth_aligned_to_color=self.parameters["depth_aligned_to_color"],
                robot_id=self.parameters["robot_id"], sensor_id=self.parameters["sensor_id"],
                metadata={
                    "quaternion_xyzw": dict(zip(("x", "y", "z", "w"), map(float, quaternion))),
                    "quality": {"depth": depth_stats, "pose": pose_metrics},
                },
            )
            frame.validate(require_color_alignment=True)
            self.counters["frames_with_valid_pose"] += 1
            preview_points = None
            if (
                self.parameters["live_preview_enabled"]
                and self._preview_due(frame.timestamp_ns)
            ):
                preview_points = self._preview_points(frame)
                self.live_cloud_publisher.publish(
                    colored_cloud_message(
                        preview_points[0], preview_points[1],
                        self.parameters["world_frame"], pair.color.header.stamp,
                    )
                )
                self.last_preview_timestamp_ns = frame.timestamp_ns
            decision = decide_keyframe(
                self.keyframe_config, frame.timestamp_ns, frame.T_world_camera,
                self.previous_keyframe_timestamp_ns, self.previous_keyframe_pose
            )
            self.previous_valid_pose = matrix
            self.previous_valid_pose_timestamp_ns = frame.timestamp_ns
            if not decision.selected:
                self.counters[f"rejected_keyframe_{decision.rejection.lower()}"] += 1
                return
            sharpness = laplacian_variance(frame.color)
            self.writer.write(
                frame, decision, sharpness,
                self.parameters["depth_min"], self.parameters["depth_max"]
            )
            if self.first_frame is None:
                self.first_frame = frame
            self.previous_keyframe_pose = matrix
            self.previous_keyframe_timestamp_ns = frame.timestamp_ns
            self.counters["frames_saved"] += 1
            if self.parameters["live_preview_enabled"]:
                if preview_points is None:
                    preview_points = self._preview_points(frame)
                snapshot = self.preview.add_keyframe(
                    frame.timestamp_ns, preview_points[0], preview_points[1],
                    frame.T_world_camera,
                )
                self.latest_preview_snapshot = snapshot
                accumulated_points, accumulated_colors = self.preview.accumulated_arrays(
                    int(self.parameters["preview_min_observations"])
                )
                self.accumulated_cloud_publisher.publish(
                    colored_cloud_message(
                        accumulated_points, accumulated_colors,
                        self.parameters["world_frame"], pair.color.header.stamp,
                    )
                )
                self._append_path(frame, pair.color.header.stamp)
                self._publish_status_and_markers(
                    frame, pair.color.header.stamp, snapshot, depth_stats,
                    frame.rgb_depth_delta_ms, frame.pose_delta_ms,
                )
            self.get_logger().info(
                f"saved keyframe {frame.frame_index}: {'|'.join(decision.reasons)} "
                f"rgb-depth={frame.rgb_depth_delta_ms:.2f} ms "
                f"pose={frame.pose_delta_ms:.2f} ms"
            )
        except Exception as exc:
            self.counters["dropped_frame_exception"] += 1
            self.get_logger().error(
                f"skipping invalid RGB-D frame: {type(exc).__name__}: {exc}\n"
                f"{traceback.format_exc()}"
            )

    def _summary(self):
        counters = {**self.sync.statistics(), **dict(self.counters)}
        for field in (
            "received_color_frames", "received_depth_frames", "synchronized_rgbd_frames",
            "frames_with_valid_pose", "frames_saved", "dropped_rgb_depth_sync",
            "dropped_pose_timeout", "dropped_invalid_depth", "dropped_invalid_transform",
        ):
            counters.setdefault(field, 0)
        return {
            "pose_source": self.parameters["pose_source"],
            "world_frame": self.parameters["world_frame"],
            "camera_frame": self.parameters["camera_frame"],
            "robot_id": self.parameters["robot_id"],
            "sensor_id": self.parameters["sensor_id"],
            "configuration": self.parameters,
            "counters": counters,
            "first_frame": self.first_frame,
        }

    def finalize(self, complete=True):
        if self.finalized:
            return
        self.sync.flush()
        preview_path = str(self.parameters["preview_output_path"])
        if preview_path and self.parameters["live_preview_enabled"] and self.preview.preview:
            try:
                points, colors = self.preview.accumulated_arrays(
                    int(self.parameters["preview_min_observations"])
                )
                source_frame = self.parameters["world_frame"]
                target_frame = (
                    str(self.parameters["preview_output_frame"]) or source_frame
                )
                if target_frame != source_frame:
                    transform = self.tf_buffer.lookup_transform(
                        target_frame, source_frame, Time(),
                        timeout=Duration(seconds=self.parameters["tf_timeout_s"]),
                    )
                    value = transform.transform
                    matrix = transform_matrix(
                        np.array([
                            value.translation.x, value.translation.y,
                            value.translation.z,
                        ]),
                        np.array([
                            value.rotation.x, value.rotation.y,
                            value.rotation.z, value.rotation.w,
                        ]),
                    )
                    points = (
                        points @ matrix[:3, :3].T + matrix[:3, 3]
                    ).astype(np.float32)
                save_preview_archive(preview_path, points, colors, target_frame)
                self.get_logger().info(
                    f"saved map-aligned RGB-D preview to {preview_path}"
                )
            except Exception as exc:
                self.get_logger().error(
                    f"failed to save RGB-D preview archive {preview_path}: "
                    f"{type(exc).__name__}: {exc}"
                )
        self.writer.finalize(self._summary(), complete=complete)
        self.finalized = True
        if rclpy.ok():
            self.get_logger().info(f"Phase 4B dataset finalized at {self.writer.root}")

    def destroy_node(self):
        try:
            self.finalize(complete=bool(self.writer.writer.rows))
        finally:
            return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=2)
    try:
        node = Phase4BCaptureNode()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
