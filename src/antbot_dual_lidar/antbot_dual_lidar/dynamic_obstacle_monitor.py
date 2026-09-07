"""Extract persistent live obstacles that are absent from the saved map."""

from functools import partial
import json
import math
import time

from antbot_rgbd_dataset.pointcloud2 import colored_cloud_message
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformException, TransformListener

from .filter_core import transform_xyz


TRANSIENT_QOS = QoSProfile(depth=1)
TRANSIENT_QOS.reliability = ReliabilityPolicy.RELIABLE
TRANSIENT_QOS.durability = DurabilityPolicy.TRANSIENT_LOCAL


def quaternion_yaw(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


class DynamicObstacleMonitor(Node):
    """Compare lidar and RGB-D points against static 2D/3D map evidence."""

    def __init__(self):
        super().__init__("antbot_dynamic_obstacle_monitor")
        self.declare_parameter("voxel_size", 0.12)
        self.declare_parameter("confirmation_hits", 3)
        self.declare_parameter("persistence_sec", 2.5)
        self.declare_parameter("visual_stride", 6)
        self.declare_parameter("min_height", 0.06)
        self.declare_parameter("max_height", 1.9)
        self.declare_parameter("max_range", 6.0)
        self.voxel_size = float(self.get_parameter("voxel_size").value)
        self.confirmation_hits = int(self.get_parameter("confirmation_hits").value)
        self.persistence_sec = float(self.get_parameter("persistence_sec").value)
        self.visual_stride = int(self.get_parameter("visual_stride").value)
        self.min_height = float(self.get_parameter("min_height").value)
        self.max_height = float(self.get_parameter("max_height").value)
        self.max_range = float(self.get_parameter("max_range").value)
        if self.voxel_size <= 0.0 or self.confirmation_hits <= 0:
            raise ValueError("voxel_size and confirmation_hits must be positive")

        self.map = None
        self.static_sets = {"lidar": set(), "visual": set()}
        self.static_voxels = set()
        self.static_shapes = {}
        self.tracks = {"lidar": {}, "visual": {}}
        self.latest_color = None
        self.latest_camera_info = None
        self.tf_buffer = Buffer(cache_time=Duration(seconds=20.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.lidar_pub = self.create_publisher(
            PointCloud2, "/antbot/dynamic_obstacles/lidar_points", qos_profile_sensor_data
        )
        self.visual_pub = self.create_publisher(
            PointCloud2, "/antbot/dynamic_obstacles/visual_points", qos_profile_sensor_data
        )
        self.status_pub = self.create_publisher(
            String, "/antbot/dynamic_obstacles/status", TRANSIENT_QOS
        )
        self.create_subscription(OccupancyGrid, "/map", self.map_callback, TRANSIENT_QOS)
        self.create_subscription(
            PointCloud2, "/antbot/offline_map_points",
            partial(self.static_cloud_callback, "lidar"), TRANSIENT_QOS,
        )
        self.create_subscription(
            PointCloud2, "/antbot/rgbd/offline_cloud",
            partial(self.static_cloud_callback, "visual"), TRANSIENT_QOS,
        )
        for topic in (
            "/antbot/lidar/front_left/points_filtered",
            "/antbot/lidar/rear_right/points_filtered",
        ):
            self.create_subscription(
                PointCloud2, topic, self.live_lidar_callback, qos_profile_sensor_data
            )
        self.create_subscription(
            Image, "/antbot/camera/color/image_raw", self.color_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo, "/antbot/camera/depth/camera_info", self.camera_info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image, "/antbot/camera/depth/image_raw", self.depth_callback,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0, self.publish_status)

    def map_callback(self, message):
        self.map = message

    def xyz_from_cloud(self, message):
        names = {field.name for field in message.fields}
        if not {"x", "y", "z"}.issubset(names):
            raise ValueError("PointCloud2 has no XYZ fields")
        structured = point_cloud2.read_points(
            message, field_names=("x", "y", "z"), skip_nans=True
        )
        return np.column_stack(
            (structured["x"], structured["y"], structured["z"])
        ).astype(np.float64, copy=False)

    def voxel_keys(self, points):
        if not len(points):
            return np.empty((0, 3), dtype=np.int64)
        return np.floor(points / self.voxel_size).astype(np.int64)

    def static_cloud_callback(self, source, message):
        signature = (message.width, message.height, message.point_step, len(message.data))
        if self.static_shapes.get(source) == signature and self.static_sets[source]:
            return
        try:
            points = self.xyz_from_cloud(message)
            keys = np.unique(self.voxel_keys(points), axis=0)
            self.static_sets[source] = {tuple(key) for key in keys}
            self.static_voxels = self.static_sets["lidar"] | self.static_sets["visual"]
            self.static_shapes[source] = signature
            self.get_logger().info(
                f"Loaded {len(self.static_sets[source])} static {source} voxels"
            )
        except (ValueError, KeyError) as error:
            self.get_logger().warning(f"Static {source} cloud ignored: {error}")

    def lookup_transform(self, frame_id, stamp):
        return self.tf_buffer.lookup_transform(
            "map", frame_id, Time.from_msg(stamp),
            timeout=Duration(seconds=0.08),
        ).transform

    def transform_points(self, points, frame_id, stamp):
        transform = self.lookup_transform(frame_id, stamp)
        translation = transform.translation
        rotation = transform.rotation
        return transform_xyz(
            points,
            (translation.x, translation.y, translation.z),
            (rotation.x, rotation.y, rotation.z, rotation.w),
        )

    def map_free_mask(self, points):
        if self.map is None or not len(points):
            return np.zeros(len(points), dtype=bool)
        origin = self.map.info.origin
        yaw = quaternion_yaw(origin.orientation)
        dx = points[:, 0] - origin.position.x
        dy = points[:, 1] - origin.position.y
        cosine, sine = math.cos(yaw), math.sin(yaw)
        gx = np.floor(
            (cosine * dx + sine * dy) / self.map.info.resolution
        ).astype(np.int64)
        gy = np.floor(
            (-sine * dx + cosine * dy) / self.map.info.resolution
        ).astype(np.int64)
        inside = (
            (gx >= 0) & (gy >= 0)
            & (gx < self.map.info.width) & (gy < self.map.info.height)
        )
        result = np.zeros(len(points), dtype=bool)
        indices = np.flatnonzero(inside)
        if len(indices):
            data = np.asarray(self.map.data, dtype=np.int16)
            values = data[gy[indices] * self.map.info.width + gx[indices]]
            result[indices] = (values >= 0) & (values <= 20)
        return result

    def absent_from_static_mask(self, keys):
        static = self.static_voxels
        if not static:
            return np.zeros(len(keys), dtype=bool)
        offsets = (
            (0, 0, 0), (1, 0, 0), (-1, 0, 0),
            (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1),
        )
        result = []
        for key in keys:
            base = tuple(int(value) for value in key)
            nearby = any(
                (base[0] + dx, base[1] + dy, base[2] + dz) in static
                for dx, dy, dz in offsets
            )
            result.append(not nearby)
        return np.asarray(result, dtype=bool)

    def dynamic_candidates(self, points):
        if not len(points):
            return points, np.empty((0, 3), dtype=np.int64), np.empty(0, dtype=np.int64)
        finite = np.isfinite(points).all(axis=1)
        height = (points[:, 2] >= self.min_height) & (points[:, 2] <= self.max_height)
        filtered = points[finite & height]
        original_indices = np.flatnonzero(finite & height)
        if not len(filtered):
            return filtered, np.empty((0, 3), dtype=np.int64), original_indices
        keys = self.voxel_keys(filtered)
        _, unique_indices = np.unique(keys, axis=0, return_index=True)
        filtered = filtered[unique_indices]
        keys = keys[unique_indices]
        original_indices = original_indices[unique_indices]
        dynamic = self.map_free_mask(filtered) & self.absent_from_static_mask(keys)
        return filtered[dynamic], keys[dynamic], original_indices[dynamic]

    def update_tracks(self, source, points, keys, colors=None):
        now = time.monotonic()
        tracks = self.tracks[source]
        for index, (point, key_array) in enumerate(zip(points, keys)):
            key = tuple(int(value) for value in key_array)
            track = tracks.get(key)
            if track is None or now - track["last"] > 0.8:
                track = {"hits": 0, "last": now, "point": point}
                tracks[key] = track
            track["hits"] += 1
            track["last"] = now
            track["point"] = point
            if colors is not None:
                track["color"] = colors[index]
        expired = [
            key for key, track in tracks.items()
            if now - track["last"] > self.persistence_sec
        ]
        for key in expired:
            del tracks[key]
        confirmed = [
            track for track in tracks.values()
            if track["hits"] >= self.confirmation_hits
            and now - track["last"] <= self.persistence_sec
        ]
        return confirmed

    def live_lidar_callback(self, message):
        if self.map is None or not any(self.static_sets.values()):
            return
        try:
            points = self.xyz_from_cloud(message)
            points = self.transform_points(points, message.header.frame_id, message.header.stamp)
            candidates, keys, _ = self.dynamic_candidates(points)
            confirmed = self.update_tracks("lidar", candidates, keys)
            header = Header()
            header.stamp = message.header.stamp
            header.frame_id = "map"
            output_points = [track["point"] for track in confirmed]
            self.lidar_pub.publish(point_cloud2.create_cloud_xyz32(header, output_points))
        except (TransformException, ValueError, KeyError) as error:
            self.get_logger().warning(
                f"Live lidar comparison skipped: {error}", throttle_duration_sec=2.0
            )

    def color_callback(self, message):
        self.latest_color = message

    def camera_info_callback(self, message):
        self.latest_camera_info = message

    @staticmethod
    def image_array(message):
        if message.encoding == "32FC1":
            return np.ndarray(
                (message.height, message.width), dtype=np.float32,
                buffer=message.data, strides=(message.step, 4),
            )
        if message.encoding in ("16UC1", "mono16"):
            return np.ndarray(
                (message.height, message.width), dtype=np.uint16,
                buffer=message.data, strides=(message.step, 2),
            ).astype(np.float32) * 0.001
        raise ValueError(f"unsupported depth encoding: {message.encoding}")

    @staticmethod
    def color_array(message):
        if message.encoding not in ("rgb8", "bgr8"):
            raise ValueError(f"unsupported color encoding: {message.encoding}")
        array = np.ndarray(
            (message.height, message.width, 3), dtype=np.uint8,
            buffer=message.data, strides=(message.step, 3, 1),
        )
        return array[:, :, ::-1] if message.encoding == "bgr8" else array

    def depth_callback(self, message):
        color_message = self.latest_color
        info = self.latest_camera_info
        if (
            self.map is None or not any(self.static_sets.values())
            or color_message is None or info is None
        ):
            return
        try:
            depth = self.image_array(message)
            color = self.color_array(color_message)
            if depth.shape != color.shape[:2] or info.width != message.width:
                return
            rows = np.arange(0, message.height, self.visual_stride)
            columns = np.arange(0, message.width, self.visual_stride)
            uu, vv = np.meshgrid(columns, rows)
            z = depth[vv, uu]
            valid = np.isfinite(z) & (z > 0.20) & (z <= self.max_range)
            z = z[valid]
            u = uu[valid].astype(np.float64)
            v = vv[valid].astype(np.float64)
            fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
            camera_points = np.column_stack(
                ((u - cx) * z / fx, (v - cy) * z / fy, z)
            )
            colors = color[vv[valid], uu[valid]]
            map_points = self.transform_points(
                camera_points, message.header.frame_id, message.header.stamp
            )
            candidates, keys, selected = self.dynamic_candidates(map_points)
            selected_colors = colors[selected]
            confirmed = self.update_tracks(
                "visual", candidates, keys, selected_colors
            )
            points = np.asarray([track["point"] for track in confirmed], dtype=np.float32)
            output_colors = np.asarray(
                [track.get("color", (255, 0, 255)) for track in confirmed],
                dtype=np.uint8,
            )
            if points.size == 0:
                points = np.empty((0, 3), dtype=np.float32)
                output_colors = np.empty((0, 3), dtype=np.uint8)
            self.visual_pub.publish(
                colored_cloud_message(
                    points, output_colors, "map", message.header.stamp
                )
            )
        except (TransformException, ValueError, KeyError, IndexError) as error:
            self.get_logger().warning(
                f"RGB-D comparison skipped: {error}", throttle_duration_sec=2.0
            )

    def publish_status(self):
        now = time.monotonic()
        payload = {
            "static_lidar_voxels": len(self.static_sets["lidar"]),
            "static_visual_voxels": len(self.static_sets["visual"]),
            "lidar_dynamic_voxels": sum(
                track["hits"] >= self.confirmation_hits
                and now - track["last"] <= self.persistence_sec
                for track in self.tracks["lidar"].values()
            ),
            "visual_dynamic_voxels": sum(
                track["hits"] >= self.confirmation_hits
                and now - track["last"] <= self.persistence_sec
                for track in self.tracks["visual"].values()
            ),
            "confirmation_hits": self.confirmation_hits,
        }
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacleMonitor()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        # Lidar/RGB-D comparisons stay serialized in the default callback
        # group, while TransformListener's reentrant group uses the second
        # thread.  This prevents image/cloud processing from starving /tf.
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            executor.shutdown()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
