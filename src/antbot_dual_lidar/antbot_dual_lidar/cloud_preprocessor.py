"""Transform and independently filter two PointCloud2 streams."""

from functools import partial
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener

from .filter_core import FilterConfig, filter_xyz, transform_xyz


class CloudPreprocessor(Node):
    def __init__(self):
        super().__init__("antbot_dual_lidar_preprocessor")
        self.declare_parameter("lidar_profile", "mapping")
        self.lidar_profile = str(self.get_parameter("lidar_profile").value)
        profile_defaults = {
            "mapping": {"voxel_leaf_size": 0.03, "max_range": 20.0},
            "navigation": {"voxel_leaf_size": 0.04, "max_range": 12.0},
        }
        if self.lidar_profile not in profile_defaults:
            raise ValueError(f"unsupported lidar_profile: {self.lidar_profile}")
        defaults = profile_defaults[self.lidar_profile]
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("tf_timeout_sec", 0.08)
        self.declare_parameter("input_timeout_sec", 1.5)
        self.declare_parameter("front_left.input_topic", "/antbot/lidar/front_left/points")
        self.declare_parameter("front_left.output_topic", "/antbot/lidar/front_left/points_filtered")
        self.declare_parameter("rear_right.input_topic", "/antbot/lidar/rear_right/points")
        self.declare_parameter("rear_right.output_topic", "/antbot/lidar/rear_right/points_filtered")
        for prefix in ("front_left", "rear_right"):
            for name, default in (
                ("min_range", 0.15), ("max_range", defaults["max_range"]),
                ("min_height", 0.02), ("max_height", 2.0),
                ("voxel_leaf_size", defaults["voxel_leaf_size"]),
            ):
                self.declare_parameter(f"{prefix}.{name}", default)
        for name, default in (
            ("enabled", True),
            ("min_x", -0.48), ("max_x", 0.48),
            ("min_y", -0.34), ("max_y", 0.34),
            ("min_z", -0.10), ("max_z", 0.50),
        ):
            self.declare_parameter(f"body_filter.{name}", default)

        self.target_frame = str(self.get_parameter("target_frame").value)
        self.tf_timeout = float(self.get_parameter("tf_timeout_sec").value)
        self.input_timeout = float(self.get_parameter("input_timeout_sec").value)
        if not self.target_frame or self.tf_timeout <= 0.0 or self.input_timeout <= 0.0:
            raise ValueError("target_frame must be set and timeout parameters must be positive")
        self.tf_buffer = Buffer(cache_time=Duration(seconds=15.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.streams = {}
        for prefix in ("front_left", "rear_right"):
            config = self._config(prefix)
            output = str(self.get_parameter(f"{prefix}.output_topic").value)
            input_topic = str(self.get_parameter(f"{prefix}.input_topic").value)
            if not input_topic or not output:
                raise ValueError(f"{prefix} topics must not be empty")
            publisher = self.create_publisher(PointCloud2, output, qos_profile_sensor_data)
            subscription = self.create_subscription(
                PointCloud2, input_topic, partial(self._cloud, prefix),
                qos_profile_sensor_data,
            )
            self.streams[prefix] = {
                "config": config, "publisher": publisher, "subscription": subscription,
                "last_wall": None, "last_stamp": None, "received": 0, "published": 0,
            }
        self.get_logger().info(
            f"lidar_profile={self.lidar_profile} target_frame={self.target_frame}"
        )
        self.create_timer(max(0.1, self.input_timeout / 2.0), self._check_timeouts)

    def _config(self, prefix):
        value = lambda name: self.get_parameter(name).value
        config = FilterConfig(
            min_range=float(value(f"{prefix}.min_range")),
            max_range=float(value(f"{prefix}.max_range")),
            min_height=float(value(f"{prefix}.min_height")),
            max_height=float(value(f"{prefix}.max_height")),
            voxel_leaf_size=float(value(f"{prefix}.voxel_leaf_size")),
            body_filter_enabled=bool(value("body_filter.enabled")),
            body_min_x=float(value("body_filter.min_x")),
            body_max_x=float(value("body_filter.max_x")),
            body_min_y=float(value("body_filter.min_y")),
            body_max_y=float(value("body_filter.max_y")),
            body_min_z=float(value("body_filter.min_z")),
            body_max_z=float(value("body_filter.max_z")),
        )
        config.validate()
        return config

    def _lookup(self, frame_id, stamp):
        return self.tf_buffer.lookup_transform(
            self.target_frame, frame_id, Time.from_msg(stamp),
            timeout=Duration(seconds=self.tf_timeout),
        ).transform

    def _cloud(self, prefix, msg):
        state = self.streams[prefix]
        state["received"] += 1
        state["last_wall"] = time.monotonic()
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if state["last_stamp"] is not None and stamp_ns < state["last_stamp"]:
            self.get_logger().warning(
                f"{prefix}: timestamp moved backwards (simulation reset accepted)",
                throttle_duration_sec=2.0,
            )
        state["last_stamp"] = stamp_ns
        if not msg.header.frame_id:
            self.get_logger().warning(f"{prefix}: empty frame_id", throttle_duration_sec=2.0)
            return
        names = {field.name for field in msg.fields}
        if not {"x", "y", "z"}.issubset(names):
            self.get_logger().warning(f"{prefix}: PointCloud2 has no XYZ fields", throttle_duration_sec=2.0)
            return
        try:
            transform = self._lookup(msg.header.frame_id, msg.header.stamp)
            input_fields = ["x", "y", "z"]
            if "intensity" in names:
                input_fields.append("intensity")
            structured = point_cloud2.read_points(msg, field_names=input_fields, skip_nans=False)
            xyz = np.column_stack((structured["x"], structured["y"], structured["z"]))
            q = transform.rotation
            t = transform.translation
            xyz = transform_xyz(xyz, (t.x, t.y, t.z), (q.x, q.y, q.z, q.w))
            filtered, stats, indices = filter_xyz(
                xyz, state["config"], return_indices=True
            )
        except (TransformException, ValueError, AssertionError) as error:
            self.get_logger().warning(
                f"{prefix}: cloud skipped: {error}", throttle_duration_sec=2.0
            )
            return

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = self.target_frame
        if "intensity" in structured.dtype.names:
            fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            values = np.column_stack(
                (filtered, np.asarray(structured["intensity"][indices], dtype=np.float32))
            ).astype(np.float32)
            output = point_cloud2.create_cloud(header, fields, values)
        else:
            output = point_cloud2.create_cloud_xyz32(header, filtered)
        output.is_dense = True
        state["publisher"].publish(output)
        state["published"] += 1
        if state["published"] == 1:
            self.get_logger().info(
                f"{prefix}: first cloud input={stats['input']} output={stats['output']} "
                f"body_removed={stats['body']} frame={self.target_frame}"
            )

    def _check_timeouts(self):
        now = time.monotonic()
        for prefix, state in self.streams.items():
            last = state["last_wall"]
            if last is None or now - last > self.input_timeout:
                age = "never" if last is None else f"{now - last:.2f}s"
                self.get_logger().warning(
                    f"{prefix}: input timeout (last={age})", throttle_duration_sec=2.0
                )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CloudPreprocessor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
