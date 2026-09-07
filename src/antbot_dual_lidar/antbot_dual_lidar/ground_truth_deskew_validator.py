"""Ground-truth-only native-scan deskew and world-reference node."""

from collections import defaultdict
import time
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from .deskew_core import (
    POINT_TIME_CONVENTIONS,
    PoseBuffer,
    deskew_points,
    point_times_ns,
    points_to_world,
)


STREAMS = {
    "front_left": {
        "input": "/antbot/lidar/front_left/points_raw_native",
        "output": "/antbot/lidar/front_left/points_deskew_truth",
        "world": "/antbot/lidar/front_left/points_world_reference",
        "translation": [0.322, 0.222, 0.414],
        "quaternion": [0.0, 0.0, 0.0, 1.0],
    },
    "rear_right": {
        "input": "/antbot/lidar/rear_right/points_raw_native",
        "output": "/antbot/lidar/rear_right/points_deskew_truth",
        "world": "/antbot/lidar/rear_right/points_world_reference",
        "translation": [-0.322, -0.222, 0.414],
        "quaternion": [0.0, 0.0, 1.0, 0.0],
    },
}


class GroundTruthDeskewValidator(Node):
    def __init__(self):
        super().__init__("ground_truth_deskew_validator")
        self.declare_parameter("ground_truth_topic", "/antbot/ground_truth/odom")
        self.declare_parameter("point_time_convention", "header_plus_offset")
        self.declare_parameter("scan_period_sec", 0.1)
        self.declare_parameter("pose_buffer_sec", 10.0)
        self.convention = self.get_parameter("point_time_convention").value
        if self.convention not in POINT_TIME_CONVENTIONS:
            raise ValueError(f"invalid point_time_convention: {self.convention}")
        self.scan_period = float(self.get_parameter("scan_period_sec").value)
        self.poses = PoseBuffer(float(self.get_parameter("pose_buffer_sec").value))
        self.counters = defaultdict(int)
        self.timings_ms = defaultdict(list)
        self.output_publishers = {}
        self.world_publishers = {}
        self.input_subscriptions = []
        truth_topic = self.get_parameter("ground_truth_topic").value
        self.input_subscriptions.append(
            self.create_subscription(
                Odometry, truth_topic, self._odom, qos_profile_sensor_data
            )
        )
        for name, config in STREAMS.items():
            self.output_publishers[name] = self.create_publisher(
                PointCloud2, config["output"], qos_profile_sensor_data
            )
            self.world_publishers[name] = self.create_publisher(
                PointCloud2, config["world"], qos_profile_sensor_data
            )
            self.input_subscriptions.append(
                self.create_subscription(
                    PointCloud2,
                    config["input"],
                    lambda message, stream=name: self._cloud(stream, message),
                    qos_profile_sensor_data,
                )
            )
        self.create_timer(10.0, self._report_performance)

    def _odom(self, message):
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        stamp = message.header.stamp
        self.poses.add(
            stamp.sec * 1_000_000_000 + stamp.nanosec,
            [position.x, position.y, position.z],
            [orientation.x, orientation.y, orientation.z, orientation.w],
        )

    def _cloud(self, name, message):
        started = time.perf_counter_ns()
        self.counters[f"{name}.input"] += 1
        fields = {field.name: field for field in message.fields}
        required = {"x", "y", "z", "time_offset_ns"}
        if not required.issubset(fields):
            self.get_logger().error(f"{name}: missing fields {sorted(required - fields)}")
            return
        if fields["time_offset_ns"].datatype != PointField.INT32:
            self.get_logger().error(
                f"{name}: time_offset_ns datatype={fields['time_offset_ns'].datatype}, expected INT32"
            )
            self.counters[f"{name}.semantic_reject"] += 1
            return
        names = ["x", "y", "z", "time_offset_ns"]
        include_intensity = "intensity" in fields
        if include_intensity:
            names.insert(3, "intensity")
        values = point_cloud2.read_points(message, field_names=names, skip_nans=False)
        parsed = time.perf_counter_ns()
        if len(values) == 0:
            self.output_publishers[name].publish(message)
            return
        xyz = np.column_stack((values["x"], values["y"], values["z"]))
        offsets = np.asarray(values["time_offset_ns"], dtype=np.int32)
        header_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        point_stamps = point_times_ns(
            header_ns, offsets, self.convention, self.scan_period
        )
        timed = time.perf_counter_ns()
        # Input coordinates are native per-ray sensor coordinates. Header is a
        # chosen common reference only for the truth-deskew output.
        reference_ns = header_ns
        config = STREAMS[name]
        corrected = deskew_points(
            xyz,
            point_stamps,
            reference_ns,
            self.poses,
            config["translation"],
            config["quaternion"],
        )
        transformed = time.perf_counter_ns()
        world = points_to_world(
            xyz,
            point_stamps,
            self.poses,
            config["translation"],
            config["quaternion"],
        )
        if corrected is None or world is None:
            self.counters[f"{name}.coverage_drop"] += 1
            self.get_logger().warning(
                f"{name}: truth does not cover complete scan; "
                f"dropped={self.counters[f'{name}.coverage_drop']}",
                throttle_duration_sec=2.0,
            )
            return
        header = Header()
        header.frame_id = message.header.frame_id
        header.stamp.sec = reference_ns // 1_000_000_000
        header.stamp.nanosec = reference_ns % 1_000_000_000
        dtype = [
            ("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
            ("intensity", "<f4"), ("time_offset_ns", "<i4"),
        ]
        output_values = np.empty(len(corrected), dtype=np.dtype(dtype))
        output_values["x"], output_values["y"], output_values["z"] = corrected.astype(np.float32).T
        output_values["intensity"] = (
            np.asarray(values["intensity"], dtype=np.float32)
            if include_intensity else np.zeros(len(corrected), dtype=np.float32)
        )
        output_values["time_offset_ns"] = offsets
        output_fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name="time_offset_ns", offset=16, datatype=PointField.INT32, count=1),
        ]
        output = point_cloud2.create_cloud(header, output_fields, output_values)
        output.is_dense = bool(np.isfinite(corrected).all())
        serialized = time.perf_counter_ns()
        self.output_publishers[name].publish(output)
        world_header = Header()
        world_header.frame_id = "odom"
        world_header.stamp = header.stamp
        world_values = output_values.copy()
        world_values["x"], world_values["y"], world_values["z"] = world.astype(np.float32).T
        world_output = point_cloud2.create_cloud(
            world_header, output_fields, world_values
        )
        world_output.is_dense = bool(np.isfinite(world).all())
        self.world_publishers[name].publish(world_output)
        published = time.perf_counter_ns()
        self.counters[f"{name}.output"] += 1
        for stage, lower, upper in (
            ("parse", started, parsed),
            ("point_time", parsed, timed),
            ("truth_interpolation_transform", timed, transformed),
            ("serialize", transformed, serialized),
            ("publish_calls", serialized, published),
            ("total", started, published),
        ):
            self.timings_ms[f"{name}.{stage}"].append((upper - lower) * 1e-6)

    def _report_performance(self):
        parts = []
        for name in STREAMS:
            values = np.asarray(self.timings_ms[f"{name}.total"], dtype=np.float64)
            if not len(values):
                continue
            parts.append(
                f"{name}:input={self.counters[f'{name}.input']} "
                f"output={self.counters[f'{name}.output']} "
                f"coverage_drop={self.counters[f'{name}.coverage_drop']} "
                f"mean_ms={values.mean():.3f} p95_ms={np.percentile(values,95):.3f} "
                f"p99_ms={np.percentile(values,99):.3f} max_ms={values.max():.3f}"
            )
        if parts:
            self.get_logger().info("DESKEW_PERFORMANCE " + " | ".join(parts))


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthDeskewValidator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
