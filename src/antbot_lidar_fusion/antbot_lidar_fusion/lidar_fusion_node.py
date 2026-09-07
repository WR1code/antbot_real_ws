#!/usr/bin/env python3
"""Transform two LaserScan messages to base_link and publish one point cloud."""

import math
from typing import Iterable

from message_filters import ApproximateTimeSynchronizer, Subscriber
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener


def scan_xy(scan: LaserScan) -> list[tuple[float, float, float]]:
    """Return valid finite scan samples in the scan frame."""
    points = []
    angle = scan.angle_min
    for distance in scan.ranges:
        if math.isfinite(distance) and scan.range_min <= distance <= scan.range_max:
            points.append((distance * math.cos(angle), distance * math.sin(angle), 0.0))
        angle += scan.angle_increment
    return points


def transform_xy(
    points: Iterable[tuple[float, float, float]], transform
) -> list[tuple[float, float, float]]:
    """Apply a geometry_msgs Transform (full quaternion) to XYZ points."""
    q = transform.rotation
    tx, ty, tz = transform.translation.x, transform.translation.y, transform.translation.z
    # Quaternion to rotation matrix.
    xx, yy, zz = q.x * q.x, q.y * q.y, q.z * q.z
    xy, xz, yz = q.x * q.y, q.x * q.z, q.y * q.z
    wx, wy, wz = q.w * q.x, q.w * q.y, q.w * q.z
    r00, r01, r02 = 1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)
    r10, r11, r12 = 2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)
    r20, r21, r22 = 2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)
    return [
        (
            r00 * x + r01 * y + r02 * z + tx,
            r10 * x + r11 * y + r12 * z + ty,
            r20 * x + r21 * y + r22 * z + tz,
        )
        for x, y, z in points
    ]


class LidarFusionNode(Node):
    """Synchronize, transform, self-filter, and combine two scans."""

    def __init__(self) -> None:
        super().__init__("lidar_fusion_node")
        self.declare_parameter("front_scan_topic", "/scan_0")
        self.declare_parameter("rear_scan_topic", "/scan_1")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("body_min_x", -0.43)
        self.declare_parameter("body_max_x", 0.43)
        self.declare_parameter("body_min_y", -0.29)
        self.declare_parameter("body_max_y", 0.29)

        self.target_frame = str(self.get_parameter("target_frame").value)
        self.bounds = tuple(
            float(self.get_parameter(name).value)
            for name in ("body_min_x", "body_max_x", "body_min_y", "body_max_y")
        )
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(
            PointCloud2, "/antbot/lidar/combined_points", qos_profile_sensor_data
        )
        self.front_sub = Subscriber(
            self,
            LaserScan,
            str(self.get_parameter("front_scan_topic").value),
            qos_profile=qos_profile_sensor_data,
        )
        self.rear_sub = Subscriber(
            self,
            LaserScan,
            str(self.get_parameter("rear_scan_topic").value),
            qos_profile=qos_profile_sensor_data,
        )
        self.sync = ApproximateTimeSynchronizer(
            [self.front_sub, self.rear_sub], queue_size=10, slop=0.10
        )
        self.sync.registerCallback(self.synchronized_callback)
        self.last_log_ns = 0
        self.get_logger().info(
            f"Combining scans in {self.target_frame}; self-filter bounds={self.bounds}"
        )

    def _lookup(self, scan: LaserScan):
        stamp = Time.from_msg(scan.header.stamp)
        try:
            return self.tf_buffer.lookup_transform(
                self.target_frame,
                scan.header.frame_id,
                stamp,
                timeout=Duration(seconds=0.05),
            ).transform
        except TransformException:
            # Static transforms are valid at all times. This fallback also tolerates
            # a just-started simulation whose scan stamp precedes the TF cache.
            return self.tf_buffer.lookup_transform(
                self.target_frame,
                scan.header.frame_id,
                Time(),
                timeout=Duration(seconds=0.05),
            ).transform

    def synchronized_callback(self, front: LaserScan, rear: LaserScan) -> None:
        """Process a synchronized scan pair without terminating on transient TF errors."""
        if not front.header.frame_id or not rear.header.frame_id:
            self.get_logger().warning("Received a scan with an empty frame_id", throttle_duration_sec=2.0)
            return
        try:
            front_local = scan_xy(front)
            rear_local = scan_xy(rear)
            front_base = transform_xy(front_local, self._lookup(front))
            rear_base = transform_xy(rear_local, self._lookup(rear))
        except TransformException as error:
            self.get_logger().warning(
                f"TF temporarily unavailable: {error}", throttle_duration_sec=2.0
            )
            return

        min_x, max_x, min_y, max_y = self.bounds
        all_points = front_base + rear_base
        kept = [
            point
            for point in all_points
            if not (min_x <= point[0] <= max_x and min_y <= point[1] <= max_y)
        ]
        filtered = len(all_points) - len(kept)
        header = front.header
        header.frame_id = self.target_frame
        # Use the newer of the two input stamps.
        if (rear.header.stamp.sec, rear.header.stamp.nanosec) > (
            front.header.stamp.sec,
            front.header.stamp.nanosec,
        ):
            header.stamp = rear.header.stamp
        self.publisher.publish(point_cloud2.create_cloud_xyz32(header, kept))

        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_log_ns >= 5_000_000_000 or now_ns < self.last_log_ns:
            front_t = front.header.stamp.sec + front.header.stamp.nanosec * 1e-9
            rear_t = rear.header.stamp.sec + rear.header.stamp.nanosec * 1e-9
            self.get_logger().info(
                f"front={len(front_local)} rear={len(rear_local)} "
                f"body_filtered={filtered} combined={len(kept)} "
                f"stamp_delta={abs(front_t - rear_t):.4f}s"
            )
            self.last_log_ns = now_ns


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
