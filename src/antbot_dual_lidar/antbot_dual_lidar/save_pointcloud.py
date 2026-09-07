"""Explicit one-shot saver for the latest transient-local accumulated cloud."""

import argparse
from pathlib import Path
import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener

from .pcd_io import save_pcd, write_cloud_metadata, write_manifest


def _arguments(args):
    parser = argparse.ArgumentParser(
        description="Save the latest accumulated ANTBot PointCloud2 as binary PCD"
    )
    parser.add_argument("--map-name", required=True)
    parser.add_argument(
        "--output-directory",
        help="exact map bundle directory (default: artifacts/maps/<map-name>)",
    )
    parser.add_argument("--source-topic", default="/antbot/lidar/map_points")
    parser.add_argument("--navigation-frame", default="map")
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--voxel-size", type=float, default=0.05)
    parser.add_argument("--maximum-points", type=int, default=300000)
    return parser.parse_args(args)


class LatestCloudSaver(Node):
    def __init__(self, topic):
        super().__init__("antbot_pointcloud_saver")
        self.message = None
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.subscription = self.create_subscription(
            PointCloud2, topic, self._receive, qos
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _receive(self, message):
        self.message = message


def _stored_transform(node, message, target_frame):
    if target_frame == message.header.frame_id:
        return {
            "parent_frame": target_frame,
            "child_frame": message.header.frame_id,
            "translation": [0.0, 0.0, 0.0],
            "rotation_xyzw": [0.0, 0.0, 0.0, 1.0],
        }
    source_frame = message.header.frame_id
    cloud_time = Time.from_msg(message.header.stamp)
    try:
        stamped = node.tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            cloud_time,
            timeout=Duration(seconds=1.0),
        )
    except TransformException as error:
        # This saver starts only when Step1 exits.  The accumulated cloud is
        # transient-local and can therefore predate this node's TF buffer,
        # while map <- odom is still being published at the current time.
        # The final/latest transform is the correct alignment between the
        # odom-frame accumulated cloud and the final 2D SLAM map.
        node.get_logger().warning(
            f"cannot look up {target_frame} <- {source_frame} at cloud time "
            f"{message.header.stamp.sec}."
            f"{message.header.stamp.nanosec:09d}: {error}; "
            "falling back to the latest available transform"
        )
        try:
            stamped = node.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=1.0),
            )
        except TransformException as latest_error:
            node.get_logger().error(
                f"cannot store latest {target_frame} <- {source_frame} "
                f"transform: {latest_error}"
            )
            return None
    transform = stamped.transform
    return {
        "parent_frame": target_frame,
        "child_frame": source_frame,
        "translation": [
            float(transform.translation.x),
            float(transform.translation.y),
            float(transform.translation.z),
        ],
        "rotation_xyzw": [
            float(transform.rotation.x),
            float(transform.rotation.y),
            float(transform.rotation.z),
            float(transform.rotation.w),
        ],
    }


def main(args=None):
    raw_args = sys.argv[1:] if args is None else args
    rclpy.init(args=raw_args)
    parsed = _arguments(rclpy.utilities.remove_ros_args(args=raw_args))
    output = Path(
        parsed.output_directory
        or Path.cwd() / "artifacts" / "maps" / parsed.map_name
    ).expanduser().resolve()
    if parsed.timeout <= 0 or parsed.voxel_size <= 0 or parsed.maximum_points <= 0:
        raise ValueError("timeout, voxel-size, and maximum-points must be positive")
    node = LatestCloudSaver(parsed.source_topic)
    try:
        deadline = time.monotonic() + parsed.timeout
        while node.message is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.message is None:
            raise RuntimeError(
                f"no PointCloud2 received from {parsed.source_topic!r} "
                f"within {parsed.timeout:g} seconds"
            )
        message = node.message
        # Allow transient /tf_static callbacks queued with the cloud to run.
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.1)
        transform = _stored_transform(node, message, parsed.navigation_frame)
        if transform is None and parsed.navigation_frame != message.header.frame_id:
            node.get_logger().warning(
                "cloud will be saved, but offline publication in the navigation "
                "frame will be rejected until a valid transform is supplied"
            )
        output.mkdir(parents=True, exist_ok=True)
        summary = save_pcd(message, output / "cloud.pcd")
        metadata = write_cloud_metadata(
            output / "cloud_metadata.yaml",
            message,
            summary,
            source_topic=parsed.source_topic,
            voxel_size=parsed.voxel_size,
            maximum_points=parsed.maximum_points,
            transform_to_map=transform,
        )
        write_manifest(
            output,
            parsed.map_name,
            navigation_frame=parsed.navigation_frame,
            cloud_frame=message.header.frame_id,
            source_topic=parsed.source_topic,
            transform=transform,
            notes=metadata["warning"],
        )
        node.get_logger().info(
            f"saved {summary['point_count']} points to {output / 'cloud.pcd'}; "
            f"metadata: {output / 'cloud_metadata.yaml'}; "
            f"manifest: {output / 'manifest.yaml'}"
        )
    except Exception as error:
        node.get_logger().error(f"point cloud save failed: {error}")
        raise SystemExit(1) from error
    finally:
        node.destroy_node()
        rclpy.shutdown()
