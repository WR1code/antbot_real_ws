"""Publish a saved PCD in its frame or numerically transformed target frame."""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

from .pcd_io import (
    array_to_pointcloud2,
    load_metadata,
    load_pcd,
    transform_cloud,
)


class OfflinePointCloudPublisher(Node):
    def __init__(self):
        super().__init__("antbot_offline_pointcloud_publisher")
        self.declare_parameter("pointcloud_path", "")
        self.declare_parameter("metadata_path", "")
        self.declare_parameter("publish_topic", "/antbot/offline_map_points")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("publish_rate", 0.5)

        cloud_path = str(self.get_parameter("pointcloud_path").value)
        metadata_path = str(self.get_parameter("metadata_path").value)
        topic = str(self.get_parameter("publish_topic").value)
        target_frame = str(self.get_parameter("target_frame").value)
        rate = float(self.get_parameter("publish_rate").value)
        if not cloud_path or not metadata_path or not topic:
            raise ValueError(
                "pointcloud_path, metadata_path, and publish_topic are required"
            )
        if rate < 0:
            raise ValueError("publish_rate must be >= 0")

        array, fields = load_pcd(cloud_path)
        metadata = load_metadata(metadata_path)
        if int(metadata["point_count"]) != len(array):
            raise ValueError(
                "metadata point_count does not match PCD: "
                f"{metadata['point_count']} != {len(array)}"
            )
        source_frame = metadata["frame_id"]
        output_frame = target_frame or source_frame
        if output_frame != source_frame:
            array = transform_cloud(
                array,
                metadata.get("transform_to_map"),
                source_frame,
                output_frame,
            )
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(
            PointCloud2,
            topic,
            qos,
        )
        self.message = array_to_pointcloud2(array, fields, output_frame)
        self._published_once = False
        self.timer = self.create_timer(
            1.0 / rate if rate > 0 else 0.1, self._publish
        )
        self.publish_once = rate == 0
        self.get_logger().info(
            f"loaded {len(array)} points from {cloud_path}; publishing "
            f"{topic} in frame {output_frame!r} with transient-local QoS"
        )

    def _publish(self):
        self.message.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.message)
        if self.publish_once and not self._published_once:
            self._published_once = True
            self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = OfflinePointCloudPublisher()
        rclpy.spin(node)
    except Exception as error:
        if node is not None:
            node.get_logger().error(f"offline point cloud publisher failed: {error}")
        else:
            print(f"offline point cloud publisher failed: {error}")
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
