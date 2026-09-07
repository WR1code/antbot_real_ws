"""Publish a Step 1 RGB-D preview archive for Steps 2 and 3."""

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import rclpy
from sensor_msgs.msg import PointCloud2

from .live_preview import load_preview_archive
from .pointcloud2 import colored_cloud_message


class OfflinePreviewPublisher(Node):
    """Latched publisher for a saved, map-aligned colored point cloud."""

    def __init__(self):
        super().__init__("antbot_rgbd_offline_cloud_publisher")
        self.declare_parameter("preview_path", "")
        self.declare_parameter("publish_topic", "/antbot/rgbd/offline_cloud")
        self.declare_parameter("publish_rate", 0.5)
        preview_path = str(self.get_parameter("preview_path").value)
        if not preview_path:
            raise ValueError("preview_path is required")
        self.points, self.colors, self.frame_id = load_preview_archive(preview_path)
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        topic = str(self.get_parameter("publish_topic").value)
        self.publisher = self.create_publisher(PointCloud2, topic, qos)
        rate = float(self.get_parameter("publish_rate").value)
        if rate <= 0:
            raise ValueError("publish_rate must be positive")
        self.timer = self.create_timer(1.0 / rate, self._publish)
        self._publish()
        self.get_logger().info(
            f"loaded {len(self.points)} RGB-D preview points from {preview_path}; "
            f"publishing {topic} in {self.frame_id}"
        )

    def _publish(self):
        self.publisher.publish(
            colored_cloud_message(
                self.points, self.colors, self.frame_id, self.get_clock().now().to_msg()
            )
        )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = OfflinePreviewPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
