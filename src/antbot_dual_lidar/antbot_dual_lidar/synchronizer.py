"""Dual-cloud pairing diagnostics with optional identity-preserving outputs."""

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

from .sync_core import SyncMatcher


class DualCloudSynchronizer(Node):
    def __init__(self):
        super().__init__("antbot_dual_lidar_synchronizer")
        self.declare_parameter("front_topic", "/antbot/lidar/front_left/points")
        self.declare_parameter("rear_topic", "/antbot/lidar/rear_right/points")
        self.declare_parameter(
            "front_output_topic", "/antbot/lidar/synchronized/front_left"
        )
        self.declare_parameter(
            "rear_output_topic", "/antbot/lidar/synchronized/rear_right"
        )
        self.declare_parameter("strategy", "approximate")
        self.declare_parameter("maximum_pair_delta_sec", 0.02)
        self.declare_parameter("expected_period_sec", 0.10)
        self.declare_parameter("queue_size", 20)
        self.declare_parameter("publish_synchronized", False)
        self.declare_parameter("report_period_sec", 5.0)
        max_delta = float(self.get_parameter("maximum_pair_delta_sec").value)
        expected = float(self.get_parameter("expected_period_sec").value)
        self.matcher = SyncMatcher(
            strategy=str(self.get_parameter("strategy").value),
            max_delta_ns=int(max_delta * 1e9),
            queue_size=int(self.get_parameter("queue_size").value),
            expected_period_ns=int(expected * 1e9),
        )
        self.publish_enabled = bool(
            self.get_parameter("publish_synchronized").value
        )
        self.front_publisher = None
        self.rear_publisher = None
        if self.publish_enabled:
            self.front_publisher = self.create_publisher(
                PointCloud2,
                str(self.get_parameter("front_output_topic").value),
                qos_profile_sensor_data,
            )
            self.rear_publisher = self.create_publisher(
                PointCloud2,
                str(self.get_parameter("rear_output_topic").value),
                qos_profile_sensor_data,
            )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("front_topic").value),
            lambda message: self._cloud("front", message),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("rear_topic").value),
            lambda message: self._cloud("rear", message),
            qos_profile_sensor_data,
        )
        self.create_timer(
            float(self.get_parameter("report_period_sec").value), self._report
        )

    @staticmethod
    def _stamp_ns(message):
        return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec

    def _cloud(self, side, message):
        pair = self.matcher.add(side, self._stamp_ns(message), message)
        if pair is None:
            return
        front, rear, _delta_ns = pair
        if self.publish_enabled:
            # Publish the original messages unchanged. The topics retain sensor
            # identity and each message retains its own acquisition stamp.
            self.front_publisher.publish(front)
            self.rear_publisher.publish(rear)

    def _report(self):
        self.get_logger().info(
            "SYNC_STATS " + json.dumps(self.matcher.stats.summary(), sort_keys=True)
        )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DualCloudSynchronizer()
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

