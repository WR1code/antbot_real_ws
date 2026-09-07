"""Relay two Livox clouds onto stable AntBot topics with distinct TF frames.

livox_ros_driver2 uses one global ``frame_id`` parameter even when
``multi_topic`` is enabled.  AntBot needs one frame per physical sensor, so
this node changes only ``header.frame_id`` and republishes the original
PointCloud2 message.  All point fields and bytes, including Livox per-point
timestamps, tag, and line, remain untouched.
"""

from functools import partial

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


class LivoxFrameRelay(Node):
    """Assign distinct frames to the two IP-specific Livox point topics."""

    def __init__(self):
        super().__init__('antbot_livox_frame_relay')
        defaults = {
            'front_left': {
                'input_topic': '/livox/lidar_192_168_1_12',
                'output_topic': '/antbot/lidar/front_left/points_raw_native',
                'frame_id': 'lidar_2d_front_scan',
            },
            'rear_right': {
                'input_topic': '/livox/lidar_192_168_1_13',
                'output_topic': '/antbot/lidar/rear_right/points_raw_native',
                'frame_id': 'lidar_2d_back_scan',
            },
        }

        self.streams = {}
        for name, stream_defaults in defaults.items():
            for parameter, default in stream_defaults.items():
                self.declare_parameter(f'{name}.{parameter}', default)

            input_topic = str(
                self.get_parameter(f'{name}.input_topic').value
            )
            output_topic = str(
                self.get_parameter(f'{name}.output_topic').value
            )
            frame_id = str(self.get_parameter(f'{name}.frame_id').value)
            if not input_topic or not output_topic or not frame_id:
                raise ValueError(f'{name}: topics and frame_id must not be empty')
            if input_topic == output_topic:
                raise ValueError(f'{name}: input and output topics must differ')

            publisher = self.create_publisher(
                PointCloud2, output_topic, qos_profile_sensor_data
            )
            subscription = self.create_subscription(
                PointCloud2,
                input_topic,
                partial(self._relay, name),
                qos_profile_sensor_data,
            )
            self.streams[name] = {
                'frame_id': frame_id,
                'publisher': publisher,
                'subscription': subscription,
                'count': 0,
            }
            self.get_logger().info(
                f'{name}: {input_topic} -> {output_topic} frame_id={frame_id}'
            )

    def _relay(self, name, message):
        stream = self.streams[name]
        message.header.frame_id = stream['frame_id']
        stream['publisher'].publish(message)
        stream['count'] += 1
        if stream['count'] == 1:
            fields = ','.join(field.name for field in message.fields)
            self.get_logger().info(
                f'{name}: first cloud relayed; fields=[{fields}] '
                f'points={message.width * message.height}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LivoxFrameRelay()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
