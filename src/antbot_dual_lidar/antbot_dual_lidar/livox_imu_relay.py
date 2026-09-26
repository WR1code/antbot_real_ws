"""Normalize Livox IMU units and assign a unique frame per physical device."""

from functools import partial

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


STANDARD_GRAVITY = 9.80665


def normalize_livox_imu(message, frame_id, acceleration_scale=STANDARD_GRAVITY):
    """Convert Livox acceleration from g to SI and mark orientation unknown."""
    message.header.frame_id = frame_id
    message.linear_acceleration.x *= acceleration_scale
    message.linear_acceleration.y *= acceleration_scale
    message.linear_acceleration.z *= acceleration_scale
    message.orientation.x = 0.0
    message.orientation.y = 0.0
    message.orientation.z = 0.0
    message.orientation.w = 0.0
    message.orientation_covariance[0] = -1.0
    return message


class LivoxImuRelay(Node):
    def __init__(self):
        super().__init__("antbot_livox_imu_relay")
        defaults = {
            "front_left": {
                "input_topic": "/livox/imu_192_168_1_116",
                "output_topic": "/antbot/lidar/front_left/imu_raw_si",
                "frame_id": "mid360_front_imu",
            },
            "rear_right": {
                "input_topic": "/livox/imu_192_168_1_139",
                "output_topic": "/antbot/lidar/rear_right/imu_raw_si",
                "frame_id": "mid360_rear_imu",
            },
        }
        self.declare_parameter("acceleration_scale", STANDARD_GRAVITY)
        self.acceleration_scale = float(
            self.get_parameter("acceleration_scale").value
        )
        if self.acceleration_scale <= 0.0:
            raise ValueError("acceleration_scale must be positive")
        self.streams = {}
        for name, values in defaults.items():
            for key, value in values.items():
                self.declare_parameter(f"{name}.{key}", value)
            input_topic = str(self.get_parameter(f"{name}.input_topic").value)
            output_topic = str(self.get_parameter(f"{name}.output_topic").value)
            frame_id = str(self.get_parameter(f"{name}.frame_id").value)
            if not all((input_topic, output_topic, frame_id)):
                raise ValueError(f"{name}: topic and frame parameters are required")
            if input_topic == output_topic:
                raise ValueError(f"{name}: input and output topics must differ")
            publisher = self.create_publisher(
                Imu, output_topic, qos_profile_sensor_data
            )
            self.streams[name] = {"publisher": publisher, "frame_id": frame_id}
            self.create_subscription(
                Imu,
                input_topic,
                partial(self._relay, name),
                qos_profile_sensor_data,
            )
            self.get_logger().info(
                f"{name}: {input_topic} -> {output_topic} frame_id={frame_id}; "
                f"acceleration x{self.acceleration_scale:g}"
            )

    def _relay(self, name, message):
        stream = self.streams[name]
        stream["publisher"].publish(normalize_livox_imu(
            message, stream["frame_id"], self.acceleration_scale
        ))


def main(args=None):
    rclpy.init(args=args)
    node = LivoxImuRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
