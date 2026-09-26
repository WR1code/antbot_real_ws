"""Convert FAST-LIO's IMU-body odometry into the AntBot base_link contract."""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster


def quaternion_multiply(a, b):
    return (
        a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
        a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
        a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
        a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
    )


def quaternion_conjugate(q):
    return (-q[0], -q[1], -q[2], q[3])


def rotate_vector(q, v):
    rotated = quaternion_multiply(
        quaternion_multiply(q, (v[0], v[1], v[2], 0.0)),
        quaternion_conjugate(q),
    )
    return rotated[:3]


def base_pose_from_imu_pose(position, orientation, base_to_imu_translation):
    """Return T_odom_base from T_odom_imu and a translation-only T_base_imu."""
    norm = math.sqrt(sum(value * value for value in orientation))
    if norm == 0.0:
        raise ValueError("input orientation quaternion has zero norm")
    q = tuple(value / norm for value in orientation)
    offset = rotate_vector(q, base_to_imu_translation)
    return tuple(position[i] - offset[i] for i in range(3)), q


class LioOdomAdapter(Node):
    def __init__(self):
        super().__init__("antbot_lio_odom_adapter")
        self.declare_parameter("input_topic", "/Odometry")
        self.declare_parameter("output_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter(
            "base_to_imu_translation", [0.333, 0.24529, 0.36988]
        )
        self.translation = tuple(float(value) for value in self.get_parameter(
            "base_to_imu_translation"
        ).value)
        if len(self.translation) != 3:
            raise ValueError("base_to_imu_translation must contain three values")
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.publisher = self.create_publisher(
            Odometry, str(self.get_parameter("output_topic").value), 10
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Odometry,
            str(self.get_parameter("input_topic").value),
            self._callback,
            qos_profile_sensor_data,
        )

    def _callback(self, source):
        p = source.pose.pose.position
        q = source.pose.pose.orientation
        position, orientation = base_pose_from_imu_pose(
            (p.x, p.y, p.z), (q.x, q.y, q.z, q.w), self.translation
        )
        output = Odometry()
        output.header.stamp = source.header.stamp
        output.header.frame_id = self.odom_frame
        output.child_frame_id = self.base_frame
        output.pose = source.pose
        output.twist = source.twist
        output.pose.pose.position.x = position[0]
        output.pose.pose.position.y = position[1]
        output.pose.pose.position.z = position[2]
        output.pose.pose.orientation.x = orientation[0]
        output.pose.pose.orientation.y = orientation[1]
        output.pose.pose.orientation.z = orientation[2]
        output.pose.pose.orientation.w = orientation[3]
        self.publisher.publish(output)

        transform = TransformStamped()
        transform.header = output.header
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = position[0]
        transform.transform.translation.y = position[1]
        transform.transform.translation.z = position[2]
        transform.transform.rotation = output.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = LioOdomAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
