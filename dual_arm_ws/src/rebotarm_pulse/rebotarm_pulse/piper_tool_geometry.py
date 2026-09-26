"""Visualize and, after explicit calibration, publish the Piper-H pulse tool frames."""

from __future__ import annotations

import math

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


def _triple(values, name: str) -> tuple[float, float, float]:
    result = tuple(float(value) for value in values)
    if len(result) != 3 or not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} must contain three finite values")
    return result


def _quaternion_from_rpy(roll: float, pitch: float, yaw: float):
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _quaternion_align_z(direction):
    dx, dy, dz = _triple(direction, "contact_normal_xyz")
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length < 1e-6:
        raise ValueError("contact_normal_xyz must be non-zero when geometry_calibrated=true")
    dx, dy, dz = dx / length, dy / length, dz / length
    if dz < -0.999999:
        return (1.0, 0.0, 0.0, 0.0)
    qx, qy, qz, qw = -dy, dx, 0.0, 1.0 + dz
    qlen = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    return qx / qlen, qy / qlen, qz / qlen, qw / qlen


class PiperToolGeometry(Node):
    """Publish an approximate marker; calibrated sensor TFs require explicit opt-in."""

    def __init__(self) -> None:
        super().__init__("piper_tool_geometry")
        self.declare_parameter("parent_frame", "piperh/pulse_tool_envelope")
        self.declare_parameter("geometry_calibrated", False)
        self.declare_parameter("support_length_m", 0.11)
        self.declare_parameter("support_width_m", 0.025)
        self.declare_parameter("support_depth_m", 0.018)
        self.declare_parameter("approx_support_center_xyz", [0.0, 0.0, -0.055])
        self.declare_parameter("approx_support_rpy", [0.0, 0.0, 0.0])
        self.declare_parameter(
            "approx_sensor_offsets_m",
            [0.0, 0.010, -0.025, 0.0, 0.010, -0.055, 0.0, 0.010, -0.085],
        )
        self.declare_parameter("calibrated_sensor_offsets_m", [0.0] * 9)
        self.declare_parameter("contact_normal_xyz", [0.0, 0.0, 0.0])
        self.parent_frame = str(self.get_parameter("parent_frame").value)
        self.calibrated = bool(self.get_parameter("geometry_calibrated").value)
        self.support_length = float(self.get_parameter("support_length_m").value)
        self.support_width = float(self.get_parameter("support_width_m").value)
        self.support_depth = float(self.get_parameter("support_depth_m").value)
        self.support_center = _triple(
            self.get_parameter("approx_support_center_xyz").value,
            "approx_support_center_xyz",
        )
        self.support_rpy = _triple(
            self.get_parameter("approx_support_rpy").value, "approx_support_rpy"
        )
        if not self.parent_frame or min(
            self.support_length, self.support_width, self.support_depth
        ) <= 0.0:
            raise ValueError("parent_frame and positive support dimensions are required")
        source = (
            self.get_parameter("calibrated_sensor_offsets_m").value
            if self.calibrated
            else self.get_parameter("approx_sensor_offsets_m").value
        )
        values = tuple(float(value) for value in source)
        if len(values) != 9 or not all(math.isfinite(value) for value in values):
            raise ValueError("sensor offsets must contain S1/S2/S3 XYZ (nine values)")
        self.sensor_offsets = [values[index:index + 3] for index in range(0, 9, 3)]

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(
            MarkerArray, "/piperh/pulse/tool_markers", qos
        )
        self.broadcaster = StaticTransformBroadcaster(self)
        if self.calibrated:
            self._publish_calibrated_transforms()
            self.get_logger().info("publishing calibrated Piper-H pulse sensor frames")
        else:
            self.get_logger().warn(
                "Piper-H pulse fixture geometry is approximate; no sensor TF/TCP is published"
            )
        self.create_timer(1.0, self.publish_markers)
        self.publish_markers()

    def _publish_calibrated_transforms(self) -> None:
        quaternion = _quaternion_align_z(
            self.get_parameter("contact_normal_xyz").value
        )
        stamp = self.get_clock().now().to_msg()
        transforms = []
        for channel, offset in zip(("s1", "s2", "s3"), self.sensor_offsets):
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self.parent_frame
            transform.child_frame_id = f"piperh_pulse_{channel}"
            transform.transform.translation.x = offset[0]
            transform.transform.translation.y = offset[1]
            transform.transform.translation.z = offset[2]
            transform.transform.rotation.x = quaternion[0]
            transform.transform.rotation.y = quaternion[1]
            transform.transform.rotation.z = quaternion[2]
            transform.transform.rotation.w = quaternion[3]
            transforms.append(transform)
        self.broadcaster.sendTransform(transforms)

    def publish_markers(self) -> None:
        stamp = self.get_clock().now().to_msg()
        markers = []
        body = Marker()
        body.header.frame_id = self.parent_frame
        body.header.stamp = stamp
        body.ns = "piper_pulse_fixture"
        body.id = 0
        body.type = Marker.CUBE
        body.action = Marker.ADD
        body.pose.position.x, body.pose.position.y, body.pose.position.z = self.support_center
        quaternion = _quaternion_from_rpy(*self.support_rpy)
        body.pose.orientation.x, body.pose.orientation.y = quaternion[0], quaternion[1]
        body.pose.orientation.z, body.pose.orientation.w = quaternion[2], quaternion[3]
        body.scale.x = self.support_width
        body.scale.y = self.support_depth
        body.scale.z = self.support_length
        body.color.r, body.color.g, body.color.b = (
            (0.25, 0.75, 0.38) if self.calibrated else (0.65, 0.65, 0.65)
        )
        body.color.a = 0.72
        body.frame_locked = True
        markers.append(body)
        for marker_id, (channel, offset) in enumerate(
            zip(("S1", "S2", "S3"), self.sensor_offsets), start=1
        ):
            sensor = Marker()
            sensor.header = body.header
            sensor.ns = "piper_pulse_fixture"
            sensor.id = marker_id
            sensor.type = Marker.SPHERE
            sensor.action = Marker.ADD
            sensor.pose.position.x, sensor.pose.position.y, sensor.pose.position.z = offset
            sensor.pose.orientation.w = 1.0
            sensor.scale.x = sensor.scale.y = sensor.scale.z = 0.012
            sensor.color.r, sensor.color.g, sensor.color.b = (
                (0.18, 0.72, 1.0) if self.calibrated else (1.0, 0.72, 0.12)
            )
            sensor.color.a = 0.95
            sensor.frame_locked = True
            markers.append(sensor)
            label = Marker()
            label.header = body.header
            label.ns = "piper_pulse_fixture_labels"
            label.id = marker_id
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = offset[0]
            label.pose.position.y = offset[1] + 0.018
            label.pose.position.z = offset[2]
            label.pose.orientation.w = 1.0
            label.scale.z = 0.014
            label.color.r = label.color.g = label.color.b = 1.0
            label.color.a = 0.95
            label.text = channel
            label.frame_locked = True
            markers.append(label)
        warning = Marker()
        warning.header = body.header
        warning.ns = "piper_pulse_fixture_labels"
        warning.id = 10
        warning.type = Marker.TEXT_VIEW_FACING
        warning.action = Marker.ADD
        warning.pose.position.x = self.support_center[0]
        warning.pose.position.y = self.support_center[1]
        tool_direction = 1.0 if self.support_center[2] >= 0.0 else -1.0
        warning.pose.position.z = (
            self.support_center[2]
            + tool_direction * (self.support_length / 2.0 + 0.025)
        )
        warning.pose.orientation.w = 1.0
        warning.scale.z = 0.013
        warning.color.r, warning.color.g, warning.color.b, warning.color.a = 1.0, 0.35, 0.1, 1.0
        warning.text = (
            "CALIBRATED"
            if self.calibrated
            else f"{self.support_length * 1000.0:.2f} mm / TCP NOT CALIBRATED"
        )
        warning.frame_locked = True
        markers.append(warning)
        self.publisher.publish(MarkerArray(markers=markers))


def main() -> None:
    rclpy.init()
    node = PiperToolGeometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
