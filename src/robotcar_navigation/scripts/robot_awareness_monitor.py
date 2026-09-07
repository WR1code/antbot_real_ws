#!/usr/bin/env python3
"""Publish robot view direction, dynamic safety footprint and stuck history."""

import json
import math
import os
from pathlib import Path
import tempfile
import time

from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Point, Point32, Polygon, PolygonStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


TRANSIENT_QOS = QoSProfile(depth=1)
TRANSIENT_QOS.reliability = ReliabilityPolicy.RELIABLE
TRANSIENT_QOS.durability = DurabilityPolicy.TRANSIENT_LOCAL


def point32(x, y, z=0.0):
    point = Point()
    point.x, point.y, point.z = float(x), float(y), float(z)
    return point


def polygon_point(x, y, z=0.0):
    point = Point32()
    point.x, point.y, point.z = float(x), float(y), float(z)
    return point


class RobotAwarenessMonitor(Node):
    """Make invisible navigation state understandable in RViz."""

    def __init__(self):
        super().__init__("robot_awareness_monitor")
        self.declare_parameter("stuck_history_file", "")
        self.declare_parameter("base_half_length", 0.43)
        self.declare_parameter("base_half_width", 0.29)
        self.declare_parameter("base_margin", 0.08)
        self.declare_parameter("speed_margin_gain", 0.45)
        self.declare_parameter("view_range", 3.5)
        self.declare_parameter("view_fov_deg", 70.0)
        self.history_file = str(self.get_parameter("stuck_history_file").value)
        self.base_half_length = float(self.get_parameter("base_half_length").value)
        self.base_half_width = float(self.get_parameter("base_half_width").value)
        self.base_margin = float(self.get_parameter("base_margin").value)
        self.speed_gain = float(self.get_parameter("speed_margin_gain").value)
        self.view_range = float(self.get_parameter("view_range").value)
        self.view_fov = math.radians(float(self.get_parameter("view_fov_deg").value))
        self.speed = 0.0
        self.map_pose = None
        self.stuck_points = []
        self.recovery_goals = {"spin": set(), "backup": set(), "wait": set()}
        self.last_recovery_wall = 0.0

        self.tf_buffer = Buffer(cache_time=Duration(seconds=15.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.marker_pub = self.create_publisher(
            MarkerArray, "/antbot/awareness_markers", TRANSIENT_QOS
        )
        self.envelope_pub = self.create_publisher(
            PolygonStamped, "/antbot/safety_envelope", TRANSIENT_QOS
        )
        self.local_footprint_pub = self.create_publisher(
            Polygon, "/local_costmap/footprint", 10
        )
        self.global_footprint_pub = self.create_publisher(
            Polygon, "/global_costmap/footprint", 10
        )
        self.status_pub = self.create_publisher(
            String, "/antbot/awareness_status", TRANSIENT_QOS
        )
        self.create_subscription(
            Odometry, "/odometry/filtered", self.odom_callback, 20
        )
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self.amcl_callback, 10
        )
        for action_name in self.recovery_goals:
            self.create_subscription(
                GoalStatusArray,
                f"/{action_name}/_action/status",
                lambda message, name=action_name: self.recovery_callback(name, message),
                10,
            )
        self.load_history()
        self.create_timer(0.2, self.publish_state)

    def odom_callback(self, message):
        linear = message.twist.twist.linear
        self.speed = math.hypot(linear.x, linear.y)

    def amcl_callback(self, message):
        pose = message.pose.pose
        self.map_pose = (pose.position.x, pose.position.y)

    def update_map_pose_from_tf(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "base_link", Time(), timeout=Duration(seconds=0.03)
            )
            self.map_pose = (
                transform.transform.translation.x,
                transform.transform.translation.y,
            )
        except TransformException:
            pass

    @staticmethod
    def goal_key(status):
        return bytes(status.goal_info.goal_id.uuid).hex()

    def recovery_callback(self, name, message):
        executing = {
            self.goal_key(status)
            for status in message.status_list
            if status.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)
        }
        new_goals = executing - self.recovery_goals[name]
        self.recovery_goals[name] = executing
        if new_goals and time.monotonic() - self.last_recovery_wall >= 2.0:
            self.last_recovery_wall = time.monotonic()
            self.record_recovery(name)

    def record_recovery(self, behavior):
        self.update_map_pose_from_tf()
        if self.map_pose is None:
            self.get_logger().warning("Recovery detected but map pose is unavailable")
            return
        x, y = self.map_pose
        nearest = None
        nearest_distance = 0.65
        for point in self.stuck_points:
            distance = math.hypot(point["x"] - x, point["y"] - y)
            if distance < nearest_distance:
                nearest = point
                nearest_distance = distance
        if nearest is None:
            nearest = {"x": x, "y": y, "stuck_score": 0, "events": {}}
            self.stuck_points.append(nearest)
        nearest["stuck_score"] += 1
        nearest["events"][behavior] = nearest["events"].get(behavior, 0) + 1
        nearest["last_time"] = self.get_clock().now().nanoseconds
        self.save_history()
        self.get_logger().warning(
            f"Recovery accumulated at ({x:.2f}, {y:.2f}); "
            f"stuck_score={nearest['stuck_score']}"
        )

    def load_history(self):
        if not self.history_file or not Path(self.history_file).is_file():
            return
        try:
            with Path(self.history_file).open(encoding="utf-8") as stream:
                document = json.load(stream)
            points = document.get("points", [])
            if isinstance(points, list):
                self.stuck_points = [
                    point for point in points
                    if isinstance(point, dict) and point.get("stuck_score", 0) > 0
                ]
        except (OSError, ValueError, TypeError) as error:
            self.get_logger().error(f"Failed to load stuck history: {error}")

    def save_history(self):
        if not self.history_file:
            return
        path = Path(self.history_file).expanduser().resolve()
        path.parent.mkdir(parents=True, exist_ok=True)
        descriptor, temporary = tempfile.mkstemp(
            prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
        )
        try:
            with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
                json.dump(
                    {"version": 1, "points": self.stuck_points},
                    stream, ensure_ascii=False, indent=2,
                )
                stream.write("\n")
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, path)
        except Exception:
            if os.path.exists(temporary):
                os.unlink(temporary)
            raise

    def footprint(self):
        margin = self.base_margin + self.speed_gain * min(self.speed, 1.2)
        front = self.base_half_length + margin * 1.35
        rear = self.base_half_length + margin * 0.85
        side = self.base_half_width + margin
        polygon = Polygon()
        polygon.points = [
            polygon_point(front, side), polygon_point(front, -side),
            polygon_point(-rear, -side), polygon_point(-rear, side),
        ]
        return polygon, front, rear, side, margin

    def view_markers(self, stamp):
        half = self.view_fov * 0.5
        left = (self.view_range * math.cos(half), self.view_range * math.sin(half))
        right = (self.view_range * math.cos(half), -self.view_range * math.sin(half))
        fan = Marker()
        fan.header.frame_id = "base_link"
        fan.header.stamp = stamp
        fan.ns = "robot_view"
        fan.id = 0
        fan.type = Marker.TRIANGLE_LIST
        fan.action = Marker.ADD
        fan.pose.orientation.w = 1.0
        fan.points = [point32(0.30, 0.0, 0.65), point32(*left, 0.65), point32(*right, 0.65)]
        fan.color.r, fan.color.g, fan.color.b, fan.color.a = 0.05, 0.85, 1.0, 0.20
        outline = Marker()
        outline.header = fan.header
        outline.ns = "robot_view"
        outline.id = 1
        outline.type = Marker.LINE_STRIP
        outline.action = Marker.ADD
        outline.pose.orientation.w = 1.0
        outline.scale.x = 0.045
        outline.points = [
            point32(0.30, 0.0, 0.66), point32(*left, 0.66),
            point32(*right, 0.66), point32(0.30, 0.0, 0.66),
        ]
        outline.color.r, outline.color.g = 0.1, 0.95
        outline.color.b, outline.color.a = 1.0, 0.85
        label = Marker()
        label.header = fan.header
        label.ns = "robot_view"
        label.id = 2
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = min(1.8, self.view_range * 0.55)
        label.pose.position.z = 0.82
        label.pose.orientation.w = 1.0
        label.scale.z = 0.20
        label.color.r, label.color.g = 0.25, 0.95
        label.color.b, label.color.a = 1.0, 1.0
        label.text = "机器人现在正在看这里"
        return [fan, outline, label]

    def safety_markers(self, stamp, front, rear, side, margin):
        area = Marker()
        area.header.frame_id = "base_link"
        area.header.stamp = stamp
        area.ns = "dynamic_safety_envelope"
        area.id = 0
        area.type = Marker.CUBE
        area.action = Marker.ADD
        area.pose.position.x = (front - rear) * 0.5
        area.pose.position.z = 0.025
        area.pose.orientation.w = 1.0
        area.scale.x = front + rear
        area.scale.y = side * 2.0
        area.scale.z = 0.05
        ratio = min(1.0, self.speed / 0.8)
        area.color.r = 0.25 + 0.75 * ratio
        area.color.g = 0.85 - 0.45 * ratio
        area.color.b = 0.15
        area.color.a = 0.22
        label = Marker()
        label.header = area.header
        label.ns = "dynamic_safety_envelope"
        label.id = 1
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = 0.0
        label.pose.position.y = side + 0.18
        label.pose.position.z = 0.16
        label.pose.orientation.w = 1.0
        label.scale.z = 0.17
        label.color.r, label.color.g = area.color.r, area.color.g
        label.color.b, label.color.a = area.color.b, 1.0
        label.text = f"动态安全包络：{self.speed:.2f} m/s，余量 {margin:.2f} m"
        return [area, label]

    def stuck_markers(self, stamp):
        markers = []
        for index, point in enumerate(self.stuck_points):
            score = int(point.get("stuck_score", 0))
            heat = Marker()
            heat.header.frame_id = "map"
            heat.header.stamp = stamp
            heat.ns = "stuck_history"
            heat.id = index * 2
            heat.type = Marker.CYLINDER
            heat.action = Marker.ADD
            heat.pose.position.x = float(point["x"])
            heat.pose.position.y = float(point["y"])
            heat.pose.position.z = 0.035
            heat.pose.orientation.w = 1.0
            heat.scale.x = heat.scale.y = min(1.8, 0.45 + score * 0.12)
            heat.scale.z = 0.07
            heat.color.r = 1.0
            heat.color.g = max(0.05, 0.65 - score * 0.08)
            heat.color.b = 0.05
            heat.color.a = min(0.65, 0.22 + score * 0.06)
            markers.append(heat)
            label = Marker()
            label.header = heat.header
            label.ns = "stuck_history"
            label.id = index * 2 + 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = float(point["x"])
            label.pose.position.y = float(point["y"])
            label.pose.position.z = 0.32
            label.pose.orientation.w = 1.0
            label.scale.z = 0.19
            label.color.r, label.color.g = 1.0, 0.45
            label.color.b, label.color.a = 0.1, 1.0
            label.text = f"这里过去发生过 {score} 次卡死"
            markers.append(label)
        return markers

    def publish_state(self):
        self.update_map_pose_from_tf()
        polygon, front, rear, side, margin = self.footprint()
        self.local_footprint_pub.publish(polygon)
        self.global_footprint_pub.publish(polygon)
        envelope = PolygonStamped()
        envelope.header.stamp = self.get_clock().now().to_msg()
        envelope.header.frame_id = "base_link"
        envelope.polygon = polygon
        self.envelope_pub.publish(envelope)
        markers = MarkerArray()
        clear = Marker()
        clear.header.frame_id = "map"
        clear.header.stamp = envelope.header.stamp
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        markers.markers.extend(self.view_markers(envelope.header.stamp))
        markers.markers.extend(
            self.safety_markers(envelope.header.stamp, front, rear, side, margin)
        )
        markers.markers.extend(self.stuck_markers(envelope.header.stamp))
        self.marker_pub.publish(markers)
        payload = {
            "speed_mps": round(self.speed, 3),
            "safety_margin_m": round(margin, 3),
            "stuck_locations": len(self.stuck_points),
            "stuck_score_total": sum(
                int(point.get("stuck_score", 0)) for point in self.stuck_points
            ),
        }
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))


def main():
    rclpy.init()
    node = RobotAwarenessMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
