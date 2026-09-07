#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener
from tf_transformations import euler_from_quaternion


class PurePursuitPlanner(Node):
    def __init__(self):
        super().__init__("pure_pursuit_planner")

        self.declare_parameter("lookahead_distance", 0.6)
        self.declare_parameter("target_speed", 0.3)
        self.declare_parameter("max_omega", 1.0)
        self.declare_parameter("path_topic", "/plan")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("base_frame", "base_footprint")

        self.lookahead_distance = self.get_parameter("lookahead_distance").value
        self.target_speed = self.get_parameter("target_speed").value
        self.max_omega = self.get_parameter("max_omega").value
        self.base_frame = self.get_parameter("base_frame").value
        self.xy_goal_tolerance = 0.075
        self.yaw_goal_tolerance = 0.05

        self.current_path = []
        self.final_yaw = 0.0
        self.path_frame = "map"
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.path_sub = self.create_subscription(
            Path, self.get_parameter("path_topic").value, self.path_callback, 10
        )
        self.cmd_pub = self.create_publisher(Twist, self.get_parameter("cmd_vel_topic").value, 10)
        self.timer = self.create_timer(0.05, self.control_step)

    def path_callback(self, msg):
        self.path_frame = msg.header.frame_id or "map"
        self.current_path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        if not msg.poses:
            return

        orientation = msg.poses[-1].pose.orientation
        _, _, self.final_yaw = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )

    @staticmethod
    def distance(p1, p2):
        return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

    def find_lookahead_point(self, robot_pos):
        if not self.current_path:
            return None

        closest_idx = min(
            range(len(self.current_path)),
            key=lambda i: self.distance(robot_pos, self.current_path[i]),
        )
        for point in self.current_path[closest_idx:]:
            if self.distance(robot_pos, point) >= self.lookahead_distance:
                return point
        return self.current_path[-1]

    def control_step(self):
        cmd = Twist()
        if not self.current_path:
            self.cmd_pub.publish(cmd)
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.path_frame, self.base_frame, rclpy.time.Time()
            )
            self.robot_x = transform.transform.translation.x
            self.robot_y = transform.transform.translation.y
            rot = transform.transform.rotation
            _, _, self.robot_yaw = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
        except TransformException:
            return

        dist_to_goal = self.distance((self.robot_x, self.robot_y), self.current_path[-1])
        if dist_to_goal <= self.xy_goal_tolerance:
            yaw_error = self.final_yaw - self.robot_yaw
            yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))
            cmd.linear.x = 0.0
            if abs(yaw_error) > self.yaw_goal_tolerance:
                cmd.angular.z = math.copysign(max(0.2, min(abs(yaw_error), 0.5)), yaw_error)
            self.cmd_pub.publish(cmd)
            return

        target_point = self.find_lookahead_point((self.robot_x, self.robot_y))
        if target_point is None:
            self.cmd_pub.publish(cmd)
            return

        tx, ty = target_point
        angle_to_target = math.atan2(ty - self.robot_y, tx - self.robot_x)
        alpha = math.atan2(math.sin(angle_to_target - self.robot_yaw), math.cos(angle_to_target - self.robot_yaw))

        if abs(alpha) > math.pi / 3.0:
            cmd.linear.x = 0.0
            cmd.angular.z = math.copysign(min(abs(alpha) * 1.5, self.max_omega), alpha)
        else:
            speed_multiplier = 1.0
            if abs(alpha) > math.pi / 6.0:
                speed_multiplier = max(0.1, 1.0 - (abs(alpha) - math.pi / 6.0) / (math.pi / 6.0))
            current_speed = self.target_speed * speed_multiplier
            omega = (2.0 * current_speed * math.sin(alpha)) / self.lookahead_distance
            cmd.linear.x = current_speed
            cmd.angular.z = max(min(omega, self.max_omega), -self.max_omega)

        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = PurePursuitPlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
