#!/usr/bin/env python3
"""Convert Nav2 plans and motion feedback into human-readable robot intent."""

import json
import math
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String


LATCHED_QOS = QoSProfile(depth=1)
LATCHED_QOS.reliability = ReliabilityPolicy.RELIABLE
LATCHED_QOS.durability = DurabilityPolicy.TRANSIENT_LOCAL


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class RobotIntentMonitor(Node):
    def __init__(self):
        super().__init__("robot_intent_monitor")
        self.intent_pub = self.create_publisher(
            String, "/antbot/robot_intent", LATCHED_QOS
        )
        self.led_pub = self.create_publisher(
            String, "/antbot/led_intent", LATCHED_QOS
        )
        self.create_subscription(Twist, "/cmd_vel", self.cmd_callback, 10)
        self.create_subscription(
            Odometry, "/odometry/filtered", self.odom_callback, 20
        )
        self.create_subscription(Path, "/plan", self.plan_callback, 10)
        self.create_subscription(
            String, "/antbot/passing_request", self.passing_callback, 10
        )
        self.create_subscription(
            String, "/antbot/dock_request", self.dock_callback, 10
        )
        self.create_subscription(
            String, "/waterplus/route_progress", self.route_callback, 10
        )
        self.create_subscription(BatteryState, "/battery", self.battery_callback, 10)

        self.cmd = Twist()
        self.actual_speed = 0.0
        self.actual_wz = 0.0
        self.planned = None
        self.passing = None
        self.docking = None
        self.route_state = ""
        self.charging = False
        self.last_intent_key = None
        self.last_publish_time = 0.0
        self.create_timer(0.2, self.update_intent)

    def cmd_callback(self, message):
        self.cmd = message

    def odom_callback(self, message):
        twist = message.twist.twist
        self.actual_speed = math.hypot(twist.linear.x, twist.linear.y)
        self.actual_wz = twist.angular.z

    def plan_callback(self, message):
        points = message.poses
        if len(points) < 4:
            return
        first_index = 0
        while first_index + 1 < len(points):
            first = points[first_index].pose.position
            second = points[first_index + 1].pose.position
            if math.hypot(second.x - first.x, second.y - first.y) > 0.03:
                break
            first_index += 1
        later_index = min(len(points) - 1, first_index + 12)
        if later_index <= first_index + 1:
            return
        first = points[first_index].pose.position
        second = points[first_index + 1].pose.position
        before_later = points[later_index - 1].pose.position
        later = points[later_index].pose.position
        start_heading = math.atan2(second.y - first.y, second.x - first.x)
        later_heading = math.atan2(later.y - before_later.y, later.x - before_later.x)
        turn = normalize_angle(later_heading - start_heading)
        expires = time.monotonic() + 4.0
        if abs(turn) > math.radians(135):
            self.planned = ("准备掉头", "hazard", "u_turn", expires)
        elif turn > math.radians(22):
            self.planned = ("准备向左转", "left_flow", "turn_left", expires)
        elif turn < -math.radians(22):
            self.planned = ("准备向右转", "right_flow", "turn_right", expires)
        else:
            self.planned = None

    def passing_callback(self, message):
        side = "right"
        duration = 5.0
        try:
            document = json.loads(message.data)
            side = str(document.get("side", side)).lower()
            duration = float(document.get("duration", duration))
        except (ValueError, TypeError, json.JSONDecodeError):
            if message.data.strip().lower() in ("left", "right"):
                side = message.data.strip().lower()
        if side == "left":
            self.passing = (
                "我从你左边经过，请保持不动",
                "left_flow", "passing_left", time.monotonic() + duration,
            )
        else:
            self.passing = (
                "我从你右边经过，请保持不动",
                "right_flow", "passing_right", time.monotonic() + duration,
            )

    def dock_callback(self, message):
        try:
            document = json.loads(message.data)
            command = document.get("command", "start_docking")
            charger = document.get("charger", "充电点")
        except json.JSONDecodeError:
            command = message.data
            charger = "充电点"
        if command in ("start_docking", "navigate"):
            self.docking = (
                f"准备前往{charger}充电", "breathe_blue", "charging",
                time.monotonic() + 8.0,
            )
        elif command in ("cancel", "done"):
            self.docking = None

    def route_callback(self, message):
        try:
            self.route_state = str(json.loads(message.data).get("state", ""))
        except json.JSONDecodeError:
            pass

    def battery_callback(self, message):
        self.charging = (
            message.power_supply_status
            == BatteryState.POWER_SUPPLY_STATUS_CHARGING
        )

    def choose_intent(self):
        now = time.monotonic()
        if self.charging:
            return "正在充电", "breathe_green", "charging", "battery", 1.0
        for attribute in ("docking", "passing", "planned"):
            value = getattr(self, attribute)
            if value and value[3] > now:
                return value[0], value[1], value[2], attribute, 0.9
            if value:
                setattr(self, attribute, None)
        if self.route_state == "paused":
            return "巡航已暂停", "solid_yellow", "paused", "route", 1.0

        cmd_speed = math.hypot(self.cmd.linear.x, self.cmd.linear.y)
        if cmd_speed > 0.05 and self.actual_speed < 0.025:
            return "准备启动，请注意", "hazard", "starting", "cmd_vel", 0.85
        if abs(self.cmd.angular.z) > 0.35:
            if abs(self.cmd.angular.z) > 0.75 and cmd_speed < 0.08:
                return "正在掉头", "hazard", "u_turn", "cmd_vel", 0.8
            if self.cmd.angular.z > 0.0:
                return "正在向左转", "left_flow", "turn_left", "cmd_vel", 0.8
            return "正在向右转", "right_flow", "turn_right", "cmd_vel", 0.8
        if self.cmd.linear.y > 0.08:
            return "准备向左侧移动", "left_flow", "move_left", "cmd_vel", 0.8
        if self.cmd.linear.y < -0.08:
            return "准备向右侧移动", "right_flow", "move_right", "cmd_vel", 0.8
        if self.cmd.linear.x < -0.05:
            return "准备后退，请注意", "hazard", "reversing", "cmd_vel", 0.8
        if cmd_speed > 0.05 or self.actual_speed > 0.04:
            return "正在通行", "forward_flow", "moving", "odometry", 0.7
        return "当前静止", "off", "idle", "odometry", 1.0

    def update_intent(self):
        text, led_pattern, code, source, confidence = self.choose_intent()
        key = (text, led_pattern, code, source)
        now = time.monotonic()
        if key == self.last_intent_key and now - self.last_publish_time < 1.0:
            return
        payload = {
            "text": text,
            "code": code,
            "source": source,
            "confidence": confidence,
            "led_pattern": led_pattern,
            "actual_speed_mps": self.actual_speed,
            "actual_angular_z_rps": self.actual_wz,
        }
        self.intent_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        self.led_pub.publish(
            String(
                data=json.dumps(
                    {"pattern": led_pattern, "intent": code, "text": text},
                    ensure_ascii=False,
                )
            )
        )
        self.last_intent_key = key
        self.last_publish_time = now


def main():
    rclpy.init()
    node = RobotIntentMonitor()
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
