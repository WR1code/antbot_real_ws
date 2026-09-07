#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import os
import re
import sys
import termios
import tty
import xml.etree.ElementTree as ET

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

GREEN = "\033[92m"
YELLOW = "\033[93m"
RESET = "\033[0m"


class NavigateWaypoints(Node):
    def __init__(self):
        super().__init__("wr_navigate_py")
        self.declare_parameter("waypoints_file", "")
        self.declare_parameter("route_file", "")

        self.waypoints = []
        self.route_name = ""
        self.yolo_done = False
        self.last_yolo_result = {}
        self.control_command = ""
        self.active_index = 0
        self.active_completed = 0
        self.active_name = ""
        self.route_progress_pub = self.create_publisher(
            String, "/waterplus/route_progress", 10
        )

        waypoints_file = self.get_parameter("waypoints_file").value
        if not waypoints_file:
            self.get_logger().error("No waypoints_file parameter found")
            return

        self.load_waypoints(waypoints_file)
        self.route_name = os.path.splitext(os.path.basename(waypoints_file))[0]
        route_file = self.get_parameter("route_file").value
        if route_file:
            self.apply_route_group(route_file)
        self.action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.yolo_trigger_pub = self.create_publisher(String, "/yolo_trigger", 10)
        self.wp_reached_pub = self.create_publisher(String, "/waypoint_reached", 10)
        self.yolo_result_sub = self.create_subscription(
            String, "/yolo_result", self.yolo_result_callback, 10
        )
        self.route_control_sub = self.create_subscription(
            String, "/waterplus/route_control", self.route_control_callback, 10
        )

    def load_waypoints(self, filename):
        try:
            tree = ET.parse(filename)
            root = tree.getroot()
            for wp_elem in root.findall("waypoint"):
                pose_elem = wp_elem.find("pose")
                if pose_elem is None:
                    continue

                pos_elem = pose_elem.find("position")
                ori_elem = pose_elem.find("orientation")
                action_elem = wp_elem.find("action")

                self.waypoints.append(
                    {
                        "name": wp_elem.get("name", "unnamed_wp"),
                        "pose": {
                            "position": {
                                "x": float(pos_elem.get("x", 0.0))
                                if pos_elem is not None else 0.0,
                                "y": float(pos_elem.get("y", 0.0))
                                if pos_elem is not None else 0.0,
                                "z": float(pos_elem.get("z", 0.0))
                                if pos_elem is not None else 0.0,
                            },
                            "orientation": {
                                "x": float(ori_elem.get("x", 0.0))
                                if ori_elem is not None else 0.0,
                                "y": float(ori_elem.get("y", 0.0))
                                if ori_elem is not None else 0.0,
                                "z": float(ori_elem.get("z", 0.0))
                                if ori_elem is not None else 0.0,
                                "w": float(ori_elem.get("w", 1.0))
                                if ori_elem is not None else 1.0,
                            },
                        },
                        "action": action_elem.text.strip()
                        if action_elem is not None and action_elem.text
                        else "none",
                    }
                )

            def natural_sort_key(wp):
                return [
                    int(text) if text.isdigit() else text.lower()
                    for text in re.split(r"(\d+)", wp["name"])
                ]

            self.waypoints.sort(key=natural_sort_key)
        except Exception as exc:
            self.get_logger().error(f"Failed to load waypoints: {exc}")

    def apply_route_group(self, filename):
        """Select and order loaded waypoints using an RViz route-group JSON file."""
        try:
            with open(filename, encoding="utf-8") as stream:
                route = json.load(stream)
            names = route.get("waypoints", [])
            if not isinstance(names, list) or not all(isinstance(name, str) for name in names):
                raise ValueError("waypoints must be a string array")
            associated_group = route.get("waypoint_group", "")
            if associated_group:
                self.waypoints = []
                self.load_waypoints(associated_group)
            by_name = {waypoint["name"]: waypoint for waypoint in self.waypoints}
            missing = [name for name in names if name not in by_name]
            if missing:
                raise ValueError("unknown waypoint(s): " + ", ".join(missing))
            self.waypoints = [by_name[name] for name in names]
            self.route_name = str(route.get("name") or os.path.basename(filename))
            self.get_logger().info(
                f"Loaded route group '{route.get('name', filename)}' with {len(names)} stops"
            )
        except Exception as exc:
            self.waypoints = []
            self.get_logger().error(f"Failed to load route group {filename}: {exc}")

    def publish_progress(
        self, state, current_index=0, completed=0, current_name="", message=""
    ):
        payload = {
            "state": state,
            "route_name": self.route_name,
            "current_index": current_index,
            "completed": completed,
            "total": len(self.waypoints),
            "current_name": current_name,
            "message": message,
        }
        self.route_progress_pub.publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )

    @staticmethod
    def get_key():
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def yolo_result_callback(self, msg):
        try:
            self.last_yolo_result = json.loads(msg.data)
            self.get_logger().info(f"Received YOLO result: {json.dumps(self.last_yolo_result)}")
            self.yolo_done = True
        except Exception as exc:
            self.get_logger().error(f"Error processing YOLO result: {exc}")

    def route_control_callback(self, msg):
        command = msg.data.strip().lower()
        if command in ("pause", "resume", "stop"):
            self.control_command = command
            self.get_logger().info(f"Route control request: {command}")

    def send_goal(self, waypoint):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        position = waypoint["pose"]["position"]
        orientation = waypoint["pose"]["orientation"]
        goal.pose.pose.position.x = position["x"]
        goal.pose.pose.position.y = position["y"]
        goal.pose.pose.position.z = position["z"]
        goal.pose.pose.orientation.x = orientation["x"]
        goal.pose.pose.orientation.y = orientation["y"]
        goal.pose.pose.orientation.z = orientation["z"]
        goal.pose.pose.orientation.w = orientation["w"]

        self.action_client.wait_for_server()
        send_future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            return "failed"

        result_future = goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.control_command == "stop":
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)
                self.control_command = ""
                return "stopped"
            if self.control_command == "pause":
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)
                self.control_command = ""
                self.publish_progress(
                    "paused", self.active_index, self.active_completed,
                    self.active_name, "巡航已暂停，等待继续",
                )
                while rclpy.ok():
                    rclpy.spin_once(self, timeout_sec=0.1)
                    if self.control_command == "resume":
                        self.control_command = ""
                        return "retry"
                    if self.control_command == "stop":
                        self.control_command = ""
                        return "stopped"
        if not rclpy.ok() or not result_future.done():
            return "stopped"
        return (
            "success"
            if result_future.result().status == GoalStatus.STATUS_SUCCEEDED
            else "failed"
        )

    def execute(self):
        detect_wps = [w["name"] for w in self.waypoints if w.get("action") == "detect"]
        detect_info = ", ".join(detect_wps) if detect_wps else "None"
        self.get_logger().info(
            f"{GREEN}Loaded {len(self.waypoints)} waypoints. "
            f"Vision recognition enabled at: [{detect_info}]{RESET}"
        )
        self.get_logger().info("Press Spacebar to start navigation")
        self.publish_progress("waiting", message="等待空格开始巡航")

        while rclpy.ok():
            key = self.get_key()
            if key == " ":
                break
            if ord(key) == 3:
                self.publish_progress("stopped", message="用户取消巡航")
                return

        completed = 0
        failed = False
        for index, waypoint in enumerate(self.waypoints, start=1):
            if not rclpy.ok():
                break

            self.get_logger().info(
                f"{YELLOW}>>> [{index}/{len(self.waypoints)}] "
                f"Navigating to Waypoint {waypoint['name']}...{RESET}"
            )
            self.publish_progress(
                "navigating", index, completed, waypoint["name"]
            )
            self.active_index = index
            self.active_completed = completed
            self.active_name = waypoint["name"]
            navigation_result = self.send_goal(waypoint)
            while navigation_result == "retry" and rclpy.ok():
                self.publish_progress(
                    "navigating", index, completed, waypoint["name"], "巡航已继续"
                )
                navigation_result = self.send_goal(waypoint)
            if navigation_result == "stopped":
                self.publish_progress(
                    "stopped", index, completed, waypoint["name"], "用户停止巡航"
                )
                return
            if navigation_result == "success":
                completed = index
                self.get_logger().info(f"{GREEN}Target reached: {waypoint['name']}{RESET}")
                self.wp_reached_pub.publish(String(data=waypoint["name"]))
                self.publish_progress(
                    "reached", index, completed, waypoint["name"]
                )

                if waypoint.get("action") == "detect":
                    self.get_logger().info(
                        f"{GREEN}[VISION] Triggering detection at {waypoint['name']}...{RESET}"
                    )
                    self.yolo_done = False
                    while not self.yolo_done and rclpy.ok():
                        self.yolo_trigger_pub.publish(String(data="start_detection"))
                        rclpy.spin_once(self, timeout_sec=1.0)
            else:
                self.get_logger().warn("Failed to reach waypoint")
                failed = True
                self.publish_progress(
                    "failed", index, completed, waypoint["name"], "导航失败"
                )
                break

        if not failed and completed == len(self.waypoints):
            self.publish_progress("completed", completed=completed)
            self.get_logger().info("Navigation sequence finished")


def main():
    rclpy.init()
    navigator = NavigateWaypoints()
    try:
        if navigator.waypoints:
            navigator.execute()
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
