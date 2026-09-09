#!/usr/bin/env python3
"""Own teleoperation selection and the RViz-controlled mapping process."""

import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger


TELEOP_MODES = ("xbox", "keyboard")


def limited_twist(message: Twist, max_linear_speed: float) -> Twist:
    """Copy a finite translation-only command and clamp its planar magnitude."""
    result = Twist()
    vx = float(message.linear.x)
    vy = float(message.linear.y)
    if not math.isfinite(vx) or not math.isfinite(vy):
        return result
    magnitude = math.hypot(vx, vy)
    if magnitude > max_linear_speed and magnitude > 0.0:
        scale = max_linear_speed / magnitude
        vx *= scale
        vy *= scale
    result.linear.x = vx
    result.linear.y = vy
    return result


class OperatorManager(Node):
    """Multiplex operator inputs and manage an embedded SLAM Toolbox child."""

    def __init__(self) -> None:
        super().__init__("antbot_operator_manager")
        self.declare_parameter("default_teleop_mode", "xbox")
        self.declare_parameter("xbox_cmd_topic", "/antbot/cmd_vel/xbox")
        self.declare_parameter("keyboard_cmd_topic", "/antbot/cmd_vel/keyboard")
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("output_cmd_topic", "/cmd_vel")
        self.declare_parameter("max_linear_speed", 0.10)
        self.declare_parameter("command_timeout", 0.35)
        self.declare_parameter("scan_topic", "/scan_0")
        self.declare_parameter("mapping_topic", "/antbot/mapping/map")
        self.declare_parameter(
            "mapping_output_prefix", "/tmp/antbot_mapping/map"
        )

        mode = str(self.get_parameter("default_teleop_mode").value).lower()
        if mode not in TELEOP_MODES:
            raise ValueError("default_teleop_mode must be xbox or keyboard")
        self.teleop_mode = mode
        self.max_linear_speed = float(
            self.get_parameter("max_linear_speed").value
        )
        self.command_timeout = float(
            self.get_parameter("command_timeout").value
        )
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.mapping_topic = str(self.get_parameter("mapping_topic").value)
        self.mapping_output_prefix = str(
            self.get_parameter("mapping_output_prefix").value
        )
        self.mapping_process = None
        self.mapping_log_handle = None
        self.mapping_error = ""
        self.last_command_time = 0.0
        self.zero_sent = True

        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.cmd_publisher = self.create_publisher(
            Twist, str(self.get_parameter("output_cmd_topic").value), 10
        )
        self.status_publisher = self.create_publisher(
            String, "/antbot/operator_ui_status", state_qos
        )
        self.xbox_subscription = self.create_subscription(
            Twist,
            str(self.get_parameter("xbox_cmd_topic").value),
            lambda message: self.forward_command("xbox", message),
            10,
        )
        self.keyboard_subscription = self.create_subscription(
            Twist,
            str(self.get_parameter("keyboard_cmd_topic").value),
            lambda message: self.forward_command("keyboard", message),
            10,
        )
        self.teleop_service = self.create_service(
            SetBool, "/antbot/teleop/use_xbox", self.set_teleop_mode
        )
        self.mapping_service = self.create_service(
            SetBool, "/antbot/mapping/set_enabled", self.set_mapping_enabled
        )
        self.save_mapping_service = self.create_service(
            Trigger, "/antbot/mapping/save", self.save_mapping
        )
        self.command_timer = self.create_timer(0.05, self.command_watchdog)
        self.status_timer = self.create_timer(0.5, self.publish_status)
        self.publish_zero()
        self.publish_status()

    def forward_command(self, source: str, message: Twist) -> None:
        """Forward only the currently selected operator source."""
        if source != self.teleop_mode:
            return
        self.cmd_publisher.publish(limited_twist(message, self.max_linear_speed))
        self.last_command_time = time.monotonic()
        self.zero_sent = False

    def publish_zero(self) -> None:
        self.cmd_publisher.publish(Twist())
        self.zero_sent = True

    def command_watchdog(self) -> None:
        if (
            not self.zero_sent
            and time.monotonic() - self.last_command_time > self.command_timeout
        ):
            self.publish_zero()

    def set_teleop_mode(self, request, response):
        self.publish_zero()
        self.teleop_mode = "xbox" if request.data else "keyboard"
        response.success = True
        response.message = (
            "已切换 Xbox 控制；仍需 Xbox A 键解锁"
            if request.data
            else "已切换 RViz 键盘控制；点击键盘控制区后使用 Q/W/E/A/D/Z/X/C"
        )
        self.publish_status()
        return response

    def mapping_is_running(self) -> bool:
        return bool(
            self.mapping_process is not None
            and self.mapping_process.poll() is None
        )

    def start_mapping(self):
        if self.mapping_is_running():
            return True, "建图已经在运行"
        scan_publishers = self.count_publishers(self.scan_topic)
        if scan_publishers == 0:
            return False, f"无法开始：{self.scan_topic} 没有发布者"
        prefix = Path(self.mapping_output_prefix).expanduser()
        prefix.parent.mkdir(parents=True, exist_ok=True)
        log_path = prefix.parent / "mapping.log"
        self.mapping_log_handle = log_path.open("a", encoding="utf-8")
        command = [
            "ros2", "launch", "antbot_real_bringup",
            "embedded_mapping.launch.py",
            f"scan_topic:={self.scan_topic}",
            f"map_topic:={self.mapping_topic}",
        ]
        try:
            self.mapping_process = subprocess.Popen(
                command,
                stdout=self.mapping_log_handle,
                stderr=subprocess.STDOUT,
                start_new_session=True,
                text=True,
            )
        except OSError as error:
            self.mapping_log_handle.close()
            self.mapping_log_handle = None
            self.mapping_process = None
            self.mapping_error = str(error)
            return False, f"建图进程启动失败：{error}"
        self.mapping_error = ""
        return True, f"建图已启动；输出目录：{prefix.parent}"

    def stop_mapping(self) -> None:
        process = self.mapping_process
        if process is not None and process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=3.0)
            except ProcessLookupError:
                pass
        self.mapping_process = None
        if self.mapping_log_handle is not None:
            self.mapping_log_handle.close()
            self.mapping_log_handle = None

    def set_mapping_enabled(self, request, response):
        if request.data:
            response.success, response.message = self.start_mapping()
        else:
            self.stop_mapping()
            response.success = True
            response.message = "建图已停止；已生成的地图不会自动覆盖正式地图"
        self.publish_status()
        return response

    def save_mapping(self, _request, response):
        if not self.mapping_is_running():
            response.success = False
            response.message = "建图未运行，无法保存"
            return response
        prefix = Path(self.mapping_output_prefix).expanduser()
        prefix.parent.mkdir(parents=True, exist_ok=True)
        command = [
            "ros2", "run", "nav2_map_server", "map_saver_cli",
            "-t", self.mapping_topic, "-f", str(prefix),
            "--ros-args", "-p", "use_sim_time:=false",
            "-p", "save_map_timeout:=10.0",
            "-p", "map_subscribe_transient_local:=true",
        ]
        try:
            result = subprocess.run(
                command, capture_output=True, text=True, timeout=20.0,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired) as error:
            response.success = False
            response.message = f"地图保存失败：{error}"
            return response
        response.success = result.returncode == 0
        response.message = (
            f"地图已保存：{prefix}.yaml"
            if response.success
            else f"地图保存失败，退出码 {result.returncode}"
        )
        return response

    def publish_status(self) -> None:
        if self.mapping_process is not None:
            return_code = self.mapping_process.poll()
            if return_code is not None:
                if return_code != 0:
                    self.mapping_error = f"建图进程退出，代码 {return_code}"
                self.stop_mapping()
        message = String()
        message.data = json.dumps({
            "teleop_mode": self.teleop_mode,
            "xbox_publishers": self.count_publishers(
                str(self.get_parameter("xbox_cmd_topic").value)
            ),
            "joy_publishers": self.count_publishers(
                str(self.get_parameter("joy_topic").value)
            ),
            "keyboard_publishers": self.count_publishers(
                str(self.get_parameter("keyboard_cmd_topic").value)
            ),
            "mapping_running": self.mapping_is_running(),
            "mapping_error": self.mapping_error,
            "scan_topic": self.scan_topic,
            "scan_publishers": self.count_publishers(self.scan_topic),
            "mapping_topic": self.mapping_topic,
            "mapping_output_prefix": self.mapping_output_prefix,
        }, ensure_ascii=False, separators=(",", ":"))
        self.status_publisher.publish(message)

    def destroy_node(self):
        # A launch-system shutdown may invalidate the rcl context before this
        # callback runs.  The H743 bridge has its own watchdog, so only publish
        # the final zero while the ROS context can still accept messages.
        if rclpy.ok(context=self.context):
            self.publish_zero()
        self.stop_mapping()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = OperatorManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
