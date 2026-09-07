# Copyright 2026 ROBOTIS AI CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Omnidirectional smooth keyboard teleoperation with SLAM map saving."""

import math
from select import select
import signal
import subprocess
import sys
import termios
import time
import tty

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


HELP = """
ANTBot Isaac 扫描建图键盘控制
------------------------------------------------
   q    w    e
   a  空格   d
   z    x    c

q/w/e : 左前 / 前 / 右前
a/d   : 左移 / 右移
z/x/c : 左后 / 后 / 右后
r/t   : 顺时针 / 逆时针原地旋转
空格: 急停              1~9 : 速度档位
m   : 立即保存地图       Ctrl-C : 保存地图并退出第一阶段
------------------------------------------------
"""


def step_towards(current, target, max_step):
    """Move current towards target without overshooting."""
    if current < target:
        return min(current + max_step, target)
    if current > target:
        return max(current - max_step, target)
    return current


def motion_for_key(key, linear, angular):
    """Return body-frame (vx, vy, wz) for an omnidirectional key."""
    diagonal = linear / math.sqrt(2.0)
    return {
        'q': (diagonal, diagonal, 0.0),
        'w': (linear, 0.0, 0.0),
        'e': (diagonal, -diagonal, 0.0),
        'd': (0.0, -linear, 0.0),
        'c': (-diagonal, -diagonal, 0.0),
        'x': (-linear, 0.0, 0.0),
        'z': (-diagonal, diagonal, 0.0),
        'a': (0.0, linear, 0.0),
        # ROS positive angular.z is counter-clockwise.
        'r': (0.0, 0.0, -angular),
        't': (0.0, 0.0, angular),
    }.get(key)


class MappingKeyboard(Node):
    """Publish smooth omnidirectional Twist commands and save the SLAM map."""

    def __init__(self):
        super().__init__('antbot_mapping_keyboard')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('map_prefix', '')
        self.declare_parameter('max_linear_vel', 0.60)
        self.declare_parameter('max_angular_vel', 1.00)
        self.declare_parameter('linear_accel', 1.20)
        self.declare_parameter('angular_accel', 2.40)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('speed_level', 5)
        self.declare_parameter('key_timeout', 0.45)

        self.map_prefix = str(self.get_parameter('map_prefix').value)
        topic = str(self.get_parameter('cmd_vel_topic').value)
        self.max_linear = float(self.get_parameter('max_linear_vel').value)
        self.max_angular = float(self.get_parameter('max_angular_vel').value)
        self.linear_accel = float(self.get_parameter('linear_accel').value)
        self.angular_accel = float(self.get_parameter('angular_accel').value)
        self.rate = max(float(self.get_parameter('publish_rate').value), 1.0)
        self.speed_level = max(
            1, min(9, int(self.get_parameter('speed_level').value)))
        self.key_timeout = max(
            float(self.get_parameter('key_timeout').value), 0.05)

        self.publisher = self.create_publisher(Twist, topic, 10)
        self.target = [0.0, 0.0, 0.0]
        self.current = [0.0, 0.0, 0.0]
        self.last_motion_key = 0.0
        self.terminal_settings = termios.tcgetattr(sys.stdin)
        signal.signal(signal.SIGTERM, self._signal_handler)
        signal.signal(signal.SIGHUP, self._signal_handler)

    def _signal_handler(self, _signum, _frame):
        try:
            self.stop(immediate=True)
        except Exception:
            # The ROS context may already be tearing down when a process group
            # receives the same termination signal more than once.
            pass
        try:
            self.restore_terminal()
        except termios.error:
            pass
        raise SystemExit

    def restore_terminal(self):
        """Restore terminal line discipline after raw keyboard input."""
        termios.tcsetattr(
            sys.stdin, termios.TCSADRAIN, self.terminal_settings)

    def get_key(self, timeout):
        """Read one key without waiting longer than timeout."""
        tty.setraw(sys.stdin.fileno())
        ready, _, _ = select([sys.stdin], [], [], timeout)
        key = sys.stdin.read(1) if ready else None
        termios.tcsetattr(
            sys.stdin, termios.TCSADRAIN, self.terminal_settings)
        return key

    def publish(self):
        """Publish the current planar command."""
        message = Twist()
        message.linear.x = self.current[0]
        message.linear.y = self.current[1]
        message.angular.z = self.current[2]
        try:
            self.publisher.publish(message)
        except Exception:
            # Ctrl-C can invalidate the rclpy context before the finally block
            # publishes its last zero command. The workflow also sends a stop.
            pass

    def stop(self, immediate=False):
        """Stop the requested motion, optionally without a deceleration ramp."""
        self.target = [0.0, 0.0, 0.0]
        if immediate:
            self.current = [0.0, 0.0, 0.0]
            self.publish()

    def set_motion(self, key):
        """Apply eight-direction translation and independent rotation."""
        ratio = self.speed_level / 9.0
        motion = motion_for_key(
            key, self.max_linear * ratio, self.max_angular * ratio)
        if motion is not None:
            self.target = list(motion)
            self.last_motion_key = time.monotonic()

    def save_map(self):
        """Save /map to the configured YAML/image prefix."""
        if not self.map_prefix:
            self.get_logger().error('map_prefix is empty; cannot save map')
            return
        self.stop(immediate=True)
        self.restore_terminal()
        self.get_logger().info(f'Saving map to {self.map_prefix}.yaml')
        command = [
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
            '-t', '/map', '-f', self.map_prefix,
            '--ros-args',
            '-p', 'use_sim_time:=true',
            '-p', 'save_map_timeout:=10.0',
            '-p', 'map_subscribe_transient_local:=true',
        ]
        try:
            result = subprocess.run(
                command, check=False, timeout=20)
            if result.returncode == 0:
                self.get_logger().info('Map saved successfully')
            else:
                self.get_logger().error(
                    f'map_saver_cli exited with {result.returncode}')
        except subprocess.TimeoutExpired:
            self.get_logger().error('Map save timed out')

    def run(self):
        """Run the direct-terminal keyboard loop."""
        print(HELP)
        period = 1.0 / self.rate
        try:
            while rclpy.ok():
                key = self.get_key(period)
                if key == '\x03':
                    break
                if key == ' ':
                    self.stop(immediate=True)
                elif key == 'm':
                    self.save_map()
                elif key is not None and key.isdigit() and key != '0':
                    self.speed_level = int(key)
                elif key is not None:
                    self.set_motion(key.lower())

                if time.monotonic() - self.last_motion_key > self.key_timeout:
                    self.stop()

                self.current[0] = step_towards(
                    self.current[0], self.target[0],
                    self.linear_accel / self.rate)
                self.current[1] = step_towards(
                    self.current[1], self.target[1],
                    self.linear_accel / self.rate)
                self.current[2] = step_towards(
                    self.current[2], self.target[2],
                    self.angular_accel / self.rate)
                self.publish()
        finally:
            self.stop(immediate=True)
            self.restore_terminal()
            print()


def main(args=None):
    """Run the mapping keyboard node."""
    rclpy.init(args=args)
    node = MappingKeyboard()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
