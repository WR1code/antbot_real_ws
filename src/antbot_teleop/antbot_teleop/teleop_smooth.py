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

"""Smooth keyboard teleoperation shared by ANTBot simulation and hardware."""

from select import select
import signal
import sys
import termios
import time
import tty

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


HELP = """
ANTBot 平滑键盘控制（仿真和实车通用）
------------------------------------------------
   q    w    e
   a    s    d
        x

w/x : 前进 / 后退       a/d : 左移 / 右移
q/e : 左转 / 右转       s/空格 : 急停
1~9 : 速度档位          Ctrl-C : 退出
------------------------------------------------
"""


def step_towards(current, target, max_step):
    """Move current towards target without overshooting."""
    if current < target:
        return min(current + max_step, target)
    if current > target:
        return max(current - max_step, target)
    return current


class SmoothTeleop(Node):

    def __init__(self):
        super().__init__('antbot_teleop_smooth')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('max_linear_vel', 0.5)
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('linear_accel', 0.8)
        self.declare_parameter('angular_accel', 1.8)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('speed_level', 5)
        self.declare_parameter('key_timeout', 0.45)

        topic = self.get_parameter('cmd_vel_topic').value
        self.max_linear = float(self.get_parameter('max_linear_vel').value)
        self.max_angular = float(self.get_parameter('max_angular_vel').value)
        self.linear_accel = float(self.get_parameter('linear_accel').value)
        self.angular_accel = float(self.get_parameter('angular_accel').value)
        self.rate = max(float(self.get_parameter('publish_rate').value), 1.0)
        self.speed_level = max(1, min(9, int(self.get_parameter('speed_level').value)))
        self.key_timeout = max(float(self.get_parameter('key_timeout').value), 0.05)

        self.publisher = self.create_publisher(Twist, topic, 10)
        self.target = [0.0, 0.0, 0.0]
        self.current = [0.0, 0.0, 0.0]
        self.last_motion_key = 0.0
        self.terminal_settings = termios.tcgetattr(sys.stdin)
        signal.signal(signal.SIGTERM, self._signal_handler)
        signal.signal(signal.SIGHUP, self._signal_handler)

    def _signal_handler(self, _signum, _frame):
        self.stop(immediate=True)
        self.restore_terminal()
        raise SystemExit

    def restore_terminal(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.terminal_settings)

    def get_key(self, timeout):
        tty.setraw(sys.stdin.fileno())
        ready, _, _ = select([sys.stdin], [], [], timeout)
        return sys.stdin.read(1) if ready else None

    def stop(self, immediate=False):
        self.target = [0.0, 0.0, 0.0]
        if immediate:
            self.current = [0.0, 0.0, 0.0]
            self.publish()

    def publish(self):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = self.current
        self.publisher.publish(msg)

    def set_motion(self, key):
        ratio = self.speed_level / 9.0
        linear = self.max_linear * ratio
        angular = self.max_angular * ratio
        bindings = {
            'w': (linear, 0.0, 0.0),
            'x': (-linear, 0.0, 0.0),
            'a': (0.0, linear, 0.0),
            'd': (0.0, -linear, 0.0),
            'q': (0.0, 0.0, angular),
            'e': (0.0, 0.0, -angular),
        }
        if key in bindings:
            self.target = list(bindings[key])
            self.last_motion_key = time.monotonic()
            return True
        return False

    def run(self):
        print(HELP)
        period = 1.0 / self.rate
        try:
            while rclpy.ok():
                key = self.get_key(period)
                if key == '\x03':
                    break
                if key in (' ', 's'):
                    self.stop(immediate=True)
                elif key is not None and key.isdigit() and key != '0':
                    self.speed_level = int(key)
                elif key is not None:
                    self.set_motion(key.lower())

                if time.monotonic() - self.last_motion_key > self.key_timeout:
                    self.stop()

                linear_step = self.linear_accel / self.rate
                angular_step = self.angular_accel / self.rate
                self.current[0] = step_towards(
                    self.current[0], self.target[0], linear_step)
                self.current[1] = step_towards(
                    self.current[1], self.target[1], linear_step)
                self.current[2] = step_towards(
                    self.current[2], self.target[2], angular_step)
                self.publish()
        finally:
            self.stop(immediate=True)
            self.restore_terminal()
            print()


def main(args=None):
    rclpy.init(args=args)
    node = SmoothTeleop()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
