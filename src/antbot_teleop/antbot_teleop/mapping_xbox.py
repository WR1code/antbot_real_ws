"""Safe Xbox teleoperation for ANTBot SLAM mapping."""

import json
import signal
import subprocess
import time
from pathlib import Path

from geometry_msgs.msg import Twist

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions

from sensor_msgs.msg import Joy

from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


CONTROL_BASE = 'base'
CONTROL_ARM = 'arm'


def apply_deadzone(value, deadzone):
    """Return zero inside the deadzone and preserve values outside it."""
    return 0.0 if abs(value) <= deadzone else float(value)


def trigger_amount(raw, deadzone):
    """Convert a +1 released/-1 pressed Xbox trigger to a 0..1 amount."""
    amount = (1.0 - max(-1.0, min(1.0, float(raw)))) / 2.0
    return 0.0 if amount <= deadzone else amount


def planar_command(left_x, left_y, lt, rt, linear_speed, angular_speed):
    """Return ROS body-frame vx, vy and wz for the measured Xbox mapping."""
    wz = (lt - rt) * angular_speed
    if abs(wz) > 0.0:
        # A trigger requests a true point turn. Giving rotation priority also
        # prevents accidental mixed motion when the left stick is not exactly
        # centered.
        return (0.0, 0.0, wz)
    return (
        -left_y * linear_speed,
        left_x * linear_speed,
        0.0,
    )


class MappingXbox(Node):
    """Publish guarded holonomic commands and manage the SLAM map."""

    def __init__(self):
        """Create publishers, load the measured mapping and start locked."""
        super().__init__('antbot_mapping_xbox')
        self._declare_parameters()

        self.axes = {
            'left_x': int(self.get_parameter('axes.left_x').value),
            'left_y': int(self.get_parameter('axes.left_y').value),
            'lt': int(self.get_parameter('axes.lt').value),
            'rt': int(self.get_parameter('axes.rt').value),
        }
        self.buttons = {
            'safety_lock': int(
                self.get_parameter('buttons.safety_lock').value),
            'save_map': int(self.get_parameter('buttons.save_map').value),
            'save_and_exit': int(
                self.get_parameter('buttons.save_and_exit').value),
            'control_switch': int(
                self.get_parameter('buttons.control_switch').value),
            'speed_down': int(self.get_parameter('buttons.speed_down').value),
            'speed_up': int(self.get_parameter('buttons.speed_up').value),
        }
        self.deadzone = float(self.get_parameter('deadzone').value)
        self.trigger_deadzone = float(
            self.get_parameter('trigger_deadzone').value)
        self.joy_timeout = float(self.get_parameter('joy_timeout').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.max_linear = float(self.get_parameter('max_linear_vel').value)
        self.max_angular = float(self.get_parameter('max_angular_vel').value)
        self.speed_levels = [
            float(value) for value in self.get_parameter('speed_levels').value
        ]
        initial_level = int(self.get_parameter('initial_speed_level').value)
        self.speed_index = max(
            0, min(initial_level - 1, len(self.speed_levels) - 1))
        self.map_prefix = str(self.get_parameter('map_prefix').value)
        self.use_sim_time_for_map = bool(
            self.get_parameter('map_use_sim_time').value)
        self.shutdown_zero_frames = max(
            5, int(self.get_parameter('shutdown_zero_frames').value))

        if not 0.0 <= self.deadzone < 1.0:
            raise ValueError('deadzone must be in [0, 1)')
        if not 0.0 <= self.trigger_deadzone < 1.0:
            raise ValueError('trigger_deadzone must be in [0, 1)')
        if self.joy_timeout <= 0.0 or self.publish_rate <= 0.0:
            raise ValueError('joy_timeout and publish_rate must be positive')
        if (
            not self.speed_levels
            or self.speed_levels != sorted(set(self.speed_levels))
            or any(level <= 0.0 or level > 1.0 for level in self.speed_levels)
        ):
            raise ValueError(
                'speed_levels must be unique, ascending values in (0, 1]')

        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.cmd_vel_publisher = self.create_publisher(
            Twist, str(self.get_parameter('topics.cmd_vel').value), 10)
        self.armed_publisher = self.create_publisher(
            Bool, str(self.get_parameter('topics.armed').value), state_qos)
        self.target_publisher = self.create_publisher(
            String,
            str(self.get_parameter('topics.control_target').value),
            state_qos,
        )
        self.status_publisher = self.create_publisher(
            String,
            str(self.get_parameter('topics.status').value),
            state_qos,
        )
        self.joy_subscription = self.create_subscription(
            Joy,
            str(self.get_parameter('topics.joy').value),
            self._joy_callback,
            20,
        )
        self.speed_down_service = self.create_service(
            Trigger,
            str(self.get_parameter('topics.speed_down_service').value),
            self._speed_down_service,
        )
        self.speed_up_service = self.create_service(
            Trigger,
            str(self.get_parameter('topics.speed_up_service').value),
            self._speed_up_service,
        )

        self.control_target = CONTROL_BASE
        self.armed = False
        self.previous_buttons = []
        self.triggers_blocked_until_release = False
        self.joy_seen = False
        self.joystick_timed_out = False
        self.last_joy_time = self.get_clock().now()
        self.last_command = Twist()
        self._reported_mapping_errors = set()
        self._stop_requested = False
        self.timer = self.create_timer(
            1.0 / self.publish_rate, self._timer_callback)
        self.status_timer = self.create_timer(0.2, self._publish_status)

        self._publish_target()
        self._publish_armed()
        self.get_logger().info('[ANTBOT XBOX] CONTROL: BASE')
        self.get_logger().info('[ANTBOT XBOX] LOCKED')
        self._log_speed()

    def _declare_parameters(self):
        default_map = str(Path.home() / 'maps' / 'antbot_map')
        self.declare_parameter('axes.left_x', 0)
        self.declare_parameter('axes.left_y', 1)
        self.declare_parameter('axes.lt', 2)
        self.declare_parameter('axes.rt', 5)
        self.declare_parameter('buttons.safety_lock', 0)
        self.declare_parameter('buttons.save_map', 1)
        self.declare_parameter('buttons.save_and_exit', 2)
        self.declare_parameter('buttons.control_switch', 8)
        self.declare_parameter('buttons.speed_down', 9)
        self.declare_parameter('buttons.speed_up', 10)
        self.declare_parameter('deadzone', 0.10)
        self.declare_parameter('trigger_deadzone', 0.05)
        self.declare_parameter('joy_timeout', 0.50)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('max_linear_vel', 1.50)
        self.declare_parameter('max_angular_vel', 1.00)
        self.declare_parameter('speed_levels', [0.10, 0.25, 0.50, 0.75, 1.00])
        self.declare_parameter('initial_speed_level', 1)
        self.declare_parameter('map_prefix', default_map)
        self.declare_parameter('map_use_sim_time', True)
        self.declare_parameter('shutdown_zero_frames', 5)
        self.declare_parameter('topics.joy', '/joy')
        self.declare_parameter('topics.cmd_vel', '/cmd_vel')
        self.declare_parameter('topics.armed', '/antbot_xbox/armed')
        self.declare_parameter('topics.status', '/antbot_xbox/status')
        self.declare_parameter(
            'topics.speed_down_service', '/antbot_xbox/speed_down')
        self.declare_parameter(
            'topics.speed_up_service', '/antbot_xbox/speed_up')
        self.declare_parameter(
            'topics.control_target', '/xbox/control_target')

    def _report_mapping_error(self, key, text):
        if key not in self._reported_mapping_errors:
            self._reported_mapping_errors.add(key)
            self.get_logger().error(f'[ANTBOT XBOX] {text}')

    def _axis(self, msg, name):
        index = self.axes[name]
        if index < 0 or index >= len(msg.axes):
            self._report_mapping_error(
                f'axis:{name}',
                f'axis {name} index {index} is outside '
                f'axes[0:{len(msg.axes)}]',
            )
            return None
        return max(-1.0, min(1.0, float(msg.axes[index])))

    def _button(self, msg, name):
        index = self.buttons[name]
        if index < 0 or index >= len(msg.buttons):
            self._report_mapping_error(
                f'button:{name}',
                f'button {name} index {index} is outside '
                f'buttons[0:{len(msg.buttons)}]',
            )
            return 0
        return int(msg.buttons[index])

    def _previous_button(self, name):
        index = self.buttons[name]
        if index < 0 or index >= len(self.previous_buttons):
            return 0
        return int(self.previous_buttons[index])

    def _rising_edge(self, msg, name):
        return (
            self._button(msg, name) == 1
            and self._previous_button(name) == 0
        )

    def _trigger(self, msg, name):
        raw = self._axis(msg, name)
        return 0.0 if raw is None else trigger_amount(
            raw, self.trigger_deadzone)

    def _sticks_centered(self, msg):
        values = (self._axis(msg, 'left_x'), self._axis(msg, 'left_y'))
        return all(
            value is not None and abs(value) <= self.deadzone
            for value in values
        )

    def _joy_callback(self, msg):
        self.last_joy_time = self.get_clock().now()
        recovering = self.joystick_timed_out
        self.joy_seen = True
        lt = self._trigger(msg, 'lt')
        rt = self._trigger(msg, 'rt')

        if not self.previous_buttons or recovering:
            self.previous_buttons = list(msg.buttons)
            self.triggers_blocked_until_release = lt > 0.0 or rt > 0.0
            self.joystick_timed_out = False
            self._lock()
            return

        if self._rising_edge(msg, 'control_switch'):
            self.control_target = (
                CONTROL_ARM
                if self.control_target == CONTROL_BASE
                else CONTROL_BASE
            )
            self._lock()
            self._publish_target()
            self.get_logger().warning(
                f'[ANTBOT XBOX] CONTROL: {self.control_target.upper()}; '
                'press A to arm the selected device')

        base_selected = self.control_target == CONTROL_BASE
        if base_selected and self._rising_edge(msg, 'safety_lock'):
            if self.armed:
                self._lock()
                self.get_logger().warning('[ANTBOT XBOX] LOCKED')
            elif not self._sticks_centered(msg):
                self.get_logger().warning(
                    '[ANTBOT XBOX] 请先将左摇杆回中')
            else:
                self.armed = True
                self.last_command = Twist()
                self._publish_armed()
                self.get_logger().info('[ANTBOT XBOX] ARMED')

        if not (lt > 0.0 or rt > 0.0):
            self.triggers_blocked_until_release = False

        if base_selected and self._rising_edge(msg, 'speed_down'):
            self._change_speed(-1)
        if base_selected and self._rising_edge(msg, 'speed_up'):
            self._change_speed(1)

        if base_selected and self._rising_edge(msg, 'save_map'):
            self.save_map()
        if base_selected and self._rising_edge(msg, 'save_and_exit'):
            self.save_map()
            self._stop_requested = True

        if base_selected and self.armed:
            left_x = apply_deadzone(
                self._axis(msg, 'left_x') or 0.0, self.deadzone)
            left_y = apply_deadzone(
                self._axis(msg, 'left_y') or 0.0, self.deadzone)
            if self.triggers_blocked_until_release:
                lt = 0.0
                rt = 0.0
            ratio = self.speed_levels[self.speed_index]
            vx, vy, wz = planar_command(
                left_x,
                left_y,
                lt,
                rt,
                self.max_linear * ratio,
                self.max_angular * ratio,
            )
            command = Twist()
            command.linear.x = vx
            command.linear.y = vy
            command.angular.z = wz
            self.last_command = command
        else:
            self.last_command = Twist()

        self.previous_buttons = list(msg.buttons)

    def _lock(self):
        self.armed = False
        self.last_command = Twist()
        self.cmd_vel_publisher.publish(Twist())
        self._publish_armed()

    def _publish_armed(self):
        self.armed_publisher.publish(Bool(data=self.armed))

    def _publish_target(self):
        self.target_publisher.publish(String(data=self.control_target))

    def _publish_status(self):
        age = (
            self.get_clock().now() - self.last_joy_time
        ).nanoseconds / 1_000_000_000.0
        message = String()
        message.data = json.dumps({
            'armed': self.armed,
            'joy_seen': self.joy_seen,
            'input_fresh': self.joy_seen and age <= self.joy_timeout,
            'control_target': self.control_target,
            'speed_level': self.speed_index + 1,
            'speed_level_count': len(self.speed_levels),
            'speed_ratio': self.speed_levels[self.speed_index],
            'speed_limit_mps': (
                self.max_linear * self.speed_levels[self.speed_index]
            ),
            'max_linear_mps': self.max_linear,
        }, separators=(',', ':'))
        self.status_publisher.publish(message)

    def _log_speed(self):
        percentage = round(self.speed_levels[self.speed_index] * 100)
        limit = self.max_linear * self.speed_levels[self.speed_index]
        self.get_logger().info(
            f'[ANTBOT XBOX] SPEED: {self.speed_index + 1}/'
            f'{len(self.speed_levels)} · {percentage}% · {limit:.3f} m/s')
        self._publish_status()

    def _change_speed(self, step):
        old_index = self.speed_index
        self.speed_index = max(
            0, min(self.speed_index + step, len(self.speed_levels) - 1))
        if self.speed_index != old_index:
            self._log_speed()
            return True
        self._publish_status()
        return False

    def _speed_down_service(self, _request, response):
        changed = self._change_speed(-1)
        response.success = True
        response.message = (
            f"速度档 {self.speed_index + 1}/{len(self.speed_levels)} · "
            f"{self.max_linear * self.speed_levels[self.speed_index]:.3f} m/s"
            + ("" if changed else "（已是最低档）")
        )
        return response

    def _speed_up_service(self, _request, response):
        changed = self._change_speed(1)
        response.success = True
        response.message = (
            f"速度档 {self.speed_index + 1}/{len(self.speed_levels)} · "
            f"{self.max_linear * self.speed_levels[self.speed_index]:.3f} m/s"
            + ("" if changed else "（已是最高档）")
        )
        return response

    def _timer_callback(self):
        if self.joy_seen and not self.joystick_timed_out:
            age = (
                self.get_clock().now() - self.last_joy_time
            ).nanoseconds / 1_000_000_000.0
            if age > self.joy_timeout:
                self.joystick_timed_out = True
                self._lock()
                self.get_logger().warning(
                    '[ANTBOT XBOX] JOYSTICK TIMEOUT - LOCKED')

        command = (
            self.last_command
            if self.control_target == CONTROL_BASE and self.armed
            else Twist()
        )
        self.cmd_vel_publisher.publish(command)

    def save_map(self):
        """Stop immediately and synchronously save the transient /map."""
        self._lock()
        prefix = Path(self.map_prefix).expanduser()
        prefix.parent.mkdir(parents=True, exist_ok=True)
        self.get_logger().info(f'[ANTBOT XBOX] Saving map to {prefix}.yaml')
        command = [
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
            '-t', '/map', '-f', str(prefix),
            '--ros-args',
            '-p', f'use_sim_time:={str(self.use_sim_time_for_map).lower()}',
            '-p', 'save_map_timeout:=10.0',
            '-p', 'map_subscribe_transient_local:=true',
        ]
        try:
            result = subprocess.run(command, check=False, timeout=20)
        except subprocess.TimeoutExpired:
            self.get_logger().error('[ANTBOT XBOX] Map save timed out')
            return False
        if result.returncode != 0:
            self.get_logger().error(
                f'[ANTBOT XBOX] map_saver_cli exited with '
                f'{result.returncode}')
            return False
        self.get_logger().info('[ANTBOT XBOX] Map saved successfully')
        return True

    def request_stop(self):
        """Request termination from a signal handler."""
        self._stop_requested = True

    def publish_shutdown_zeros(self):
        """Lock and publish redundant stop frames before shutdown."""
        self._lock()
        # Leave a simultaneously running arm selected but locked after the
        # mapping process that owns the Mode-button state disappears.
        self.control_target = CONTROL_ARM
        self._publish_target()
        period = 1.0 / self.publish_rate
        for _ in range(self.shutdown_zero_frames):
            self.cmd_vel_publisher.publish(Twist())
            time.sleep(period)


def main(args=None):
    """Run until Ctrl-C or the X save-and-exit button is pressed."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = MappingXbox()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def handle_signal(_signum, _frame):
        node.request_stop()

    old_sigint = signal.signal(signal.SIGINT, handle_signal)
    old_sigterm = signal.signal(signal.SIGTERM, handle_signal)
    try:
        while rclpy.ok() and not node._stop_requested:
            executor.spin_once(timeout_sec=0.1)
    finally:
        node.publish_shutdown_zeros()
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGINT, old_sigint)
        signal.signal(signal.SIGTERM, old_sigterm)


if __name__ == '__main__':
    main()
