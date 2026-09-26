"""Velocity-proportional Xbox gripper adapter for real hardware."""

import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rebotarm_msgs.msg import JointMotorState, JointPosVelCmd
from std_msgs.msg import Float64


def gripper_command(
    open_amount: float,
    close_amount: float,
    current_position: float,
    open_position: float,
    closed_position: float,
    maximum_velocity: float,
    minimum_velocity: float,
    *,
    stale: bool = False,
    torque_latched: bool = False,
):
    """Return target and velocity, or a hold command represented by None."""
    conflict = open_amount > 0.0 and close_amount > 0.0
    amount = open_amount if open_amount > 0.0 else close_amount
    if stale or conflict or amount <= 0.0 or torque_latched:
        return None
    target = open_position if open_amount > 0.0 else closed_position
    velocity = max(minimum_velocity, amount * maximum_velocity)
    return float(target), float(velocity)


class HardwareGripper(Node):
    def __init__(self) -> None:
        super().__init__("rebot_xbox_hardware_gripper")
        self.declare_parameter("open_topic", "/rebot_xbox/gripper_open")
        self.declare_parameter("close_topic", "/rebot_xbox/gripper_close")
        self.declare_parameter("state_topic", "/rebotarm/gripper/state")
        self.declare_parameter("command_topic", "/rebotarm/gripper/cmd/pos_vel")
        self.declare_parameter("open_position", 5.0)
        self.declare_parameter("closed_position", 0.0)
        self.declare_parameter("maximum_velocity", 2.0)
        self.declare_parameter("minimum_velocity", 0.10)
        self.declare_parameter("command_timeout", 0.30)
        self.declare_parameter("maximum_closing_torque", 0.80)

        self._open_position = float(self.get_parameter("open_position").value)
        self._closed_position = float(self.get_parameter("closed_position").value)
        self._maximum_velocity = float(self.get_parameter("maximum_velocity").value)
        self._minimum_velocity = float(self.get_parameter("minimum_velocity").value)
        self._timeout = float(self.get_parameter("command_timeout").value)
        self._torque_limit = float(
            self.get_parameter("maximum_closing_torque").value
        )
        self._position = self._closed_position
        self._open_amount = 0.0
        self._close_amount = 0.0
        self._last_input = time.monotonic()
        self._moving = False
        self._torque_latched = False

        self._publisher = self.create_publisher(
            JointPosVelCmd, str(self.get_parameter("command_topic").value), 10
        )
        self.create_subscription(
            Float64, str(self.get_parameter("open_topic").value), self._open, 10
        )
        self.create_subscription(
            Float64, str(self.get_parameter("close_topic").value), self._close, 10
        )
        self.create_subscription(
            JointMotorState,
            str(self.get_parameter("state_topic").value),
            self._state,
            qos_profile_sensor_data,
        )
        self.create_timer(0.05, self._tick)

    def _open(self, message: Float64) -> None:
        self._open_amount = max(0.0, min(1.0, float(message.data)))
        self._last_input = time.monotonic()
        if self._open_amount == 0.0:
            self._torque_latched = False

    def _close(self, message: Float64) -> None:
        self._close_amount = max(0.0, min(1.0, float(message.data)))
        self._last_input = time.monotonic()
        if self._close_amount == 0.0:
            self._torque_latched = False

    def _state(self, message: JointMotorState) -> None:
        self._position = float(message.position)
        if (
            self._close_amount > 0.0
            and self._torque_limit > 0.0
            and abs(float(message.torque)) >= self._torque_limit
        ):
            self._torque_latched = True

    def _publish(self, position: float, velocity: float) -> None:
        message = JointPosVelCmd()
        message.pos = float(position)
        message.vlim = float(velocity)
        message.stamp = self.get_clock().now().to_msg()
        self._publisher.publish(message)

    def _tick(self) -> None:
        stale = time.monotonic() - self._last_input > self._timeout
        command = gripper_command(
            self._open_amount,
            self._close_amount,
            self._position,
            self._open_position,
            self._closed_position,
            self._maximum_velocity,
            self._minimum_velocity,
            stale=stale,
            torque_latched=self._torque_latched,
        )
        if command is None:
            if self._moving:
                self._publish(self._position, self._minimum_velocity)
                self._moving = False
            return
        target, velocity = command
        self._publish(target, velocity)
        self._moving = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HardwareGripper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(signal, "pthread_sigmask"):
            signal.pthread_sigmask(
                signal.SIG_BLOCK,
                {signal.SIGINT, signal.SIGTERM},
            )
        node.destroy_node()
        rclpy.try_shutdown()
