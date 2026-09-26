#!/usr/bin/env python3
"""Simulation-only adapter from Xbox gripper events to ros2_control."""

from collections.abc import Sequence
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def make_gripper_trajectory(
    joint_names: Sequence[str],
    positions: Sequence[float],
    move_duration: float,
) -> JointTrajectory:
    """Build a bounded-shape two-finger position trajectory."""
    if len(joint_names) != 2 or len(positions) != 2:
        raise ValueError("simulation gripper requires exactly two joints")
    if move_duration <= 0.0:
        raise ValueError("move_duration must be positive")

    seconds = int(move_duration)
    nanoseconds = int(round((move_duration - seconds) * 1_000_000_000))
    if nanoseconds == 1_000_000_000:
        seconds += 1
        nanoseconds = 0

    trajectory = JointTrajectory()
    trajectory.joint_names = [str(name) for name in joint_names]
    point = JointTrajectoryPoint()
    point.positions = [float(position) for position in positions]
    point.time_from_start.sec = seconds
    point.time_from_start.nanosec = nanoseconds
    trajectory.points = [point]
    return trajectory


def integrate_gripper_positions(
    positions: Sequence[float],
    velocity: float,
    period: float,
    lower_positions: Sequence[float],
    upper_positions: Sequence[float],
) -> list[float]:
    """Integrate a bounded two-finger velocity command for one control tick."""
    if not all(
        len(values) == 2
        for values in (positions, lower_positions, upper_positions)
    ):
        raise ValueError("simulation gripper requires exactly two joints")
    if period <= 0.0:
        raise ValueError("period must be positive")
    return [
        max(float(lower), min(float(upper), float(position) + velocity * period))
        for position, lower, upper in zip(
            positions, lower_positions, upper_positions
        )
    ]


class SimGripper(Node):
    """Drive only the mock simulation gripper controller."""

    def __init__(self) -> None:
        super().__init__("rebot_xbox_sim_gripper")
        self.declare_parameter("topics.open_event", "/rebot_xbox/gripper_open")
        self.declare_parameter("topics.close_event", "/rebot_xbox/gripper_close")
        self.declare_parameter("topics.joint_states", "/joint_states")
        self.declare_parameter(
            "topics.command", "/gripper_controller/joint_trajectory"
        )
        self.declare_parameter(
            "joint_names", ["gripper_joint1", "gripper_joint2"]
        )
        self.declare_parameter("open_positions", [0.045, 0.045])
        self.declare_parameter("closed_positions", [0.0, 0.0])
        self.declare_parameter("maximum_speed", 0.045)
        self.declare_parameter("control_rate", 20.0)
        self.declare_parameter("command_duration", 0.05)
        self.declare_parameter("command_timeout", 0.30)

        self._joint_names = list(self.get_parameter("joint_names").value)
        self._open_positions = list(self.get_parameter("open_positions").value)
        self._closed_positions = list(self.get_parameter("closed_positions").value)
        self._maximum_speed = float(self.get_parameter("maximum_speed").value)
        self._control_rate = float(self.get_parameter("control_rate").value)
        self._command_duration = float(
            self.get_parameter("command_duration").value
        )
        self._command_timeout = float(
            self.get_parameter("command_timeout").value
        )
        if (
            self._maximum_speed <= 0.0
            or self._control_rate <= 0.0
            or self._command_duration <= 0.0
            or self._command_timeout <= 0.0
        ):
            raise ValueError("gripper speed and timing parameters must be positive")
        make_gripper_trajectory(
            self._joint_names, self._open_positions, self._command_duration
        )
        make_gripper_trajectory(
            self._joint_names, self._closed_positions, self._command_duration
        )
        self._target_positions = list(self._closed_positions)
        self._measured_positions: list[float] | None = None
        self._open_amount = 0.0
        self._close_amount = 0.0
        self._last_input_time = time.monotonic()
        self._was_moving = False
        self._publisher = self.create_publisher(
            JointTrajectory,
            str(self.get_parameter("topics.command").value),
            10,
        )
        self._open_subscription = self.create_subscription(
            Float64,
            str(self.get_parameter("topics.open_event").value),
            self._open,
            10,
        )
        self._close_subscription = self.create_subscription(
            Float64,
            str(self.get_parameter("topics.close_event").value),
            self._close,
            10,
        )
        self._joint_state_subscription = self.create_subscription(
            JointState,
            str(self.get_parameter("topics.joint_states").value),
            self._joint_state_callback,
            qos_profile_sensor_data,
        )
        self._timer = self.create_timer(
            1.0 / self._control_rate, self._control_tick
        )
        self.get_logger().info(
            "[XBOX SERVO] Isaac Sim gripper adapter ready (simulation only)"
        )

    def _publish_target(self, positions: Sequence[float]) -> None:
        command = make_gripper_trajectory(
            self._joint_names, positions, self._command_duration
        )
        self._publisher.publish(command)

    def _open(self, message: Float64) -> None:
        self._open_amount = max(0.0, min(1.0, float(message.data)))
        self._last_input_time = time.monotonic()

    def _close(self, message: Float64) -> None:
        self._close_amount = max(0.0, min(1.0, float(message.data)))
        self._last_input_time = time.monotonic()

    def _joint_state_callback(self, message: JointState) -> None:
        if len(message.name) != len(message.position):
            return
        positions_by_name = dict(zip(message.name, message.position))
        if not all(name in positions_by_name for name in self._joint_names):
            return
        self._measured_positions = [
            float(positions_by_name[name]) for name in self._joint_names
        ]
        if not self._was_moving:
            self._target_positions = list(self._measured_positions)

    def _control_tick(self) -> None:
        if time.monotonic() - self._last_input_time > self._command_timeout:
            self._open_amount = 0.0
            self._close_amount = 0.0

        conflict = self._open_amount > 0.0 and self._close_amount > 0.0
        velocity = 0.0 if conflict else (
            self._open_amount - self._close_amount
        ) * self._maximum_speed
        if velocity == 0.0:
            if self._was_moving:
                if self._measured_positions is not None:
                    self._target_positions = list(self._measured_positions)
                self._publish_target(self._target_positions)
                self.get_logger().info("[XBOX SERVO] GRIPPER: STOP / HOLD")
            self._was_moving = False
            return

        start_positions = (
            self._measured_positions
            if self._measured_positions is not None
            else self._target_positions
        )
        self._target_positions = integrate_gripper_positions(
            start_positions,
            velocity,
            1.0 / self._control_rate,
            self._closed_positions,
            self._open_positions,
        )
        self._publish_target(self._target_positions)
        self._was_moving = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimGripper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
