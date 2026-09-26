#!/usr/bin/env python3
"""Move the simulation arm slowly from its sleep pose to a Servo-safe pose."""

import sys
import time
from collections.abc import Sequence

from controller_manager_msgs.srv import ListControllers
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def controller_is_active(controller_states, expected_name: str) -> bool:
    """Return whether the required trajectory controller is active."""
    return any(
        state.name == expected_name and state.state == "active"
        for state in controller_states
    )


def make_startup_trajectory(
    joint_names: Sequence[str],
    start_positions: Sequence[float],
    target_positions: Sequence[float],
    move_duration: float,
) -> JointTrajectory:
    """Build a smooth two-point trajectory from the measured current pose."""
    count = len(joint_names)
    lengths_match = (
        len(start_positions) == count and len(target_positions) == count
    )
    if count == 0 or not lengths_match:
        raise ValueError(
            "joint names, start positions, and targets must match"
        )
    if move_duration <= 0.5:
        raise ValueError(
            "startup move_duration must be greater than 0.5 seconds"
        )

    trajectory = JointTrajectory()
    trajectory.joint_names = [str(name) for name in joint_names]

    start = JointTrajectoryPoint()
    start.positions = [float(value) for value in start_positions]
    start.velocities = [0.0] * count
    start.time_from_start.nanosec = 200_000_000

    target = JointTrajectoryPoint()
    target.positions = [float(value) for value in target_positions]
    target.velocities = [0.0] * count
    seconds = int(move_duration)
    target.time_from_start.sec = seconds
    target.time_from_start.nanosec = int(
        round((move_duration - seconds) * 1_000_000_000)
    )
    trajectory.points = [start, target]
    return trajectory


class ArmInitializer(Node):
    """Command the startup pose after ros2_control becomes available."""

    def __init__(self) -> None:
        super().__init__("rebot_xbox_arm_initializer")
        self.declare_parameter(
            "joint_names", [f"joint{index}" for index in range(1, 7)]
        )
        self.declare_parameter(
            "target_positions", [0.0, 1.75, 0.7, -0.7, 0.0, 0.0]
        )
        self.declare_parameter("move_duration", 6.0)
        self.declare_parameter("position_tolerance", 0.01)
        self.declare_parameter("controller_name", "rebotarm_controller")
        self.declare_parameter("controller_wait_timeout", 30.0)
        self.declare_parameter("completion_timeout", 12.0)
        self.declare_parameter("topics.joint_states", "/joint_states")
        self.declare_parameter("topics.armed", "/rebot_xbox/armed")
        self.declare_parameter(
            "topics.command", "/rebotarm_controller/joint_trajectory"
        )
        self.declare_parameter(
            "services.list_controllers", "/controller_manager/list_controllers"
        )

        configured_names = self.get_parameter("joint_names").value
        self.joint_names = [str(value) for value in configured_names]
        configured_targets = self.get_parameter("target_positions").value
        self.target_positions = [float(value) for value in configured_targets]
        self.move_duration = float(self.get_parameter("move_duration").value)
        self.position_tolerance = float(
            self.get_parameter("position_tolerance").value
        )
        self.controller_name = str(self.get_parameter("controller_name").value)
        self.controller_wait_timeout = float(
            self.get_parameter("controller_wait_timeout").value
        )
        self.completion_timeout = float(
            self.get_parameter("completion_timeout").value
        )
        if len(self.joint_names) != 6 or len(self.target_positions) != 6:
            raise ValueError(
                "startup initialization requires exactly six arm joints"
            )
        invalid_timing = self.completion_timeout <= self.move_duration
        invalid_parameters = (
            self.position_tolerance <= 0.0,
            self.controller_wait_timeout <= 0.0,
            invalid_timing,
        )
        if any(invalid_parameters):
            raise ValueError("invalid startup tolerance or timeout")

        self.publisher = self.create_publisher(
            JointTrajectory,
            str(self.get_parameter("topics.command").value),
            10,
        )
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.armed_publisher = self.create_publisher(
            Bool,
            str(self.get_parameter("topics.armed").value),
            state_qos,
        )
        self.armed_publisher.publish(Bool(data=False))
        self.subscription = self.create_subscription(
            JointState,
            str(self.get_parameter("topics.joint_states").value),
            self._joint_state_callback,
            20,
        )
        self.controller_client = self.create_client(
            ListControllers,
            str(self.get_parameter("services.list_controllers").value),
        )
        self.positions: dict[str, float] = {}
        self.started_at = time.monotonic()
        self.commanded_at: float | None = None
        self.controller_query = None
        self.controller_active = False
        self.last_controller_query_at = 0.0
        self.done = False
        self.failed = False
        self.get_logger().info(
            "[XBOX SERVO] INITIALIZING - Xbox motion remains LOCKED"
        )

    def _joint_state_callback(self, message: JointState) -> None:
        self.positions.update(
            (name, float(position))
            for name, position in zip(message.name, message.position)
        )

    def step(self) -> None:
        """Advance the startup state machine without blocking ROS callbacks."""
        now = time.monotonic()

        if self.commanded_at is None:
            if now - self.started_at > self.controller_wait_timeout:
                self.get_logger().error(
                    "[XBOX SERVO] INITIALIZATION FAILED - controller did not "
                    "become active"
                )
                self.failed = True
                return
            self._update_controller_state(now)
            if not self.controller_active:
                return
            if self.publisher.get_subscription_count() == 0:
                return
            if any(name not in self.positions for name in self.joint_names):
                return
            start_positions = [
                self.positions[name] for name in self.joint_names
            ]
            trajectory = make_startup_trajectory(
                self.joint_names,
                start_positions,
                self.target_positions,
                self.move_duration,
            )
            trajectory.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(trajectory)
            self.commanded_at = now
            self.get_logger().info(
                "[XBOX SERVO] INITIALIZATION MOVE: %s -> %s (%.1f s)"
                % (start_positions, self.target_positions, self.move_duration)
            )
            return

        if now - self.commanded_at > self.completion_timeout:
            self.get_logger().error(
                "[XBOX SERVO] INITIALIZATION FAILED - trajectory did not "
                "reach its target"
            )
            self.failed = True
            return

        errors = [
            abs(self.positions.get(name, float("inf")) - target)
            for name, target in zip(self.joint_names, self.target_positions)
        ]
        if max(errors) <= self.position_tolerance:
            self.get_logger().info(
                "[XBOX SERVO] INITIALIZATION COMPLETE - Servo may start"
            )
            self.done = True

    def _update_controller_state(self, now: float) -> None:
        """Poll controller_manager until the arm controller is active."""
        if self.controller_query is not None:
            if not self.controller_query.done():
                return
            try:
                response = self.controller_query.result()
            except Exception as error:  # pragma: no cover
                self.get_logger().warning(
                    f"[XBOX SERVO] controller state query failed: {error}"
                )
            else:
                self.controller_active = controller_is_active(
                    response.controller,
                    self.controller_name,
                )
                if self.controller_active:
                    self.get_logger().info(
                        "[XBOX SERVO] CONTROLLER ACTIVE: "
                        f"{self.controller_name}"
                    )
            self.controller_query = None

        if self.controller_active:
            return
        if now - self.last_controller_query_at < 0.25:
            return
        self.last_controller_query_at = now
        if self.controller_client.service_is_ready():
            self.controller_query = self.controller_client.call_async(
                ListControllers.Request()
            )


def main(args=None) -> int:
    rclpy.init(args=args)
    node = ArmInitializer()
    return_code = 1
    try:
        while rclpy.ok() and not node.done and not node.failed:
            rclpy.spin_once(node, timeout_sec=0.05)
            node.step()
        return_code = 0 if node.done else 1
    except KeyboardInterrupt:
        return_code = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return return_code


if __name__ == "__main__":
    sys.exit(main())
