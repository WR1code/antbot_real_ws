"""Watchdog-guarded MoveIt Servo joint targets for real hardware."""

from __future__ import annotations

import math
import time

from rclpy.qos import QoSProfile, ReliabilityPolicy
from trajectory_msgs.msg import JointTrajectory


def validated_servo_target(message: JointTrajectory, joint_names: list[str]):
    """Return the newest complete finite target, or raise ValueError."""
    if list(message.joint_names) != list(joint_names):
        raise ValueError(f"joint_names must be {joint_names}")
    if not message.points:
        raise ValueError("trajectory must contain at least one point")
    positions = list(message.points[-1].positions)
    if len(positions) != len(joint_names) or not all(map(math.isfinite, positions)):
        raise ValueError("trajectory positions must be complete and finite")
    return positions


class ServoTrajectoryInput:
    """Apply atomic joint targets and hold when the stream becomes stale."""

    def __init__(self, node, hardware, topic: str, timeout: float) -> None:
        if timeout <= 0.0:
            raise ValueError("servo command timeout must be positive")
        self._node = node
        self._hardware = hardware
        self._timeout = timeout
        self._last_command = 0.0
        self._active = False
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self._subscription = node.create_subscription(
            JointTrajectory,
            topic,
            self._command,
            qos,
            callback_group=node.reentrant_group,
        )
        self._timer = node.create_timer(
            min(timeout / 2.0, 0.05),
            self._watchdog,
            callback_group=node.reentrant_group,
        )

    def _command(self, message: JointTrajectory) -> None:
        try:
            target = validated_servo_target(message, self._hardware.joint_names)
            self._hardware.set_servo_joint_position_target(target)
        except Exception as exc:
            self._node.get_logger().warn(f"rejecting Servo joint target: {exc}")
            return
        self._last_command = time.monotonic()
        self._active = True

    def _watchdog(self) -> None:
        if not self._active or time.monotonic() - self._last_command <= self._timeout:
            return
        try:
            self._hardware.stop_servo_stream()
        except Exception as exc:
            self._node.get_logger().error(f"Servo watchdog hold failed: {exc}")
        self._active = False
        self._node.publish_arm_status()
        self._node.get_logger().warn("Servo command timeout: holding current position")
