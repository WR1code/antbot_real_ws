"""ROS 2 Piper-H gravity dry-run. This executable has no command publisher."""

from __future__ import annotations

import math
from pathlib import Path
import threading
import time

import numpy as np
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from .gravity_compensator import PiperHGravityCompensator, rotation_from_rpy


JOINTS = tuple(f"joint{index}" for index in range(1, 7))


class GravityDryRun(Node):
    """Read normal JointState feedback and print calculations; never transmit."""

    def __init__(self) -> None:
        super().__init__("piperh_gravity_dry_run")
        self.declare_parameter("joint_states_topic", "/piperh/joint_states")
        self.declare_parameter("base_mount_confirmed", False)
        self.declare_parameter("base_mount_roll_deg", 0.0)
        self.declare_parameter("base_mount_pitch_deg", 0.0)
        self.declare_parameter("base_mount_yaw_deg", 0.0)
        self.declare_parameter("report_period_sec", 1.0)
        if not bool(self.get_parameter("base_mount_confirmed").value):
            raise RuntimeError(
                "base mount orientation is unconfirmed; set base_mount_confirmed:=true "
                "and provide measured roll/pitch/yaw degrees"
            )
        rpy_deg = np.asarray([
            self.get_parameter("base_mount_roll_deg").value,
            self.get_parameter("base_mount_pitch_deg").value,
            self.get_parameter("base_mount_yaw_deg").value,
        ], dtype=float)
        if not np.all(np.isfinite(rpy_deg)):
            raise ValueError("base mount degrees must be finite")
        urdf = Path(get_package_share_directory("piper_h_description")) / (
            "urdf/piper_h_description.urdf"
        )
        self._rpy_deg = rpy_deg
        self._model = PiperHGravityCompensator(
            str(urdf), rotation_from_rpy(*np.deg2rad(rpy_deg))
        )
        period = float(self.get_parameter("report_period_sec").value)
        if not math.isfinite(period) or not 0.1 <= period <= 10.0:
            raise ValueError("report_period_sec must be within 0.1..10.0")
        self._lock = threading.Lock()
        self._sample = None
        self._previous = None
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_states_topic").value),
            self._feedback,
            qos_profile_sensor_data,
        )
        self.create_timer(period, self._report)
        self.get_logger().warning(
            "DRY RUN ONLY: this node creates no command publisher and sends no torque"
        )

    @staticmethod
    def _stamp(message: JointState) -> float:
        return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9

    def _feedback(self, message: JointState) -> None:
        try:
            if len(message.name) != len(set(message.name)):
                raise ValueError("duplicate joint names")
            lookup = {name: index for index, name in enumerate(message.name)}
            q = np.asarray([message.position[lookup[name]] for name in JOINTS], dtype=float)
            stamp = self._stamp(message)
            if q.shape != (6,) or not np.all(np.isfinite(q)) or stamp <= 0:
                raise ValueError("invalid normal joint feedback")
            if len(message.velocity) == len(message.name):
                qd = np.asarray([message.velocity[lookup[name]] for name in JOINTS], dtype=float)
            else:
                qd = np.zeros(6)
                if self._previous is not None and stamp > self._previous[0]:
                    qd = (q - self._previous[1]) / (stamp - self._previous[0])
            if not np.all(np.isfinite(qd)):
                raise ValueError("invalid joint velocity")
        except (KeyError, IndexError, ValueError) as error:
            self.get_logger().warning(f"rejecting normal joint feedback: {error}")
            return
        with self._lock:
            self._previous = (stamp, q)
            self._sample = (stamp, time.monotonic(), q, qd)

    def _report(self) -> None:
        with self._lock:
            sample = self._sample
        if sample is None:
            self.get_logger().warning("waiting for normal /piperh/joint_states feedback")
            return
        stamp, arrival, q, qd = sample
        age = time.monotonic() - arrival
        if age > 0.3:
            self.get_logger().error(f"normal joint feedback stale: age={age:.3f}s")
            return
        tau = self._model.torque(q, qd)
        self.get_logger().info(
            "DRY RUN (no command)\n"
            f"source_timestamp={stamp:.9f}\n"
            f"q={q.tolist()}\nqd={qd.tolist()}\n"
            f"base_mount_rpy_deg={self._rpy_deg.tolist()}\n"
            f"gravity_world={self._model.gravity_world.tolist()}\n"
            f"gravity_base={self._model.gravity_base.tolist()}\n"
            f"gravity_norm={np.linalg.norm(self._model.gravity_base):.9f}\n"
            f"gravity_torque={tau.tolist()}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GravityDryRun()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
