"""Drive the real AntBot control chain and measure IMU/truth motion signs."""

import argparse
import os
import math
from pathlib import Path

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu


MOTIONS = (
    ("static", 0.0, 0.0, 0.0, 2.5),
    ("left_rotation_low", 0.0, 0.0, 0.30, 3.0),
    ("right_rotation_low", 0.0, 0.0, -0.30, 3.0),
    ("left_rotation_medium", 0.0, 0.0, 0.80, 3.0),
    ("right_rotation_medium", 0.0, 0.0, -0.80, 3.0),
    ("left_rotation_high", 0.0, 0.0, 1.20, 3.0),
    ("right_rotation_high", 0.0, 0.0, -1.20, 3.0),
)


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def yaw_from_quaternion(value):
    return math.atan2(
        2.0 * (value.w * value.z + value.x * value.y),
        1.0 - 2.0 * (value.y * value.y + value.z * value.z),
    )


def angle_delta(end, start):
    return math.atan2(math.sin(end - start), math.cos(end - start))


def integrate_window(samples, start_ns, duration_sec):
    end_ns = start_ns + int(duration_sec * 1e9)
    selected = [(t, w) for t, w, _ax in samples if start_ns <= t <= end_ns]
    if len(selected) < 2 or selected[-1][0] < end_ns - 20_000_000:
        return None
    return float(np.trapz(
        [item[1] for item in selected],
        x=np.asarray([item[0] for item in selected], dtype=np.float64) * 1e-9,
    ))


class MotionExperiment(Node):
    def __init__(self, output):
        super().__init__("phase2a_motion_experiment")
        self.output = Path(output)
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Clock, "/clock", self._clock_callback, 100)
        self.create_subscription(Imu, "/antbot/imu/data", self._imu, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, "/antbot/ground_truth/odom", self._odom, qos_profile_sensor_data
        )
        self.now_ns = None
        self.imu = []
        self.truth = []
        self.events = []
        self.index = 0
        self.phase = "wait"
        self.phase_start = None
        self.done = False
        self.timer = self.create_timer(0.05, self._tick)

    def _clock_callback(self, message):
        self.now_ns = stamp_ns(message.clock)

    def _imu(self, message):
        self.imu.append((
            stamp_ns(message.header.stamp),
            float(message.angular_velocity.z),
            float(message.linear_acceleration.x),
        ))

    def _odom(self, message):
        self.truth.append((
            stamp_ns(message.header.stamp),
            yaw_from_quaternion(message.pose.pose.orientation),
            float(message.twist.twist.linear.x),
            float(message.twist.twist.angular.z),
        ))

    def _publish(self, vx=0.0, wz=0.0):
        message = Twist()
        message.linear.x = vx
        message.angular.z = wz
        self.publisher.publish(message)

    def _tick(self):
        if self.now_ns is None or self.done:
            return
        if self.phase == "wait":
            self._publish()
            if self.phase_start is None:
                self.phase_start = self.now_ns
            if self.now_ns - self.phase_start >= 2_000_000_000:
                self.phase = "pre"
                self.phase_start = self.now_ns
            return
        if self.index >= len(MOTIONS):
            self._publish()
            self._finish()
            return
        name, vx, _vy, wz, duration = MOTIONS[self.index]
        if self.phase == "pre":
            self._publish()
            if self.now_ns - self.phase_start >= 1_000_000_000:
                self.phase = "active"
                self.phase_start = self.now_ns
                self.events.append({
                    "name": name, "vx": vx, "wz": wz, "start_ns": self.now_ns,
                })
        elif self.phase == "active":
            self._publish(vx, wz)
            if self.now_ns - self.phase_start >= int(duration * 1e9):
                self.events[-1]["end_ns"] = self.now_ns
                self.phase = "post"
                self.phase_start = self.now_ns
        elif self.phase == "post":
            self._publish()
            if self.now_ns - self.phase_start >= 1_000_000_000:
                self.index += 1
                self.phase = "pre"
                self.phase_start = self.now_ns

    def _truth_yaw(self, query_ns):
        selected = min(self.truth, key=lambda item: abs(item[0] - query_ns))
        return selected[1]

    def _finish(self):
        rows = []
        imu_stamps = [item[0] for item in self.imu]
        regressions = sum(b < a for a, b in zip(imu_stamps, imu_stamps[1:]))
        duplicates = sum(b == a for a, b in zip(imu_stamps, imu_stamps[1:]))
        for event in self.events:
            samples = [item for item in self.imu if event["start_ns"] <= item[0] <= event["end_ns"]]
            yaw_start = self._truth_yaw(event["start_ns"])
            yaw_end = self._truth_yaw(event["end_ns"])
            row = dict(event)
            row.update({
                "truth_yaw_delta_rad": angle_delta(yaw_end, yaw_start),
                "imu_wz_mean_rad_s": float(np.mean([item[1] for item in samples])),
                "imu_wz_min_rad_s": float(np.min([item[1] for item in samples])),
                "imu_wz_max_rad_s": float(np.max([item[1] for item in samples])),
                "imu_ax_mean_m_s2": float(np.mean([item[2] for item in samples])),
                "imu_sample_count": len(samples),
                "integrals": {},
            })
            for window in (1.0, 2.0, 5.0):
                integral = integrate_window(self.imu, event["start_ns"], window)
                truth_end = self._truth_yaw(event["start_ns"] + int(window * 1e9))
                row["integrals"][str(int(window))] = {
                    "imu_yaw_delta_rad": integral,
                    "truth_yaw_delta_rad": angle_delta(truth_end, yaw_start),
                    "error_rad": (
                        angle_delta(integral, angle_delta(truth_end, yaw_start))
                        if integral is not None else None
                    ),
                }
            rows.append(row)
        result = {
            "status": "executed",
            "imu_timestamp_regressions": regressions,
            "imu_duplicate_timestamps": duplicates,
            "motions": rows,
        }
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.output.write_text(yaml.safe_dump(result, sort_keys=False), encoding="utf-8")
        self.done = True


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output",
        default=os.path.expanduser(
            "~/antbot_data/phase2a_imu_motion_results.yaml"
        ),
    )
    parsed, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = MotionExperiment(parsed.output)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
