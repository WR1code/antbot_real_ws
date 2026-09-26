"""Fail-closed checks performed after sensors start and before FAST-LIO starts."""

import os
import shutil
import subprocess
import time

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from livox_ros_driver2.msg import CustomMsg


class MappingPreflight(Node):
    def __init__(self):
        super().__init__("antbot_mapping_preflight")
        self.declare_parameter("front_ip", "")
        self.declare_parameter("rear_ip", "")
        self.declare_parameter("config_file", "")
        self.declare_parameter("sample_seconds", 4.0)
        self.declare_parameter("minimum_free_gib", 10.0)
        self.front_ip = str(self.get_parameter("front_ip").value)
        self.rear_ip = str(self.get_parameter("rear_ip").value)
        self.config_file = str(self.get_parameter("config_file").value)
        self.sample_seconds = float(self.get_parameter("sample_seconds").value)
        self.samples = {"front": [], "rear": [], "imu": []}
        self.direct_transforms = set()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        sensor_qos = QoSProfile(
            depth=50, reliability=ReliabilityPolicy.BEST_EFFORT
        )
        static_qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            CustomMsg, f"/livox/lidar_{self.front_ip.replace('.', '_')}",
            lambda msg: self.samples["front"].append(msg), sensor_qos
        )
        self.create_subscription(
            CustomMsg, f"/livox/lidar_{self.rear_ip.replace('.', '_')}",
            lambda msg: self.samples["rear"].append(msg), sensor_qos
        )
        self.create_subscription(
            Imu, "/antbot/lidar/front_left/imu_raw_si",
            lambda msg: self.samples["imu"].append(msg), sensor_qos
        )
        self.create_subscription(TFMessage, "/tf", self._tf_callback, sensor_qos)
        self.create_subscription(
            TFMessage, "/tf_static", self._tf_callback, static_qos
        )

    def _tf_callback(self, message):
        for transform in message.transforms:
            self.direct_transforms.add(
                (transform.header.frame_id.lstrip("/"),
                 transform.child_frame_id.lstrip("/"))
            )

    def _record(self, ok, label, detail, failures):
        state = "PASS" if ok else "FAIL"
        self.get_logger().info(f"[{state}] {label}: {detail}")
        if not ok:
            failures.append(f"{label}: {detail}")

    def run(self):
        failures = []
        for label, address in (("front MID360S", self.front_ip),
                               ("rear MID360S", self.rear_ip)):
            result = subprocess.run(
                ["ping", "-c", "1", "-W", "1", address],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                check=False,
            )
            self._record(result.returncode == 0, label, address, failures)

        deadline = time.monotonic() + self.sample_seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

        for key, expected_hz in (("front", 10.0), ("rear", 10.0), ("imu", 200.0)):
            messages = self.samples[key]
            stamps = [
                msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                for msg in messages
            ]
            duration = stamps[-1] - stamps[0] if len(stamps) > 1 else 0.0
            rate = (len(stamps) - 1) / duration if duration > 0.0 else 0.0
            monotonic = all(b > a for a, b in zip(stamps, stamps[1:]))
            ok = rate >= expected_hz * 0.8 and monotonic
            self._record(
                ok, f"{key} rate/time", f"{rate:.2f} Hz; monotonic={monotonic}", failures
            )

        for key in ("front", "rear"):
            messages = self.samples[key]
            offsets = list(messages[-1].points) if messages else []
            valid = (
                bool(messages) and messages[-1].point_num == len(offsets)
                and len(offsets) > 1
                and offsets[-1].offset_time > offsets[0].offset_time
                and offsets[-1].offset_time - offsets[0].offset_time > 50_000_000
            )
            span_ms = (
                (offsets[-1].offset_time - offsets[0].offset_time) / 1e6
                if len(offsets) > 1 else 0.0
            )
            self._record(
                valid, f"{key} per-point time", f"CustomMsg span={span_ms:.2f} ms", failures
            )

        for target in (
            "lidar_2d_front_scan", "lidar_2d_back_scan", "mid360_front_imu"
        ):
            available = self.tf_buffer.can_transform(
                "base_link", target, rclpy.time.Time(), timeout=Duration(seconds=0.5)
            )
            self._record(
                available, f"base_link -> {target}", "TF lookup", failures
            )

        conflicts = sorted(
            parent for parent, child in self.direct_transforms
            if child == "base_link" and parent in {"world", "map", "odom"}
        )
        self._record(
            not conflicts, "mapping TF conflict",
            "none" if not conflicts else f"direct parents present: {conflicts}", failures
        )

        try:
            prefix = get_package_prefix("fast_lio")
            executable = os.path.join(prefix, "lib", "fast_lio", "fastlio_mapping")
            lio_ok = os.path.isfile(executable) and os.access(executable, os.X_OK)
        except LookupError:
            executable = "package not found"
            lio_ok = False
        self._record(lio_ok, "FAST-LIO executable", executable, failures)
        self._record(
            os.path.isfile(self.config_file), "FAST-LIO config", self.config_file, failures
        )
        free_gib = shutil.disk_usage(os.path.dirname(self.config_file) or ".").free / 2**30
        minimum = float(self.get_parameter("minimum_free_gib").value)
        self._record(
            free_gib >= minimum, "disk space", f"{free_gib:.1f} GiB free", failures
        )

        if failures:
            self.get_logger().error("PRE-MAPPING PREFLIGHT FAILED")
            for failure in failures:
                self.get_logger().error(f"  - {failure}")
            return 1
        self.get_logger().info("PRE-MAPPING PREFLIGHT PASSED; FAST-LIO may start")
        return 0


def main(args=None):
    rclpy.init(args=args)
    node = MappingPreflight()
    try:
        return_code = node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(return_code)
