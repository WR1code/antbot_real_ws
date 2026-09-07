"""Runtime diagnostics for two clouds and the Isaac simulation clock."""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2

from .diagnostic_core import ClockState, StreamState


class DualLidarDiagnostics(Node):
    def __init__(self):
        super().__init__("antbot_dual_lidar_diagnostics")
        self.declare_parameter(
            "front_topic", "/antbot/lidar/front_left/points_raw_native"
        )
        self.declare_parameter(
            "rear_topic", "/antbot/lidar/rear_right/points_raw_native"
        )
        self.declare_parameter("max_cloud_time_difference_sec", 0.05)
        self.declare_parameter("cloud_timeout_sec", 1.5)
        self.declare_parameter("minimum_expected_frequency", 5.0)
        self.declare_parameter("report_period_sec", 2.0)
        self.limit = float(self.get_parameter("max_cloud_time_difference_sec").value)
        self.timeout = float(self.get_parameter("cloud_timeout_sec").value)
        self.minimum_hz = float(self.get_parameter("minimum_expected_frequency").value)
        period = float(self.get_parameter("report_period_sec").value)
        if min(self.limit, self.timeout, self.minimum_hz, period) <= 0.0:
            raise ValueError("all diagnostic limits must be positive")
        self.states = {"front_left": StreamState(), "rear_right": StreamState()}
        self.clock = ClockState()
        # Isaac and rosbag2 may offer /clock as best-effort. A best-effort
        # subscription is compatible with both best-effort and reliable writers.
        self.create_subscription(
            Clock, "/clock", self._clock_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            PointCloud2, str(self.get_parameter("front_topic").value),
            lambda msg: self._cloud("front_left", msg), qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2, str(self.get_parameter("rear_topic").value),
            lambda msg: self._cloud("rear_right", msg), qos_profile_sensor_data,
        )
        self.create_timer(period, self._report)

    def _clock_callback(self, msg):
        stamp_ns = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec
        previous_regressions = self.clock.regressions
        self.clock.update(stamp_ns)
        if self.clock.regressions > previous_regressions:
            self.get_logger().warning("simulation clock reset detected")

    def _cloud(self, key, msg):
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        reset = self.states[key].update(
            stamp_ns, time.monotonic(), msg.header.frame_id, int(msg.width * msg.height)
        )
        if reset:
            self.get_logger().warning(f"{key}: timestamp reset detected")

    def _report(self):
        now = time.monotonic()
        front, rear = self.states["front_left"], self.states["rear_right"]
        delta = None
        if front.last_stamp_ns is not None and rear.last_stamp_ns is not None:
            delta = abs(front.last_stamp_ns - rear.last_stamp_ns) * 1e-9
        for name, state in self.states.items():
            if state.timed_out(now, self.timeout):
                self.get_logger().warning(f"{name}: point cloud timeout")
            if state.count > 1 and state.frequency < self.minimum_hz:
                self.get_logger().warning(
                    f"{name}: low simulation frequency {state.frequency:.2f} Hz"
                )
        if delta is not None and delta > self.limit:
            self.get_logger().warning(f"cloud timestamp difference {delta:.4f}s exceeds limit")
        clock_delta = {}
        if self.clock.last_stamp_ns is not None:
            for name, state in self.states.items():
                if state.last_stamp_ns is not None:
                    clock_delta[name] = abs(
                        self.clock.last_stamp_ns - state.last_stamp_ns
                    ) * 1e-9
        self.get_logger().info(
            f"clock={'ok' if self.clock.count else 'missing'} "
            f"sim_ns={self.clock.last_stamp_ns} total={self.clock.count} "
            f"unique={len(self.clock.unique_stamps)} "
            f"consecutive_duplicates={self.clock.consecutive_duplicates} "
            f"nonconsecutive_duplicates={self.clock.nonconsecutive_duplicates} "
            f"regressions={self.clock.regressions} "
            f"max_duplicate_group={self.clock.maximum_group_length} "
            f"front[h={front.frequency:.2f},frame={front.frame_id},points={front.point_count}] "
            f"rear[h={rear.frequency:.2f},frame={rear.frame_id},points={rear.point_count}] "
            f"pair_delta={delta} clock_delta={clock_delta}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DualLidarDiagnostics()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
