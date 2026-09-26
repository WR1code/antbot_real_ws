"""Read-only ROS 2 bridge for the Piper-H-mounted three-channel pulse sensor."""

from __future__ import annotations

from collections import deque
import math
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import FluidPressure, Temperature
import serial
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from .pressure_protocol import parse_pressure_line
from .pressure_serial_owner import (
    PressureSerialLease, SerialDeviceInUseError, serial_device,
)


class PressureSerialBridge(Node):
    def __init__(self) -> None:
        super().__init__("pressure_serial_bridge")
        self.declare_parameter("port", "")
        self.declare_parameter("baud_rate", 921600)
        self.declare_parameter("topic_prefix", "/piperh/pulse")
        self.declare_parameter("sensor_frame", "")
        self.declare_parameter("zero_calibrated", False)
        self.declare_parameter("zero_baselines_pa", [0.0, 0.0, 0.0])
        self.declare_parameter("zero_window_seconds", 3.0)
        self.declare_parameter("zero_min_samples", 30)
        self.port = str(self.get_parameter("port").value).strip()
        if not self.port:
            raise ValueError("port is required; use the ESP32-S3 /dev/serial/by-id path")
        self.baud_rate = int(self.get_parameter("baud_rate").value)
        prefix = str(self.get_parameter("topic_prefix").value).rstrip("/")
        self.sensor_frame = str(self.get_parameter("sensor_frame").value)
        self.zero_calibrated = bool(self.get_parameter("zero_calibrated").value)
        baseline_values = list(self.get_parameter("zero_baselines_pa").value)
        self.zero_window_seconds = float(
            self.get_parameter("zero_window_seconds").value
        )
        self.zero_min_samples = int(self.get_parameter("zero_min_samples").value)
        if not prefix.startswith("/") or self.baud_rate <= 0:
            raise ValueError("topic_prefix must be absolute and baud_rate must be positive")
        if len(baseline_values) != 3 or not all(
            math.isfinite(float(value)) for value in baseline_values
        ):
            raise ValueError("zero_baselines_pa must contain three finite values")
        if self.zero_calibrated and not all(
            float(value) > 0.0 for value in baseline_values
        ):
            raise ValueError(
                "calibrated zero_baselines_pa must contain three positive values"
            )
        if self.zero_window_seconds <= 0.0 or self.zero_min_samples < 3:
            raise ValueError(
                "zero_window_seconds must be positive and zero_min_samples at least 3"
            )
        self.zero_baselines_pa = {
            channel: float(baseline_values[index])
            for index, channel in enumerate(("S1", "S2", "S3"))
        }
        # Acquire before creating any publishers. A duplicate bridge must not
        # leave a latched serial_connected=false beside the real data owner.
        self.serial_lease = PressureSerialLease(self.port)
        self.serial_lease.acquire()

        self.pressure_publishers = {
            channel: self.create_publisher(
                FluidPressure, f"{prefix}/raw/{channel.lower()}/pressure", 50
            )
            for channel in ("S1", "S2", "S3")
        }
        self.temperature_publishers = {
            channel: self.create_publisher(
                Temperature, f"{prefix}/raw/{channel.lower()}/temperature", 50
            )
            for channel in ("S1", "S2", "S3")
        }
        self.zeroed_pressure_publishers = {
            channel: self.create_publisher(
                FluidPressure, f"{prefix}/zeroed/{channel.lower()}/pressure", 50
            )
            for channel in ("S1", "S2", "S3")
        }
        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status_publisher = self.create_publisher(
            Bool, f"{prefix}/serial_connected", status_qos
        )
        self.zero_status_publisher = self.create_publisher(
            Bool, f"{prefix}/zero_calibrated", status_qos
        )
        self.recent_pressure_pa = {
            channel: deque(maxlen=2000) for channel in ("S1", "S2", "S3")
        }
        self.create_service(
            Trigger, f"{prefix}/calibrate_zero", self._calibrate_zero
        )
        self.connection: serial.Serial | None = None
        self.buffer = bytearray()
        self.next_open_at = 0.0
        self.connected = False
        self.status_publisher.publish(Bool(data=False))
        self.zero_status_publisher.publish(Bool(data=self.zero_calibrated))
        self.create_timer(0.01, self.poll)

    def _calibrate_zero(self, request, response):
        del request
        now = time.monotonic()
        baselines = {}
        counts = {}
        for channel, samples in self.recent_pressure_pa.items():
            fresh = [
                pressure
                for received_at, pressure in samples
                if now - received_at <= self.zero_window_seconds
            ]
            counts[channel] = len(fresh)
            if len(fresh) < self.zero_min_samples:
                response.success = False
                response.message = (
                    "insufficient fresh samples: "
                    + ", ".join(f"{key}={value}" for key, value in counts.items())
                )
                return response
            baselines[channel] = statistics.median(fresh)
        self.zero_baselines_pa.update(baselines)
        self.zero_calibrated = True
        self.zero_status_publisher.publish(Bool(data=True))
        response.success = True
        response.message = ", ".join(
            f"{channel}={baselines[channel]:.3f} Pa (n={counts[channel]})"
            for channel in ("S1", "S2", "S3")
        )
        self.get_logger().info(f"zero calibrated: {response.message}")
        return response

    def _set_connected(self, value: bool) -> None:
        if value != self.connected:
            self.connected = value
            if rclpy.ok(context=self.context):
                self.status_publisher.publish(Bool(data=value))

    def _close(self) -> None:
        if self.connection is not None:
            try:
                self.connection.close()
            except serial.SerialException:
                pass
        self.connection = None
        self.buffer.clear()
        self.next_open_at = time.monotonic() + 2.0
        self._set_connected(False)

    def poll(self) -> None:
        if self.connection is None:
            if time.monotonic() < self.next_open_at:
                return
            if serial_device(self.port) != self.serial_lease.device:
                raise RuntimeError(
                    "pressure serial device identity changed; restart bridge "
                    "to acquire a lock for the new device"
                )
            try:
                self.connection = serial.Serial(
                    self.port, self.baud_rate, timeout=0, write_timeout=0,
                    exclusive=True,
                )
                self.get_logger().info(f"reading ESP32-S3 from {self.port}")
                self._set_connected(True)
            except serial.SerialException as error:
                self.get_logger().warn(f"ESP32-S3 serial unavailable: {error}")
                self._close()
                return
        try:
            data = self.connection.read(min(max(self.connection.in_waiting, 1), 65536))
        except (serial.SerialException, OSError) as error:
            self.get_logger().warn(f"ESP32-S3 serial disconnected: {error}")
            self._close()
            return
        if not data:
            return
        self.buffer.extend(data)
        if len(self.buffer) > 65536:
            self.get_logger().warn("discarding oversized ESP32-S3 serial buffer")
            self.buffer.clear()
            return
        while b"\n" in self.buffer:
            raw, _, rest = self.buffer.partition(b"\n")
            self.buffer = bytearray(rest)
            measurement = parse_pressure_line(raw.decode("ascii", errors="replace"))
            if measurement is None:
                continue
            channel, pressure_hpa, temperature_c = measurement
            # Firmware has no sample timestamp; this is host receipt time.
            stamp = self.get_clock().now().to_msg()
            pressure = FluidPressure()
            pressure.header.stamp = stamp
            pressure.header.frame_id = self.sensor_frame
            pressure.fluid_pressure = pressure_hpa * 100.0
            self.pressure_publishers[channel].publish(pressure)
            self.recent_pressure_pa[channel].append(
                (time.monotonic(), pressure.fluid_pressure)
            )
            if self.zero_calibrated:
                zeroed = FluidPressure()
                zeroed.header = pressure.header
                zeroed.fluid_pressure = (
                    pressure.fluid_pressure - self.zero_baselines_pa[channel]
                )
                self.zeroed_pressure_publishers[channel].publish(zeroed)
            temperature = Temperature()
            temperature.header = pressure.header
            temperature.temperature = temperature_c
            self.temperature_publishers[channel].publish(temperature)


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = PressureSerialBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except (SerialDeviceInUseError, OSError, RuntimeError) as error:
        print(f"FATAL pressure_serial_bridge: {error}", file=sys.stderr)
        raise SystemExit(2) from error
    finally:
        if node is not None:
            node._close()
            node.serial_lease.release()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
