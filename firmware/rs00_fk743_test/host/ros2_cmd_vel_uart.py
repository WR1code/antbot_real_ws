#!/usr/bin/env python3
"""Forward cmd_vel and expose STM32 motor telemetry on the same UART."""

import json
import math
import time
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import serial
from std_msgs.msg import String

from chassis_uart_protocol import (
    ACK_POSITION_INVALID,
    Ack,
    AckStreamParser,
    CHASSIS_STATE_NAMES,
    CONTROL_IDS,
    encode_cmd_vel,
    encode_control,
    limit_linear_velocity,
)
from serial_port import resolve_uart_port


class CmdVelUartBridge(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_uart_bridge")
        self.declare_parameter(
            "port",
            resolve_uart_port(),
        )
        self.declare_parameter("baud", 115200)
        self.declare_parameter("topic", "/cmd_vel")
        self.declare_parameter("max_linear_speed", 0.5)
        self.declare_parameter("status_topic", "/rs00/motor_status")
        self.declare_parameter("telemetry_period", 0.2)

        port = self.get_parameter("port").value
        baud = self.get_parameter("baud").value
        topic = self.get_parameter("topic").value
        status_topic = self.get_parameter("status_topic").value
        telemetry_period = float(
            self.get_parameter("telemetry_period").value
        )
        if telemetry_period < 0.1:
            raise ValueError("telemetry_period must be at least 0.1 seconds")
        self.max_linear_speed = float(
            self.get_parameter("max_linear_speed").value
        )
        self.serial = serial.Serial(
            port=port, baudrate=baud, timeout=0, write_timeout=0.1
        )
        self.sequence = 0
        self.ack_parser = AckStreamParser()
        self.latest_ack = None
        self.feedback = {}
        self.telemetry_index = 0
        self.telemetry_requests = [
            (CONTROL_IDS["QUERY_STATUS"], b""),
            (CONTROL_IDS["RS00_QUERY_FEEDBACK_AGE"], b""),
            (CONTROL_IDS["DRIVE_QUERY_FEEDBACK"], b"\x00"),
            (CONTROL_IDS["DRIVE_QUERY_FEEDBACK"], b"\x01"),
            (CONTROL_IDS["DRIVE_QUERY_FEEDBACK"], b"\x02"),
            (CONTROL_IDS["DRIVE_QUERY_FEEDBACK"], b"\x03"),
            (CONTROL_IDS["DRIVE_QUERY_FEEDBACK"], b"\x04"),
            (CONTROL_IDS["DRIVE_QUERY_FEEDBACK"], b"\x05"),
            (CONTROL_IDS["DRIVE_QUERY_FEEDBACK"], b"\x06"),
            (CONTROL_IDS["DRIVE_QUERY_FEEDBACK"], b"\x07"),
            (CONTROL_IDS["DRIVE_QUERY_FEEDBACK"], b"\x08"),
        ]
        self.subscription = self.create_subscription(
            Twist, topic, self.on_cmd_vel, 10
        )
        self.status_publisher = self.create_publisher(String, status_topic, 10)
        self.serial_timer = self.create_timer(0.02, self.poll_serial)
        self.telemetry_timer = self.create_timer(
            telemetry_period, self.request_telemetry
        )
        self.status_timer = self.create_timer(1.0, self.publish_status)
        self.get_logger().info(
            f"forwarding {topic} to {port} at {baud} baud; "
            f"motor telemetry: {status_topic}"
        )

    def write_frame(self, frame: bytes) -> bool:
        try:
            self.serial.write(frame)
            return True
        except serial.SerialException as error:
            self.get_logger().error(f"serial write failed: {error}")
            return False

    def on_cmd_vel(self, message: Twist) -> None:
        vx = float(message.linear.x)
        vy = float(message.linear.y)
        wz = float(message.angular.z)
        if not all(math.isfinite(value) for value in (vx, vy, wz)):
            self.get_logger().error("discarded non-finite cmd_vel")
            return

        magnitude = math.hypot(vx, vy)
        if magnitude > self.max_linear_speed and magnitude > 0.0:
            vx, vy = limit_linear_velocity(
                vx, vy, self.max_linear_speed
            )
            self.get_logger().warning(
                f"linear cmd_vel limited to {self.max_linear_speed:.3f} m/s",
                throttle_duration_sec=1.0,
            )

        frame = encode_cmd_vel(self.sequence, vx, vy, wz)
        if not self.write_frame(frame):
            return
        self.sequence = (self.sequence + 1) & 0xFF

    def request_telemetry(self) -> None:
        command_id, payload = self.telemetry_requests[self.telemetry_index]
        frame = encode_control(command_id, self.sequence, payload)
        if self.write_frame(frame):
            self.sequence = (self.sequence + 1) & 0xFF
            self.telemetry_index = (
                self.telemetry_index + 1
            ) % len(self.telemetry_requests)

    def poll_serial(self) -> None:
        try:
            waiting = self.serial.in_waiting
            if not waiting:
                return
            data = self.serial.read(waiting)
        except serial.SerialException as error:
            self.get_logger().error(f"serial read failed: {error}")
            return
        for ack in self.ack_parser.feed(data):
            self.latest_ack = ack
            if 2 <= ack.detail_type <= 9 or 17 <= ack.detail_type <= 19:
                self.feedback[ack.detail_type] = ack

    @staticmethod
    def detail_values(ack: Ack | None, scale: float = 1.0):
        if ack is None:
            return [None] * 4
        return [
            value * scale if ack.detail_valid_mask & (1 << index) else None
            for index, value in enumerate(ack.detail_values)
        ]

    def publish_status(self) -> None:
        ack = self.latest_ack
        if ack is None:
            self.get_logger().warning(
                "等待 STM32 状态回包", throttle_duration_sec=2.0
            )
            return
        steering_deg = [
            None if value == ACK_POSITION_INVALID else round(value * 0.0572958, 1)
            for value in ack.steering_position_mrad
        ]
        status = {
            "chassis_state": CHASSIS_STATE_NAMES.get(
                ack.chassis_state, str(ack.chassis_state)
            ),
            "fault_flags": ack.fault_flags,
            "steering": {
                "enabled": bool(ack.steering_flags & 0x01),
                "homed": bool(ack.steering_flags & 0x02),
                "ready": bool(ack.steering_flags & 0x04),
                "fault": bool(ack.steering_flags & 0x08),
                "position_deg": steering_deg,
                "feedback_age_ms": self.detail_values(self.feedback.get(9)),
            },
            "mini": {
                "speed_erpm": self.detail_values(self.feedback.get(2)),
                "current_a": self.detail_values(self.feedback.get(3), 0.01),
                "position_deg": self.detail_values(self.feedback.get(4), 0.01),
                "temperature_c": self.detail_values(self.feedback.get(5)),
                "fault_code": self.detail_values(self.feedback.get(6)),
                # Backward-compatible alias. Values are single codes, not bits.
                "fault_bits": self.detail_values(self.feedback.get(6)),
                "valid_mask": self.detail_values(self.feedback.get(7)),
                "feedback_age_ms": self.detail_values(self.feedback.get(17)),
                "safety_flags": self.detail_values(self.feedback.get(18)),
                "voltage_v": self.detail_values(self.feedback.get(19)),
            },
            "can": {
                "flags": ack.can_flags,
                "tx_count": ack.can_tx_count,
                "rx_count": ack.can_rx_count,
            },
            "stm32_tick_ms": ack.stm32_tick,
        }
        message = String()
        message.data = json.dumps(status, ensure_ascii=False, separators=(",", ":"))
        self.status_publisher.publish(message)
        speed = status["mini"]["speed_erpm"]
        current = status["mini"]["current_a"]
        faults = status["mini"]["fault_code"]
        self.get_logger().info(
            f"状态={status['chassis_state']} RS00角度°={steering_deg} "
            f"MINI转速erpm={speed} 电流A={current} 故障={faults}"
        )

    def destroy_node(self):
        if self.serial.is_open:
            # Do not rely only on the MCU's 300 ms watchdog during a normal
            # ROS shutdown.  Redundant zero frames stop the chassis first.
            try:
                for _ in range(5):
                    self.serial.write(
                        encode_cmd_vel(self.sequence, 0.0, 0.0, 0.0)
                    )
                    self.sequence = (self.sequence + 1) & 0xFF
                    time.sleep(0.01)
                self.serial.flush()
            except serial.SerialException as error:
                self.get_logger().error(
                    f"failed to send shutdown stop frames: {error}"
                )
            self.serial.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = CmdVelUartBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
