#!/usr/bin/env python3
"""Forward cmd_vel and expose STM32 motor telemetry on the same UART."""

import json
import math
import time
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import BatteryState, JointState
import serial
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from .chassis_uart_protocol import (
    ACK_POSITION_INVALID,
    Ack,
    AckStreamParser,
    CHASSIS_STATE_NAMES,
    CONTROL_IDS,
    encode_cmd_vel,
    encode_control,
    limit_linear_velocity,
)
from .serial_port import resolve_uart_port


WHEEL_CORNERS = ("front_left", "front_right", "rear_left", "rear_right")
JOINT_NAMES = tuple(
    joint_name
    for corner in WHEEL_CORNERS
    for joint_name in (f"steering_{corner}_joint", f"wheel_{corner}_joint")
)


def estimate_battery_percentage(
    voltage: float,
    empty_voltage: float,
    full_voltage: float,
) -> float:
    """Estimate charge from the configured DC-bus voltage endpoints."""
    if not all(math.isfinite(value) for value in (
        voltage, empty_voltage, full_voltage
    )) or full_voltage <= empty_voltage:
        return math.nan
    return max(0.0, min(1.0, (
        voltage - empty_voltage
    ) / (full_voltage - empty_voltage)))


def joint_positions_from_feedback(
    ack: Ack | None,
    drive_position_ack: Ack | None,
    previous: list[float] | tuple[float, ...] | None = None,
) -> list[float]:
    """Convert H743 feedback into the eight URDF joint positions in radians."""
    positions = list(previous) if previous is not None else [0.0] * len(JOINT_NAMES)
    if len(positions) != len(JOINT_NAMES):
        raise ValueError("previous joint position count does not match AntBot URDF")

    if ack is not None:
        for index, value_mrad in enumerate(ack.steering_position_mrad):
            if value_mrad != ACK_POSITION_INVALID:
                positions[index * 2] = value_mrad * 0.001

    if drive_position_ack is not None and drive_position_ack.detail_type == 4:
        for index, value_centideg in enumerate(drive_position_ack.detail_values):
            if drive_position_ack.detail_valid_mask & (1 << index):
                positions[index * 2 + 1] = math.radians(value_centideg * 0.01)
    return positions


def ack_is_motion_ready(ack: Ack | None) -> bool:
    """Return whether fresh H743 telemetry permits host motion commands."""
    return bool(
        ack is not None
        and ack.fault_flags == 0
        and ack.steering_flags & 0x04
        and not ack.steering_flags & 0x08
    )


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
        self.declare_parameter("vehicle_status_topic", "/antbot/vehicle_status")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("battery_topic", "/battery")
        self.declare_parameter("battery_empty_voltage", 18.0)
        self.declare_parameter("battery_full_voltage", 30.0)
        self.declare_parameter("telemetry_period", 0.2)
        self.declare_parameter("allow_disconnected", False)
        self.declare_parameter("reconnect_period", 1.0)
        self.declare_parameter("telemetry_timeout", 1.5)
        self.declare_parameter(
            "operator_enable_service", "/antbot/operator_enable"
        )
        self.declare_parameter(
            "system_reset_service", "/antbot/system_reset"
        )

        self.port = str(self.get_parameter("port").value)
        self.baud = int(self.get_parameter("baud").value)
        topic = self.get_parameter("topic").value
        status_topic = self.get_parameter("status_topic").value
        vehicle_status_topic = self.get_parameter(
            "vehicle_status_topic"
        ).value
        joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        battery_topic = str(self.get_parameter("battery_topic").value)
        self.battery_empty_voltage = float(
            self.get_parameter("battery_empty_voltage").value
        )
        self.battery_full_voltage = float(
            self.get_parameter("battery_full_voltage").value
        )
        telemetry_period = float(
            self.get_parameter("telemetry_period").value
        )
        reconnect_period = float(
            self.get_parameter("reconnect_period").value
        )
        self.telemetry_timeout = float(
            self.get_parameter("telemetry_timeout").value
        )
        if telemetry_period < 0.1:
            raise ValueError("telemetry_period must be at least 0.1 seconds")
        if reconnect_period < 0.2:
            raise ValueError("reconnect_period must be at least 0.2 seconds")
        if self.telemetry_timeout <= telemetry_period:
            raise ValueError("telemetry_timeout must exceed telemetry_period")
        self.max_linear_speed = float(
            self.get_parameter("max_linear_speed").value
        )
        self.allow_disconnected = bool(
            self.get_parameter("allow_disconnected").value
        )
        self.serial = None
        self.connection_error = "尚未连接"
        self.operator_requested = False
        self.sequence = 0
        self.ack_parser = AckStreamParser()
        self.latest_ack = None
        self.last_ack_monotonic = 0.0
        self.feedback = {}
        # Publish a complete zero pose before hardware feedback arrives.  This
        # keeps robot_state_publisher's dynamic wheel transforms connected in
        # the offline operator UI; valid H743 feedback replaces each value.
        self.joint_positions = [0.0] * len(JOINT_NAMES)
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
        self.vehicle_status_publisher = (
            None if vehicle_status_topic == status_topic
            else self.create_publisher(String, vehicle_status_topic, 10)
        )
        self.joint_state_publisher = self.create_publisher(
            JointState, joint_state_topic, 10
        )
        self.battery_publisher = self.create_publisher(
            BatteryState, battery_topic, 10
        )
        self.operator_service = self.create_service(
            SetBool,
            str(self.get_parameter("operator_enable_service").value),
            self.set_operator_enabled,
        )
        self.system_reset_service = self.create_service(
            Trigger,
            str(self.get_parameter("system_reset_service").value),
            self.reset_system,
        )
        self.serial_timer = self.create_timer(0.02, self.poll_serial)
        self.telemetry_timer = self.create_timer(
            telemetry_period, self.request_telemetry
        )
        self.status_timer = self.create_timer(1.0, self.publish_status)
        self.joint_state_timer = self.create_timer(0.1, self.publish_joint_states)
        self.reconnect_timer = self.create_timer(
            reconnect_period, self.try_connect
        )
        if not self.try_connect() and not self.allow_disconnected:
            raise RuntimeError(
                f"cannot open H743 serial port {self.port}: "
                f"{self.connection_error}"
            )
        self.get_logger().info(
            f"guarding {topic} for {self.port} at {self.baud} baud; "
            "motion remains locked until /antbot/operator_enable succeeds"
        )

    def try_connect(self) -> bool:
        """Open the configured UART, keeping the node alive when permitted."""
        if self.serial is not None and self.serial.is_open:
            return True
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=0,
                write_timeout=0.1,
            )
        except (OSError, serial.SerialException) as error:
            self.serial = None
            self.connection_error = str(error)
            self.get_logger().warning(
                f"H743 未连接：{self.port}（{error}）",
                throttle_duration_sec=5.0,
            )
            return False
        self.connection_error = ""
        self.operator_requested = False
        self.latest_ack = None
        self.last_ack_monotonic = 0.0
        self.feedback.clear()
        self.ack_parser = AckStreamParser()
        self.get_logger().info(f"H743 已连接：{self.port}")
        return True

    def mark_disconnected(self, error: Exception) -> None:
        """Close a failed UART and require a fresh operator confirmation."""
        self.connection_error = str(error)
        self.operator_requested = False
        if self.serial is not None:
            try:
                self.serial.close()
            except (OSError, serial.SerialException):
                pass
        self.serial = None
        self.latest_ack = None
        self.last_ack_monotonic = 0.0
        self.feedback.clear()
        self.get_logger().error(f"H743 连接断开并已锁定：{error}")

    def write_frame(self, frame: bytes) -> bool:
        if self.serial is None:
            return False
        try:
            self.serial.write(frame)
            return True
        except (OSError, serial.SerialException) as error:
            self.mark_disconnected(error)
            return False

    def send_control(self, command_id: int, payload: bytes = b"") -> bool:
        """Send one sequenced H743 control request."""
        frame = encode_control(command_id, self.sequence, payload)
        if not self.write_frame(frame):
            return False
        self.sequence = (self.sequence + 1) & 0xFF
        return True

    def send_stop_frames(self) -> None:
        """Send redundant zero velocity frames when a UART is available."""
        if self.serial is None:
            return
        for _ in range(5):
            frame = encode_cmd_vel(self.sequence, 0.0, 0.0, 0.0)
            if not self.write_frame(frame):
                return
            self.sequence = (self.sequence + 1) & 0xFF
            time.sleep(0.01)

    def set_operator_enabled(self, request, response):
        """Apply the RViz operator gate without ever enabling offline."""
        if not request.data:
            self.operator_requested = False
            self.send_stop_frames()
            response.success = True
            response.message = "控制已锁定，底盘已发送零速"
            self.publish_status()
            return response

        if self.serial is None:
            response.success = False
            response.message = "H743 未连接，保持锁定"
            self.publish_status()
            return response
        if not self.send_control(CONTROL_IDS["STEERING_ENABLE"]):
            response.success = False
            response.message = "转向使能请求发送失败，保持锁定"
            return response

        self.operator_requested = True
        response.success = True
        response.message = "安全确认已提交；等待 H743 ready 后开放运动"
        self.publish_status()
        return response

    def reset_system(self, _request, response):
        """Lock motion and request a guarded H743 system reset."""
        self.operator_requested = False
        if self.serial is None:
            response.success = False
            response.message = "H743 未连接，RESET 未发送"
            self.publish_status()
            return response

        self.send_stop_frames()
        if self.serial is None or not self.send_control(
            CONTROL_IDS["SYSTEM_RESET"], b"RST!"
        ):
            response.success = False
            response.message = "RESET 指令发送失败，安全门禁保持锁定"
            self.publish_status()
            return response

        self.latest_ack = None
        self.last_ack_monotonic = 0.0
        self.feedback.clear()
        self.ack_parser = AckStreamParser()
        response.success = True
        response.message = "RESET 已发送，安全门禁已锁定；正在等待 H743 重启"
        self.publish_status()
        return response

    def motion_allowed(self) -> bool:
        """Require both the RViz gate and fault-free H743 readiness."""
        telemetry_fresh = (
            self.last_ack_monotonic > 0.0
            and time.monotonic() - self.last_ack_monotonic
            <= self.telemetry_timeout
        )
        return (
            self.operator_requested
            and telemetry_fresh
            and ack_is_motion_ready(self.latest_ack)
        )

    def on_cmd_vel(self, message: Twist) -> None:
        vx = float(message.linear.x)
        vy = float(message.linear.y)
        wz = float(message.angular.z)
        if not all(math.isfinite(value) for value in (vx, vy, wz)):
            self.get_logger().error("discarded non-finite cmd_vel")
            return

        if abs(wz) > 1.0e-6:
            vx = 0.0
            vy = 0.0
            wz = max(-1.0, min(1.0, wz))

        if not self.motion_allowed():
            if (abs(vx) > 1.0e-6 or abs(vy) > 1.0e-6
                    or abs(wz) > 1.0e-6):
                self.get_logger().warning(
                    "motion command blocked: operator/H743 safety gate is locked",
                    throttle_duration_sec=1.0,
                )
            vx = 0.0
            vy = 0.0
            wz = 0.0

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
        if self.serial is None:
            return
        command_id, payload = self.telemetry_requests[self.telemetry_index]
        if self.send_control(command_id, payload):
            self.telemetry_index = (
                self.telemetry_index + 1
            ) % len(self.telemetry_requests)

    def poll_serial(self) -> None:
        if self.serial is None:
            return
        try:
            waiting = self.serial.in_waiting
            if not waiting:
                return
            data = self.serial.read(waiting)
        except (OSError, serial.SerialException) as error:
            self.mark_disconnected(error)
            return
        for ack in self.ack_parser.feed(data):
            self.latest_ack = ack
            self.last_ack_monotonic = time.monotonic()
            if ack.fault_flags or ack.steering_flags & 0x08:
                self.operator_requested = False
            if 2 <= ack.detail_type <= 9 or 17 <= ack.detail_type <= 19:
                self.feedback[ack.detail_type] = ack
            self.joint_positions = joint_positions_from_feedback(
                ack, self.feedback.get(4), self.joint_positions
            )

    def publish_joint_states(self) -> None:
        """Keep all four steering and wheel links connected to the TF tree."""
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(JOINT_NAMES)
        message.position = list(self.joint_positions)
        self.joint_state_publisher.publish(message)

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
        connected = self.serial is not None
        telemetry_fresh = (
            self.last_ack_monotonic > 0.0
            and time.monotonic() - self.last_ack_monotonic
            <= self.telemetry_timeout
        )
        if self.operator_requested and not telemetry_fresh and ack is not None:
            self.operator_requested = False
        base_status = {
            "connection": "connected" if connected else "disconnected",
            "connection_detail": self.port if connected else self.connection_error,
            "autonomous_navigation": "disabled_no_odom_or_angular_z",
            "operator_requested": self.operator_requested,
            "operator_enabled": self.motion_allowed(),
            "telemetry_fresh": telemetry_fresh,
        }
        if ack is None:
            base_status["chassis_state"] = (
                "WAITING_TELEMETRY" if connected else "DISCONNECTED"
            )
            base_status["fault_flags"] = 0
            self.publish_status_message(base_status)
            return
        steering_deg = [
            None if value == ACK_POSITION_INVALID else round(value * 0.0572958, 1)
            for value in ack.steering_position_mrad
        ]
        status = {
            **base_status,
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
        self.publish_status_message(status)
        self.publish_battery_state(status)
        speed = status["mini"]["speed_erpm"]
        current = status["mini"]["current_a"]
        faults = status["mini"]["fault_code"]
        self.get_logger().info(
            f"状态={status['chassis_state']} RS00角度°={steering_deg} "
            f"MINI转速erpm={speed} 电流A={current} 故障={faults}"
        )

    def publish_battery_state(self, status: dict) -> None:
        """Publish a documented voltage-based estimate for the 24 V bus."""
        voltages = [
            float(value) for value in status["mini"]["voltage_v"]
            if value is not None and math.isfinite(float(value))
        ]
        if not voltages:
            return
        temperatures = [
            float(value) for value in status["mini"]["temperature_c"]
            if value is not None and math.isfinite(float(value))
        ]
        faults = [
            int(value) for value in status["mini"]["fault_code"]
            if value is not None
        ]
        voltage = sum(voltages) / len(voltages)
        message = BatteryState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.voltage = voltage
        message.temperature = max(temperatures) if temperatures else math.nan
        message.current = math.nan
        message.charge = math.nan
        message.capacity = math.nan
        message.design_capacity = math.nan
        message.percentage = estimate_battery_percentage(
            voltage,
            self.battery_empty_voltage,
            self.battery_full_voltage,
        )
        message.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        message.power_supply_health = (
            BatteryState.POWER_SUPPLY_HEALTH_GOOD
            if faults and all(value == 0 for value in faults)
            else BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        )
        message.power_supply_technology = (
            BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        )
        message.present = True
        message.location = "MINI 24 V DC bus (voltage estimate)"
        self.battery_publisher.publish(message)

    def publish_status_message(self, status: dict) -> None:
        """Publish the same status to the legacy and RViz-facing topics."""
        message = String()
        message.data = json.dumps(
            status, ensure_ascii=False, separators=(",", ":")
        )
        self.status_publisher.publish(message)
        if self.vehicle_status_publisher is not None:
            self.vehicle_status_publisher.publish(message)

    def destroy_node(self):
        self.operator_requested = False
        if self.serial is not None and self.serial.is_open:
            # Do not rely only on the MCU's 300 ms watchdog during a normal
            # ROS shutdown.  Redundant zero frames stop the chassis first.
            try:
                self.send_stop_frames()
                if self.serial is not None:
                    self.serial.flush()
            except KeyboardInterrupt:
                # A second shutdown signal may arrive while the redundant
                # zero frames are being sent. Closing the port below remains
                # safe and must not turn an orderly shutdown into a traceback.
                pass
            except serial.SerialException as error:
                self.get_logger().error(
                    f"failed to send shutdown stop frames: {error}"
                )
            if self.serial is not None:
                self.serial.close()
                self.serial = None
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
