"""Shared STM32 chassis UART command/ACK wire protocol."""

from __future__ import annotations

from dataclasses import dataclass
import math
import struct

CMD_SYNC = b"\xAA\x55"
CMD_VERSION = 1
CMD_FRAME_SIZE = 12
ACK_SYNC = b"\xA5\x5A"
ACK_VERSION = 3
ACK_FRAME_SIZE = 52
ACK_POSITION_INVALID = -32768
CONTROL_VERSION = 2
CONTROL_FRAME_SIZE = 16

CONTROL_IDS = {
    "QUERY_STATUS": 0x01,
    "QUERY_SYSTEM_BOOT": 0x02,
    "QUERY_SYSTEM_RUNTIME": 0x03,
    "QUERY_FIRMWARE": 0x04,
    "QUERY_CRASH_REGISTERS": 0x05,
    "QUERY_CRASH_FAULTS": 0x06,
    "QUERY_EVENT_LOG": 0x07,
    "QUERY_POWER": 0x08,
    "STEERING_ENABLE": 0x10,
    "STEERING_SET_ALL": 0x11,
    "STEERING_SET_EACH": 0x12,
    "STEERING_DISABLE": 0x13,
    "RS00_CLEAR_FAULT": 0x14,
    "RS00_QUERY_UID": 0x15,
    "RS00_READ_POSITION": 0x16,
    "RS00_SET_CSP": 0x17,
    "RS00_SET_LIMITS": 0x18,
    "RS00_QUERY_FEEDBACK_AGE": 0x19,
    "DRIVE_SET_ACCELERATION": 0x20,
    "DRIVE_SET_DECELERATION": 0x21,
    "DRIVE_SET_ALL": 0x22,
    "DRIVE_STOP": 0x23,
    "DRIVE_WITHDRAW_CURRENT": 0x24,
    "DRIVE_QUERY_FEEDBACK": 0x25,
    "EMERGENCY_STOP": 0x30,
    "CLEAR_FAULT_RESTART": 0x31,
    "SYSTEM_RESET": 0x32,
}

ACK_STATUS_NAMES = {
    0: "ACK_OK",
    1: "ACK_BAD_HEADER",
    2: "ACK_BAD_LENGTH",
    3: "ACK_BAD_CRC",
    4: "ACK_UNSUPPORTED_COMMAND",
    5: "ACK_REJECTED_DISABLED",
    6: "ACK_REJECTED_NOT_HOMED",
    7: "ACK_REJECTED_STEERING_FAULT",
    8: "ACK_REJECTED_ANGULAR_Z",
    9: "ACK_ACCEPTED_WAIT_STEERING",
    10: "ACK_ACCEPTED_DRIVING",
    11: "ACK_TIMEOUT_STOP",
    12: "ACK_CAN_TX_ERROR",
    13: "ACK_CAN_BUS_OFF",
    14: "ACK_ACCEPTED_RESET",
}

CHASSIS_STATE_NAMES = {
    0: "BOOT",
    1: "DISABLED",
    2: "WAIT_ENABLE",
    3: "WAIT_HOMING",
    4: "IDLE",
    5: "STEERING",
    6: "DRIVE",
    7: "TIMEOUT_STOP",
    8: "FAULT",
    9: "EMERGENCY_STOP",
}

REJECT_REASON_NAMES = {
    0: "NONE",
    1: "DISABLED",
    2: "NOT_HOMED",
    3: "STEERING_FAULT",
    4: "NONZERO_ANGULAR_Z",
    5: "COMM_TIMEOUT",
    6: "INVALID_COMMAND",
    7: "CAN_FAULT",
}


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = (
                ((crc << 1) ^ 0x1021) & 0xFFFF
                if crc & 0x8000
                else (crc << 1) & 0xFFFF
            )
    return crc


def limit_linear_velocity(vx: float, vy: float, maximum: float) -> tuple[float, float]:
    if maximum <= 0.0:
        raise ValueError("maximum linear speed must be positive")
    magnitude = math.hypot(vx, vy)
    if magnitude > maximum:
        scale = maximum / magnitude
        return vx * scale, vy * scale
    return vx, vy


def encode_cmd_vel(sequence: int, vx: float, vy: float, wz: float) -> bytes:
    if not all(math.isfinite(value) for value in (vx, vy, wz)):
        raise ValueError("cmd_vel values must be finite")
    values = []
    for value in (vx, vy, wz):
        encoded = round(value * 1000.0)
        if not -32768 <= encoded <= 32767:
            raise ValueError("cmd_vel value exceeds the int16 wire range")
        values.append(encoded)
    payload = struct.pack(
        "<2sBBhhh", CMD_SYNC, CMD_VERSION, sequence & 0xFF, *values
    )
    return payload + struct.pack("<H", crc16_ccitt(payload))


def encode_control(command_id: int, sequence: int, payload: bytes = b"") -> bytes:
    if not 0 <= command_id <= 0xFF:
        raise ValueError("command ID is outside uint8 range")
    if len(payload) > 8:
        raise ValueError("control payload exceeds 8 bytes")
    body = struct.pack(
        "<2sBBBB8s", CMD_SYNC, CONTROL_VERSION, command_id,
        sequence & 0xFF, len(payload), payload.ljust(8, b"\0"),
    )
    return body + struct.pack("<H", crc16_ccitt(body))


@dataclass(frozen=True)
class Ack:
    sequence: int
    status: int
    chassis_state: int
    reject_reason: int
    fault_flags: int
    steering_flags: int
    can_flags: int
    uart_valid_count: int
    can_tx_count: int
    can_rx_count: int
    stm32_tick: int
    steering_position_mrad: tuple[int, int, int, int] = (
        ACK_POSITION_INVALID,
        ACK_POSITION_INVALID,
        ACK_POSITION_INVALID,
        ACK_POSITION_INVALID,
    )
    control_id: int = 0
    detail_type: int = 0
    detail_values: tuple[int, int, int, int] = (0, 0, 0, 0)
    detail_valid_mask: int = 0


def encode_ack(ack: Ack) -> bytes:
    payload = struct.pack(
        "<2sBBBBBBHBBHHHI",
        ACK_SYNC,
        ACK_VERSION,
        ACK_FRAME_SIZE,
        ack.sequence,
        ack.status,
        ack.chassis_state,
        ack.reject_reason,
        ack.fault_flags,
        ack.steering_flags,
        ack.can_flags,
        ack.uart_valid_count,
        ack.can_tx_count,
        ack.can_rx_count,
        ack.stm32_tick,
    ) + struct.pack(
        "<hhhhBBiiiiH", *ack.steering_position_mrad,
        ack.control_id, ack.detail_type, *ack.detail_values,
        ack.detail_valid_mask,
    )
    return payload + struct.pack("<H", crc16_ccitt(payload))


def decode_ack(frame: bytes) -> Ack:
    if len(frame) != ACK_FRAME_SIZE:
        raise ValueError(f"ACK length is {len(frame)}, expected {ACK_FRAME_SIZE}")
    if frame[:2] != ACK_SYNC:
        raise ValueError("bad ACK header")
    if frame[2] != ACK_VERSION:
        raise ValueError("unsupported ACK version")
    if frame[3] != ACK_FRAME_SIZE:
        raise ValueError("bad ACK length field")
    expected = struct.unpack_from("<H", frame, 50)[0]
    if crc16_ccitt(frame[:50]) != expected:
        raise ValueError("bad ACK CRC")
    values = struct.unpack_from("<2sBBBBBBHBBHHHI", frame)
    return Ack(
        sequence=values[3],
        status=values[4],
        chassis_state=values[5],
        reject_reason=values[6],
        fault_flags=values[7],
        steering_flags=values[8],
        can_flags=values[9],
        uart_valid_count=values[10],
        can_tx_count=values[11],
        can_rx_count=values[12],
        stm32_tick=values[13],
        steering_position_mrad=struct.unpack_from("<hhhh", frame, 22),
        control_id=frame[30],
        detail_type=frame[31],
        detail_values=struct.unpack_from("<iiii", frame, 32),
        detail_valid_mask=struct.unpack_from("<H", frame, 48)[0],
    )


class AckStreamParser:
    def __init__(self) -> None:
        self._buffer = bytearray()
        self.crc_errors = 0
        self.resync_count = 0

    def feed(self, data: bytes) -> list[Ack]:
        self._buffer.extend(data)
        output = []
        while True:
            start = self._buffer.find(ACK_SYNC)
            if start < 0:
                if self._buffer[-1:] == ACK_SYNC[:1]:
                    del self._buffer[:-1]
                else:
                    self._buffer.clear()
                return output
            if start:
                del self._buffer[:start]
                self.resync_count += 1
            if len(self._buffer) < ACK_FRAME_SIZE:
                return output
            candidate = bytes(self._buffer[:ACK_FRAME_SIZE])
            try:
                output.append(decode_ack(candidate))
                del self._buffer[:ACK_FRAME_SIZE]
            except ValueError as error:
                if "CRC" in str(error):
                    self.crc_errors += 1
                del self._buffer[0]
                self.resync_count += 1


class SequenceCounter:
    def __init__(self, initial: int = 0) -> None:
        self.value = initial & 0xFF

    def take(self) -> int:
        value = self.value
        self.value = (self.value + 1) & 0xFF
        return value
