#!/usr/bin/env python3
"""Send explicit steering, drive, safety, configuration and query commands."""

from __future__ import annotations

import argparse
import math
import struct
import sys
import time

from chassis_uart_protocol import (
    ACK_POSITION_INVALID,
    ACK_STATUS_NAMES,
    CHASSIS_STATE_NAMES,
    CONTROL_IDS,
    REJECT_REASON_NAMES,
    AckStreamParser,
    encode_control,
)
from serial_port import resolve_uart_port

WHEELS = ("FL", "FR", "RL", "RR")
STEERING_STATE_NAMES = (
    "IDLE", "BOOT_WAIT", "STOP_SEND", "STOP_WAIT", "QUERY_UID_SEND",
    "QUERY_UID_WAIT", "READ_POSITION_SEND", "READ_POSITION_WAIT",
    "VALIDATE_POSITION", "SET_MODE_SEND", "SET_MODE_WAIT",
    "VERIFY_MODE_SEND", "VERIFY_MODE_WAIT", "SET_LIMIT_SPEED_SEND",
    "SET_LIMIT_SPEED_WAIT", "VERIFY_LIMIT_SPEED_SEND",
    "VERIFY_LIMIT_SPEED_WAIT", "SET_LIMIT_CURRENT_SEND",
    "SET_LIMIT_CURRENT_WAIT", "VERIFY_LIMIT_CURRENT_SEND",
    "VERIFY_LIMIT_CURRENT_WAIT", "SET_TIMEOUT_SEND", "SET_TIMEOUT_WAIT",
    "VERIFY_TIMEOUT_SEND", "VERIFY_TIMEOUT_WAIT", "PRELOAD_POSITION_SEND",
    "PRELOAD_POSITION_WAIT", "VERIFY_POSITION_SEND",
    "VERIFY_POSITION_WAIT", "ARMED", "ENABLE_SEND", "ENABLE_WAIT",
    "VERIFY_ALL", "READY", "FAULT",
)
STEERING_ERROR_NAMES = (
    "NONE", "ARGUMENT", "FDCAN_START", "TX_FIFO_FULL", "HAL_TX",
    "TIMEOUT", "RESPONSE_TYPE", "RESPONSE_MOTOR", "PARAM_INDEX",
    "PARAM_READ", "INVALID_UID", "UID_MISMATCH", "INVALID_POSITION",
    "MECHANICAL_RANGE", "PARAM_VERIFY", "MOTOR_FAULT", "MODE",
    "FEEDBACK_STALE", "TARGET_RANGE", "TARGET_STEP",
    "OVERTEMPERATURE",
)

TRANSLATION_STATE_NAMES = (
    "IDLE", "STOPPING_DRIVE", "STEERING", "WAIT_ALIGNMENT", "DRIVING",
    "TIMEOUT_STOP", "FAULT",
)
TRANSLATION_ERROR_NAMES = (
    "NONE", "DRIVE", "STEERING", "ALIGNMENT_TIMEOUT",
)


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=1.0)
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("status")
    sub.add_parser("steering-enable")
    all_angle = sub.add_parser("steering-set-all")
    all_angle.add_argument("angle_deg", type=float)
    each = sub.add_parser("steering-set-each")
    each.add_argument("fl_deg", type=float)
    each.add_argument("fr_deg", type=float)
    each.add_argument("rl_deg", type=float)
    each.add_argument("rr_deg", type=float)
    sub.add_parser("steering-disable")
    sub.add_parser("rs00-clear-fault")
    uid = sub.add_parser("rs00-query-uid")
    uid.add_argument("wheel", choices=("fl", "fr", "rl", "rr", "all"))
    sub.add_parser("rs00-read-position")
    sub.add_parser("rs00-feedback-age")
    sub.add_parser("rs00-set-csp")
    limits = sub.add_parser("rs00-set-limits")
    limits.add_argument("speed_rad_s", type=float)
    limits.add_argument("current_a", type=float)
    accel = sub.add_parser("drive-set-acceleration")
    accel.add_argument("erpm_per_s", type=int)
    decel = sub.add_parser("drive-set-deceleration")
    decel.add_argument("erpm_per_s", type=int)
    drive = sub.add_parser("drive-set-all")
    drive.add_argument("fl_mps", type=float)
    drive.add_argument("fr_mps", type=float)
    drive.add_argument("rl_mps", type=float)
    drive.add_argument("rr_mps", type=float)
    sub.add_parser("drive-stop")
    sub.add_parser("drive-withdraw-current")
    feedback = sub.add_parser("drive-feedback")
    feedback.add_argument(
        "kind", choices=("all", "valid", "fault", "speed", "current",
                         "position", "temperature", "age", "safety", "voltage"),
        default="all", nargs="?"
    )
    sub.add_parser("emergency-stop")
    sub.add_parser("clear-fault-restart")
    sub.add_parser("system-reset")
    sub.add_parser("system-health")
    event_log = sub.add_parser("event-log")
    event_log.add_argument("count", type=int, nargs="?", default=8)
    args = parser.parse_args()
    try:
        args.port = resolve_uart_port(args.port)
    except RuntimeError as error:
        parser.error(str(error))
    return args


def angle_payload(values: list[float]) -> bytes:
    encoded = []
    for value in values:
        if not math.isfinite(value) or not 0.0 <= value <= 180.0:
            raise ValueError("steering angles must be within 0..180 degrees")
        encoded.append(round(math.radians(value) * 1000.0))
    return struct.pack("<" + "h" * len(encoded), *encoded)


def speed_payload(values: list[float]) -> bytes:
    encoded = []
    for value in values:
        if not math.isfinite(value) or abs(value) > 0.5:
            raise ValueError("wheel speeds must be within -0.5..0.5 m/s")
        encoded.append(round(value * 1000.0))
    return struct.pack("<hhhh", *encoded)


def show_ack(ack) -> None:
    steering = (
        f"enabled={'yes' if ack.steering_flags & 0x01 else 'no'} "
        f"homed={'yes' if ack.steering_flags & 0x02 else 'no'} "
        f"ready={'yes' if ack.steering_flags & 0x04 else 'no'} "
        f"fault={'yes' if ack.steering_flags & 0x08 else 'no'}"
    )
    positions = []
    for name, value in zip(WHEELS, ack.steering_position_mrad):
        positions.append(
            f"{name}=n/a" if value == ACK_POSITION_INVALID
            else f"{name}={math.degrees(value / 1000.0):.1f}deg"
        )
    print(
        f"ACK seq={ack.sequence} control=0x{ack.control_id:02X} "
        f"status={ACK_STATUS_NAMES.get(ack.status, ack.status)} "
        f"state={CHASSIS_STATE_NAMES.get(ack.chassis_state, ack.chassis_state)} "
        f"reject={REJECT_REASON_NAMES.get(ack.reject_reason, ack.reject_reason)} "
        f"fault=0x{ack.fault_flags:04X} steering=[{steering}] "
        f"positions=[{' '.join(positions)}]"
    )
    if ack.detail_type == 1:
        raw = struct.pack("<ii", ack.detail_values[0], ack.detail_values[1])
        print(f"UID wheel={WHEELS[ack.detail_values[2]]} value={raw.hex().upper()}")
    elif ack.detail_type == 8:
        state, error, fault_context, packed = ack.detail_values
        context = fault_context & 0xFFFFFFFF
        motor_id = context & 0xFF
        translation_state = (context >> 8) & 0xFF
        translation_error = (context >> 16) & 0xFF
        translation_wheel = (context >> 24) & 0xFF
        motor_status = []
        for index, name in enumerate(WHEELS):
            flags = (packed >> (index * 8)) & 0xFF
            motor_status.append(
                f"{name}:mode={flags & 0x03},init={bool(flags & 0x04)},"
                f"enabled={bool(flags & 0x08)},online={bool(flags & 0x10)},"
                f"fault={bool(flags & 0x20)}"
            )
        state_name = (
            STEERING_STATE_NAMES[state]
            if 0 <= state < len(STEERING_STATE_NAMES) else str(state)
        )
        error_name = (
            STEERING_ERROR_NAMES[error]
            if 0 <= error < len(STEERING_ERROR_NAMES) else str(error)
        )
        print(
            f"steering_debug state={state_name} error={error_name} "
            f"fault_motor=0x{motor_id:02X} "
            f"motors=[{' '.join(motor_status)}]"
        )
        if context > 0xFF:
            translation_state_name = (
                TRANSLATION_STATE_NAMES[translation_state]
                if translation_state < len(TRANSLATION_STATE_NAMES)
                else str(translation_state)
            )
            translation_error_name = (
                TRANSLATION_ERROR_NAMES[translation_error]
                if translation_error < len(TRANSLATION_ERROR_NAMES)
                else str(translation_error)
            )
            wheel_name = (
                WHEELS[translation_wheel]
                if translation_wheel < len(WHEELS) else str(translation_wheel)
            )
            print(
                f"translation_debug state={translation_state_name} "
                f"error={translation_error_name} fault_wheel={wheel_name}"
            )
    elif ack.detail_type == 10:
        boot, reset_flags, watchdogs, crashes = (
            value & 0xFFFFFFFF for value in ack.detail_values
        )
        print(
            f"system_boot boot_count={boot} reset_flags=0x{reset_flags:08X} "
            f"watchdog_resets={watchdogs} cpu_faults={crashes}"
        )
    elif ack.detail_type == 11:
        last_us, max_us, average_us, stack_bytes = (
            value & 0xFFFFFFFF for value in ack.detail_values
        )
        print(
            f"runtime loop_us(last/max/avg)={last_us}/{max_us}/{average_us} "
            f"stack_min_free={stack_bytes}B"
        )
    elif ack.detail_type == 12:
        version, git_hash, config_hash, capabilities = (
            value & 0xFFFFFFFF for value in ack.detail_values
        )
        print(
            f"firmware version={(version >> 16) & 0xFF}."
            f"{(version >> 8) & 0xFF}.{version & 0xFF} "
            f"git={git_hash:08x} config={config_hash:08x} "
            f"capabilities=0x{capabilities:08X}"
        )
    elif ack.detail_type == 13:
        values = " ".join(f"0x{value & 0xFFFFFFFF:08X}"
                          for value in ack.detail_values)
        print(f"crash_registers={values} valid=0x{ack.detail_valid_mask:04X}")
    elif ack.detail_type == 14:
        values = " ".join(f"0x{value & 0xFFFFFFFF:08X}"
                          for value in ack.detail_values)
        print(f"crash_faults={values} valid=0x{ack.detail_valid_mask:04X}")
    elif ack.detail_type == 15:
        if ack.detail_valid_mask == 0x0F:
            tick, code, detail, total = (
                value & 0xFFFFFFFF for value in ack.detail_values
            )
            print(
                f"event tick={tick} code={code} detail=0x{detail:08X} "
                f"total={total}"
            )
        else:
            print("event: no entry")
    elif ack.detail_type == 16:
        low, threshold, external_mv, temperature_mc = ack.detail_values
        print(
            f"power vdd={'LOW' if low else 'OK'} threshold={threshold}mV "
            f"external={external_mv if ack.detail_valid_mask & 4 else 'n/a'}mV "
            f"mcu_temperature="
            f"{temperature_mc / 1000 if ack.detail_valid_mask & 8 else 'n/a'}C"
        )
    elif ack.detail_type != 0:
        labels = {
            2: "speed_erpm",
            3: "current_10mA",
            4: "position_0.01deg",
            5: "temperature_C",
            6: "fault_code",
            7: "valid_mask",
            9: "steering_feedback_age_ms",
            17: "drive_feedback_age_ms",
            18: "drive_safety_flags",
            19: "drive_voltage_V",
        }
        values = " ".join(
            f"{name}={value}" for name, value in zip(WHEELS, ack.detail_values)
        )
        print(f"{labels.get(ack.detail_type, ack.detail_type)}: {values}")


def transact(port, parser, sequence: int, command_id: int, payload: bytes,
             timeout: float):
    frame = encode_control(command_id, sequence, payload)
    port.write(frame)
    port.flush()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        data = port.read(port.in_waiting or 1)
        for ack in parser.feed(data):
            if ack.sequence == sequence and ack.control_id == command_id:
                show_ack(ack)
                return ack
    raise TimeoutError(f"ACK timeout for control 0x{command_id:02X}")


def command_requests(args: argparse.Namespace) -> list[tuple[int, bytes]]:
    cid = CONTROL_IDS
    name = args.command
    if name == "status":
        return [(cid["QUERY_STATUS"], b"")]
    if name == "steering-enable":
        return [(cid["STEERING_ENABLE"], b"")]
    if name == "steering-set-all":
        return [(cid["STEERING_SET_ALL"], angle_payload([args.angle_deg]))]
    if name == "steering-set-each":
        return [(cid["STEERING_SET_EACH"], angle_payload(
            [args.fl_deg, args.fr_deg, args.rl_deg, args.rr_deg]))]
    if name == "steering-disable":
        return [(cid["STEERING_DISABLE"], b"")]
    if name == "rs00-clear-fault":
        return [(cid["RS00_CLEAR_FAULT"], b"")]
    if name == "rs00-query-uid":
        wheels = range(4) if args.wheel == "all" else [WHEELS.index(args.wheel.upper())]
        return [(cid["RS00_QUERY_UID"], bytes([wheel])) for wheel in wheels]
    if name == "rs00-read-position":
        return [(cid["RS00_READ_POSITION"], b"")]
    if name == "rs00-feedback-age":
        return [(cid["RS00_QUERY_FEEDBACK_AGE"], b"")]
    if name == "rs00-set-csp":
        return [(cid["RS00_SET_CSP"], b"")]
    if name == "rs00-set-limits":
        speed = round(args.speed_rad_s * 1000.0)
        current = round(args.current_a * 1000.0)
        if not 1 <= speed <= 1000 or not 1 <= current <= 2000:
            raise ValueError(
                "safe limits are speed 0..1 rad/s and current 0..2 A"
            )
        return [(cid["RS00_SET_LIMITS"], struct.pack("<HH", speed, current))]
    if name == "drive-set-acceleration":
        if not 1 <= args.erpm_per_s <= 1000:
            raise ValueError("safe acceleration range is 1..1000 erpm/s")
        return [(cid["DRIVE_SET_ACCELERATION"], struct.pack("<i", args.erpm_per_s))]
    if name == "drive-set-deceleration":
        if not 1 <= args.erpm_per_s <= 1500:
            raise ValueError("safe deceleration range is 1..1500 erpm/s")
        return [(cid["DRIVE_SET_DECELERATION"], struct.pack("<i", args.erpm_per_s))]
    if name == "drive-set-all":
        return [(cid["DRIVE_SET_ALL"], speed_payload(
            [args.fl_mps, args.fr_mps, args.rl_mps, args.rr_mps]))]
    if name == "drive-stop":
        return [(cid["DRIVE_STOP"], b"")]
    if name == "drive-withdraw-current":
        return [(cid["DRIVE_WITHDRAW_CURRENT"], b"")]
    if name == "drive-feedback":
        selectors = {"valid": 0, "fault": 1, "speed": 2, "current": 3,
                     "position": 4, "temperature": 5, "age": 6,
                     "safety": 7, "voltage": 8}
        kinds = selectors if args.kind == "all" else {args.kind: selectors[args.kind]}
        return [(cid["DRIVE_QUERY_FEEDBACK"], bytes([selector]))
                for selector in kinds.values()]
    if name == "emergency-stop":
        return [(cid["EMERGENCY_STOP"], b"")]
    if name == "clear-fault-restart":
        return [(cid["CLEAR_FAULT_RESTART"], b"")]
    if name == "system-reset":
        return [(cid["SYSTEM_RESET"], b"RST!")]
    if name == "system-health":
        return [
            (cid["QUERY_SYSTEM_BOOT"], b""),
            (cid["QUERY_SYSTEM_RUNTIME"], b""),
            (cid["QUERY_FIRMWARE"], b""),
            (cid["QUERY_POWER"], b""),
            (cid["QUERY_CRASH_REGISTERS"], b"\x01"),
            (cid["QUERY_CRASH_FAULTS"], b"\x00"),
        ]
    if name == "event-log":
        if not 1 <= args.count <= 32:
            raise ValueError("event-log count must be within 1..32")
        return [(cid["QUERY_EVENT_LOG"], bytes([index]))
                for index in range(args.count)]
    raise ValueError(f"unsupported command {name}")


def main() -> int:
    args = arguments()
    try:
        import serial
        requests = command_requests(args)
        port = serial.Serial(args.port, args.baud, timeout=0.02, write_timeout=0.1)
        parser = AckStreamParser()
        port.reset_input_buffer()
        try:
            for sequence, (command_id, payload) in enumerate(requests):
                transact(port, parser, sequence, command_id, payload, args.timeout)
        finally:
            port.close()
        return 0
    except (ValueError, TimeoutError, OSError) as error:
        print(error, file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
