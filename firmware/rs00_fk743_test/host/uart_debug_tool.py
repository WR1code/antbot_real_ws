#!/usr/bin/env python3
"""Standalone UART sender and binary ACK monitor for chassis bring-up."""

from __future__ import annotations

import argparse
from datetime import datetime
import math
import sys
import time

from chassis_uart_protocol import (
    ACK_POSITION_INVALID,
    ACK_STATUS_NAMES,
    CHASSIS_STATE_NAMES,
    REJECT_REASON_NAMES,
    AckStreamParser,
    SequenceCounter,
    encode_cmd_vel,
    limit_linear_velocity,
)
from serial_port import resolve_uart_port


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--vx", type=float, default=0.0)
    parser.add_argument("--vy", type=float, default=0.0)
    parser.add_argument("--wz", type=float, default=0.0)
    parser.add_argument("--max-linear-speed", type=float, default=0.5)
    parser.add_argument("--rate", type=float, default=5.0)
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--send-once", action="store_true")
    parser.add_argument("--stop", action="store_true")
    parser.add_argument("--listen-only", action="store_true")
    parser.add_argument("--print-hex", action="store_true")
    parser.add_argument("--wait-ack", action="store_true")
    parser.add_argument("--ack-timeout-ms", type=int, default=200)
    parser.add_argument("--print-frame-only", action="store_true")
    args = parser.parse_args()
    try:
        args.port = resolve_uart_port(args.port)
    except RuntimeError as error:
        parser.error(str(error))
    return args


def print_ack(ack, latency_ms: float | None) -> None:
    steering = (
        f"enabled={bool(ack.steering_flags & 1)} "
        f"homed={bool(ack.steering_flags & 2)} "
        f"ready={bool(ack.steering_flags & 4)} "
        f"fault={bool(ack.steering_flags & 8)}"
    )
    can = (
        f"tx_ok={bool(ack.can_flags & 1)} "
        f"feedback={bool(ack.can_flags & 2)} "
        f"bus_off={bool(ack.can_flags & 4)} "
        f"passive={bool(ack.can_flags & 8)}"
    )
    latency = "n/a" if latency_ms is None else f"{latency_ms:.1f} ms"
    positions = []
    for name, value in zip(("FL", "FR", "RL", "RR"),
                           ack.steering_position_mrad):
        if value == ACK_POSITION_INVALID:
            positions.append(f"{name}=n/a")
        else:
            radians = value / 1000.0
            positions.append(
                f"{name}={radians:.3f}rad/{math.degrees(radians):.1f}deg"
            )
    print(
        f"RX seq={ack.sequence} "
        f"status={ACK_STATUS_NAMES.get(ack.status, ack.status)} "
        f"state={CHASSIS_STATE_NAMES.get(ack.chassis_state, ack.chassis_state)} "
        f"reject={REJECT_REASON_NAMES.get(ack.reject_reason, ack.reject_reason)} "
        f"fault=0x{ack.fault_flags:04X} {steering} {can} "
        f"valid={ack.uart_valid_count} can_tx={ack.can_tx_count} "
        f"can_rx={ack.can_rx_count} positions=[{' '.join(positions)}] "
        f"crc=OK rtt={latency}"
    )


def main() -> int:
    args = arguments()
    vx, vy, wz = (0.0, 0.0, 0.0) if args.stop else (args.vx, args.vy, args.wz)
    vx, vy = limit_linear_velocity(vx, vy, args.max_linear_speed)
    preview = encode_cmd_vel(0, vx, vy, wz)
    if args.print_frame_only:
        print(preview.hex(" ").upper())
        return 0

    try:
        import serial
    except ImportError:
        print("pyserial is required: python3 -m pip install pyserial", file=sys.stderr)
        return 2

    sequence = SequenceCounter()
    ack_parser = AckStreamParser()
    sent_at: dict[int, float] = {}
    send_count = 0
    started = time.monotonic()
    next_send = started
    period = 1.0 / args.rate if args.rate > 0.0 else 0.0

    try:
        port = serial.Serial(
            port=args.port,
            baudrate=args.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.02,
            write_timeout=0.1,
        )
    except (serial.SerialException, OSError) as error:
        print(
            f"cannot open {args.port}: {error}\n"
            "The same serial port cannot be shared with the ROS bridge, "
            "VSCode Serial Monitor, or another terminal.",
            file=sys.stderr,
        )
        return 2

    try:
        while True:
            now = time.monotonic()
            should_send = (
                not args.listen_only
                and now >= next_send
                and (args.send_once is False or send_count == 0)
                and (args.send_once or now - started < args.duration)
            )
            if should_send:
                seq = sequence.take()
                frame = encode_cmd_vel(seq, vx, vy, wz)
                port.write(frame)
                sent_at[seq] = time.monotonic()
                send_count += 1
                stamp = datetime.now().isoformat(timespec="milliseconds")
                line = (
                    f"TX seq={seq} vx={vx:.3f} vy={vy:.3f} wz={wz:.3f} "
                    f"time={stamp} count={send_count}"
                )
                if args.print_hex:
                    line += f" hex={frame.hex(' ').upper()}"
                print(line)
                next_send = now + period

            data = port.read(port.in_waiting or 1)
            for ack in ack_parser.feed(data):
                timestamp = sent_at.pop(ack.sequence, None)
                latency = (
                    None
                    if timestamp is None
                    else (time.monotonic() - timestamp) * 1000.0
                )
                print_ack(ack, latency)
                if args.wait_ack and args.send_once:
                    return 0

            if args.send_once and send_count and not args.wait_ack:
                return 0
            if args.wait_ack and args.send_once and sent_at:
                oldest = min(sent_at.values())
                if (now - oldest) * 1000.0 >= args.ack_timeout_ms:
                    print("ACK timeout", file=sys.stderr)
                    return 1
            if (
                not args.listen_only
                and not args.send_once
                and now - started >= args.duration
            ):
                return 0
    except KeyboardInterrupt:
        if not args.listen_only:
            seq = sequence.take()
            port.write(encode_cmd_vel(seq, 0.0, 0.0, 0.0))
            print(f"\nCtrl+C: safe stop sent, seq={seq}")
        return 130
    finally:
        port.close()


if __name__ == "__main__":
    raise SystemExit(main())
