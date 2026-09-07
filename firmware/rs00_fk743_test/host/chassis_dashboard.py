#!/usr/bin/env python3
"""Read-only Tk dashboard for STM32 chassis telemetry."""

from __future__ import annotations

import argparse
import math
import queue
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

import serial

from chassis_uart_protocol import (
    ACK_POSITION_INVALID,
    ACK_STATUS_NAMES,
    CHASSIS_STATE_NAMES,
    CONTROL_IDS,
    REJECT_REASON_NAMES,
    Ack,
    AckStreamParser,
    encode_control,
)
from serial_port import resolve_uart_port

WHEELS = ("FL / 左前", "FR / 右前", "RL / 左后", "RR / 右后")

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

FAULT_NAMES = (
    (1 << 0, "转向"),
    (1 << 1, "行走"),
    (1 << 2, "CAN"),
    (1 << 3, "CAN Bus-Off"),
    (1 << 4, "UART"),
)

TELEMETRY_REQUESTS = (
    (CONTROL_IDS["QUERY_STATUS"], b""),
    (CONTROL_IDS["QUERY_SYSTEM_BOOT"], b""),
    (CONTROL_IDS["QUERY_SYSTEM_RUNTIME"], b""),
    (CONTROL_IDS["QUERY_FIRMWARE"], b""),
    (CONTROL_IDS["QUERY_CRASH_REGISTERS"], b"\x01"),
    (CONTROL_IDS["QUERY_CRASH_FAULTS"], b"\x00"),
    (CONTROL_IDS["QUERY_EVENT_LOG"], b"\x00"),
    (CONTROL_IDS["QUERY_POWER"], b""),
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
)

EVENT_NAMES = {
    1: "启动", 2: "看门狗复位", 3: "CPU异常", 4: "主机复位",
    5: "VDD欠压", 6: "VDD恢复", 7: "Error_Handler", 8: "物理急停",
    9: "MINI安全故障", 10: "RS00安全故障",
}

CAPABILITY_NAMES = (
    (1 << 0, "IWDG"), (1 << 1, "备份黑匣子"), (1 << 2, "异常捕获"),
    (1 << 3, "循环监控"), (1 << 4, "栈水位"), (1 << 5, "PVD 2.85V"),
    (1 << 6, "外部电压"), (1 << 7, "MCU温度"),
    (1 << 8, "外部看门狗"), (1 << 9, "安全升级"),
    (1 << 10, "物理急停"),
    (1 << 11, "转向已标定"), (1 << 12, "MINI反馈保护"),
)

DRIVE_SAFETY_NAMES = (
    (1 << 0, "缺反馈"), (1 << 1, "反馈过期"), (1 << 2, "驱动器故障"),
    (1 << 3, "过流"), (1 << 4, "过温"), (1 << 5, "电压异常"),
    (1 << 6, "超速"),
)


def unsigned32(value: int) -> int:
    return value & 0xFFFFFFFF


def version_string(packed: int) -> str:
    value = unsigned32(packed)
    return f"{(value >> 16) & 0xFF}.{(value >> 8) & 0xFF}.{value & 0xFF}"


def enum_name(names: tuple[str, ...], value: int) -> str:
    return names[value] if 0 <= value < len(names) else str(value)


def bit_names(value: int, definitions: tuple[tuple[int, str], ...]) -> str:
    names = [name for mask, name in definitions if value & mask]
    return ", ".join(names) if names else "无"


def format_uptime(milliseconds: int) -> str:
    seconds = milliseconds // 1000
    days, seconds = divmod(seconds, 86400)
    hours, seconds = divmod(seconds, 3600)
    minutes, seconds = divmod(seconds, 60)
    if days:
        return f"{days}天 {hours:02d}:{minutes:02d}:{seconds:02d}"
    return f"{hours:02d}:{minutes:02d}:{seconds:02d}"


class TelemetryStore:
    """Accumulate paged ACK details into one coherent dashboard snapshot."""

    def __init__(self) -> None:
        self.latest_ack: Ack | None = None
        self.details: dict[int, Ack] = {}
        self.last_update_monotonic = 0.0

    def apply(self, ack: Ack, now: float | None = None) -> None:
        self.latest_ack = ack
        if ack.detail_type:
            self.details[ack.detail_type] = ack
        self.last_update_monotonic = time.monotonic() if now is None else now

    def detail_values(self, detail_type: int, scale: float = 1.0) -> list[float | None]:
        ack = self.details.get(detail_type)
        if ack is None:
            return [None] * 4
        return [
            value * scale if ack.detail_valid_mask & (1 << index) else None
            for index, value in enumerate(ack.detail_values)
        ]

    def steering_rows(self) -> list[tuple[object, ...]]:
        ack = self.latest_ack
        ages = self.detail_values(9)
        status = self.details.get(8)
        packed_flags = status.detail_values[3] if status is not None else 0
        positions = (
            ack.steering_position_mrad if ack is not None
            else (ACK_POSITION_INVALID,) * 4
        )
        rows = []
        for index, wheel in enumerate(WHEELS):
            raw_position = positions[index]
            position = (
                None if raw_position == ACK_POSITION_INVALID
                else math.degrees(raw_position / 1000.0)
            )
            flags = (packed_flags >> (index * 8)) & 0xFF
            rows.append((
                wheel,
                position,
                ages[index],
                flags & 0x03,
                bool(flags & 0x04),
                bool(flags & 0x08),
                bool(flags & 0x10),
                bool(flags & 0x20),
            ))
        return rows

    def drive_rows(self) -> list[tuple[object, ...]]:
        speed = self.detail_values(2)
        current = self.detail_values(3, 0.01)
        position = self.detail_values(4, 0.01)
        temperature = self.detail_values(5)
        faults = self.detail_values(6)
        valid = self.detail_values(7)
        feedback_age = self.detail_values(17)
        safety = self.detail_values(18)
        voltage = self.detail_values(19)
        return [
            (WHEELS[index], speed[index], current[index], position[index],
             temperature[index], voltage[index], feedback_age[index],
             faults[index], safety[index], valid[index])
            for index in range(4)
        ]


class TelemetryPoller(threading.Thread):
    def __init__(self, port: str, baud: int, request_period: float,
                 events: queue.Queue[dict[str, object]]) -> None:
        super().__init__(daemon=True)
        self.port_name = port
        self.baud = baud
        self.request_period = request_period
        self.events = events
        self.stop_event = threading.Event()
        self.sequence = 0
        self.timeout_count = 0

    def stop(self) -> None:
        self.stop_event.set()

    def emit(self, kind: str, **values: object) -> None:
        self.events.put({"kind": kind, **values})

    def wait_for_ack(self, port: serial.Serial, parser: AckStreamParser,
                     sequence: int, command_id: int) -> Ack | None:
        deadline = time.monotonic() + 0.5
        while not self.stop_event.is_set() and time.monotonic() < deadline:
            data = port.read(port.in_waiting or 1)
            for ack in parser.feed(data):
                self.emit(
                    "ack", ack=ack, crc_errors=parser.crc_errors,
                    resync_count=parser.resync_count,
                    timeout_count=self.timeout_count,
                )
                if ack.sequence == sequence and ack.control_id == command_id:
                    return ack
        return None

    def poll_connected(self, port: serial.Serial) -> None:
        parser = AckStreamParser()
        request_index = 0
        port.reset_input_buffer()
        self.emit("connected", port=self.port_name)
        while not self.stop_event.is_set():
            command_id, payload = TELEMETRY_REQUESTS[request_index]
            sequence = self.sequence
            port.write(encode_control(command_id, sequence, payload))
            port.flush()
            self.sequence = (self.sequence + 1) & 0xFF
            if self.wait_for_ack(port, parser, sequence, command_id) is None:
                self.timeout_count += 1
                self.emit("timeout", count=self.timeout_count)
            request_index = (request_index + 1) % len(TELEMETRY_REQUESTS)
            self.stop_event.wait(self.request_period)

    def run(self) -> None:
        while not self.stop_event.is_set():
            port = None
            try:
                port = serial.Serial(
                    self.port_name, self.baud, timeout=0.05,
                    write_timeout=0.2,
                )
                self.poll_connected(port)
            except (serial.SerialException, OSError) as error:
                self.emit("error", message=str(error))
            finally:
                if port is not None and port.is_open:
                    port.close()
                self.emit("disconnected")
            self.stop_event.wait(1.0)


class ChassisDashboard:
    def __init__(self, root: tk.Tk, port: str, baud: int,
                 request_period: float, autoconnect: bool) -> None:
        self.root = root
        self.request_period = request_period
        self.events: queue.Queue[dict[str, object]] = queue.Queue()
        self.poller: TelemetryPoller | None = None
        self.store = TelemetryStore()
        self.connected = False
        self.crc_errors = 0
        self.resync_count = 0
        self.timeout_count = 0

        root.title("STM32H743 底盘状态监控")
        root.minsize(1080, 680)
        root.protocol("WM_DELETE_WINDOW", self.close)

        self.port_var = tk.StringVar(value=port)
        self.baud_var = tk.StringVar(value=str(baud))
        self.connection_var = tk.StringVar(value="未连接")
        self.last_error_var = tk.StringVar(value="")
        self.system_vars = {
            key: tk.StringVar(value="--") for key in (
                "ack", "state", "reject", "fault", "uptime", "uart",
                "can", "can_flags", "steering", "steering_error",
                "translation", "protocol", "updated", "boot", "reset",
                "watchdog", "crash", "firmware", "identity", "capabilities",
                "loop", "stack", "power", "last_event", "crash_pc", "cfsr",
                "motion_lock",
            )
        }

        self.build_ui()
        self.root.after(50, self.process_events)
        if autoconnect:
            self.root.after(150, self.connect)

    def build_ui(self) -> None:
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(3, weight=1)

        controls = ttk.Frame(self.root, padding=8)
        controls.grid(row=0, column=0, sticky="ew")
        controls.columnconfigure(1, weight=1)
        ttk.Label(controls, text="串口").grid(row=0, column=0, padx=(0, 6))
        ttk.Entry(controls, textvariable=self.port_var).grid(
            row=0, column=1, sticky="ew", padx=(0, 10)
        )
        ttk.Label(controls, text="波特率").grid(row=0, column=2, padx=(0, 6))
        ttk.Entry(controls, textvariable=self.baud_var, width=10).grid(
            row=0, column=3, padx=(0, 10)
        )
        self.connect_button = ttk.Button(
            controls, text="连接", command=self.toggle_connection
        )
        self.connect_button.grid(row=0, column=4, padx=(0, 10))
        ttk.Label(controls, textvariable=self.connection_var).grid(row=0, column=5)

        self.state_banner = tk.Label(
            self.root, text="等待 STM32 数据", font=("Sans", 18, "bold"),
            bg="#5f6368", fg="white", pady=9,
        )
        self.state_banner.grid(row=1, column=0, sticky="ew", padx=8)

        summary = ttk.LabelFrame(self.root, text="系统状态", padding=8)
        summary.grid(row=2, column=0, sticky="ew", padx=8, pady=8)
        labels = (
            ("ACK", "ack"), ("底盘状态", "state"), ("拒绝原因", "reject"),
            ("故障", "fault"), ("MCU 运行时间", "uptime"),
            ("UART", "uart"), ("CAN 计数", "can"), ("CAN 状态", "can_flags"),
            ("转向状态", "steering"), ("转向错误", "steering_error"),
            ("平移状态", "translation"), ("协议健康", "protocol"),
            ("最后更新", "updated"),
            ("启动次数", "boot"), ("复位标志", "reset"),
            ("看门狗复位", "watchdog"), ("CPU异常次数", "crash"),
            ("固件版本", "firmware"), ("构建身份", "identity"),
            ("可靠性能力", "capabilities"), ("主循环", "loop"),
            ("最小空闲栈", "stack"), ("供电监控", "power"),
            ("最近事件", "last_event"), ("异常 PC", "crash_pc"),
            ("CFSR", "cfsr"),
            ("整车运动锁", "motion_lock"),
        )
        for index, (label, key) in enumerate(labels):
            row, pair = divmod(index, 4)
            column = pair * 2
            ttk.Label(summary, text=f"{label}：").grid(
                row=row, column=column, sticky="e", padx=(2, 3), pady=2
            )
            ttk.Label(summary, textvariable=self.system_vars[key]).grid(
                row=row, column=column + 1, sticky="w", padx=(0, 16), pady=2
            )

        tables = ttk.Frame(self.root, padding=(8, 0, 8, 8))
        tables.grid(row=3, column=0, sticky="nsew")
        tables.columnconfigure(0, weight=1)
        tables.rowconfigure(0, weight=1)
        tables.rowconfigure(1, weight=1)

        self.steering_tree = self.make_tree(
            tables, "RS00 转向电机",
            ("wheel", "position", "age", "mode", "init", "enable", "online", "fault"),
            ("车轮", "角度 °", "反馈年龄 ms", "模式", "初始化", "使能", "在线", "故障"),
            row=0,
        )
        self.drive_tree = self.make_tree(
            tables, "MINI 行走电机",
            ("wheel", "speed", "current", "position", "temperature", "voltage",
             "age", "fault", "safety", "valid"),
            ("车轮", "转速 erpm", "电流 A", "位置 °", "温度 °C", "电压 V",
             "反馈年龄 ms", "故障码", "软件保护", "有效位"),
            row=1,
        )

        error = ttk.Label(
            self.root, textvariable=self.last_error_var, foreground="#b3261e",
            padding=(8, 0, 8, 8),
        )
        error.grid(row=4, column=0, sticky="ew")

    @staticmethod
    def make_tree(parent: ttk.Frame, title: str, columns: tuple[str, ...],
                  headings: tuple[str, ...], row: int) -> ttk.Treeview:
        frame = ttk.LabelFrame(parent, text=title, padding=6)
        frame.grid(row=row, column=0, sticky="nsew", pady=(0, 8) if row == 0 else 0)
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(0, weight=1)
        tree = ttk.Treeview(frame, columns=columns, show="headings", height=5)
        for column, heading in zip(columns, headings):
            tree.heading(column, text=heading)
            tree.column(column, anchor="center", width=120)
        tree.column(columns[0], width=140)
        tree.grid(row=0, column=0, sticky="nsew")
        return tree

    def toggle_connection(self) -> None:
        if self.poller is None:
            self.connect()
        else:
            self.disconnect()

    def connect(self) -> None:
        if self.poller is not None:
            return
        port = self.port_var.get().strip()
        try:
            baud = int(self.baud_var.get())
            if not port or baud <= 0:
                raise ValueError
        except ValueError:
            messagebox.showerror("参数错误", "请输入有效串口路径和波特率")
            return
        self.last_error_var.set("")
        self.connection_var.set("正在连接…")
        self.connect_button.configure(text="断开")
        self.poller = TelemetryPoller(
            port, baud, self.request_period, self.events
        )
        self.poller.start()

    def disconnect(self) -> None:
        if self.poller is not None:
            self.poller.stop()
            self.poller = None
        self.connected = False
        self.connection_var.set("已断开")
        self.connect_button.configure(text="连接")

    def close(self) -> None:
        self.disconnect()
        self.root.destroy()

    def process_events(self) -> None:
        try:
            while True:
                event = self.events.get_nowait()
                kind = event["kind"]
                if kind == "connected":
                    self.connected = True
                    self.connection_var.set(f"已连接：{event['port']}")
                    self.last_error_var.set("")
                elif kind == "disconnected":
                    self.connected = False
                    if self.poller is not None:
                        self.connection_var.set("连接中断，正在重试…")
                elif kind == "error":
                    self.last_error_var.set(f"串口错误：{event['message']}")
                elif kind == "timeout":
                    self.timeout_count = int(event["count"])
                    self.last_error_var.set("等待 STM32 ACK 超时；继续轮询")
                elif kind == "ack":
                    ack = event["ack"]
                    if isinstance(ack, Ack):
                        self.store.apply(ack)
                    self.crc_errors = int(event["crc_errors"])
                    self.resync_count = int(event["resync_count"])
                    self.timeout_count = int(event["timeout_count"])
                    self.last_error_var.set("")
        except queue.Empty:
            pass
        self.refresh_display()
        self.root.after(50, self.process_events)

    @staticmethod
    def value(value: float | None, digits: int = 1) -> str:
        return "--" if value is None else f"{value:.{digits}f}"

    def refresh_display(self) -> None:
        ack = self.store.latest_ack
        if ack is None:
            self.state_banner.configure(
                text="等待 STM32 数据", bg="#5f6368"
            )
            return

        age = time.monotonic() - self.store.last_update_monotonic
        state = CHASSIS_STATE_NAMES.get(ack.chassis_state, str(ack.chassis_state))
        healthy = ack.fault_flags == 0 and age < 2.0
        identity_page = self.store.details.get(12)
        calibration_locked = (
            identity_page is not None
            and not (unsigned32(identity_page.detail_values[3]) & (1 << 11))
        )
        if age >= 2.0:
            banner, color = f"遥测已过期 {age:.1f} 秒", "#b26a00"
        elif ack.fault_flags:
            banner, color = f"故障：{bit_names(ack.fault_flags, FAULT_NAMES)}", "#b3261e"
        elif calibration_locked:
            banner, color = "调试模式：转向未标定，整车运动已锁定", "#b26a00"
        elif healthy:
            banner, color = f"运行正常 · {state}", "#188038"
        else:
            banner, color = state, "#5f6368"
        self.state_banner.configure(text=banner, bg=color)

        self.system_vars["ack"].set(ACK_STATUS_NAMES.get(ack.status, str(ack.status)))
        self.system_vars["state"].set(state)
        self.system_vars["reject"].set(
            REJECT_REASON_NAMES.get(ack.reject_reason, str(ack.reject_reason))
        )
        self.system_vars["fault"].set(
            f"0x{ack.fault_flags:04X} · {bit_names(ack.fault_flags, FAULT_NAMES)}"
        )
        self.system_vars["uptime"].set(format_uptime(ack.stm32_tick))
        self.system_vars["uart"].set(f"有效帧 {ack.uart_valid_count}")
        self.system_vars["can"].set(
            f"TX {ack.can_tx_count} / RX {ack.can_rx_count}"
        )
        can_parts = (
            f"TX={'正常' if ack.can_flags & 0x01 else '无确认'}",
            f"RX={'新鲜' if ack.can_flags & 0x02 else '过期'}",
            f"Bus-Off={'是' if ack.can_flags & 0x04 else '否'}",
            f"Passive={'是' if ack.can_flags & 0x08 else '否'}",
        )
        self.system_vars["can_flags"].set(" / ".join(can_parts))

        status = self.store.details.get(8)
        if status is not None:
            steering_state, steering_error, context, _ = status.detail_values
            self.system_vars["steering"].set(
                enum_name(STEERING_STATE_NAMES, steering_state)
            )
            self.system_vars["steering_error"].set(
                enum_name(STEERING_ERROR_NAMES, steering_error)
            )
            unsigned_context = context & 0xFFFFFFFF
            translation_state = (unsigned_context >> 8) & 0xFF
            translation_error = (unsigned_context >> 16) & 0xFF
            self.system_vars["translation"].set(
                f"{enum_name(TRANSLATION_STATE_NAMES, translation_state)} / "
                f"{enum_name(TRANSLATION_ERROR_NAMES, translation_error)}"
            )
        self.system_vars["protocol"].set(
            f"CRC错误 {self.crc_errors} / 重同步 {self.resync_count} / 超时 {self.timeout_count}"
        )
        self.system_vars["updated"].set(f"{age:.1f} 秒前")

        boot = self.store.details.get(10)
        if boot is not None:
            boots, reset_flags, watchdogs, crashes = boot.detail_values
            self.system_vars["boot"].set(str(unsigned32(boots)))
            self.system_vars["reset"].set(f"0x{unsigned32(reset_flags):08X}")
            self.system_vars["watchdog"].set(str(unsigned32(watchdogs)))
            self.system_vars["crash"].set(str(unsigned32(crashes)))
        runtime = self.store.details.get(11)
        if runtime is not None:
            last_us, max_us, average_us, stack_bytes = runtime.detail_values
            self.system_vars["loop"].set(
                f"本次 {unsigned32(last_us)} / 平均 {unsigned32(average_us)} / "
                f"最大 {unsigned32(max_us)} µs"
            )
            self.system_vars["stack"].set(f"{unsigned32(stack_bytes)} B")
        identity = self.store.details.get(12)
        if identity is not None:
            version, git_hash, config_hash, capabilities = identity.detail_values
            self.system_vars["firmware"].set(version_string(version))
            self.system_vars["identity"].set(
                f"git {unsigned32(git_hash):08x} / cfg {unsigned32(config_hash):08x}"
            )
            self.system_vars["capabilities"].set(
                bit_names(unsigned32(capabilities), CAPABILITY_NAMES)
            )
            self.system_vars["motion_lock"].set(
                "已解锁" if unsigned32(capabilities) & (1 << 11)
                else "未标定：禁止整车运动"
            )
        crash_regs = self.store.details.get(13)
        if crash_regs is not None and crash_regs.detail_valid_mask & 0x04:
            self.system_vars["crash_pc"].set(
                f"0x{unsigned32(crash_regs.detail_values[2]):08X}"
            )
        crash_faults = self.store.details.get(14)
        if crash_faults is not None and crash_faults.detail_valid_mask & 0x01:
            self.system_vars["cfsr"].set(
                f"0x{unsigned32(crash_faults.detail_values[0]):08X}"
            )
        event = self.store.details.get(15)
        if event is not None and event.detail_valid_mask == 0x0F:
            tick, code, detail, count = event.detail_values
            self.system_vars["last_event"].set(
                f"#{unsigned32(count)} {EVENT_NAMES.get(code, str(code))} "
                f"@{unsigned32(tick)} ms，详情 0x{unsigned32(detail):08X}"
            )
        power = self.store.details.get(16)
        if power is not None:
            low, threshold, _, _ = power.detail_values
            self.system_vars["power"].set(
                f"{'欠压' if low else '正常'}（PVD {unsigned32(threshold)} mV）"
            )

        steering_values = []
        for row in self.store.steering_rows():
            wheel, position, feedback_age, mode, initialized, enabled, online, fault = row
            steering_values.append((
                wheel, self.value(position), self.value(feedback_age, 0), mode,
                "是" if initialized else "否", "是" if enabled else "否",
                "是" if online else "否", "是" if fault else "否",
            ))
        self.replace_rows(self.steering_tree, steering_values)

        drive_values = []
        for row in self.store.drive_rows():
            (wheel, speed, current, position, temperature, voltage,
             feedback_age, fault, safety, valid) = row
            drive_values.append((
                wheel, self.value(speed, 0), self.value(current, 2),
                self.value(position), self.value(temperature),
                self.value(voltage, 0), self.value(feedback_age, 0),
                "--" if fault is None else f"0x{int(fault):04X}",
                "--" if safety is None else bit_names(
                    int(safety), DRIVE_SAFETY_NAMES
                ),
                "--" if valid is None else f"0x{int(valid):02X}",
            ))
        self.replace_rows(self.drive_tree, drive_values)

    @staticmethod
    def replace_rows(tree: ttk.Treeview, values: list[tuple[object, ...]]) -> None:
        children = tree.get_children()
        for index, row in enumerate(values):
            if index < len(children):
                tree.item(children[index], values=row)
            else:
                tree.insert("", "end", values=row)


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--request-period", type=float, default=0.1,
        help="seconds between paged telemetry requests (minimum 0.05)",
    )
    parser.add_argument("--no-autoconnect", action="store_true")
    args = parser.parse_args()
    try:
        args.port = resolve_uart_port(args.port)
    except RuntimeError as error:
        parser.error(str(error))
    if args.request_period < 0.05:
        parser.error("--request-period must be at least 0.05 seconds")
    return args


def main() -> int:
    args = arguments()
    try:
        root = tk.Tk()
    except tk.TclError as error:
        print(f"无法打开图形界面：{error}")
        return 2
    ChassisDashboard(
        root, args.port, args.baud, args.request_period,
        autoconnect=not args.no_autoconnect,
    )
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
