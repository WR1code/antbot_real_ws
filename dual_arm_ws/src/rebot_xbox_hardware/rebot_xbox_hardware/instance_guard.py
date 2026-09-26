"""Hold a process-wide lock for the combined real-robot launch."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import fcntl
import os
from pathlib import Path
import re
import signal
import sys
import threading
import time


LOCK_EXIT_CODE = 73
LEGACY_NODE_EXIT_CODE = 74
STALE_PROCESS_EXIT_CODE = 75
CORE_NODE_NAMES = {
    "/reBotArmController",
    "/piper_ctrl_single_node",
    "/piperh_hardware_adapter",
    "/move_group",
    "/servo_node",
    "/rviz2",
    "/rebot_xbox_twist",
    "/piperh_joy",
}

# Executables owned by the integrated workbench which can retain CAN/serial
# handles, publish duplicate TF, or leave a second control/UI stack alive.
CONFLICT_EXECUTABLES = {
    "active_arm_manager",
    "forbidden_zone_manager",
    "hardware_adapter",
    "hardware_gripper",
    "instance_guard",
    "joy_linux_node",
    "move_group",
    "piper_single_ctrl",
    "reBotArmController",
    "rebot_xbox_twist",
    "robot_state_publisher",
    "ros2_control_node",
    "rviz2",
    "servo_node",
    "static_transform_publisher",
}
CONFLICT_LAUNCH_FILES = {
    "dual_arm_hardware.launch.py",
    "dual_arm_offline_preview.launch.py",
    "hardware_selector.launch.py",
    "xbox_hardware_servo.launch.py",
}


@dataclass(frozen=True)
class ProcessInfo:
    pid: int
    parent_pid: int
    command: str
    arguments: tuple[str, ...]


def lock_path(name: str, directory: str = "/tmp") -> Path:
    """Return a deterministic, non-broad lock path for one arm namespace."""
    safe_name = re.sub(r"[^A-Za-z0-9_.-]+", "_", name.strip("/")) or "rebotarm"
    return Path(directory) / f"rebotarm-xbox-hardware-{safe_name}.lock"


def acquire_lock(path: Path) -> int:
    """Acquire and identify the owner of an advisory lock."""
    descriptor = os.open(path, os.O_RDWR | os.O_CREAT, 0o644)
    try:
        fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        os.close(descriptor)
        raise
    os.ftruncate(descriptor, 0)
    os.write(descriptor, f"{os.getpid()}\n".encode())
    os.fsync(descriptor)
    return descriptor


def owner_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8").strip() or "unknown"
    except OSError:
        return "unknown"


def ancestor_pids(pid: int | None = None) -> set[int]:
    """Return self/current launch ancestry so the preflight ignores itself."""
    current = os.getpid() if pid is None else pid
    ancestors = set()
    while current > 1 and current not in ancestors:
        ancestors.add(current)
        try:
            fields = Path(f"/proc/{current}/stat").read_text(
                encoding="utf-8", errors="replace"
            ).split()
            current = int(fields[3])
        except (OSError, ValueError, IndexError):
            break
    return ancestors


def running_processes(proc_root: Path = Path("/proc")) -> list[ProcessInfo]:
    """Read a best-effort process snapshot without invoking a shell utility."""
    processes = []
    for entry in proc_root.iterdir():
        if not entry.name.isdigit():
            continue
        try:
            stat_fields = (entry / "stat").read_text(
                encoding="utf-8", errors="replace"
            ).split()
            raw_arguments = (entry / "cmdline").read_bytes().split(b"\0")
            arguments = tuple(
                value.decode(errors="replace") for value in raw_arguments if value
            )
            if not arguments:
                continue
            command = (entry / "comm").read_text(
                encoding="utf-8", errors="replace"
            ).strip()
            processes.append(
                ProcessInfo(
                    pid=int(entry.name),
                    parent_pid=int(stat_fields[3]),
                    command=command,
                    arguments=arguments,
                )
            )
        except (OSError, ValueError, IndexError):
            # A process may exit between listing and reading /proc.
            continue
    return processes


def process_conflict_reason(process: ProcessInfo) -> str | None:
    """Describe why a process can collide with this workbench."""
    candidates = {process.command}
    if process.arguments:
        candidates.add(Path(process.arguments[0]).name)
    if (
        len(process.arguments) > 1
        and Path(process.arguments[0]).name.startswith("python")
    ):
        candidates.add(Path(process.arguments[1]).name)
    executable = sorted(CONFLICT_EXECUTABLES.intersection(candidates))
    if executable:
        return f"executable={executable[0]}"

    launch_files = CONFLICT_LAUNCH_FILES.intersection(
        Path(argument).name for argument in process.arguments
    )
    if launch_files:
        return f"launch={sorted(launch_files)[0]}"
    return None


def conflicting_processes(
    processes, excluded_pids=()
) -> list[tuple[ProcessInfo, str]]:
    excluded = set(excluded_pids)
    conflicts = []
    for process in processes:
        if process.pid in excluded:
            continue
        reason = process_conflict_reason(process)
        if reason:
            conflicts.append((process, reason))
    return sorted(conflicts, key=lambda item: item[0].pid)


def format_process_conflict(process: ProcessInfo, reason: str) -> str:
    arguments = " ".join(process.arguments)
    if len(arguments) > 180:
        arguments = arguments[:177] + "..."
    return f"pid={process.pid} {reason} command={arguments}"


def conflicting_node_names(node_names) -> list[str]:
    """Return legacy/core nodes that make a new combined stack unsafe."""
    normalized = {
        name if name.startswith("/") else f"/{name}" for name in node_names
    }
    return sorted(CORE_NODE_NAMES.intersection(normalized))


def discover_conflicting_nodes(ros_args, timeout: float) -> list[str]:
    """Probe the live ROS graph before any delayed stack process is started."""
    import rclpy

    rclpy.init(args=ros_args)
    node = rclpy.create_node("rebotarm_single_instance_probe")
    try:
        deadline = time.monotonic() + max(0.1, timeout)
        conflicts: list[str] = []
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            conflicts = conflicting_node_names(node.get_node_names())
            if conflicts:
                break
        return conflicts
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--lock-name", default="rebotarm")
    parser.add_argument("--discovery-timeout", type=float, default=0.6)
    # launch_ros appends ROS arguments even though this guard is not a ROS node.
    args, ros_args = parser.parse_known_args(argv)
    path = lock_path(args.lock_name)
    try:
        descriptor = acquire_lock(path)
    except BlockingIOError:
        print(
            f"[reBotArm] 拒绝重复启动：已有工作台持有锁 lock={path} "
            f"owner_pid={owner_text(path)}。请先正常关闭旧工作台。",
            file=sys.stderr,
            flush=True,
        )
        return LOCK_EXIT_CODE

    process_conflicts = conflicting_processes(
        running_processes(), excluded_pids=ancestor_pids()
    )
    if process_conflicts:
        details = "; ".join(
            format_process_conflict(process, reason)
            for process, reason in process_conflicts
        )
        print(
            "[reBotArm] 拒绝启动：检测到可能冲突的残留进程："
            + details
            + "。请先正常停止上述进程，再重新启动。",
            file=sys.stderr,
            flush=True,
        )
        os.close(descriptor)
        return STALE_PROCESS_EXIT_CODE

    try:
        conflicts = discover_conflicting_nodes(ros_args, args.discovery_timeout)
    except Exception as error:
        print(
            f"[reBotArm] 拒绝启动：ROS 节点图预检失败：{error}",
            file=sys.stderr,
            flush=True,
        )
        os.close(descriptor)
        return LEGACY_NODE_EXIT_CODE
    if conflicts:
        print(
            "[reBotArm] 拒绝启动：ROS 图中已有可能冲突的核心节点："
            + ", ".join(conflicts)
            + "。请先关闭旧 launch，再启动双臂工作台。",
            file=sys.stderr,
            flush=True,
        )
        os.close(descriptor)
        return LEGACY_NODE_EXIT_CODE

    stop = threading.Event()

    def request_stop(_signum, _frame):
        stop.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    print(f"[reBotArm] single-instance guard ready: {path}", flush=True)
    try:
        stop.wait()
    finally:
        os.close(descriptor)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
