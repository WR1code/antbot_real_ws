"""Read-only serial owner discovery and an inter-process pressure-port lock."""

from __future__ import annotations

import fcntl
import hashlib
import os
import tempfile


def serial_device(port: str) -> str:
    """Normalize by-id and tty paths to the same physical device."""
    return os.path.realpath(port) if port else ""


def serial_owner_pids(port: str, exclude_pid: int | None = None) -> list[int]:
    """Find processes currently holding the device open (Linux /proc)."""
    device = serial_device(port)
    if not device or not os.path.exists(device):
        return []
    owners = []
    try:
        pids = os.listdir("/proc")
    except OSError:
        return []
    for entry in pids:
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid == exclude_pid:
            continue
        try:
            descriptors = os.listdir(f"/proc/{pid}/fd")
        except (OSError, PermissionError):
            continue
        for descriptor in descriptors:
            try:
                opened = os.readlink(f"/proc/{pid}/fd/{descriptor}")
            except (OSError, PermissionError):
                continue
            if opened == device:
                owners.append(pid)
                break
    return sorted(owners)


class SerialDeviceInUseError(RuntimeError):
    """A pressure serial device already has another owner."""


class PressureSerialLease:
    """Prevent updated bridge instances from sharing one physical serial port."""

    def __init__(self, port: str) -> None:
        self.device = serial_device(port)
        digest = hashlib.sha256(self.device.encode("utf-8")).hexdigest()[:16]
        self.lock_path = os.path.join(
            tempfile.gettempdir(), f"rebotarm_pulse_serial_{digest}.lock"
        )
        self._fd: int | None = None

    def acquire(self) -> None:
        if not self.device:
            raise ValueError("pressure serial device path is empty")
        fd = os.open(self.lock_path, os.O_CREAT | os.O_RDWR, 0o600)
        try:
            try:
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as error:
                raise SerialDeviceInUseError(
                    f"pressure serial device {self.device} already leased"
                ) from error
            owners = serial_owner_pids(self.device, exclude_pid=os.getpid())
            if owners:
                raise SerialDeviceInUseError(
                    f"pressure serial device {self.device} already open by PID(s) {owners}"
                )
            self._fd = fd
        except BaseException:
            os.close(fd)
            raise

    def release(self) -> None:
        if self._fd is not None:
            os.close(self._fd)
            self._fd = None
