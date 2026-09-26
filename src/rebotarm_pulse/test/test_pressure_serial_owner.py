import os
import pty

import pytest

from rebotarm_pulse.pressure_serial_owner import (
    PressureSerialLease,
    SerialDeviceInUseError,
    serial_device,
    serial_owner_pids,
)


def test_by_id_and_device_path_have_same_identity(tmp_path):
    device = tmp_path / "tty-test"
    device.touch()
    alias = tmp_path / "by-id"
    alias.symlink_to(device)
    assert serial_device(str(alias)) == str(device)


def test_duplicate_lease_cannot_claim_same_device(tmp_path):
    device = tmp_path / "tty-test"
    device.touch()
    first = PressureSerialLease(str(device))
    second = PressureSerialLease(str(device))
    first.acquire()
    try:
        with pytest.raises(SerialDeviceInUseError, match="already leased"):
            second.acquire()
    finally:
        first.release()
    second.acquire()
    second.release()


def test_proc_owner_scan_finds_open_serial_descriptor():
    master, slave = pty.openpty()
    try:
        device = os.ttyname(slave)
        assert os.getpid() in serial_owner_pids(device)
        assert os.getpid() not in serial_owner_pids(device, exclude_pid=os.getpid())
    finally:
        os.close(slave)
        os.close(master)
