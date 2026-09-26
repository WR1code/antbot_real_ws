"""Firmware-pinned Piper-H MIT transport with no role-switching support."""

from __future__ import annotations

import re
import time
from collections.abc import Sequence


_FIRMWARE = re.compile(rb"S-V(\d+)\.(\d+)-(\d)")


def firmware_profile(version: str) -> str:
    match = _FIRMWARE.fullmatch(version.encode("ascii", errors="strict"))
    if match is None:
        raise ValueError(f"unrecognized Piper firmware version: {version!r}")
    major, minor, patch = (int(part) for part in match.groups())
    if major != 1:
        raise ValueError(f"unsupported Piper firmware version: {version}")
    if minor < 8 or (minor == 8 and patch <= 2):
        return "default"
    if minor == 8 and patch <= 7:
        return "v183"
    if minor == 8 and patch == 8:
        return "v188"
    return "v189"


def read_firmware(can_port: str, timeout: float = 2.0) -> str:
    """Send only the documented 0x4AF read query and parse its reply."""
    import can

    with can.Bus(channel=can_port, interface="socketcan", receive_own_messages=False) as bus:
        bus.set_filters([{"can_id": 0x4AF, "can_mask": 0x7FF, "extended": False}])
        bus.send(
            can.Message(
                arbitration_id=0x4AF,
                is_extended_id=False,
                data=[1, 0, 0, 0, 0, 0, 0, 0],
            ),
            timeout=0.2,
        )
        response = bytearray()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            frame = bus.recv(timeout=max(0.0, deadline - time.monotonic()))
            if frame is None:
                break
            if frame.arbitration_id == 0x4AF and not frame.is_extended_id:
                response.extend(frame.data)
                match = _FIRMWARE.search(response)
                if match:
                    return match.group().decode("ascii")
    raise RuntimeError("Piper firmware query 0x4AF did not return a valid version")


class OfficialMitBackend:
    """Small Piper-H-only wrapper around pyAgxArm's public MIT API."""

    def __init__(self, can_port: str, firmware: str):
        from pyAgxArm import AgxArmFactory, ArmModel, create_agx_arm_config

        config = create_agx_arm_config(
            robot=ArmModel.PIPER_H,
            firmeware_version=firmware_profile(firmware),
            channel=can_port,
        )
        self._arm = AgxArmFactory.create_arm(config)

    def connect(self) -> None:
        self._arm.connect()
        self._arm.set_auto_set_motion_mode_enabled(False)

    def enter_mit(self) -> None:
        self._arm.set_motion_mode("mit")

    def read_motor_state(self):
        motors = [self._arm.get_motor_states(index) for index in range(1, 7)]
        if not all(motor is not None for motor in motors):
            raise RuntimeError("Piper-H MIT motor feedback is unavailable")
        return (
            [float(motor.msg.position) for motor in motors],
            [float(motor.msg.velocity) for motor in motors],
            [float(motor.msg.torque) for motor in motors],
            min(float(motor.timestamp) for motor in motors),
        )

    def wait_motor_state(self, timeout: float = 1.5):
        deadline = time.monotonic() + timeout
        last_error = None
        while time.monotonic() < deadline:
            try:
                return self.read_motor_state()
            except RuntimeError as error:
                last_error = error
                time.sleep(0.02)
        raise RuntimeError(
            f"Piper-H MIT motor feedback unavailable after {timeout:.1f}s"
        ) from last_error

    def send_joint(
        self, index: int, position: float, kp: float, kd: float, torque: float
    ) -> None:
        self._arm.move_mit(
            joint_index=index, p_des=position, v_des=0.0,
            kp=kp, kd=kd, t_ff=torque,
        )

    def hold(self, positions: Sequence[float]) -> None:
        self._arm.set_motion_mode("j")
        target = [float(value) for value in positions]
        for _ in range(3):
            self._arm.move_j(target)
            time.sleep(0.02)

    def disconnect(self) -> None:
        self._arm.disconnect()
