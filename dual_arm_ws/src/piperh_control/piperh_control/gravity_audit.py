"""Passive Piper-H gravity-model audit. Never sends a motor command."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import time

import numpy as np
from ament_index_python.packages import get_package_share_directory

from .gravity_compensator import PiperHGravityCompensator, rotation_from_rpy
from .mit_backend import firmware_profile, read_firmware


def collect(can_port: str, mount_rpy: list[float], duration: float) -> dict:
    from pyAgxArm import AgxArmFactory, ArmModel, create_agx_arm_config

    firmware = read_firmware(can_port)
    profile = firmware_profile(firmware)
    arm = AgxArmFactory.create_arm(create_agx_arm_config(
        robot=ArmModel.PIPER_H, firmeware_version=profile, channel=can_port,
    ))
    urdf = Path(get_package_share_directory("piper_h_description")) / (
        "urdf/piper_h_description.urdf"
    )
    model = PiperHGravityCompensator(
        str(urdf), rotation_from_rpy(*mount_rpy)
    )
    samples = []
    try:
        arm.connect()
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            angles = arm.get_joint_angles()
            motors = [arm.get_motor_states(index) for index in range(1, 7)]
            if angles is not None and all(motor is not None for motor in motors):
                q = np.asarray(angles.msg, dtype=float)
                feedback = np.asarray([motor.msg.torque for motor in motors], dtype=float)
                if q.shape == (6,) and np.all(np.isfinite(q)) and np.all(np.isfinite(feedback)):
                    samples.append((q, feedback))
            time.sleep(0.1)
        if len(samples) < 5:
            raise RuntimeError("too few valid joint/torque feedback samples")
        q = np.median([sample[0] for sample in samples], axis=0)
        feedback = np.median([sample[1] for sample in samples], axis=0)
        joint_range = np.ptp([sample[0] for sample in samples], axis=0)
        status = arm.get_arm_status()
        enabled = [bool(arm.get_joint_enable_status(index)) for index in range(1, 7)]
        return {
            "timestamp_utc": datetime.now(timezone.utc).isoformat(),
            "firmware": firmware,
            "sdk_profile": profile,
            "mount_rpy_rad": mount_rpy,
            "sample_count": len(samples),
            "joint_positions_rad": q.tolist(),
            "joint_position_range_rad": joint_range.tolist(),
            "gravity_world_m_s2": model.gravity_world.tolist(),
            "gravity_base_m_s2": model.gravity_base.tolist(),
            "model_gravity_nm": model.torque(q, np.zeros(6)).tolist(),
            "motor_feedback_nm": feedback.tolist(),
            "motor_enabled": enabled,
            "arm_status": str(status.msg) if status else "unavailable",
            "stationary": bool(np.all(joint_range < 0.01)),
            "note": (
                "Passive observation only. Motor feedback in position/standby mode "
                "does not calibrate MIT command direction or gain."
            ),
        }
    finally:
        arm.disconnect()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--can-port", default="can0")
    parser.add_argument("--mount-rpy", nargs=3, type=float, required=True,
                        metavar=("ROLL", "PITCH", "YAW"))
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if not 1.0 <= args.duration <= 30.0:
        parser.error("--duration must be in [1, 30] seconds")
    report = collect(args.can_port, args.mount_rpy, args.duration)
    content = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(content, encoding="utf-8")
    print(content, end="")


if __name__ == "__main__":
    main()
