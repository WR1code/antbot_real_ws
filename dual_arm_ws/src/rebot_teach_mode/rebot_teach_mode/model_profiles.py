"""Model-specific safety limits and storage locations for drag teaching."""

from __future__ import annotations

from copy import deepcopy
import os


_COMMON_ROOT = os.environ.get("ROBOT_MOTION_ROOT", "motions")

_MODEL_PROFILES = {
    "dm": {
        "supports_gravity_compensation": True,
        "robot_model": "rebotarm_dm",
        # DM mode switching includes serial feedback, MIT-mode configuration,
        # disable/enable, and control-loop startup.  It can exceed the generic
        # service-client timeout on a shared USB serial bus.
        "driver_service_timeout_sec": 15.0,
        # The DM serial publisher can deliver feedback in short bursts with
        # observed scheduler gaps around 0.34 s. Keep true disconnect
        # detection below one second without treating one normal gap as loss.
        "feedback_timeout_sec": 0.75,
        "status_timeout_sec": 0.75,
        # Reproduce the demonstrated timing.  The independent 0.30 rad/s
        # joint-velocity ceiling and driver tracking guard remain enforced.
        "replay_speed_scale": 1.0,
        "trajectory_path": (
            f"{_COMMON_ROOT}/dm/teach/latest_dm.motion.json"
        ),
        "action_library_dir": (
            f"{_COMMON_ROOT}/dm/action_groups"
        ),
        # gripper_tcp is 44.3 mm from gripper_link while the DM gripper mesh
        # reaches 150.7 mm along local -X. L is measured from this front face.
        "shape_pen_mount_offset_m": 0.1064,
        # reBot_B601_DM_with_gripper.urdf.  joint2/joint3 have a small
        # positive tolerance at their nominal zero position.
        "joint_lower_limits": [-2.8, -3.14, -3.14, -1.87, -1.57, -3.14],
        "joint_upper_limits": [2.8, 0.005, 0.005, 1.57, 1.57, 3.14],
    },
    "rs": {
        "supports_gravity_compensation": True,
        "robot_model": "rebotarm_rs",
        "driver_service_timeout_sec": 5.0,
        "trajectory_path": (
            f"{_COMMON_ROOT}/rs/teach/latest.motion.json"
        ),
        # Retain the original RS library location for existing recordings.
        "action_library_dir": (
            f"{_COMMON_ROOT}/rs/action_groups"
        ),
        # RS gripper_tcp is at gripper_end origin; its front face is 157.2 mm
        # along local -X.
        "shape_pen_mount_offset_m": 0.1572,
        "joint_lower_limits": [-2.8, 0.0, 0.0, -1.57, -1.57, -3.14],
        "joint_upper_limits": [2.8, 3.14, 3.14, 1.57, 1.57, 3.14],
    },
    "piperh": {
        "supports_gravity_compensation": True,
        "robot_model": "piperh",
        # Sample the ordinary six-joint feedback at 100 Hz when fresh frames
        # permit. A slower upstream stream is never duplicated to fill ticks.
        "sample_period_sec": 0.01,
        "minimum_position_delta_rad": 0.0,
        "maximum_samples": 30000,
        "driver_service_timeout_sec": 6.5,
        # The raw CAN stream stays within the hardware adapter's guarded
        # 0.75-second timeout, but DDS/Python forwarding can pause for about
        # 1.1 seconds under camera load.  Keep the hardware command gate at
        # 0.75 seconds and give this supervisory replay observer more margin.
        "feedback_timeout_sec": 2.0,
        "status_timeout_sec": 0.75,
        "replay_speed_scale": 1.0,
        "trajectory_path": (
            f"{_COMMON_ROOT}/piperh/teach/latest.motion.json"
        ),
        "action_library_dir": (
            f"{_COMMON_ROOT}/piperh/action_groups"
        ),
        "shape_pen_mount_offset_m": 0.0,
        "tcp_base_frame": "base_link",
        "tcp_link_name": "Link6",
        "trajectory_action": "/piperh/arm_controller/follow_joint_trajectory",
        "joint_lower_limits": [-2.618, 0.0, -2.96706, -2.356195, -1.56207, -3.14],
        "joint_upper_limits": [2.618, 3.14, 0.0, 2.356195, 1.56207, 3.14],
    },
}


def teaching_parameters_for_model(model: str) -> dict:
    """Return an isolated parameter mapping for a supported robot model."""
    normalized = str(model).strip().lower()
    try:
        return deepcopy(_MODEL_PROFILES[normalized])
    except KeyError as error:
        raise ValueError(
            f"unsupported teaching model {model!r}; expected dm, rs, or piperh"
        ) from error
