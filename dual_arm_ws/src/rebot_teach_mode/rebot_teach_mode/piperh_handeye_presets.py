"""Install collision-checkable Piper-H hand-eye calibration action presets."""

from __future__ import annotations

from datetime import datetime, timezone

from .action_library import ActionGroupLibrary
from .trajectory import RecordedPoint, RecordedTrajectory


PIPERH_EYE_ON_BASE_SEQUENCE = "自动手眼标定_PiperH_12姿态"
PIPERH_EYE_IN_HAND_SEQUENCE = "自动手眼标定_PiperH_眼在手上_Link5_18姿态"
PIPERH_EYE_IN_HAND_ACTION_PREFIX = "眼在手上Link5标定姿态_"

_JOINT_NAMES = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
_EYE_ON_BASE_POSES = (
    (0.00, 1.40, -1.00, 0.00, 0.50, 0.00),
    (-0.25, 1.40, -1.00, 0.00, 0.50, 0.00),
    (0.25, 1.40, -1.00, 0.00, 0.50, 0.00),
    (0.00, 1.20, -0.90, 0.00, 0.50, 0.00),
    (0.00, 1.60, -1.10, 0.00, 0.50, 0.00),
    (0.00, 1.40, -1.00, 0.35, 0.50, 0.00),
    (0.00, 1.40, -1.00, -0.35, 0.50, 0.00),
    (0.00, 1.40, -1.00, 0.00, 0.30, 0.00),
    (0.00, 1.40, -1.00, 0.00, 0.70, 0.00),
    (0.00, 1.40, -1.00, 0.00, 0.50, 0.45),
    (-0.18, 1.28, -0.90, 0.25, 0.40, -0.25),
    (0.18, 1.52, -1.10, -0.25, 0.60, 0.25),
)
_EYE_IN_HAND_POSES = (
    (0.00, 1.40, -1.00, 0.00, 0.50, 0.00),
    (-0.15, 1.40, -1.00, 0.00, 0.50, 0.00),
    (0.15, 1.40, -1.00, 0.00, 0.50, 0.00),
    (0.00, 1.30, -0.95, 0.00, 0.50, 0.00),
    (0.00, 1.50, -1.05, 0.00, 0.50, 0.00),
    (0.00, 1.40, -1.00, 0.20, 0.50, 0.00),
    (0.00, 1.40, -1.00, -0.20, 0.50, 0.00),
    (0.00, 1.40, -1.00, 0.00, 0.40, 0.00),
    (0.00, 1.40, -1.00, 0.00, 0.60, 0.00),
    (0.00, 1.40, -1.00, 0.22, 0.62, 0.00),
    (-0.10, 1.32, -0.94, 0.15, 0.44, 0.00),
    (0.10, 1.48, -1.06, -0.15, 0.56, 0.00),
    (-0.12, 1.34, -0.92, 0.30, 0.42, 0.00),
    (0.12, 1.46, -1.08, -0.30, 0.58, 0.00),
    (-0.08, 1.48, -1.02, -0.28, 0.42, 0.00),
    (0.08, 1.32, -0.98, 0.28, 0.58, 0.00),
    (-0.12, 1.38, -1.08, 0.35, 0.55, 0.00),
    (0.12, 1.42, -0.92, -0.35, 0.45, 0.00),
)


def _stationary_trajectory(position) -> RecordedTrajectory:
    created = datetime.now(timezone.utc).isoformat()
    positions = tuple(float(value) for value in position)
    return RecordedTrajectory(
        joint_names=_JOINT_NAMES,
        points=(RecordedPoint(0.0, positions), RecordedPoint(3.0, positions)),
        created_utc=created,
    )


def _install_sequence(
    library: ActionGroupLibrary,
    sequence_name: str,
    action_prefix: str,
    poses,
    layout_name: str,
) -> None:
    action_names = []
    for index, pose in enumerate(poses, 1):
        name = f"{action_prefix}{index:02d}"
        action_names.append(name)
        if not library.path_for(name).is_file():
            library.save(
                name,
                f"Piper-H {layout_name}自动标定静止采样姿态 {index:02d}",
                _stationary_trajectory(pose),
            )
    if not library.sequence_path_for(sequence_name).is_file():
        library.save_sequence(sequence_name, action_names)


def ensure_piperh_handeye_presets(library: ActionGroupLibrary) -> None:
    """Create missing presets without overwriting user-edited Piper-H actions."""
    if library.robot_model != "piperh" or library.joint_names != _JOINT_NAMES:
        raise ValueError("Piper-H hand-eye presets require the Piper-H six-axis library")
    _install_sequence(
        library,
        PIPERH_EYE_ON_BASE_SEQUENCE,
        "手眼标定姿态_",
        _EYE_ON_BASE_POSES,
        "眼在手外",
    )
    _install_sequence(
        library,
        PIPERH_EYE_IN_HAND_SEQUENCE,
        PIPERH_EYE_IN_HAND_ACTION_PREFIX,
        _EYE_IN_HAND_POSES,
        "眼在手上",
    )
