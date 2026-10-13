"""Optional offline Ceres interface; not imported by the calibration runtime."""

import importlib
import math
from pathlib import Path

import numpy as np
import yaml

from .adaptive_handeye import Transform, quaternion_to_rotation, rotation_to_quaternion


def _pose(value):
    if isinstance(value, Transform):
        # Revalidate, as numpy arrays inside a frozen Transform remain mutable.
        return Transform(value.rotation, value.translation)
    if "quaternion" in value:
        quaternion = np.asarray(value["quaternion"], dtype=float).reshape(4) # xyzw
        if not np.all(np.isfinite(quaternion)) or abs(np.linalg.norm(quaternion) - 1.0) > 1e-8:
            raise ValueError("quaternion must be finite and unit length")
        return Transform(quaternion_to_rotation(*quaternion), value["translation"])
    return Transform(value["rotation"], value["translation"])


def _pack(pose):
    x, y, z, w = rotation_to_quaternion(pose.rotation)
    return [*pose.translation.tolist(), w, x, y, z]


def _unpack(values):
    tx, ty, tz, w, x, y, z = values
    if abs(np.linalg.norm([w, x, y, z]) - 1.0) > 1e-8:
        raise ValueError("Ceres returned a non-unit quaternion")
    return Transform(quaternion_to_rotation(x, y, z, w), [tx, ty, tz])


def optimize_handeye(robot_poses, camera_board_poses, initial_extrinsic, config=None):
    """Return shared X,Y refinement or initial X with CERES_REJECTED.

    Inputs: Transform or {rotation,translation}/{quaternion: xyzw,translation}.
    Config: dictionary, or explicit path to an offline YAML file.
    Outputs: extrinsic/board_pose are Transform; RMS is metres/radians;
    final_cost is Ceres' dimensionless weighted Huber objective.
    On rejection RMS/cost describe the returned INITIAL solution; an attempted
    optimized cost, if available, is separate. An invalid initial input is
    returned untouched and never forwarded to Ceres.
    """
    fallback = {
        "success": False, "extrinsic": initial_extrinsic, "board_pose": None,
        "translation_rms": None, "rotation_rms": None, "iterations": 0,
        "final_cost": None, "failure_reason": "",
    }
    try:
        initial = _pose(initial_extrinsic)
    except (ValueError, TypeError, KeyError) as error:
        fallback["failure_reason"] = f"CERES_REJECTED: INVALID_INITIAL_EXTRINSIC: {error}"
        return fallback
    fallback["extrinsic"] = initial
    try:
        backend = importlib.import_module("_handeye_ceres")
        settings = backend.Config()
        if isinstance(config, (str, Path)):
            config = yaml.safe_load(Path(config).read_text())
        if config is not None:
            if not isinstance(config, dict):
                raise ValueError("config must be a mapping")
            for name, value in config.items():
                if not hasattr(settings, name):
                    raise ValueError(f"unknown Ceres parameter: {name}")
                if not isinstance(value, (int, float)) or not math.isfinite(value):
                    raise ValueError(f"non-finite Ceres parameter: {name}")
                setattr(settings, name, value)
        robot = [_pack(_pose(item)) for item in robot_poses]
        camera = [_pack(_pose(item)) for item in camera_board_poses]
        result = dict(backend.optimize_handeye(robot, camera, _pack(initial), settings))
        result["extrinsic"] = _unpack(result["extrinsic"]) if result["success"] else initial
        if result["board_pose"] is not None:
            result["board_pose"] = _unpack(result["board_pose"])
        return result
    except (ImportError, ValueError, TypeError, KeyError, RuntimeError, OSError) as error:
        fallback["failure_reason"] = f"CERES_REJECTED: {type(error).__name__}: {error}"
        return fallback
