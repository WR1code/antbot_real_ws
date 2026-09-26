"""Pure Piper-H Pinocchio gravity calculations and command limiting.

This module has no CAN/ROS command path.  ``R_world_base`` means the rotation
of the Piper base frame expressed in the world frame, so the model gravity is
``R_world_base.T @ [0, 0, -9.81]``.
"""

from __future__ import annotations

from collections.abc import Sequence
import math

import numpy as np


GRAVITY_WORLD = np.array([0.0, 0.0, -9.81], dtype=float)


def rotation_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Return ``R_world_base`` for fixed-axis XYZ roll/pitch/yaw radians."""
    values = np.asarray([roll, pitch, yaw], dtype=float)
    if not np.all(np.isfinite(values)):
        raise ValueError("base mount roll/pitch/yaw must be finite")
    cr, cp, cy = np.cos(values)
    sr, sp, sy = np.sin(values)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


def validate_rotation(rotation: Sequence[Sequence[float]]) -> np.ndarray:
    value = np.asarray(rotation, dtype=float)
    if value.shape != (3, 3) or not np.all(np.isfinite(value)):
        raise ValueError("R_world_base must be a finite 3x3 matrix")
    if not np.allclose(value.T @ value, np.eye(3), atol=1e-6):
        raise ValueError("R_world_base must be orthonormal")
    if not math.isclose(float(np.linalg.det(value)), 1.0, abs_tol=1e-6):
        raise ValueError("R_world_base must have determinant +1")
    return value


def limit_torque(
    torque: Sequence[float], scale: float, limits_nm: Sequence[float]
) -> np.ndarray:
    """Apply one conservative scale and then symmetric per-joint clamps."""
    tau = np.asarray(torque, dtype=float)
    limits = np.asarray(limits_nm, dtype=float)
    if tau.shape != (6,) or not np.all(np.isfinite(tau)):
        raise ValueError("gravity torque must contain six finite values")
    if not math.isfinite(scale) or not 0.0 <= scale <= 1.0:
        raise ValueError("torque_scale must be within 0.0..1.0")
    if limits.shape != (6,) or not np.all(np.isfinite(limits)) or np.any(limits <= 0):
        raise ValueError("max_torque_nm must contain six positive finite values")
    return np.clip(scale * tau, -limits, limits)


class PiperHGravityCompensator:
    """Compute Piper-H inverse dynamics in an explicitly oriented base frame."""

    def __init__(
        self,
        urdf_path: str,
        r_world_base: Sequence[Sequence[float]],
        payload_mass_kg: float = 0.0,
        payload_com_xyz_m: Sequence[float] = (0.0, 0.0, 0.0),
    ) -> None:
        import pinocchio as pin

        self._pin = pin
        self.r_world_base = validate_rotation(r_world_base)
        self.gravity_world = GRAVITY_WORLD.copy()
        self.gravity_base = self.r_world_base.T @ self.gravity_world
        if not math.isclose(float(np.linalg.norm(self.gravity_base)), 9.81, abs_tol=1e-6):
            raise ValueError("base-frame gravity magnitude is not 9.81 m/s^2")
        self._model = pin.buildModelFromUrdf(str(urdf_path))
        if self._model.nq != 6 or self._model.nv != 6:
            raise ValueError("Piper-H gravity model must contain exactly six joints")
        self._model.gravity.linear = self.gravity_base.copy()

        payload_com = np.asarray(payload_com_xyz_m, dtype=float)
        if (
            not math.isfinite(payload_mass_kg)
            or not 0.0 <= payload_mass_kg <= 3.0
            or payload_com.shape != (3,)
            or not np.all(np.isfinite(payload_com))
            or np.any(np.abs(payload_com) > 0.5)
        ):
            raise ValueError("invalid Piper-H payload mass or center of mass")
        if payload_mass_kg:
            joint6 = self._model.getJointId("joint6")
            self._model.inertias[joint6] += pin.Inertia(
                float(payload_mass_kg), payload_com, np.zeros((3, 3))
            )
        self._data = self._model.createData()

    @staticmethod
    def _state(values: Sequence[float], label: str) -> np.ndarray:
        result = np.asarray(values, dtype=float)
        if result.shape != (6,) or not np.all(np.isfinite(result)):
            raise ValueError(f"{label} must contain six finite values")
        return result

    def torque(self, q: Sequence[float], qd: Sequence[float]) -> np.ndarray:
        """Return RNEA(q, qd, 0), matching the AgileX reference algorithm."""
        position = self._state(q, "q")
        velocity = self._state(qd, "qd")
        result = np.asarray(
            self._pin.rnea(
                self._model,
                self._data,
                position,
                velocity,
                np.zeros(6, dtype=float),
            ),
            dtype=float,
        )
        if result.shape != (6,) or not np.all(np.isfinite(result)):
            raise RuntimeError("Pinocchio produced invalid gravity torque")
        return result
