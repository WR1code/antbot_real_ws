from pathlib import Path

import numpy as np
import pytest

from piperh_control.gravity_compensator import (
    GRAVITY_WORLD,
    PiperHGravityCompensator,
    limit_torque,
    rotation_from_rpy,
    validate_rotation,
)


URDF = Path(__file__).parents[2] / "piper_h_description/urdf/piper_h_description.urdf"


@pytest.mark.parametrize(
    "rpy",
    [(0.0, 0.0, 0.0), (np.pi / 2, 0.0, 0.0), (-np.pi / 2, 0.0, 0.0),
     (0.0, np.pi / 2, 0.0), (0.0, -np.pi / 2, 0.0)],
)
def test_upright_and_side_mounts_preserve_gravity_norm(rpy):
    rotation = rotation_from_rpy(*rpy)
    model = PiperHGravityCompensator(str(URDF), rotation)
    assert np.linalg.norm(rotation.T @ GRAVITY_WORLD) == pytest.approx(9.81)
    assert np.all(np.isfinite(model.torque(np.zeros(6), np.zeros(6))))


@pytest.mark.parametrize(
    "rotation",
    [np.zeros((3, 3)), np.diag([1.0, 1.0, -1.0]), [[float("nan")]*3]*3],
)
def test_invalid_rotation_is_rejected(rotation):
    with pytest.raises(ValueError):
        validate_rotation(rotation)


def test_nonfinite_joint_state_is_rejected():
    model = PiperHGravityCompensator(str(URDF), np.eye(3))
    with pytest.raises(ValueError):
        model.torque([0, 0, 0, 0, 0, float("nan")], np.zeros(6))


def test_scale_then_clamp():
    result = limit_torque([10, -10, 2, -2, 1, -1], 0.1, [0.5] * 6)
    assert result.tolist() == pytest.approx([0.5, -0.5, 0.2, -0.2, 0.1, -0.1])


def test_disabled_scale_is_exactly_zero():
    assert np.all(limit_torque(np.ones(6), 0.0, np.ones(6)) == 0.0)
