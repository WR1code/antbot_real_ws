import math

import pytest

from rebotarm_pulse.joint_target_validation import validate_joint_target


NAMES = [f"joint{index}" for index in range(1, 7)]
LOWER = [-2.8, -3.14, -3.14, -1.87, -1.57, -3.14]
UPPER = [2.8, 0.005, 0.005, 1.57, 1.57, 3.14]


def validate(values):
    return validate_joint_target(values, NAMES, LOWER, UPPER, [2, 4], [0.15, 0.15])


def test_default_dm_escape_target_is_accepted():
    target = [0.0, -1.60, -0.90, -0.90, 0.50, 0.0]
    assert validate(target) == target


@pytest.mark.parametrize(
    "target, message",
    [
        ([0.0] * 5, "must contain 6"),
        ([0.0, -0.35, -0.55, 0.20, math.nan, 0.0], "not finite"),
        ([0.0, 0.20, -0.55, 0.20, 0.30, 0.0], "outside"),
        ([0.0, -0.35, -0.05, 0.20, 0.30, 0.0], "joint3 must be"),
        ([0.0, -0.35, -0.55, 0.20, 0.02, 0.0], "joint5 must be"),
    ],
)
def test_invalid_escape_target_is_rejected(target, message):
    with pytest.raises(ValueError, match=message):
        validate(target)
