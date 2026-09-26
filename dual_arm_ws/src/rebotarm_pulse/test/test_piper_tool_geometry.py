import math

import pytest

from rebotarm_pulse.piper_tool_geometry import (
    _quaternion_align_z,
    _quaternion_from_rpy,
    _triple,
)


def test_geometry_helpers_validate_and_align_sensor_normal():
    assert _triple([1, 2, 3], "point") == (1.0, 2.0, 3.0)
    with pytest.raises(ValueError, match="three finite"):
        _triple([1, 2], "point")
    assert _quaternion_align_z([0, 0, 1]) == pytest.approx((0, 0, 0, 1))
    assert _quaternion_align_z([0, 0, -1]) == pytest.approx((1, 0, 0, 0))
    with pytest.raises(ValueError, match="non-zero"):
        _quaternion_align_z([0, 0, 0])


def test_rpy_quaternion_is_normalized():
    quaternion = _quaternion_from_rpy(0.2, -0.3, 0.4)
    assert math.sqrt(sum(value * value for value in quaternion)) == pytest.approx(1.0)
