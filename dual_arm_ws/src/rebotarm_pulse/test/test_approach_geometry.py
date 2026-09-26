import math

import pytest

from rebotarm_pulse.approach_geometry import (
    inside_workspace,
    piper_current_tip_gap,
    piper_precontact_link_pose,
    precontact_tcp_pose,
    quaternion_align_x,
    rotate_vector,
    translation_within_axis_limits,
)


def _rotate_x(quaternion):
    x, y, z, w = quaternion
    return (
        1.0 - 2.0 * (y * y + z * z),
        2.0 * (x * y + w * z),
        2.0 * (x * z - w * y),
    )


def test_quaternion_aligns_probe_x_axis_to_target():
    quaternion = quaternion_align_x((0.0, 0.0, 1.0))
    assert _rotate_x(quaternion) == pytest.approx((0.0, 0.0, 1.0), abs=1e-8)
    assert math.sqrt(sum(value * value for value in quaternion)) == pytest.approx(1.0)


def test_precontact_stops_before_surface_and_accounts_for_tip_offset():
    tcp, quaternion, tip = precontact_tcp_pose(
        (0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 0.06, 0.10
    )
    assert tip == pytest.approx((0.0, 0.0, 0.94))
    assert tcp == pytest.approx((0.0, 0.0, 0.84))
    assert _rotate_x(quaternion) == pytest.approx((0.0, 0.0, 1.0), abs=1e-8)


def test_precontact_rejects_contact_distance():
    with pytest.raises(ValueError, match="at least"):
        precontact_tcp_pose((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 0.0, 0.0)


def test_workspace_limits_are_inclusive():
    limits = (0.1, 0.6, -0.4, 0.4, 0.0, 0.5)
    assert inside_workspace((0.1, -0.4, 0.5), limits)
    assert not inside_workspace((0.7, 0.0, 0.1), limits)


def test_translation_limits_use_xy_radius_and_independent_absolute_z():
    origin = (0.0, 0.0, 0.0)
    assert translation_within_axis_limits((0.18, 0.24, 0.20), origin, 0.30, 0.20)
    assert not translation_within_axis_limits(
        (0.181, 0.24, 0.20), origin, 0.30, 0.20
    )
    assert not translation_within_axis_limits(
        (0.18, 0.24, -0.201), origin, 0.30, 0.20
    )


def test_piper_precontact_preserves_orientation_and_uses_local_positive_z():
    goal, orientation, tip, distance = piper_precontact_link_pose(
        current_link_origin=(0.0, 0.0, 0.50),
        current_orientation=(0.0, 0.0, 0.0, 1.0),
        target=(0.02, -0.01, 0.70),
        standoff_m=0.03,
        tip_offset_m=0.11599,
        maximum_horizontal_translation_m=0.10,
        maximum_vertical_translation_m=0.10,
    )
    assert goal == pytest.approx((0.02, -0.01, 0.55401))
    assert tip == pytest.approx((0.02, -0.01, 0.67))
    assert orientation == pytest.approx((0.0, 0.0, 0.0, 1.0))
    assert distance == pytest.approx(math.sqrt(0.02 ** 2 + 0.01 ** 2 + 0.05401 ** 2))


def test_piper_precontact_rejects_large_horizontal_automatic_correction():
    with pytest.raises(ValueError, match="horizontal/vertical"):
        piper_precontact_link_pose(
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, 1.0),
            (0.20, 0.0, 0.20),
            0.03,
            0.11599,
            0.05,
            0.20,
        )


def test_piper_precontact_rejects_large_vertical_automatic_correction():
    with pytest.raises(ValueError, match="horizontal/vertical"):
        piper_precontact_link_pose(
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, 1.0),
            (0.01, 0.01, 0.50),
            0.03,
            0.11599,
            0.30,
            0.20,
        )


def test_rotate_vector_rejects_zero_quaternion():
    with pytest.raises(ValueError, match="quaternion"):
        rotate_vector((0.0, 0.0, 0.0, 0.0), (0.0, 0.0, 1.0))


def test_piper_current_tip_gap_rejects_start_below_standoff():
    assert piper_current_tip_gap(
        (0.0, 0.0, 0.50), (0.0, 0.0, 0.0, 1.0),
        (0.02, 0.0, 0.70), 0.11599,
    ) == pytest.approx(0.08401)
    assert piper_current_tip_gap(
        (0.0, 0.0, 0.66), (0.0, 0.0, 0.0, 1.0),
        (0.02, 0.0, 0.70), 0.11599,
    ) < 0.0
