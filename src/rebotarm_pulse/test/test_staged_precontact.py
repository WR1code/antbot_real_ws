import pytest

from rebotarm_pulse.staged_precontact import (
    approach_candidates,
    dot,
    first_valid_candidate,
    make_staged_precontact,
)
from rebotarm_pulse.approach_geometry import subtract


Q_IDENTITY = (0.0, 0.0, 0.0, 1.0)


def _plan(current_link=(0.0, 0.0, 0.10), target=(0.0, 0.0, 0.30)):
    return make_staged_precontact(
        current_link, Q_IDENTITY, target, 0.060, 0.11599, 0.300, 0.200, 0.001
    )


def test_already_above_target_has_no_meaningless_lateral_waypoint():
    plan = _plan()
    assert plan.lateral_offset == pytest.approx((0.0, 0.0, 0.0))
    assert plan.hover_tip == pytest.approx(plan.current_tip)
    assert plan.hover_link == pytest.approx((0.0, 0.0, 0.10))


def test_lateral_offset_is_completed_without_reducing_clearance():
    plan = _plan(target=(0.020, -0.010, 0.30))
    assert plan.lateral_offset == pytest.approx((0.020, -0.010, 0.0))
    assert dot(subtract(plan.target, plan.hover_tip), plan.nominal_approach) == pytest.approx(
        plan.clearance_m
    )
    assert plan.hover_tip[:2] == pytest.approx(plan.target[:2])


def test_nominal_straight_approach_has_first_priority():
    candidates = approach_candidates(_plan(), (1.0, 0.0, 0.0))
    assert [candidate.angle_deg for candidate in candidates] == [0.0, 3.0, -3.0, 6.0, -6.0]
    assert all(candidate.tangent_shift_m <= 0.008 for candidate in candidates)
    selected = first_valid_candidate(candidates, lambda candidate: True)
    assert selected is not None
    assert selected.angle_deg == 0.0
    assert selected.tangent_shift_m == pytest.approx(0.0)


def test_dummy_target_mode_requires_explicit_opt_in_and_ends_at_target():
    with pytest.raises(ValueError, match="standoff_m"):
        make_staged_precontact(
            (0.0, 0.0, 0.10), Q_IDENTITY, (0.0, 0.0, 0.30),
            0.0, 0.11599, 0.300, 0.200,
        )
    plan = make_staged_precontact(
        (0.0, 0.0, 0.10), Q_IDENTITY, (0.0, 0.0, 0.30),
        0.0, 0.11599, 0.300, 0.200, allow_zero_standoff=True,
    )
    assert plan.nominal_precontact_tip == pytest.approx(plan.target)
    assert plan.nominal_precontact_link == pytest.approx((0.0, 0.0, 0.18401))


def test_small_deviation_is_used_when_nominal_path_is_unavailable():
    candidates = approach_candidates(_plan(), (1.0, 0.0, 0.0))
    selected = first_valid_candidate(
        candidates, lambda candidate: candidate.angle_deg > 0.0
    )
    assert selected is not None
    assert selected.angle_deg == pytest.approx(3.0)
    assert selected.tangent_shift_m > 0.0


def test_collision_failure_rejects_every_candidate():
    candidates = approach_candidates(_plan(), (1.0, 0.0, 0.0))
    assert first_valid_candidate(candidates, lambda candidate: False) is None


def test_optimization_never_descends_below_precontact_standoff():
    plan = _plan(target=(0.010, 0.0, 0.30))
    for candidate in approach_candidates(plan, (1.0, 0.0, 0.0)):
        clearance = dot(
            subtract(plan.target, candidate.tip), plan.nominal_approach
        )
        assert clearance >= 0.060 - 1e-9
        assert dot(
            subtract(plan.target, plan.hover_tip), plan.nominal_approach
        ) == pytest.approx(plan.clearance_m)


def test_59_point_6_mm_clearance_is_accepted_without_inward_approach():
    current_link = (0.0, 0.0, 0.10)
    current_tip_z = current_link[2] + 0.11599
    plan = make_staged_precontact(
        current_link, Q_IDENTITY, (0.0, 0.0, current_tip_z + 0.0596),
        0.060, 0.11599, 0.300, 0.200, 0.001,
    )
    assert plan.clearance_m == pytest.approx(0.0596)
    assert plan.approach_distance_m == 0.0
    assert plan.hover_tip == pytest.approx(plan.nominal_precontact_tip)
    candidates = approach_candidates(plan, (1.0, 0.0, 0.0))
    assert len(candidates) == 1
    for candidate in candidates:
        assert candidate.tip == pytest.approx(plan.nominal_precontact_tip)


def test_close_clearance_uses_direct_nominal_goal_without_retreat_waypoint():
    current_link = (0.0, 0.0, 0.10)
    current_tip_z = current_link[2] + 0.11599
    plan = make_staged_precontact(
        current_link, Q_IDENTITY, (0.02, -0.01, current_tip_z - 0.0092),
        0.060, 0.11599, 0.300, 0.200, 0.001,
    )
    assert plan.clearance_m == pytest.approx(-0.0092)
    assert plan.hover_tip == pytest.approx((0.02, -0.01, current_tip_z - 0.0692))
    assert plan.nominal_precontact_tip == pytest.approx(plan.hover_tip)
    assert plan.approach_distance_m == 0.0
    candidates = approach_candidates(plan, (1.0, 0.0, 0.0))
    assert len(candidates) == 1
    for candidate in candidates:
        assert candidate.tip == pytest.approx(plan.nominal_precontact_tip)
