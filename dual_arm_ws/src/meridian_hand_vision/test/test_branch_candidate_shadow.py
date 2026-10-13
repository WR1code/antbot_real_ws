from copy import deepcopy
from types import SimpleNamespace

from meridian_hand_vision.branch_candidate_shadow import (
    BranchCandidateShadowState,
    update_branch_candidate_shadow,
)
from meridian_hand_vision.pulse_region import WristGeometryBranchDescriptor


def _descriptor(axis=(1.0, 0.0, 0.0), offset=0.0):
    return WristGeometryBranchDescriptor(
        axis=axis,
        centroid_m=(0.08 + offset, 0.0, 0.7),
        near_endpoint_m=(0.03 + offset, 0.0, 0.7),
        far_endpoint_m=(0.14 + offset, 0.0, 0.7),
        effective_length_m=0.11,
        median_width_m=0.055,
    )


def _branch(
    branch_id=0,
    *,
    reason="NONE",
    valid=True,
    descriptor=None,
    linearity=4.0,
    length=0.11,
    rms=0.002,
):
    descriptor = descriptor or _descriptor()
    features = SimpleNamespace(
        effective_length_m=length,
        continuous_span_m=0.10,
        descriptor=descriptor,
    )
    return SimpleNamespace(
        branch_id=branch_id,
        point_count=420,
        span_m=0.12,
        pca_direction=SimpleNamespace(vector=descriptor.axis),
        pca_linearity=linearity,
        valid_section_count=7,
        inlier_section_count=6,
        inlier_ratio=6.0 / 7.0,
        axis=(SimpleNamespace(vector=descriptor.axis) if rms is not None else None),
        axis_length_m=length if rms is not None else None,
        rms_m=rms,
        valid=valid,
        reason=reason,
        sections=tuple(range(8)),
        features=features,
    )


def _estimate(branches=(), selected=None, reason="NO_VALID_BRANCH"):
    return SimpleNamespace(
        branches=tuple(branches),
        selected_branch=selected,
        reason=reason,
    )


def _tracking(active=None, mappings=()):
    return SimpleNamespace(
        active_track_id=active,
        branches=tuple(
            SimpleNamespace(branch_id=branch_id, track_id=track_id)
            for branch_id, track_id in mappings
        ),
    )


def test_records_every_rejected_candidate_and_best_rejection():
    branches = (
        _branch(
            0, reason="BRANCH_TOO_SHORT", valid=False,
            length=0.035, rms=None,
        ),
        _branch(
            1, reason="BRANCH_NOT_LINEAR", valid=False,
            linearity=1.8, rms=None,
        ),
    )
    result = update_branch_candidate_shadow(
        BranchCandidateShadowState(),
        _estimate(branches),
        _tracking(),
        SimpleNamespace(connected_point_count=900),
    )

    assert result["candidate_count"] == 2
    assert result["valid_candidate_count"] == 0
    assert {item["branch_id"] for item in result["candidates"]} == {0, 1}
    assert result["best_rejected_candidate"]["branch_id"] == 1
    assert result["best_candidate_reason"] == "BRANCH_EXISTS_BUT_NOT_LINEAR"
    assert result["failure_hypothesis"] == "B_CANDIDATE_REJECTED_BY_QUALITY"


def test_connected_cloud_failure_has_no_geometric_candidate():
    result = update_branch_candidate_shadow(
        BranchCandidateShadowState(),
        _estimate((), reason="CONNECTED_CLOUD_TOO_SMALL"),
        _tracking(),
        SimpleNamespace(connected_point_count=52),
    )

    assert result["candidate_count"] == 0
    assert result["best_rejected_candidate"] is None
    assert result["best_candidate_reason"] == "NO_GEOMETRIC_CANDIDATE"
    assert result["failure_hypothesis"] == "A_BRANCH_REALLY_DISAPPEARED"


def test_previous_forearm_geometry_finds_similar_rejected_candidate():
    state = BranchCandidateShadowState()
    accepted = _branch(0)
    update_branch_candidate_shadow(
        state,
        _estimate((accepted,), selected=0, reason="NONE"),
        _tracking(),
    )
    rejected = _branch(
        4,
        reason="BRANCH_NOT_LINEAR",
        valid=False,
        descriptor=_descriptor(offset=0.004),
        linearity=2.1,
        rms=None,
    )
    result = update_branch_candidate_shadow(
        state,
        _estimate((rejected,)),
        _tracking(),
    )

    association = result["association"]
    assert association["previous_reference_available"]
    assert association["similar_candidate_count"] == 1
    assert association["best_geometric_match"]["branch_id"] == 4
    assert association["assessment"] == (
        "B_SIMILAR_CANDIDATE_REJECTED_BY_QUALITY"
    )


def test_valid_candidates_without_selection_are_classified_as_selection_issue():
    result = update_branch_candidate_shadow(
        BranchCandidateShadowState(),
        _estimate((_branch(0), _branch(1)), reason="AMBIGUOUS_BRANCH"),
        _tracking(),
    )

    assert result["valid_candidate_count"] == 2
    assert result["selected_branch_id"] is None
    assert result["failure_hypothesis"] == "C_VALID_CANDIDATE_NOT_SELECTED"


def test_shadow_does_not_mutate_estimate_or_tracking():
    estimate = _estimate((_branch(0),), selected=0, reason="NONE")
    tracking = _tracking(active=8, mappings=((0, 8),))
    estimate_before = deepcopy(estimate)
    tracking_before = deepcopy(tracking)

    result = update_branch_candidate_shadow(
        BranchCandidateShadowState(), estimate, tracking
    )

    assert result["shadow_only"]
    assert estimate == estimate_before
    assert tracking == tracking_before
