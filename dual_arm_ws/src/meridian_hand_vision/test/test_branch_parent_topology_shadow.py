from copy import deepcopy
from types import SimpleNamespace

import numpy as np

from meridian_hand_vision.branch_parent_topology_shadow import (
    analyze_branch_parent_topology_shadow,
)


def _descriptor(axis, centroid, near, far, length=0.12, width=0.045):
    return SimpleNamespace(
        axis=axis,
        centroid_m=centroid,
        near_endpoint_m=near,
        far_endpoint_m=far,
        effective_length_m=length,
        median_width_m=width,
    )


def _branch(branch_id, parent_id, descriptor, *, valid=True, reason="NONE", local=None):
    return SimpleNamespace(
        branch_id=branch_id,
        parent_component_id=parent_id,
        valid=valid,
        reason=reason,
        axis=SimpleNamespace(vector=descriptor.axis) if valid else None,
        pca_direction=SimpleNamespace(vector=descriptor.axis),
        axis_length_m=descriptor.effective_length_m if valid else None,
        span_m=descriptor.effective_length_m,
        features=SimpleNamespace(
            descriptor=descriptor,
            effective_length_m=descriptor.effective_length_m,
        ),
        local_recovery_shadow=local,
    )


def _straight(start, end, count=80):
    return np.linspace(np.asarray(start), np.asarray(end), count)


def test_sibling_topology_identifies_forearm_and_palm_sibling():
    wrist = np.asarray((0.0, 0.0, 0.70))
    forearm_points = _straight((0.03, 0.0, 0.70), (0.16, 0.0, 0.70))
    palm_points = _straight((0.0, 0.03, 0.70), (0.0, 0.13, 0.70))
    points = np.vstack((forearm_points, palm_points))
    forearm_descriptor = _descriptor(
        (1.0, 0.0, 0.0), (0.095, 0.0, 0.70),
        (0.03, 0.0, 0.70), (0.16, 0.0, 0.70), length=0.13,
    )
    palm_descriptor = _descriptor(
        (0.0, 1.0, 0.0), (0.0, 0.08, 0.70),
        (0.0, 0.03, 0.70), (0.0, 0.13, 0.70), length=0.10,
    )
    branches = (
        _branch(0, 4, forearm_descriptor),
        _branch(1, 4, palm_descriptor, valid=False, reason="BRANCH_NOT_LINEAR"),
    )

    result = analyze_branch_parent_topology_shadow(
        wrist,
        points,
        {4: np.arange(points.shape[0])},
        {0: np.arange(80), 1: np.arange(80, 160)},
        branches,
        previous_active_descriptor=forearm_descriptor,
    )
    by_id = {item["branch_id"]: item for item in result["branches"]}

    assert by_id[0]["classification"] == "FOREARM_LIKE"
    assert by_id[1]["classification"] == "PALM_LIKE"
    assert by_id[1]["origin_hypothesis"] == "SIBLING_OF_ACTIVE_FOREARM"
    assert by_id[1]["siblings"]["sibling_count"] == 1
    assert 89.0 <= by_id[1]["siblings"]["sibling_axis_angles_deg"][0] <= 90.0


def test_previous_active_relation_records_explainable_geometry():
    wrist = np.asarray((0.0, 0.0, 0.70))
    points = _straight((0.03, 0.0, 0.70), (0.15, 0.0, 0.70))
    previous = _descriptor(
        (1.0, 0.0, 0.0), (0.09, 0.0, 0.70),
        (0.03, 0.0, 0.70), (0.15, 0.0, 0.70),
    )
    current = _descriptor(
        (1.0, 0.0, 0.0), (0.094, 0.0, 0.70),
        (0.034, 0.0, 0.70), (0.154, 0.0, 0.70),
    )
    branch = _branch(0, 0, current)

    result = analyze_branch_parent_topology_shadow(
        wrist, points, {0: np.arange(80)}, {0: np.arange(80)},
        (branch,), previous_active_descriptor=previous,
    )
    relation = result["branches"][0]["previous_active_relation"]

    assert relation["axis_angle_deg"] == 0.0
    assert np.isclose(relation["centroid_distance_m"], 0.004)
    assert np.isclose(relation["endpoint_distance_m"], 0.004)
    assert relation["match_cost"] is not None
    assert relation["geometrically_similar"]


def test_unsplit_parent_mixture_is_reported_as_mixed_structure():
    wrist = np.asarray((0.0, 0.0, 0.70))
    first = _straight((0.03, 0.0, 0.70), (0.09, 0.0, 0.70), 50)
    second = _straight((0.09, 0.002, 0.70), (0.09, 0.09, 0.70), 60)
    points = np.vstack((first, second))
    descriptor = _descriptor(
        (1.0, 0.0, 0.0), (0.07, 0.03, 0.70),
        (0.03, 0.0, 0.70), (0.09, 0.09, 0.70), length=0.15,
    )
    branch = _branch(
        0, 2, descriptor, valid=False, reason="BRANCH_NOT_LINEAR",
        local={"interpretation": "A_TRUE_NONLINEAR_OR_MIXED_GEOMETRY"},
    )

    result = analyze_branch_parent_topology_shadow(
        wrist, points, {2: np.arange(110)}, {0: np.arange(110)}, (branch,)
    )
    diagnostic = result["branches"][0]

    assert diagnostic["classification"] == "MIXED_STRUCTURE"
    assert diagnostic["origin_hypothesis"] == "UNSPLIT_PARENT_MIXTURE"
    assert "INTERNAL_DIRECTION_CONFLICT" in diagnostic["classification_reasons"]


def test_shadow_does_not_mutate_selection_tracker_scores_or_inputs():
    wrist = np.asarray((0.0, 0.0, 0.70))
    points = _straight((0.03, 0.0, 0.70), (0.15, 0.0, 0.70))
    descriptor = _descriptor(
        (1.0, 0.0, 0.0), (0.09, 0.0, 0.70),
        (0.03, 0.0, 0.70), (0.15, 0.0, 0.70),
    )
    branches = (_branch(0, 0, descriptor),)
    pipeline_state = SimpleNamespace(
        selected_branch=0,
        active_forearm_track=None,
        tracker={"active_track": 8},
        score_v2=0.72,
        topology_score=0.64,
        depth_surface_score=0.81,
    )
    branches_before = deepcopy(branches)
    state_before = deepcopy(pipeline_state)
    points_before = points.copy()

    result = analyze_branch_parent_topology_shadow(
        wrist, points, {0: np.arange(80)}, {0: np.arange(80)}, branches,
        previous_active_descriptor=descriptor,
    )

    assert result["shadow_only"]
    assert pipeline_state == state_before
    assert branches == branches_before
    assert np.array_equal(points, points_before)
