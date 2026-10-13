from copy import deepcopy
from types import SimpleNamespace

import numpy as np

from meridian_hand_vision.parent_component_split_shadow import (
    analyze_parent_component_split_shadow,
)


WRIST = np.asarray((0.0, 0.0, 0.70))


def _descriptor(axis=(1.0, 0.0, 0.0), endpoint=(0.16, 0.0, 0.70)):
    return SimpleNamespace(
        axis=axis,
        centroid_m=(0.095, 0.0, 0.70),
        near_endpoint_m=(0.03, 0.0, 0.70),
        far_endpoint_m=endpoint,
        effective_length_m=0.13,
        median_width_m=0.04,
    )


def _trigger(parent_id=0, branch_id=0):
    return {
        "branches": [{
            "branch_id": branch_id,
            "parent_component_id": parent_id,
            "classification": "MIXED_STRUCTURE",
            "origin_hypothesis": "UNSPLIT_PARENT_MIXTURE",
        }]
    }


def _mixed_forearm_and_palm():
    count = 90
    forearm = np.column_stack((
        np.linspace(0.03, 0.17, count),
        np.zeros(count),
        np.full(count, 0.70),
    ))
    palm = np.column_stack((
        np.full(count, 0.03),
        np.linspace(0.0015, 0.14, count),
        np.full(count, 0.70),
    ))
    pixels = np.vstack((
        np.column_stack((np.arange(count), np.full(count, 10))),
        np.column_stack((np.zeros(count, dtype=int), 11 + np.arange(count))),
    ))
    return np.vstack((forearm, palm)), pixels


def test_forearm_and_palm_mixture_recovers_forearm_candidate():
    points, pixels = _mixed_forearm_and_palm()
    result = analyze_parent_component_split_shadow(
        WRIST, points, pixels, {0: np.arange(points.shape[0])}, _trigger(),
        previous_active_descriptor=_descriptor(), graph_stride=1,
    )
    parent = result["parents"][0]

    assert parent["direction_cluster_count"] >= 2
    assert parent["classification"] == "FOREARM_RECOVERABLE"
    assert parent["recoverable_candidate_ids"]
    recovered = [
        item for item in parent["split_candidates"]
        if item["candidate_id"] in parent["recoverable_candidate_ids"]
    ][0]
    assert recovered["length_m"] >= 0.060
    assert recovered["rms_m"] <= 0.006
    assert recovered["previous_active_angle_deg"] < 15.0


def test_single_forearm_parent_is_recoverable_as_one_direction():
    count = 100
    points = np.column_stack((
        np.linspace(0.03, 0.17, count), np.zeros(count), np.full(count, 0.70)
    ))
    pixels = np.column_stack((np.arange(count), np.zeros(count, dtype=int)))
    result = analyze_parent_component_split_shadow(
        WRIST, points, pixels, {3: np.arange(count)}, _trigger(3),
        previous_active_descriptor=_descriptor(), graph_stride=1,
    )
    parent = result["parents"][0]

    assert parent["direction_cluster_count"] == 1
    assert parent["classification"] == "FOREARM_RECOVERABLE"


def test_multidirectional_short_structure_remains_unresolved():
    angles = np.linspace(-1.2, 1.2, 140)
    radius = np.linspace(0.035, 0.070, 140)
    points = np.column_stack((
        radius * np.cos(angles), radius * np.sin(angles), np.full(140, 0.70)
    ))
    pixels = np.column_stack((np.arange(140), np.zeros(140, dtype=int)))
    result = analyze_parent_component_split_shadow(
        WRIST, points, pixels, {0: np.arange(140)}, _trigger(),
        previous_active_descriptor=_descriptor(), graph_stride=1,
    )
    parent = result["parents"][0]

    assert parent["classification"] in {"MIXED_UNRESOLVED", "INSUFFICIENT"}
    assert not parent["recoverable_candidate_ids"]


def test_shadow_does_not_change_existing_outputs_or_inputs():
    points, pixels = _mixed_forearm_and_palm()
    pipeline = SimpleNamespace(
        selected_branch=0,
        active_forearm_track=None,
        tracker={"active": 4},
        score_v2=0.8,
        topology_score=0.7,
        depth_surface_score=0.9,
    )
    before = deepcopy(pipeline)
    points_before = points.copy()
    pixels_before = pixels.copy()

    result = analyze_parent_component_split_shadow(
        WRIST, points, pixels, {0: np.arange(points.shape[0])}, _trigger(),
        previous_active_descriptor=_descriptor(), graph_stride=1,
    )

    assert result["shadow_only"]
    assert pipeline == before
    assert np.array_equal(points, points_before)
    assert np.array_equal(pixels, pixels_before)
