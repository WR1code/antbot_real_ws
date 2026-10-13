from copy import deepcopy
from types import SimpleNamespace

import numpy as np

from meridian_hand_vision.branch_local_recovery_shadow import (
    BranchLocalRecoveryShadowState,
    analyze_branch_local_recovery_shadow,
    update_branch_local_recovery_shadow,
)


def _tube(center_tangents, *, radius=0.008, points_per_ring=16, seed=7):
    rng = np.random.default_rng(seed)
    points = []
    for center, tangent in center_tangents:
        tangent = np.asarray(tangent, dtype=np.float64)
        tangent /= np.linalg.norm(tangent)
        helper = np.asarray((0.0, 0.0, 1.0))
        if abs(float(np.dot(tangent, helper))) > 0.9:
            helper = np.asarray((0.0, 1.0, 0.0))
        first = np.cross(tangent, helper)
        first /= np.linalg.norm(first)
        second = np.cross(tangent, first)
        for angle in np.linspace(0.0, 2.0 * np.pi, points_per_ring, endpoint=False):
            noise = rng.normal(0.0, 0.0003, 3)
            points.append(
                center
                + radius * (np.cos(angle) * first + np.sin(angle) * second)
                + noise
            )
    return np.asarray(points)


def _analyze(points):
    eigenvalues, eigenvectors = np.linalg.eigh(np.cov(points.T))
    axis = eigenvectors[:, -1]
    return analyze_branch_local_recovery_shadow(
        3,
        points,
        axis,
        np.asarray((0.0, 0.0, 0.70)),
        original_reason="BRANCH_NOT_LINEAR",
        global_linearity=float(eigenvalues[-1] / max(eigenvalues[-2], 1e-12)),
    )


def test_straight_branch_recovers_stable_local_segment():
    centers = [
        (np.asarray((x, 0.0, 0.70)), np.asarray((1.0, 0.0, 0.0)))
        for x in np.linspace(0.03, 0.17, 30)
    ]
    result = _analyze(_tube(centers))

    assert result["recovery_success"]
    assert result["recovered_length_m"] >= 0.060
    assert result["recovered_rms"] <= 0.006
    assert result["recovered_section_count"] >= 4
    assert result["interpretation"] == "B_GLOBAL_LINEARITY_GATE_TOO_STRICT"


def test_globally_non_linear_bent_branch_recovers_local_segment():
    bend_angle = np.radians(105.0)
    first_direction = np.asarray((1.0, 0.0, 0.0))
    second_direction = np.asarray((
        np.cos(bend_angle), np.sin(bend_angle), 0.0
    ))
    centers = [
        (np.asarray((x, 0.0, 0.70)), first_direction)
        for x in np.linspace(0.03, 0.15, 35)
    ]
    bend_origin = np.asarray((0.15, 0.0, 0.70))
    centers.extend(
        (bend_origin + distance * second_direction, second_direction)
        for distance in np.linspace(0.003, 0.12, 30)
    )
    points = _tube(centers, radius=0.006, seed=12)
    eigenvalues = np.linalg.eigvalsh(np.cov(points.T))
    assert eigenvalues[-1] / eigenvalues[-2] < 2.5

    result = _analyze(points)

    assert result["recovery_success"]
    assert result["stable_window_count"] >= 1
    assert result["recovered_length_m"] >= 0.060


def test_palm_like_mixed_geometry_has_no_stable_sixty_mm_segment():
    centers = []
    for x in np.linspace(0.03, 0.17, 30):
        y = 0.025 * np.sin((x - 0.03) / 0.012 * np.pi)
        derivative = 0.025 * np.pi / 0.012 * np.cos(
            (x - 0.03) / 0.012 * np.pi
        )
        centers.append((
            np.asarray((x, y, 0.70)),
            np.asarray((1.0, derivative, 0.0)),
        ))
    result = _analyze(_tube(centers, radius=0.006, seed=13))

    assert not result["recovery_success"]
    assert result["stable_window_count"] == 0
    assert result["interpretation"] == "A_TRUE_NONLINEAR_OR_MIXED_GEOMETRY"


def test_insufficient_cloud_returns_insufficient_geometry():
    points = np.column_stack((
        np.linspace(0.03, 0.05, 30),
        np.zeros(30),
        np.full(30, 0.70),
    ))
    result = analyze_branch_local_recovery_shadow(
        1,
        points,
        np.asarray((1.0, 0.0, 0.0)),
        np.asarray((0.0, 0.0, 0.70)),
        original_reason="BRANCH_NOT_LINEAR",
        global_linearity=1.5,
    )

    assert not result["recovery_success"]
    assert result["interpretation"] == "INSUFFICIENT_GEOMETRY"
    assert result["window_count"] == 0


def test_temporal_shadow_does_not_mutate_estimate_tracker_or_scores():
    recovery = {
        "branch_id": 0,
        "original_failure_reason": "BRANCH_NOT_LINEAR",
        "recovery_success": True,
        "recovered_length_m": 0.085,
        "recovered_axis": (1.0, 0.0, 0.0),
        "recovered_rms": 0.003,
        "recovered_end_point": (0.12, 0.0, 0.70),
        "interpretation": "B_GLOBAL_LINEARITY_GATE_TOO_STRICT",
    }
    branch = SimpleNamespace(local_recovery_shadow=recovery)
    estimate = SimpleNamespace(branches=(branch,))
    descriptor = SimpleNamespace(
        axis=(1.0, 0.0, 0.0),
        effective_length_m=0.090,
        far_endpoint_m=(0.13, 0.0, 0.70),
    )
    tracked_branch = SimpleNamespace(
        track_id=8,
        features=SimpleNamespace(descriptor=descriptor),
    )
    tracking = SimpleNamespace(
        active_track_id=8,
        branches=(tracked_branch,),
        score_v2_shadow={"best_score": 0.7},
        topology_score=0.6,
        depth_surface_score=0.8,
    )
    estimate_before = deepcopy(estimate)
    tracking_before = deepcopy(tracking)
    state = BranchLocalRecoveryShadowState()

    result = update_branch_local_recovery_shadow(state, estimate, tracking)

    assert result["shadow_only"]
    assert result["best_candidate_id"] == 0
    assert result["track_statistics"]["recovery_success_count"] == 1
    assert estimate == estimate_before
    assert tracking == tracking_before
