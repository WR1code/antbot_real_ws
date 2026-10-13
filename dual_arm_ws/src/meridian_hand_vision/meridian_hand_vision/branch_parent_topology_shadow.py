"""Shadow-only parent topology diagnostics for wrist geometry branches.

This module deliberately returns JSON-serializable dictionaries and never
feeds a value back into branch generation, selection, tracking, or scoring.
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np


def _vector(value: Any) -> np.ndarray | None:
    if value is None:
        return None
    if hasattr(value, "vector"):
        value = value.vector
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (3,) or not np.all(np.isfinite(result)):
        return None
    norm = float(np.linalg.norm(result))
    return result / norm if norm > 1e-12 else None


def _tuple(value: np.ndarray | None) -> tuple[float, float, float] | None:
    if value is None:
        return None
    return tuple(float(item) for item in value)


def _angle(first: np.ndarray | None, second: np.ndarray | None, *, unsigned=False):
    if first is None or second is None:
        return None
    cosine = float(np.dot(first, second))
    if unsigned:
        cosine = abs(cosine)
    return float(np.degrees(np.arccos(np.clip(cosine, -1.0, 1.0))))


def _descriptor(branch):
    features = getattr(branch, "features", None)
    return getattr(features, "descriptor", None) if features is not None else None


def _branch_length(branch) -> float | None:
    value = getattr(branch, "axis_length_m", None)
    features = getattr(branch, "features", None)
    if value is None and features is not None:
        value = getattr(features, "effective_length_m", None)
    if value is None:
        value = getattr(branch, "span_m", None)
    return float(value) if value is not None and math.isfinite(float(value)) else None


def _relation(previous, current) -> dict:
    empty = {
        "previous_active_available": previous is not None,
        "axis_angle_deg": None,
        "centroid_distance_m": None,
        "endpoint_distance_m": None,
        "length_delta": None,
        "length_relative_delta": None,
        "match_cost": None,
        "match_reason": "NO_PREVIOUS_ACTIVE_REFERENCE" if previous is None else "MATCH_DESCRIPTOR_MISSING",
        "geometrically_similar": False,
    }
    if previous is None or current is None:
        return empty
    previous_axis = _vector(getattr(previous, "axis", None))
    current_axis = _vector(getattr(current, "axis", None))
    if previous_axis is None or current_axis is None:
        return empty
    axis_angle = _angle(previous_axis, current_axis, unsigned=True)
    centroid_distance = float(np.linalg.norm(
        np.asarray(previous.centroid_m) - np.asarray(current.centroid_m)
    ))
    endpoint_distance = float(np.linalg.norm(
        np.asarray(previous.far_endpoint_m) - np.asarray(current.far_endpoint_m)
    ))
    length_delta = abs(
        float(previous.effective_length_m) - float(current.effective_length_m)
    ) / max(
        float(previous.effective_length_m), float(current.effective_length_m), 1e-12
    )
    width_delta = 0.0
    if previous.median_width_m is not None and current.median_width_m is not None:
        width_delta = abs(
            float(previous.median_width_m) - float(current.median_width_m)
        ) / max(
            float(previous.median_width_m), float(current.median_width_m), 1e-12
        )
    reason = "MATCH_OK"
    if axis_angle > 35.0:
        reason = "MATCH_AXIS_GATE"
    elif centroid_distance > 0.10:
        reason = "MATCH_CENTROID_GATE"
    elif endpoint_distance > 0.12:
        reason = "MATCH_ENDPOINT_GATE"
    elif length_delta > 0.70:
        reason = "MATCH_LENGTH_GATE"
    cost = None
    if reason == "MATCH_OK":
        cost = float(
            0.40 * axis_angle / 35.0
            + 0.25 * centroid_distance / 0.10
            + 0.20 * endpoint_distance / 0.12
            + 0.10 * length_delta / 0.70
            + 0.05 * min(width_delta, 1.0)
        )
        if cost > 1.0:
            reason = "MATCH_TOTAL_COST_GATE"
    similar = cost is not None and cost <= 1.0
    return {
        "previous_active_available": True,
        "axis_angle_deg": axis_angle,
        "centroid_distance_m": centroid_distance,
        "endpoint_distance_m": endpoint_distance,
        "length_delta": length_delta,
        "length_relative_delta": length_delta,
        "match_cost": cost,
        "match_reason": reason,
        "geometrically_similar": similar,
    }


def _root_and_direction(points, indices, wrist_point, branch) -> dict:
    branch_points = points[indices]
    radial = np.linalg.norm(branch_points - wrist_point, axis=1)
    minimum = float(np.min(radial))
    root_points = branch_points[radial <= minimum + 0.010]
    root_point = np.median(root_points, axis=0)
    root_residual = np.linalg.norm(root_points - root_point, axis=1)
    root_spread = float(np.percentile(root_residual, 90.0))
    wrist_direction = _vector(root_point - wrist_point)
    if wrist_direction is not None:
        root_delta = root_points - root_point
        root_axial = root_delta @ wrist_direction
        root_transverse = root_delta - np.outer(root_axial, wrist_direction)
        root_width = 2.0 * float(np.percentile(
            np.linalg.norm(root_transverse, axis=1), 90.0
        ))
    else:
        root_width = 2.0 * root_spread

    far_threshold = float(np.percentile(radial, 90.0))
    distal_points = branch_points[radial >= far_threshold]
    distal_center = np.median(distal_points, axis=0)
    distal_direction = _vector(distal_center - root_point)
    axis_value = getattr(branch, "axis", None)
    axis_source = "RANSAC"
    if axis_value is None:
        axis_value = getattr(branch, "pca_direction", None)
        axis_source = "PCA" if axis_value is not None else "NONE"
    axis = _vector(axis_value)
    return {
        "root": {
            "root_point_m": _tuple(root_point),
            "root_to_wrist_distance_m": float(np.linalg.norm(root_point - wrist_point)),
            "root_support_count": int(root_points.shape[0]),
            "root_spread_m": root_spread,
            "root_width_m": root_width,
            "root_method": "NEAR_SHELL_10MM",
        },
        "directions": {
            "axis": _tuple(axis),
            "axis_source": axis_source,
            "wrist_relative_direction": _tuple(wrist_direction),
            "distal_direction": _tuple(distal_direction),
            "distal_center_m": _tuple(distal_center),
            "axis_distal_bend_deg": _angle(axis, distal_direction),
            "wrist_distal_bend_deg": _angle(wrist_direction, distal_direction),
        },
        "_axis": axis,
        "_distal": distal_direction,
        "_root_point": root_point,
    }


def analyze_branch_parent_topology_shadow(
    wrist_point,
    points,
    parent_component_indices,
    branch_indices,
    branches,
    previous_active_descriptor=None,
) -> dict:
    """Describe existing branch partitions without changing them."""
    wrist = np.asarray(wrist_point, dtype=np.float64)
    cloud = np.asarray(points, dtype=np.float64)
    branch_by_id = {int(branch.branch_id): branch for branch in branches}
    working = {}
    for branch_id, indices in branch_indices.items():
        branch = branch_by_id.get(int(branch_id))
        index_array = np.asarray(indices, dtype=np.int64)
        if branch is None or index_array.size == 0:
            continue
        item = _root_and_direction(cloud, index_array, wrist, branch)
        item["relation"] = _relation(previous_active_descriptor, _descriptor(branch))
        working[int(branch_id)] = item

    parent_groups = {}
    for branch in branches:
        parent_groups.setdefault(int(branch.parent_component_id), []).append(
            int(branch.branch_id)
        )
    active_like_ids = {
        branch_id for branch_id, item in working.items()
        if item["relation"]["geometrically_similar"]
    }

    output = []
    for branch_id in sorted(working):
        branch = branch_by_id[branch_id]
        item = working[branch_id]
        parent_id = int(branch.parent_component_id)
        sibling_ids = [value for value in parent_groups[parent_id] if value != branch_id]
        sibling_items = []
        for sibling_id in sibling_ids:
            sibling = branch_by_id[sibling_id]
            sibling_work = working.get(sibling_id)
            if sibling_work is None:
                continue
            sibling_items.append({
                "branch_id": sibling_id,
                "length_m": _branch_length(sibling),
                "valid": bool(sibling.valid),
                "reason": sibling.reason,
                "axis_angle_deg": _angle(item["_axis"], sibling_work["_axis"], unsigned=True),
                "root_distance_m": float(np.linalg.norm(
                    item["_root_point"] - sibling_work["_root_point"]
                )),
                "distal_angle_deg": _angle(item["_distal"], sibling_work["_distal"]),
            })

        relation = item["relation"]
        sibling_matches_active = any(value in active_like_ids for value in sibling_ids)
        local_recovery = getattr(branch, "local_recovery_shadow", None) or {}
        local_mixed = local_recovery.get("interpretation") == (
            "A_TRUE_NONLINEAR_OR_MIXED_GEOMETRY"
        )
        axis_bend = item["directions"]["axis_distal_bend_deg"]
        broad_root = item["root"]["root_width_m"] > 0.050
        internal_conflict = local_mixed or (
            axis_bend is not None and axis_bend > 25.0
        )

        if relation["geometrically_similar"]:
            classification = "FOREARM_LIKE"
            origin = "ACTIVE_FOREARM_FRAGMENT" if not branch.valid else "PARENT_COMPONENT_AMBIGUOUS"
            reasons = ["MATCHES_PREVIOUS_ACTIVE"]
            confidence = 0.90 if branch.valid else 0.75
        elif sibling_matches_active:
            classification = "PALM_LIKE"
            origin = "SIBLING_OF_ACTIVE_FOREARM"
            reasons = ["SIBLING_MATCHES_PREVIOUS_ACTIVE", "CURRENT_BRANCH_GEOMETRY_MISMATCH"]
            confidence = 0.90
        elif internal_conflict or (len(sibling_ids) == 0 and branch.reason == "BRANCH_NOT_LINEAR"):
            classification = "MIXED_STRUCTURE"
            origin = "UNSPLIT_PARENT_MIXTURE"
            reasons = ["INTERNAL_DIRECTION_CONFLICT"]
            if broad_root:
                reasons.append("BROAD_ROOT")
            confidence = 0.85 if local_mixed else 0.70
        elif previous_active_descriptor is None:
            classification = "MIXED_STRUCTURE"
            origin = "NO_PREVIOUS_ACTIVE_REFERENCE"
            reasons = ["NO_PREVIOUS_ACTIVE_REFERENCE"]
            confidence = 0.30
        elif active_like_ids:
            classification = "PALM_LIKE"
            origin = "ISOLATED_NONFOREARM_COMPONENT"
            reasons = ["ACTIVE_FOREARM_EXISTS_IN_OTHER_PARENT"]
            confidence = 0.70
        else:
            classification = "MIXED_STRUCTURE"
            origin = "PARENT_COMPONENT_AMBIGUOUS"
            reasons = ["NO_MATCHED_FOREARM_SIBLING"]
            confidence = 0.40

        parent_indices = np.asarray(parent_component_indices.get(parent_id, ()), dtype=np.int64)
        angles = [value["axis_angle_deg"] for value in sibling_items if value["axis_angle_deg"] is not None]
        output.append({
            "branch_id": branch_id,
            "parent_component_id": parent_id,
            "valid": bool(branch.valid),
            "original_reason": branch.reason,
            "root": item["root"],
            "directions": item["directions"],
            "siblings": {
                "sibling_count": len(sibling_items),
                "sibling_lengths_m": [value["length_m"] for value in sibling_items],
                "sibling_axis_angles_deg": angles,
                "sibling_axis_angle_min_deg": min(angles) if angles else None,
                "sibling_axis_angle_median_deg": float(np.median(angles)) if angles else None,
                "sibling_axis_angle_max_deg": max(angles) if angles else None,
                "items": sibling_items,
            },
            "parent_point_count": int(parent_indices.size),
            "branch_point_fraction": float(
                len(branch_indices[branch_id]) / max(1, parent_indices.size)
            ),
            "previous_active_relation": relation,
            "classification": classification,
            "classification_confidence": confidence,
            "classification_reasons": reasons,
            "origin_hypothesis": origin,
        })

    distribution = {
        label: sum(item["classification"] == label for item in output)
        for label in ("FOREARM_LIKE", "PALM_LIKE", "MIXED_STRUCTURE")
    }
    return {
        "candidate_count": len(output),
        "parent_component_count": len(parent_component_indices),
        "branches": output,
        "classification_distribution": distribution,
        "previous_active_available": previous_active_descriptor is not None,
        "shadow_only": True,
    }
