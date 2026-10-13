from __future__ import annotations

from dataclasses import dataclass
import math

from .pulse_region import (
    WristGeometryBranchDescriptor,
    _wrist_branch_descriptor_match_diagnostic,
)


@dataclass
class BranchCandidateShadowState:
    """Previous visible forearm geometry used only for shadow comparison."""

    frame_index: int = 0
    reference_descriptor: WristGeometryBranchDescriptor | None = None
    reference_frame_index: int | None = None
    reference_branch_id: int | None = None
    reference_track_id: int | None = None
    has_seen_active_track: bool = False


def _finite_or_none(value):
    if value is None:
        return None
    number = float(value)
    return number if math.isfinite(number) else None


def _rejection_group(reason: str, valid: bool) -> str:
    if valid or reason == "NONE":
        return "ACCEPTED"
    if reason == "BRANCH_NOT_LINEAR":
        return "BRANCH_EXISTS_BUT_NOT_LINEAR"
    if reason in {"BRANCH_TOO_SHORT", "TOO_SHORT"}:
        return "BRANCH_TOO_SHORT"
    if reason in {
        "TOO_FEW_SECTIONS",
        "INSUFFICIENT_SECTIONS",
        "RANSAC_INSUFFICIENT_SECTIONS",
    }:
        return "BRANCH_EXISTS_LOW_SECTION"
    if reason in {"RMS_TOO_HIGH", "RANSAC_RMS_HIGH"}:
        return "BRANCH_EXISTS_RMS_GATE_FAILED"
    if reason in {"INLIER_RATIO_LOW", "INLIER_CONTINUITY_LOW"}:
        return "BRANCH_EXISTS_INLIER_GATE_FAILED"
    if reason in {"BRANCH_PCA_DEGENERATE", "RANSAC_REFINE_DEGENERATE"}:
        return "BRANCH_EXISTS_DEGENERATE"
    if reason == "RANSAC_FAILED":
        return "BRANCH_EXISTS_RANSAC_FAILED"
    return "BRANCH_EXISTS_OTHER_REJECTION"


def _rejection_stage(reason: str, valid: bool) -> int:
    """Diagnostic-only progress through the existing branch pipeline."""
    if valid or reason == "NONE":
        return 6
    if reason in {"RMS_TOO_HIGH", "TOO_SHORT", "INLIER_RATIO_LOW",
                  "INLIER_CONTINUITY_LOW"}:
        return 5
    if reason in {"RANSAC_FAILED", "RANSAC_REFINE_DEGENERATE"}:
        return 4
    if reason in {"TOO_FEW_SECTIONS", "INSUFFICIENT_SECTIONS",
                  "RANSAC_INSUFFICIENT_SECTIONS"}:
        return 3
    if reason == "BRANCH_NOT_LINEAR":
        return 2
    if reason == "BRANCH_PCA_DEGENERATE":
        return 1
    return 1


def _candidate_diagnostic(branch) -> dict:
    features = branch.features
    effective_length = (
        _finite_or_none(branch.axis_length_m)
        or (
            _finite_or_none(features.effective_length_m)
            if features is not None else None
        )
        or _finite_or_none(branch.span_m)
    )
    continuous_span = (
        _finite_or_none(features.continuous_span_m)
        if features is not None else None
    )
    valid_sections = int(branch.valid_section_count)
    inlier_sections = int(branch.inlier_section_count)
    return {
        "branch_id": int(branch.branch_id),
        "point_count": int(branch.point_count),
        "length": effective_length,
        "axis_length": _finite_or_none(branch.axis_length_m),
        "component_span": _finite_or_none(branch.span_m),
        "continuous_span": continuous_span,
        "pca_linearity": _finite_or_none(branch.pca_linearity),
        "rms": _finite_or_none(branch.rms_m),
        "section_count": valid_sections,
        "total_section_count": len(branch.sections),
        "inlier_count": inlier_sections,
        "inlier_ratio": _finite_or_none(branch.inlier_ratio),
        "valid": bool(branch.valid),
        "original_reason": branch.reason,
        "rejection_reason": _rejection_group(branch.reason, branch.valid),
        "rejection_stage": _rejection_stage(branch.reason, branch.valid),
        "axis": branch.axis.vector if branch.axis is not None else (
            branch.pca_direction.vector
            if branch.pca_direction is not None else None
        ),
    }


def _best_rejected(branches) -> object | None:
    rejected = [branch for branch in branches if not branch.valid]
    if not rejected:
        return None

    def rank(branch):
        diagnostic = _candidate_diagnostic(branch)
        rms = diagnostic["rms"]
        return (
            diagnostic["rejection_stage"],
            diagnostic["inlier_count"],
            diagnostic["section_count"],
            diagnostic["length"] or 0.0,
            diagnostic["pca_linearity"] or 0.0,
            -(rms if rms is not None else math.inf),
            -diagnostic["branch_id"],
        )

    return max(rejected, key=rank)


def _association_diagnostics(state: BranchCandidateShadowState, branches) -> dict:
    reference = state.reference_descriptor
    if reference is None:
        return {
            "previous_reference_available": False,
            "previous_reference_frame": None,
            "previous_reference_branch_id": None,
            "previous_reference_track_id": None,
            "candidate_matches": [],
            "similar_candidate_count": 0,
            "best_geometric_match": None,
            "assessment": "NO_PREVIOUS_REFERENCE",
        }

    matches = []
    for branch in branches:
        descriptor = branch.features.descriptor if branch.features is not None else None
        if descriptor is None:
            matches.append({
                "branch_id": int(branch.branch_id),
                "valid": bool(branch.valid),
                "rejection_reason": _rejection_group(branch.reason, branch.valid),
                "geometrically_similar": False,
                "match_cost": None,
                "match_reason": "MATCH_DESCRIPTOR_MISSING",
            })
            continue
        cost, reason, details = _wrist_branch_descriptor_match_diagnostic(
            reference, descriptor
        )
        matches.append({
            "branch_id": int(branch.branch_id),
            "valid": bool(branch.valid),
            "rejection_reason": _rejection_group(branch.reason, branch.valid),
            "geometrically_similar": cost is not None and cost <= 1.0,
            "match_cost": cost,
            "match_reason": reason,
            **details,
        })

    similar = [item for item in matches if item["geometrically_similar"]]
    best = min(
        similar, key=lambda item: (item["match_cost"], item["branch_id"])
    ) if similar else None
    if not branches:
        assessment = "A_BRANCH_DISAPPEARED"
    elif not similar:
        assessment = "A_NO_GEOMETRICALLY_SIMILAR_CANDIDATE"
    elif not any(item["valid"] for item in similar):
        assessment = "B_SIMILAR_CANDIDATE_REJECTED_BY_QUALITY"
    else:
        assessment = "GEOMETRICALLY_SIMILAR_VALID_CANDIDATE_EXISTS"
    return {
        "previous_reference_available": True,
        "previous_reference_frame": state.reference_frame_index,
        "previous_reference_branch_id": state.reference_branch_id,
        "previous_reference_track_id": state.reference_track_id,
        "candidate_matches": matches,
        "similar_candidate_count": len(similar),
        "best_geometric_match": best,
        "assessment": assessment,
    }


def _current_reference_branch(estimate, tracking):
    by_id = {branch.branch_id: branch for branch in estimate.branches}
    active_track_id = tracking.active_track_id
    if active_track_id is not None:
        tracked = next(
            (
                item for item in tracking.branches
                if item.track_id == active_track_id
            ),
            None,
        )
        if tracked is not None:
            return by_id.get(tracked.branch_id), active_track_id
        return None, active_track_id
    if estimate.selected_branch is not None:
        return by_id.get(estimate.selected_branch), None
    return None, None


def update_branch_candidate_shadow(
    state: BranchCandidateShadowState,
    estimate,
    tracking,
    mask_estimate=None,
) -> dict:
    """Inspect existing candidates without changing estimation or tracking."""
    state.frame_index += 1
    branches = tuple(estimate.branches)
    candidate_details = [_candidate_diagnostic(branch) for branch in branches]
    valid = [branch for branch in branches if branch.valid]
    rejected = [branch for branch in branches if not branch.valid]
    best_rejected = _best_rejected(branches)
    best_rejected_details = (
        _candidate_diagnostic(best_rejected)
        if best_rejected is not None else None
    )
    association = _association_diagnostics(state, branches)

    selected_id = estimate.selected_branch
    similar_ids = {
        item["branch_id"] for item in association["candidate_matches"]
        if item["geometrically_similar"]
    }
    if not branches:
        hypothesis = "A_BRANCH_REALLY_DISAPPEARED"
    elif not valid:
        hypothesis = "B_CANDIDATE_REJECTED_BY_QUALITY"
    elif selected_id is None:
        hypothesis = "C_VALID_CANDIDATE_NOT_SELECTED"
    elif similar_ids and selected_id not in similar_ids:
        hypothesis = "C_SELECTED_BRANCH_NOT_GEOMETRIC_MATCH"
    else:
        hypothesis = "NONE"

    connected_points = (
        int(getattr(mask_estimate, "connected_point_count", 0))
        if mask_estimate is not None else None
    )
    result = {
        "frame_index": state.frame_index,
        "estimator_reason": estimate.reason,
        "connected_point_count": connected_points,
        "candidate_count": len(branches),
        "valid_candidate_count": len(valid),
        "rejected_candidate_count": len(rejected),
        "selected_branch_id": selected_id,
        "candidates": candidate_details,
        "best_rejected_candidate": best_rejected_details,
        "best_candidate_reason": (
            best_rejected_details["rejection_reason"]
            if best_rejected_details is not None
            else "NO_GEOMETRIC_CANDIDATE" if not branches else "NONE"
        ),
        "failure_hypothesis": hypothesis,
        "association": association,
        "shadow_only": True,
    }

    current, current_track_id = _current_reference_branch(estimate, tracking)
    if tracking.active_track_id is not None:
        state.has_seen_active_track = True
    may_bootstrap = not state.has_seen_active_track
    if (
        current is not None
        and current.features is not None
        and (current_track_id is not None or may_bootstrap)
    ):
        state.reference_descriptor = current.features.descriptor
        state.reference_frame_index = state.frame_index
        state.reference_branch_id = current.branch_id
        state.reference_track_id = current_track_id
    return result
