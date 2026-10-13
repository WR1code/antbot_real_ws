from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import math

import numpy as np


def _unsigned_angle_deg(first, second) -> float | None:
    if first is None or second is None:
        return None
    first = np.asarray(first, dtype=np.float64)
    second = np.asarray(second, dtype=np.float64)
    denominator = float(np.linalg.norm(first) * np.linalg.norm(second))
    if denominator <= 1e-12:
        return None
    cosine = abs(float(np.dot(first, second))) / denominator
    return float(np.degrees(np.arccos(np.clip(cosine, 0.0, 1.0))))


def _fit_axis(points: np.ndarray, reference=None):
    if points.shape[0] < 2:
        return None, None, None
    origin = np.mean(points, axis=0)
    centered = points - origin
    covariance = centered.T @ centered / max(1, points.shape[0] - 1)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    if not np.all(np.isfinite(eigenvalues)) or eigenvalues[-1] <= 1e-12:
        return None, None, None
    axis = eigenvectors[:, -1]
    if reference is not None and float(np.dot(axis, reference)) < 0.0:
        axis = -axis
    return origin, axis, eigenvalues


def _axis_stability(section_centers: np.ndarray, reference) -> dict:
    if section_centers.shape[0] < 3:
        return {
            "axis_angle_median_deg": None,
            "axis_angle_p95_deg": None,
            "axis_angle_max_deg": None,
            "half_window_axis_angle_deg": None,
        }
    tangents = np.diff(section_centers, axis=0)
    tangent_norms = np.linalg.norm(tangents, axis=1)
    tangents = tangents[tangent_norms > 1e-9]
    tangent_norms = tangent_norms[tangent_norms > 1e-9]
    if tangents.shape[0] == 0:
        angles = np.empty(0, dtype=np.float64)
    else:
        tangents = tangents / tangent_norms[:, None]
        for index in range(tangents.shape[0]):
            if float(np.dot(tangents[index], reference)) < 0.0:
                tangents[index] = -tangents[index]
        angles = np.asarray([
            _unsigned_angle_deg(tangents[index - 1], tangents[index])
            for index in range(1, tangents.shape[0])
        ], dtype=np.float64)
    split = section_centers.shape[0] // 2
    first = section_centers[:split]
    second = section_centers[split:]
    _, first_axis, _ = _fit_axis(first, reference)
    _, second_axis, _ = _fit_axis(second, reference)
    half_angle = _unsigned_angle_deg(first_axis, second_axis)
    return {
        "axis_angle_median_deg": (
            float(np.median(angles)) if angles.size else 0.0
        ),
        "axis_angle_p95_deg": (
            float(np.percentile(angles, 95.0)) if angles.size else 0.0
        ),
        "axis_angle_max_deg": float(np.max(angles)) if angles.size else 0.0,
        "half_window_axis_angle_deg": half_angle,
    }


@dataclass
class _LocalWindow:
    start_m: float
    end_m: float
    axis: np.ndarray | None
    origin: np.ndarray | None
    centers: np.ndarray
    diagnostic: dict


def _analyze_window(
    points: np.ndarray,
    global_projection: np.ndarray,
    global_axis: np.ndarray,
    start_m: float,
    end_m: float,
) -> _LocalWindow:
    selected = (global_projection >= start_m) & (global_projection <= end_m)
    window_points = points[selected]
    reasons = []
    if window_points.shape[0] < 60:
        reasons.append("WINDOW_POINT_COUNT_LOW")
    if window_points.shape[0] >= 4:
        selected_projection = global_projection[selected]
        lower, upper = np.percentile(selected_projection, (2.0, 98.0))
        trimmed = (selected_projection >= lower) & (selected_projection <= upper)
        window_points = window_points[trimmed]
    origin, local_axis, eigenvalues = _fit_axis(window_points, global_axis)
    if local_axis is None:
        reasons.append("LOCAL_PCA_DEGENERATE")
        return _LocalWindow(start_m, end_m, None, None, np.empty((0, 3)), {
            "start_m": start_m,
            "end_m": end_m,
            "point_count": int(window_points.shape[0]),
            "local_length_m": None,
            "point_linearity": None,
            "total_section_count": 0,
            "valid_section_count": 0,
            "section_centers": [],
            "centerline_linearity": None,
            "local_rms": None,
            "median_residual": None,
            "p95_residual": None,
            "local_axis": None,
            "axis_angle_median_deg": None,
            "axis_angle_p95_deg": None,
            "axis_angle_max_deg": None,
            "half_window_axis_angle_deg": None,
            "window_valid": False,
            "rejection_reasons": reasons,
        })

    local_projection = (window_points - origin) @ local_axis
    lower, upper = np.percentile(local_projection, (2.0, 98.0))
    local_length = float(upper - lower)
    point_linearity = float(eigenvalues[-1] / max(eigenvalues[-2], 1e-12))
    total_sections = max(4, int(math.ceil(max(local_length, 1e-6) / 0.015)))
    edges = np.linspace(lower, upper, total_sections + 1)
    minimum_points = max(
        6, int(window_points.shape[0] / max(1, total_sections) * 0.15)
    )
    centers = []
    center_positions = []
    section_details = []
    for section_index in range(total_sections):
        in_section = (local_projection >= edges[section_index]) & (
            local_projection <= edges[section_index + 1]
            if section_index == total_sections - 1
            else local_projection < edges[section_index + 1]
        )
        section_points = window_points[in_section]
        valid = section_points.shape[0] >= minimum_points
        center = np.median(section_points, axis=0) if valid else None
        if center is not None:
            centers.append(center)
            center_positions.append(float(np.dot(center - origin, local_axis)))
        section_details.append({
            "section_index": section_index,
            "center": (
                tuple(float(value) for value in center)
                if center is not None else None
            ),
            "point_count": int(section_points.shape[0]),
            "valid": bool(valid),
        })
    center_array = (
        np.asarray(centers, dtype=np.float64)
        if centers else np.empty((0, 3), dtype=np.float64)
    )
    center_positions = np.asarray(center_positions, dtype=np.float64)
    center_origin, center_axis, center_eigenvalues = _fit_axis(
        center_array, local_axis
    )
    centerline_linearity = local_rms = median_residual = p95_residual = None
    if center_axis is not None:
        centered = center_array - center_origin
        axial = centered @ center_axis
        residuals = np.linalg.norm(
            centered - np.outer(axial, center_axis), axis=1
        )
        centerline_linearity = float(
            center_eigenvalues[-1] / max(center_eigenvalues[-2], 1e-12)
        )
        local_rms = float(math.sqrt(np.mean(np.square(residuals))))
        median_residual = float(np.median(residuals))
        p95_residual = float(np.percentile(residuals, 95.0))
    else:
        reasons.append("CENTERLINE_PCA_DEGENERATE")

    order = np.argsort(center_positions)
    center_positions = center_positions[order]
    center_array = center_array[order]
    stability = _axis_stability(
        center_array, center_axis if center_axis is not None else local_axis
    )
    valid_ratio = center_array.shape[0] / max(1, total_sections)
    center_coverage = (
        float((center_positions[-1] - center_positions[0]) / local_length)
        if center_positions.size >= 2 and local_length > 1e-9 else 0.0
    )
    maximum_gap = (
        float(np.max(np.diff(center_positions)))
        if center_positions.size >= 2 else math.inf
    )
    start_margin = (
        float((center_positions[0] - lower) / local_length)
        if center_positions.size and local_length > 1e-9 else math.inf
    )
    end_margin = (
        float((upper - center_positions[-1]) / local_length)
        if center_positions.size and local_length > 1e-9 else math.inf
    )
    if local_length < 0.060:
        reasons.append("LOCAL_LENGTH_LOW")
    if center_array.shape[0] < 4:
        reasons.append("VALID_SECTION_COUNT_LOW")
    if valid_ratio < 0.70 or center_coverage < 0.65:
        reasons.append("VALID_COVERAGE_LOW")
    if start_margin > 0.25 or end_margin > 0.25:
        reasons.append("SECTIONS_CONCENTRATED_AT_ONE_END")
    if maximum_gap > 0.025:
        reasons.append("SECTION_GAP_TOO_LARGE")
    if local_rms is None or local_rms > 0.006:
        reasons.append("LOCAL_RMS_HIGH")
    if centerline_linearity is None or centerline_linearity < 4.0:
        reasons.append("CENTERLINE_LINEARITY_LOW")
    if (
        stability["axis_angle_p95_deg"] is None
        or stability["axis_angle_p95_deg"] > 12.0
    ):
        reasons.append("LOCAL_AXIS_DISCONTINUOUS")
    if (
        stability["half_window_axis_angle_deg"] is None
        or stability["half_window_axis_angle_deg"] > 15.0
    ):
        reasons.append("HALF_WINDOW_AXIS_MISMATCH")
    diagnostic = {
        "start_m": start_m,
        "end_m": end_m,
        "point_count": int(window_points.shape[0]),
        "local_length_m": local_length,
        "point_linearity": point_linearity,
        "point_eigenvalues": tuple(
            float(value) for value in eigenvalues[::-1]
        ),
        "total_section_count": total_sections,
        "valid_section_count": int(center_array.shape[0]),
        "section_centers": section_details,
        "centerline_linearity": centerline_linearity,
        "local_rms": local_rms,
        "median_residual": median_residual,
        "p95_residual": p95_residual,
        "local_axis": tuple(float(value) for value in (
            center_axis if center_axis is not None else local_axis
        )),
        **stability,
        "valid_section_ratio": valid_ratio,
        "section_coverage": center_coverage,
        "section_start_margin_ratio": (
            start_margin if math.isfinite(start_margin) else None
        ),
        "section_end_margin_ratio": (
            end_margin if math.isfinite(end_margin) else None
        ),
        "maximum_section_gap_m": maximum_gap if math.isfinite(maximum_gap) else None,
        "window_valid": not reasons,
        "rejection_reasons": reasons,
    }
    return _LocalWindow(
        start_m, end_m,
        center_axis if center_axis is not None else local_axis,
        center_origin if center_origin is not None else origin,
        center_array,
        diagnostic,
    )


def _line_offset(first: _LocalWindow, second: _LocalWindow) -> float:
    delta = second.origin - first.origin
    average_axis = first.axis + second.axis
    if np.linalg.norm(average_axis) <= 1e-9:
        average_axis = first.axis
    average_axis = average_axis / max(float(np.linalg.norm(average_axis)), 1e-12)
    return float(np.linalg.norm(delta - np.dot(delta, average_axis) * average_axis))


def _merge_valid_windows(windows: list[_LocalWindow], global_axis: np.ndarray):
    valid = [window for window in windows if window.diagnostic["window_valid"]]
    valid.sort(key=lambda item: item.start_m)
    groups: list[list[_LocalWindow]] = []
    for window in valid:
        if not groups:
            groups.append([window])
            continue
        previous = groups[-1][-1]
        gap = window.start_m - previous.end_m
        angle = _unsigned_angle_deg(previous.axis, window.axis)
        offset = _line_offset(previous, window)
        if gap <= 0.015 and angle is not None and angle <= 12.0 and offset <= 0.008:
            groups[-1].append(window)
        else:
            groups.append([window])

    recovered = []
    for group in groups:
        centers = np.vstack([window.centers for window in group])
        ordering = np.argsort(centers @ global_axis)
        centers = centers[ordering]
        clusters = []
        for center in centers:
            if not clusters:
                clusters.append([center])
                continue
            previous = np.mean(clusters[-1], axis=0)
            if abs(float(np.dot(center - previous, global_axis))) <= 0.005:
                clusters[-1].append(center)
            else:
                clusters.append([center])
        unique_centers = np.asarray(
            [np.mean(cluster, axis=0) for cluster in clusters], dtype=np.float64
        )
        origin, axis, _ = _fit_axis(unique_centers, global_axis)
        if axis is None:
            continue
        centered = unique_centers - origin
        projection = centered @ axis
        residuals = np.linalg.norm(
            centered - np.outer(projection, axis), axis=1
        )
        length = float(np.percentile(projection, 98.0) - np.percentile(
            projection, 2.0
        ))
        rms = float(math.sqrt(np.mean(np.square(residuals))))
        stability = _axis_stability(unique_centers, axis)
        start_index = int(np.argmin(projection))
        end_index = int(np.argmax(projection))
        recovered.append({
            "recovered_length_m": length,
            "recovered_axis": tuple(float(value) for value in axis),
            "recovered_rms": rms,
            "recovered_section_count": int(unique_centers.shape[0]),
            "recovered_axis_p95_deg": stability["axis_angle_p95_deg"],
            "start_offset_from_wrist": float(group[0].start_m),
            "end_offset_from_wrist": float(group[-1].end_m),
            "recovered_start_point": tuple(
                float(value) for value in unique_centers[start_index]
            ),
            "recovered_end_point": tuple(
                float(value) for value in unique_centers[end_index]
            ),
            "window_count": len(group),
            "valid": bool(
                length >= 0.060
                and unique_centers.shape[0] >= 4
                and rms <= 0.006
            ),
        })
    return recovered


def analyze_branch_local_recovery_shadow(
    branch_id: int,
    branch_points,
    global_axis,
    wrist_point,
    *,
    original_reason: str,
    global_linearity: float | None,
    window_length_m: float = 0.080,
    step_m: float = 0.010,
) -> dict:
    """Diagnose local centerline recovery without changing branch validity."""
    points = np.asarray(branch_points, dtype=np.float64)
    axis = np.asarray(global_axis, dtype=np.float64)
    wrist = np.asarray(wrist_point, dtype=np.float64)
    base = {
        "branch_id": int(branch_id),
        "original_failure_reason": original_reason,
        "global_linearity": (
            float(global_linearity) if global_linearity is not None else None
        ),
    }
    if (
        original_reason != "BRANCH_NOT_LINEAR"
        or points.ndim != 2
        or points.shape[1:] != (3,)
        or points.shape[0] < 60
        or not np.all(np.isfinite(axis))
        or np.linalg.norm(axis) <= 1e-12
    ):
        return {
            **base,
            "window_count": 0,
            "stable_window_count": 0,
            "windows": [],
            "recovery_success": False,
            "recovered_length_m": None,
            "recovered_axis": None,
            "recovered_rms": None,
            "recovered_section_count": 0,
            "axis_stability_p95_deg": None,
            "interpretation": "INSUFFICIENT_GEOMETRY",
        }
    axis = axis / np.linalg.norm(axis)
    projection = (points - wrist) @ axis
    lower, upper = np.percentile(projection, (2.0, 98.0))
    if upper - lower < 0.060:
        return {
            **base,
            "window_count": 0,
            "stable_window_count": 0,
            "windows": [],
            "recovery_success": False,
            "recovered_length_m": None,
            "recovered_axis": None,
            "recovered_rms": None,
            "recovered_section_count": 0,
            "axis_stability_p95_deg": None,
            "interpretation": "INSUFFICIENT_GEOMETRY",
        }
    last_start = max(lower, upper - window_length_m)
    starts = list(np.arange(lower, last_start + 1e-9, step_m))
    if not starts or abs(starts[-1] - last_start) > 1e-6:
        starts.append(last_start)
    windows = [
        _analyze_window(
            points, projection, axis, float(start),
            float(min(start + window_length_m, upper)),
        )
        for start in starts
    ]
    merged = _merge_valid_windows(windows, axis)
    valid_merged = [item for item in merged if item["valid"]]
    direction_conflict = any(
        _unsigned_angle_deg(first["recovered_axis"], second["recovered_axis"])
        > 25.0
        for index, first in enumerate(valid_merged)
        for second in valid_merged[index + 1:]
    )
    best = min(
        valid_merged,
        key=lambda item: (
            -item["recovered_length_m"],
            item["recovered_rms"],
            -item["recovered_section_count"],
        ),
    ) if valid_merged and not direction_conflict else None
    recovery_success = best is not None
    geometry_sufficient = any(
        (window.diagnostic["local_length_m"] or 0.0) >= 0.060
        and window.diagnostic["valid_section_count"] >= 4
        and window.diagnostic.get("valid_section_ratio", 0.0) >= 0.70
        for window in windows
    )
    if recovery_success:
        interpretation = "B_GLOBAL_LINEARITY_GATE_TOO_STRICT"
    elif not geometry_sufficient:
        interpretation = "INSUFFICIENT_GEOMETRY"
    else:
        interpretation = "A_TRUE_NONLINEAR_OR_MIXED_GEOMETRY"
    return {
        **base,
        "window_count": len(windows),
        "stable_window_count": sum(
            window.diagnostic["window_valid"] for window in windows
        ),
        "windows": [window.diagnostic for window in windows],
        "recovery_success": recovery_success,
        "recovered_length_m": (
            best["recovered_length_m"] if best is not None else None
        ),
        "recovered_axis": best["recovered_axis"] if best is not None else None,
        "recovered_rms": best["recovered_rms"] if best is not None else None,
        "recovered_section_count": (
            best["recovered_section_count"] if best is not None else 0
        ),
        "axis_stability_p95_deg": (
            best["recovered_axis_p95_deg"] if best is not None else None
        ),
        "start_offset_from_wrist": (
            best["start_offset_from_wrist"] if best is not None else None
        ),
        "end_offset_from_wrist": (
            best["end_offset_from_wrist"] if best is not None else None
        ),
        "recovered_start_point": (
            best["recovered_start_point"] if best is not None else None
        ),
        "recovered_end_point": (
            best["recovered_end_point"] if best is not None else None
        ),
        "stable_segments": merged,
        "direction_conflict": direction_conflict,
        "interpretation": interpretation,
    }


@dataclass
class BranchLocalRecoveryShadowState:
    statistics_track_id: int | None = None
    recovery_success_count: int = 0
    recovery_failure_count: int = 0
    consecutive_recovery_success_frames: int = 0
    recovered_length_history: deque = field(
        default_factory=lambda: deque(maxlen=30)
    )
    recovered_axis_history: deque = field(
        default_factory=lambda: deque(maxlen=30)
    )
    previous_axis: tuple[float, float, float] | None = None
    previous_length_m: float | None = None
    previous_far_endpoint: tuple[float, float, float] | None = None


def _previous_comparison(candidate: dict, state: BranchLocalRecoveryShadowState):
    angle = _unsigned_angle_deg(candidate.get("recovered_axis"), state.previous_axis)
    recovered_length = candidate.get("recovered_length_m")
    length_ratio = (
        recovered_length / state.previous_length_m
        if recovered_length is not None
        and state.previous_length_m is not None
        and state.previous_length_m > 1e-9 else None
    )
    endpoint = candidate.get("recovered_end_point")
    endpoint_distance = (
        float(np.linalg.norm(
            np.asarray(endpoint) - np.asarray(state.previous_far_endpoint)
        ))
        if endpoint is not None and state.previous_far_endpoint is not None else None
    )
    return {
        "previous_axis_angle_deg": angle,
        "previous_length_ratio": length_ratio,
        "previous_endpoint_distance": endpoint_distance,
    }


def update_branch_local_recovery_shadow(
    state: BranchLocalRecoveryShadowState,
    estimate,
    tracking,
) -> dict:
    """Aggregate embedded local recovery diagnostics without pipeline feedback."""
    active_track_id = tracking.active_track_id
    if (
        active_track_id is not None
        and active_track_id != state.statistics_track_id
    ):
        state.statistics_track_id = active_track_id
        state.recovery_success_count = 0
        state.recovery_failure_count = 0
        state.consecutive_recovery_success_frames = 0
        state.recovered_length_history.clear()
        state.recovered_axis_history.clear()
    candidates = [
        dict(local_recovery)
        for branch in estimate.branches
        if (local_recovery := getattr(branch, "local_recovery_shadow", None))
        is not None
    ]
    for candidate in candidates:
        candidate.update(_previous_comparison(candidate, state))
    successful = [item for item in candidates if item["recovery_success"]]
    best = min(
        successful,
        key=lambda item: (
            -item["recovered_length_m"], item["recovered_rms"], item["branch_id"]
        ),
    ) if successful else None
    best_any = best or (
        max(
            candidates,
            key=lambda item: (
                item.get("stable_window_count", 0),
                item.get("recovered_length_m") or 0.0,
                -item["branch_id"],
            ),
        )
        if candidates else None
    )
    if candidates:
        if best is not None:
            state.recovery_success_count += 1
            state.consecutive_recovery_success_frames += 1
            state.recovered_length_history.append(best["recovered_length_m"])
            state.recovered_axis_history.append(best["recovered_axis"])
        else:
            state.recovery_failure_count += 1
            state.consecutive_recovery_success_frames = 0
    else:
        state.consecutive_recovery_success_frames = 0

    active_branch = next(
        (
            item for item in tracking.branches
            if active_track_id is not None and item.track_id == active_track_id
        ),
        None,
    )
    if active_branch is not None and active_branch.features is not None:
        descriptor = active_branch.features.descriptor
        state.previous_axis = descriptor.axis
        state.previous_length_m = descriptor.effective_length_m
        state.previous_far_endpoint = descriptor.far_endpoint_m

    return {
        "candidate_count": len(candidates),
        "candidates": candidates,
        "best_candidate_id": (
            best_any["branch_id"] if best_any is not None else None
        ),
        "estimator_reason": getattr(estimate, "reason", "UNKNOWN"),
        "frame_interpretation": (
            best["interpretation"]
            if best is not None
            else "A_TRUE_NONLINEAR_OR_MIXED_GEOMETRY"
            if any(
                item["interpretation"] == "A_TRUE_NONLINEAR_OR_MIXED_GEOMETRY"
                for item in candidates
            )
            else "INSUFFICIENT_GEOMETRY"
        ),
        "track_statistics": {
            "track_id": state.statistics_track_id,
            "recovery_success_count": state.recovery_success_count,
            "recovery_failure_count": state.recovery_failure_count,
            "consecutive_recovery_success_frames": (
                state.consecutive_recovery_success_frames
            ),
            "recovered_length_history": tuple(state.recovered_length_history),
            "recovered_axis_history": tuple(state.recovered_axis_history),
        },
        "shadow_only": True,
    }
