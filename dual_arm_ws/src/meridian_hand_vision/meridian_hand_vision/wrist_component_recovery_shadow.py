"""Shadow-only recovery candidate for a collapsed wrist-connected mask.

The production mask remains owned by :func:`estimate_wrist_geometry_mask`.
This module deliberately has no tracker or publisher access.  It enumerates
depth-continuous skin components which actually approach the MediaPipe wrist,
then prefers local wrist support over global image area.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import time

import cv2
import numpy as np

from .arm_contour import WristGeometryMaskEstimate


@dataclass(frozen=True)
class WristComponentRecoveryShadowEstimate:
    mask: np.ndarray
    wrist_seed_pixel: tuple[int, int] | None
    wrist_seed_point_m: tuple[float, float, float] | None
    diagnostics_data: dict

    def diagnostics(self) -> dict:
        return dict(self.diagnostics_data)


def _empty(shape, reason: str, elapsed_ms: float = 0.0):
    return WristComponentRecoveryShadowEstimate(
        mask=np.zeros(shape, dtype=np.uint8),
        wrist_seed_pixel=None,
        wrist_seed_point_m=None,
        diagnostics_data={
            "shadow_only": True,
            "reason": reason,
            "candidate_seed_source": "NONE",
            "candidate_seed_pixel": None,
            "candidate_seed_depth_m": None,
            "component_count": 0,
            "components": [],
            "selected_component_id": None,
            "selected_connected_point_count": 0,
            "elapsed_ms": float(elapsed_ms),
        },
    )


def _camera_arrays(depth_m, camera_info, mirrored: bool):
    height, width = depth_m.shape
    calibration_width = int(getattr(camera_info, "width", 0) or width)
    calibration_height = int(getattr(camera_info, "height", 0) or height)
    matrix = tuple(getattr(camera_info, "k", ()))
    if len(matrix) != 9:
        return None
    fx, fy, cx, cy = map(float, (matrix[0], matrix[4], matrix[2], matrix[5]))
    if min(fx, fy) <= 0.0 or not np.all(np.isfinite((fx, fy, cx, cy))):
        return None
    rows, columns = np.indices(depth_m.shape, dtype=np.float64)
    scale_x = calibration_width / float(width)
    scale_y = calibration_height / float(height)
    deprojection_columns = width - 1.0 - columns if mirrored else columns
    u_info = (deprojection_columns + 0.5) * scale_x - 0.5
    v_info = (rows + 0.5) * scale_y - 0.5
    z = depth_m.astype(np.float64)
    x = (u_info - cx) * z / fx
    y = (v_info - cy) * z / fy
    return x, y, z


def estimate_wrist_component_recovery_shadow(
    wrist_landmark,
    frame_bgr: np.ndarray,
    depth_m: np.ndarray,
    camera_info,
    production_mask: WristGeometryMaskEstimate,
    *,
    mirrored: bool = False,
    physical_roi_radius_m: float = 0.25,
    wrist_candidate_radius_px: int = 32,
    local_support_radius_px: int = 24,
    maximum_seed_depth_delta_m: float = 0.05,
) -> WristComponentRecoveryShadowEstimate:
    """Enumerate wrist-near components without replacing the production mask.

    A component is eligible only if it contains a skin+depth pixel close to the
    wrist, stays in one local depth layer and has strong adjacent 3-D support.
    Consequently the globally largest skin component cannot be selected merely
    because it is large.
    """
    started = time.perf_counter()
    if frame_bgr is None or depth_m is None or frame_bgr.shape[:2] != depth_m.shape:
        return _empty(depth_m.shape if depth_m is not None else (0, 0), "INVALID_INPUT")
    height, width = depth_m.shape
    if wrist_landmark is None or camera_info is None:
        return _empty((height, width), "WRIST_OR_CAMERA_INFO_MISSING")
    try:
        wrist_pixel = np.asarray((
            float(np.clip(wrist_landmark.x, 0.0, 1.0) * (width - 1)),
            float(np.clip(wrist_landmark.y, 0.0, 1.0) * (height - 1)),
        ), dtype=np.float64)
    except (AttributeError, TypeError, ValueError):
        return _empty((height, width), "WRIST_INVALID")
    if not np.all(np.isfinite(wrist_pixel)):
        return _empty((height, width), "WRIST_INVALID")

    camera = _camera_arrays(depth_m, camera_info, mirrored)
    if camera is None:
        return _empty((height, width), "INVALID_CAMERA_INFO")
    x_camera, y_camera, z_camera = camera
    valid_depth = (
        np.isfinite(depth_m) & (depth_m >= 0.10) & (depth_m <= 2.0)
    )
    ycrcb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2YCrCb)
    skin = cv2.inRange(ycrcb, (0, 133, 77), (255, 180, 135)) > 0
    skin = cv2.morphologyEx(
        np.uint8(skin) * 255,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)),
        iterations=1,
    ) > 0
    base_candidate = valid_depth & skin
    original_seed_source = "NONE"
    if production_mask.wrist_seed_pixel is not None:
        original_column, original_row = production_mask.wrist_seed_pixel
        original_seed_source = (
            "SKIN_DEPTH"
            if 0 <= original_row < height and 0 <= original_column < width
            and base_candidate[original_row, original_column]
            else "DEPTH_FALLBACK"
        )
    original_diagnostics = {
        "original_seed_source": original_seed_source,
        "original_seed_pixel": production_mask.wrist_seed_pixel,
        "original_seed_point_m": production_mask.wrist_seed_point_m,
        "original_connected_point_count": production_mask.connected_point_count,
        "original_reason": production_mask.reason,
    }
    rows, columns = np.indices(depth_m.shape)
    wrist_distance_sq = (
        np.square(columns.astype(np.float64) - wrist_pixel[0])
        + np.square(rows.astype(np.float64) - wrist_pixel[1])
    )
    local = wrist_distance_sq <= float(max(1, wrist_candidate_radius_px) ** 2)
    starts = np.argwhere(base_candidate & local)
    if starts.size == 0:
        elapsed = (time.perf_counter() - started) * 1000.0
        result = _empty((height, width), "NO_WRIST_NEAR_SKIN_DEPTH", elapsed)
        diagnostics = result.diagnostics()
        diagnostics.update(original_diagnostics)
        diagnostics["wrist_pixel"] = wrist_pixel.tolist()
        return WristComponentRecoveryShadowEstimate(
            result.mask, None, None, diagnostics
        )

    local_depths = depth_m[base_candidate & local].astype(np.float64)
    reference_depth = float(np.median(local_depths))
    components: list[dict] = []
    component_masks: list[np.ndarray] = []
    # OpenCV labels the 2-D skin islands in native code.  Each wrist-near
    # island is then split again by seed depth and the 250 mm 3-D sphere.  This
    # preserves the production geometric constraints without adding a second
    # Python BFS over tens of thousands of pixels to every camera frame.
    _, skin_labels = cv2.connectedComponents(
        np.uint8(base_candidate), connectivity=8
    )
    local_labels = np.unique(skin_labels[base_candidate & local])
    local_labels = local_labels[local_labels > 0]
    for skin_label in local_labels:
        island = skin_labels == int(skin_label)
        island_rows, island_columns = np.nonzero(island & local)
        if island_rows.size == 0:
            continue
        root_distances = wrist_distance_sq[island_rows, island_columns]
        nearest = int(np.argmin(root_distances))
        seed_row = int(island_rows[nearest])
        seed_column = int(island_columns[nearest])
        seed_point = np.asarray((
            x_camera[seed_row, seed_column],
            y_camera[seed_row, seed_column],
            z_camera[seed_row, seed_column],
        ), dtype=np.float64)
        physical_distance_sq = (
            np.square(x_camera - seed_point[0])
            + np.square(y_camera - seed_point[1])
            + np.square(z_camera - seed_point[2])
        )
        depth_and_roi = (
            island
            & (np.abs(depth_m - float(seed_point[2])) <= maximum_seed_depth_delta_m)
            & (physical_distance_sq <= physical_roi_radius_m ** 2)
        )
        _, refined_labels = cv2.connectedComponents(
            np.uint8(depth_and_roi), connectivity=8
        )
        root_label = int(refined_labels[seed_row, seed_column])
        if root_label <= 0:
            continue
        component = refined_labels == root_label
        component_rows, component_columns = np.nonzero(component)
        if component_rows.size == 0:
            continue
        distances_sq = wrist_distance_sq[component_rows, component_columns]
        root_index = int(np.argmin(distances_sq))
        root_row = int(component_rows[root_index])
        root_column = int(component_columns[root_index])
        root_distance_px = math.sqrt(float(distances_sq[root_index]))
        support = int(np.count_nonzero(
            component
            & (wrist_distance_sq <= float(local_support_radius_px ** 2))
        ))
        edge_distances = []
        for row_slice_a, column_slice_a, row_slice_b, column_slice_b in (
            (slice(None), slice(None, -1), slice(None), slice(1, None)),
            (slice(None, -1), slice(None), slice(1, None), slice(None)),
        ):
            paired = (
                component[row_slice_a, column_slice_a]
                & component[row_slice_b, column_slice_b]
            )
            if not np.any(paired):
                continue
            dx = (
                x_camera[row_slice_b, column_slice_b]
                - x_camera[row_slice_a, column_slice_a]
            )[paired]
            dy = (
                y_camera[row_slice_b, column_slice_b]
                - y_camera[row_slice_a, column_slice_a]
            )[paired]
            dz = (
                z_camera[row_slice_b, column_slice_b]
                - z_camera[row_slice_a, column_slice_a]
            )[paired]
            edge_distances.append(np.sqrt(dx * dx + dy * dy + dz * dz))
        neighbor_gaps = (
            np.concatenate(edge_distances)
            if edge_distances else np.empty(0, dtype=np.float64)
        )
        local_edge_threshold = max(0.006, 0.012 * float(seed_point[2]))
        edge_support_ratio = (
            float(np.mean(neighbor_gaps <= local_edge_threshold))
            if neighbor_gaps.size else 0.0
        )
        component_depths = depth_m[component].astype(np.float64)
        component_id = len(components)
        components.append({
            "component_id": component_id,
            "point_count": int(component_rows.size),
            "local_support_count": support,
            "root_pixel": [root_column, root_row],
            "root_distance_px": root_distance_px,
            "root_depth_m": float(depth_m[root_row, root_column]),
            "median_depth_m": float(np.median(component_depths)),
            "depth_p10_m": float(np.percentile(component_depths, 10.0)),
            "depth_p90_m": float(np.percentile(component_depths, 90.0)),
            "local_3d_edge_support_ratio": edge_support_ratio,
            "maximum_neighbor_gap_m": (
                float(np.max(neighbor_gaps)) if neighbor_gaps.size else None
            ),
            "eligible": bool(
                root_distance_px <= wrist_candidate_radius_px
                and support >= 3
                and edge_support_ratio >= 0.90
                and abs(float(depth_m[root_row, root_column]) - reference_depth)
                <= maximum_seed_depth_delta_m
            ),
        })
        component_masks.append(component)

    eligible = [item for item in components if item["eligible"]]
    if not eligible:
        elapsed = (time.perf_counter() - started) * 1000.0
        result = _empty((height, width), "NO_ELIGIBLE_WRIST_COMPONENT", elapsed)
        diagnostics = result.diagnostics()
        diagnostics.update({
            "wrist_pixel": wrist_pixel.tolist(),
            "reference_depth_m": reference_depth,
            "component_count": len(components),
            "components": components,
        })
        diagnostics.update(original_diagnostics)
        return WristComponentRecoveryShadowEstimate(
            result.mask, None, None, diagnostics
        )

    # Local wrist support is deliberately the first key.  Total area is only
    # a tie-breaker and cannot make a distant full-image skin region eligible.
    selected = max(
        eligible,
        key=lambda item: (
            item["local_support_count"],
            min(item["point_count"], 50000),
            -item["root_distance_px"],
        ),
    )
    selected_mask = component_masks[int(selected["component_id"])]
    root_column, root_row = selected["root_pixel"]
    seed_point = (
        float(x_camera[root_row, root_column]),
        float(y_camera[root_row, root_column]),
        float(z_camera[root_row, root_column]),
    )
    original = production_mask.mask > 0
    intersection = int(np.count_nonzero(original & selected_mask))
    union = int(np.count_nonzero(original | selected_mask))
    elapsed = (time.perf_counter() - started) * 1000.0
    diagnostics = {
        "shadow_only": True,
        "reason": "NONE",
        "physical_roi_radius_m": float(physical_roi_radius_m),
        "wrist_candidate_radius_px": int(wrist_candidate_radius_px),
        "local_support_radius_px": int(local_support_radius_px),
        "wrist_pixel": wrist_pixel.tolist(),
        "reference_depth_m": reference_depth,
        **original_diagnostics,
        "candidate_seed_source": "WRIST_NEAR_SKIN_DEPTH_COMPONENT_SUPPORT",
        "candidate_seed_pixel": [root_column, root_row],
        "candidate_seed_depth_m": float(depth_m[root_row, root_column]),
        "component_count": len(components),
        "components": components,
        "selected_component_id": selected["component_id"],
        "selected_connected_point_count": int(np.count_nonzero(selected_mask)),
        "selection_basis": "LOCAL_SUPPORT_THEN_COMPONENT_SIZE",
        "mask_intersection_px": intersection,
        "mask_union_px": union,
        "mask_iou": float(intersection / union) if union else None,
        "symmetric_difference_px": int(np.count_nonzero(original ^ selected_mask)),
        "elapsed_ms": elapsed,
    }
    return WristComponentRecoveryShadowEstimate(
        mask=np.uint8(selected_mask) * 255,
        wrist_seed_pixel=(root_column, root_row),
        wrist_seed_point_m=seed_point,
        diagnostics_data=diagnostics,
    )


def add_branch_comparison(
    diagnostics: dict,
    production_estimate,
    candidate_estimate,
    historical_axis=None,
) -> dict:
    """Attach estimator results without changing either estimator object."""
    result = dict(diagnostics)
    production = production_estimate.diagnostics()
    candidate = candidate_estimate.diagnostics()

    def axis_angle(left, right):
        if left is None or right is None:
            return None
        a = np.asarray(left, dtype=np.float64)
        b = np.asarray(right, dtype=np.float64)
        if a.shape != (3,) or b.shape != (3,):
            return None
        denominator = float(np.linalg.norm(a) * np.linalg.norm(b))
        if denominator <= 1e-12 or not np.all(np.isfinite(a)) or not np.all(np.isfinite(b)):
            return None
        cosine = abs(float(np.dot(a, b))) / denominator
        return float(np.degrees(np.arccos(np.clip(cosine, 0.0, 1.0))))

    production_axis = (
        production_estimate.direction.vector
        if production_estimate.direction is not None else None
    )
    candidate_axis = (
        candidate_estimate.direction.vector
        if candidate_estimate.direction is not None else None
    )

    def branch_diagnostics(estimate):
        return [
            {
                "branch_id": branch.branch_id,
                "parent_component_id": branch.parent_component_id,
                "point_count": branch.point_count,
                "span_m": branch.span_m,
                "pca_linearity": branch.pca_linearity,
                "valid_section_count": branch.valid_section_count,
                "inlier_section_count": branch.inlier_section_count,
                "inlier_ratio": branch.inlier_ratio,
                "axis_length_m": branch.axis_length_m,
                "rms_m": branch.rms_m,
                "valid": branch.valid,
                "reason": branch.reason,
            }
            for branch in getattr(estimate, "branches", ())
        ]

    result.update({
        "production_branch": production,
        "candidate_branch": candidate,
        "production_branch_candidates": branch_diagnostics(production_estimate),
        "candidate_branch_candidates": branch_diagnostics(candidate_estimate),
        "candidate_parent_topology_shadow": (
            getattr(candidate_estimate, "parent_topology_shadow", None)
        ),
        "production_success": production_estimate.direction is not None,
        "candidate_success": candidate_estimate.direction is not None,
        "original_failed_candidate_succeeded": bool(
            production_estimate.direction is None
            and candidate_estimate.direction is not None
        ),
        "original_succeeded_candidate_failed": bool(
            production_estimate.direction is not None
            and candidate_estimate.direction is None
        ),
        "candidate_vs_production_axis_deg": axis_angle(
            candidate_axis, production_axis
        ),
        "candidate_vs_v2_2_history_axis_deg": axis_angle(
            candidate_axis, historical_axis
        ),
    })
    return result
