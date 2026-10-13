"""Shadow-only re-splitting diagnostics for mixed wrist parent components."""

from __future__ import annotations

import heapq
import math
from typing import Any

import numpy as np


def _unit(value: Any) -> np.ndarray | None:
    if value is None:
        return None
    if hasattr(value, "vector"):
        value = value.vector
    vector = np.asarray(value, dtype=np.float64)
    if vector.shape != (3,) or not np.all(np.isfinite(vector)):
        return None
    norm = float(np.linalg.norm(vector))
    return vector / norm if norm > 1e-12 else None


def _tuple(value):
    return None if value is None else tuple(float(item) for item in value)


def _unsigned_angle(first, second):
    first = _unit(first)
    second = _unit(second)
    if first is None or second is None:
        return None
    cosine = abs(float(np.dot(first, second)))
    return float(np.degrees(np.arccos(np.clip(cosine, 0.0, 1.0))))


def _pixel_neighbors(points: np.ndarray, pixels: np.ndarray, stride: int):
    lookup = {
        (int(pixel[0]), int(pixel[1])): index
        for index, pixel in enumerate(pixels)
    }
    offsets = (
        (-stride, -stride), (0, -stride), (stride, -stride),
        (-stride, 0), (stride, 0), (-stride, stride),
        (0, stride), (stride, stride),
    )
    neighbors = []
    for index, pixel in enumerate(pixels):
        column, row = int(pixel[0]), int(pixel[1])
        connected = []
        for dx, dy in offsets:
            neighbor = lookup.get((column + dx, row + dy))
            if neighbor is None:
                continue
            edge = float(np.linalg.norm(points[neighbor] - points[index]))
            threshold = max(
                0.006,
                0.012 * min(float(points[index, 2]), float(points[neighbor, 2])),
            )
            if edge <= threshold:
                connected.append(neighbor)
        neighbors.append(tuple(connected))
    return neighbors


def _geodesic_distances(points, pixels, wrist_point, stride):
    neighbors = _pixel_neighbors(points, pixels, stride)
    root = int(np.argmin(np.linalg.norm(points - wrist_point, axis=1)))
    distances = np.full(points.shape[0], np.inf, dtype=np.float64)
    distances[root] = 0.0
    queue = [(0.0, root)]
    while queue:
        distance, index = heapq.heappop(queue)
        if distance != distances[index]:
            continue
        for neighbor in neighbors[index]:
            edge = float(np.linalg.norm(points[neighbor] - points[index]))
            candidate = distance + edge
            if candidate < distances[neighbor]:
                distances[neighbor] = candidate
                heapq.heappush(queue, (candidate, neighbor))
    return distances, neighbors, root


def _component_count(selected: np.ndarray, neighbors) -> int:
    remaining = set(int(value) for value in np.flatnonzero(selected))
    count = 0
    while remaining:
        count += 1
        stack = [remaining.pop()]
        while stack:
            for neighbor in neighbors[stack.pop()]:
                if neighbor in remaining:
                    remaining.remove(neighbor)
                    stack.append(neighbor)
    return count


def _largest_component(indices: np.ndarray, neighbors) -> np.ndarray:
    allowed = set(int(value) for value in indices)
    components = []
    while allowed:
        component = [allowed.pop()]
        stack = list(component)
        while stack:
            for neighbor in neighbors[stack.pop()]:
                if neighbor in allowed:
                    allowed.remove(neighbor)
                    component.append(neighbor)
                    stack.append(neighbor)
        components.append(component)
    if not components:
        return np.empty(0, dtype=np.int64)
    return np.asarray(max(components, key=len), dtype=np.int64)


def _shell_diagnostics(points, wrist_point, radial, neighbors):
    output = []
    for radius in (0.020, 0.040, 0.060, 0.080):
        selected = np.abs(radial - radius) <= 0.010
        shell = points[selected]
        if shell.shape[0] < 8:
            output.append({
                "radius_m": radius,
                "point_count": int(shell.shape[0]),
                "width_m": None,
                "component_count": 0,
                "valid": False,
            })
            continue
        center = np.median(shell, axis=0)
        direction = _unit(center - wrist_point)
        delta = shell - center
        if direction is None:
            width = None
        else:
            transverse = delta - np.outer(delta @ direction, direction)
            width = 2.0 * float(np.percentile(
                np.linalg.norm(transverse, axis=1), 90.0
            ))
        output.append({
            "radius_m": radius,
            "point_count": int(shell.shape[0]),
            "width_m": width,
            "component_count": _component_count(selected, neighbors),
            "valid": True,
        })
    valid = [item for item in output if item["valid"] and item["width_m"] is not None]
    split_location = None
    if valid:
        median_count = max(float(np.median([item["point_count"] for item in valid])), 1.0)
        median_width = max(float(np.median([item["width_m"] for item in valid])), 1e-6)
        for item in valid:
            item["bottleneck_score"] = float(
                0.5 * item["point_count"] / median_count
                + 0.5 * item["width_m"] / median_width
            )
        split_location = min(valid, key=lambda item: item["bottleneck_score"])["radius_m"]
    return output, split_location


def _direction_clusters(points, wrist_point, radial, minimum_points=30):
    far = np.flatnonzero(radial >= max(0.040, float(np.percentile(radial, 35.0))))
    if far.size < minimum_points:
        return [], np.full(points.shape[0], -1, dtype=np.int32)
    rays = points[far] - wrist_point
    rays /= np.maximum(np.linalg.norm(rays, axis=1, keepdims=True), 1e-12)
    order = np.argsort(radial[far])[::-1]
    cosine_gate = math.cos(math.radians(30.0))
    sums = []
    for local_index in order:
        ray = rays[local_index]
        if not sums:
            sums.append(ray.copy())
            continue
        centers = [value / max(float(np.linalg.norm(value)), 1e-12) for value in sums]
        similarities = [float(np.dot(ray, center)) for center in centers]
        best = int(np.argmax(similarities))
        if similarities[best] >= cosine_gate:
            sums[best] += ray
        elif len(sums) < 8:
            sums.append(ray.copy())
    centers = np.asarray([
        value / max(float(np.linalg.norm(value)), 1e-12) for value in sums
    ])
    all_rays = points - wrist_point
    all_rays /= np.maximum(np.linalg.norm(all_rays, axis=1, keepdims=True), 1e-12)
    assignments = np.argmax(all_rays @ centers.T, axis=1).astype(np.int32)

    clusters = []
    remap = np.full(centers.shape[0], -1, dtype=np.int32)
    for old_id in range(centers.shape[0]):
        indices = np.flatnonzero(assignments == old_id)
        if indices.size < minimum_points:
            continue
        centered = points[indices] - np.mean(points[indices], axis=0)
        covariance = centered.T @ centered / max(1, indices.size - 1)
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        axis = eigenvectors[:, -1]
        if float(np.dot(axis, centers[old_id])) < 0.0:
            axis = -axis
        new_id = len(clusters)
        remap[old_id] = new_id
        clusters.append({
            "cluster_id": new_id,
            "direction": _tuple(centers[old_id]),
            "local_pca_direction": _tuple(axis),
            "point_count": int(indices.size),
            "fraction": float(indices.size / points.shape[0]),
            "indices": indices,
        })
    mapped = np.asarray([
        remap[value] if 0 <= value < remap.size else -1 for value in assignments
    ], dtype=np.int32)
    return clusters, mapped


def _fit_centerline(points, wrist_point):
    result = {
        "point_count": int(points.shape[0]),
        "length_m": None,
        "axis": None,
        "rms_m": None,
        "section_count": 0,
        "endpoint_m": None,
        "geometry_valid": False,
        "reason": "INSUFFICIENT_POINTS",
    }
    if points.shape[0] < 40:
        return result
    center = np.mean(points, axis=0)
    covariance = np.cov((points - center).T)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    if eigenvalues[-1] <= 1e-10:
        result["reason"] = "PCA_DEGENERATE"
        return result
    axis = eigenvectors[:, -1]
    farthest = points[int(np.argmax(np.linalg.norm(points - wrist_point, axis=1)))]
    if float(np.dot(axis, farthest - wrist_point)) < 0.0:
        axis = -axis
    axial = (points - center) @ axis
    lower, upper = np.percentile(axial, (2.0, 98.0))
    span = float(upper - lower)
    if span < 0.040:
        result["reason"] = "TOO_SHORT"
        return result
    section_count = int(np.clip(math.ceil(span / 0.020), 4, 12))
    edges = np.linspace(lower, upper, section_count + 1)
    centers = []
    for section_index in range(section_count):
        selected = (axial >= edges[section_index]) & (
            axial <= edges[section_index + 1]
            if section_index == section_count - 1
            else axial < edges[section_index + 1]
        )
        section = points[selected]
        if section.shape[0] >= max(8, int(points.shape[0] / section_count * 0.12)):
            centers.append(np.median(section, axis=0))
    result["section_count"] = len(centers)
    if len(centers) < 4:
        result["reason"] = "INSUFFICIENT_SECTIONS"
        return result
    centers = np.asarray(centers)
    line_origin = np.mean(centers, axis=0)
    line_eigenvalues, line_eigenvectors = np.linalg.eigh(np.cov((centers - line_origin).T))
    line_axis = line_eigenvectors[:, -1]
    if float(np.dot(line_axis, farthest - wrist_point)) < 0.0:
        line_axis = -line_axis
    projection = (centers - line_origin) @ line_axis
    length = float(np.max(projection) - np.min(projection))
    residual = (centers - line_origin) - np.outer(projection, line_axis)
    rms = float(math.sqrt(np.mean(np.sum(np.square(residual), axis=1))))
    endpoint = centers[int(np.argmax(projection))]
    result.update({
        "length_m": length,
        "axis": _tuple(line_axis),
        "rms_m": rms,
        "endpoint_m": _tuple(endpoint),
        "geometry_valid": length >= 0.060 and rms <= 0.006,
        "reason": "NONE" if length >= 0.060 and rms <= 0.006 else (
            "TOO_SHORT" if length < 0.060 else "RMS_TOO_HIGH"
        ),
    })
    return result


def _analyze_parent(points, pixels, wrist_point, stride, previous_descriptor):
    radial = np.linalg.norm(points - wrist_point, axis=1)
    if points.shape[0] < 80:
        return {
            "classification": "INSUFFICIENT",
            "reason": "PARENT_TOO_SMALL",
            "direction_cluster_count": 0,
            "direction_clusters": [],
            "cluster_angles_deg": [],
            "cluster_sizes": [],
            "root_bottleneck": {"shells": [], "split_location_m": None},
            "split_candidates": [],
        }
    geodesic, neighbors, root_index = _geodesic_distances(
        points, pixels, wrist_point, stride
    )
    shells, split_location = _shell_diagnostics(
        points, wrist_point, radial, neighbors
    )
    clusters, assignments = _direction_clusters(points, wrist_point, radial)
    cluster_angles = []
    for first in range(len(clusters)):
        for second in range(first + 1, len(clusters)):
            cluster_angles.append(_unsigned_angle(
                clusters[first]["direction"], clusters[second]["direction"]
            ))

    previous_axis = _unit(getattr(previous_descriptor, "axis", None))
    previous_endpoint = (
        np.asarray(previous_descriptor.far_endpoint_m, dtype=np.float64)
        if previous_descriptor is not None else None
    )
    candidates = []
    for cluster in clusters:
        selected = assignments == cluster["cluster_id"]
        selected &= np.isfinite(geodesic)
        if split_location is not None:
            selected &= radial >= max(0.020, split_location - 0.010)
        component = _largest_component(np.flatnonzero(selected), neighbors)
        fit = _fit_centerline(points[component], wrist_point)
        axis = fit["axis"]
        endpoint = fit["endpoint_m"]
        fit.update({
            "candidate_id": int(cluster["cluster_id"]),
            "source_cluster_size": int(cluster["point_count"]),
            "geodesic_span_m": (
                float(np.max(geodesic[component]) - np.min(geodesic[component]))
                if component.size else None
            ),
            "previous_active_angle_deg": _unsigned_angle(axis, previous_axis),
            "previous_endpoint_distance_m": (
                float(np.linalg.norm(np.asarray(endpoint) - previous_endpoint))
                if endpoint is not None and previous_endpoint is not None else None
            ),
        })
        candidates.append(fit)

    recoverable = [
        item for item in candidates
        if item["geometry_valid"]
        and item["previous_active_angle_deg"] is not None
        and item["previous_active_angle_deg"] < 15.0
    ]
    geometrically_valid = [item for item in candidates if item["geometry_valid"]]
    if previous_descriptor is None:
        classification, reason = "INSUFFICIENT", "NO_PREVIOUS_ACTIVE_REFERENCE"
    elif recoverable:
        classification, reason = "FOREARM_RECOVERABLE", "MATCHED_SPLIT_CANDIDATE"
    elif geometrically_valid:
        classification, reason = "PALM_ONLY", "NO_FOREARM_ALIGNED_CANDIDATE"
    elif len(clusters) > 1:
        classification, reason = "MIXED_UNRESOLVED", "MULTIMODAL_WITHOUT_VALID_CENTERLINE"
    else:
        classification, reason = "INSUFFICIENT", "NO_VALID_SPLIT_GEOMETRY"
    return {
        "classification": classification,
        "reason": reason,
        "point_count": int(points.shape[0]),
        "geodesic_reachable_count": int(np.count_nonzero(np.isfinite(geodesic))),
        "geodesic_root_point_m": _tuple(points[root_index]),
        "direction_cluster_count": len(clusters),
        "direction_clusters": [
            {key: value for key, value in cluster.items() if key != "indices"}
            for cluster in clusters
        ],
        "direction_histogram": [int(cluster["point_count"]) for cluster in clusters],
        "cluster_angles_deg": cluster_angles,
        "cluster_sizes": [int(cluster["point_count"]) for cluster in clusters],
        "root_bottleneck": {
            "shells": shells,
            "split_location_m": split_location,
        },
        "split_candidates": candidates,
        "recoverable_candidate_ids": [item["candidate_id"] for item in recoverable],
    }


def analyze_parent_component_split_shadow(
    wrist_point,
    points,
    pixels,
    parent_component_indices,
    step7_diagnostic,
    previous_active_descriptor=None,
    graph_stride=1,
) -> dict:
    """Re-split only Step7 UNSPLIT_PARENT_MIXTURE parents for diagnostics."""
    target_parent_ids = sorted({
        int(branch["parent_component_id"])
        for branch in step7_diagnostic.get("branches", ())
        if branch.get("classification") == "MIXED_STRUCTURE"
        and branch.get("origin_hypothesis") == "UNSPLIT_PARENT_MIXTURE"
    })
    parent_results = []
    for parent_id in target_parent_ids:
        indices = np.asarray(parent_component_indices.get(parent_id, ()), dtype=np.int64)
        if indices.size == 0:
            continue
        result = _analyze_parent(
            np.asarray(points, dtype=np.float64)[indices],
            np.asarray(pixels, dtype=np.int32)[indices],
            np.asarray(wrist_point, dtype=np.float64),
            max(1, int(graph_stride)),
            previous_active_descriptor,
        )
        result["parent_component_id"] = parent_id
        result["existing_branch_ids"] = [
            int(branch["branch_id"])
            for branch in step7_diagnostic.get("branches", ())
            if int(branch["parent_component_id"]) == parent_id
        ]
        parent_results.append(result)
    distribution = {
        label: sum(parent["classification"] == label for parent in parent_results)
        for label in (
            "FOREARM_RECOVERABLE", "PALM_ONLY", "MIXED_UNRESOLVED", "INSUFFICIENT"
        )
    }
    return {
        "triggered_parent_count": len(parent_results),
        "parents": parent_results,
        "classification_distribution": distribution,
        "previous_active_available": previous_active_descriptor is not None,
        "shadow_only": True,
    }
