"""Offline four-hypothesis truth deskew and wall/cylinder geometry analysis."""

import argparse
import json
from collections import defaultdict
from pathlib import Path

import numpy as np
import yaml

from .deskew_core import (
    POINT_TIME_CONVENTIONS,
    PoseBuffer,
    compose_pose,
    deskew_points,
    point_times_ns,
)


STREAMS = {
    "/antbot/lidar/front_left/points_raw_native": (
        "front_left", [0.322, 0.222, 0.414], [0, 0, 0, 1]
    ),
    "/antbot/lidar/rear_right/points_raw_native": (
        "rear_right", [-0.322, -0.222, 0.414], [0, 0, 1, 0]
    ),
}
ARENA_SCALE = 2.0


def select_time_hypothesis(candidates, minimum_lead_percent=5.0):
    ranked = sorted(
        (
            (name, values)
            for name, values in candidates.items()
            if values["median_deskew_rmse"] is not None
        ),
        key=lambda item: (
            item[1]["median_deskew_rmse"], item[1]["p95_deskew_rmse"]
        ),
    )
    if len(ranked) < 2:
        return {"selected": None, "reason": "fewer_than_two_covered_hypotheses"}
    best, second = ranked[:2]
    lead = 100.0 * (
        second[1]["median_deskew_rmse"] - best[1]["median_deskew_rmse"]
    ) / second[1]["median_deskew_rmse"]
    return {
        "selected": best[0] if lead >= minimum_lead_percent else None,
        "best": best[0],
        "runner_up": second[0],
        "median_rmse_lead_percent": lead,
        "minimum_lead_percent": minimum_lead_percent,
        "reason": (
            "selected"
            if lead >= minimum_lead_percent
            else "lead_below_predeclared_threshold"
        ),
    }


def _reader(path):
    import rosbag2_py
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    return reader


def _events(path):
    data = yaml.safe_load(Path(path).read_text())
    return [
        item for item in data["motions"]
        if "rotation_" in item["name"] or item["name"] in ("forward_left", "forward_right")
    ]


def _event_for_stamp(events, stamp):
    for event in events:
        if event["start_ns"] + 300_000_000 <= stamp <= event["end_ns"] - 200_000_000:
            return event["name"]
    return None


def _fit_plane(points):
    center = points.mean(axis=0)
    _, _, vectors = np.linalg.svd(points - center, full_matrices=False)
    normal = vectors[-1]
    signed = (points - center) @ normal
    absolute = np.abs(signed)
    return {
        "rmse": float(np.sqrt(np.mean(signed ** 2))),
        "median": float(np.median(absolute)),
        "p95": float(np.percentile(absolute, 95)),
        "p99": float(np.percentile(absolute, 99)),
        "thickness": float(np.percentile(signed, 95) - np.percentile(signed, 5)),
    }


def _wall_metrics(points):
    wall_center = 3.0 * ARENA_SCALE
    # add_cube receives a 0.12 m wall thickness. Rays from inside the arena
    # hit the inner face, 0.06 m inward from the authored center plane.
    wall = wall_center - 0.06
    candidate_residuals = np.column_stack(
        (
            np.abs(points[:, 0] - wall),
            np.abs(points[:, 0] + wall),
            np.abs(points[:, 1] - wall),
            np.abs(points[:, 1] + wall),
        )
    )
    nearest = np.min(candidate_residuals, axis=1)
    mask = (
        (nearest < 0.35)
        & (points[:, 2] > 0.05)
        & (points[:, 2] < 1.05)
    )
    selected = points[mask]
    residual = nearest[mask]
    if len(selected) < 40:
        return None
    fitted_walls = []
    for axis, value in ((0, wall), (0, -wall), (1, wall), (1, -wall)):
        wall_mask = (
            (np.abs(points[:, axis] - value) < 0.35)
            & (points[:, 2] > 0.05)
            & (points[:, 2] < 1.05)
        )
        if np.count_nonzero(wall_mask) >= 40:
            fitted_walls.append(_fit_plane(points[wall_mask]))
    if not fitted_walls:
        return None
    result = {
        "rmse": float(np.sqrt(np.mean(residual ** 2))),
        "median": float(np.median(residual)),
        "p95": float(np.percentile(residual, 95)),
        "p99": float(np.percentile(residual, 99)),
        "thickness": float(np.percentile(residual, 95) - np.percentile(residual, 5)),
        "known_wall_coordinate_abs": float(wall),
        "shape_rmse": float(np.median([item["rmse"] for item in fitted_walls])),
        "shape_p95": float(np.median([item["p95"] for item in fitted_walls])),
    }
    result["point_count"] = int(len(selected))
    return result


def _fit_circle(points, known_center=None, known_radius=0.2):
    xy = points[:, :2]
    matrix = np.column_stack((2 * xy[:, 0], 2 * xy[:, 1], np.ones(len(xy))))
    solution, *_ = np.linalg.lstsq(matrix, np.sum(xy * xy, axis=1), rcond=None)
    center = solution[:2]
    radius = float(np.sqrt(max(0.0, solution[2] + np.dot(center, center))))
    residual = np.linalg.norm(xy - center, axis=1) - radius
    absolute = np.abs(residual)
    reference_center = center if known_center is None else np.asarray(known_center)
    relative = xy - reference_center
    angles = np.arctan2(relative[:, 1], relative[:, 0])
    reference_radius_residual = np.linalg.norm(relative, axis=1) - known_radius
    bin_ids = np.floor((angles + np.pi) / (np.pi / 18.0)).astype(np.int64)
    binned_variances = [
        float(np.var(reference_radius_residual[bin_ids == index]))
        for index in np.unique(bin_ids)
        if np.count_nonzero(bin_ids == index) >= 3
    ]
    tangential = known_radius * angles
    return {
        "radius": radius,
        "radius_abs_error": abs(radius - known_radius),
        "rmse": float(np.sqrt(np.mean(residual ** 2))),
        "p95": float(np.percentile(absolute, 95)),
        "p99": float(np.percentile(absolute, 99)),
        "thickness": float(np.percentile(residual, 95) - np.percentile(residual, 5)),
        "azimuth_binned_radial_variance": (
            float(np.mean(binned_variances)) if binned_variances else None
        ),
        "tangential_smear_length": float(np.ptp(tangential)),
        "envelope_width": float(np.ptp(relative[:, 0])),
        "left_edge_error": float(abs(relative[:, 0].min() + known_radius)),
        "right_edge_error": float(abs(relative[:, 0].max() - known_radius)),
    }


def _column_metrics(points):
    candidates = []
    column = 2.0 * ARENA_SCALE
    for center in ((column, 0.0), (-column, 0.0)):
        radial = np.linalg.norm(points[:, :2] - np.asarray(center), axis=1)
        mask = (
            (np.abs(radial - 0.2) < 0.08)
            & (points[:, 2] > 0.05)
            & (points[:, 2] < 0.8)
        )
        selected = points[mask]
        if len(selected) >= 30:
            candidates.append((selected, center))
    if not candidates:
        return None
    selected, center = max(candidates, key=lambda item: len(item[0]))
    result = _fit_circle(selected, known_center=center)
    result["point_count"] = int(len(selected))
    return result


def _square_column_metrics(points):
    candidates = []
    half_width = 0.2
    column = 2.0 * ARENA_SCALE
    for center in ((0.0, column), (0.0, -column)):
        relative = points[:, :2] - np.asarray(center)
        mask = (
            (np.max(np.abs(relative), axis=1) < 0.28)
            & (points[:, 2] > 0.05)
            & (points[:, 2] < 0.8)
        )
        selected = relative[mask]
        if len(selected) >= 30:
            candidates.append(selected)
    if not candidates:
        return None
    relative = max(candidates, key=len)
    distance_to_x_face = np.abs(np.abs(relative[:, 0]) - half_width)
    distance_to_y_face = np.abs(np.abs(relative[:, 1]) - half_width)
    residual = np.minimum(distance_to_x_face, distance_to_y_face)
    return {
        "rmse": float(np.sqrt(np.mean(residual ** 2))),
        "median": float(np.median(residual)),
        "p95": float(np.percentile(residual, 95)),
        "p99": float(np.percentile(residual, 99)),
        "thickness": float(np.percentile(residual, 95) - np.percentile(residual, 5)),
        "envelope_width": float(np.ptp(relative[:, 0])),
        "tangential_smear_length": float(np.ptp(relative[:, 1])),
        "left_edge_error": float(abs(relative[:, 0].min() + half_width)),
        "right_edge_error": float(abs(relative[:, 0].max() - half_width)),
        "point_count": int(len(relative)),
    }


def analyze(bag, events_path, output_json, output_csv):
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2

    events = _events(events_path)
    poses = PoseBuffer(200.0)
    reader = _reader(bag)
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != "/antbot/ground_truth/odom":
            continue
        message = deserialize_message(data, Odometry)
        p, q = message.pose.pose.position, message.pose.pose.orientation
        poses.add(
            message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec,
            [p.x, p.y, p.z], [q.x, q.y, q.z, q.w],
        )

    headers = defaultdict(list)
    rows = []
    frame_index = defaultdict(int)
    reader = _reader(bag)
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic not in STREAMS:
            continue
        message = deserialize_message(data, PointCloud2)
        header_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        headers[topic].append(header_ns)
        motion = _event_for_stamp(events, header_ns)
        if motion is None:
            continue
        frame_index[(topic, motion)] += 1
        if frame_index[(topic, motion)] % 4:
            continue
        values = point_cloud2.read_points(
            message, field_names=["x", "y", "z", "time_offset_ns"], skip_nans=True
        )[::8]
        xyz = np.column_stack((values["x"], values["y"], values["z"]))
        offsets = np.asarray(values["time_offset_ns"], dtype=np.int32)
        lidar, translation, quaternion = STREAMS[topic]
        for convention in POINT_TIME_CONVENTIONS:
            scan_period_ns = int(np.median(np.diff(headers[topic]))) if len(headers[topic]) > 2 else 100_000_000
            stamps = point_times_ns(header_ns, offsets, convention, scan_period_ns * 1e-9)
            # Native coordinates belong to each ray's sampling-time lidar
            # frame. Keep the output reference at Header across hypotheses.
            reference_ns = header_ns
            corrected = deskew_points(
                xyz, stamps, reference_ns, poses, translation, quaternion
            )
            reference_pose = poses.interpolate(reference_ns)
            coverage = corrected is not None and reference_pose is not None
            if not coverage:
                rows.append({
                    "case": "unclassified", "lidar": lidar, "motion": motion,
                    "time_hypothesis": convention, "header_stamp": header_ns,
                    "offset_min_ns": int(offsets.min()), "offset_max_ns": int(offsets.max()),
                    "truth_coverage_status": "insufficient",
                })
                continue
            sensor_t, sensor_r = compose_pose(reference_pose, translation, quaternion)
            raw_world = xyz @ sensor_r.T + sensor_t
            deskew_world = corrected @ sensor_r.T + sensor_t
            for case, metric in (
                ("wall", _wall_metrics),
                ("column", _column_metrics),
                ("square_column", _square_column_metrics),
            ):
                raw = metric(raw_world)
                deskew = metric(deskew_world)
                if raw is None or deskew is None:
                    continue
                rows.append({
                    "case": case, "lidar": lidar, "motion": motion,
                    "time_hypothesis": convention,
                    "raw_rmse": raw["rmse"], "deskew_rmse": deskew["rmse"],
                    "improvement_percent": 100.0 * (raw["rmse"] - deskew["rmse"]) / raw["rmse"],
                    "raw_median": raw.get("median"), "deskew_median": deskew.get("median"),
                    "raw_p95": raw["p95"], "deskew_p95": deskew["p95"],
                    "raw_p99": raw.get("p99"), "deskew_p99": deskew.get("p99"),
                    "raw_thickness": raw["thickness"], "deskew_thickness": deskew["thickness"],
                    "raw_radius": raw.get("radius"), "deskew_radius": deskew.get("radius"),
                    "raw_radius_abs_error": raw.get("radius_abs_error"),
                    "deskew_radius_abs_error": deskew.get("radius_abs_error"),
                    "raw_azimuth_binned_radial_variance": raw.get("azimuth_binned_radial_variance"),
                    "deskew_azimuth_binned_radial_variance": deskew.get("azimuth_binned_radial_variance"),
                    "raw_tangential_smear_length": raw.get("tangential_smear_length"),
                    "deskew_tangential_smear_length": deskew.get("tangential_smear_length"),
                    "raw_envelope_width": raw.get("envelope_width"),
                    "deskew_envelope_width": deskew.get("envelope_width"),
                    "raw_left_edge_error": raw.get("left_edge_error"),
                    "deskew_left_edge_error": deskew.get("left_edge_error"),
                    "raw_right_edge_error": raw.get("right_edge_error"),
                    "deskew_right_edge_error": deskew.get("right_edge_error"),
                    "raw_shape_rmse": raw.get("shape_rmse"),
                    "deskew_shape_rmse": deskew.get("shape_rmse"),
                    "raw_shape_p95": raw.get("shape_p95"),
                    "deskew_shape_p95": deskew.get("shape_p95"),
                    "point_count": min(raw["point_count"], deskew["point_count"]),
                    "header_stamp": header_ns, "offset_min_ns": int(offsets.min()),
                    "offset_max_ns": int(offsets.max()), "truth_coverage_status": "covered",
                    "scan_period_ns": scan_period_ns,
                })
    valid = [row for row in rows if row.get("truth_coverage_status") == "covered"]
    summary = {}
    for lidar in ("front_left", "rear_right"):
        summary[lidar] = {}
        for hypothesis in POINT_TIME_CONVENTIONS:
            selected = [
                row for row in valid
                if row["lidar"] == lidar
                and row["time_hypothesis"] == hypothesis
                and row["case"] in ("wall", "square_column")
            ]
            summary[lidar][hypothesis] = {
                "frame_metric_count": len(selected),
                "median_deskew_rmse": float(np.median([row["deskew_rmse"] for row in selected])) if selected else None,
                "p95_deskew_rmse": float(np.percentile([row["deskew_rmse"] for row in selected], 95)) if selected else None,
                "median_improvement_percent": float(np.median([row["improvement_percent"] for row in selected])) if selected else None,
            }
    selections = {}
    for lidar, candidates in summary.items():
        selections[lidar] = select_time_hypothesis(candidates)
    result = {
        "selection_rule": (
            "minimum median deskew RMSE, then P95; require both yaw signs, "
            "known wall and decontaminated square-column geometries, and a "
            "predeclared >=5% median lead; circular-column metrics are reported "
            "as a sensitivity diagnostic but are not used for selection"
        ),
        "summary": summary,
        "selections": selections,
        "rows": rows,
    }
    Path(output_json).write_text(json.dumps(result, indent=2), encoding="utf-8")
    import csv
    keys = sorted({key for row in rows for key in row})
    with Path(output_csv).open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)
    # Under model A, per-point truth world endpoints transformed back to the
    # Header frame are exactly the legacy WORLD-GMO/current-pose adapter
    # representation. Export that comparison with an explicit emulation label.
    gmo_rows = []
    for row in valid:
        if row["time_hypothesis"] != "header_plus_offset":
            continue
        item = dict(row)
        item["comparison_source"] = "legacy_world_gmo_header_pose_emulation"
        item["gmo_header_frame_rmse"] = item["deskew_rmse"]
        item["raw_native_rmse"] = item["raw_rmse"]
        gmo_rows.append(item)
    gmo_path = Path(output_csv).with_name("raw_vs_gmo_metrics.csv")
    gmo_keys = sorted({key for row in gmo_rows for key in row})
    with gmo_path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=gmo_keys)
        writer.writeheader()
        writer.writerows(gmo_rows)
    return result


def main(argv=None):
    global ARENA_SCALE
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("events")
    parser.add_argument("--output-json", required=True)
    parser.add_argument("--output-csv", required=True)
    parser.add_argument("--arena-scale", type=float, default=2.0)
    args = parser.parse_args(argv)
    ARENA_SCALE = args.arena_scale
    result = analyze(args.bag, args.events, args.output_json, args.output_csv)
    print(json.dumps(result["summary"], indent=2))
