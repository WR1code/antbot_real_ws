from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math

import cv2
import numpy as np


def _landmark_pixels(landmarks, width: int, height: int) -> list[tuple[int, int]]:
    """Convert normalized landmarks without importing the MediaPipe runtime."""
    return [
        (
            int(np.clip(point.x, 0.0, 1.0) * (width - 1)),
            int(np.clip(point.y, 0.0, 1.0) * (height - 1)),
        )
        for point in landmarks
    ]


@dataclass
class ArmObservation:
    contour: np.ndarray
    wrist: tuple[int, int]
    wrist_depth_m: float | None


@dataclass(frozen=True)
class WristGeometryMaskEstimate:
    """Direction-free wrist-seeded mask used only by the V2 shadow path."""

    mask: np.ndarray
    mask_area_px: int
    connected_point_count: int
    wrist_seed_pixel: tuple[int, int] | None
    wrist_seed_point_m: tuple[float, float, float] | None
    physical_roi_radius_m: float
    contains_palm_region: bool | None
    palm_region_reason: str
    reason: str

    def diagnostics(self) -> dict:
        return {
            "mask_area_px": self.mask_area_px,
            "connected_point_count": self.connected_point_count,
            "wrist_seed_pixel": self.wrist_seed_pixel,
            "wrist_seed_point_m": self.wrist_seed_point_m,
            "physical_roi_radius_m": self.physical_roi_radius_m,
            "contains_palm_region": self.contains_palm_region,
            "palm_region_reason": self.palm_region_reason,
            "reason": self.reason,
        }


def _median_depth(depth_m: np.ndarray, center: tuple[int, int], radius: int) -> float | None:
    height, width = depth_m.shape
    x, y = center
    y0, y1 = max(0, y - radius), min(height, y + radius + 1)
    x0, x1 = max(0, x - radius), min(width, x + radius + 1)
    patch = depth_m[y0:y1, x0:x1]
    valid = patch[np.isfinite(patch) & (patch > 0.05)]
    if valid.size == 0:
        return None
    return float(np.median(valid))


def _forearm_roi(points: list[tuple[int, int]], shape: tuple[int, int]) -> np.ndarray:
    height, width = shape
    wrist = np.asarray(points[0], dtype=np.float32)
    palm = np.mean(np.asarray([points[5], points[9], points[13], points[17]]), axis=0)
    outward = wrist - palm
    norm = float(np.linalg.norm(outward))
    if norm < 1.0:
        outward = np.asarray([0.0, 1.0], dtype=np.float32)
    else:
        outward /= norm
    perpendicular = np.asarray([-outward[1], outward[0]], dtype=np.float32)
    palm_width = float(np.linalg.norm(np.asarray(points[5]) - np.asarray(points[17])))
    palm_width = float(np.clip(palm_width, 24.0, min(width, height) * 0.30))
    far = wrist + outward * palm_width * 4.5
    near_half = palm_width * 0.72
    far_half = palm_width * 1.05
    polygon = np.asarray(
        [
            wrist + perpendicular * near_half,
            wrist - perpendicular * near_half,
            far - perpendicular * far_half,
            far + perpendicular * far_half,
        ],
        dtype=np.int32,
    )
    polygon[:, 0] = np.clip(polygon[:, 0], 0, width - 1)
    polygon[:, 1] = np.clip(polygon[:, 1], 0, height - 1)
    return polygon


def estimate_arm_contours(
    frame_bgr: np.ndarray,
    depth_m: np.ndarray,
    hands,
    depth_tolerance_m: float = 0.18,
) -> tuple[np.ndarray, list[ArmObservation]]:
    """Estimate forearm contours from wrist direction and registered depth."""
    height, width = frame_bgr.shape[:2]
    combined_mask = np.zeros((height, width), dtype=np.uint8)
    observations: list[ArmObservation] = []
    ycrcb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2YCrCb)
    skin = cv2.inRange(ycrcb, (0, 133, 77), (255, 180, 135))

    for landmarks in hands:
        points = _landmark_pixels(landmarks, width, height)
        wrist = points[0]
        palm_width = max(12, int(np.linalg.norm(np.asarray(points[5]) - np.asarray(points[17]))))
        wrist_depth = _median_depth(depth_m, wrist, max(3, palm_width // 7))
        roi = np.zeros((height, width), dtype=np.uint8)
        cv2.fillConvexPoly(roi, _forearm_roi(points, (height, width)), 255)

        valid_depth = np.isfinite(depth_m) & (depth_m > 0.05)
        if wrist_depth is not None:
            close_depth = valid_depth & (np.abs(depth_m - wrist_depth) <= depth_tolerance_m)
            candidate = (roi > 0) & (close_depth | (skin > 0))
        else:
            candidate = (roi > 0) & (skin > 0)

        mask = np.uint8(candidate) * 255
        kernel_size = max(3, (palm_width // 8) | 1)
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (kernel_size, kernel_size)
        )
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            continue

        wrist_point = np.asarray(wrist, dtype=np.float32)

        def score(contour: np.ndarray) -> float:
            area = cv2.contourArea(contour)
            distance = abs(cv2.pointPolygonTest(contour, tuple(wrist_point), True))
            return area - distance * palm_width

        contour = max(contours, key=score)
        if cv2.contourArea(contour) < palm_width * palm_width * 0.25:
            continue
        cv2.drawContours(combined_mask, [contour], -1, 255, -1)
        observations.append(ArmObservation(contour, wrist, wrist_depth))

    return combined_mask, observations


def estimate_wrist_geometry_mask(
    wrist_landmark,
    frame_bgr: np.ndarray,
    depth_m: np.ndarray,
    camera_info,
    *,
    mirrored: bool = False,
    physical_roi_radius_m: float = 0.25,
    wrist_search_radius_px: int = 18,
    minimum_depth_m: float = 0.10,
    maximum_depth_m: float = 2.0,
) -> WristGeometryMaskEstimate:
    """Build a direction-free 3D component around one wrist landmark.

    The API intentionally accepts one landmark rather than the MediaPipe hand
    landmark collection.  Consequently palm landmarks, palm width,
    handedness, and a palm-to-wrist direction cannot enter this shadow mask.
    RGB contributes only a direction-free skin candidate; connectivity and
    the physical ROI are evaluated from registered depth geometry.
    """

    if depth_m.ndim == 2:
        empty_mask = np.zeros(depth_m.shape, dtype=np.uint8)
    elif frame_bgr.ndim >= 2:
        empty_mask = np.zeros(frame_bgr.shape[:2], dtype=np.uint8)
    else:
        empty_mask = np.zeros((0, 0), dtype=np.uint8)

    def rejected(reason: str) -> WristGeometryMaskEstimate:
        return WristGeometryMaskEstimate(
            mask=empty_mask,
            mask_area_px=0,
            connected_point_count=0,
            wrist_seed_pixel=None,
            wrist_seed_point_m=None,
            physical_roi_radius_m=float(physical_roi_radius_m),
            contains_palm_region=None,
            palm_region_reason="UNOBSERVABLE_WITH_WRIST_ONLY_INPUT",
            reason=reason,
        )

    if (
        wrist_landmark is None
        or camera_info is None
        or frame_bgr.ndim != 3
        or frame_bgr.shape[:2] != depth_m.shape
        or depth_m.size == 0
    ):
        return rejected("INVALID_INPUT")
    try:
        wrist_x = float(wrist_landmark.x)
        wrist_y = float(wrist_landmark.y)
        calibration_width = int(camera_info.width)
        calibration_height = int(camera_info.height)
        calibration = camera_info.k
        fx = float(calibration[0])
        fy = float(calibration[4])
        cx = float(calibration[2])
        cy = float(calibration[5])
    except (AttributeError, IndexError, TypeError, ValueError):
        return rejected("INVALID_INPUT")
    if (
        not math.isfinite(wrist_x + wrist_y + fx + fy + cx + cy)
        or calibration_width <= 0
        or calibration_height <= 0
        or fx <= 0.0
        or fy <= 0.0
        or physical_roi_radius_m <= 0.0
    ):
        return rejected("INVALID_INPUT")

    height, width = depth_m.shape
    wrist_pixel = np.asarray((
        np.clip(wrist_x, 0.0, 1.0) * (width - 1),
        np.clip(wrist_y, 0.0, 1.0) * (height - 1),
    ), dtype=np.float64)
    valid_depth = (
        np.isfinite(depth_m)
        & (depth_m >= minimum_depth_m)
        & (depth_m <= maximum_depth_m)
    )

    # A fixed, isotropic color gate is intentionally independent of hand
    # direction and size.  It prevents a depth-continuous supporting surface
    # from automatically becoming part of the wrist component.
    ycrcb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2YCrCb)
    skin = cv2.inRange(ycrcb, (0, 133, 77), (255, 180, 135)) > 0
    skin = cv2.morphologyEx(
        np.uint8(skin) * 255,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)),
        iterations=1,
    ) > 0

    rows, columns = np.indices(depth_m.shape, dtype=np.float64)
    wrist_distance_px_sq = (
        np.square(columns - wrist_pixel[0])
        + np.square(rows - wrist_pixel[1])
    )
    seed_window = wrist_distance_px_sq <= float(max(1, wrist_search_radius_px) ** 2)
    seed_candidates = seed_window & valid_depth & skin
    if not np.any(seed_candidates):
        # Color is only a candidate gate, not permission to invent a wrist
        # depth.  A valid depth seed may bridge a small local skin-mask hole.
        seed_candidates = seed_window & valid_depth
    seed_rows, seed_columns = np.nonzero(seed_candidates)
    if seed_rows.size == 0:
        return rejected("WRIST_SEED_UNSUPPORTED")
    seed_depths = depth_m[seed_rows, seed_columns].astype(np.float64)
    median_seed_depth = float(np.median(seed_depths))
    depth_consistent = np.abs(seed_depths - median_seed_depth) <= 0.05
    if np.any(depth_consistent):
        seed_rows = seed_rows[depth_consistent]
        seed_columns = seed_columns[depth_consistent]
    seed_distances = wrist_distance_px_sq[seed_rows, seed_columns]
    seed_choice = int(np.argmin(seed_distances))
    seed_row = int(seed_rows[seed_choice])
    seed_column = int(seed_columns[seed_choice])

    scale_x = calibration_width / float(width)
    scale_y = calibration_height / float(height)
    deprojection_columns = width - 1.0 - columns if mirrored else columns
    u_info = (deprojection_columns + 0.5) * scale_x - 0.5
    v_info = (rows + 0.5) * scale_y - 0.5
    z_camera = depth_m.astype(np.float64)
    x_camera = (u_info - cx) * z_camera / fx
    y_camera = (v_info - cy) * z_camera / fy
    seed_point = np.asarray((
        x_camera[seed_row, seed_column],
        y_camera[seed_row, seed_column],
        z_camera[seed_row, seed_column],
    ), dtype=np.float64)
    if not np.all(np.isfinite(seed_point)):
        return rejected("WRIST_DEPTH_INVALID")

    physical_distance_sq = (
        np.square(x_camera - seed_point[0])
        + np.square(y_camera - seed_point[1])
        + np.square(z_camera - seed_point[2])
    )
    candidate = (
        valid_depth
        & skin
        & (physical_distance_sq <= physical_roi_radius_m ** 2)
    )
    # Always retain the validated seed so a one-pixel color dropout does not
    # make a geometrically supported wrist impossible to start from.
    candidate[seed_row, seed_column] = True

    connected = np.zeros(depth_m.shape, dtype=bool)
    connected[seed_row, seed_column] = True
    queue = deque([(seed_row, seed_column)])
    neighbor_offsets = (
        (-1, -1), (0, -1), (1, -1),
        (-1, 0), (1, 0),
        (-1, 1), (0, 1), (1, 1),
    )
    while queue:
        row, column = queue.popleft()
        point_x = x_camera[row, column]
        point_y = y_camera[row, column]
        point_z = z_camera[row, column]
        for delta_column, delta_row in neighbor_offsets:
            neighbor_column = column + delta_column
            neighbor_row = row + delta_row
            if not (0 <= neighbor_column < width and 0 <= neighbor_row < height):
                continue
            if connected[neighbor_row, neighbor_column] or not candidate[
                neighbor_row, neighbor_column
            ]:
                continue
            neighbor_z = z_camera[neighbor_row, neighbor_column]
            edge_threshold = max(0.006, 0.012 * min(point_z, neighbor_z))
            edge_distance = math.sqrt(
                (x_camera[neighbor_row, neighbor_column] - point_x) ** 2
                + (y_camera[neighbor_row, neighbor_column] - point_y) ** 2
                + (neighbor_z - point_z) ** 2
            )
            if edge_distance > edge_threshold:
                continue
            connected[neighbor_row, neighbor_column] = True
            queue.append((neighbor_row, neighbor_column))

    connected_point_count = int(np.count_nonzero(connected))
    output_mask = np.uint8(connected) * 255
    reason = "NONE" if connected_point_count >= 80 else "CONNECTED_COMPONENT_TOO_SMALL"
    return WristGeometryMaskEstimate(
        mask=output_mask,
        mask_area_px=int(np.count_nonzero(output_mask)),
        connected_point_count=connected_point_count,
        wrist_seed_pixel=(seed_column, seed_row),
        wrist_seed_point_m=tuple(float(value) for value in seed_point),
        physical_roi_radius_m=float(physical_roi_radius_m),
        contains_palm_region=None,
        palm_region_reason="UNOBSERVABLE_WITH_WRIST_ONLY_INPUT",
        reason=reason,
    )
