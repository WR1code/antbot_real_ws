from __future__ import annotations

from dataclasses import dataclass

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
