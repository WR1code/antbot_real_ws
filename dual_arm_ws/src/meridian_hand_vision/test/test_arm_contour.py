from types import SimpleNamespace

import cv2
import numpy as np

from meridian_hand_vision.arm_contour import (
    estimate_arm_contours,
    estimate_wrist_geometry_mask,
)


def _point(x: float, y: float) -> SimpleNamespace:
    return SimpleNamespace(x=x, y=y, z=0.0)


def test_depth_guided_arm_contour() -> None:
    height, width = 240, 320
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    depth = np.full((height, width), 2.0, dtype=np.float32)
    cv2.rectangle(frame, (120, 110), (200, 239), (80, 120, 180), -1)
    depth[110:240, 120:201] = 1.0

    landmarks = [_point(0.5, 0.52) for _ in range(21)]
    landmarks[0] = _point(0.5, 0.62)
    landmarks[5] = _point(0.42, 0.48)
    landmarks[9] = _point(0.5, 0.45)
    landmarks[13] = _point(0.55, 0.47)
    landmarks[17] = _point(0.60, 0.50)

    mask, observations = estimate_arm_contours(frame, depth, [landmarks])
    assert np.count_nonzero(mask) > 500
    assert len(observations) == 1
    assert observations[0].wrist_depth_m is not None
    assert abs(observations[0].wrist_depth_m - 1.0) < 1e-5


def test_wrist_geometry_mask_is_direction_free_and_keeps_both_sides() -> None:
    height, width = 240, 320
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    depth = np.full((height, width), 2.0, dtype=np.float32)
    cv2.rectangle(frame, (145, 10), (175, 230), (80, 120, 180), -1)
    depth[10:231, 145:176] = 0.75
    camera_info = SimpleNamespace(
        width=width,
        height=height,
        k=(500.0, 0.0, 160.0, 0.0, 500.0, 120.0, 0.0, 0.0, 1.0),
    )

    estimate = estimate_wrist_geometry_mask(
        _point(0.5, 0.5), frame, depth, camera_info
    )

    assert estimate.reason == "NONE"
    assert estimate.wrist_seed_pixel is not None
    assert estimate.connected_point_count == estimate.mask_area_px
    assert estimate.mask_area_px > 5000
    assert np.count_nonzero(estimate.mask[:100]) > 0
    assert np.count_nonzero(estimate.mask[140:]) > 0
    assert estimate.contains_palm_region is None
    assert estimate.palm_region_reason == "UNOBSERVABLE_WITH_WRIST_ONLY_INPUT"


def test_wrist_geometry_mask_rejects_missing_wrist_depth() -> None:
    height, width = 120, 160
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    depth = np.zeros((height, width), dtype=np.float32)
    camera_info = SimpleNamespace(
        width=width,
        height=height,
        k=(300.0, 0.0, 80.0, 0.0, 300.0, 60.0, 0.0, 0.0, 1.0),
    )

    estimate = estimate_wrist_geometry_mask(
        _point(0.5, 0.5), frame, depth, camera_info
    )

    assert estimate.reason == "WRIST_SEED_UNSUPPORTED"
    assert estimate.mask_area_px == 0
    assert not np.any(estimate.mask)
