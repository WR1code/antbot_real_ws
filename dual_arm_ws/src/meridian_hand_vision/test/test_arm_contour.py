from types import SimpleNamespace

import cv2
import numpy as np

from meridian_hand_vision.arm_contour import estimate_arm_contours


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
