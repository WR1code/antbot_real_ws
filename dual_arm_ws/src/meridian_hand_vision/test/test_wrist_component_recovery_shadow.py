from types import SimpleNamespace

import cv2
import numpy as np

from meridian_hand_vision.arm_contour import estimate_wrist_geometry_mask
from meridian_hand_vision.wrist_component_recovery_shadow import (
    add_branch_comparison,
    estimate_wrist_component_recovery_shadow,
)


def _point(x=0.5, y=0.5):
    return SimpleNamespace(x=x, y=y, z=0.0)


def _camera(width=320, height=240):
    return SimpleNamespace(
        width=width, height=height,
        k=(500.0, 0.0, width / 2, 0.0, 500.0, height / 2, 0.0, 0.0, 1.0),
    )


def _scene_with_isolated_nearest_seed():
    height, width = 240, 320
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    depth = np.full((height, width), 2.0, dtype=np.float32)
    # This BGR color lies in the production YCrCb skin interval.
    skin_color = (80, 120, 180)
    cv2.circle(frame, (160, 120), 2, skin_color, -1)
    depth[118:123, 158:163] = 0.75
    # A well-supported wrist-near forearm component starts 10 px away.
    cv2.rectangle(frame, (170, 35), (194, 220), skin_color, -1)
    depth[35:221, 170:195] = 0.75
    return frame, depth


def test_shadow_recovers_supported_component_when_nearest_seed_is_isolated():
    frame, depth = _scene_with_isolated_nearest_seed()
    production = estimate_wrist_geometry_mask(
        _point(), frame, depth, _camera()
    )
    production_mask_before = production.mask.copy()
    production_diagnostics_before = production.diagnostics()
    assert production.connected_point_count < 80

    shadow = estimate_wrist_component_recovery_shadow(
        _point(), frame, depth, _camera(), production
    )

    assert shadow.diagnostics()["shadow_only"] is True
    assert shadow.diagnostics()["selected_connected_point_count"] > 3000
    assert shadow.diagnostics()["component_count"] >= 2
    assert production.connected_point_count < shadow.diagnostics()[
        "selected_connected_point_count"
    ]
    assert np.array_equal(production.mask, production_mask_before)
    assert production.diagnostics() == production_diagnostics_before


def test_shadow_does_not_select_distant_largest_skin_component():
    height, width = 240, 320
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    depth = np.full((height, width), 2.0, dtype=np.float32)
    cv2.rectangle(frame, (5, 5), (120, 100), (80, 120, 180), -1)
    depth[5:101, 5:121] = 0.75
    production = estimate_wrist_geometry_mask(
        _point(), frame, depth, _camera()
    )

    shadow = estimate_wrist_component_recovery_shadow(
        _point(), frame, depth, _camera(), production
    )

    assert shadow.diagnostics()["reason"] == "NO_WRIST_NEAR_SKIN_DEPTH"
    assert shadow.diagnostics()["selected_connected_point_count"] == 0


def test_shadow_does_not_cross_depth_discontinuity():
    height, width = 240, 320
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    depth = np.full((height, width), 2.0, dtype=np.float32)
    cv2.rectangle(frame, (150, 20), (190, 220), (80, 120, 180), -1)
    depth[20:221, 150:170] = 0.75
    depth[20:221, 171:191] = 0.90
    production = estimate_wrist_geometry_mask(
        _point(), frame, depth, _camera()
    )
    shadow = estimate_wrist_component_recovery_shadow(
        _point(), frame, depth, _camera(), production
    )
    selected = shadow.mask > 0

    assert shadow.diagnostics()["selected_connected_point_count"] < 5000
    assert np.count_nonzero(selected[:, 171:191]) == 0


def test_branch_comparison_is_diagnostic_only():
    class Estimate:
        direction = None
        def diagnostics(self):
            return {"reason": "NO_VALID_BRANCH", "branch_count": 0}

    production = Estimate()
    candidate = Estimate()
    before = production.diagnostics().copy()
    result = add_branch_comparison({}, production, candidate)

    assert result["production_success"] is False
    assert result["candidate_success"] is False
    assert production.diagnostics() == before
