from copy import deepcopy
from types import SimpleNamespace

import numpy as np

from meridian_hand_vision.invalid_input_shadow import (
    InvalidInputShadowState,
    update_invalid_input_shadow,
)


class Landmark:
    x = 0.5
    y = 0.5


def _hands(present=True):
    return [[Landmark()]] if present else []


def _wrist(*, available=True, seed=True):
    return {
        "wrist_temporal": {
            "raw_wrist_available": available,
            "raw_wrist_pixel": (20.0, 20.0) if available else None,
            "raw_wrist_3d": (0.0, 0.0, 0.7) if available else None,
            "raw_wrist_reason": "NONE" if available else "NO_DETECTION",
            "depth_seed_valid": seed,
        }
    }


def _mask(area=100, connected=100, reason="NONE"):
    return SimpleNamespace(
        mask_area_px=area,
        connected_point_count=connected,
        reason=reason,
    )


def _axis(reason="INVALID_INPUT", branches=()):
    return SimpleNamespace(reason=reason, branches=tuple(branches))


def _tracking(active=3, state="ACTIVE_MISSING"):
    return SimpleNamespace(active_track_id=active, active_state=state)


def _update(
    state,
    *,
    hands=None,
    wrist=None,
    depth=None,
    mask=None,
    axis=None,
    candidate_count=0,
    tracking=None,
    frame=1,
):
    return update_invalid_input_shadow(
        state,
        _hands() if hands is None else hands,
        0.95,
        np.full((40, 40), 0.7, dtype=np.float32) if depth is None else depth,
        _mask() if mask is None else mask,
        _axis() if axis is None else axis,
        _tracking() if tracking is None else tracking,
        _wrist() if wrist is None else wrist,
        {"candidate_count": candidate_count},
        frame_index=frame,
    )


def test_no_hand_is_wrist_no_detection():
    result = _update(
        InvalidInputShadowState(),
        hands=[],
        wrist=_wrist(available=False, seed=False),
        mask=_mask(0, 0, "INVALID_INPUT"),
    )

    assert result["classification"] == "WRIST_NO_DETECTION"
    assert not result["hand_detection"]["hand_detected"]
    assert result["temporal_attribution"]["first_failure_stage"] == "HAND"


def test_missing_depth_is_depth_seed_failure():
    result = _update(
        InvalidInputShadowState(),
        wrist=_wrist(available=True, seed=False),
        depth=np.zeros((40, 40), dtype=np.float32),
        mask=_mask(0, 0, "WRIST_SEED_UNSUPPORTED"),
    )

    assert result["classification"] == "DEPTH_SEED_FAILURE"
    assert result["depth"]["depth_roi_empty"]
    assert result["wrist"]["depth_seed_reason"] == "WRIST_SEED_UNSUPPORTED"


def test_empty_mask_is_distinguished_after_valid_seed():
    result = _update(
        InvalidInputShadowState(),
        mask=_mask(0, 0, "MASK_EMPTY"),
    )

    assert result["classification"] == "MASK_EMPTY"
    assert result["wrist"]["depth_seed_valid"]


def test_empty_connected_cloud_is_distinguished_from_mask():
    result = _update(
        InvalidInputShadowState(),
        mask=_mask(200, 0, "CONNECTED_COMPONENT_TOO_SMALL"),
        axis=_axis("CONNECTED_CLOUD_TOO_SMALL"),
    )

    assert result["classification"] == "CONNECTED_CLOUD_EMPTY"
    assert result["wrist_centered_mask"]["mask_area_px"] == 200


def test_cloud_without_branch_is_branch_no_candidate():
    result = _update(
        InvalidInputShadowState(),
        mask=_mask(300, 260),
        axis=_axis("NO_BRANCH_AFTER_WRIST_CUT"),
    )

    assert result["classification"] == "BRANCH_NO_CANDIDATE"
    assert result["branch"]["candidate_count"] == 0


def test_temporal_shadow_does_not_mutate_inputs_and_marks_track_expiry():
    state = InvalidInputShadowState()
    mask = _mask()
    axis = _axis("NONE", branches=(SimpleNamespace(branch_id=0),))
    tracking = _tracking(active=7, state="TRACKING")
    originals = deepcopy((mask, axis, tracking))
    first = _update(
        state,
        mask=mask,
        axis=axis,
        candidate_count=1,
        tracking=tracking,
        frame=10,
    )
    assert not first["invalid"]

    _update(
        state,
        hands=[],
        wrist=_wrist(available=False, seed=False),
        mask=_mask(0, 0, "INVALID_INPUT"),
        tracking=_tracking(active=7, state="ACTIVE_MISSING"),
        frame=11,
    )
    expired = _update(
        state,
        hands=[],
        wrist=_wrist(available=False, seed=False),
        mask=_mask(0, 0, "INVALID_INPUT"),
        tracking=_tracking(active=None, state="NO_ACTIVE_TRACK"),
        frame=12,
    )

    assert expired["temporal_attribution"]["track_failure_event"] == "TRACK_EXPIRED"
    assert expired["temporal_attribution"]["first_failure_frame"] == 11
    assert expired["shadow_only"]
    assert (mask, axis, tracking) == originals
