from copy import deepcopy
from types import SimpleNamespace

import numpy as np

from meridian_hand_vision.pulse_region import (
    CameraIntrinsics,
    WristGeometryBranchTrackerState,
    estimate_forearm_axis_wrist_geometry,
    update_wrist_geometry_branch_tracks,
)
from meridian_hand_vision.wrist_input_robustness import (
    WristTemporalShadowState,
    update_wrist_input_robustness_shadow,
)


def _point(x=0.5, y=0.5):
    return SimpleNamespace(x=x, y=y, z=0.0)


def _fake_pipeline(
    *,
    wrist=None,
    mask=None,
    branch_length=0.12,
    branch_reason="NONE",
    branch_valid=True,
    estimator_reason="NONE",
    seed_valid=True,
):
    depth = np.full((120, 160), 0.75, dtype=np.float32)
    intrinsics = CameraIntrinsics(300.0, 300.0, 80.0, 60.0, 160, 120)
    wrist = wrist or _point()
    raw_pixel = (
        int(round(wrist.x * (depth.shape[1] - 1))),
        int(round(wrist.y * (depth.shape[0] - 1))),
    )
    if mask is None:
        mask = np.zeros_like(depth, dtype=np.uint8)
        mask[35:105, 55:106] = 255
    seed_point = (
        ((raw_pixel[0] - intrinsics.cx) * 0.75 / intrinsics.fx),
        ((raw_pixel[1] - intrinsics.cy) * 0.75 / intrinsics.fy),
        0.75,
    )
    mask_estimate = SimpleNamespace(
        mask=mask,
        mask_area_px=int(np.count_nonzero(mask)),
        connected_point_count=int(np.count_nonzero(mask)),
        wrist_seed_pixel=raw_pixel if seed_valid else None,
        wrist_seed_point_m=seed_point if seed_valid else None,
        reason="NONE" if seed_valid else "WRIST_SEED_UNSUPPORTED",
    )
    features = SimpleNamespace(
        effective_length_m=branch_length,
        continuous_span_m=branch_length * 0.85,
    )
    branch = SimpleNamespace(
        branch_id=0,
        features=features,
        valid_section_count=7,
        inlier_section_count=6,
        reason=branch_reason,
        valid=branch_valid,
    )
    estimate = SimpleNamespace(
        branches=(branch,), selected_branch=0, reason=estimator_reason
    )
    tracked_branch = SimpleNamespace(
        branch_id=0,
        track_id=12,
        consecutive_hits=8,
        valid=branch_valid,
    )
    tracking = SimpleNamespace(
        branches=(tracked_branch,), active_track_id=12
    )
    return wrist, depth, intrinsics, mask_estimate, estimate, tracking


def _update(state, inputs, timestamp):
    return update_wrist_input_robustness_shadow(
        state, *inputs, timestamp
    )


def test_normal_wrist_motion_stays_diagnostic_normal():
    state = WristTemporalShadowState()
    result = None
    for index in range(10):
        inputs = _fake_pipeline(wrist=_point(0.50 + index * 0.0005, 0.50))
        result = _update(state, inputs, index * 0.1)

    assert result["shadow_only"]
    assert result["wrist_temporal"]["filter_state"] == "TRACKING"
    assert result["wrist_temporal"]["raw_wrist_available"]
    assert result["mask_robustness"]["status"] == "MASK_STABLE"
    assert result["branch_survival"]["branch_reason"] == "NONE"
    assert result["attribution"]["root_cause"] == "F_UNKNOWN"


def test_wrist_jump_is_attributed_to_wrist_detection():
    state = WristTemporalShadowState()
    for index in range(4):
        _update(state, _fake_pipeline(), index * 0.1)
    shifted_mask = np.zeros((120, 160), dtype=np.uint8)
    shifted_mask[35:105, 95:146] = 255
    result = _update(
        state,
        _fake_pipeline(wrist=_point(0.75, 0.50), mask=shifted_mask),
        0.4,
    )

    temporal = result["wrist_temporal"]
    assert temporal["filter_state"] == "RESET_LARGE_JUMP"
    assert temporal["threshold_flags"]["pixel_jump"]
    assert result["attribution"]["root_cause"] == "A_WRIST_DETECTION"


def test_mask_area_collapse_is_attributed_to_mask():
    state = WristTemporalShadowState()
    for index in range(8):
        _update(state, _fake_pipeline(), index * 0.1)
    small_mask = np.zeros((120, 160), dtype=np.uint8)
    small_mask[55:65, 75:85] = 255
    result = None
    for index in range(2):
        result = _update(
            state, _fake_pipeline(mask=small_mask), 0.8 + index * 0.1
        )

    assert result["mask_robustness"]["area_ratio_to_baseline"] < 0.1
    assert result["mask_robustness"]["status"] == "MASK_COLLAPSED"
    assert result["attribution"]["root_cause"] == "B_MASK"


def test_branch_length_failure_is_attributed_to_branch_extraction():
    state = WristTemporalShadowState()
    for index in range(8):
        _update(state, _fake_pipeline(), index * 0.1)
    result = None
    for index in range(2):
        result = _update(
            state,
            _fake_pipeline(
                branch_length=0.035,
                branch_reason="BRANCH_TOO_SHORT",
                branch_valid=False,
                estimator_reason="NO_VALID_BRANCH",
            ),
            0.8 + index * 0.1,
        )

    assert result["branch_survival"]["effective_length_m"] == 0.035
    assert result["branch_survival"]["branch_reason"] == "BRANCH_TOO_SHORT"
    assert result["attribution"]["root_cause"] == "C_BRANCH_EXTRACTION"


def test_depth_seed_failure_is_separate_from_wrist_detection():
    state = WristTemporalShadowState()
    for index in range(4):
        _update(state, _fake_pipeline(), index * 0.1)
    result = _update(
        state, _fake_pipeline(seed_valid=False), 0.4
    )

    assert result["wrist_temporal"]["raw_wrist_available"]
    assert not result["wrist_temporal"]["depth_seed_valid"]
    assert result["attribution"]["root_cause"] == "D_DEPTH_SEED"


def test_shadow_call_does_not_mutate_v2_estimate_tracker_or_scores():
    hand = [_point(0.5, 0.42) for _ in range(21)]
    depth = np.full((240, 320), 0.75, dtype=np.float32)
    mask = np.zeros((240, 320), dtype=np.uint8)
    mask[106:230, 140:181] = 255
    intrinsics = CameraIntrinsics(500.0, 500.0, 160.0, 120.0, 320, 240)
    estimate = estimate_forearm_axis_wrist_geometry(
        hand, mask, depth, intrinsics
    )
    tracker_state = WristGeometryBranchTrackerState(
        minimum_confirmation_frames=2
    )
    tracked = update_wrist_geometry_branch_tracks(
        estimate, tracker_state, timestamp_sec=0.0
    )
    estimate_before = deepcopy(estimate)
    tracked_before = deepcopy(tracked)
    tracker_before = deepcopy(tracker_state)
    fake_mask = SimpleNamespace(
        mask=mask,
        mask_area_px=int(np.count_nonzero(mask)),
        connected_point_count=int(np.count_nonzero(mask)),
        wrist_seed_pixel=(160, 101),
        wrist_seed_point_m=(0.0, -0.0285, 0.75),
        reason="NONE",
    )

    result = update_wrist_input_robustness_shadow(
        WristTemporalShadowState(),
        hand[0],
        depth,
        intrinsics,
        fake_mask,
        estimate,
        tracked,
        0.0,
    )

    assert result["shadow_only"]
    assert estimate == estimate_before
    assert tracked == tracked_before
    assert tracker_state == tracker_before
    assert tracked.tracks[0].forearm_score_v2 == (
        tracked_before.tracks[0].forearm_score_v2
    )
    assert tracked.tracks[0].topology_score == (
        tracked_before.tracks[0].topology_score
    )
    assert tracked.tracks[0].depth_surface_score == (
        tracked_before.tracks[0].depth_surface_score
    )
