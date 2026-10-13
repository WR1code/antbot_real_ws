from copy import deepcopy
from types import SimpleNamespace

import pytest

from meridian_hand_vision.persistent_forearm_validity_shadow import (
    PersistentForearmValidityConfig,
    PersistentForearmValidityShadowState,
    update_persistent_forearm_validity_shadow,
)


def _objects(
    *,
    track_id=3,
    active_state="TRACKING",
    axis=(1.0, 0.0, 0.0),
    consecutive_hits=8,
    update_count=8,
    confidence=0.9,
    length=0.11,
    rms=0.002,
    sections=6,
    seed_valid=True,
    branch_associated=True,
):
    descriptor = SimpleNamespace(
        axis=axis,
        centroid_m=(0.08, 0.0, 0.75),
        near_endpoint_m=(0.03, 0.0, 0.75),
        far_endpoint_m=(0.14, 0.0, 0.75),
    )
    track = SimpleNamespace(
        descriptor=descriptor,
        current_branch_id=0 if branch_associated else None,
        consecutive_hits=consecutive_hits,
        confidence_v2=confidence,
        feature_ema=SimpleNamespace(update_count=update_count),
        last_lost_reason=("ESTIMATOR_NO_BRANCH" if not branch_associated else None),
        last_lost_detail={},
    )
    tracker_state = SimpleNamespace(tracks={track_id: track})
    branch = SimpleNamespace(
        branch_id=0,
        axis=SimpleNamespace(vector=axis),
        axis_length_m=length,
        rms_m=rms,
        valid_section_count=sections,
        valid=True,
        reason="NONE",
    )
    estimate = SimpleNamespace(
        direction=SimpleNamespace(vector=axis),
        branches=(branch,),
        selected_branch=0,
        reason="NONE",
    )
    tracked = SimpleNamespace(branch_id=0, track_id=track_id)
    tracking = SimpleNamespace(
        active_track_id=track_id,
        active_state=active_state,
        branches=((tracked,) if branch_associated else ()),
    )
    mask = SimpleNamespace(
        wrist_seed_point_m=((0.0, 0.0, 0.75) if seed_valid else None),
        connected_point_count=(1000 if seed_valid else 0),
        mask_area_px=(1000 if seed_valid else 0),
        reason=("NONE" if seed_valid else "WRIST_SEED_UNSUPPORTED"),
    )
    return estimate, tracking, tracker_state, mask


def _update(state, objects, index, *, rgb_offset_ns=0):
    stamp = 1_000_000_000 + index * 100_000_000
    return update_persistent_forearm_validity_shadow(
        state,
        *objects,
        timestamp_sec=index * 0.1,
        rgb_stamp_ns=stamp + rgb_offset_ns,
        depth_stamp_ns=stamp,
        frame_seq=index + 1,
        detection_valid=True,
        v1_primary_axis=(0.0, 1.0, 0.0),
        actual_selector_axis=(0.0, 1.0, 0.0),
        experiment_phase="STATIC",
    )


def test_track_requires_temporal_acquisition_before_valid():
    state = PersistentForearmValidityShadowState()
    result = None
    for index in range(8):
        objects = _objects(
            consecutive_hits=index + 1,
            update_count=index + 1,
            confidence=0.9,
        )
        result = _update(state, objects, index)
        if index < 7:
            assert result["v2_2_shadow_validity"] == "ACQUIRING"

    assert result["v2_2_shadow_validity"] == "VALID"
    assert result["v2_2_shadow_direction"] == (1.0, 0.0, 0.0)
    assert result["statistics"]["first_valid_establishment_ms"] == pytest.approx(700.0)
    assert result["eligible_for_robot_target"] is False


def test_active_missing_is_stale_display_only_after_valid():
    state = PersistentForearmValidityShadowState()
    for index in range(8):
        result = _update(
            state,
            _objects(consecutive_hits=index + 1, update_count=index + 1),
            index,
        )
    assert result["v2_2_shadow_validity"] == "VALID"

    missing = _objects(
        active_state="ACTIVE_MISSING",
        consecutive_hits=0,
        branch_associated=False,
    )
    result = _update(state, missing, 8)
    assert result["v2_2_shadow_validity"] == "STALE"
    assert result["overlay_direction"] == (1.0, 0.0, 0.0)
    assert result["v2_2_shadow_direction"] is None
    assert "HISTORICAL_DIRECTION_DISPLAY_ONLY" in result["v2_2_reject_reasons"]


def test_v2_1_candidate_waiting_for_track_is_acquiring_not_valid():
    state = PersistentForearmValidityShadowState()
    estimate, tracking, tracker_state, mask = _objects()
    tracking.active_track_id = None
    tracking.active_state = "WAITING_FOR_CONFIRMATION"
    tracking.branches = ()
    tracker_state.tracks = {}
    result = _update(state, (estimate, tracking, tracker_state, mask), 0)
    assert result["v2_2_overlay_source"] == "V2.1_AXIS"
    assert result["v2_2_shadow_validity"] == "ACQUIRING"
    assert result["v2_2_shadow_direction"] is None


def test_track_id_switch_reenters_acquiring_then_reconfirms():
    state = PersistentForearmValidityShadowState()
    for index in range(8):
        _update(
            state,
            _objects(consecutive_hits=index + 1, update_count=index + 1),
            index,
        )

    switched = _objects(track_id=9, consecutive_hits=1, update_count=1)
    result = _update(state, switched, 8)
    assert result["v2_2_shadow_validity"] == "ACQUIRING"
    assert result["v2_2_track_id"] == 9
    assert result["statistics"]["track_switch_count"] == 1

    for index in range(9, 16):
        hits = index - 7
        result = _update(
            state,
            _objects(track_id=9, consecutive_hits=hits, update_count=hits),
            index,
        )
    assert result["v2_2_shadow_validity"] == "VALID"


def test_depth_seed_loss_invalidates_current_measurement():
    state = PersistentForearmValidityShadowState()
    result = _update(state, _objects(seed_valid=False), 0)
    assert result["v2_2_shadow_validity"] == "INVALID"
    assert "WRIST_SEED_VALID" in result["v2_2_reject_reasons"]
    assert result["v2_2_shadow_direction"] is None


def test_timestamp_mismatch_and_regression_are_rejected():
    state = PersistentForearmValidityShadowState()
    mismatch = _update(state, _objects(), 0, rgb_offset_ns=80_000_000)
    assert mismatch["v2_2_shadow_validity"] == "INVALID"
    assert "SAMPLE_TIME_CONSISTENT" in mismatch["v2_2_reject_reasons"]

    normal = _update(state, _objects(), 2)
    assert normal["temporal"]["timestamp_monotonic"]
    regressed = update_persistent_forearm_validity_shadow(
        state,
        *_objects(),
        timestamp_sec=0.3,
        rgb_stamp_ns=1_100_000_000,
        depth_stamp_ns=1_100_000_000,
        detection_valid=True,
    )
    assert not regressed["temporal"]["timestamp_monotonic"]
    assert "SAMPLE_TIME_CONSISTENT" in regressed["v2_2_reject_reasons"]


def test_large_direction_jump_is_rejected():
    state = PersistentForearmValidityShadowState()
    for index in range(8):
        _update(
            state,
            _objects(consecutive_hits=index + 1, update_count=index + 1),
            index,
        )
    jumped = _objects(axis=(0.0, 1.0, 0.0), consecutive_hits=9, update_count=9)
    result = _update(state, jumped, 8)
    assert result["v2_2_shadow_validity"] == "INVALID"
    assert "DIRECTION_DELTA_P95" in result["v2_2_reject_reasons"]
    assert "SINGLE_DIRECTION_DELTA" in result["v2_2_reject_reasons"]


def test_shadow_does_not_mutate_estimator_or_tracker_objects():
    state = PersistentForearmValidityShadowState(
        config=PersistentForearmValidityConfig()
    )
    objects = _objects()
    before = deepcopy(objects)
    _update(state, objects, 0)
    assert objects[0].direction.vector == before[0].direction.vector
    assert objects[1].active_track_id == before[1].active_track_id
    assert objects[2].tracks[3].descriptor.axis == before[2].tracks[3].descriptor.axis
