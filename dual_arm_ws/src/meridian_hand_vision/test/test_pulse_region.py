from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from meridian_hand_vision.pulse_region import (
    CameraIntrinsics,
    ForearmSection,
    ForearmAxisSelectorState,
    PulseRegion,
    StableArmDirectionFilter,
    StablePulseFilter,
    StableSurfaceNormalFilter,
    SurfaceNormal,
    WristGeometryBranchTrackerState,
    deproject_pixel,
    compute_forearm_sections,
    estimate_pulse_region,
    estimate_pulse_region_diagnostic,
    estimate_arm_direction,
    estimate_forearm_axis_3d_diagnostic,
    estimate_forearm_axis_wrist_geometry,
    estimate_surface_normal,
    estimate_surface_patch_diagnostic,
    pulse_pixel,
    refine_forearm_axis,
    select_forearm_axis,
    update_wrist_geometry_branch_tracks,
)


def _point(x, y):
    return SimpleNamespace(x=x, y=y, z=0.0)


def _hand():
    points = [_point(0.50, 0.62) for _ in range(21)]
    points[0] = _point(0.50, 0.66)
    points[5] = _point(0.42, 0.50)
    points[9] = _point(0.49, 0.47)
    points[13] = _point(0.56, 0.49)
    points[17] = _point(0.62, 0.53)
    return points


def test_pulse_pixel_is_proximal_and_on_thumb_side():
    target = pulse_pixel(_hand(), 1000, 1000)
    assert target[1] > 660
    assert target[0] < 500


def test_deprojection_uses_pinhole_intrinsics_and_unmirrors():
    intrinsics = CameraIntrinsics(500.0, 500.0, 320.0, 240.0, 640, 480)
    point = deproject_pixel((420, 240), 1.0, intrinsics, 640, 480)
    mirrored = deproject_pixel((219, 240), 1.0, intrinsics, 640, 480, mirrored=True)
    assert np.allclose(point, (0.2, 0.0, 1.0), atol=0.002)
    assert np.allclose(mirrored, point, atol=0.002)


def test_region_uses_registered_depth_and_camera_info():
    depth = np.full((1000, 1000), 0.75, dtype=np.float32)
    intrinsics = CameraIntrinsics(800.0, 800.0, 500.0, 500.0, 1000, 1000)
    region = estimate_pulse_region(_hand(), depth, intrinsics)
    assert region is not None
    assert region.point_m[2] == 0.75
    assert 0.006 <= region.radius_m <= 0.018


def test_region_diagnostic_reports_depth_rejection_and_raw_pixel():
    depth = np.zeros((1000, 1000), dtype=np.float32)
    intrinsics = CameraIntrinsics(800.0, 800.0, 500.0, 500.0, 1000, 1000)
    region, reason, details = estimate_pulse_region_diagnostic(
        _hand(), depth, intrinsics
    )

    assert region is None
    assert reason == "INVALID_DEPTH"
    assert details["raw_pixel_uv"] is not None
    assert details["raw_depth"] is None


def _sample(x=0.1):
    return PulseRegion((10, 10), (x, 0.2, 0.5), 0.01)


@pytest.mark.parametrize("rate_hz", [6, 13, 30])
def test_target_stabilization_has_no_duration_or_fps_gate(rate_hz):
    stable = StablePulseFilter()
    first_ready = None
    for index in range(2 * rate_hz + 1):
        timestamp = index / rate_hz
        result = stable.update(_sample(0.1 + (index % 3 - 1) * 0.0005), timestamp)
        if result is not None and first_ready is None:
            first_ready = timestamp
    assert first_ready is not None
    assert first_ready <= 2.0 / rate_hz + 1e-6
    metrics = stable.diagnostics(first_ready)
    assert metrics["stable_required_ms"] == 0.0
    assert metrics["vision_valid_samples"] >= 3
    assert metrics["spatial_jitter_mm"] < 8.0


def test_single_or_two_missing_frames_preserve_history_without_republishing():
    stable = StablePulseFilter()
    assert stable.update(_sample(), 0.0) is None
    assert stable.update(None, 0.1) is None
    assert stable.update(_sample(), 0.2) is None
    assert stable.update(None, 0.3) is None
    assert stable.update(None, 0.4) is None
    assert stable.update(_sample(), 0.5) is not None
    assert stable.reset_count == 0
    assert stable.diagnostics(0.5)["vision_valid_samples"] == 3


def test_pre_latch_timer_survives_quarter_second_camera_gap():
    stable = StablePulseFilter()
    assert stable.update(_sample(), 0.0) is None
    assert stable.update(_sample(), 0.15) is None
    assert stable.update(_sample(), 0.30) is not None
    before = stable.diagnostics(0.43)
    assert before["target_stable"]
    stable.observe_gap(0.68)
    during = stable.diagnostics(0.68)
    assert during["target_stable"]
    assert during["stable_reset_count"] == 0
    assert stable.update(_sample(), 0.69) is not None
    after = stable.diagnostics(0.69)
    assert after["stable_latched"]
    assert after["stable_reset_count"] == 0


def test_elapsed_time_alone_cannot_pass_without_three_valid_samples():
    stable = StablePulseFilter()
    assert stable.update(_sample(), 0.0) is None
    assert stable.update(_sample(), 0.6) is None
    assert stable.diagnostics(0.6)["vision_valid_samples"] == 2


def test_spatial_jitter_p95_is_the_acquisition_gate():
    stable = StablePulseFilter()
    stable.update(_sample(0.1), 0.0)
    stable.update(_sample(0.1), 0.1)
    assert stable.update(_sample(0.112), 0.2) is None
    metrics = stable.diagnostics(0.2)
    assert metrics["jitter_p95_mm"] > 5.0
    assert stable.reset_count == 0  # Soft variation is not a hard jump.


def test_acquisition_transient_does_not_reset_but_identity_and_timeout_do():
    stable = StablePulseFilter()
    stable.update(_sample(), 0.0, "Left")
    stable.update(_sample(), 0.2, "Left")
    assert stable.update(_sample(0.2), 0.3, "Left") is None
    assert stable.diagnostics(0.3)["last_stable_reset_reason"] == "NONE"
    assert stable.diagnostics(0.3)["vision_valid_samples"] == 3
    stable.update(_sample(0.2), 0.4, "Right")
    assert stable.diagnostics(0.4)["last_stable_reset_reason"] == "HAND_IDENTITY_CHANGED"
    assert stable.update(None, 2.5) is None
    assert stable.diagnostics(2.5)["last_stable_reset_reason"] == "TARGET_HARD_TIMEOUT"
    assert stable.reset_count == 2


def _latched_filter():
    stable = StablePulseFilter()
    for index in range(6):
        result = stable.update(_sample(), index * 0.1, "Left")
    assert result is not None
    assert stable.stable_latched
    return stable


@pytest.mark.parametrize(
    "gap_age,state", [
        (0.4, "TARGET_STABLE"),
        (0.8, "TARGET_TEMPORARILY_STALE"),
        (1.8, "WAIT_TARGET_RECOVERY"),
    ],
)
def test_data_gap_preserves_latch_frozen_target_and_timer(gap_age, state):
    stable = _latched_filter()
    frozen = stable.frozen_region
    stable_since = stable._stable_since
    assert stable.update(None, 0.5 + gap_age) is None
    metrics = stable.diagnostics(0.5 + gap_age)
    assert metrics["target_tracking_state"] == state
    assert metrics["stable_latched"]
    assert metrics["frozen_target"] == frozen.point_m
    assert stable._stable_since == stable_since
    assert stable.reset_count == 0


@pytest.mark.parametrize("gap_age", [0.4, 0.8, 1.8])
def test_same_target_reacquires_immediately_without_old_target_republish(gap_age):
    stable = _latched_filter()
    stable.observe_gap(0.5 + gap_age)
    assert stable.diagnostics(0.5 + gap_age)["stable_latched"]
    result = stable.update(_sample(0.1028), 0.5 + gap_age)
    assert result is not None
    assert result.point_m[0] == pytest.approx(0.1, abs=0.003)
    assert stable.diagnostics(0.5 + gap_age)["reacquire_result"] == "REACQUIRED_SAME_TARGET"
    assert stable.reset_count == 0


def test_8_to_15_mm_is_observed_without_reset_or_publish():
    stable = _latched_filter()
    assert stable.update(_sample(0.111), 1.2) is None
    metrics = stable.diagnostics(1.2)
    assert metrics["stable_latched"]
    assert metrics["reacquire_result"] == "OBSERVE_SPATIAL_DRIFT"
    assert stable.reset_count == 0
    assert stable.update(_sample(0.102), 1.3) is not None
    assert stable.reset_count == 0


def test_more_than_15_mm_is_target_change_and_hard_timeout_is_only_gap_reset():
    stable = _latched_filter()
    assert stable.update(_sample(0.118), 0.6) is None
    assert stable.diagnostics(0.6)["reacquire_result"] == "TARGET_CHANGED"
    assert stable.last_reset_reason == "TARGET_SPATIAL_DRIFT"
    stable = _latched_filter()
    stable.observe_gap(2.51)
    metrics = stable.diagnostics(2.51)
    assert not metrics["stable_latched"]
    assert metrics["target_tracking_state"] == "TARGET_LOST"
    assert stable.last_reset_reason == "TARGET_HARD_TIMEOUT"


def test_early_acquisition_outliers_age_out_without_reentering_view():
    stable = StablePulseFilter()
    assert stable.update(_sample(-0.0368), 0.00, "Left") is None
    assert stable.update(_sample(0.060), 0.10, "Left") is None
    for timestamp, x in ((0.20, 0.014), (0.35, 0.015), (0.50, 0.013)):
        assert stable.update(_sample(x), timestamp, "Left") is None
    result = stable.update(_sample(0.014), 0.72, "Left")
    assert result is not None
    metrics = stable.diagnostics(0.72)
    assert metrics["acquisition_state"] == "TARGET_STABLE"
    assert metrics["window_valid_samples"] >= 3
    assert metrics["jitter_p95_mm"] <= 5.0
    assert metrics["stable_reset_count"] == 0
    assert metrics["window_center_xyz"][0] == pytest.approx(0.014, abs=0.001)


def test_stable_hysteresis_holds_brief_excursion_above_8mm():
    stable = _latched_filter()
    assert stable.update(_sample(0.114), 0.60) is None
    assert stable.diagnostics(0.60)["stable_latched"]
    assert stable.update(_sample(0.114), 0.81) is None
    assert not stable.diagnostics(0.81)["stable_latched"]


def test_arm_direction_uses_registered_3d_palm_to_wrist_axis():
    depth = np.full((1000, 1000), 0.75, dtype=np.float32)
    intrinsics = CameraIntrinsics(800.0, 800.0, 500.0, 500.0, 1000, 1000)
    result = estimate_arm_direction(_hand(), depth, intrinsics)
    assert result is not None
    assert np.linalg.norm(result.vector) == pytest.approx(1.0)
    assert result.vector[1] > 0.9


def test_arm_direction_filter_treats_opposite_vectors_as_same_axis():
    from meridian_hand_vision.pulse_region import ArmDirection

    stable = StableArmDirectionFilter(window_size=3, maximum_angle_deg=5.0)
    assert stable.update(ArmDirection((2.0, 0.0, 0.0))) is None
    assert stable.update(ArmDirection((-1.0, 0.0, 0.0))) is None
    result = stable.update(ArmDirection((1.0, 0.01, 0.0)))
    assert result is not None
    assert result.vector[0] > 0.99


def _forearm_fixture():
    hand = [_point(0.50, 0.30) for _ in range(21)]
    hand[0] = _point(0.50, 0.42)
    hand[5] = _point(0.42, 0.30)
    hand[9] = _point(0.48, 0.28)
    hand[13] = _point(0.53, 0.29)
    hand[17] = _point(0.58, 0.31)
    depth = np.full((240, 320), 0.75, dtype=np.float32)
    mask = np.zeros((240, 320), dtype=np.uint8)
    mask[106:230, 140:181] = 255
    intrinsics = CameraIntrinsics(500.0, 500.0, 160.0, 120.0, 320, 240)
    return hand, mask, depth, intrinsics


def test_primary_forearm_axis_uses_mask_depth_sections_ransac_and_pca():
    hand, mask, depth, intrinsics = _forearm_fixture()
    fallback = estimate_arm_direction(hand, depth, intrinsics)
    result = estimate_forearm_axis_3d_diagnostic(
        hand, mask, depth, intrinsics, reference_direction=fallback
    )
    assert result.valid
    assert result.reason == "NONE"
    assert result.quality_level == "PRIMARY_STRONG"
    assert result.direction is not None
    assert result.direction.vector == pytest.approx((0.0, 1.0, 0.0), abs=1e-6)
    assert result.axis_length_m > 0.10
    assert result.rms_m < 1e-6
    assert result.inlier_section_count >= 4
    assert result.confidence >= 0.40


def test_wrist_geometry_shadow_recovers_forearm_without_palm_landmarks():
    hand, mask, depth, intrinsics = _forearm_fixture()

    class WristOnlyLandmarks:
        def __getitem__(self, index):
            if index != 0:
                raise AssertionError(f"V2 accessed forbidden landmark {index}")
            return hand[0]

    result = estimate_forearm_axis_wrist_geometry(
        WristOnlyLandmarks(), mask, depth, intrinsics
    )
    assert result.direction is not None
    assert result.quality == "V2_STRONG"
    assert result.branch_count == 1
    assert result.selected_branch == 0
    assert result.reason == "NONE"
    assert result.direction.vector == pytest.approx((0.0, 1.0, 0.0), abs=1e-6)
    assert result.axis_length_m > 0.10
    assert result.rms_m < 1e-6
    assert result.diagnostics()["v2_axis"] is not None


def test_wrist_geometry_branch_features_are_reported_without_changing_v2_result():
    hand, mask, depth, intrinsics = _forearm_fixture()
    result = estimate_forearm_axis_wrist_geometry(hand, mask, depth, intrinsics)

    assert result.quality == "V2_STRONG"
    assert result.selected_branch == 0
    assert len(result.branches) == 1
    features = result.branches[0].features
    assert features is not None
    assert features.effective_length_m > 0.08
    assert features.continuous_span_m > 0.05
    assert len(features.width_profile_m) >= 5
    assert all(width is None or width > 0.0 for width in features.width_profile_m)
    assert features.width_cv is not None
    assert 0.0 <= features.section_stability <= 1.0
    assert 0.0 <= features.continuity_score <= 1.0
    assert 0.0 <= features.branching_complexity <= 1.0
    assert features.descriptor.axis is not None


def test_parent_split_shadow_does_not_change_wrist_geometry_output():
    hand, mask, depth, intrinsics = _forearm_fixture()
    baseline = estimate_forearm_axis_wrist_geometry(
        hand, mask, depth, intrinsics
    )
    descriptor = baseline.branches[0].features.descriptor

    shadowed = estimate_forearm_axis_wrist_geometry(
        hand,
        mask,
        depth,
        intrinsics,
        previous_active_descriptor=descriptor,
    )

    assert shadowed.direction == baseline.direction
    assert shadowed.quality == baseline.quality
    assert shadowed.selected_branch == baseline.selected_branch
    assert shadowed.axis_length_m == baseline.axis_length_m
    assert shadowed.rms_m == baseline.rms_m
    assert shadowed.reason == baseline.reason
    assert shadowed.parent_split_shadow["shadow_only"]
    assert shadowed.parent_split_shadow["triggered_parent_count"] == 0


def test_wrist_geometry_branch_tracking_survives_per_frame_branch_id_changes():
    hand, mask, depth, intrinsics = _forearm_fixture()
    estimate = estimate_forearm_axis_wrist_geometry(hand, mask, depth, intrinsics)
    state = WristGeometryBranchTrackerState(minimum_confirmation_frames=2)

    first = update_wrist_geometry_branch_tracks(estimate, state)
    first_track_id = first.branches[0].track_id
    assert first_track_id is not None
    assert first.active_track_id is None
    assert first.active_state == "WAITING_FOR_CONFIRMATION"

    renumbered_branch = replace(estimate.branches[0], branch_id=7)
    renumbered_estimate = replace(
        estimate,
        selected_branch=7,
        branches=(renumbered_branch,),
    )
    second = update_wrist_geometry_branch_tracks(renumbered_estimate, state)

    assert second.branches[0].branch_id == 7
    assert second.branches[0].track_id == first_track_id
    assert second.branches[0].match_cost == pytest.approx(0.0, abs=1e-9)
    assert second.branches[0].confirmed
    assert second.active_track_id == first_track_id
    assert second.active_state == "TRACKING"


def test_track_forearm_likelihood_is_ema_shadow_without_active_selection():
    hand, mask, depth, intrinsics = _forearm_fixture()
    estimate = estimate_forearm_axis_wrist_geometry(hand, mask, depth, intrinsics)
    state = WristGeometryBranchTrackerState(minimum_confirmation_frames=2)

    tracked = None
    for index in range(10):
        tracked = update_wrist_geometry_branch_tracks(
            estimate, state, timestamp_sec=index * 0.1
        )

    assert tracked is not None
    assert len(tracked.tracks) == 1
    likelihood = tracked.tracks[0]
    assert likelihood.ema_feature["update_count"] == 10
    assert likelihood.ema_feature["length"] > 0.10
    assert likelihood.ema_feature["continuity"] > 0.8
    assert likelihood.ema_feature["axis_jitter_deg"] == pytest.approx(0.0)
    assert 0.0 <= likelihood.forearm_score <= 1.0
    assert likelihood.forearm_score > 0.7
    assert likelihood.confidence > 0.8
    diagnostics = tracked.diagnostics()
    assert diagnostics["active_track"] == likelihood.track_id
    assert diagnostics["active_forearm_track"] is None


def test_score_v2_radius_shape_penalizes_palm_like_expansion():
    hand, forearm_mask, depth, intrinsics = _forearm_fixture()
    palm_like_mask = np.zeros_like(forearm_mask)
    for row in range(106, 230):
        half_width = 12 if row < 145 else (42 if row < 185 else 10)
        palm_like_mask[row, 160 - half_width:161 + half_width] = 255

    scores = {}
    for name, mask in (("forearm", forearm_mask), ("palm", palm_like_mask)):
        estimate = estimate_forearm_axis_wrist_geometry(
            hand, mask, depth, intrinsics
        )
        state = WristGeometryBranchTrackerState(minimum_confirmation_frames=2)
        for index in range(12):
            tracked = update_wrist_geometry_branch_tracks(
                estimate, state, timestamp_sec=index * 0.1
            )
        scores[name] = tracked.tracks[0]

    forearm = scores["forearm"]
    palm = scores["palm"]
    assert len(forearm.radius_components["profile"]) == 8
    assert forearm.radius_components["shape_score"] > 0.8
    assert palm.radius_components["shape_score"] < 0.1
    assert palm.forearm_score_v2 < palm.forearm_score - 0.05
    assert forearm.forearm_score_v2 > palm.forearm_score_v2 + 0.25


def test_score_v2_axis_maturity_prevents_first_frame_full_credit():
    hand, mask, depth, intrinsics = _forearm_fixture()
    estimate = estimate_forearm_axis_wrist_geometry(hand, mask, depth, intrinsics)
    state = WristGeometryBranchTrackerState(minimum_confirmation_frames=2)

    first = update_wrist_geometry_branch_tracks(
        estimate, state, timestamp_sec=0.0
    ).tracks[0]
    assert first.axis_stability_raw is None
    assert first.axis_maturity == pytest.approx(0.0)
    assert first.axis_stability_effective == pytest.approx(0.5)
    assert first.confidence_v2 < 0.25

    for index in range(1, 12):
        mature = update_wrist_geometry_branch_tracks(
            estimate, state, timestamp_sec=index * 0.1
        ).tracks[0]
    assert mature.axis_stability_raw == pytest.approx(1.0)
    assert mature.axis_maturity == pytest.approx(1.0)
    assert mature.axis_stability_effective == pytest.approx(1.0)


def test_score_v2_robust_axis_window_recovers_after_early_noise():
    hand, mask, depth, intrinsics = _forearm_fixture()
    estimate = estimate_forearm_axis_wrist_geometry(hand, mask, depth, intrinsics)
    base_branch = estimate.branches[0]
    assert base_branch.features is not None
    state = WristGeometryBranchTrackerState(minimum_confirmation_frames=2)
    angles_deg = (0.0, 20.0, -20.0, 15.0, -15.0) + (0.0,) * 14

    for index, angle_deg in enumerate(angles_deg):
        angle = np.radians(angle_deg)
        descriptor = replace(
            base_branch.features.descriptor,
            axis=(float(np.sin(angle)), float(np.cos(angle)), 0.0),
        )
        branch = replace(
            base_branch,
            features=replace(base_branch.features, descriptor=descriptor),
        )
        tracked = update_wrist_geometry_branch_tracks(
            replace(estimate, branches=(branch,)),
            state,
            timestamp_sec=index * 0.1,
        )

    recovered = tracked.tracks[0]
    assert recovered.ema_feature["update_count"] == len(angles_deg)
    assert recovered.axis_jitter_robust_deg < 3.0
    assert recovered.axis_stability_effective > 0.85


def test_score_v2_best_second_margin_does_not_select_forearm_track():
    intrinsics = CameraIntrinsics(500.0, 500.0, 160.0, 120.0, 320, 240)
    depth = np.full((240, 320), 0.75, dtype=np.float32)
    mask = np.zeros((240, 320), dtype=np.uint8)
    mask[10:231, 150:171] = 255
    estimate = estimate_forearm_axis_wrist_geometry(
        [_point(0.50, 0.50)], mask, depth, intrinsics
    )
    state = WristGeometryBranchTrackerState(minimum_confirmation_frames=2)
    for index in range(3):
        tracked = update_wrist_geometry_branch_tracks(
            estimate, state, timestamp_sec=index * 0.1
        )

    diagnostics = tracked.diagnostics()
    margin = diagnostics["score_v2_shadow"]
    assert margin["candidate_count"] == 2
    assert margin["best_track_id"] is not None
    assert margin["second_track_id"] is not None
    assert margin["raw_margin"] is not None
    assert margin["reliable_margin"] is not None
    assert margin["reason"] == "SHADOW_ONLY"
    assert diagnostics["active_forearm_track"] is None


def test_topology_shadow_penalizes_rapid_palm_expansion():
    hand, forearm_mask, depth, intrinsics = _forearm_fixture()
    palm_like_mask = np.zeros_like(forearm_mask)
    for row in range(106, 230):
        half_width = 12 if row < 145 else (42 if row < 185 else 10)
        palm_like_mask[row, 160 - half_width:161 + half_width] = 255

    results = {}
    for name, mask in (("forearm", forearm_mask), ("palm", palm_like_mask)):
        estimate = estimate_forearm_axis_wrist_geometry(
            hand, mask, depth, intrinsics
        )
        topology = estimate.branches[0].features.topology
        state = WristGeometryBranchTrackerState(minimum_confirmation_frames=2)
        for index in range(12):
            tracked = update_wrist_geometry_branch_tracks(
                estimate, state, timestamp_sec=index * 0.1
            )
        results[name] = (topology, tracked.tracks[0])

    forearm_topology, forearm_track = results["forearm"]
    palm_topology, palm_track = results["palm"]
    assert forearm_topology.distal_branch_count == 1
    assert forearm_topology.bottleneck_ratio == pytest.approx(1.0)
    assert palm_topology.distal_branch_count >= 2
    assert palm_topology.growth_60mm > 1.0
    assert palm_topology.max_local_growth > 1.0
    assert palm_topology.growth_concentration > 0.7
    assert palm_topology.bottleneck_ratio > 2.0
    assert forearm_track.topology_score > 0.8
    assert palm_track.topology_score < 0.25
    assert palm_track.forearm_score_topology < palm_track.forearm_score_v2 - 0.08
    assert forearm_track.forearm_score_topology >= forearm_track.forearm_score_v2 - 0.05


def test_topology_shadow_reports_far_shell_fanout():
    hand, _mask, depth, intrinsics = _forearm_fixture()
    fan_mask = np.zeros_like(depth, dtype=np.uint8)
    fan_mask[106:155, 150:171] = 255
    for row in range(155, 230):
        offset = int((row - 155) * 0.32)
        fan_mask[row, 150 - offset:161 - offset] = 255
        fan_mask[row, 160 + offset:171 + offset] = 255

    estimate = estimate_forearm_axis_wrist_geometry(
        hand, fan_mask, depth, intrinsics
    )
    topology = estimate.branches[0].features.topology

    assert topology.distal_valid
    assert topology.far_shell_component_count >= 2
    assert topology.distal_angular_spread_deg is not None


def _cylindrical_forearm_depth(depth, intrinsics, radius_m=0.040):
    curved = depth.copy()
    for column in range(140, 181):
        transverse_m = (
            (column - intrinsics.cx) * 0.75 / intrinsics.fx
        )
        curved[:, column] = (
            0.75 + radius_m
            - np.sqrt(max(radius_m * radius_m - transverse_m * transverse_m, 0.0))
        )
    return curved


def test_depth_surface_shadow_separates_cylinder_from_planar_palm_like_surface():
    hand, mask, flat_depth, intrinsics = _forearm_fixture()
    curved_depth = _cylindrical_forearm_depth(flat_depth, intrinsics)
    curved = estimate_forearm_axis_wrist_geometry(
        hand, mask, curved_depth, intrinsics
    )
    planar = estimate_forearm_axis_wrist_geometry(
        hand, mask, flat_depth, intrinsics
    )

    curved_surface = curved.branches[0].features.depth_surface
    planar_surface = planar.branches[0].features.depth_surface
    assert curved_surface.curvature_valid
    assert curved_surface.normal_valid
    assert curved_surface.boundary_valid
    assert curved_surface.curvature_anisotropy > 0.95
    assert curved_surface.radius_estimate_m == pytest.approx(0.035, abs=0.008)
    assert curved_surface.normal_jump_deg < 5.0
    assert curved_surface.object_boundary_ratio == pytest.approx(1.0)
    assert curved_surface.score_raw > 0.90
    assert planar_surface.score_raw < 0.60
    assert curved_surface.score_raw > planar_surface.score_raw + 0.30


def test_depth_surface_track_ema_is_maturity_gated_and_shadow_only():
    hand, mask, depth, intrinsics = _forearm_fixture()
    depth = _cylindrical_forearm_depth(depth, intrinsics)
    estimate = estimate_forearm_axis_wrist_geometry(
        hand, mask, depth, intrinsics
    )
    state = WristGeometryBranchTrackerState(minimum_confirmation_frames=2)

    first = update_wrist_geometry_branch_tracks(
        estimate, state, timestamp_sec=0.0
    ).tracks[0]
    assert first.depth_surface_score_raw > 0.90
    assert first.depth_surface_score_ema > 0.90
    assert first.depth_surface_maturity == pytest.approx(0.0)
    assert first.depth_surface_score == pytest.approx(0.5)

    for index in range(1, 12):
        tracked = update_wrist_geometry_branch_tracks(
            estimate, state, timestamp_sec=index * 0.1
        )
    mature = tracked.tracks[0]
    assert mature.depth_surface_maturity == pytest.approx(1.0)
    assert mature.depth_surface_score > 0.90
    assert mature.depth_surface_components["curvature_score"] > 0.90
    assert tracked.diagnostics()["active_forearm_track"] is None


def test_depth_surface_score_does_not_change_existing_shadow_scores():
    hand, mask, depth, intrinsics = _forearm_fixture()
    estimate = estimate_forearm_axis_wrist_geometry(
        hand, mask, depth, intrinsics
    )
    branch = estimate.branches[0]
    features = branch.features
    changed_surface = replace(
        features.depth_surface,
        score_raw=0.0,
        evidence=1.0,
    )
    changed = replace(
        estimate,
        branches=(replace(
            branch,
            features=replace(features, depth_surface=changed_surface),
        ),),
    )

    outputs = []
    for source in (estimate, changed):
        state = WristGeometryBranchTrackerState(minimum_confirmation_frames=2)
        for index in range(12):
            tracked = update_wrist_geometry_branch_tracks(
                source, state, timestamp_sec=index * 0.1
            )
        outputs.append(tracked.tracks[0])

    baseline, altered = outputs
    assert baseline.forearm_score_v2 == pytest.approx(altered.forearm_score_v2)
    assert baseline.topology_score == pytest.approx(altered.topology_score)
    assert baseline.forearm_score_topology == pytest.approx(
        altered.forearm_score_topology
    )
    assert baseline.depth_surface_score != pytest.approx(
        altered.depth_surface_score
    )


def test_track_lost_reason_is_diagnostic_only():
    hand, mask, depth, intrinsics = _forearm_fixture()
    estimate = estimate_forearm_axis_wrist_geometry(hand, mask, depth, intrinsics)
    state = WristGeometryBranchTrackerState(maximum_missed_frames=3)
    tracked = update_wrist_geometry_branch_tracks(estimate, state)
    track_id = tracked.tracks[0].track_id
    rejected = replace(
        estimate,
        direction=None,
        quality="V2_REJECTED",
        selected_branch=None,
        reason="CONNECTED_CLOUD_TOO_SMALL",
        branches=(),
    )

    missing = update_wrist_geometry_branch_tracks(rejected, state)
    assert missing.tracks[0].track_id == track_id
    assert missing.tracks[0].track_lost_reason == "ESTIMATOR_NO_BRANCH"
    assert missing.tracks[0].track_lost_detail["estimator_reason"] == (
        "CONNECTED_CLOUD_TOO_SMALL"
    )
    assert missing.diagnostics()["active_forearm_track"] is None

    update_wrist_geometry_branch_tracks(rejected, state)
    update_wrist_geometry_branch_tracks(rejected, state)
    expired = update_wrist_geometry_branch_tracks(rejected, state)
    assert not expired.tracks
    assert expired.track_lost_events[0]["track_id"] == track_id
    assert expired.track_lost_events[0]["reason"] == "TRACK_EXPIRED"


def test_wrist_geometry_opposing_branches_get_distinct_track_ids():
    intrinsics = CameraIntrinsics(500.0, 500.0, 160.0, 120.0, 320, 240)
    depth = np.full((240, 320), 0.75, dtype=np.float32)
    mask = np.zeros((240, 320), dtype=np.uint8)
    mask[10:231, 150:171] = 255
    estimate = estimate_forearm_axis_wrist_geometry(
        [_point(0.50, 0.50)], mask, depth, intrinsics
    )
    state = WristGeometryBranchTrackerState()

    tracked = update_wrist_geometry_branch_tracks(estimate, state)
    track_ids = [branch.track_id for branch in tracked.branches]

    assert len(track_ids) == 2
    assert all(track_id is not None for track_id in track_ids)
    assert len(set(track_ids)) == 2
    assert tracked.candidate_track_id is None


def test_wrist_geometry_shadow_is_invariant_to_palm_landmark_changes():
    hand, mask, depth, intrinsics = _forearm_fixture()
    baseline = estimate_forearm_axis_wrist_geometry(
        hand, mask, depth, intrinsics
    )
    changed = list(hand)
    changed[5] = _point(0.02, 0.98)
    changed[9] = _point(0.98, 0.02)
    changed[13] = _point(0.05, 0.05)
    changed[17] = _point(0.95, 0.95)
    estimate = estimate_forearm_axis_wrist_geometry(
        changed, mask, depth, intrinsics
    )
    assert estimate == baseline


def test_wrist_geometry_shadow_rejects_missing_wrist_connected_depth():
    hand, mask, depth, intrinsics = _forearm_fixture()
    result = estimate_forearm_axis_wrist_geometry(
        hand, mask, np.zeros_like(depth), intrinsics
    )
    assert result.direction is None
    assert result.quality == "V2_REJECTED"
    assert result.reason == "CONNECTED_CLOUD_TOO_SMALL"


def test_wrist_geometry_shadow_reports_ambiguous_opposing_branches():
    intrinsics = CameraIntrinsics(500.0, 500.0, 160.0, 120.0, 320, 240)
    depth = np.full((240, 320), 0.75, dtype=np.float32)
    mask = np.zeros((240, 320), dtype=np.uint8)
    mask[10:231, 150:171] = 255
    wrist_only = [_point(0.50, 0.50)]
    result = estimate_forearm_axis_wrist_geometry(
        wrist_only, mask, depth, intrinsics
    )
    assert result.direction is None
    assert result.branch_count == 2
    assert result.reason == "AMBIGUOUS_BRANCH"


def test_forearm_sections_keep_invalid_indices():
    longitudinal = np.concatenate((np.full(50, 10.0), np.full(50, 90.0)))
    pixels = np.column_stack((longitudinal, np.zeros_like(longitudinal)))
    points = np.column_stack((
        longitudinal * 0.001,
        np.zeros_like(longitudinal),
        np.full_like(longitudinal, 0.75),
    ))
    sections = compute_forearm_sections(
        points, pixels, (0.0, 0.0), (1.0, 0.0),
        section_count=9, minimum_points_per_section=10,
    )
    assert len(sections) == 9
    assert [section.section_index for section in sections] == list(range(9))
    assert [section.section_index for section in sections if section.valid] == [0, 8]
    assert all(section.center_m is None for section in sections[1:8])


def test_low_inlier_ratio_can_be_classified_degraded_for_shadow_logging():
    sections = [
        ForearmSection(
            section_index=index,
            center_m=(index * 0.03, 0.0, 0.75),
            longitudinal_px=float(index * 20),
            point_count=100,
            radius_m=0.02,
            depth_spread_m=0.001,
            valid=True,
        )
        for index in range(9)
    ]
    result = refine_forearm_axis(
        sections, np.asarray([0, 1, 2, 3], dtype=np.int64),
        reference_direction=None,
    )
    assert result.quality_level == "PRIMARY_DEGRADED"
    assert not result.valid
    assert result.direction is not None
    assert result.inlier_ratio == pytest.approx(4.0 / 9.0)
    assert result.max_consecutive_inlier_sections == 4
    assert result.max_consecutive_inlier_span_m == pytest.approx(0.09)
    assert [section.inlier for section in result.sections[:5]] == [
        True, True, True, True, False,
    ]


def test_forearm_axis_selector_uses_probation_hold_and_committed_switches():
    hand, mask, depth, intrinsics = _forearm_fixture()
    state = ForearmAxisSelectorState()
    first = select_forearm_axis(hand, mask, depth, intrinsics, state)
    assert first.candidate_source == "PRIMARY_3D_FOREARM"
    assert first.active_source == "FALLBACK_MEDIAPIPE"
    assert first.selector_state == "PRIMARY_PROBATION"
    assert first.source_changed
    assert first.switch_reason == "INITIAL_FALLBACK_LOCK"

    second = select_forearm_axis(hand, mask, depth, intrinsics, state)
    assert second.active_source == "FALLBACK_MEDIAPIPE"
    assert not second.source_changed
    third = select_forearm_axis(hand, mask, depth, intrinsics, state)
    assert third.active_source == "PRIMARY_3D_FOREARM"
    assert third.source_changed
    assert third.switch_reason == "PRIMARY_STRONG_CONFIRMED"
    assert third.unsigned_angle_deg is not None
    diagnostics = third.diagnostics()
    assert diagnostics["primary_axis"] is not None
    assert diagnostics["mediapipe_axis"] is not None
    assert diagnostics["unsigned_angle"] is not None
    assert diagnostics["axis_delta_prev"] is not None
    assert diagnostics["selector_state"] == "ACTIVE_PRIMARY"
    assert diagnostics["candidate_source"] == "PRIMARY_3D_FOREARM"
    assert diagnostics["active_source"] == "PRIMARY_3D_FOREARM"
    assert diagnostics["switch_reason"] == "PRIMARY_STRONG_CONFIRMED"

    hold = select_forearm_axis(
        hand, np.zeros_like(mask), depth, intrinsics, state
    )
    assert hold.active_source == "PRIMARY_3D_FOREARM"
    assert hold.selector_state == "PRIMARY_HOLD"
    assert hold.direction is None
    assert not hold.source_changed

    recovered = select_forearm_axis(hand, mask, depth, intrinsics, state)
    assert recovered.active_source == "PRIMARY_3D_FOREARM"
    assert recovered.direction is not None
    assert not recovered.source_changed

    for _ in range(2):
        fallback = select_forearm_axis(
            hand, np.zeros_like(mask), depth, intrinsics, state
        )
        assert fallback.active_source == "PRIMARY_3D_FOREARM"
        assert not fallback.source_changed
    fallback = select_forearm_axis(
        hand, np.zeros_like(mask), depth, intrinsics, state
    )
    assert fallback.active_source == "FALLBACK_MEDIAPIPE"
    assert fallback.direction is not None
    assert fallback.source_changed
    assert fallback.switch_reason == "PRIMARY_FAILURE_CONFIRMED"

    invalid = select_forearm_axis(
        hand, np.zeros_like(mask), np.zeros_like(depth), intrinsics, state
    )
    assert invalid.source == "INVALID"
    assert invalid.direction is None
    assert invalid.source_changed
    assert invalid.reason == "MASK_EMPTY;FALLBACK_UNAVAILABLE"
    assert state.source_switch_count == 4


def test_surface_normal_plane_fit_points_from_camera_into_surface():
    intrinsics = CameraIntrinsics(500.0, 500.0, 50.0, 50.0, 100, 100)
    depth = np.full((100, 100), 0.75, dtype=np.float32)
    normal = estimate_surface_normal(depth, (50, 50), intrinsics, radius_px=15)
    assert normal is not None
    assert normal.vector == pytest.approx((0.0, 0.0, 1.0), abs=1e-5)
    assert normal.rms_m < 1e-5
    assert normal.point_count >= 30


def test_surface_patch_ransac_rejects_background_and_returns_plane_point():
    intrinsics = CameraIntrinsics(500.0, 500.0, 50.0, 50.0, 100, 100)
    depth = np.full((100, 100), 0.75, dtype=np.float32)
    depth[38:43, 38:43] = 0.762
    depth[57:62, 57:62] = 0.738
    result, reason, details = estimate_surface_patch_diagnostic(
        depth, (50, 50), intrinsics, radius_px=18
    )
    assert reason == "NONE"
    assert result is not None
    assert result.point_m == pytest.approx((0.0, 0.0, 0.75), abs=2e-4)
    assert result.normal.vector == pytest.approx((0.0, 0.0, 1.0), abs=1e-4)
    camera_ray = np.asarray(result.point_m) / np.linalg.norm(result.point_m)
    assert np.dot(result.normal.vector, camera_ray) > 0.0
    goal = np.asarray(result.point_m) - 0.10 * np.asarray(result.normal.vector)
    assert np.linalg.norm(goal) < np.linalg.norm(result.point_m)
    assert details["plane_inlier_count"] >= 400
    assert details["plane_residual"] < 1e-4
    assert details["normal_confidence"] > 0.5
    assert details["depth_valid_ratio"] > 0.9


def test_surface_patch_quality_gate_blocks_sparse_depth():
    intrinsics = CameraIntrinsics(500.0, 500.0, 50.0, 50.0, 100, 100)
    depth = np.zeros((100, 100), dtype=np.float32)
    depth[49:52, 49:52] = 0.75
    result, reason, details = estimate_surface_patch_diagnostic(
        depth, (50, 50), intrinsics, radius_px=18
    )
    assert result is None
    assert reason == "PLANE_INSUFFICIENT_DEPTH"
    assert details["depth_valid_ratio"] < 0.01


def test_surface_point_comes_from_plane_not_target_pixel_depth():
    intrinsics = CameraIntrinsics(500.0, 500.0, 50.0, 50.0, 100, 100)
    depth = np.full((100, 100), 0.75, dtype=np.float32)
    # A valid but wrong center-pixel depth must not become the control target.
    depth[50, 50] = 0.738
    result, reason, _details = estimate_surface_patch_diagnostic(
        depth, (50, 50), intrinsics, radius_px=18,
        reference_depth_m=0.75,
    )
    assert reason == "NONE"
    assert result is not None
    assert result.point_m[2] == pytest.approx(0.75, abs=2e-4)
    assert result.point_m[2] != pytest.approx(float(depth[50, 50]), abs=1e-3)


def test_surface_normal_filter_rejects_direction_flip():
    stable = StableSurfaceNormalFilter(window_size=3, maximum_angle_deg=5.0)
    sample = SurfaceNormal((0.0, 0.0, 1.0), 0.001, 100)
    assert stable.update(sample) is None
    assert stable.update(sample) is None
    assert stable.update(SurfaceNormal((0.0, 0.0, -1.0), 0.001, 100)) is None
