import numpy as np
import pytest

from meridian_hand_vision.axis_overlay import (
    draw_arm_axis_overlay,
    draw_candidate_forearm_axis_overlay,
    draw_forearm_diagnostic_overlay,
    draw_forearm_direction_overlay,
    project_camera_point_to_image,
    select_vision_forearm_axis,
    unsigned_axis_angle_deg,
)
from meridian_hand_vision.pulse_region import CameraIntrinsics


def test_overlay_axis_angles_are_unoriented():
    assert unsigned_axis_angle_deg((1.0, 0.0, 0.0), (-1.0, 0.0, 0.0)) == (
        pytest.approx(0.0)
    )
    assert unsigned_axis_angle_deg((1.0, 0.0, 0.0), (0.0, 1.0, 0.0)) == (
        pytest.approx(90.0)
    )
    assert unsigned_axis_angle_deg(None, (1.0, 0.0, 0.0)) is None


def test_overlay_camera_projection_handles_display_mirroring():
    intrinsics = CameraIntrinsics(500.0, 500.0, 160.0, 120.0, 320, 240)
    shape = (240, 320, 3)
    normal = project_camera_point_to_image(
        (0.10, 0.0, 1.0), intrinsics, shape
    )
    mirrored = project_camera_point_to_image(
        (0.10, 0.0, 1.0), intrinsics, shape, mirrored=True
    )
    assert normal == (210, 120)
    assert mirrored == (109, 120)


def test_overlay_draws_three_axes_with_distinct_colors():
    panel = np.zeros((240, 320, 3), dtype=np.uint8)
    intrinsics = CameraIntrinsics(500.0, 500.0, 160.0, 120.0, 320, 240)
    draw_arm_axis_overlay(
        panel,
        (0.0, 0.0, 0.75),
        intrinsics,
        (
            {
                "label": "MP PALM", "color": (0, 0, 255),
                "vector": (1.0, 0.0, 0.0), "length_m": 0.080,
                "origin_offset_px": (-5, -4), "status": "OK",
            },
            {
                "label": "V2.1", "color": (255, 80, 0),
                "vector": (0.0, 1.0, 0.0), "length_m": 0.120,
                "origin_offset_px": (0, 0), "status": "OK",
                "detail": "norm=1.000",
            },
            {
                "label": "V2.2", "color": (0, 255, 0),
                "vector": (-1.0, 0.0, 0.0), "length_m": 0.160,
                "origin_offset_px": (5, 4), "status": "OK",
                "detail": "track=7 conf=0.90",
            },
        ),
        {
            "mediapipe_v2_1": 90.0,
            "mediapipe_v2_2": 0.0,
            "v2_1_v2_2": 90.0,
        },
    )

    assert np.any((panel[:, :, 2] > 200) & (panel[:, :, 1] < 80))
    assert np.any((panel[:, :, 0] > 200) & (panel[:, :, 1] > 40))
    assert np.any((panel[:, :, 1] > 200) & (panel[:, :, 2] < 80))


def test_candidate_draws_while_unstable_robot_direction_does_not():
    panel = np.zeros((480, 640, 3), dtype=np.uint8)
    intrinsics = CameraIntrinsics(500.0, 500.0, 320.0, 240.0, 640, 480)
    candidate_drawn = draw_candidate_forearm_axis_overlay(
        panel, (0.0, 0.0, 0.75), intrinsics, (1.0, 0.0, 0.0),
        "PRIMARY_3D_FOREARM",
    )
    robot_drawn = draw_forearm_direction_overlay(
        panel, (0.0, 0.0, 0.75), intrinsics, None,
        "NO_STABLE_DIRECTION", "FILTER_NOT_STABLE",
        filter_state="STABILIZING",
    )
    assert candidate_drawn is True
    assert robot_drawn is False
    assert np.any((panel[:, :, 1] > 200) & (panel[:, :, 2] > 200))


def test_stable_robot_direction_draws_blue_arrow():
    panel = np.zeros((480, 640, 3), dtype=np.uint8)
    intrinsics = CameraIntrinsics(500.0, 500.0, 320.0, 240.0, 640, 480)
    assert draw_forearm_direction_overlay(
        panel, (0.0, 0.0, 0.75), intrinsics, (1.0, 0.0, 0.0),
        "STABLE_DIRECTION", "OK", filter_state="STABLE",
    ) is True
    assert np.any((panel[:, :, 0] > 200) & (panel[:, :, 1] < 80))


def test_missing_candidate_and_robot_directions_draw_no_arrows():
    panel = np.zeros((480, 640, 3), dtype=np.uint8)
    intrinsics = CameraIntrinsics(500.0, 500.0, 320.0, 240.0, 640, 480)
    candidate_details = {}
    robot_details = {}
    assert draw_candidate_forearm_axis_overlay(
        panel, (0.0, 0.0, 0.75), intrinsics, None, "INVALID",
        diagnostics=candidate_details,
    ) is False
    assert draw_forearm_direction_overlay(
        panel, (0.0, 0.0, 0.75), intrinsics, None,
        "NO_STABLE_DIRECTION", "FILTER_NOT_STABLE",
        filter_state="NO_INPUT", diagnostics=robot_details,
    ) is False
    assert candidate_details["draw_failure_reason"] == "NO_AXIS"
    assert robot_details["draw_failure_reason"] == "NO_AXIS"


def test_vision_axis_prefers_persistent_v2_2_then_v2_1():
    axis, source = select_vision_forearm_axis(
        (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)
    )
    assert tuple(axis) == pytest.approx((1.0, 0.0, 0.0))
    assert source == "V2.2_TRACK"

    axis, source = select_vision_forearm_axis(None, (0.0, 1.0, 0.0))
    assert tuple(axis) == pytest.approx((0.0, 1.0, 0.0))
    assert source == "V2.1_AXIS"

    axis, source = select_vision_forearm_axis(None, None)
    assert axis is None
    assert source == "NONE"


def test_multi_source_diagnostic_overlay_draws_all_four_semantics():
    panel = np.zeros((480, 640, 3), dtype=np.uint8)
    intrinsics = CameraIntrinsics(500.0, 500.0, 320.0, 240.0, 640, 480)
    diagnostics = draw_forearm_diagnostic_overlay(
        panel,
        (0.0, 0.0, 0.75),
        intrinsics,
        robot_axis=(1.0, 0.0, 0.0),
        vision_axis=(0.0, -1.0, 0.0),
        vision_source="V2.2_TRACK",
        selector_axis=(-1.0, 0.0, 0.0),
        selector_source="PRIMARY_3D_FOREARM",
        b_plus_variants={
            "hold_0_2s": {
                "current_source": "HOLD_LAST_PRIMARY_DIRECTION",
                "current_direction": (1.0, 1.0, 0.0),
            },
            "hold_0_5s": {
                "current_source": "PRIMARY_3D_FOREARM",
                "current_direction": (1.0, -1.0, 0.0),
            },
            "hold_1_0s": {
                "current_source": "NONE",
                "current_direction": None,
            },
        },
    )
    assert diagnostics["robot_drawn"] is True
    assert diagnostics["vision_drawn"] is True
    assert diagnostics["selector_drawn"] is True
    assert diagnostics["b_plus_drawn"] is True
    assert diagnostics["robot_status"] == "STABLE_DIRECTION"
    assert diagnostics["vision_source"] == "V2.2_TRACK"
    assert diagnostics["selector_source"] == "PRIMARY_3D_FOREARM"
    assert diagnostics["b_plus_status"] == {
        "hold_0_2s": "HOLD",
        "hold_0_5s": "PRIMARY",
        "hold_1_0s": "NONE",
    }


def test_multi_source_overlay_does_not_fallback_robot_or_vision_axes():
    panel = np.zeros((480, 640, 3), dtype=np.uint8)
    intrinsics = CameraIntrinsics(500.0, 500.0, 320.0, 240.0, 640, 480)
    diagnostics = draw_forearm_diagnostic_overlay(
        panel,
        (0.0, 0.0, 0.75),
        intrinsics,
        robot_axis=None,
        vision_axis=None,
        vision_source="NONE",
        selector_axis=(1.0, 0.0, 0.0),
        selector_source="FALLBACK_MEDIAPIPE",
        b_plus_variants={},
    )
    assert diagnostics["robot_drawn"] is False
    assert diagnostics["vision_drawn"] is False
    assert diagnostics["selector_drawn"] is True
    assert diagnostics["b_plus_drawn"] is False
    assert diagnostics["robot_status"] == "NO_STABLE_DIRECTION"
    assert diagnostics["vision_source"] == "NONE"
