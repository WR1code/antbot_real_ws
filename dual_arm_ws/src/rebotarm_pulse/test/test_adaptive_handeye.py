import math
from types import SimpleNamespace

import numpy as np
import pytest

from rebotarm_pulse.adaptive_handeye import (
    PosePair,
    Transform,
    generate_hemisphere,
    look_at_roll_deg,
    look_at_rotation,
    plan_adaptive_increment,
    pose_dispersion,
    rotation_exp,
    solve_handeye,
    split_bootstrap_train_holdout,
    target_to_link5_pose,
)
from rebotarm_pulse.piperh_auto_handeye import (
    LOWER,
    UPPER,
    PiperHAutoHandeye,
    calibration_joint_margins,
    values_within_calibration_margins,
)


def test_locked_j6_uses_fixed_margin_not_fraction_of_full_range():
    margins = calibration_joint_margins(0.08, 0.10)
    np.testing.assert_allclose(margins[:5], 0.08 * (UPPER - LOWER)[:5])
    assert margins[5] == 0.10

    validated_anchor = (-0.258206088, 1.565755996, -1.272784016,
                        -0.240901640, 0.694742188, -1.935150140)
    assert values_within_calibration_margins(validated_anchor, margins)
    assert not values_within_calibration_margins(
        (*validated_anchor[:5], LOWER[5] + 0.099), margins
    )


@pytest.mark.parametrize("j6", [LOWER[5] + 0.10, UPPER[5] - 0.10])
def test_locked_j6_calibration_margin_accepts_exact_boundaries(j6):
    margins = calibration_joint_margins(0.08, 0.10)
    centre = (LOWER + UPPER) / 2.0
    centre[5] = j6
    assert values_within_calibration_margins(centre, margins)


@pytest.mark.parametrize(
    "j6",
    [LOWER[5] + 0.10 - 1e-6, UPPER[5] - 0.10 + 1e-6, math.nan, math.inf],
)
def test_locked_j6_calibration_margin_fails_closed(j6):
    margins = calibration_joint_margins(0.08, 0.10)
    centre = (LOWER + UPPER) / 2.0
    centre[5] = j6
    assert not values_within_calibration_margins(centre, margins)


def test_bootstrap_goals_remain_available_at_validated_j6_mount():
    parameters = {
        "joint_margin_fraction": 0.08,
        "locked_j6_margin_rad": 0.10,
    }
    node = SimpleNamespace(
        get_parameter=lambda name: SimpleNamespace(value=parameters[name])
    )
    anchor = (-0.258206088, 1.565755996, -1.272784016,
              -0.240901640, 0.694742188, -1.935150140)

    goals = list(PiperHAutoHandeye._bootstrap_goals(node, anchor))

    assert len(goals) == 18
    assert all(goal[5] == anchor[5] for goal in goals)


@pytest.mark.parametrize(
    ("fraction", "j6_margin"),
    [(0.0, 0.1), (0.5, 0.1), (math.nan, 0.1), (0.08, 0.0),
     (0.08, math.nan), (0.08, 2.1)],
)
def test_invalid_calibration_margins_fail_closed(fraction, j6_margin):
    with pytest.raises(ValueError):
        calibration_joint_margins(fraction, j6_margin)


def test_adaptive_increment_accepts_a_final_partial_batch():
    assert plan_adaptive_increment(37, 80, 8) == (8, 4)
    assert plan_adaptive_increment(77, 80, 8) == (3, 3)
    assert plan_adaptive_increment(80, 80, 8) == (0, 0)


def _synthetic_pairs():
    link5_T_camera = Transform(
        rotation_exp((0.35, -0.22, 0.18)),
        (0.055, -0.105, -0.048),
    )
    base_T_target = Transform(rotation_exp((0.08, -0.04, 0.12)), (0.42, 0.03, 0.18))
    pairs = []
    motions = (
        ((0.0, 0.0, 0.0), (0.24, 0.00, 0.35)),
        ((0.18, 0.02, -0.04), (0.26, -0.04, 0.34)),
        ((-0.15, 0.12, 0.03), (0.22, 0.05, 0.38)),
        ((0.05, -0.20, 0.14), (0.29, 0.02, 0.32)),
        ((-0.11, -0.08, -0.18), (0.21, -0.05, 0.36)),
        ((0.22, 0.15, 0.09), (0.27, 0.06, 0.39)),
        ((-0.20, 0.04, 0.16), (0.23, -0.07, 0.33)),
        ((0.09, 0.23, -0.12), (0.30, 0.04, 0.37)),
        ((-0.06, -0.19, 0.21), (0.20, 0.07, 0.35)),
        ((0.17, -0.14, -0.10), (0.28, -0.02, 0.40)),
    )
    for vector, translation in motions:
        base_T_link5 = Transform(rotation_exp(vector), translation)
        camera_T_target = link5_T_camera.inverse() @ base_T_link5.inverse() @ base_T_target
        pairs.append(PosePair(base_T_link5, camera_T_target))
    return link5_T_camera, pairs


def test_all_valid_solvers_recover_synthetic_link5_camera_transform():
    expected, pairs = _synthetic_pairs()
    candidates = solve_handeye(pairs[:8], pairs[8:])

    assert len(candidates) >= 3
    best = candidates[0].link5_T_camera
    assert np.linalg.norm(best.translation - expected.translation) < 1.0e-7
    relative = best.rotation.T @ expected.rotation
    angle = math.acos(np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0))
    assert angle < 1.0e-7
    assert candidates[0].holdout_translation_p95_m < 1.0e-7


def test_board_centred_caps_expand_and_camera_z_points_at_centre():
    board = Transform(np.eye(3), (0.4, 0.0, 0.1))
    centre_offset = (0.0875, 0.1225, 0.0)
    centre = board.point(centre_offset)
    poses25 = generate_hemisphere(
        board, centre_offset, (0.4, 0.0, 0.5), (0.36, 0.4, 0.44), 25.0, (-15, 0, 15)
    )
    poses45 = generate_hemisphere(
        board, centre_offset, (0.4, 0.0, 0.5), (0.36, 0.4, 0.44), 45.0, (-30, 0, 30)
    )

    assert len(poses25) == 3 * (4 + 8) * 3
    assert len(poses45) == 3 * (4 + 8 + 12 + 16 + 18) * 3
    assert len(poses45) > len(poses25)
    for item in poses45[::29]:
        expected_z = centre - item.base_T_camera.translation
        expected_z /= np.linalg.norm(expected_z)
        np.testing.assert_allclose(item.base_T_camera.rotation[:, 2], expected_z, atol=1.0e-10)


def test_look_at_roll_uses_same_absolute_convention_as_generated_pose():
    centre = np.asarray((0.4, -0.1, 0.2))
    position = np.asarray((0.3, -0.2, 0.6))
    board_x = np.asarray((1.0, 0.0, 0.0))
    for expected in (-95.0, -15.0, 0.0, 90.0, 137.0):
        pose = Transform(look_at_rotation(position, centre, board_x, expected), position)
        assert np.isclose(look_at_roll_deg(pose, centre, board_x), expected)


def test_camera_target_round_trip_returns_link5_target():
    link5_T_camera, _ = _synthetic_pairs()
    base_T_camera = Transform(rotation_exp((0.1, 0.2, -0.1)), (0.3, -0.1, 0.4))
    base_T_link5 = target_to_link5_pose(base_T_camera, link5_T_camera)
    recovered = base_T_link5 @ link5_T_camera
    np.testing.assert_allclose(recovered.translation, base_T_camera.translation)
    np.testing.assert_allclose(recovered.rotation, base_T_camera.rotation)


def test_pose_dispersion_reports_stationary_board_consistency():
    transforms = [Transform(np.eye(3), (0.4, 0.1, 0.2)) for _ in range(5)]
    metrics = pose_dispersion(transforms)
    assert metrics["translation_p95_m"] == 0.0
    assert metrics["rotation_p95_deg"] == 0.0


def test_pose_dispersion_scores_three_pose_holdout_with_one_outlier():
    transforms = [
        Transform(np.eye(3), (0.0, 0.0, 0.0)),
        Transform(np.eye(3), (0.001, 0.0, 0.0)),
        Transform(np.eye(3), (0.100, 0.0, 0.0)),
    ]

    metrics = pose_dispersion(transforms)

    # The outlier remains visible in the score, but cannot make metric
    # calculation fail merely because a three-pose set cannot discard a pose.
    assert metrics["translation_p95_m"] > 0.06
    assert metrics["translation_max_m"] > 0.06


def test_holdout_scoring_does_not_discard_valid_handeye_solutions():
    _, pairs = _synthetic_pairs()
    holdout = list(pairs[7:])
    outlier = holdout[-1]
    holdout[-1] = PosePair(
        outlier.base_T_link5,
        Transform(
            outlier.camera_T_target.rotation,
            outlier.camera_T_target.translation + np.asarray((0.100, 0.0, 0.0)),
        ),
    )

    candidates = solve_handeye(pairs[:7], holdout)

    assert len(candidates) >= 3
    assert candidates[0].holdout_translation_p95_m > 0.06


def test_bootstrap_holdout_is_stratified_without_removing_required_large_rotation():
    vectors_deg = (
        (0, 0, 0),
        (2, 0, 0), (0, 3, 0), (0, 0, 4),
        (5, 2, 0), (-5, 0, 2), (0, -6, 2),
        (7, 0, 3), (-7, 3, 0), (2, 8, 0),
        (0, -9, 2), (6, 0, -8), (-8, 6, 0),
        (10, 0, 0), (0, 11, 0), (0, 0, 18),
    )
    pairs = [
        PosePair(
            Transform(rotation_exp(np.radians(vector)), (index * 0.003, 0, 0)),
            Transform.identity(),
        )
        for index, vector in enumerate(vectors_deg)
    ]

    train, holdout, audit = split_bootstrap_train_holdout(
        pairs, 3, min_sv_ratio=0.06,
        min_maximum_rotation_deg=12.0, min_translation_span_m=0.025,
    )

    assert len(train) == 13
    assert len(holdout) == 3
    assert 0 in audit["train_indices"]
    assert 15 in audit["train_indices"]
    assert audit["holdout_indices"] != [13, 14, 15]
    assert audit["train_observability"]["maximum_relative_deg"] >= 12.0
    assert audit["train_observability"]["sv_ratio"] >= 0.06
