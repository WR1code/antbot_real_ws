"""Quality gates and geometric information diagnostics, without robot motion."""

from dataclasses import replace

import numpy as np
import pytest

from rebotarm_pulse.adaptive_handeye import (
    PosePair, Transform, compute_sampling_score, rotation_exp,
)
from rebotarm_pulse.handeye_observation import Observation
from rebotarm_pulse.handeye_quality import (
    QualityConfig, evaluate_observation_quality, summarize_quality,
)


def normal_observation():
    return Observation(
        "session:1", 1000000000, "camera",
        {"rotation": np.eye(3).tolist(), "translation": [0, 0, 0.4]},
        0.3, 40000.0, [[100, 100], [300, 100], [300, 300], [100, 300]],
        image_width=640, image_height=480,
    )


def test_far_small_board_is_low_area():
    small = replace(normal_observation(), board_area=100.0,
                    corners=[[100, 100], [110, 100], [110, 110], [100, 110]])
    result = evaluate_observation_quality(small)
    assert not result.valid
    assert "LOW_AREA" in result.reasons


def test_severe_pnp_error_is_rejected():
    result = evaluate_observation_quality(replace(normal_observation(), reprojection_error=8.0))
    assert result.reasons == ["HIGH_REPROJECTION_ERROR"]


def test_corners_concentrated_on_one_side_have_poor_distribution():
    item = replace(normal_observation(), board_area=400.0,
                   corners=[[100, 100], [102, 100], [102, 300], [100, 300]])
    result = evaluate_observation_quality(item)
    assert not result.valid
    assert "POOR_CORNER_DISTRIBUTION" in result.reasons


def test_normal_observation_and_zero_rms_have_finite_valid_score():
    item = normal_observation()
    result = evaluate_observation_quality(item)
    assert result.valid and not result.reasons
    assert result.normalized_board_area == pytest.approx(40000 / (640 * 480))
    # Four rectangle vertices have population variances width^2/4,height^2/4.
    assert result.corner_distribution == pytest.approx(3 * 40000 / (640 * 480))
    assert np.isfinite(evaluate_observation_quality(replace(item, reprojection_error=0.0)).score)


def test_corner_distribution_is_resolution_and_offset_invariant():
    item = normal_observation()
    scaled = replace(item, image_width=1280, image_height=960, board_area=160000.0,
                     corners=(np.asarray(item.corners) * 2).tolist())
    shifted = replace(item, corners=(np.asarray(item.corners) + [50, 0]).tolist())
    baseline = evaluate_observation_quality(item)
    assert evaluate_observation_quality(scaled).score == pytest.approx(baseline.score)
    assert evaluate_observation_quality(shifted).corner_distribution == pytest.approx(baseline.corner_distribution)


def test_missing_dimensions_and_collinear_corners_fail_closed():
    assert not evaluate_observation_quality(replace(normal_observation(), image_width=0)).valid
    result = evaluate_observation_quality(replace(normal_observation(), corners=[[1, 1], [2, 2], [3, 3]]))
    assert result.corner_distribution == 0.0
    assert "POOR_CORNER_DISTRIBUTION" in result.reasons


def test_quality_summary_counts_all_reasons_without_deleting_samples():
    good = evaluate_observation_quality(normal_observation())
    bad = evaluate_observation_quality(replace(normal_observation(), board_area=1, reprojection_error=5))
    summary = summarize_quality([good, bad])
    assert summary["total"] == 2 and summary["valid"] == 1 and summary["invalid"] == 1
    assert summary["reject_reason_count"] == {"LOW_AREA": 1, "HIGH_REPROJECTION_ERROR": 1}
    assert summary["mean_score"] == pytest.approx((good.score + bad.score) / 2)
    assert summarize_quality([])["total"] == 0


def diverse_samples():
    poses = [Transform.identity()]
    for axis in np.eye(3):
        poses.append(Transform(rotation_exp(axis * 0.3), axis * 0.04))
    return [PosePair(pose, Transform.identity()) for pose in poses]


def test_repeated_candidate_adds_no_information_but_new_pose_does():
    samples = diverse_samples()
    repeated = compute_sampling_score(samples[1].base_T_link5, samples)
    novel = compute_sampling_score(Transform(rotation_exp([0.2, 0.3, -0.2]), [0.05, -0.02, 0.03]), samples)
    assert not repeated["degenerate"]
    assert repeated["information_gain"] == 0.0
    assert novel["information_gain"] > repeated["information_gain"]
    assert 0 < novel["score"] < 1


def test_insufficient_or_single_axis_samples_are_degenerate():
    assert compute_sampling_score(Transform.identity(), diverse_samples()[:3])["degenerate"]
    samples = [Transform(rotation_exp([0, 0, angle]), [0, 0, 0]) for angle in (0, 0.1, 0.2, 0.3)]
    assert compute_sampling_score(Transform.identity(), samples)["degenerate"]


@pytest.mark.parametrize("config", [dict(reprojection_epsilon=0), dict(area_weight=-1), dict(min_board_area=2)])
def test_invalid_quality_configuration_is_rejected(config):
    with pytest.raises(ValueError):
        QualityConfig(**config)
