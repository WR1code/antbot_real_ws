from types import SimpleNamespace

import pytest

from rebotarm_pulse.auto_handeye_sequence import calibration_residuals


def transform(x=0.0, y=0.0, z=0.0, q=(0.0, 0.0, 0.0, 1.0)):
    return SimpleNamespace(
        translation=SimpleNamespace(x=x, y=y, z=z),
        rotation=SimpleNamespace(x=q[0], y=q[1], z=q[2], w=q[3]),
    )


def sample(robot, tracking):
    return SimpleNamespace(robot=robot, tracking=tracking)


def test_calibration_residuals_are_zero_for_consistent_samples():
    samples = [
        sample(transform(x=offset), transform(x=-offset, z=0.5))
        for offset in (-0.2, 0.0, 0.2)
    ]

    result = calibration_residuals(samples, transform())

    assert result["translation_rms_m"] == pytest.approx(0.0)
    assert result["translation_max_m"] == pytest.approx(0.0)
    assert result["rotation_rms_deg"] == pytest.approx(0.0)


def test_calibration_residuals_reveal_bad_tracking_sample():
    samples = [
        sample(transform(), transform(z=0.5)),
        sample(transform(), transform(z=0.5)),
        sample(transform(), transform(x=0.12, z=0.5)),
    ]

    result = calibration_residuals(samples, transform())

    assert result["translation_rms_m"] > 0.05
    assert result["translation_max_m"] > 0.07


def test_calibration_residuals_reject_zero_quaternion():
    samples = [sample(transform(), transform()) for _ in range(3)]

    with pytest.raises(ValueError, match="zero-length quaternion"):
        calibration_residuals(samples, transform(q=(0.0, 0.0, 0.0, 0.0)))
