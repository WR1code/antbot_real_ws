import pytest

from rebotarm_pulse.pressure_safety import local_pressure_baseline


def test_current_pose_baseline_accepts_stable_offset_from_flat_zero():
    values = [(9.0 + index * 0.01, -130.0 + index % 3) for index in range(60)]
    assert local_pressure_baseline(values, 9.6, 1.0, 20, 5.0) == -129.0


def test_local_baseline_rejects_unstable_or_stale_data():
    unstable = [(9.0 + index * 0.01, -100.0 if index % 2 else 100.0)
                for index in range(60)]
    with pytest.raises(ValueError, match="unstable"):
        local_pressure_baseline(unstable, 9.6, 1.0, 20, 5.0)
    with pytest.raises(ValueError, match="insufficient"):
        local_pressure_baseline(unstable, 11.0, 1.0, 20, 5.0)
