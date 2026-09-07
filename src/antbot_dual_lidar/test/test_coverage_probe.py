import numpy as np
import pytest

from antbot_dual_lidar import coverage_probe


def test_angular_accumulator_bins_ranges_and_height(monkeypatch):
    points = np.asarray(
        [[1.0, 0.0, -0.1], [0.0, 2.0, 0.5], [-1.0, 0.0, 1.0]],
        dtype=np.float32,
    )
    monkeypatch.setattr(
        coverage_probe.point_cloud2,
        "read_points_numpy",
        lambda *_args, **_kwargs: points,
    )
    accumulator = coverage_probe.AngularAccumulator(90.0)
    accumulator.add(object())
    rows = list(accumulator.rows("front_left"))
    assert accumulator.frames == 1
    assert sum(row["return_count"] for row in rows) == 3
    assert min(
        row["min_range_m"] for row in rows if row["min_range_m"] != ""
    ) == pytest.approx(np.sqrt(1.01))
    assert max(row["max_z_m"] for row in rows if row["max_z_m"] != "") == 1.0
