import math

import numpy as np

from antbot_dual_lidar.phase2a_bag_analysis import (
    _fit_circle,
    _fit_plane,
    select_time_hypothesis,
)
from antbot_dual_lidar.phase2a_motion_experiment import integrate_window


def test_constant_angular_velocity_integrates_in_seconds():
    samples = [(index * 10_000_000, 0.3, 0.0) for index in range(201)]
    assert math.isclose(integrate_window(samples, 0, 1.0), 0.3, abs_tol=1e-12)
    assert math.isclose(integrate_window(samples, 0, 2.0), 0.6, abs_tol=1e-12)


def test_plane_and_circle_metrics_recover_noise_free_geometry():
    grid = np.linspace(-1.0, 1.0, 20)
    plane = np.asarray([[2.0, x, y] for x in grid for y in grid])
    assert _fit_plane(plane)["rmse"] < 1e-12

    angles = np.linspace(0.0, 2.0 * math.pi, 200, endpoint=False)
    circle = np.column_stack((
        4.0 + 0.2 * np.cos(angles),
        0.2 * np.sin(angles),
        np.full_like(angles, 0.4),
    ))
    result = _fit_circle(circle)
    assert math.isclose(result["radius"], 0.2, abs_tol=1e-12)
    assert result["rmse"] < 1e-12


def test_time_model_requires_predeclared_five_percent_lead():
    candidates = {
        "A": {"median_deskew_rmse": 0.096, "p95_deskew_rmse": 0.12},
        "B": {"median_deskew_rmse": 0.100, "p95_deskew_rmse": 0.13},
    }
    assert select_time_hypothesis(candidates)["selected"] is None
    candidates["A"]["median_deskew_rmse"] = 0.094
    assert select_time_hypothesis(candidates)["selected"] == "A"
