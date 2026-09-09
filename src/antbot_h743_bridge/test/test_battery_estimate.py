"""Tests for the explicitly voltage-based battery estimate."""

import math

from antbot_h743_bridge.bridge import estimate_battery_percentage


def test_battery_estimate_clamps_to_configured_range():
    assert estimate_battery_percentage(18.0, 18.0, 30.0) == 0.0
    assert estimate_battery_percentage(24.0, 18.0, 30.0) == 0.5
    assert estimate_battery_percentage(30.0, 18.0, 30.0) == 1.0
    assert estimate_battery_percentage(31.0, 18.0, 30.0) == 1.0


def test_battery_estimate_rejects_invalid_calibration():
    assert math.isnan(estimate_battery_percentage(24.0, 30.0, 18.0))
    assert math.isnan(estimate_battery_percentage(math.nan, 18.0, 30.0))
