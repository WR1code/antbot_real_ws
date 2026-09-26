"""Pressure checks used only to abort unexpected non-contact motion."""

from __future__ import annotations

import math
import statistics


def local_pressure_baseline(
    samples: list[tuple[float, float]],
    now: float,
    window_sec: float,
    minimum_samples: int,
    maximum_std_pa: float,
) -> float:
    """Return a stable recent median without assuming the saved flat zero."""
    recent = [
        value for received, value in samples
        if 0.0 <= now - received <= window_sec and math.isfinite(value)
    ]
    if len(recent) < minimum_samples:
        raise ValueError("insufficient recent pressure samples")
    if statistics.pstdev(recent) > maximum_std_pa:
        raise ValueError("local pressure is unstable")
    return statistics.median(recent)
