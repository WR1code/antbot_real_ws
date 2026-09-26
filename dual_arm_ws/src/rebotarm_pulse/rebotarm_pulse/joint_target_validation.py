"""Pure validation helpers for guarded joint-space escape targets."""

from __future__ import annotations

import math
from collections.abc import Sequence


def validate_joint_target(
    target: Sequence[float],
    joint_names: Sequence[str],
    lower_limits: Sequence[float],
    upper_limits: Sequence[float],
    guard_indices: Sequence[int],
    guard_minimum_abs: Sequence[float],
) -> list[float]:
    """Validate dimensions, finiteness, limits, and simple singular guards."""
    count = len(joint_names)
    if len(target) != count:
        raise ValueError(
            f"target_joint_values must contain {count} values, got {len(target)}"
        )
    if len(lower_limits) != count or len(upper_limits) != count:
        raise ValueError("joint limit arrays must match joint_names")
    if len(guard_indices) != len(guard_minimum_abs):
        raise ValueError("singularity guard indices and thresholds must have equal length")

    values = [float(value) for value in target]
    for index, (name, value, lower, upper) in enumerate(
        zip(joint_names, values, lower_limits, upper_limits)
    ):
        if not math.isfinite(value):
            raise ValueError(f"{name} target is not finite")
        if value < float(lower) or value > float(upper):
            raise ValueError(
                f"{name} target {value:.4f} is outside "
                f"[{float(lower):.4f}, {float(upper):.4f}]"
            )

    for index, minimum in zip(guard_indices, guard_minimum_abs):
        joint_index = int(index)
        threshold = float(minimum)
        if joint_index < 0 or joint_index >= count:
            raise ValueError(f"invalid singularity guard joint index {joint_index}")
        if threshold < 0.0:
            raise ValueError("singularity guard threshold must be non-negative")
        if abs(values[joint_index]) < threshold:
            raise ValueError(
                f"{joint_names[joint_index]} must be at least {threshold:.3f} rad "
                "away from zero for this escape target"
            )
    return values
