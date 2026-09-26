"""Pure geometry for a future Piper-H J6 sensor-array alignment.

This module computes a dry-run angle only. It does not create ROS publishers,
clients, actions, or robot commands.
"""

from __future__ import annotations

from dataclasses import dataclass
import math


Vector3 = tuple[float, float, float]


@dataclass(frozen=True)
class SensorAxisAlignment:
    valid: bool
    delta_rad: float
    delta_deg: float
    current_sensor_axis_base: Vector3 | None
    target_arm_axis_base: Vector3 | None
    j6_axis_base: Vector3 | None
    projected_sensor_axis: Vector3 | None
    projected_arm_axis: Vector3 | None
    reason: str


@dataclass(frozen=True)
class TargetOrientationAxes:
    """Reserved full-orientation basis once a surface normal is measured."""

    sensor_axis: Vector3
    contact_normal: Vector3
    lateral_axis: Vector3


def _finite_vector(values, name: str) -> Vector3:
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must contain three finite values") from error
    if len(result) != 3 or not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} must contain three finite values")
    return result


def _dot(first: Vector3, second: Vector3) -> float:
    return sum(a * b for a, b in zip(first, second))


def _cross(first: Vector3, second: Vector3) -> Vector3:
    ax, ay, az = first
    bx, by, bz = second
    return ay * bz - az * by, az * bx - ax * bz, ax * by - ay * bx


def _scale(vector: Vector3, factor: float) -> Vector3:
    return tuple(value * factor for value in vector)


def _subtract(first: Vector3, second: Vector3) -> Vector3:
    return tuple(a - b for a, b in zip(first, second))


def _norm(vector: Vector3) -> float:
    return math.sqrt(_dot(vector, vector))


def _unit(values, name: str, epsilon: float) -> Vector3:
    vector = _finite_vector(values, name)
    length = _norm(vector)
    if length < epsilon:
        raise ValueError(f"{name} is zero or too small")
    return _scale(vector, 1.0 / length)


def direction_is_fresh(
    received_monotonic: float,
    now_monotonic: float,
    maximum_age_sec: float,
) -> tuple[bool, str]:
    """Validate receive-time freshness separately from ROS timestamp/TF checks."""
    values = (received_monotonic, now_monotonic, maximum_age_sec)
    if not all(math.isfinite(value) for value in values) or maximum_age_sec <= 0.0:
        return False, "arm_direction freshness values are invalid"
    age = now_monotonic - received_monotonic
    if age < 0.0 or age > maximum_age_sec:
        return False, f"arm_direction is stale (age={age:.3f}s)"
    return True, "arm_direction is fresh"


def compute_sensor_axis_alignment(
    sensor_axis_base,
    arm_direction_base,
    j6_axis_base,
    *,
    sensor_array_axis_configured: bool,
    current_j6_rad: float | None = None,
    j6_limits_rad: tuple[float, float] | None = None,
    projection_epsilon: float = 1e-4,
) -> SensorAxisAlignment:
    """Return the smallest signed J6 rotation aligning two unoriented axes."""
    def invalid(reason: str) -> SensorAxisAlignment:
        return SensorAxisAlignment(
            False, math.nan, math.nan, None, None, None, None, None, reason
        )
    try:
        sensor = _unit(sensor_axis_base, "sensor axis", projection_epsilon)
        arm = _unit(arm_direction_base, "arm direction", projection_epsilon)
        axis = _unit(j6_axis_base, "J6 axis", projection_epsilon)
    except ValueError as error:
        return invalid(str(error))
    if not sensor_array_axis_configured:
        return SensorAxisAlignment(
            False, math.nan, math.nan, sensor, arm, axis, None, None,
            "sensor_array_axis_configured=false; confirm the physical Chi-to-Cun axis",
        )

    sensor_projected = _subtract(sensor, _scale(axis, _dot(sensor, axis)))
    arm_projected = _subtract(arm, _scale(axis, _dot(arm, axis)))
    sensor_length = _norm(sensor_projected)
    arm_length = _norm(arm_projected)
    if sensor_length < projection_epsilon:
        return SensorAxisAlignment(
            False, math.nan, math.nan, sensor, arm, axis, None, None,
            "sensor array axis is nearly parallel to J6; J6 rotation cannot align it",
        )
    if arm_length < projection_epsilon:
        return SensorAxisAlignment(
            False, math.nan, math.nan, sensor, arm, axis, None, None,
            "arm direction is nearly parallel to J6; J6 rotation cannot align it",
        )
    sensor_projected = _scale(sensor_projected, 1.0 / sensor_length)
    arm_projected = _scale(arm_projected, 1.0 / arm_length)

    def signed_angle(target: Vector3) -> float:
        sine = _dot(axis, _cross(sensor_projected, target))
        cosine = max(-1.0, min(1.0, _dot(sensor_projected, target)))
        return math.atan2(sine, cosine)

    direct = signed_angle(arm_projected)
    reverse_target = _scale(arm_projected, -1.0)
    reverse = signed_angle(reverse_target)
    if abs(reverse) < abs(direct):
        delta = reverse
        selected_arm = _scale(arm, -1.0)
        selected_projected_arm = reverse_target
    else:
        delta = direct
        selected_arm = arm
        selected_projected_arm = arm_projected
    if not math.isfinite(delta):
        return invalid("computed J6 alignment angle is NaN or Inf")

    if current_j6_rad is not None or j6_limits_rad is not None:
        if current_j6_rad is None or j6_limits_rad is None:
            return invalid("current J6 angle and J6 limits must be provided together")
        try:
            lower, upper = (float(value) for value in j6_limits_rad)
            current = float(current_j6_rad)
        except (TypeError, ValueError) as error:
            return invalid(f"J6 angle or limits are invalid: {error}")
        target = current + delta
        if not all(math.isfinite(value) for value in (lower, upper, current, target)):
            return invalid("J6 angle or limits contain NaN or Inf")
        if lower >= upper or target < lower or target > upper:
            return SensorAxisAlignment(
                False, delta, math.degrees(delta), sensor, selected_arm, axis,
                sensor_projected, selected_projected_arm,
                f"suggested J6 target {target:.4f} rad is outside "
                f"[{lower:.4f}, {upper:.4f}] rad",
            )

    return SensorAxisAlignment(
        True, delta, math.degrees(delta), sensor, selected_arm, axis,
        sensor_projected, selected_projected_arm, "alignment dry-run is valid",
    )


def construct_target_orientation_axes(
    arm_direction_base,
    surface_normal_base,
    epsilon: float = 1e-4,
) -> TargetOrientationAxes:
    """Build future target axes; requires a real measured surface normal.

    A future motion implementation must additionally map these axes to the
    physically calibrated local sensor-array and contact-normal axes.
    """
    arm = _unit(arm_direction_base, "arm direction", epsilon)
    normal = _unit(surface_normal_base, "surface normal", epsilon)
    sensor = _subtract(arm, _scale(normal, _dot(arm, normal)))
    sensor = _unit(sensor, "arm direction projected on surface", epsilon)
    lateral = _unit(_cross(normal, sensor), "target lateral axis", epsilon)
    return TargetOrientationAxes(sensor, normal, lateral)
