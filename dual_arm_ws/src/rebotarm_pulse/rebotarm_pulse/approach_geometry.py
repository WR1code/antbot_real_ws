"""Geometry helpers for safe pulse-region pre-contact positioning."""

from __future__ import annotations

import math


Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]


def subtract(first: Vector3, second: Vector3) -> Vector3:
    return tuple(a - b for a, b in zip(first, second))


def add(first: Vector3, second: Vector3) -> Vector3:
    return tuple(a + b for a, b in zip(first, second))


def scale(vector: Vector3, factor: float) -> Vector3:
    return tuple(value * factor for value in vector)


def norm(vector: Vector3) -> float:
    return math.sqrt(sum(value * value for value in vector))


def horizontal_vertical_displacement(
    destination: Vector3, origin: Vector3
) -> tuple[float, float]:
    """Return XY-plane distance and absolute Z travel in the shared world frame."""
    dx, dy, dz = subtract(destination, origin)
    return math.hypot(dx, dy), abs(dz)


def translation_within_axis_limits(
    destination: Vector3,
    origin: Vector3,
    maximum_horizontal_translation_m: float,
    maximum_vertical_translation_m: float,
) -> bool:
    """Check independent horizontal and vertical automatic-motion limits."""
    if maximum_horizontal_translation_m <= 0.0:
        raise ValueError("maximum_horizontal_translation_m must be positive")
    if maximum_vertical_translation_m <= 0.0:
        raise ValueError("maximum_vertical_translation_m must be positive")
    horizontal, vertical = horizontal_vertical_displacement(destination, origin)
    return (
        horizontal <= maximum_horizontal_translation_m
        and vertical <= maximum_vertical_translation_m
    )


def normalize(vector: Vector3) -> Vector3:
    length = norm(vector)
    if not math.isfinite(length) or length < 1e-6:
        raise ValueError("camera and pulse target are too close to define an approach direction")
    return scale(vector, 1.0 / length)


def quaternion_align_x(direction: Vector3) -> Quaternion:
    """Return a quaternion that rotates local +X onto ``direction``."""
    dx, dy, dz = normalize(direction)
    dot = dx
    if dot < -0.999999:
        return (0.0, 0.0, 1.0, 0.0)
    # Quaternion from the cross product X x direction and 1 + dot.
    qx, qy, qz, qw = 0.0, -dz, dy, 1.0 + dot
    length = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    return (qx / length, qy / length, qz / length, qw / length)


def rotate_vector(quaternion: Quaternion, vector: Vector3) -> Vector3:
    """Rotate ``vector`` by an XYZW quaternion."""
    qx, qy, qz, qw = quaternion
    qnorm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if not math.isfinite(qnorm) or qnorm < 1e-9:
        raise ValueError("tool orientation quaternion is invalid")
    qx, qy, qz, qw = qx / qnorm, qy / qnorm, qz / qnorm, qw / qnorm
    vx, vy, vz = vector
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


def piper_precontact_link_pose(
    current_link_origin: Vector3,
    current_orientation: Quaternion,
    target: Vector3,
    standoff_m: float,
    tip_offset_m: float,
    maximum_horizontal_translation_m: float,
    maximum_vertical_translation_m: float,
    allow_zero_standoff: bool = False,
) -> tuple[Vector3, Quaternion, Vector3, float]:
    """Preserve J6 orientation and translate the axial Piper probe to standoff."""
    if standoff_m < 0.0 or (standoff_m < 0.02 and not (allow_zero_standoff and standoff_m == 0.0)):
        raise ValueError("standoff_m must be at least 0.02 m")
    if tip_offset_m <= 0.0:
        raise ValueError("tip_offset_m must be positive")
    tool_axis = normalize(rotate_vector(current_orientation, (0.0, 0.0, 1.0)))
    desired_tip = add(target, scale(tool_axis, -standoff_m))
    desired_origin = add(desired_tip, scale(tool_axis, -tip_offset_m))
    translation_vector = subtract(desired_origin, current_link_origin)
    translation = norm(translation_vector)
    horizontal, vertical = horizontal_vertical_displacement(
        desired_origin, current_link_origin
    )
    if not translation_within_axis_limits(
        desired_origin,
        current_link_origin,
        maximum_horizontal_translation_m,
        maximum_vertical_translation_m,
    ):
        raise ValueError(
            f"required horizontal/vertical translation {horizontal:.3f}/{vertical:.3f} m "
            "exceeds maximum_horizontal_translation_m/maximum_vertical_translation_m "
            f"{maximum_horizontal_translation_m:.3f}/{maximum_vertical_translation_m:.3f} m"
        )
    qx, qy, qz, qw = current_orientation
    qnorm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    orientation = (qx / qnorm, qy / qnorm, qz / qnorm, qw / qnorm)
    return desired_origin, orientation, desired_tip, translation


def piper_current_tip_gap(
    current_link_origin: Vector3,
    orientation: Quaternion,
    target: Vector3,
    tip_offset_m: float,
) -> float:
    """Signed clearance from the current tip to target along probe +Z."""
    axis = normalize(rotate_vector(orientation, (0.0, 0.0, 1.0)))
    current_tip = add(current_link_origin, scale(axis, tip_offset_m))
    return sum(a * b for a, b in zip(subtract(target, current_tip), axis))


def precontact_tcp_pose(
    camera_origin: Vector3,
    pulse_target: Vector3,
    standoff_m: float,
    probe_tip_offset_m: float,
) -> tuple[Vector3, Quaternion, Vector3]:
    """Compute TCP position and orientation without commanding human contact.

    The probe extends along the TCP's local +X axis. ``probe_tip_offset_m`` is
    the measured distance from ``gripper_tcp`` to the physical probe tip.
    """
    if standoff_m < 0.02:
        raise ValueError("standoff_m must be at least 0.02 m")
    if probe_tip_offset_m < 0.0:
        raise ValueError("probe_tip_offset_m cannot be negative")
    direction = normalize(subtract(pulse_target, camera_origin))
    probe_tip = add(pulse_target, scale(direction, -standoff_m))
    tcp = add(probe_tip, scale(direction, -probe_tip_offset_m))
    return tcp, quaternion_align_x(direction), probe_tip


def inside_workspace(point: Vector3, limits: tuple[float, ...]) -> bool:
    if len(limits) != 6 or not all(math.isfinite(value) for value in limits):
        raise ValueError("workspace_limits must be [xmin, xmax, ymin, ymax, zmin, zmax]")
    x, y, z = point
    xmin, xmax, ymin, ymax, zmin, zmax = limits
    return xmin <= x <= xmax and ymin <= y <= ymax and zmin <= z <= zmax
