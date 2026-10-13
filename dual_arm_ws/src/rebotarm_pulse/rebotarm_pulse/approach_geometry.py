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


def point_to_segment_distance(
    point: Vector3, segment_start: Vector3, segment_end: Vector3
) -> float:
    """Return the shortest Euclidean distance from a point to a closed segment."""
    direction = subtract(segment_end, segment_start)
    length_squared = sum(value * value for value in direction)
    if not math.isfinite(length_squared):
        raise ValueError("segment coordinates are not finite")
    if length_squared < 1e-12:
        return norm(subtract(point, segment_start))
    projection = sum(
        offset * axis for offset, axis in zip(subtract(point, segment_start), direction)
    ) / length_squared
    projection = max(0.0, min(1.0, projection))
    closest = add(segment_start, scale(direction, projection))
    return norm(subtract(point, closest))


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


def quaternion_align_local_z(
    current_orientation: Quaternion, desired_world_direction: Vector3
) -> Quaternion:
    """Minimally rotate an attitude so local blue +Z follows a world vector."""
    desired = normalize(desired_world_direction)
    current_z = normalize(rotate_vector(current_orientation, (0.0, 0.0, 1.0)))
    dot_value = max(-1.0, min(1.0, sum(a * b for a, b in zip(current_z, desired))))
    cross = (
        current_z[1] * desired[2] - current_z[2] * desired[1],
        current_z[2] * desired[0] - current_z[0] * desired[2],
        current_z[0] * desired[1] - current_z[1] * desired[0],
    )
    if dot_value < -0.999999:
        # The 180-degree case has no unique shortest axis. Use current local +X
        # in the world frame to preserve a deterministic roll choice.
        axis = normalize(rotate_vector(current_orientation, (1.0, 0.0, 0.0)))
        delta = (axis[0], axis[1], axis[2], 0.0)
    else:
        delta = (cross[0], cross[1], cross[2], 1.0 + dot_value)
        length = math.sqrt(sum(value * value for value in delta))
        delta = tuple(value / length for value in delta)
    dx, dy, dz, dw = delta
    qx, qy, qz, qw = current_orientation
    result = (
        dw * qx + dx * qw + dy * qz - dz * qy,
        dw * qy - dx * qz + dy * qw + dz * qx,
        dw * qz + dx * qy - dy * qx + dz * qw,
        dw * qw - dx * qx - dy * qy - dz * qz,
    )
    length = math.sqrt(sum(value * value for value in result))
    return tuple(value / length for value in result)


def quaternion_roll_about_local_z(
    orientation: Quaternion, angle_deg: float
) -> Quaternion:
    """Apply a roll about local +Z without changing its world direction."""
    qx, qy, qz, qw = orientation
    qnorm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if not math.isfinite(qnorm) or qnorm < 1e-9 or not math.isfinite(angle_deg):
        raise ValueError("tool orientation or local-Z roll is invalid")
    qx, qy, qz, qw = qx / qnorm, qy / qnorm, qz / qnorm, qw / qnorm
    half_angle = math.radians(angle_deg) * 0.5
    sin_half = math.sin(half_angle)
    cos_half = math.cos(half_angle)
    # Compose q * q_local_z: the added rotation is expressed in the tool frame.
    result = (
        qx * cos_half + qy * sin_half,
        qy * cos_half - qx * sin_half,
        qz * cos_half + qw * sin_half,
        qw * cos_half - qz * sin_half,
    )
    length = math.sqrt(sum(value * value for value in result))
    return tuple(value / length for value in result)


def piper_precontact_link_pose(
    current_link_origin: Vector3,
    current_orientation: Quaternion,
    target: Vector3,
    standoff_m: float,
    tip_offset_m: float,
    maximum_horizontal_translation_m: float,
    maximum_vertical_translation_m: float,
    allow_zero_standoff: bool = False,
    approach_axis: Vector3 | None = None,
) -> tuple[Vector3, Quaternion, Vector3, float]:
    """Preserve J6 orientation and translate the selected endpoint to standoff.

    ``approach_axis`` is expressed in the planning frame.  When omitted, the
    installed probe's Link6-local +Z axis is used.  A zero ``tip_offset_m`` is
    valid for a bare Link6 flange.
    """
    if standoff_m < 0.0 or (standoff_m < 0.02 and not (allow_zero_standoff and standoff_m == 0.0)):
        raise ValueError("standoff_m must be at least 0.02 m")
    if tip_offset_m < 0.0:
        raise ValueError("tip_offset_m cannot be negative")
    tool_axis = normalize(
        approach_axis
        if approach_axis is not None
        else rotate_vector(current_orientation, (0.0, 0.0, 1.0))
    )
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
