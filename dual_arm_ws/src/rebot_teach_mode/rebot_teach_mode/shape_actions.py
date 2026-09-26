"""Geometry and trajectory helpers for the built-in drawing action pack."""

from __future__ import annotations

import math
from datetime import datetime, timezone
from typing import Iterable, Sequence

from .drawing_paths import image_strokes, text_strokes
from .trajectory import RecordedPoint, RecordedTrajectory


SUPPORTED_SHAPES = ("rectangle", "triangle", "circle", "star", "heart")
SUPPORTED_DRAWINGS = SUPPORTED_SHAPES + ("text", "image")

SHAPE_NAMES_ZH = {
    "rectangle": "矩形",
    "triangle": "三角形",
    "circle": "圆形",
    "star": "五角星",
    "heart": "心形",
    "text": "字符",
    "image": "图片简笔画",
}


def normalize_quaternion(
    quaternion: Sequence[float],
) -> tuple[float, float, float, float]:
    """Return a finite unit quaternion in ``(x, y, z, w)`` order."""
    if len(quaternion) != 4:
        raise ValueError("quaternion must contain four values")
    values = tuple(float(value) for value in quaternion)
    norm = math.sqrt(sum(value * value for value in values))
    if not all(math.isfinite(value) for value in values) or norm < 1e-12:
        raise ValueError("quaternion must be finite and non-zero")
    return tuple(value / norm for value in values)


def multiply_quaternions(
    left: Sequence[float], right: Sequence[float]
) -> tuple[float, float, float, float]:
    """Compose two quaternions, applying ``right`` and then ``left``."""
    lx, ly, lz, lw = normalize_quaternion(left)
    rx, ry, rz, rw = normalize_quaternion(right)
    return normalize_quaternion((
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    ))


def transform_points(
    points: Iterable[Sequence[float]],
    translation: Sequence[float],
    orientation: Sequence[float],
) -> tuple[tuple[float, float, float], ...]:
    """Transform local points by a pose expressed in the base frame."""
    if len(translation) != 3:
        raise ValueError("translation must contain three values")
    tx, ty, tz = (float(value) for value in translation)
    if not all(math.isfinite(value) for value in (tx, ty, tz)):
        raise ValueError("translation must be finite")
    qx, qy, qz, qw = normalize_quaternion(orientation)
    result = []
    for point in points:
        if len(point) != 3:
            raise ValueError("each point must contain three values")
        x, y, z = (float(value) for value in point)
        if not all(math.isfinite(value) for value in (x, y, z)):
            raise ValueError("points must be finite")
        # Equivalent to q * (x, y, z, 0) * conjugate(q), expanded.
        uvx = qy * z - qz * y
        uvy = qz * x - qx * z
        uvz = qx * y - qy * x
        uuvx = qy * uvz - qz * uvy
        uuvy = qz * uvx - qx * uvz
        uuvz = qx * uvy - qy * uvx
        scale = 2.0 * qw
        result.append((
            tx + x + scale * uvx + 2.0 * uuvx,
            ty + y + scale * uvy + 2.0 * uuvy,
            tz + z + scale * uvz + 2.0 * uuvz,
        ))
    return tuple(result)


def parent_pose_for_child_pose(
    local_position: Sequence[float],
    local_orientation: Sequence[float],
    target_position: Sequence[float],
    target_orientation: Sequence[float],
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    """Place a parent frame so one local child pose coincides with a target pose."""
    child = normalize_quaternion(local_orientation)
    target = normalize_quaternion(target_orientation)
    parent_orientation = multiply_quaternions(
        target, (-child[0], -child[1], -child[2], child[3])
    )
    rotated = transform_points(
        (local_position,), (0.0, 0.0, 0.0), parent_orientation
    )[0]
    if len(target_position) != 3:
        raise ValueError("target position must contain three values")
    target_xyz = tuple(float(value) for value in target_position)
    if not all(math.isfinite(value) for value in target_xyz):
        raise ValueError("target position must be finite")
    return (
        tuple(target_xyz[index] - rotated[index] for index in range(3)),
        parent_orientation,
    )


def shape_outline(
    shape: str,
    center: Sequence[float],
    width: float,
    height: float,
) -> tuple[tuple[float, float, float], ...]:
    """Return a closed XY-plane outline in the configured base frame."""
    kind = str(shape).strip().lower()
    if kind not in SUPPORTED_SHAPES:
        raise ValueError(f"unsupported shape {shape!r}")
    if len(center) != 3 or not all(math.isfinite(float(value)) for value in center):
        raise ValueError("shape center must contain three finite values")
    if not math.isfinite(width) or not math.isfinite(height) or width <= 0 or height <= 0:
        raise ValueError("shape width and height must be positive")
    cx, cy, cz = (float(value) for value in center)
    half_w, half_h = width * 0.5, height * 0.5

    if kind == "rectangle":
        unit = ((-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0))
    elif kind == "triangle":
        unit = ((0.0, 1.0), (-1.0, -1.0), (1.0, -1.0))
    elif kind == "circle":
        unit = tuple(
            (math.cos(2.0 * math.pi * index / 48.0),
             math.sin(2.0 * math.pi * index / 48.0))
            for index in range(48)
        )
    elif kind == "star":
        unit = tuple(
            ((1.0 if index % 2 == 0 else 0.42) * math.cos(math.pi / 2 + index * math.pi / 5),
             (1.0 if index % 2 == 0 else 0.42) * math.sin(math.pi / 2 + index * math.pi / 5))
            for index in range(10)
        )
    else:
        # Classic parametric heart, normalized to roughly [-1, 1] in both axes.
        unit = tuple(
            (math.sin(angle) ** 3,
             (13 * math.cos(angle) - 5 * math.cos(2 * angle)
              - 2 * math.cos(3 * angle) - math.cos(4 * angle)) / 17.0)
            for angle in (2.0 * math.pi * index / 64.0 for index in range(64))
        )

    result = tuple((cx + x * half_w, cy + y * half_h, cz) for x, y in unit)
    return result + (result[0],)


def drawing_strokes(
    drawing: str,
    source: str,
    width: float,
    height: float,
    *,
    image_maximum_strokes: int = 24,
    image_maximum_points: int = 480,
    image_minimum_contour_length_px: float = 24.0,
    image_simplify_epsilon_px: float = 2.0,
) -> tuple[tuple[tuple[float, float, float], ...], ...]:
    """Return one or more local drawing-plane strokes for a preset or custom source."""
    kind = str(drawing).strip().lower()
    if kind in SUPPORTED_SHAPES:
        return (shape_outline(kind, (0.0, 0.0, 0.0), width, height),)
    if kind == "text":
        return text_strokes(source, width, height)
    if kind == "image":
        return image_strokes(
            source,
            width,
            height,
            maximum_strokes=image_maximum_strokes,
            maximum_points=image_maximum_points,
            minimum_contour_length_px=image_minimum_contour_length_px,
            simplify_epsilon_px=image_simplify_epsilon_px,
        )
    raise ValueError(f"unsupported drawing {drawing!r}")


def connect_strokes_with_lifts(
    strokes: Sequence[Sequence[Sequence[float]]],
    pen_lift: float,
) -> tuple[tuple[float, float, float], ...]:
    """Join local strokes with +Z pen-up travel, returning points after the first."""
    if not math.isfinite(pen_lift) or pen_lift <= 0.0:
        raise ValueError("pen lift must be positive")
    cleaned: list[tuple[tuple[float, float, float], ...]] = []
    for stroke in strokes:
        points = tuple(tuple(float(value) for value in point) for point in stroke)
        if len(points) < 2 or any(len(point) != 3 for point in points):
            raise ValueError("every drawing stroke must contain at least two 3-D points")
        if not all(math.isfinite(value) for point in points for value in point):
            raise ValueError("drawing strokes must be finite")
        cleaned.append(points)
    if not cleaned:
        raise ValueError("drawing contains no strokes")
    result: list[tuple[float, float, float]] = list(cleaned[0][1:])
    for previous, current in zip(cleaned, cleaned[1:]):
        end, start = previous[-1], current[0]
        result.extend((
            (end[0], end[1], end[2] + pen_lift),
            (start[0], start[1], start[2] + pen_lift),
            start,
        ))
        result.extend(current[1:])
    return tuple(result)


def pen_tcp_path(
    strokes: Sequence[Sequence[Sequence[float]]],
    pen_length: float,
    pen_lift: float,
) -> tuple[tuple[float, float, float], ...]:
    """Build the complete TCP path for a pen whose tip follows ``strokes``.

    Drawing points lie on the local XY plane. For the physical reBot pen mount,
    the pen extends along gripper_tcp +X. With the configured drawing orientation
    this is local -Z, while local +Z is the pen-up direction towards the arm. A
    TCP holding a pen of ``pen_length`` is therefore that far on the +Z side of
    the physical tip. The returned path starts
    above the first stroke, lowers vertically, lifts between disconnected
    strokes, and finishes above the last stroke. This keeps both the approach
    and retreat from marking blank parts of the workpiece.
    """
    if not math.isfinite(pen_length) or pen_length < 0.0:
        raise ValueError("pen length must be finite and non-negative")
    if not math.isfinite(pen_lift) or pen_lift <= 0.0:
        raise ValueError("pen lift must be positive")

    cleaned: list[tuple[tuple[float, float, float], ...]] = []
    for stroke in strokes:
        points = tuple(tuple(float(value) for value in point) for point in stroke)
        if len(points) < 2 or any(len(point) != 3 for point in points):
            raise ValueError("every drawing stroke must contain at least two 3-D points")
        if not all(math.isfinite(value) for point in points for value in point):
            raise ValueError("drawing strokes must be finite")
        cleaned.append(points)
    if not cleaned:
        raise ValueError("drawing contains no strokes")

    def tcp(point: Sequence[float], extra_lift: float = 0.0):
        return (point[0], point[1], point[2] + pen_length + extra_lift)

    first = cleaned[0][0]
    result: list[tuple[float, float, float]] = [
        tcp(first, pen_lift),
        tcp(first),
    ]
    result.extend(tcp(point) for point in cleaned[0][1:])
    for previous, current in zip(cleaned, cleaned[1:]):
        result.extend((
            tcp(previous[-1], pen_lift),
            tcp(current[0], pen_lift),
            tcp(current[0]),
        ))
        result.extend(tcp(point) for point in current[1:])
    result.append(tcp(cleaned[-1][-1], pen_lift))
    return tuple(result)


def sample_cartesian_path(
    points: Sequence[Sequence[float]],
    maximum_step: float,
    maximum_points: int,
) -> tuple[tuple[float, float, float], ...]:
    """Densify a Cartesian polyline and uniformly cap diagnostic samples."""
    if not math.isfinite(maximum_step) or maximum_step <= 0.0:
        raise ValueError("maximum Cartesian sample step must be positive")
    if maximum_points < 2:
        raise ValueError("maximum Cartesian sample count must be at least two")
    cleaned = tuple(tuple(float(value) for value in point) for point in points)
    if len(cleaned) < 2 or any(len(point) != 3 for point in cleaned):
        raise ValueError("Cartesian path must contain at least two 3-D points")
    if not all(math.isfinite(value) for point in cleaned for value in point):
        raise ValueError("Cartesian path points must be finite")
    dense = [cleaned[0]]
    for first, second in zip(cleaned, cleaned[1:]):
        distance = math.sqrt(sum((b - a) ** 2 for a, b in zip(first, second)))
        steps = max(1, int(math.ceil(distance / maximum_step)))
        dense.extend(
            tuple(a + (b - a) * index / steps for a, b in zip(first, second))
            for index in range(1, steps + 1)
        )
    if len(dense) <= maximum_points:
        return tuple(dense)
    indices = [
        round(index * (len(dense) - 1) / (maximum_points - 1))
        for index in range(maximum_points)
    ]
    return tuple(dense[index] for index in indices)


def retime_joint_path(
    joint_names: Sequence[str],
    position_rows: Iterable[Sequence[float]],
    maximum_joint_velocity: float,
    minimum_step_sec: float = 0.02,
) -> RecordedTrajectory:
    """Deduplicate and conservatively retime a planned joint-space path."""
    names = tuple(str(name) for name in joint_names)
    if not names or maximum_joint_velocity <= 0.0 or minimum_step_sec <= 0.0:
        raise ValueError("invalid path timing configuration")
    rows: list[tuple[float, ...]] = []
    for raw in position_rows:
        row = tuple(float(value) for value in raw)
        if len(row) != len(names) or not all(math.isfinite(value) for value in row):
            raise ValueError("planned path contains an invalid joint row")
        if not rows or max(abs(a - b) for a, b in zip(row, rows[-1])) > 1e-8:
            rows.append(row)
    if len(rows) < 2:
        raise ValueError("planned shape path contains fewer than two distinct points")

    stamp = 0.0
    points = [RecordedPoint(stamp, rows[0])]
    for previous, current in zip(rows, rows[1:]):
        maximum_delta = max(abs(a - b) for a, b in zip(previous, current))
        stamp += max(minimum_step_sec, maximum_delta / maximum_joint_velocity)
        points.append(RecordedPoint(stamp, current))
    return RecordedTrajectory(
        joint_names=names,
        points=tuple(points),
        created_utc=datetime.now(timezone.utc).isoformat(),
    )
