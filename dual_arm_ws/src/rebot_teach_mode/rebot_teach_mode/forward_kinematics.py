"""Small URDF serial-chain FK implementation used for TCP path visualization."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Mapping, Sequence


Matrix = tuple[tuple[float, float, float, float], ...]


def _identity() -> Matrix:
    return ((1.0, 0.0, 0.0, 0.0), (0.0, 1.0, 0.0, 0.0),
            (0.0, 0.0, 1.0, 0.0), (0.0, 0.0, 0.0, 1.0))


def _multiply(left: Matrix, right: Matrix) -> Matrix:
    return tuple(tuple(sum(left[row][k] * right[k][column] for k in range(4))
                       for column in range(4)) for row in range(4))


def _xyz_rpy(xyz: Sequence[float], rpy: Sequence[float]) -> Matrix:
    x, y, z = xyz
    roll, pitch, yaw = rpy
    cr, sr, cp, sp, cy, sy = (
        math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch),
        math.cos(yaw), math.sin(yaw),
    )
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, x),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, y),
        (-sp, cp * sr, cp * cr, z),
        (0.0, 0.0, 0.0, 1.0),
    )


def _axis_rotation(axis: Sequence[float], angle: float) -> Matrix:
    x, y, z = axis
    length = math.sqrt(x * x + y * y + z * z)
    if length <= 1e-12:
        raise ValueError("URDF joint axis must be non-zero")
    x, y, z = x / length, y / length, z / length
    c, s, one = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    return (
        (c + x * x * one, x * y * one - z * s, x * z * one + y * s, 0.0),
        (y * x * one + z * s, c + y * y * one, y * z * one - x * s, 0.0),
        (z * x * one - y * s, z * y * one + x * s, c + z * z * one, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )


def _values(element: ET.Element | None, attribute: str, default: str) -> tuple[float, ...]:
    text = default if element is None else element.attrib.get(attribute, default)
    return tuple(float(value) for value in text.split())


@dataclass(frozen=True)
class _Joint:
    name: str
    kind: str
    parent: str
    child: str
    origin: Matrix
    axis: tuple[float, float, float]


class SerialChainFK:
    """Compute a tip position for a non-branching path through a URDF tree."""

    def __init__(self, robot_description: str, base_link: str, tip_link: str) -> None:
        root = ET.fromstring(robot_description)
        by_child: dict[str, _Joint] = {}
        for element in root.findall("joint"):
            parent = element.find("parent").attrib["link"]
            child = element.find("child").attrib["link"]
            origin = element.find("origin")
            joint = _Joint(
                name=element.attrib["name"],
                kind=element.attrib.get("type", "fixed"),
                parent=parent,
                child=child,
                origin=_xyz_rpy(_values(origin, "xyz", "0 0 0"),
                                _values(origin, "rpy", "0 0 0")),
                axis=_values(element.find("axis"), "xyz", "1 0 0"),
            )
            by_child[child] = joint

        reversed_chain: list[_Joint] = []
        current = tip_link
        visited: set[str] = set()
        while current != base_link:
            if current in visited or current not in by_child:
                raise ValueError(f"no URDF chain from {base_link!r} to {tip_link!r}")
            visited.add(current)
            joint = by_child[current]
            reversed_chain.append(joint)
            current = joint.parent
        self.chain = tuple(reversed(reversed_chain))

    def _transform(self, joints: Mapping[str, float]) -> Matrix:
        transform = _identity()
        for joint in self.chain:
            transform = _multiply(transform, joint.origin)
            value = float(joints.get(joint.name, 0.0))
            if joint.kind in {"revolute", "continuous"}:
                transform = _multiply(transform, _axis_rotation(joint.axis, value))
            elif joint.kind == "prismatic":
                transform = _multiply(
                    transform,
                    _xyz_rpy(tuple(value * item for item in joint.axis), (0.0, 0.0, 0.0)),
                )
        return transform

    def pose(
        self, joints: Mapping[str, float]
    ) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
        """Return the tip position and quaternion (x, y, z, w) in the base frame."""
        transform = self._transform(joints)
        trace = transform[0][0] + transform[1][1] + transform[2][2]
        if trace > 0.0:
            scale = math.sqrt(trace + 1.0) * 2.0
            qw = 0.25 * scale
            qx = (transform[2][1] - transform[1][2]) / scale
            qy = (transform[0][2] - transform[2][0]) / scale
            qz = (transform[1][0] - transform[0][1]) / scale
        elif transform[0][0] > transform[1][1] and transform[0][0] > transform[2][2]:
            scale = math.sqrt(1.0 + transform[0][0] - transform[1][1] - transform[2][2]) * 2.0
            qw = (transform[2][1] - transform[1][2]) / scale
            qx = 0.25 * scale
            qy = (transform[0][1] + transform[1][0]) / scale
            qz = (transform[0][2] + transform[2][0]) / scale
        elif transform[1][1] > transform[2][2]:
            scale = math.sqrt(1.0 + transform[1][1] - transform[0][0] - transform[2][2]) * 2.0
            qw = (transform[0][2] - transform[2][0]) / scale
            qx = (transform[0][1] + transform[1][0]) / scale
            qy = 0.25 * scale
            qz = (transform[1][2] + transform[2][1]) / scale
        else:
            scale = math.sqrt(1.0 + transform[2][2] - transform[0][0] - transform[1][1]) * 2.0
            qw = (transform[1][0] - transform[0][1]) / scale
            qx = (transform[0][2] + transform[2][0]) / scale
            qy = (transform[1][2] + transform[2][1]) / scale
            qz = 0.25 * scale
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        return (
            (transform[0][3], transform[1][3], transform[2][3]),
            (qx / norm, qy / norm, qz / norm, qw / norm),
        )

    def position(
        self,
        joints: Mapping[str, float],
        local_point: Sequence[float] = (0.0, 0.0, 0.0),
    ) -> tuple[float, float, float]:
        """Return a point fixed in the tip frame, expressed in the base frame."""
        if len(local_point) != 3:
            raise ValueError("tip-frame point must contain three values")
        point = tuple(float(value) for value in local_point)
        if not all(math.isfinite(value) for value in point):
            raise ValueError("tip-frame point must be finite")
        transform = self._transform(joints)
        x, y, z = point
        return tuple(
            transform[row][0] * x
            + transform[row][1] * y
            + transform[row][2] * z
            + transform[row][3]
            for row in range(3)
        )
