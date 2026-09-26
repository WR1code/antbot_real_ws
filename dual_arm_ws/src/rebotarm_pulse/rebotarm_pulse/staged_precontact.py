"""Geometry and ordered, collision-gated candidates for Piper-H dry-run planning."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Callable

from .approach_geometry import (
    Vector3, Quaternion, add, norm, normalize, piper_precontact_link_pose,
    rotate_vector, scale, subtract,
)


def dot(a: Vector3, b: Vector3) -> float:
    return sum(x * y for x, y in zip(a, b))


def cross(a: Vector3, b: Vector3) -> Vector3:
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


@dataclass(frozen=True)
class StagedPrecontact:
    current_tip: Vector3
    target: Vector3
    hover_tip: Vector3
    hover_link: Vector3
    nominal_precontact_tip: Vector3
    nominal_precontact_link: Vector3
    nominal_approach: Vector3
    lateral_offset: Vector3
    clearance_m: float
    approach_distance_m: float
    correction_m: float
    orientation: Quaternion
    tip_offset_m: float


@dataclass(frozen=True)
class ApproachCandidate:
    tip: Vector3
    link: Vector3
    direction: Vector3
    angle_deg: float
    tangent_shift_m: float


def make_staged_precontact(
    current_link: Vector3,
    current_orientation: Quaternion,
    target: Vector3,
    standoff_m: float,
    tip_offset_m: float,
    maximum_horizontal_translation_m: float,
    maximum_vertical_translation_m: float,
    lateral_deadband_m: float = 0.001,
    allow_zero_standoff: bool = False,
) -> StagedPrecontact:
    """Plan a hover when farther than standoff, otherwise use the direct goal.

    With no measured surface normal, the current probe +Z is the nominal
    inward approach. Horizontal XY and vertical Z correction guards are
    evaluated independently in the planning frame.
    """
    nominal_link, orientation, nominal_tip, correction = piper_precontact_link_pose(
        current_link, current_orientation, target, standoff_m, tip_offset_m,
        maximum_horizontal_translation_m, maximum_vertical_translation_m,
        allow_zero_standoff=allow_zero_standoff,
    )
    inward = normalize(rotate_vector(orientation, (0.0, 0.0, 1.0)))
    current_tip = add(current_link, scale(inward, tip_offset_m))
    clearance = dot(subtract(target, current_tip), inward)
    if not math.isfinite(clearance):
        raise ValueError("current tip clearance is not finite")
    lateral = subtract(subtract(target, current_tip), scale(inward, clearance))
    if norm(lateral) <= lateral_deadband_m:
        lateral = (0.0, 0.0, 0.0)
    if clearance >= standoff_m:
        hover_tip = add(current_tip, lateral)
        hover_link = subtract(hover_tip, scale(inward, tip_offset_m))
        approach_distance = clearance - standoff_m
    else:
        # No retreat or close-range lateral waypoint: plan one direct Cartesian
        # segment from the current pose to the 60 mm nominal endpoint.
        hover_tip = nominal_tip
        hover_link = nominal_link
        approach_distance = 0.0
    return StagedPrecontact(
        current_tip, target, hover_tip, hover_link, nominal_tip, nominal_link,
        inward, lateral, clearance, approach_distance,
        correction, orientation, tip_offset_m,
    )


def approach_candidates(
    plan: StagedPrecontact,
    forearm_axis: Vector3,
    max_deviation_deg: float = 6.0,
    max_tangent_shift_m: float = 0.008,
) -> tuple[ApproachCandidate, ...]:
    """Order nominal first, then small tangential deviations at fixed attitude.

    Candidate endpoints remain at or above the nominal pre-contact clearance.
    The arm axis only selects tangent directions; it is not a surface normal.
    """
    inward = plan.nominal_approach
    if plan.approach_distance_m <= 1e-9:
        return (ApproachCandidate(
            plan.nominal_precontact_tip,
            plan.nominal_precontact_link,
            inward,
            0.0,
            0.0,
        ),)
    tangent = subtract(forearm_axis, scale(inward, dot(forearm_axis, inward)))
    if norm(tangent) < 1e-5:
        raise ValueError("forearm axis is parallel to approach; tangent undefined")
    tangent = normalize(tangent)
    angles = (0.0, 3.0, -3.0, 6.0, -6.0)
    result = []
    for angle in angles:
        if abs(angle) > max_deviation_deg + 1e-9:
            continue
        theta = math.radians(angle)
        axial_distance = plan.approach_distance_m * math.cos(theta)
        signed_tangent = plan.approach_distance_m * math.sin(theta)
        signed_tangent = max(
            -max_tangent_shift_m, min(max_tangent_shift_m, signed_tangent)
        )
        displacement = add(
            scale(inward, axial_distance), scale(tangent, signed_tangent)
        )
        tip = add(plan.hover_tip, displacement)
        link = subtract(tip, scale(inward, plan.tip_offset_m))
        direction = (
            normalize(displacement) if norm(displacement) > 1e-9 else inward
        )
        result.append(ApproachCandidate(
            tip, link, direction, angle, abs(signed_tangent)
        ))
    return tuple(result)


def first_valid_candidate(
    candidates: tuple[ApproachCandidate, ...],
    validate: Callable[[ApproachCandidate], bool],
) -> ApproachCandidate | None:
    """A failed IK/collision/path check rejects that candidate completely."""
    for candidate in candidates:
        if validate(candidate):
            return candidate
    return None
