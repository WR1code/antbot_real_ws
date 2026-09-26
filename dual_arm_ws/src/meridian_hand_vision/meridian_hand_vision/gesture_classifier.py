"""Small deterministic gesture vocabulary for opt-in demo triggering."""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass
from typing import Any


UNKNOWN = "unknown"
FINGER_JOINTS = (
    (5, 6, 8),
    (9, 10, 12),
    (13, 14, 16),
    (17, 18, 20),
)


def _xy(point: Any) -> tuple[float, float]:
    return float(point.x), float(point.y)


def _distance(first: Any, second: Any) -> float:
    ax, ay = _xy(first)
    bx, by = _xy(second)
    return math.hypot(ax - bx, ay - by)


def _joint_angle(first: Any, middle: Any, last: Any) -> float:
    ax, ay = _xy(first)
    bx, by = _xy(middle)
    cx, cy = _xy(last)
    first_vector = (ax - bx, ay - by)
    second_vector = (cx - bx, cy - by)
    denominator = math.hypot(*first_vector) * math.hypot(*second_vector)
    if denominator <= 1e-9:
        return 0.0
    cosine = max(
        -1.0,
        min(
            1.0,
            (
                first_vector[0] * second_vector[0]
                + first_vector[1] * second_vector[1]
            )
            / denominator,
        ),
    )
    return math.degrees(math.acos(cosine))


def _finger_extended(landmarks: Sequence[Any], joints: tuple[int, int, int]) -> bool:
    mcp, pip, tip = (landmarks[index] for index in joints)
    return (
        _joint_angle(mcp, pip, tip) >= 150.0
        and _distance(landmarks[0], tip) >= _distance(landmarks[0], pip) * 1.08
    )


def classify_gesture(landmarks: Sequence[Any]) -> str:
    """Classify four deliberately distinct, front-facing demonstration poses."""
    if len(landmarks) < 21:
        return UNKNOWN
    extended = tuple(_finger_extended(landmarks, joints) for joints in FINGER_JOINTS)
    thumb_extended = (
        _joint_angle(landmarks[2], landmarks[3], landmarks[4]) >= 145.0
        and _distance(landmarks[0], landmarks[4])
        >= _distance(landmarks[0], landmarks[3]) * 1.08
    )

    if all(extended):
        return "open_palm"
    if extended == (True, True, False, False):
        return "victory"
    if not any(extended):
        wrist_y = float(landmarks[0].y)
        thumb_tip_y = float(landmarks[4].y)
        if thumb_extended and thumb_tip_y <= wrist_y - 0.12:
            return "thumbs_up"
        if not thumb_extended:
            return "fist"
    return UNKNOWN


@dataclass
class StableGestureFilter:
    """Emit only a gesture held for N frames and only once per stable hold."""

    hold_frames: int = 12
    _candidate: str = UNKNOWN
    _count: int = 0
    _emitted: str = UNKNOWN

    def __post_init__(self) -> None:
        if self.hold_frames < 1:
            raise ValueError("hold_frames must be positive")

    def update(self, gesture: str) -> str | None:
        if gesture != self._candidate:
            self._candidate = gesture
            self._count = 1
        else:
            self._count += 1
        if self._count < self.hold_frames or gesture == self._emitted:
            return None
        self._emitted = gesture
        if gesture == UNKNOWN:
            return None
        return gesture
