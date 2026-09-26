from dataclasses import dataclass

from meridian_hand_vision.gesture_classifier import (
    StableGestureFilter,
    classify_gesture,
)


@dataclass
class Point:
    x: float = 0.5
    y: float = 0.5


def hand(fingers, thumb="folded"):
    points = [Point() for _ in range(21)]
    points[0] = Point(0.5, 0.9)
    for extended, indices, x in zip(
        fingers,
        ((5, 6, 8), (9, 10, 12), (13, 14, 16), (17, 18, 20)),
        (0.35, 0.45, 0.55, 0.65),
    ):
        mcp, pip, tip = indices
        points[mcp] = Point(x, 0.68)
        points[pip] = Point(x, 0.48)
        points[tip] = Point(x, 0.22) if extended else Point(x + 0.05, 0.68)
    if thumb == "up":
        points[2] = Point(0.32, 0.68)
        points[3] = Point(0.32, 0.48)
        points[4] = Point(0.32, 0.20)
    else:
        points[2] = Point(0.36, 0.72)
        points[3] = Point(0.30, 0.72)
        points[4] = Point(0.40, 0.72)
    return points


def test_distinct_demo_gestures():
    assert classify_gesture(hand((True, True, True, True))) == "open_palm"
    assert classify_gesture(hand((True, True, False, False))) == "victory"
    assert classify_gesture(hand((False, False, False, False), "up")) == "thumbs_up"
    assert classify_gesture(hand((False, False, False, False))) == "fist"


def test_filter_requires_hold_and_rearm():
    stable = StableGestureFilter(hold_frames=3)
    assert stable.update("victory") is None
    assert stable.update("victory") is None
    assert stable.update("victory") == "victory"
    assert stable.update("victory") is None
    assert stable.update("unknown") is None
    assert stable.update("unknown") is None
    assert stable.update("unknown") is None
    assert stable.update("victory") is None
    assert stable.update("victory") is None
    assert stable.update("victory") == "victory"
