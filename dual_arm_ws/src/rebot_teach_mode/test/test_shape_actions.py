import math

import pytest

from rebot_teach_mode.drawing_paths import SUPPORTED_TEXT_CHARACTERS
from rebot_teach_mode.shape_actions import (
    SUPPORTED_SHAPES,
    connect_strokes_with_lifts,
    drawing_strokes,
    multiply_quaternions,
    pen_tcp_path,
    parent_pose_for_child_pose,
    sample_cartesian_path,
    retime_joint_path,
    shape_outline,
    transform_points,
)


def test_parent_pose_aligns_local_child_with_target_pose():
    half = math.sqrt(0.5)
    parent_position, parent_orientation = parent_pose_for_child_pose(
        (0.1, 0.0, 0.0),
        (0.0, half, 0.0, half),
        (0.4, 0.2, 0.3),
        (0.0, 0.0, half, half),
    )
    assert multiply_quaternions(
        parent_orientation, (0.0, half, 0.0, half)
    ) == pytest.approx((0.0, 0.0, half, half))
    assert transform_points(
        ((0.1, 0.0, 0.0),), parent_position, parent_orientation
    )[0] == pytest.approx((0.4, 0.2, 0.3))


@pytest.mark.parametrize("shape", SUPPORTED_SHAPES)
def test_every_shape_outline_is_closed_and_finite(shape):
    points = shape_outline(shape, (0.28, 0.0, 0.12), 0.06, 0.08)
    assert len(points) >= 4
    assert points[0] == points[-1]
    assert all(math.isfinite(value) for point in points for value in point)
    assert all(point[2] == pytest.approx(0.12) for point in points)


def test_retime_joint_path_deduplicates_and_enforces_velocity():
    trajectory = retime_joint_path(
        ("joint1", "joint2"),
        ((0.0, 0.0), (0.0, 0.0), (0.1, 0.2), (0.2, 0.2)),
        maximum_joint_velocity=0.1,
    )
    assert len(trajectory.points) == 3
    for first, second in zip(trajectory.points, trajectory.points[1:]):
        delta = max(abs(a - b) for a, b in zip(first.positions, second.positions))
        assert delta / (second.time_from_start - first.time_from_start) <= 0.100001


def test_shape_outline_rejects_unknown_or_invalid_dimensions():
    with pytest.raises(ValueError, match="unsupported"):
        shape_outline("hexagon", (0.0, 0.0, 0.0), 1.0, 1.0)
    with pytest.raises(ValueError, match="positive"):
        shape_outline("circle", (0.0, 0.0, 0.0), 0.0, 1.0)


def test_transform_points_applies_shape_pose_translation_and_rotation():
    half_turn_about_z = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    transformed = transform_points(
        ((1.0, 0.0, 0.0), (0.0, 2.0, 0.0)),
        (0.2, -0.1, 0.3),
        half_turn_about_z,
    )
    assert transformed[0] == pytest.approx((0.2, 0.9, 0.3))
    assert transformed[1] == pytest.approx((-1.8, -0.1, 0.3))


def test_shape_frame_rotation_is_composed_with_tcp_orientation():
    z_90 = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    x_90 = (math.sin(math.pi / 4), 0.0, 0.0, math.cos(math.pi / 4))
    composed = multiply_quaternions(z_90, x_90)
    rotated = transform_points(((0.0, 1.0, 0.0),), (0.0, 0.0, 0.0), composed)
    assert rotated[0] == pytest.approx((0.0, 0.0, 1.0), abs=1e-9)


@pytest.mark.parametrize(
    "orientation",
    ((0.0, 0.0, 0.0, 0.0), (float("nan"), 0.0, 0.0, 1.0)),
)
def test_transform_points_rejects_invalid_orientation(orientation):
    with pytest.raises(ValueError, match="quaternion"):
        transform_points(((0.0, 0.0, 0.0),), (0.0, 0.0, 0.0), orientation)


def test_text_drawing_supports_letters_numbers_symbols_and_pen_lifts():
    strokes = drawing_strokes("text", "Az 09+?", 0.12, 0.05)
    assert len(strokes) >= 7
    assert all(len(stroke) >= 2 for stroke in strokes)
    points = [point for stroke in strokes for point in stroke]
    assert max(point[0] for point in points) <= 0.06 + 1e-9
    assert min(point[0] for point in points) >= -0.06 - 1e-9
    path = connect_strokes_with_lifts(strokes, 0.01)
    assert any(point[2] == pytest.approx(0.01) for point in path)


def test_pen_tcp_path_offsets_tool_and_lifts_before_and_after_drawing():
    strokes = (
        ((0.0, 0.0, 0.0), (0.1, 0.0, 0.0)),
        ((0.2, 0.0, 0.0), (0.3, 0.0, 0.0)),
    )
    path = pen_tcp_path(strokes, pen_length=0.08, pen_lift=0.012)
    assert path[0] == pytest.approx((0.0, 0.0, 0.092))
    assert path[1] == pytest.approx((0.0, 0.0, 0.08))
    assert path[-1] == pytest.approx((0.3, 0.0, 0.092))
    assert (0.1, 0.0, 0.092) in path
    assert (0.2, 0.0, 0.092) in path
    assert (0.2, 0.0, 0.08) in path


@pytest.mark.parametrize("length", (-0.01, float("nan")))
def test_pen_tcp_path_rejects_invalid_length(length):
    with pytest.raises(ValueError, match="pen length"):
        pen_tcp_path((((0.0, 0.0, 0.0), (0.1, 0.0, 0.0)),), length, 0.01)


def test_cartesian_preflight_sampling_densifies_and_caps_path():
    sampled = sample_cartesian_path(
        ((0.0, 0.0, 0.0), (0.1, 0.0, 0.0)), 0.01, 6
    )
    assert len(sampled) == 6
    assert sampled[0] == (0.0, 0.0, 0.0)
    assert sampled[-1] == (0.1, 0.0, 0.0)
    dense = sample_cartesian_path(
        ((0.0, 0.0, 0.0), (0.02, 0.0, 0.0)), 0.01, 20
    )
    assert dense == ((0.0, 0.0, 0.0), (0.01, 0.0, 0.0), (0.02, 0.0, 0.0))


def test_text_drawing_rejects_unsupported_or_excessive_text():
    with pytest.raises(ValueError, match="unsupported"):
        drawing_strokes("text", "机器人", 0.1, 0.1)
    with pytest.raises(ValueError, match="limited"):
        drawing_strokes("text", "A" * 25, 0.1, 0.1)


def test_every_advertised_text_character_has_valid_strokes():
    for character in SUPPORTED_TEXT_CHARACTERS.replace(" ", ""):
        strokes = drawing_strokes("text", character, 0.1, 0.1)
        assert strokes
        assert all(len(stroke) >= 2 for stroke in strokes)


def test_image_drawing_extracts_simplified_bounded_contours(tmp_path):
    cv2 = pytest.importorskip("cv2")
    numpy = pytest.importorskip("numpy")
    image = numpy.full((160, 240), 255, dtype=numpy.uint8)
    cv2.rectangle(image, (30, 30), (210, 130), 0, 4)
    image_path = tmp_path / "outline.png"
    assert cv2.imwrite(str(image_path), image)
    strokes = drawing_strokes(
        "image", str(image_path), 0.12, 0.08,
        image_maximum_strokes=4, image_maximum_points=80,
    )
    assert 1 <= len(strokes) <= 4
    assert sum(len(stroke) for stroke in strokes) <= 80
    points = [point for stroke in strokes for point in stroke]
    assert max(abs(point[0]) for point in points) <= 0.06 + 1e-9
    assert max(abs(point[1]) for point in points) <= 0.04 + 1e-9
    assert all(stroke[0] == stroke[-1] for stroke in strokes)
