"""Convert text and raster images into small, planar drawing stroke sets."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Iterable, Sequence


Point2 = tuple[float, float]
Stroke2 = tuple[Point2, ...]
Point3 = tuple[float, float, float]
Stroke3 = tuple[Point3, ...]

MAX_TEXT_CHARACTERS = 24
SUPPORTED_TEXT_CHARACTERS = (
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz"
    "0123456789 .,:;!?+-*/=_#@%&()[]<>"
)


def _arc(
    cx: float,
    cy: float,
    rx: float,
    ry: float,
    start_degrees: float,
    end_degrees: float,
    samples: int = 9,
) -> Stroke2:
    return tuple(
        (
            cx + rx * math.cos(math.radians(
                start_degrees + (end_degrees - start_degrees) * index / (samples - 1)
            )),
            cy + ry * math.sin(math.radians(
                start_degrees + (end_degrees - start_degrees) * index / (samples - 1)
            )),
        )
        for index in range(samples)
    )


def _glyphs() -> dict[str, tuple[Stroke2, ...]]:
    """Return a compact single-line drafting font in a 1 x 1 em box."""
    def line(*points: tuple[float, float]) -> Stroke2:
        return tuple((float(x), float(y)) for x, y in points)

    circle = _arc(0.5, 0.5, 0.48, 0.5, 0.0, 360.0, 17)
    glyphs: dict[str, tuple[Stroke2, ...]] = {
        "A": (line((0, 0), (0.5, 1), (1, 0)), line((0.22, 0.42), (0.78, 0.42))),
        "B": (line((0, 0), (0, 1), (0.62, 1), (0.92, 0.78), (0.62, 0.54),
                   (0, 0.54), (0.64, 0.54), (0.96, 0.27), (0.64, 0), (0, 0)),),
        "C": (_arc(0.52, 0.5, 0.5, 0.5, 45, 315, 13),),
        "D": (line((0, 0), (0, 1), (0.55, 1), (0.95, 0.72), (0.95, 0.28),
                   (0.55, 0), (0, 0)),),
        "E": (line((0.95, 1), (0, 1), (0, 0), (0.95, 0)), line((0, 0.52), (0.72, 0.52))),
        "F": (line((0, 0), (0, 1), (0.95, 1)), line((0, 0.52), (0.72, 0.52))),
        "G": (_arc(0.52, 0.5, 0.5, 0.5, 45, 315, 13),
              line((0.53, 0.48), (1, 0.48), (1, 0.12))),
        "H": (line((0, 0), (0, 1)), line((1, 0), (1, 1)), line((0, 0.5), (1, 0.5))),
        "I": (line((0, 1), (1, 1)), line((0.5, 1), (0.5, 0)), line((0, 0), (1, 0))),
        "J": (line((0, 1), (1, 1), (1, 0.25), (0.75, 0), (0.3, 0), (0.05, 0.22)),),
        "K": (line((0, 0), (0, 1)), line((1, 1), (0, 0.46), (1, 0))),
        "L": (line((0, 1), (0, 0), (1, 0)),),
        "M": (line((0, 0), (0, 1), (0.5, 0.45), (1, 1), (1, 0)),),
        "N": (line((0, 0), (0, 1), (1, 0), (1, 1)),),
        "O": (circle,),
        "P": (line((0, 0), (0, 1), (0.62, 1), (0.96, 0.76), (0.62, 0.51), (0, 0.51)),),
        "Q": (circle, line((0.58, 0.35), (1.0, -0.06))),
        "R": (line((0, 0), (0, 1), (0.62, 1), (0.96, 0.76), (0.62, 0.51), (0, 0.51)),
              line((0.55, 0.51), (1, 0))),
        "S": (_arc(0.5, 0.75, 0.48, 0.25, 25, 270, 9) +
              _arc(0.5, 0.25, 0.48, 0.25, 90, 335, 9)[1:],),
        "T": (line((0, 1), (1, 1)), line((0.5, 1), (0.5, 0))),
        "U": (line((0, 1), (0, 0.25), (0.25, 0), (0.75, 0), (1, 0.25), (1, 1)),),
        "V": (line((0, 1), (0.5, 0), (1, 1)),),
        "W": (line((0, 1), (0.2, 0), (0.5, 0.55), (0.8, 0), (1, 1)),),
        "X": (line((0, 1), (1, 0)), line((1, 1), (0, 0))),
        "Y": (line((0, 1), (0.5, 0.5), (1, 1)), line((0.5, 0.5), (0.5, 0))),
        "Z": (line((0, 1), (1, 1), (0, 0), (1, 0)),),
        "0": (circle, line((0.18, 0.15), (0.82, 0.85))),
        "1": (line((0.22, 0.76), (0.5, 1), (0.5, 0)), line((0.15, 0), (0.85, 0))),
        "2": (line((0.05, 0.76), (0.28, 1), (0.72, 1), (0.95, 0.76),
                   (0.05, 0), (1, 0)),),
        "3": (line((0.05, 0.92), (0.42, 1), (0.87, 0.88), (0.55, 0.52),
                   (0.9, 0.28), (0.72, 0), (0.22, 0), (0.03, 0.1)),),
        "4": (line((0.75, 0), (0.75, 1), (0, 0.32), (1, 0.32)),),
        "5": (line((0.95, 1), (0.08, 1), (0.08, 0.55), (0.7, 0.55),
                   (0.95, 0.32), (0.75, 0), (0.22, 0), (0.03, 0.12)),),
        "6": (line((0.88, 0.9), (0.65, 1), (0.25, 0.82), (0.05, 0.45),
                   (0.18, 0.08), (0.55, 0), (0.92, 0.2), (0.78, 0.52), (0.1, 0.52)),),
        "7": (line((0, 1), (1, 1), (0.28, 0)),),
        "8": (_arc(0.5, 0.75, 0.42, 0.25, 0, 360, 11),
              _arc(0.5, 0.25, 0.46, 0.25, 0, 360, 11)),
        "9": (line((0.12, 0.1), (0.35, 0), (0.75, 0.18), (0.95, 0.55),
                   (0.82, 0.92), (0.45, 1), (0.08, 0.8), (0.22, 0.48), (0.9, 0.48)),),
        ".": (line((0.47, 0), (0.53, 0)),),
        ",": (line((0.55, 0.08), (0.42, -0.12)),),
        ":": (line((0.47, 0.72), (0.53, 0.72)), line((0.47, 0.18), (0.53, 0.18))),
        ";": (line((0.47, 0.72), (0.53, 0.72)), line((0.55, 0.18), (0.42, -0.08))),
        "!": (line((0.5, 1), (0.5, 0.25)), line((0.47, 0), (0.53, 0))),
        "?": (line((0.05, 0.75), (0.25, 1), (0.7, 1), (0.95, 0.75),
                   (0.55, 0.43), (0.5, 0.25)), line((0.47, 0), (0.53, 0))),
        "+": (line((0.5, 0.15), (0.5, 0.85)), line((0.15, 0.5), (0.85, 0.5))),
        "-": (line((0.12, 0.5), (0.88, 0.5)),),
        "_": (line((0, 0), (1, 0)),),
        "=": (line((0.12, 0.65), (0.88, 0.65)), line((0.12, 0.35), (0.88, 0.35))),
        "/": (line((0, 0), (1, 1)),),
        "*": (line((0.5, 0.1), (0.5, 0.9)), line((0.15, 0.3), (0.85, 0.7)),
              line((0.15, 0.7), (0.85, 0.3))),
        "#": (line((0.3, 0), (0.45, 1)), line((0.62, 0), (0.77, 1)),
              line((0.1, 0.35), (0.9, 0.35)), line((0.1, 0.68), (0.9, 0.68))),
        "@": (circle, _arc(0.52, 0.5, 0.25, 0.27, 40, 360, 10) +
              line((0.77, 0.5), (0.77, 0.22), (0.95, 0.22))),
        "%": (line((0, 0), (1, 1)), _arc(0.22, 0.78, 0.18, 0.18, 0, 360, 9),
              _arc(0.78, 0.22, 0.18, 0.18, 0, 360, 9)),
        "&": (_arc(0.43, 0.72, 0.3, 0.28, -35, 300, 12) +
              line((0.62, 0.5), (0.1, 0.12), (0.35, 0), (0.95, 0.65)),),
        "(": (_arc(0.72, 0.5, 0.45, 0.55, 105, 255, 9),),
        ")": (_arc(0.28, 0.5, 0.45, 0.55, -75, 75, 9),),
        "[": (line((0.75, 1), (0.25, 1), (0.25, 0), (0.75, 0)),),
        "]": (line((0.25, 1), (0.75, 1), (0.75, 0), (0.25, 0)),),
        "<": (line((0.85, 0.9), (0.15, 0.5), (0.85, 0.1)),),
        ">": (line((0.15, 0.9), (0.85, 0.5), (0.15, 0.1)),),
    }
    # A readable, compact approximation is preferable to rejecting lowercase input.
    glyphs.update({letter.lower(): strokes for letter, strokes in tuple(glyphs.items())
                   if "A" <= letter <= "Z"})
    return glyphs


_GLYPHS = _glyphs()


def _validate_size(width: float, height: float) -> tuple[float, float]:
    width, height = float(width), float(height)
    if not math.isfinite(width) or not math.isfinite(height) or width <= 0 or height <= 0:
        raise ValueError("drawing width and height must be positive")
    return width, height


def text_strokes(text: str, width: float, height: float) -> tuple[Stroke3, ...]:
    """Lay out supported ASCII text as centered, pen-up-separated line strokes."""
    width, height = _validate_size(width, height)
    value = str(text)
    if not value.strip():
        raise ValueError("text drawing cannot be empty")
    if len(value) > MAX_TEXT_CHARACTERS:
        raise ValueError(f"text drawing is limited to {MAX_TEXT_CHARACTERS} characters")
    unsupported = sorted({
        character
        for character in value
        if character != " " and character not in _GLYPHS
    })
    if unsupported:
        raise ValueError(
            "unsupported text characters: "
            + " ".join(repr(item) for item in unsupported)
        )

    advances = [0.62 if character == " " else 1.22 for character in value]
    total = max(1.0, sum(advances) - 0.22)
    cursor = 0.0
    result: list[Stroke3] = []
    for character, advance in zip(value, advances):
        for stroke in _GLYPHS.get(character, ()):
            result.append(tuple(
                (
                    ((cursor + x) / total - 0.5) * width,
                    (y - 0.5) * height,
                    0.0,
                )
                for x, y in stroke
            ))
        cursor += advance
    if not result:
        raise ValueError("text drawing contains no drawable characters")
    return tuple(result)


def _subsample_closed(points: Sequence[Point2], maximum: int) -> list[Point2]:
    if len(points) <= maximum:
        return list(points)
    count = max(3, int(maximum))
    return [points[round(index * (len(points) - 1) / (count - 1))]
            for index in range(count)]


def image_strokes(
    image_path: str,
    width: float,
    height: float,
    *,
    maximum_strokes: int = 24,
    maximum_points: int = 480,
    minimum_contour_length_px: float = 24.0,
    simplify_epsilon_px: float = 2.0,
) -> tuple[Stroke3, ...]:
    """Extract, simplify and center the strongest Canny contours from an image."""
    width, height = _validate_size(width, height)
    source = Path(str(image_path)).expanduser().resolve()
    if not source.is_file():
        raise ValueError(f"image file does not exist: {source}")
    if source.stat().st_size > 20 * 1024 * 1024:
        raise ValueError("image file must not exceed 20 MiB")
    if maximum_strokes < 1 or maximum_points < 6:
        raise ValueError("image path limits are invalid")
    try:
        import cv2
        import numpy as np
    except ImportError as error:
        raise RuntimeError("image drawing requires python3-opencv and python3-numpy") from error

    encoded = np.fromfile(str(source), dtype=np.uint8)
    gray = cv2.imdecode(encoded, cv2.IMREAD_GRAYSCALE)
    if gray is None or gray.size == 0:
        raise ValueError(f"could not decode image: {source}")
    source_height, source_width = gray.shape[:2]
    scale = min(1.0, 640.0 / max(source_width, source_height))
    if scale < 1.0:
        gray = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    gray = cv2.GaussianBlur(gray, (5, 5), 0.0)
    median = float(np.median(gray))
    lower = max(20, int(0.66 * median))
    upper = max(lower + 20, min(255, int(1.33 * median)))
    edges = cv2.Canny(gray, lower, upper)
    contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

    candidates: list[tuple[float, list[Point2]]] = []
    for contour in contours:
        perimeter = float(cv2.arcLength(contour, True))
        if perimeter < float(minimum_contour_length_px):
            continue
        epsilon = max(float(simplify_epsilon_px), perimeter * 0.0025)
        simplified = cv2.approxPolyDP(contour, epsilon, True).reshape(-1, 2)
        if len(simplified) < 3:
            continue
        points = [(float(point[0]), float(point[1])) for point in simplified]
        points.append(points[0])
        candidates.append((perimeter, points))
    candidates.sort(key=lambda item: item[0], reverse=True)
    candidates = candidates[:maximum_strokes]
    if not candidates:
        raise ValueError("no usable line contours were found in the image")

    # Divide the global point budget by contour importance while reserving a valid
    # closed polygon for every selected contour.
    selected: list[list[Point2]] = []
    remaining = maximum_points
    for index, (_, points) in enumerate(candidates):
        remaining_contours = len(candidates) - index
        allowance = max(4, remaining // remaining_contours)
        sampled = _subsample_closed(points, allowance)
        if sampled[0] != sampled[-1]:
            sampled.append(sampled[0])
        selected.append(sampled)
        remaining -= len(sampled)
        if remaining < 4:
            break

    all_points = [point for stroke in selected for point in stroke]
    min_x = min(point[0] for point in all_points)
    max_x = max(point[0] for point in all_points)
    min_y = min(point[1] for point in all_points)
    max_y = max(point[1] for point in all_points)
    pixel_width = max(1.0, max_x - min_x)
    pixel_height = max(1.0, max_y - min_y)
    uniform_scale = min(width / pixel_width, height / pixel_height)
    center_x, center_y = (min_x + max_x) * 0.5, (min_y + max_y) * 0.5
    # Raster Y points down; drawing-plane Y points up.
    return tuple(
        tuple(((x - center_x) * uniform_scale, (center_y - y) * uniform_scale, 0.0)
              for x, y in stroke)
        for stroke in selected
    )


def flatten_strokes(strokes: Iterable[Sequence[Sequence[float]]]) -> tuple[Point3, ...]:
    """Return all valid 3-D stroke points, primarily for bounds and tests."""
    return tuple(tuple(float(value) for value in point) for stroke in strokes for point in stroke)
