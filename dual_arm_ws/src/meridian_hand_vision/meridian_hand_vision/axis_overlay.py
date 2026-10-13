from __future__ import annotations

import cv2
import numpy as np

from .pulse_region import CameraIntrinsics


FOREARM_DIRECTION_OVERLAY_SEMANTICS = "ROBOT_STABLE_CANDIDATE_RAW_V2"


def unsigned_axis_angle_deg(first, second) -> float | None:
    """Return an unoriented 3D axis angle for overlay diagnostics."""
    if first is None or second is None:
        return None
    first_vector = np.asarray(first, dtype=np.float64)
    second_vector = np.asarray(second, dtype=np.float64)
    first_norm = float(np.linalg.norm(first_vector))
    second_norm = float(np.linalg.norm(second_vector))
    if (
        first_vector.shape != (3,)
        or second_vector.shape != (3,)
        or not np.all(np.isfinite(first_vector))
        or not np.all(np.isfinite(second_vector))
        or first_norm <= 1e-12
        or second_norm <= 1e-12
    ):
        return None
    cosine = abs(float(np.dot(first_vector, second_vector))) / (
        first_norm * second_norm
    )
    return float(np.degrees(np.arccos(np.clip(cosine, 0.0, 1.0))))


def project_camera_point_to_image(
    point_m,
    intrinsics: CameraIntrinsics,
    image_shape: tuple[int, ...],
    *,
    mirrored: bool = False,
) -> tuple[int, int] | None:
    """Project a camera-frame point into the displayed RGB image."""
    point = np.asarray(point_m, dtype=np.float64)
    if point.shape != (3,) or not np.all(np.isfinite(point)) or point[2] <= 1e-6:
        return None
    height, width = image_shape[:2]
    if height <= 0 or width <= 0:
        return None
    calibration_u = intrinsics.fx * point[0] / point[2] + intrinsics.cx
    calibration_v = intrinsics.fy * point[1] / point[2] + intrinsics.cy
    scale_x = intrinsics.width / float(width)
    scale_y = intrinsics.height / float(height)
    image_u = (calibration_u + 0.5) / scale_x - 0.5
    image_v = (calibration_v + 0.5) / scale_y - 0.5
    if mirrored:
        image_u = width - 1.0 - image_u
    if not np.isfinite(image_u + image_v):
        return None
    return int(round(image_u)), int(round(image_v))


def _valid_axis(axis):
    if axis is None:
        return None
    vector = np.asarray(axis, dtype=np.float64)
    if (
        vector.shape != (3,)
        or not np.all(np.isfinite(vector))
        or float(np.linalg.norm(vector)) <= 1e-12
    ):
        return None
    return vector


def select_forearm_direction_overlay(
    stable_direction_axis,
):
    """Select only the filtered direction actually exposed to robot control.

    Raw V2.2, V2.1, and V1.1 candidate axes are diagnostic inputs and must
    never become the FOREARM_DIRECTION arrow.
    """
    stable_vector = _valid_axis(stable_direction_axis)
    if stable_vector is not None:
        return stable_vector, "STABLE_DIRECTION", "OK"
    return None, "NO_STABLE_DIRECTION", "FILTER_NOT_STABLE"


def select_vision_forearm_axis(v2_2_track_axis, v2_1_axis):
    """Select a geometry-only axis for visualization.

    This display helper intentionally has no MediaPipe or selector fallback
    input, so the green arrow cannot silently become a palm direction.
    """
    v2_2_vector = _valid_axis(v2_2_track_axis)
    if v2_2_vector is not None:
        return v2_2_vector, "V2.2_TRACK"
    v2_1_vector = _valid_axis(v2_1_axis)
    if v2_1_vector is not None:
        return v2_1_vector, "V2.1_AXIS"
    return None, "NONE"


def draw_forearm_direction_overlay(
    panel: np.ndarray,
    wrist_point_m,
    intrinsics: CameraIntrinsics | None,
    axis,
    source: str,
    reason: str,
    *,
    mirrored: bool = False,
    arrow_length_m: float = 0.120,
    diagnostics: dict | None = None,
    robot_direction_available: bool | None = None,
    filter_state: str | None = None,
    active_source: str | None = None,
    diagnostic_candidate_axes: dict | None = None,
) -> bool:
    """Draw only the filtered robot direction as a solid blue arrow."""
    details = diagnostics if diagnostics is not None else {}
    details.clear()
    details.update({
        "draw_call_executed": True,
        "start_pixel": None,
        "end_pixel": None,
        "projection_valid": False,
        "points_inside_image": False,
        "projection_warning": None,
        "draw_success": False,
        "draw_failure_reason": None,
        "arrow_length_m": float(arrow_length_m),
        "line_width_px": 5,
        "color_bgr": [255, 0, 0],
    })
    cv2.putText(
        panel,
        f"ROBOT FOREARM DIRECTION: {filter_state or source}",
        (12, 246),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.52,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        panel,
        f"ROBOT: stable_direction  source: {source}",
        (12, 270),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.50,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    if robot_direction_available is not None:
        cv2.putText(
            panel,
            "robot_direction_available: "
            f"{str(bool(robot_direction_available)).lower()}",
            (12, 294),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.46,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
    if filter_state is not None:
        cv2.putText(
            panel,
            f"filter_state: {filter_state}  reason: {reason}",
            (12, 316),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.46,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
    if active_source is not None:
        cv2.putText(
            panel,
            f"active_source: {active_source}",
            (12, 338),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.46,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
    if diagnostic_candidate_axes is not None:
        candidate_text = " ".join(
            f"{name}={'Y' if axis is not None else 'N'}"
            for name, axis in diagnostic_candidate_axes.items()
        )
        cv2.putText(
            panel,
            f"diagnostic_candidate_axes: {candidate_text}",
            (12, 360),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (180, 180, 180),
            1,
            cv2.LINE_AA,
        )

    vector = _valid_axis(axis)
    origin = (
        None if wrist_point_m is None
        else np.asarray(wrist_point_m, dtype=np.float64)
    )
    if vector is None:
        raw_axis = None if axis is None else np.asarray(axis)
        details["draw_failure_reason"] = (
            "NO_AXIS" if axis is None
            else "NAN_AXIS" if np.any(~np.isfinite(raw_axis))
            else "ZERO_OR_INVALID_AXIS"
        )
        return False
    if origin is None:
        details["draw_failure_reason"] = "MISSING_WRIST_POINT"
        return False
    if origin.shape != (3,):
        details["draw_failure_reason"] = "INVALID_WRIST_POINT"
        return False
    if not np.all(np.isfinite(origin)):
        details["draw_failure_reason"] = "NAN_WRIST_POINT"
        return False
    if origin[2] <= 1e-6:
        details["draw_failure_reason"] = "INVALID_DEPTH"
        return False
    if intrinsics is None:
        details["draw_failure_reason"] = "INVALID_CAMERA_INFO"
        return False
    vector /= float(np.linalg.norm(vector))
    start = project_camera_point_to_image(
        origin, intrinsics, panel.shape, mirrored=mirrored
    )
    end = project_camera_point_to_image(
        origin + float(arrow_length_m) * vector,
        intrinsics,
        panel.shape,
        mirrored=mirrored,
    )
    details["start_pixel"] = start
    details["end_pixel"] = end
    if start is None or end is None:
        details["draw_failure_reason"] = "PROJECTION_FAILED"
        return False
    details["projection_valid"] = True
    height, width = panel.shape[:2]
    start_inside = 0 <= start[0] < width and 0 <= start[1] < height
    end_inside = 0 <= end[0] < width and 0 <= end[1] < height
    details["points_inside_image"] = bool(start_inside and end_inside)
    if not start_inside or not end_inside:
        visible, clipped_start, clipped_end = cv2.clipLine(
            (0, 0, width, height), start, end
        )
        details["projection_warning"] = "POINT_OUTSIDE_IMAGE"
        details["clipped_start_pixel"] = clipped_start if visible else None
        details["clipped_end_pixel"] = clipped_end if visible else None
        if not visible:
            details["draw_failure_reason"] = "POINT_OUTSIDE_IMAGE"
            return False
        start, end = clipped_start, clipped_end
    cv2.arrowedLine(
        panel, start, end, (255, 0, 0), 5, cv2.LINE_AA, tipLength=0.16
    )
    cv2.circle(panel, start, 5, (255, 0, 0), -1, cv2.LINE_AA)
    details["draw_success"] = True
    details["draw_failure_reason"] = "NONE"
    return True


def draw_candidate_forearm_axis_overlay(
    panel: np.ndarray,
    wrist_point_m,
    intrinsics: CameraIntrinsics | None,
    axis,
    source: str,
    *,
    mirrored: bool = False,
    arrow_length_m: float = 0.120,
    diagnostics: dict | None = None,
) -> bool:
    """Draw the unfiltered selector estimate as a dashed yellow arrow."""
    details = diagnostics if diagnostics is not None else {}
    details.clear()
    details.update({
        "draw_call_executed": True,
        "start_pixel": None,
        "end_pixel": None,
        "projection_valid": False,
        "draw_success": False,
        "draw_failure_reason": None,
        "arrow_length_m": float(arrow_length_m),
        "line_width_px": 3,
        "color_bgr": [0, 255, 255],
        "line_style": "DASHED",
    })
    candidate_available = _valid_axis(axis) is not None
    displayed_source = source if candidate_available else "NO_INPUT"
    cv2.putText(
        panel,
        f"CANDIDATE FOREARM AXIS: {displayed_source}",
        (12, 384),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.50,
        (0, 255, 255) if candidate_available else (150, 150, 150),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        panel,
        "CANDIDATE: raw estimate",
        (12, 408),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.46,
        (0, 255, 255) if candidate_available else (150, 150, 150),
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        panel,
        f"active_source: {source}",
        (12, 430),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.46,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )

    vector = _valid_axis(axis)
    origin = (
        None if wrist_point_m is None
        else np.asarray(wrist_point_m, dtype=np.float64)
    )
    if vector is None:
        details["draw_failure_reason"] = "NO_AXIS"
        return False
    if origin is None:
        details["draw_failure_reason"] = "MISSING_WRIST_POINT"
        return False
    if origin.shape != (3,) or not np.all(np.isfinite(origin)):
        details["draw_failure_reason"] = "INVALID_WRIST_POINT"
        return False
    if origin[2] <= 1e-6:
        details["draw_failure_reason"] = "INVALID_DEPTH"
        return False
    if intrinsics is None:
        details["draw_failure_reason"] = "INVALID_CAMERA_INFO"
        return False

    vector /= float(np.linalg.norm(vector))
    start = project_camera_point_to_image(
        origin, intrinsics, panel.shape, mirrored=mirrored
    )
    end = project_camera_point_to_image(
        origin + float(arrow_length_m) * vector,
        intrinsics,
        panel.shape,
        mirrored=mirrored,
    )
    details["start_pixel"] = start
    details["end_pixel"] = end
    if start is None or end is None:
        details["draw_failure_reason"] = "PROJECTION_FAILED"
        return False
    height, width = panel.shape[:2]
    visible, start, end = cv2.clipLine((0, 0, width, height), start, end)
    if not visible:
        details["draw_failure_reason"] = "POINT_OUTSIDE_IMAGE"
        return False
    details["projection_valid"] = True

    start_f = np.asarray(start, dtype=np.float64)
    end_f = np.asarray(end, dtype=np.float64)
    delta = end_f - start_f
    pixel_length = float(np.linalg.norm(delta))
    if pixel_length < 2.0:
        details["draw_failure_reason"] = "ZERO_LENGTH_PROJECTION"
        return False
    direction = delta / pixel_length
    dash_px = 12.0
    gap_px = 7.0
    cursor = 0.0
    while cursor < pixel_length:
        segment_end = min(cursor + dash_px, pixel_length)
        dash_start = tuple(np.rint(start_f + cursor * direction).astype(int))
        dash_end = tuple(np.rint(start_f + segment_end * direction).astype(int))
        cv2.line(
            panel, dash_start, dash_end, (0, 255, 255), 3, cv2.LINE_AA
        )
        cursor += dash_px + gap_px
    arrow_base = tuple(
        np.rint(end_f - min(18.0, pixel_length * 0.25) * direction).astype(int)
    )
    cv2.arrowedLine(
        panel, arrow_base, end, (0, 255, 255), 3, cv2.LINE_AA,
        tipLength=0.55,
    )
    cv2.circle(panel, start, 4, (0, 255, 255), -1, cv2.LINE_AA)
    details["draw_success"] = True
    details["draw_failure_reason"] = "NONE"
    return True


def _draw_diagnostic_axis(
    panel: np.ndarray,
    wrist_point_m,
    intrinsics: CameraIntrinsics | None,
    axis,
    *,
    color: tuple[int, int, int],
    arrow_length_m: float,
    origin_offset_px: tuple[int, int],
    dashed: bool,
    mirrored: bool,
) -> bool:
    vector = _valid_axis(axis)
    if vector is None or wrist_point_m is None or intrinsics is None:
        return False
    origin = np.asarray(wrist_point_m, dtype=np.float64)
    if (
        origin.shape != (3,)
        or not np.all(np.isfinite(origin))
        or origin[2] <= 1e-6
    ):
        return False
    vector = vector / float(np.linalg.norm(vector))
    start = project_camera_point_to_image(
        origin, intrinsics, panel.shape, mirrored=mirrored
    )
    end = project_camera_point_to_image(
        origin + arrow_length_m * vector,
        intrinsics,
        panel.shape,
        mirrored=mirrored,
    )
    if start is None or end is None:
        return False
    offset_x, offset_y = origin_offset_px
    start = (start[0] + offset_x, start[1] + offset_y)
    end = (end[0] + offset_x, end[1] + offset_y)
    height, width = panel.shape[:2]
    visible, start, end = cv2.clipLine((0, 0, width, height), start, end)
    if not visible:
        return False
    if not dashed:
        cv2.arrowedLine(
            panel, start, end, color, 4, cv2.LINE_AA, tipLength=0.17
        )
        cv2.circle(panel, start, 4, color, -1, cv2.LINE_AA)
        return True

    start_f = np.asarray(start, dtype=np.float64)
    end_f = np.asarray(end, dtype=np.float64)
    delta = end_f - start_f
    pixel_length = float(np.linalg.norm(delta))
    if pixel_length < 2.0:
        return False
    direction = delta / pixel_length
    cursor = 0.0
    while cursor < pixel_length:
        segment_end = min(cursor + 10.0, pixel_length)
        dash_start = tuple(np.rint(start_f + cursor * direction).astype(int))
        dash_end = tuple(
            np.rint(start_f + segment_end * direction).astype(int)
        )
        cv2.line(panel, dash_start, dash_end, color, 3, cv2.LINE_AA)
        cursor += 16.0
    arrow_base = tuple(
        np.rint(end_f - min(17.0, pixel_length * 0.25) * direction).astype(int)
    )
    cv2.arrowedLine(
        panel, arrow_base, end, color, 3, cv2.LINE_AA, tipLength=0.55
    )
    cv2.circle(panel, start, 3, color, -1, cv2.LINE_AA)
    return True


def _short_b_plus_source(source: str | None) -> str:
    if source == "PRIMARY_3D_FOREARM":
        return "PRIMARY"
    if source == "HOLD_LAST_PRIMARY_DIRECTION":
        return "HOLD"
    return "NONE"


def draw_forearm_diagnostic_overlay(
    panel: np.ndarray,
    wrist_point_m,
    intrinsics: CameraIntrinsics | None,
    *,
    robot_axis,
    vision_axis,
    vision_source: str,
    selector_axis,
    selector_source: str,
    b_plus_variants: dict,
    mirrored: bool = False,
) -> dict:
    """Draw four independent direction sources without feeding estimation."""
    robot_drawn = _draw_diagnostic_axis(
        panel, wrist_point_m, intrinsics, robot_axis,
        color=(255, 0, 0), arrow_length_m=0.120,
        origin_offset_px=(-7, -7), dashed=False, mirrored=mirrored,
    )
    vision_drawn = _draw_diagnostic_axis(
        panel, wrist_point_m, intrinsics, vision_axis,
        color=(0, 255, 0), arrow_length_m=0.140,
        origin_offset_px=(7, -7), dashed=False, mirrored=mirrored,
    )
    selector_drawn = _draw_diagnostic_axis(
        panel, wrist_point_m, intrinsics, selector_axis,
        color=(0, 255, 255), arrow_length_m=0.105,
        origin_offset_px=(-7, 7), dashed=True, mirrored=mirrored,
    )

    variant_order = ("hold_0_5s", "hold_0_2s", "hold_1_0s")
    b_plus_drawn = False
    drawn_axes: list[np.ndarray] = []
    variant_offsets = {
        "hold_0_2s": (-10, 10),
        "hold_0_5s": (0, 11),
        "hold_1_0s": (10, 12),
    }
    variant_lengths = {
        "hold_0_2s": 0.145,
        "hold_0_5s": 0.160,
        "hold_1_0s": 0.175,
    }
    for name in variant_order:
        variant = b_plus_variants.get(name) or {}
        vector = _valid_axis(variant.get("current_direction"))
        if vector is None:
            continue
        normalized = vector / float(np.linalg.norm(vector))
        if any(
            unsigned_axis_angle_deg(normalized, previous) is not None
            and unsigned_axis_angle_deg(normalized, previous) <= 0.5
            for previous in drawn_axes
        ):
            continue
        drawn_axes.append(normalized)
        b_plus_drawn = _draw_diagnostic_axis(
            panel, wrist_point_m, intrinsics, normalized,
            color=(255, 0, 255),
            arrow_length_m=variant_lengths[name],
            origin_offset_px=variant_offsets[name],
            dashed=True,
            mirrored=mirrored,
        ) or b_plus_drawn

    robot_status = (
        "STABLE_DIRECTION" if _valid_axis(robot_axis) is not None
        else "NO_STABLE_DIRECTION"
    )
    vision_status = (
        vision_source if _valid_axis(vision_axis) is not None else "NONE"
    )
    selector_status = (
        selector_source if _valid_axis(selector_axis) is not None else "INVALID"
    )
    b02 = _short_b_plus_source(
        (b_plus_variants.get("hold_0_2s") or {}).get("current_source")
    )
    b05 = _short_b_plus_source(
        (b_plus_variants.get("hold_0_5s") or {}).get("current_source")
    )
    b10 = _short_b_plus_source(
        (b_plus_variants.get("hold_1_0s") or {}).get("current_source")
    )

    top = max(0, min(max(0, panel.shape[0] - 107), 310))
    right = min(panel.shape[1] - 5, 635)
    overlay = panel.copy()
    cv2.rectangle(overlay, (5, top), (right, top + 106), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.58, panel, 0.42, 0.0, panel)
    text_rows = (
        (f"ROBOT_DIRECTION  ROBOT: {robot_status}", (255, 80, 80)),
        (f"VISION_FOREARM_AXIS  VISION: {vision_status}", (80, 255, 80)),
        (f"SELECTOR_DIRECTION  SOURCE: {selector_status}", (0, 255, 255)),
        (
            f"B_PLUS_SHADOW_DIRECTION  B+0.2:{b02}  B+0.5:{b05}  B+1.0:{b10}",
            (255, 80, 255),
        ),
    )
    for row, (text, color) in enumerate(text_rows):
        cv2.putText(
            panel, text, (12, top + 22 + row * 25),
            cv2.FONT_HERSHEY_SIMPLEX, 0.44, color, 1, cv2.LINE_AA,
        )
    return {
        "robot_drawn": robot_drawn,
        "vision_drawn": vision_drawn,
        "selector_drawn": selector_drawn,
        "b_plus_drawn": b_plus_drawn,
        "robot_status": robot_status,
        "vision_source": vision_status,
        "selector_source": selector_status,
        "b_plus_status": {
            "hold_0_2s": b02,
            "hold_0_5s": b05,
            "hold_1_0s": b10,
        },
    }


def draw_debug_force_forearm_arrow(
    panel: np.ndarray,
    *,
    diagnostics: dict | None = None,
) -> bool:
    """Draw a fixed 100 px blue arrow for explicit display debugging only."""
    height, width = panel.shape[:2]
    start = (width // 2, height // 2)
    end = (min(width - 1, start[0] + 100), start[1])
    success = width > 1 and height > 0 and end != start
    if diagnostics is not None:
        diagnostics.clear()
        diagnostics.update({
            "enabled": True,
            "draw_start_pixel": start,
            "draw_end_pixel": end,
            "line_width_px": 6,
            "alpha": 1.0,
            "color_bgr": [255, 0, 0],
            "draw_success": success,
            "draw_failure_reason": "NONE" if success else "IMAGE_TOO_SMALL",
        })
    if not success:
        return False
    cv2.arrowedLine(
        panel, start, end, (255, 0, 0), 6, cv2.LINE_AA, tipLength=0.18
    )
    cv2.putText(
        panel,
        "DEBUG FORCE FOREARM ARROW",
        (max(0, start[0] - 105), max(20, start[1] - 14)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.52,
        (255, 0, 0),
        2,
        cv2.LINE_AA,
    )
    return True


def draw_arm_axis_overlay(
    panel: np.ndarray,
    wrist_point_m,
    intrinsics: CameraIntrinsics | None,
    axes: tuple[dict, ...],
    angles: dict[str, float | None],
    *,
    mirrored: bool = False,
) -> None:
    """Draw diagnostic axes only; this function cannot affect estimation."""
    origin_3d = (
        np.asarray(wrist_point_m, dtype=np.float64)
        if wrist_point_m is not None else None
    )
    origin_pixel = None
    if origin_3d is not None and intrinsics is not None:
        origin_pixel = project_camera_point_to_image(
            origin_3d, intrinsics, panel.shape, mirrored=mirrored
        )
    for row, axis_display in enumerate(axes):
        label = str(axis_display["label"])
        color = tuple(axis_display["color"])
        vector = axis_display.get("vector")
        arrow_length_m = float(axis_display["length_m"])
        offset_x, offset_y = axis_display.get("origin_offset_px", (0, 0))
        status = str(axis_display.get("status") or "N/A")
        detail = str(axis_display.get("detail") or "")
        vector_array = None if vector is None else np.asarray(vector, dtype=np.float64)
        valid = (
            vector_array is not None
            and vector_array.shape == (3,)
            and np.all(np.isfinite(vector_array))
            and float(np.linalg.norm(vector_array)) > 1e-12
        )
        cv2.putText(
            panel,
            f"{label}: {status}{('  ' + detail) if detail else ''}",
            (12, 92 + 24 * row),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            color if valid else (150, 150, 150),
            2,
            cv2.LINE_AA,
        )
        if not valid or origin_pixel is None or origin_3d is None or intrinsics is None:
            continue
        vector_array /= float(np.linalg.norm(vector_array))
        endpoint = project_camera_point_to_image(
            origin_3d + arrow_length_m * vector_array,
            intrinsics,
            panel.shape,
            mirrored=mirrored,
        )
        if endpoint is None:
            continue
        offset_origin = (
            int(origin_pixel[0] + offset_x), int(origin_pixel[1] + offset_y)
        )
        offset_endpoint = (
            int(endpoint[0] + offset_x), int(endpoint[1] + offset_y)
        )
        cv2.arrowedLine(
            panel, offset_origin, offset_endpoint, color, 4, cv2.LINE_AA,
            tipLength=0.16,
        )
        cv2.circle(panel, offset_origin, 4, color, -1, cv2.LINE_AA)
        cv2.circle(panel, offset_endpoint, 5, color, -1, cv2.LINE_AA)

    angle_labels = (
        ("MP-V2.1", angles.get("mediapipe_v2_1")),
        ("MP-V2.2", angles.get("mediapipe_v2_2")),
        ("V2.1-V2.2", angles.get("v2_1_v2_2")),
    )
    for row, (label, angle) in enumerate(angle_labels):
        value = f"{angle:.1f} deg" if angle is not None else "N/A"
        cv2.putText(
            panel,
            f"{label}: {value}",
            (12, 172 + 22 * row),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
