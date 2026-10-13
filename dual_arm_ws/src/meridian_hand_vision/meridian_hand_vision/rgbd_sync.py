"""Timestamp-only RGB-D pairing checks, independent of camera/ML runtimes."""

from __future__ import annotations


def validate_rgb_depth_stamps(
    rgb_stamp_ns: int,
    depth_stamp_ns: int,
    maximum_delta_ns: int,
) -> tuple[bool, float | None, str | None, str]:
    """Return validity, absolute delta, rejection reason and stale input.

    ``stale_input`` is ``rgb``, ``depth``, ``both`` or an empty string.
    """
    if rgb_stamp_ns <= 0 or depth_stamp_ns <= 0:
        return False, None, "MISSING_SENSOR_TIMESTAMP", "both"
    signed_delta = rgb_stamp_ns - depth_stamp_ns
    delta_ms = abs(signed_delta) / 1_000_000.0
    if abs(signed_delta) > maximum_delta_ns:
        stale = "rgb" if signed_delta < 0 else "depth"
        return False, delta_ms, "RGB_DEPTH_TIME_MISMATCH", stale
    return True, delta_ms, None, ""
