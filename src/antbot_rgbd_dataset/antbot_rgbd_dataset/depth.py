"""Color/depth normalization for real sensors. No ROS imports."""

from __future__ import annotations

import numpy as np


def color_to_rgb(color: np.ndarray, encoding: str) -> np.ndarray:
    value = np.asarray(color)
    encoding = encoding.lower()
    if encoding == "rgb8":
        result = value
    elif encoding == "bgr8":
        result = value[..., ::-1]
    elif encoding == "rgba8":
        result = value[..., :3]
    elif encoding == "bgra8":
        result = value[..., [2, 1, 0]]
    elif encoding in ("mono8", "8uc1"):
        result = np.repeat(value[..., None], 3, axis=2)
    else:
        raise ValueError(f"unsupported color encoding: {encoding}")
    return np.ascontiguousarray(result, dtype=np.uint8)


def depth_to_meters(
    depth: np.ndarray,
    encoding: str,
    encoding_mode: str = "auto",
    scale_override: float | None = None,
) -> np.ndarray:
    """Convert a sensor depth image to float32 meters without range clipping."""
    value = np.asarray(depth)
    normalized = encoding.upper()
    if scale_override is not None:
        scale = float(scale_override)
        if not np.isfinite(scale) or scale <= 0:
            raise ValueError("depth_scale_override must be positive and finite")
    elif encoding_mode == "auto":
        if normalized in ("16UC1", "MONO16"):
            scale = 0.001
        elif normalized == "32FC1":
            scale = 1.0
        else:
            raise ValueError(
                f"cannot infer depth scale for encoding {encoding!r}; set depth_scale_override"
            )
    elif encoding_mode == "millimeter":
        scale = 0.001
    elif encoding_mode == "meter":
        scale = 1.0
    else:
        raise ValueError(f"invalid depth_encoding_mode: {encoding_mode!r}")
    return np.ascontiguousarray(value.astype(np.float32) * np.float32(scale))


def filter_depth(
    depth_m: np.ndarray, minimum_m: float, maximum_m: float
) -> tuple[np.ndarray, dict]:
    if depth_m.dtype != np.float32:
        raise ValueError("depth must first be converted to float32 meters")
    if not 0 < minimum_m < maximum_m:
        raise ValueError("depth range must satisfy 0 < minimum < maximum")
    output = depth_m.copy()
    valid = np.isfinite(output) & (output >= minimum_m) & (output <= maximum_m)
    values = output[valid]
    statistics = {
        "valid_pixels": int(valid.sum()),
        "valid_ratio": float(valid.mean()) if valid.size else 0.0,
        "median_m": float(np.median(values)) if values.size else None,
        "p90_m": float(np.percentile(values, 90)) if values.size else None,
        "p95_m": float(np.percentile(values, 95)) if values.size else None,
        "p99_m": float(np.percentile(values, 99)) if values.size else None,
        "zero_count": int((output == 0).sum()),
        "negative_count": int((output < 0).sum()),
        "nan_count": int(np.isnan(output).sum()),
        "inf_count": int(np.isinf(output).sum()),
    }
    output[~valid] = 0
    return output, statistics

