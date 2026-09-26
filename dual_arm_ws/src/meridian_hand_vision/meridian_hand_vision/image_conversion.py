from __future__ import annotations

from typing import Any

import cv2
import numpy as np


def _rows_without_padding(message: Any, bytes_per_pixel: int) -> np.ndarray:
    row_bytes = int(message.width) * bytes_per_pixel
    if int(message.step) < row_bytes:
        raise ValueError(
            f"无效 Image.step={message.step}，至少应为 {row_bytes}。"
        )

    raw = np.frombuffer(message.data, dtype=np.uint8)
    required_size = int(message.height) * int(message.step)
    if raw.size < required_size:
        raise ValueError(
            f"图像数据不完整：收到 {raw.size} 字节，需要 {required_size} 字节。"
        )
    rows = raw[:required_size].reshape(int(message.height), int(message.step))
    return np.ascontiguousarray(rows[:, :row_bytes])


def color_message_to_bgr(message: Any) -> np.ndarray:
    """Convert common 8-bit sensor_msgs/Image encodings without cv_bridge."""
    encoding = str(message.encoding).lower()
    conversions = {
        "rgb8": (3, cv2.COLOR_RGB2BGR),
        "bgr8": (3, None),
        "rgba8": (4, cv2.COLOR_RGBA2BGR),
        "bgra8": (4, cv2.COLOR_BGRA2BGR),
        "mono8": (1, cv2.COLOR_GRAY2BGR),
        "yuyv": (2, cv2.COLOR_YUV2BGR_YUY2),
        "yuy2": (2, cv2.COLOR_YUV2BGR_YUY2),
        "yuv422_yuy2": (2, cv2.COLOR_YUV2BGR_YUY2),
    }
    if encoding not in conversions:
        raise ValueError(
            f"暂不支持彩色图编码 {message.encoding!r}；"
            "支持 rgb8/bgr8/rgba8/bgra8/mono8/YUYV。"
        )

    channels, conversion = conversions[encoding]
    rows = _rows_without_padding(message, channels)
    if channels == 1:
        pixels = rows.reshape(int(message.height), int(message.width))
    else:
        pixels = rows.reshape(int(message.height), int(message.width), channels)
    if conversion is None:
        return np.ascontiguousarray(pixels)
    return cv2.cvtColor(np.ascontiguousarray(pixels), conversion)


def depth_message_to_meters(message: Any) -> np.ndarray:
    """Convert 16UC1 millimetres or 32FC1 metres to float32 metres."""
    encoding = str(message.encoding).lower()
    big_endian = bool(message.is_bigendian)
    if encoding in {"16uc1", "mono16"}:
        rows = _rows_without_padding(message, 2)
        dtype = np.dtype(">u2" if big_endian else "<u2")
        depth = rows.reshape(-1).view(dtype).reshape(
            int(message.height), int(message.width)
        )
        return depth.astype(np.float32) / 1000.0
    if encoding == "32fc1":
        rows = _rows_without_padding(message, 4)
        dtype = np.dtype(">f4" if big_endian else "<f4")
        depth = rows.reshape(-1).view(dtype).reshape(
            int(message.height), int(message.width)
        )
        return depth.astype(np.float32)
    raise ValueError(
        f"暂不支持深度图编码 {message.encoding!r}；支持 16UC1/mono16/32FC1。"
    )


def colorize_depth(depth_m: np.ndarray, maximum_m: float = 3.0) -> np.ndarray:
    valid = np.isfinite(depth_m) & (depth_m > 0.0)
    normalized = np.zeros(depth_m.shape, dtype=np.uint8)
    if np.any(valid):
        clipped = np.clip(depth_m, 0.0, maximum_m)
        normalized[valid] = np.uint8(
            np.clip((1.0 - clipped[valid] / maximum_m) * 255.0, 0.0, 255.0)
        )
    colored = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)
    colored[~valid] = 0
    return colored
