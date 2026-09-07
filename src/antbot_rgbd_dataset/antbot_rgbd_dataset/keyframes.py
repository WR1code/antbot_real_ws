"""Real-robot keyframe selection independent of frame rate."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .transforms import transform_delta


@dataclass(frozen=True)
class KeyframeConfig:
    translation_threshold_m: float = 0.08
    rotation_threshold_deg: float = 7.0
    min_keyframe_interval_s: float = 0.1
    max_keyframe_interval_s: float = 1.0


@dataclass(frozen=True)
class KeyframeDecision:
    selected: bool
    reasons: tuple[str, ...]
    rejection: str | None
    translation_m: float
    rotation_deg: float


def decide_keyframe(
    config: KeyframeConfig,
    timestamp_ns: int,
    T_world_camera: np.ndarray,
    previous_timestamp_ns: int | None,
    T_world_previous_keyframe: np.ndarray | None,
) -> KeyframeDecision:
    if previous_timestamp_ns is None or T_world_previous_keyframe is None:
        return KeyframeDecision(True, ("FIRST_FRAME",), None, 0.0, 0.0)
    translation, rotation = transform_delta(T_world_previous_keyframe, T_world_camera)
    interval = (timestamp_ns - previous_timestamp_ns) / 1e9
    if interval <= 0:
        return KeyframeDecision(False, (), "NON_MONOTONIC_TIMESTAMP", translation, rotation)
    if interval < config.min_keyframe_interval_s:
        return KeyframeDecision(False, (), "MINIMUM_INTERVAL", translation, rotation)
    reasons = []
    if translation >= config.translation_threshold_m:
        reasons.append("TRANSLATION")
    if rotation >= config.rotation_threshold_deg:
        reasons.append("ROTATION")
    if interval >= config.max_keyframe_interval_s:
        reasons.append("MAX_INTERVAL")
    return KeyframeDecision(
        bool(reasons), tuple(reasons), None if reasons else "MOTION",
        translation, rotation
    )
