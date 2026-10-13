"""Read-only root-cause diagnostics for invalid wrist-geometry inputs."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import math

import numpy as np


@dataclass
class InvalidInputShadowState:
    frame_index: int = 0
    history: deque = field(default_factory=lambda: deque(maxlen=11))
    previous_active_track_id: int | None = None
    previous_active_state: str = "NO_ACTIVE_TRACK"


def _finite_wrist(landmarks) -> bool:
    if landmarks is None:
        return False
    try:
        wrist = landmarks[0]
        return math.isfinite(float(wrist.x)) and math.isfinite(float(wrist.y))
    except (IndexError, TypeError, AttributeError, ValueError):
        return False


def _depth_roi(depth_m: np.ndarray, pixel, radius_px: int = 18):
    if depth_m.ndim != 2 or pixel is None:
        return 0, 0.0, True
    column, row = int(round(pixel[0])), int(round(pixel[1]))
    height, width = depth_m.shape
    x0, x1 = max(0, column - radius_px), min(width, column + radius_px + 1)
    y0, y1 = max(0, row - radius_px), min(height, row + radius_px + 1)
    roi = depth_m[y0:y1, x0:x1]
    if roi.size == 0:
        return 0, 0.0, True
    valid = np.isfinite(roi) & (roi >= 0.10) & (roi <= 2.0)
    count = int(np.count_nonzero(valid))
    return count, float(count / roi.size), count == 0


def _classification(
    invalid: bool,
    hand_detected: bool,
    landmark_valid: bool,
    raw_wrist_available: bool,
    depth_seed_valid: bool,
    mask_area: int,
    connected_points: int,
    branch_count: int,
    candidate_count: int,
    estimator_reason: str,
) -> tuple[str, str, float]:
    if not invalid:
        return "NONE", "NONE", 0.0
    if not hand_detected:
        return "WRIST_NO_DETECTION", "HAND", 0.99
    if not landmark_valid or not raw_wrist_available:
        return "WRIST_INVALID", "WRIST", 0.95
    if not depth_seed_valid:
        return "DEPTH_SEED_FAILURE", "DEPTH_SEED", 0.92
    if mask_area == 0:
        return "MASK_EMPTY", "MASK", 0.92
    if connected_points == 0 or estimator_reason == "CONNECTED_CLOUD_TOO_SMALL":
        return "CONNECTED_CLOUD_EMPTY", "CONNECTED_CLOUD", 0.88
    if branch_count == 0 or candidate_count == 0:
        return "BRANCH_NO_CANDIDATE", "BRANCH", 0.85
    return "UNKNOWN_INVALID", "UNKNOWN", 0.35


def _stage_sequence(classification: str) -> list[str]:
    stages = {
        "WRIST_NO_DETECTION": ["Hand missing", "Wrist unavailable", "Mask unavailable", "Branch missing"],
        "WRIST_INVALID": ["Hand detected", "Wrist invalid", "Mask unavailable", "Branch missing"],
        "DEPTH_SEED_FAILURE": ["Hand OK", "Wrist OK", "Depth seed failed", "Mask unavailable", "Branch missing"],
        "MASK_EMPTY": ["Hand OK", "Wrist OK", "Depth seed OK", "Mask empty", "Branch missing"],
        "CONNECTED_CLOUD_EMPTY": ["Hand OK", "Wrist OK", "Depth seed OK", "Mask present", "Connected cloud empty", "Branch missing"],
        "BRANCH_NO_CANDIDATE": ["Hand OK", "Wrist OK", "Depth seed OK", "Mask OK", "Cloud present", "Branch missing"],
        "UNKNOWN_INVALID": ["Input invalid", "Cause unknown"],
        "NONE": ["Input chain OK"],
    }
    return stages[classification]


def _temporal_attribution(state, active_track_id, active_state, invalid):
    event = "NONE"
    if (
        active_state == "ACTIVE_MISSING"
        and state.previous_active_state != "ACTIVE_MISSING"
    ):
        event = "ACTIVE_MISSING_START"
    elif state.previous_active_track_id is not None and active_track_id is None:
        event = "TRACK_EXPIRED"

    abnormal = [item for item in state.history if item["classification"] != "NONE"]
    first = abnormal[0] if abnormal else None
    current = state.history[-1]
    result = {
        "track_failure_event": event,
        "window_start_frame": state.history[0]["frame"] if state.history else None,
        "window_end_frame": current["frame"],
        "first_failure_stage": first["stage"] if first is not None else "NONE",
        "first_failure_frame": first["frame"] if first is not None else None,
        "failure_sequence": _stage_sequence(current["classification"]),
        "failure_history": [
            {
                "frame": item["frame"],
                "classification": item["classification"],
                "stage": item["stage"],
            }
            for item in abnormal
        ],
        "confidence": current["confidence"] if invalid else 0.0,
    }
    state.previous_active_track_id = active_track_id
    state.previous_active_state = active_state
    return result


def update_invalid_input_shadow(
    state: InvalidInputShadowState,
    hand_landmarks,
    hand_confidence,
    depth_m: np.ndarray,
    mask_estimate,
    axis_estimate,
    tracking_estimate,
    wrist_input_diagnostic: dict,
    branch_candidate_diagnostic: dict,
    *,
    frame_index: int | None = None,
) -> dict:
    """Classify invalid inputs using already-computed pipeline diagnostics."""
    state.frame_index = int(frame_index) if frame_index is not None else state.frame_index + 1
    hands = tuple(hand_landmarks or ())
    detected_hand_count = len(hands)
    hand_detected = detected_hand_count > 0
    landmark_count = sum(len(hand) for hand in hands)
    single_hand = hands[0] if detected_hand_count == 1 else None
    landmark_valid = detected_hand_count == 1 and _finite_wrist(single_hand)

    wrist = wrist_input_diagnostic.get("wrist_temporal", {})
    raw_wrist_available = bool(wrist.get("raw_wrist_available"))
    raw_wrist_pixel = wrist.get("raw_wrist_pixel")
    depth_seed_valid = bool(wrist.get("depth_seed_valid"))
    mask_reason = str(getattr(mask_estimate, "reason", "UNKNOWN"))
    depth_seed_reason = "NONE" if depth_seed_valid else (
        mask_reason if mask_reason != "NONE" else wrist.get("raw_wrist_reason", "UNKNOWN")
    )

    roi_valid_count, roi_valid_ratio, roi_empty = _depth_roi(
        np.asarray(depth_m), raw_wrist_pixel
    )
    mask_area = int(getattr(mask_estimate, "mask_area_px", 0))
    connected_points = int(getattr(mask_estimate, "connected_point_count", 0))
    branches = tuple(getattr(axis_estimate, "branches", ()) or ())
    branch_count = len(branches)
    candidate_count = int(branch_candidate_diagnostic.get("candidate_count", branch_count))
    estimator_reason = str(getattr(axis_estimate, "reason", "UNKNOWN"))
    invalid = estimator_reason != "NONE"
    classification, stage, confidence = _classification(
        invalid,
        hand_detected,
        landmark_valid,
        raw_wrist_available,
        depth_seed_valid,
        mask_area,
        connected_points,
        branch_count,
        candidate_count,
        estimator_reason,
    )
    state.history.append({
        "frame": state.frame_index,
        "classification": classification,
        "stage": stage,
        "confidence": confidence,
    })
    active_track_id = getattr(tracking_estimate, "active_track_id", None)
    active_state = str(getattr(tracking_estimate, "active_state", "NO_ACTIVE_TRACK"))
    temporal = _temporal_attribution(
        state, active_track_id, active_state, invalid
    )
    return {
        "invalid": invalid,
        "estimator_reason": estimator_reason,
        "classification": classification,
        "classification_confidence": confidence,
        "hand_detection": {
            "hand_detected": hand_detected,
            "detected_hand_count": detected_hand_count,
            "landmark_valid": landmark_valid,
            "landmark_count": landmark_count,
            "hand_confidence": (
                float(hand_confidence) if hand_confidence is not None else None
            ),
        },
        "wrist": {
            "raw_wrist_available": raw_wrist_available,
            "raw_wrist_pixel": raw_wrist_pixel,
            "raw_wrist_3d": wrist.get("raw_wrist_3d"),
            "depth_seed_valid": depth_seed_valid,
            "depth_seed_reason": depth_seed_reason,
        },
        "depth": {
            "depth_valid_count": roi_valid_count,
            "depth_valid_ratio": roi_valid_ratio,
            "depth_roi_empty": roi_empty,
        },
        "wrist_centered_mask": {
            "mask_area_px": mask_area,
            "connected_point_count": connected_points,
            "mask_reason": mask_reason,
        },
        "branch": {
            "branch_count": branch_count,
            "candidate_count": candidate_count,
            "branch_reason": estimator_reason,
        },
        "temporal_attribution": temporal,
        "shadow_only": True,
    }
