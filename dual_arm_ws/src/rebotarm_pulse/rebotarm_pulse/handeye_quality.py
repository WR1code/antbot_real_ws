"""ROS-independent observation quality; thresholds are not solver weights."""

from collections import Counter
from dataclasses import asdict, dataclass
import math

import numpy as np


@dataclass(frozen=True)
class QualityConfig:
    min_board_area: float = 0.01
    max_reprojection_error: float = 3.0
    min_corner_distribution: float = 0.005
    reprojection_epsilon: float = 0.1
    area_weight: float = 0.4
    reprojection_weight: float = 0.03
    distribution_weight: float = 0.3

    def __post_init__(self):
        values = asdict(self)
        if any(not math.isfinite(value) or value < 0 for value in values.values()):
            raise ValueError("quality parameters must be finite and nonnegative")
        if self.reprojection_epsilon <= 0 or self.max_reprojection_error <= 0:
            raise ValueError("quality epsilon and maximum RMS must be positive")
        if self.min_board_area > 1 or self.min_corner_distribution > 1:
            raise ValueError("normalized quality thresholds must not exceed one")


@dataclass(frozen=True)
class QualityResult:
    valid: bool
    score: float
    board_area: float
    reprojection_error: float
    corner_distribution: float
    reasons: list[str]
    normalized_board_area: float

    def to_dict(self):
        return asdict(self)


def evaluate_observation_quality(observation, config=None):
    """Area is measured-corner convex hull area / image area.

    For normalized corners p=(u/width,v/height), C=mean((p-mean(p))
    (p-mean(p))^T). Distribution=min(1,12*sqrt(det(C))). The factor 12
    makes a uniform rectangular corner cloud's value equal its image-area
    fraction. This measures 2-D spread, not board-centre/image-centre bias.
    """
    config = config or QualityConfig()
    reasons = []
    width, height = observation.image_width, observation.image_height
    points = np.asarray(observation.corners, dtype=np.float64)
    rms, area = float(observation.reprojection_error), float(observation.board_area)
    normalized_area, distribution = 0.0, 0.0
    if width <= 0 or height <= 0:
        reasons.append("MISSING_IMAGE_SIZE")
    elif (points.ndim != 2 or points.shape[1] != 2 or len(points) < 3
          or not np.all(np.isfinite(points)) or not math.isfinite(area) or area < 0
          or np.any(points < 0) or np.any(points[:, 0] > width)
          or np.any(points[:, 1] > height)):
        reasons.append("INVALID_CORNERS_OR_AREA")
    else:
        normalized_area = min(1.0, area / (width * height))
        normalized = points / [width, height]
        centered = normalized - np.mean(normalized, axis=0)
        covariance = centered.T @ centered / len(points)
        distribution = min(1.0, 12.0 * math.sqrt(max(0.0, float(np.linalg.det(covariance)))))
    if normalized_area < config.min_board_area:
        reasons.append("LOW_AREA")
    if distribution < config.min_corner_distribution:
        reasons.append("POOR_CORNER_DISTRIBUTION")
    inverse_rms = 0.0
    if not math.isfinite(rms) or rms < 0 or rms > config.max_reprojection_error:
        reasons.append("HIGH_REPROJECTION_ERROR")
    if math.isfinite(rms) and rms >= 0:
        inverse_rms = 1.0 / (rms + config.reprojection_epsilon)
    score = (config.area_weight * normalized_area
             + config.reprojection_weight * inverse_rms
             + config.distribution_weight * distribution)
    return QualityResult(not reasons, float(score), area, rms, distribution,
                         reasons, normalized_area)


def summarize_quality(results):
    results = list(results)
    scores = [item.score for item in results]
    valid = sum(item.valid for item in results)
    return {
        "total": len(results), "valid": valid, "invalid": len(results) - valid,
        "mean_score": float(np.mean(scores)) if scores else 0.0,
        "min_score": min(scores) if scores else 0.0,
        "max_score": max(scores) if scores else 0.0,
        "reject_reason_count": dict(Counter(reason for item in results for reason in item.reasons)),
    }
