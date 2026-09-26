"""Replayable diagnostics and publish-gap tracking for the pulse target bridge."""

from __future__ import annotations

from collections import Counter, deque
from datetime import datetime
import json
import math
from pathlib import Path
import threading
from typing import Any


TARGET_REASON_CODES = (
    "NONE", "NO_CAMERA_FRAME", "NO_SYNCED_PAIR", "NO_DETECTION", "MULTIPLE_DETECTIONS",
    "INVALID_CAMERA_INFO", "INVALID_DEPTH", "INVALID_3D_POINT",
    "CONFIDENCE_TOO_LOW", "ROI_REJECTED", "FILTER_STABILIZING",
    "EMPTY_FRAME_ID", "MISSING_TIMESTAMP", "TARGET_MESSAGE_MISSING",
    "TF_LOOKUP_FAILED", "TF_STALE",
    "TARGET_OUT_OF_RANGE", "NONFINITE_TARGET", "RATE_LIMITED",
    "INTERNAL_EXCEPTION",
)


class TargetPipelineLog:
    def __init__(self, directory: str, now: datetime | None = None) -> None:
        stamp = now or datetime.now()
        path = Path(directory).expanduser()
        path.mkdir(parents=True, exist_ok=True)
        self.path = path / f"target_pipeline_{stamp:%Y%m%d_%H%M%S_%f}.jsonl"
        self._lock = threading.Lock()

    def write(self, event_type: str, **fields: Any) -> None:
        def safe(value: Any) -> Any:
            if isinstance(value, float) and not math.isfinite(value):
                return str(value)
            if isinstance(value, dict):
                return {key: safe(item) for key, item in value.items()}
            if isinstance(value, (tuple, list)):
                return [safe(item) for item in value]
            return value

        event = {
            "timestamp": datetime.now().astimezone().isoformat(timespec="milliseconds"),
            "event_type": event_type,
            **safe(fields),
        }
        line = json.dumps(event, ensure_ascii=False, allow_nan=False, separators=(",", ":"))
        with self._lock:
            with self.path.open("a", encoding="utf-8") as stream:
                stream.write(line + "\n")


class PublishGapMonitor:
    """Track current/max gaps and emit each configured warning level once per gap."""

    def __init__(self, thresholds_sec=(0.5, 1.0, 2.0)) -> None:
        self.thresholds_sec = tuple(sorted(float(value) for value in thresholds_sec))
        self.last_publish_time: float | None = None
        self.last_interval_sec: float | None = None
        self.max_gap_sec = 0.0
        self._warned: set[float] = set()
        self._publish_times: deque[float] = deque()

    def published(self, now: float) -> None:
        if self.last_publish_time is not None:
            self.last_interval_sec = max(0.0, now - self.last_publish_time)
            self.max_gap_sec = max(self.max_gap_sec, self.last_interval_sec)
        self.last_publish_time = now
        self._publish_times.append(now)
        self._prune(now)
        self._warned.clear()

    def _prune(self, now: float) -> None:
        while self._publish_times and now - self._publish_times[0] > 5.0:
            self._publish_times.popleft()

    def rate_hz(self, now: float) -> float:
        """Rolling 5 s rate, falling to zero during a publish drought."""
        self._prune(now)
        if len(self._publish_times) < 2:
            return 0.0
        span = self._publish_times[-1] - self._publish_times[0]
        return (len(self._publish_times) - 1) / span if span > 0.0 else 0.0

    def inspect(self, now: float) -> tuple[float | None, tuple[float, ...]]:
        if self.last_publish_time is None:
            return None, ()
        gap = max(0.0, now - self.last_publish_time)
        self.max_gap_sec = max(self.max_gap_sec, gap)
        crossed = tuple(
            threshold for threshold in self.thresholds_sec
            if gap > threshold and threshold not in self._warned
        )
        self._warned.update(crossed)
        return gap, crossed

    def current_gap(self, now: float) -> float | None:
        """Sample the gap without consuming any pending warning thresholds."""
        if self.last_publish_time is None:
            return None
        gap = max(0.0, now - self.last_publish_time)
        self.max_gap_sec = max(self.max_gap_sec, gap)
        return gap


class PipelineCounters:
    def __init__(self) -> None:
        self.frames_received = 0
        self.detections_received = 0
        self.detections_valid = 0
        self.depth_rejected = 0
        self.tf_failed = 0
        self.target_rejected = 0
        self.targets_published = 0
        self.tf_success = 0
        self.drop_reasons: Counter[str] = Counter()

    def dropped(self, reason: str, count: int = 1) -> None:
        count = max(0, count)
        self.target_rejected += count
        self.drop_reasons[reason] += count

    def snapshot(self) -> dict:
        return {
            "frames_received": self.frames_received,
            "detections_received": self.detections_received,
            "detections_valid": self.detections_valid,
            "depth_rejected": self.depth_rejected,
            "tf_failed": self.tf_failed,
            "tf_success": self.tf_success,
            "target_rejected": self.target_rejected,
            "targets_published": self.targets_published,
            "drop_reasons": dict(self.drop_reasons),
        }
