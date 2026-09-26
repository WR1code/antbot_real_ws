"""Noise-tolerant input gates for bounded Piper-H pre-contact motion."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import statistics
from typing import Iterable


Vector3 = tuple[float, float, float]


def target_gap_state(
    age_sec: float | None,
    grace_sec: float = 0.5,
    soft_sec: float = 1.5,
    hard_sec: float = 2.0,
) -> str:
    """Separate missing data from observed spatial instability."""
    if not 0.0 < grace_sec < soft_sec < hard_sec:
        raise ValueError("target timeout tiers must be strictly increasing")
    if age_sec is None:
        return "WAIT_TARGET"
    if age_sec <= grace_sec:
        return "TARGET_STABLE"
    if age_sec <= soft_sec:
        return "TARGET_TEMPORARILY_STALE"
    if age_sec <= hard_sec:
        return "WAIT_TARGET_RECOVERY"
    return "TARGET_LOST"


class RecoverableFreshnessGate:
    """A stale input is WAIT and becomes PASS again on the next fresh sample."""

    def __init__(self, max_age_sec: float) -> None:
        self.max_age_sec = max_age_sec
        self.last_received: float | None = None

    def update(self, timestamp: float) -> None:
        self.last_received = timestamp

    def result(self, now: float) -> str:
        if self.last_received is None or now - self.last_received > self.max_age_sec:
            return "WAIT"
        return "PASS"


def _distance(a: Vector3, b: Vector3) -> float:
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def _unit(vector: Vector3) -> Vector3:
    length = math.sqrt(sum(value * value for value in vector))
    if not math.isfinite(length) or length < 1e-9:
        raise ValueError("direction is not a finite non-zero vector")
    return tuple(value / length for value in vector)  # type: ignore[return-value]


def _median_vector(vectors: Iterable[Vector3]) -> Vector3:
    values = list(vectors)
    return tuple(statistics.median(row[index] for row in values) for index in range(3))  # type: ignore[return-value]


def _percentile(values: list[float], percentile: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    position = (len(ordered) - 1) * percentile
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


@dataclass(frozen=True)
class TargetStatus:
    raw: Vector3 | None
    filtered: Vector3 | None
    frozen: Vector3 | None
    sample_count: int
    jitter_m: float
    drift_m: float
    stable_duration_sec: float
    stable: bool
    hard_jump: bool
    hard_drift: bool
    stable_latched: bool = False
    reacquire_distance_m: float | None = None
    reacquire_result: str = "NONE"
    stable_reset_count: int = 0
    stable_reset_reason: str = "NONE"
    acquisition_state: str = "WAIT_TARGET"
    window_oldest_age_sec: float | None = None
    window_center: Vector3 | None = None
    oldest_sample_distance_m: float = 0.0


class TargetStabilityFilter:
    """P95 spatial target gate with 5/8/15 mm hysteresis."""

    def __init__(
        self,
        window_sec: float = 0.6,
        min_samples: int = 3,
        stable_radius_m: float = 0.005,
        stable_duration_sec: float = 0.0,
        soft_drift_m: float = 0.008,
        hard_drift_m: float = 0.015,
        hard_jump_m: float = 0.015,
        hard_drift_hold_sec: float = 0.2,
        ema_alpha: float = 0.35,
        inlier_fraction: float = 0.60,
        hard_timeout_sec: float = 2.0,
    ) -> None:
        self.window_sec = window_sec
        self.min_samples = min_samples
        self.stable_radius_m = stable_radius_m
        self.stable_duration_sec = stable_duration_sec
        self.soft_drift_m = soft_drift_m
        self.hard_drift_m = hard_drift_m
        self.hard_jump_m = hard_jump_m
        self.hard_drift_hold_sec = hard_drift_hold_sec
        self.ema_alpha = ema_alpha
        self.inlier_fraction = inlier_fraction
        self.hard_timeout_sec = hard_timeout_sec
        self.samples: deque[tuple[float, Vector3]] = deque()
        self.raw: Vector3 | None = None
        self.filtered: Vector3 | None = None
        self.frozen: Vector3 | None = None
        self.stable_since: float | None = None
        self.hard_drift_since: float | None = None
        self.acquisition_state = "WAIT_TARGET"
        self.window_center: Vector3 | None = None
        self.oldest_sample_distance_m = 0.0
        self.last_timestamp: float | None = None
        self.hard_jump = False
        self.reset_count = 0
        self.last_reset_reason = "NONE"
        self.reacquire_distance_m: float | None = None
        self.reacquire_result = "NONE"
        self.last_gap_sec: float | None = None

    def reset(self, reason: str = "NEW_TARGET") -> None:
        if self.samples or self.frozen is not None or self.filtered is not None:
            self.reset_count += 1
            self.last_reset_reason = reason
        self.samples.clear()
        self.raw = None
        self.filtered = None
        self.frozen = None
        self.stable_since = None
        self.hard_drift_since = None
        self.acquisition_state = "WAIT_TARGET"
        self.window_center = None
        self.oldest_sample_distance_m = 0.0
        self.hard_jump = False
        self.last_timestamp = None
        self.reacquire_distance_m = None
        self.reacquire_result = "NONE"
        self.last_gap_sec = None

    def add(self, timestamp: float, position: Vector3) -> TargetStatus:
        if not math.isfinite(timestamp) or not all(math.isfinite(v) for v in position):
            raise ValueError("target sample must contain finite timestamp and XYZ")
        if self.last_timestamp is not None and timestamp - self.last_timestamp > self.hard_timeout_sec:
            self.reset("TARGET_HARD_TIMEOUT")
        if self.last_timestamp is not None and timestamp < self.last_timestamp:
            raise ValueError("out-of-order target sample")
        self.last_gap_sec = timestamp - self.last_timestamp if self.last_timestamp is not None else None
        self.hard_jump = False
        self.last_timestamp = timestamp
        self.raw = position
        self.samples.append((timestamp, position))
        self._prune(timestamp)
        median = _median_vector(value for _stamp, value in self.samples)
        if self.filtered is None:
            self.filtered = median
        else:
            alpha = min(1.0, max(0.0, self.ema_alpha))
            self.filtered = tuple(
                alpha * new + (1.0 - alpha) * old
                for new, old in zip(median, self.filtered)
            )  # type: ignore[assignment]
        distances = [_distance(value, median) for _stamp, value in self.samples]
        jitter = _percentile(distances, 0.95)
        self.window_center = median
        self.oldest_sample_distance_m = distances[0]
        drift = _distance(self.filtered, self.frozen) if self.frozen is not None else _distance(position, median)
        changed = False
        if self.frozen is not None:
            self.reacquire_distance_m = drift
            raw_distance = _distance(position, self.frozen)
            if (jitter > self.hard_drift_m or drift > self.hard_drift_m
                    or raw_distance > self.hard_drift_m):
                self.reset_count += 1
                self.last_reset_reason = "TARGET_SPATIAL_DRIFT"
                self.frozen = None
                self.stable_since = None
                self.hard_drift_since = None
                self.reacquire_result = "TARGET_CHANGED"
                self.reacquire_distance_m = drift
                self.acquisition_state = "NEW_TARGET_ACQUISITION"
                changed = True
            elif jitter > self.soft_drift_m or drift > self.soft_drift_m:
                if self.hard_drift_since is None:
                    self.hard_drift_since = timestamp
                self.reacquire_result = "OBSERVE_SPATIAL_DRIFT"
                self.acquisition_state = "TARGET_OBSERVING_DRIFT"
                if timestamp - self.hard_drift_since >= self.hard_drift_hold_sec:
                    self.reset_count += 1
                    self.last_reset_reason = "TARGET_SPATIAL_DRIFT"
                    self.frozen = None
                    self.stable_since = None
                    self.hard_drift_since = None
                    self.acquisition_state = "REACQUIRING"
                    changed = True
            else:
                self.hard_drift_since = None
                self.reacquire_result = "REACQUIRED_SAME_TARGET"
                self.acquisition_state = "TARGET_STABLE"
        if self.frozen is None and not changed:
            if len(self.samples) >= self.min_samples and jitter <= self.stable_radius_m:
                self.frozen = self.filtered
                self.stable_since = timestamp
                self.reacquire_result = "NEW_STABLE_TARGET"
                self.acquisition_state = "TARGET_STABLE"
            else:
                self.acquisition_state = (
                    "NEW_TARGET_ACQUISITION" if len(self.samples) < self.min_samples
                    else "ACQUISITION_CONVERGING"
                )
        return self._status(timestamp, jitter, drift, changed)

    def status(self, now: float | None = None) -> TargetStatus:
        timestamp = self.last_timestamp if now is None else now
        if timestamp is None:
            timestamp = 0.0
        if self.last_timestamp is not None and timestamp - self.last_timestamp > self.hard_timeout_sec:
            self.reset("TARGET_HARD_TIMEOUT")
        self._prune(timestamp)
        if not self.samples or self.filtered is None:
            return self._status(timestamp, 0.0, 0.0, False)
        median = _median_vector(value for _stamp, value in self.samples)
        distances = [_distance(value, median) for _stamp, value in self.samples]
        self.window_center = median
        self.oldest_sample_distance_m = distances[0]
        drift = _distance(self.filtered, self.frozen) if self.frozen else 0.0
        return self._status(timestamp, _percentile(distances, 0.95), drift, False)

    def _status(self, timestamp: float, jitter: float, drift: float, changed: bool) -> TargetStatus:
        duration = max(0.0, timestamp - self.stable_since) if self.stable_since is not None else 0.0
        oldest_age = timestamp - self.samples[0][0] if self.samples else None
        # The 8 mm exit condition has a 200 ms hold. Until the latch is
        # actually released, spatial status remains stable.
        stable = self.frozen is not None
        return TargetStatus(
            raw=self.raw, filtered=self.filtered, frozen=self.frozen,
            sample_count=len(self.samples), jitter_m=jitter, drift_m=drift,
            stable_duration_sec=duration, stable=stable,
            hard_jump=self.hard_jump, hard_drift=changed,
            stable_latched=self.frozen is not None,
            reacquire_distance_m=self.reacquire_distance_m,
            reacquire_result=self.reacquire_result,
            stable_reset_count=self.reset_count,
            stable_reset_reason=self.last_reset_reason,
            acquisition_state=self.acquisition_state,
            window_oldest_age_sec=oldest_age,
            window_center=self.window_center,
            oldest_sample_distance_m=self.oldest_sample_distance_m,
        )

    def restart_after_drift(self) -> None:
        latest = (self.last_timestamp, self.filtered) if self.filtered is not None else None
        self.reset("TARGET_SPATIAL_DRIFT")
        if latest is not None:
            self.add(latest[0], latest[1])

    def _prune(self, now: float) -> None:
        while self.samples and now - self.samples[0][0] > self.window_sec:
            self.samples.popleft()


@dataclass(frozen=True)
class DirectionStatus:
    raw: Vector3 | None
    filtered: Vector3 | None
    sample_count: int
    jitter_deg: float
    stable: bool


class DirectionStabilityFilter:
    def __init__(self, window_sec: float = 0.8, min_samples: int = 3, stable_deg: float = 3.0) -> None:
        self.window_sec = window_sec
        self.min_samples = min_samples
        self.stable_deg = stable_deg
        self.samples: deque[tuple[float, Vector3]] = deque()

    def add(self, timestamp: float, vector: Vector3) -> DirectionStatus:
        unit = _unit(vector)
        self.samples.append((timestamp, unit))
        while self.samples and timestamp - self.samples[0][0] > self.window_sec:
            self.samples.popleft()
        return self._summarize()

    def _summarize(self) -> DirectionStatus:
        raw = self.samples[-1][1]
        try:
            mean = _unit(tuple(
                sum(vector[index] for _stamp, vector in self.samples)
                for index in range(3)
            ))  # type: ignore[arg-type]
        except ValueError:
            return DirectionStatus(raw, raw, len(self.samples), 180.0, False)
        angles = [
            math.degrees(math.acos(max(-1.0, min(1.0, sum(a * b for a, b in zip(v, mean))))))
            for _stamp, v in self.samples
        ]
        jitter = max(angles, default=0.0)
        return DirectionStatus(raw, mean, len(self.samples), jitter,
                               len(self.samples) >= self.min_samples and jitter <= self.stable_deg)

    def status(self) -> DirectionStatus:
        if not self.samples:
            return DirectionStatus(None, None, 0, 0.0, False)
        return self._summarize()


@dataclass(frozen=True)
class PressureStatus:
    raw_pa: float | None
    filtered_pa: float | None
    baseline_pa: float | None
    delta_pa: float | None
    over_threshold_duration_sec: float
    abort: bool
    emergency_abort: bool


class PressureDebouncer:
    def __init__(
        self,
        threshold_pa: float = 100.0,
        hold_sec: float = 0.15,
        window_size: int = 5,
        emergency_threshold_pa: float = 1000.0,
    ) -> None:
        self.threshold_pa = threshold_pa
        self.hold_sec = hold_sec
        self.emergency_threshold_pa = emergency_threshold_pa
        self.values: deque[float] = deque(maxlen=window_size)
        self.raw_pa: float | None = None
        self.baseline_pa: float | None = None
        self.over_since: float | None = None

    def set_baseline(self, baseline_pa: float | None) -> None:
        self.baseline_pa = baseline_pa
        self.over_since = None

    def add(self, timestamp: float, pressure_pa: float) -> PressureStatus:
        self.raw_pa = pressure_pa
        self.values.append(pressure_pa)
        filtered = statistics.median(self.values)
        delta = filtered - self.baseline_pa if self.baseline_pa is not None else None
        if delta is not None and abs(delta) > self.threshold_pa:
            if self.over_since is None:
                self.over_since = timestamp
        else:
            self.over_since = None
        duration = max(0.0, timestamp - self.over_since) if self.over_since is not None else 0.0
        emergency_abort = delta is not None and abs(delta) >= self.emergency_threshold_pa
        return PressureStatus(self.raw_pa, filtered, self.baseline_pa, delta, duration,
                              emergency_abort or (
                                  self.over_since is not None and duration >= self.hold_sec
                              ),
                              emergency_abort)

    def status(self, now: float) -> PressureStatus:
        filtered = statistics.median(self.values) if self.values else None
        delta = filtered - self.baseline_pa if filtered is not None and self.baseline_pa is not None else None
        duration = max(0.0, now - self.over_since) if self.over_since is not None else 0.0
        emergency_abort = delta is not None and abs(delta) >= self.emergency_threshold_pa
        return PressureStatus(self.raw_pa, filtered, self.baseline_pa, delta, duration,
                              emergency_abort or (
                                  self.over_since is not None and duration >= self.hold_sec
                              ),
                              emergency_abort)
