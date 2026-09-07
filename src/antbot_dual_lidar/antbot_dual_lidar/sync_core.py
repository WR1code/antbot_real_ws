"""Timestamp-only pairing logic; acquisition stamps are never modified."""

from collections import deque
from dataclasses import dataclass, field

import numpy as np


@dataclass
class SyncStatistics:
    received: dict = field(default_factory=lambda: {"front": 0, "rear": 0})
    dropped_by_stamp: dict = field(default_factory=lambda: {"front": 0, "rear": 0})
    unmatched: dict = field(default_factory=lambda: {"front": 0, "rear": 0})
    pairs: int = 0
    exact: int = 0
    within_5ms: int = 0
    within_20ms: int = 0
    over_50ms: int = 0
    around_100ms: int = 0
    deltas_ns: list = field(default_factory=list)
    previous_stamp: dict = field(default_factory=lambda: {"front": None, "rear": None})

    def receive(self, side: str, stamp_ns: int, expected_period_ns: int):
        self.received[side] += 1
        previous = self.previous_stamp[side]
        if previous is not None and stamp_ns > previous and expected_period_ns > 0:
            advance = stamp_ns - previous
            self.dropped_by_stamp[side] += max(
                0, int(round(advance / expected_period_ns)) - 1
            )
        self.previous_stamp[side] = stamp_ns

    def pair(self, delta_ns: int):
        self.pairs += 1
        self.deltas_ns.append(delta_ns)
        if len(self.deltas_ns) > 10000:
            del self.deltas_ns[:1000]
        self.exact += delta_ns == 0
        self.within_5ms += delta_ns <= 5_000_000
        self.within_20ms += delta_ns <= 20_000_000
        self.over_50ms += delta_ns > 50_000_000
        self.around_100ms += 80_000_000 <= delta_ns <= 120_000_000

    def summary(self):
        values = np.asarray(self.deltas_ns, dtype=np.float64) * 1e-9
        return {
            "received": dict(self.received),
            "dropped_by_stamp": dict(self.dropped_by_stamp),
            "unmatched": dict(self.unmatched),
            "pairs": self.pairs,
            "exact": self.exact,
            "within_5ms": self.within_5ms,
            "within_20ms": self.within_20ms,
            "over_50ms": self.over_50ms,
            "around_100ms": self.around_100ms,
            "mean_sec": float(values.mean()) if len(values) else None,
            "max_sec": float(values.max()) if len(values) else None,
            "p95_sec": float(np.percentile(values, 95)) if len(values) else None,
            "p99_sec": float(np.percentile(values, 99)) if len(values) else None,
        }


class SyncMatcher:
    """Bounded exact/approximate/latest-neighbor matcher."""

    def __init__(
        self,
        strategy="approximate",
        max_delta_ns=20_000_000,
        queue_size=20,
        expected_period_ns=100_000_000,
    ):
        if strategy not in ("exact", "approximate", "latest-neighbor"):
            raise ValueError(f"unsupported sync strategy: {strategy}")
        if max_delta_ns < 0 or queue_size < 1 or expected_period_ns < 1:
            raise ValueError("invalid synchronization limits")
        self.strategy = strategy
        self.max_delta_ns = int(max_delta_ns)
        self.queue_size = int(queue_size)
        self.expected_period_ns = int(expected_period_ns)
        self.queues = {"front": deque(), "rear": deque()}
        self.stats = SyncStatistics()

    def add(self, side, stamp_ns, payload):
        if side not in self.queues:
            raise ValueError(f"unknown side: {side}")
        self.stats.receive(side, stamp_ns, self.expected_period_ns)
        own = self.queues[side]
        own.append((int(stamp_ns), payload))
        while len(own) > self.queue_size:
            own.popleft()
            self.stats.unmatched[side] += 1
        other_side = "rear" if side == "front" else "front"
        other = self.queues[other_side]
        if not other:
            return None

        if self.strategy == "latest-neighbor":
            candidate_index = len(other) - 1
        else:
            candidate_index = min(
                range(len(other)),
                key=lambda index: abs(other[index][0] - stamp_ns),
            )
        other_stamp, other_payload = other[candidate_index]
        delta = abs(other_stamp - stamp_ns)
        allowed = delta == 0 if self.strategy == "exact" else delta <= self.max_delta_ns
        if not allowed:
            return None

        own_stamp, own_payload = own.pop()
        del other[candidate_index]
        self.stats.pair(delta)
        if side == "front":
            return own_payload, other_payload, delta
        return other_payload, own_payload, delta

