"""Bounded approximate RGB/depth and nearest-pose synchronization."""

from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class SynchronizedPair:
    color: Any
    depth: Any
    color_timestamp_ns: int
    depth_timestamp_ns: int

    @property
    def delta_ms(self) -> float:
        return abs(self.color_timestamp_ns - self.depth_timestamp_ns) / 1e6


class ApproximateRGBDSynchronizer:
    def __init__(self, max_delta_ms: float = 30.0, queue_size: int = 30):
        if max_delta_ms < 0 or queue_size < 1:
            raise ValueError("invalid approximate synchronization configuration")
        self.max_delta_ns = int(max_delta_ms * 1e6)
        self.queue_size = queue_size
        self.color: OrderedDict[int, Any] = OrderedDict()
        self.depth: OrderedDict[int, Any] = OrderedDict()
        self.received_color_frames = 0
        self.received_depth_frames = 0
        self.synchronized_rgbd_frames = 0
        self.dropped_rgb_depth_sync = 0

    def add_color(self, timestamp_ns: int, value: Any) -> SynchronizedPair | None:
        self.received_color_frames += 1
        self.color[int(timestamp_ns)] = value
        return self._match("color", int(timestamp_ns))

    def add_depth(self, timestamp_ns: int, value: Any) -> SynchronizedPair | None:
        self.received_depth_frames += 1
        self.depth[int(timestamp_ns)] = value
        return self._match("depth", int(timestamp_ns))

    def _match(self, stream: str, timestamp_ns: int) -> SynchronizedPair | None:
        own, other = (self.color, self.depth) if stream == "color" else (self.depth, self.color)
        if other:
            nearest = min(other, key=lambda value: abs(value - timestamp_ns))
            if abs(nearest - timestamp_ns) <= self.max_delta_ns:
                own_value = own.pop(timestamp_ns)
                other_value = other.pop(nearest)
                self.synchronized_rgbd_frames += 1
                if stream == "color":
                    return SynchronizedPair(own_value, other_value, timestamp_ns, nearest)
                return SynchronizedPair(other_value, own_value, nearest, timestamp_ns)
        self._trim(self.color)
        self._trim(self.depth)
        self._drop_unmatchable()
        return None

    def _trim(self, cache: OrderedDict[int, Any]) -> None:
        while len(cache) > self.queue_size:
            cache.popitem(last=False)
            self.dropped_rgb_depth_sync += 1

    def _drop_unmatchable(self) -> None:
        if not self.color or not self.depth:
            return
        newest_color, newest_depth = next(reversed(self.color)), next(reversed(self.depth))
        for cache, other_newest in ((self.color, newest_depth), (self.depth, newest_color)):
            stale = [stamp for stamp in cache if stamp < other_newest - self.max_delta_ns]
            for stamp in stale:
                del cache[stamp]
                self.dropped_rgb_depth_sync += 1

    def flush(self) -> None:
        self.dropped_rgb_depth_sync += len(self.color) + len(self.depth)
        self.color.clear()
        self.depth.clear()

    def statistics(self) -> dict[str, int]:
        return {
            "received_color_frames": self.received_color_frames,
            "received_depth_frames": self.received_depth_frames,
            "synchronized_rgbd_frames": self.synchronized_rgbd_frames,
            "dropped_rgb_depth_sync": self.dropped_rgb_depth_sync,
        }


class NearestPoseBuffer:
    def __init__(self, max_delta_ms: float = 50.0, queue_size: int = 100):
        self.max_delta_ns = int(max_delta_ms * 1e6)
        self.queue_size = queue_size
        self._poses: OrderedDict[int, Any] = OrderedDict()

    def add(self, timestamp_ns: int, value: Any) -> None:
        self._poses[int(timestamp_ns)] = value
        while len(self._poses) > self.queue_size:
            self._poses.popitem(last=False)

    def nearest(self, timestamp_ns: int) -> tuple[Any, int] | None:
        if not self._poses:
            return None
        stamp = min(self._poses, key=lambda value: abs(value - timestamp_ns))
        if abs(stamp - timestamp_ns) > self.max_delta_ns:
            return None
        return self._poses[stamp], stamp

