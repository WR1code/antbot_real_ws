"""Bounded live-frame source fed by a ROS message-conversion layer."""

from collections import deque
from threading import Lock

from ..models import RGBDFrame


class LiveROS2RGBDSource:
    """Queue adapter for online consumers; it contains no simulator dependency."""

    def __init__(self, capacity: int = 8):
        if capacity < 1:
            raise ValueError("live source capacity must be positive")
        self.capacity = capacity
        self._frames = deque()
        self._lock = Lock()
        self._started = False
        self.dropped_queue_overflow = 0

    def start(self) -> None:
        self._started = True

    def push(self, frame: RGBDFrame) -> None:
        frame.validate()
        if not self._started:
            raise RuntimeError("live source is not started")
        with self._lock:
            if len(self._frames) >= self.capacity:
                self._frames.popleft()
                self.dropped_queue_overflow += 1
            self._frames.append(frame)

    def read(self) -> RGBDFrame | None:
        if not self._started:
            raise RuntimeError("live source is not started")
        with self._lock:
            return self._frames.popleft() if self._frames else None

    def stop(self) -> None:
        with self._lock:
            self._frames.clear()
        self._started = False
