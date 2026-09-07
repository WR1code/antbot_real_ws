"""State tracking shared by the runtime diagnostics and tests."""

from collections import deque
from dataclasses import dataclass, field


@dataclass
class StreamState:
    count: int = 0
    frame_id: str = ""
    point_count: int = 0
    last_stamp_ns: int | None = None
    last_wall: float | None = None
    resets: int = 0
    intervals: deque = field(default_factory=lambda: deque(maxlen=50))

    def update(self, stamp_ns: int, wall_time: float, frame_id: str, point_count: int):
        reset = self.last_stamp_ns is not None and stamp_ns < self.last_stamp_ns
        if reset:
            self.resets += 1
            self.intervals.clear()
        elif self.last_stamp_ns is not None and stamp_ns > self.last_stamp_ns:
            self.intervals.append((stamp_ns - self.last_stamp_ns) * 1e-9)
        self.count += 1
        self.frame_id = frame_id
        self.point_count = point_count
        self.last_stamp_ns = stamp_ns
        self.last_wall = wall_time
        return reset

    @property
    def frequency(self):
        if not self.intervals:
            return 0.0
        return len(self.intervals) / sum(self.intervals)

    def timed_out(self, wall_time: float, timeout: float):
        return self.last_wall is None or wall_time - self.last_wall > timeout


@dataclass
class ClockState:
    count: int = 0
    last_stamp_ns: int | None = None
    unique_stamps: set = field(default_factory=set)
    consecutive_duplicates: int = 0
    nonconsecutive_duplicates: int = 0
    regressions: int = 0
    current_group_length: int = 0
    maximum_group_length: int = 0

    def update(self, stamp_ns: int):
        stamp_ns = int(stamp_ns)
        if self.last_stamp_ns is None:
            self.current_group_length = 1
        elif stamp_ns == self.last_stamp_ns:
            self.consecutive_duplicates += 1
            self.current_group_length += 1
        else:
            if stamp_ns in self.unique_stamps:
                self.nonconsecutive_duplicates += 1
            if stamp_ns < self.last_stamp_ns:
                self.regressions += 1
            self.current_group_length = 1
        self.maximum_group_length = max(
            self.maximum_group_length, self.current_group_length
        )
        self.unique_stamps.add(stamp_ns)
        self.last_stamp_ns = stamp_ns
        self.count += 1
