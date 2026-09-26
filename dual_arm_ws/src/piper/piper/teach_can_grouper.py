"""Group Piper 0x2A5/0x2A6/0x2A7 feedback without sending CAN frames."""

from dataclasses import dataclass
import struct


JOINT_IDS = (0x2A5, 0x2A6, 0x2A7)
RAD_PER_RAW_UNIT = 0.017444 / 1000.0


@dataclass(frozen=True)
class GroupedJointSample:
    timestamp: float
    pair_timestamps: tuple[float, float, float]
    positions: tuple[float, ...]
    span: float
    dropped_cycles: int
    wide_span_cycles: int
    timing_valid: bool


class TeachCanGrouper:
    def __init__(self, maximum_span_sec: float = 0.002,
                 realtime_jump_sec: float = 1.0):
        self.maximum_span_sec = float(maximum_span_sec)
        self.realtime_jump_sec = float(realtime_jump_sec)
        self.dropped_cycles = 0
        self.wide_span_cycles = 0
        self.timing_valid = True
        self._cycle = {}
        self._last_sample_timestamp = None

    def add(self, can_id: int, timestamp: float, payload: bytes):
        if can_id not in JOINT_IDS or len(payload) != 8:
            return None
        timestamp = float(timestamp)
        if can_id == 0x2A5:
            if self._cycle:
                self.dropped_cycles += 1
            self._cycle = {can_id: (timestamp, payload)}
            return None
        expected = 0x2A6 if can_id == 0x2A6 else 0x2A7
        predecessor = 0x2A5 if expected == 0x2A6 else 0x2A6
        if predecessor not in self._cycle or expected in self._cycle:
            if self._cycle:
                self.dropped_cycles += 1
            self._cycle = {}
            return None
        if timestamp < self._cycle[predecessor][0]:
            self.timing_valid = False
            self.dropped_cycles += 1
            self._cycle = {}
            return None
        self._cycle[can_id] = (timestamp, payload)
        if can_id != 0x2A7:
            return None
        stamps = tuple(self._cycle[value][0] for value in JOINT_IDS)
        span = max(stamps) - min(stamps)
        if span > self.maximum_span_sec:
            self.wide_span_cycles += 1
        sample_stamp = max(stamps)
        if self._last_sample_timestamp is not None:
            delta = sample_stamp - self._last_sample_timestamp
            if delta <= 0.0 or delta > self.realtime_jump_sec:
                self.timing_valid = False
        values = []
        for value in JOINT_IDS:
            values.extend(raw * RAD_PER_RAW_UNIT for raw in struct.unpack(">ii", self._cycle[value][1]))
        self._last_sample_timestamp = sample_stamp
        self._cycle = {}
        return GroupedJointSample(sample_stamp, stamps, tuple(values), span,
                                  self.dropped_cycles, self.wide_span_cycles,
                                  self.timing_valid)
