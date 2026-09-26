import struct

import pytest

from piper.teach_can_grouper import RAD_PER_RAW_UNIT, TeachCanGrouper


def payload(a, b):
    return struct.pack(">ii", a, b)


def test_groups_ordered_cycle_with_latest_timestamp():
    grouper = TeachCanGrouper()
    assert grouper.add(0x2A5, 100.0000, payload(1, 2)) is None
    assert grouper.add(0x2A6, 100.0002, payload(3, 4)) is None
    result = grouper.add(0x2A7, 100.0004, payload(5, 6))
    assert result.timestamp == pytest.approx(100.0004)
    assert result.span == pytest.approx(.0004)
    assert result.positions == pytest.approx(tuple(x * RAD_PER_RAW_UNIT for x in range(1, 7)))
    assert result.dropped_cycles == 0
    assert result.timing_valid


def test_missing_or_out_of_order_frame_drops_cycle():
    grouper = TeachCanGrouper()
    grouper.add(0x2A5, 1.0, payload(1, 2))
    grouper.add(0x2A5, 1.005, payload(1, 2))
    grouper.add(0x2A7, 1.0052, payload(5, 6))
    assert grouper.dropped_cycles == 2
    assert grouper.add(0x2A6, 1.0101, payload(3, 4)) is None


def test_wide_span_and_realtime_jump_are_diagnostic():
    grouper = TeachCanGrouper(maximum_span_sec=.002, realtime_jump_sec=1.0)
    grouper.add(0x2A5, 1.0, payload(1, 2))
    grouper.add(0x2A6, 1.001, payload(3, 4))
    first = grouper.add(0x2A7, 1.003, payload(5, 6))
    assert first.wide_span_cycles == 1
    grouper.add(0x2A5, 3.0, payload(1, 2))
    grouper.add(0x2A6, 3.001, payload(3, 4))
    second = grouper.add(0x2A7, 3.0015, payload(5, 6))
    assert not second.timing_valid
