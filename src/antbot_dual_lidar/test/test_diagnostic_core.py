from antbot_dual_lidar.diagnostic_core import ClockState, StreamState


def test_frequency_timeout_and_reset():
    state = StreamState()
    assert state.timed_out(1.0, 0.5)
    assert not state.update(1_000_000_000, 1.0, "front", 10)
    assert not state.update(1_100_000_000, 1.1, "front", 20)
    assert round(state.frequency, 6) == 10.0
    assert not state.timed_out(1.2, 0.5)
    assert state.timed_out(2.0, 0.5)
    assert state.update(0, 2.1, "front", 5)
    assert state.resets == 1
    assert state.frequency == 0.0


def test_two_streams_are_independent():
    front, rear = StreamState(), StreamState()
    front.update(1, 1.0, "front", 10)
    rear.update(2, 1.0, "rear", 20)
    front.update(3, 1.1, "front", 11)
    assert front.count == 2
    assert rear.count == 1
    assert rear.point_count == 20


def test_clock_duplicate_and_reset_statistics():
    state = ClockState()
    for stamp in (10, 10, 20, 10, 30, 30, 30):
        state.update(stamp)
    assert state.count == 7
    assert len(state.unique_stamps) == 3
    assert state.consecutive_duplicates == 3
    assert state.nonconsecutive_duplicates == 1
    assert state.regressions == 1
    assert state.maximum_group_length == 3
