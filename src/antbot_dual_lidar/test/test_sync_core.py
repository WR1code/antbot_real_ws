import pytest

from antbot_dual_lidar.sync_core import SyncMatcher


def test_exact_pairing_and_identity():
    matcher = SyncMatcher(strategy="exact")
    assert matcher.add("front", 100, "front") is None
    assert matcher.add("rear", 100, "rear") == ("front", "rear", 0)
    summary = matcher.stats.summary()
    assert summary["pairs"] == 1
    assert summary["exact"] == 1
    assert summary["within_5ms"] == 1


def test_approximate_threshold_and_percentiles():
    matcher = SyncMatcher(strategy="approximate", max_delta_ns=20_000_000)
    for index, delta in enumerate((1_000_000, 5_000_000, 19_000_000)):
        stamp = index * 100_000_000
        matcher.add("front", stamp, f"f{index}")
        pair = matcher.add("rear", stamp + delta, f"r{index}")
        assert pair[2] == delta
    summary = matcher.stats.summary()
    assert summary["pairs"] == 3
    assert summary["within_5ms"] == 2
    assert summary["within_20ms"] == 3
    assert summary["p95_sec"] <= 0.019


def test_100ms_mismatch_and_drop_counters():
    matcher = SyncMatcher(
        strategy="latest-neighbor",
        max_delta_ns=120_000_000,
        expected_period_ns=100_000_000,
    )
    matcher.add("front", 0, "f0")
    pair = matcher.add("rear", 100_000_000, "r1")
    assert pair[2] == 100_000_000
    matcher.add("front", 300_000_000, "f3")
    summary = matcher.stats.summary()
    assert summary["around_100ms"] == 1
    assert summary["over_50ms"] == 1
    assert summary["dropped_by_stamp"]["front"] == 2


def test_queue_eviction_and_invalid_configuration():
    matcher = SyncMatcher(queue_size=1)
    matcher.add("front", 0, "old")
    matcher.add("front", 100, "new")
    assert matcher.stats.unmatched["front"] == 1
    with pytest.raises(ValueError):
        SyncMatcher(strategy="invented")

