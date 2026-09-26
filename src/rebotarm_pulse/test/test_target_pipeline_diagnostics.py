from datetime import datetime
import json

import pytest

from rebotarm_pulse.target_pipeline_diagnostics import (
    PipelineCounters,
    PublishGapMonitor,
    TargetPipelineLog,
)


def test_publish_gap_thresholds_fire_once_per_gap_and_reset_after_publish():
    monitor = PublishGapMonitor((0.5, 1.0, 2.0))
    monitor.published(10.0)

    gap, crossed = monitor.inspect(10.6)
    assert gap == pytest.approx(0.6)
    assert crossed == (0.5,)
    assert monitor.inspect(10.9)[1] == ()
    assert monitor.inspect(11.1)[1] == (1.0,)
    assert monitor.inspect(12.1)[1] == (2.0,)

    monitor.published(12.2)
    assert monitor.last_interval_sec == pytest.approx(2.2)
    assert monitor.max_gap_sec == pytest.approx(2.2)
    assert monitor.inspect(12.8)[1] == (0.5,)


def test_gap_snapshot_does_not_consume_warning():
    monitor = PublishGapMonitor((0.5, 1.0, 2.0))
    monitor.published(10.0)
    assert monitor.current_gap(10.6) == pytest.approx(0.6)
    assert monitor.inspect(10.6)[1] == (0.5,)


def test_publish_rate_falls_to_zero_during_long_drought():
    monitor = PublishGapMonitor()
    for timestamp in (10.0, 10.1, 10.2, 10.3):
        monitor.published(timestamp)
    assert monitor.rate_hz(10.3) == pytest.approx(10.0)
    assert monitor.rate_hz(16.0) == 0.0


def test_pipeline_counters_keep_fixed_drop_reason_counts():
    counters = PipelineCounters()
    counters.frames_received = 3
    counters.detections_received = 2
    counters.detections_valid = 1
    counters.dropped("INVALID_DEPTH")
    counters.dropped("TF_LOOKUP_FAILED")
    snapshot = counters.snapshot()

    assert snapshot["frames_received"] == 3
    assert snapshot["target_rejected"] == 2
    assert snapshot["drop_reasons"] == {
        "INVALID_DEPTH": 1,
        "TF_LOOKUP_FAILED": 1,
    }


def test_jsonl_log_is_replayable_and_names_each_event(tmp_path):
    log = TargetPipelineLog(
        str(tmp_path), now=datetime(2026, 9, 13, 12, 34, 56)
    )
    log.write("frame_result", frame_seq=7, publish_target=False)
    log.write("target_drop", frame_seq=7, reason="ROI_REJECTED")

    assert log.path.name.startswith("target_pipeline_20260913_123456_")
    events = [json.loads(line) for line in log.path.read_text().splitlines()]
    assert [event["event_type"] for event in events] == [
        "frame_result", "target_drop"
    ]
    assert events[1]["reason"] == "ROI_REJECTED"


def test_jsonl_retains_nonfinite_rejection_as_string(tmp_path):
    log = TargetPipelineLog(str(tmp_path))
    log.write("target_drop", reason="NONFINITE_TARGET", camera_xyz=(float("nan"), 0.0, 1.0))
    event = json.loads(log.path.read_text().strip())
    assert event["camera_xyz"][0] == "nan"
