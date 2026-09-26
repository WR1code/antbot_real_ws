import json
from pathlib import Path

import numpy as np
import pytest

from rebot_teach_mode.offline_replay import prepare, read_raw


def _fixture(tmp_path: Path, times, positions):
    raw = tmp_path / "raw.teach.json"
    raw.write_text(json.dumps({
        "joint_names": [f"joint{i}" for i in range(1, 7)],
        "points": [
            {"time_from_start": t, "positions": q, "feedback_timestamp": 100 + t}
            for t, q in zip(times, positions)
        ],
    }), encoding="utf-8")
    limits = tmp_path / "limits.yaml"
    limits.write_text("joint_limits:\n" + "".join(
        f"  joint{i}: {{has_velocity_limits: true, max_velocity: 0.5, "
        "has_acceleration_limits: true, max_acceleration: 1.0}\n"
        for i in range(1, 7)), encoding="utf-8")
    return raw, limits


def test_shape_preserving_resample_and_common_time_scale(tmp_path):
    raw, limits = _fixture(
        tmp_path, [0, .1, .2, .3],
        [[0, 0, 0, 0, 0, 0], [.1, -.1, 0, 0, 0, 0],
         [.2, -.2, 0, 0, 0, 0], [.2, -.2, 0, 0, 0, 0]],
    )
    original = raw.read_bytes()
    out = tmp_path / "replay"
    meta = prepare(raw, limits, out)
    samples = np.load(out / "replay_100hz.npy", mmap_mode="r")
    assert raw.read_bytes() == original
    assert meta["time_scale"] > 1
    assert meta["raw_samples"] == 4
    assert samples.dtype.names == ("replay_time", "j1", "j2", "j3", "j4", "j5", "j6")
    assert np.diff(samples["replay_time"]).min() > 0
    assert samples["j1"].min() >= 0 and samples["j1"].max() <= .2 + 1e-12
    assert samples["j2"].max() <= 0 and samples["j2"].min() >= -.2 - 1e-12
    assert meta["max_interpolant_knot_error_rad"] < 1e-12
    assert max(meta["max_nearest_grid_error_rad"]) < .001
    assert max(np.abs(np.diff(samples["j1"]) / np.diff(samples["replay_time"]))) <= .5 + 1e-3


def test_gap_diagnostics_keep_joint_delta(tmp_path):
    raw, limits = _fixture(tmp_path, [0, .01, .06, .20],
                           [[0]*6, [0]*6, [.01]*6, [.2]*6])
    meta = prepare(raw, limits, tmp_path / "replay")
    assert meta["gap_counts"] == {"over_30_ms": 2, "over_50_ms": 1,
                                  "over_100_ms": 1}
    assert meta["max_gap"]["start_time"] == pytest.approx(.06)
    assert meta["max_gap"]["joint_delta_across_gap"] == pytest.approx([.19]*6)
    assert meta["max_gap"]["warning"]


def test_rejects_backward_raw_time(tmp_path):
    raw, _ = _fixture(tmp_path, [0, .1, .09], [[0]*6]*3)
    with pytest.raises(ValueError, match="strictly|increase"):
        read_raw(raw)
