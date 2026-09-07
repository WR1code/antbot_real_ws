from types import SimpleNamespace

import numpy as np
import pytest

from antbot_rgbd_dataset.core import (
    DatasetWriter,
    ExactStampSynchronizer,
    KeyframePolicy,
    Selection,
    depth_statistics,
    pose_delta,
    select_keyframe,
    transform_matrix,
)


def message(timestamp):
    sec, nanosec = divmod(timestamp, 1_000_000_000)
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec))
    )


def test_exact_sync_only_emits_identical_timestamp_bundle():
    sync = ExactStampSynchronizer(capacity=4)
    assert sync.add("rgb", message(10)) is None
    assert sync.add("depth", message(10)) is None
    assert sync.add("color_info", message(10)) is None
    bundle = sync.add("depth_info", message(10))
    assert bundle is not None
    assert [item.header.stamp.nanosec for item in bundle] == [10, 10, 10, 10]


def test_exact_sync_counts_evicted_unmatched_messages():
    sync = ExactStampSynchronizer(capacity=1)
    sync.add("rgb", message(10))
    sync.add("rgb", message(20))
    assert sync.sync_drop_count == 1


def test_pose_delta_uses_full_three_dimensional_rotation():
    previous = np.eye(4)
    angle = np.deg2rad(12.0)
    current = np.eye(4)
    current[:3, :3] = [
        [1, 0, 0],
        [0, np.cos(angle), -np.sin(angle)],
        [0, np.sin(angle), np.cos(angle)],
    ]
    translation, rotation = pose_delta(previous, current)
    assert translation == pytest.approx(0.0)
    assert rotation == pytest.approx(12.0)


def test_stationary_max_interval_is_suppressed():
    policy = KeyframePolicy(maximum_interval_sec=2.0)
    selection = select_keyframe(
        policy,
        3_000_000_000,
        np.eye(4),
        100.0,
        1.0,
        0,
        np.eye(4),
    )
    assert not selection.selected
    assert selection.rejection == "MOTION"


def test_translation_rotation_and_manual_reasons():
    policy = KeyframePolicy()
    pose = np.eye(4)
    pose[0, 3] = 0.2
    selected = select_keyframe(policy, 1_000_000_000, pose, 100.0, 1.0, 0, np.eye(4))
    assert selected.selected
    assert "TRANSLATION" in selected.reasons
    manual = select_keyframe(policy, 10, np.eye(4), 100.0, 1.0, 0, np.eye(4), True)
    assert manual.reasons == ("MANUAL",)


def test_depth_stats_preserve_invalid_categories():
    depth = np.array([[0.0, np.nan], [np.inf, 1.0]], dtype=np.float32)
    stats = depth_statistics(depth, 0.1, 20.0)
    assert stats.valid_pixels == 1
    assert stats.zero_count == stats.nan_count == stats.inf_count == 1


def test_transform_quaternion_matrix_direction():
    matrix = transform_matrix(np.array([1.0, 2.0, 3.0]), np.array([0, 0, 0, 1]))
    point_world = matrix @ np.array([0.0, 0.0, 2.0, 1.0])
    assert np.allclose(point_world, [1.0, 2.0, 5.0, 1.0])


def test_atomic_frame_failure_leaves_no_csv_row_or_assets(tmp_path, monkeypatch):
    writer = DatasetWriter(tmp_path, "dataset", True, True)
    rgb = np.zeros((2, 2, 3), dtype=np.uint8)
    depth = np.ones((2, 2), dtype=np.float32)
    pose = {
        "fixed_frame": "odom",
        "camera_frame": "camera_color_optical_frame",
        "transform_definition": "p_odom = T_odom_camera * p_camera",
        "translation_m": {"x": 0, "y": 0, "z": 0},
        "quaternion_xyzw": {"x": 0, "y": 0, "z": 0, "w": 1},
        "T_odom_camera": np.eye(4).tolist(),
    }
    camera = {}
    stats = depth_statistics(depth, 0.1, 20.0)

    def fail(*_args, **_kwargs):
        raise IOError("injected")

    monkeypatch.setattr("antbot_rgbd_dataset.core.atomic_json", fail)
    with pytest.raises(IOError):
        writer.write_frame(
            1, rgb, depth, pose, camera, stats, 100.0, Selection(True, ("FIRST_FRAME",), 0, 0)
        )
    assert writer.rows == []
    assert not list((writer.root / "rgb").glob("*.png"))
    assert not list((writer.root / "depth").glob("*.npy"))
    assert not list((writer.root / "depth_png").glob("*.png"))
