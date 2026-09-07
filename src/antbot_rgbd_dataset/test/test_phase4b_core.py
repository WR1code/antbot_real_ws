import importlib
from pathlib import Path
import sys

import numpy as np
import pytest
import yaml

from antbot_rgbd_dataset.configuration import (
    load_ros_parameter_yaml, validate_capture_parameters
)
from antbot_rgbd_dataset.depth import color_to_rgb, depth_to_meters, filter_depth
from antbot_rgbd_dataset.keyframes import KeyframeConfig, decide_keyframe
from antbot_rgbd_dataset.live_preview import (
    LivePreviewAccumulator, depth_continuity_mask, load_preview_archive, project_rgbd,
    project_rgbd_filtered,
    save_preview_archive, voxel_reduce,
)
from antbot_rgbd_dataset.models import CameraIntrinsics, RGBDFrame
from antbot_rgbd_dataset.phase4b_writer import Phase4BDatasetWriter
from antbot_rgbd_dataset.rebuild_preview import estimate_rigid_transform
from antbot_rgbd_dataset.sources.dataset import DatasetRGBDSource
from antbot_rgbd_dataset.sources.live_ros2 import LiveROS2RGBDSource
from antbot_rgbd_dataset.synchronization import (
    ApproximateRGBDSynchronizer, NearestPoseBuffer
)
from antbot_rgbd_dataset.transforms import (
    PoseQualityConfig, check_pose_quality, invert_transform,
    query_pose_at_timestamp,
)


PROJECT_ROOT = Path(__file__).resolve().parents[4]
PHASE2_DATASET = PROJECT_ROOT / "artifacts/rgbd_datasets/home_rgbd_phase2_01"


def transform(x=0.0, yaw_deg=0.0):
    result = np.eye(4)
    angle = np.deg2rad(yaw_deg)
    result[:3, :3] = [
        [np.cos(angle), -np.sin(angle), 0],
        [np.sin(angle), np.cos(angle), 0],
        [0, 0, 1],
    ]
    result[0, 3] = x
    return result


def frame(index=0, stamp=1_000_000_000, matrix=None):
    intr = CameraIntrinsics(
        4, 3, 3.0, 3.0, 2.0, 1.0, frame_id="camera_optical",
        rectified=True
    )
    return RGBDFrame(
        stamp, index, np.full((3, 4, 3), [240, 10, 20], np.uint8),
        np.ones((3, 4), np.float32), intr,
        transform() if matrix is None else matrix, "odometry",
        "camera_optical", "camera_optical", "camera_optical", "odom",
        stamp, stamp + 1_000_000, stamp + 2_000_000,
        metadata={"quaternion_xyzw": {"x": 0, "y": 0, "z": 0, "w": 1}},
    )


def test_transform_direction_inverse():
    T_odom_camera = transform(x=1.0, yaw_deg=90)
    assert np.allclose(invert_transform(T_odom_camera) @ T_odom_camera, np.eye(4))


def test_16uc1_millimeters_convert_to_meters():
    result = depth_to_meters(np.array([[1250]], np.uint16), "16UC1")
    assert result.dtype == np.float32
    assert result[0, 0] == pytest.approx(1.25)


def test_32fc1_meters_are_preserved():
    result = depth_to_meters(np.array([[1.25]], np.float32), "32FC1")
    assert result[0, 0] == pytest.approx(1.25)


def test_invalid_depth_categories_are_filtered():
    value = np.array([[0, -1, np.nan, np.inf, 1.0, 9.0]], np.float32)
    filtered, stats = filter_depth(value, 0.1, 8.0)
    assert np.array_equal(filtered, [[0, 0, 0, 0, 1, 0]])
    assert stats["valid_pixels"] == 1
    assert stats["negative_count"] == stats["nan_count"] == stats["inf_count"] == 1


def test_bgr_to_rgb_channel_order():
    bgr = np.array([[[20, 10, 240]]], np.uint8)
    assert color_to_rgb(bgr, "bgr8").tolist() == [[[240, 10, 20]]]


def test_rgb_depth_over_limit_is_not_synchronized():
    sync = ApproximateRGBDSynchronizer(max_delta_ms=30, queue_size=2)
    sync.add_color(0, "color")
    assert sync.add_depth(31_000_000, "depth") is None
    assert sync.dropped_rgb_depth_sync >= 1


def test_rgb_depth_within_limit_is_synchronized():
    sync = ApproximateRGBDSynchronizer(max_delta_ms=30)
    sync.add_color(100_000_000, "color")
    pair = sync.add_depth(125_000_000, "depth")
    assert pair and pair.delta_ms == pytest.approx(25)


def test_pose_over_limit_is_rejected():
    poses = NearestPoseBuffer(max_delta_ms=50)
    poses.add(0, "pose")
    assert poses.nearest(51_000_000) is None


def test_tf_query_failure_is_nonfatal():
    result, reason = query_pose_at_timestamp(
        lambda _: (_ for _ in ()).throw(RuntimeError("missing TF")), 123, 50
    )
    assert result is None and reason.startswith("POSE_QUERY_FAILED")


def test_pose_query_receives_real_sensor_timestamp_not_latest():
    queried = []
    expected = 987_654_321
    result, reason = query_pose_at_timestamp(
        lambda stamp: (queried.append(stamp) or (np.eye(4), [0, 0, 0, 1], stamp, "tf")),
        expected, 50,
    )
    assert reason is None and result is not None
    assert queried == [expected]


def test_translation_jump_is_rejected_without_changing_external_state():
    ok, reason, _ = check_pose_quality(
        PoseQualityConfig(max_translation_jump_m=0.5),
        0, transform(), 1_000_000_000, transform(x=0.6)
    )
    assert not ok and reason == "TRANSLATION_JUMP"


def test_keyframe_translation_trigger():
    decision = decide_keyframe(
        KeyframeConfig(), 1_000_000_000, transform(x=0.09), 0, transform()
    )
    assert decision.selected and "TRANSLATION" in decision.reasons


def test_keyframe_rotation_trigger():
    decision = decide_keyframe(
        KeyframeConfig(), 1_000_000_000, transform(yaw_deg=8), 0, transform()
    )
    assert decision.selected and "ROTATION" in decision.reasons


def test_keyframe_maximum_interval_trigger():
    decision = decide_keyframe(
        KeyframeConfig(), 1_100_000_000, transform(), 0, transform()
    )
    assert decision.selected and "MAX_INTERVAL" in decision.reasons


def test_keyframe_minimum_interval_rejects_motion():
    decision = decide_keyframe(
        KeyframeConfig(), 50_000_000, transform(x=1), 0, transform()
    )
    assert not decision.selected and decision.rejection == "MINIMUM_INTERVAL"


def test_unaligned_depth_rejects_colored_processing():
    value = frame()
    object.__setattr__(value, "depth_aligned_to_color", False)
    with pytest.raises(ValueError, match="depth_aligned_to_color"):
        value.validate(require_color_alignment=True)


def test_live_preview_projects_color_and_reduces_voxels():
    value = frame()
    points, colors = project_rgbd(value, pixel_stride=1, depth_minimum_m=0.1, depth_maximum_m=8)
    assert points.shape == (12, 3)
    assert colors.shape == (12, 3)
    assert colors[0].tolist() == [240, 10, 20]
    reduced_points, reduced_colors = voxel_reduce(points, colors, voxel_size_m=10)
    assert len(reduced_points) == len(reduced_colors) == 4


def test_depth_discontinuity_filter_removes_both_sides_of_object_edge():
    depth = np.ones((3, 4), np.float32)
    depth[:, 2:] = 2.0
    mask = depth_continuity_mask(depth, 0.08, 0.03)
    assert mask[:, 0].all() and mask[:, 3].all()
    assert not mask[:, 1].any() and not mask[:, 2].any()


def test_filtered_projection_rejects_depth_edge_flying_points():
    value = frame()
    value.depth_m[:, 2:] = 2.0
    points, colors = project_rgbd_filtered(value, 1, 0.1, 8.0, 0.08, 0.03)
    assert len(points) == len(colors) == 6


def test_live_preview_accumulates_coverage_and_growth():
    accumulator = LivePreviewAccumulator(
        preview_voxel_m=0.05, coverage_voxel_m=0.20, growth_window_s=10
    )
    points = np.array([[0.01, 0.01, 1.0], [0.02, 0.02, 1.01]], np.float32)
    colors = np.array([[255, 0, 0], [0, 255, 0]], np.uint8)
    first = accumulator.add_keyframe(1_000_000_000, points, colors, np.eye(4))
    second = accumulator.add_keyframe(2_000_000_000, points, colors, np.eye(4))
    assert first.occupied_voxels == second.occupied_voxels == 1
    assert first.new_voxels == 1
    assert second.new_voxels == 0
    assert second.direction_counts["front"] == 2
    cloud_points, cloud_colors = accumulator.accumulated_arrays()
    assert len(cloud_points) == len(cloud_colors) == 1
    assert cloud_points[0] == pytest.approx([0.015, 0.015, 1.005])


def test_preview_minimum_observations_removes_single_frame_voxels():
    accumulator = LivePreviewAccumulator()
    transform_value = np.eye(4)
    accumulator.add_keyframe(
        1, np.array([[0.01, 0.01, 1.0]], np.float32),
        np.array([[255, 0, 0]], np.uint8), transform_value,
    )
    points, _ = accumulator.accumulated_arrays(minimum_observations=2)
    assert len(points) == 0
    accumulator.add_keyframe(
        2, np.array([[0.02, 0.02, 1.01]], np.float32),
        np.array([[0, 0, 255]], np.uint8), transform_value,
    )
    points, colors = accumulator.accumulated_arrays(minimum_observations=2)
    assert points[0] == pytest.approx([0.015, 0.015, 1.005])
    assert colors[0].tolist() == [128, 0, 128]


def test_preview_archive_round_trip(tmp_path):
    points = np.array([[1.0, 2.0, 3.0]], np.float32)
    colors = np.array([[10, 20, 30]], np.uint8)
    path = tmp_path / "rgbd_preview.npz"
    save_preview_archive(path, points, colors, "map")
    loaded_points, loaded_colors, frame_id = load_preview_archive(path)
    assert np.array_equal(loaded_points, points)
    assert np.array_equal(loaded_colors, colors)
    assert frame_id == "map"


def test_rigid_alignment_recovers_preview_frame_transform():
    source = np.array([
        [0.0, 0.0, 0.0], [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0], [0.0, 0.0, 1.0],
    ])
    expected = transform(x=2.0, yaw_deg=30)
    target = source @ expected[:3, :3].T + expected[:3, 3]
    actual, rms = estimate_rigid_transform(source, target)
    assert actual == pytest.approx(expected)
    assert rms < 1e-12


def test_missing_configuration_field_has_explicit_error():
    with pytest.raises(ValueError, match="missing required"):
        validate_capture_parameters({"color_topic": "/color"})


def test_real_robot_configuration_is_valid():
    path = Path(__file__).resolve().parents[1] / "config/rgbd_real_robot.yaml"
    config = load_ros_parameter_yaml(path)
    assert not config["use_sim_time"]
    assert config["pose_mode"] == "tf"


def test_dataset_write_read_round_trip(tmp_path):
    writer = Phase4BDatasetWriter(tmp_path, "roundtrip", minimum_free_space_gb=0)
    value = frame()
    decision = decide_keyframe(KeyframeConfig(), value.timestamp_ns, value.T_world_camera, None, None)
    writer.write(value, decision, sharpness=100, depth_minimum_m=0.1, depth_maximum_m=8)
    writer.finalize({
        "first_frame": value, "world_frame": "odom", "camera_frame": "camera_optical",
        "pose_source": "odometry", "robot_id": "antbot", "sensor_id": "rgbd",
        "configuration": {}, "counters": {},
    })
    source = DatasetRGBDSource(tmp_path / "roundtrip")
    loaded = next(iter(source))
    assert np.array_equal(loaded.color, value.color)
    assert np.array_equal(loaded.depth_m, value.depth_m)
    assert np.allclose(loaded.T_world_camera, value.T_world_camera)
    assert loaded.color_timestamp_ns == value.color_timestamp_ns


def test_interrupted_capture_metadata_remains_readable(tmp_path):
    writer = Phase4BDatasetWriter(tmp_path, "interrupted", minimum_free_space_gb=0)
    writer.finalize({
        "world_frame": "odom", "camera_frame": "camera_optical",
        "pose_source": "tf", "robot_id": "antbot", "sensor_id": "rgbd",
        "configuration": {}, "counters": {"received_color_frames": 3},
    }, complete=False)
    metadata = yaml.safe_load((writer.root / "dataset_metadata.yaml").read_text())
    summary = yaml.safe_load((writer.root / "capture_summary.yaml").read_text())
    assert metadata["complete"] is False
    assert summary["complete"] is False


def test_core_and_source_modules_import_without_ros_or_isaac():
    before = set(sys.modules)
    for module in (
        "antbot_rgbd_dataset.models", "antbot_rgbd_dataset.depth",
        "antbot_rgbd_dataset.synchronization", "antbot_rgbd_dataset.transforms",
        "antbot_rgbd_dataset.keyframes", "antbot_rgbd_dataset.sources.dataset",
    ):
        importlib.import_module(module)
    newly_loaded = set(sys.modules) - before
    assert not any(name == "rclpy" or name.startswith("isaac") for name in newly_loaded)


def test_live_source_is_bounded_and_returns_unified_frames():
    source = LiveROS2RGBDSource(capacity=1)
    source.start()
    source.push(frame(index=0))
    source.push(frame(index=1, stamp=2_000_000_000))
    assert source.dropped_queue_overflow == 1
    assert source.read().frame_index == 1
    assert source.read() is None
    source.stop()


@pytest.mark.skipif(not PHASE2_DATASET.is_dir(), reason="Phase 2 dataset unavailable")
def test_phase2_dataset_source_preserves_all_frames_and_contract():
    source = DatasetRGBDSource(PHASE2_DATASET)
    frames = list(source)
    assert len(frames) == 35
    assert frames[0].pose_source == "dataset"
    assert frames[0].depth_m.dtype == np.float32
    assert frames[0].color.shape == (480, 640, 3)
    assert all(a.timestamp_ns < b.timestamp_ns for a, b in zip(frames, frames[1:]))
