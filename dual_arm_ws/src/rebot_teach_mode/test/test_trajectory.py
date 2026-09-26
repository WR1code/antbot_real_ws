import json

import pytest

from rebot_teach_mode.trajectory import (
    RecordedPoint,
    RecordedTrajectory,
    TrajectoryRecorder,
    build_preview_loop,
    build_playback_plan,
    concatenate_trajectories,
    concatenate_trajectories_with_transitions,
    load_trajectory,
    prepare_rviz_preview,
    sample_trajectory,
    save_trajectory,
    validate_joint_limits,
)


NAMES = tuple(f"joint{index}" for index in range(1, 7))


def trajectory():
    return RecordedTrajectory(
        NAMES,
        (
            RecordedPoint(0.0, (0.0, 0.5, 0.5, 0.0, 0.0, 0.0)),
            RecordedPoint(1.0, (0.1, 0.6, 0.6, 0.1, 0.0, 0.0)),
            RecordedPoint(2.0, (0.2, 0.7, 0.7, 0.2, 0.0, 0.0)),
        ),
        "2026-08-25T00:00:00+00:00",
    )


def test_recorder_downsamples_and_retains_final_feedback():
    recorder = TrajectoryRecorder(
        NAMES, sample_period_sec=0.02, minimum_position_delta=0.01
    )
    recorder.start(10.0, [0.0] * 6)
    assert not recorder.add(10.01, [0.1] * 6)
    assert not recorder.add(10.03, [0.001] * 6)
    assert recorder.add(10.04, [0.02] * 6)
    result = recorder.finish(
        10.5, [0.03] * 6, minimum_points=3, minimum_duration_sec=0.2
    )
    assert len(result.points) == 3
    assert result.points[-1].positions == (0.03,) * 6
    assert result.duration == pytest.approx(0.5)


def test_tagged_feedback_records_stationary_frames_without_duplicates(tmp_path):
    recorder = TrajectoryRecorder(
        NAMES, sample_period_sec=0.01, minimum_position_delta=0.0
    )
    recorder.start(10.0, [0.0] * 6, feedback_timestamp=100.0)
    assert not recorder.add(10.01, [0.0] * 6, force=True, feedback_timestamp=100.0)
    assert recorder.add(10.01, [0.0] * 6, force=True, feedback_timestamp=100.01)
    assert recorder.add(10.02, [0.0] * 6, force=True, feedback_timestamp=100.02)
    with pytest.raises(ValueError, match="moved backwards"):
        recorder.add(10.03, [0.0] * 6, force=True, feedback_timestamp=100.015)
    result = recorder.finish(
        10.02, [0.0] * 6, feedback_timestamp=100.02,
        minimum_points=3, minimum_duration_sec=0.01,
    )
    assert len(result.points) == 3
    assert [point.time_from_start for point in result.points] == pytest.approx([0.0, 0.01, 0.02])
    path = tmp_path / "stationary.motion.json"
    save_trajectory(path, result)
    assert load_trajectory(path) == result


def test_grouped_can_sample_metadata_round_trip(tmp_path):
    recorder = TrajectoryRecorder(NAMES, minimum_position_delta=0.0)
    first = {"timestamp_j12": 100.0000, "timestamp_j34": 100.0001,
             "timestamp_j56": 100.0002, "callback_monotonic_time": 20.0,
             "intra_cycle_span": .0002, "timing_valid": True}
    second = {**first, "timestamp_j12": 100.0100, "timestamp_j34": 100.0101,
              "timestamp_j56": 100.0102, "callback_monotonic_time": 20.01}
    recorder.start(100.0002, [0] * 6, feedback_timestamp=100.0002,
                   sample_metadata=first)
    recorder.add(100.0102, [.01] * 6, feedback_timestamp=100.0102,
                 sample_metadata=second, force=True)
    result = recorder.finish(100.0202, [.02] * 6,
                             feedback_timestamp=100.0202,
                             sample_metadata={**second,
                                              "timestamp_j12": 100.0200,
                                              "timestamp_j34": 100.0201,
                                              "timestamp_j56": 100.0202},
                             minimum_points=3, minimum_duration_sec=.01)
    path = tmp_path / "grouped.motion.json"
    save_trajectory(path, result)
    assert load_trajectory(path) == result
    assert result.points[0].intra_cycle_span == pytest.approx(.0002)


def test_atomic_json_round_trip(tmp_path):
    path = tmp_path / "latest.motion.json"
    save_trajectory(path, trajectory())
    loaded = load_trajectory(path)
    assert loaded == trajectory()
    document = json.loads(path.read_text(encoding="utf-8"))
    assert document["format"] == "rebot_teach_trajectory"
    assert document["version"] == 1


def test_optional_feedback_velocity_and_effort_round_trip(tmp_path):
    recorder = TrajectoryRecorder(NAMES, sample_period_sec=0.01, minimum_position_delta=0.0)
    recorder.start(1.0, [0.0] * 6, [0.1] * 6, [0.2] * 6)
    recorder.add(1.02, [0.01] * 6, velocities=[0.3] * 6, efforts=[0.4] * 6)
    result = recorder.finish(
        1.04, [0.02] * 6, velocities=[0.5] * 6, efforts=[0.6] * 6,
        minimum_points=3, minimum_duration_sec=0.01,
    )
    path = tmp_path / "feedback.motion.json"
    save_trajectory(path, result)
    loaded = load_trajectory(path)
    assert loaded == result
    assert loaded.points[-1].velocities == (0.5,) * 6
    assert loaded.points[-1].efforts == (0.6,) * 6


def test_joint_limit_guard_identifies_point_and_joint():
    with pytest.raises(ValueError, match="point 2 joint1"):
        validate_joint_limits(
            trajectory(),
            [-0.1, 0.0, 0.0, -1.0, -1.0, -1.0],
            [0.15, 1.0, 1.0, 1.0, 1.0, 1.0],
        )


def test_forward_replay_starts_from_current_and_respects_velocity():
    source = trajectory()
    plan = build_playback_plan(
        source,
        source.points[0].positions,
        speed_scale=0.5,
        maximum_joint_velocity=0.04,
    )
    assert not plan.includes_reverse_return
    assert plan.points[0].positions == source.points[0].positions
    for first, second in zip(plan.points, plan.points[1:]):
        delta = max(abs(a - b) for a, b in zip(first.positions, second.positions))
        assert delta / (second.time_from_start - first.time_from_start) <= 0.040001


def test_default_replay_matches_recorded_timestamps_at_the_start_pose():
    source = trajectory()
    plan = build_playback_plan(
        source,
        source.points[0].positions,
        maximum_joint_velocity=1.0,
    )
    assert [point.time_from_start for point in plan.points] == pytest.approx(
        [point.time_from_start for point in source.points]
    )
    assert [point.positions for point in plan.points] == [
        point.positions for point in source.points
    ]


def test_double_speed_halves_recorded_timestamps_when_within_safety_limit():
    source = trajectory()
    plan = build_playback_plan(
        source,
        source.points[0].positions,
        speed_scale=2.0,
        maximum_joint_velocity=1.0,
    )
    assert plan.duration == pytest.approx(source.duration / 2.0)


def test_end_pose_retraces_to_start_then_replays_forward():
    source = trajectory()
    plan = build_playback_plan(source, source.points[-1].positions)
    assert plan.includes_reverse_return
    positions = [point.positions for point in plan.points]
    assert source.points[0].positions in positions
    assert positions[-1] == source.points[-1].positions
    start_index = positions.index(source.points[0].positions)
    assert start_index > 0


def test_replay_rejects_unknown_mid_path_pose():
    with pytest.raises(ValueError, match="not near"):
        build_playback_plan(trajectory(), [1.0] * 6, endpoint_tolerance=0.01)


def test_planned_transition_reaches_start_then_only_plays_action_forward():
    source = trajectory()
    current = (1.0,) * 6
    transition = RecordedTrajectory(
        NAMES,
        (
            RecordedPoint(0.0, current),
            RecordedPoint(2.0, (0.5, 0.75, 0.75, 0.5, 0.5, 0.5)),
            RecordedPoint(4.0, source.points[0].positions),
        ),
        source.created_utc,
    )

    plan = build_playback_plan(
        source,
        current,
        transition_trajectory=transition,
        maximum_joint_velocity=1.0,
    )

    positions = [point.positions for point in plan.points]
    assert not plan.includes_reverse_return
    assert positions[:3] == [point.positions for point in transition.points]
    assert positions[3:] == [point.positions for point in source.points[1:]]
    assert positions[-1] == source.points[-1].positions


def test_planned_transition_must_connect_current_pose_and_action_start():
    source = trajectory()
    disconnected_start = RecordedTrajectory(
        NAMES,
        (
            RecordedPoint(0.0, (0.5,) * 6),
            RecordedPoint(1.0, source.points[0].positions),
        ),
        source.created_utc,
    )
    with pytest.raises(ValueError, match="transition start"):
        build_playback_plan(
            source,
            (1.0,) * 6,
            transition_trajectory=disconnected_start,
            endpoint_tolerance=0.01,
        )

    disconnected_end = RecordedTrajectory(
        NAMES,
        (
            RecordedPoint(0.0, (1.0,) * 6),
            RecordedPoint(1.0, (0.5,) * 6),
        ),
        source.created_utc,
    )
    with pytest.raises(ValueError, match="does not reach"):
        build_playback_plan(
            source,
            (1.0,) * 6,
            transition_trajectory=disconnected_end,
            endpoint_tolerance=0.01,
        )


def test_loader_rejects_non_monotonic_time(tmp_path):
    path = tmp_path / "bad.json"
    path.write_text(
        json.dumps(
            {
                "format": "rebot_teach_trajectory",
                "version": 1,
                "joint_names": list(NAMES),
                "points": [
                    {"time_from_start": 1.0, "positions": [0.0] * 6},
                    {"time_from_start": 0.5, "positions": [0.0] * 6},
                ],
            }
        ),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="invalid time"):
        load_trajectory(path)


def test_sample_trajectory_interpolates_normalized_time():
    source = trajectory()
    assert sample_trajectory(source, 0.0) == source.points[0].positions
    assert sample_trajectory(source, 1.0) == source.points[-1].positions
    assert sample_trajectory(source, 0.25) == pytest.approx(
        (0.05, 0.55, 0.55, 0.05, 0.0, 0.0)
    )


def test_preview_loop_seeks_by_elapsed_time_and_preserves_cycle_duration():
    source = RecordedTrajectory(
        NAMES,
        (
            RecordedPoint(0.0, (0.0,) * 6),
            RecordedPoint(1.0, (1.0,) * 6),
            RecordedPoint(4.0, (4.0,) * 6),
            RecordedPoint(10.0, (10.0,) * 6),
        ),
        "2026-08-25T00:00:00+00:00",
    )

    preview = build_preview_loop(source, 0.5)

    assert preview.duration == pytest.approx(source.duration)
    assert preview.points[0].positions == pytest.approx((5.0,) * 6)
    assert preview.points[-1].positions == pytest.approx((5.0,) * 6)
    assert [point.time_from_start for point in preview.points] == pytest.approx(
        [0.0, 5.0, 5.0, 6.0, 9.0, 10.0]
    )


def test_preview_loop_at_zero_keeps_original_timing():
    source = trajectory()
    assert build_preview_loop(source, 0.0) == source


def test_rviz_preview_compresses_idle_gaps_and_uses_uniform_frames():
    source = RecordedTrajectory(
        NAMES,
        (
            RecordedPoint(0.0, (0.0,) * 6),
            RecordedPoint(1.0, (1.0,) * 6),
            RecordedPoint(6.0, (1.001,) * 6),
            RecordedPoint(7.0, (2.0,) * 6),
        ),
        "2026-08-25T00:00:00+00:00",
    )

    preview = prepare_rviz_preview(
        source,
        frame_period_sec=0.1,
        maximum_idle_sec=0.2,
        idle_position_delta=0.002,
    )

    assert preview.duration == pytest.approx(2.2)
    assert preview.points[0].positions == source.points[0].positions
    assert preview.points[-1].positions == source.points[-1].positions
    frame_times = [point.time_from_start for point in preview.points]
    assert max(b - a for a, b in zip(frame_times, frame_times[1:])) <= 0.100001


def test_concatenated_action_preview_preserves_order_and_boundary_jump():
    first = trajectory()
    second = RecordedTrajectory(
        NAMES,
        (
            RecordedPoint(0.0, (1.0,) * 6),
            RecordedPoint(3.0, (2.0,) * 6),
        ),
        first.created_utc,
    )

    combined = concatenate_trajectories([first, second])

    assert combined.duration == pytest.approx(5.0)
    boundary = [
        point.positions
        for point in combined.points
        if point.time_from_start == pytest.approx(2.0)
    ]
    assert boundary == [first.points[-1].positions, second.points[0].positions]
    assert combined.points[-1].positions == second.points[-1].positions


def test_transitioned_action_preview_has_a_continuous_boundary_path():
    first = trajectory()
    second = RecordedTrajectory(
        NAMES,
        (
            RecordedPoint(0.0, (1.0,) * 6),
            RecordedPoint(2.0, (1.2,) * 6),
        ),
        first.created_utc,
    )
    transition = RecordedTrajectory(
        NAMES,
        (
            RecordedPoint(0.0, first.points[-1].positions),
            RecordedPoint(1.0, (0.6,) * 6),
            RecordedPoint(2.0, second.points[0].positions),
        ),
        first.created_utc,
    )

    combined = concatenate_trajectories_with_transitions(
        (first, second), (transition,), maximum_joint_velocity=1.0
    )

    assert combined.duration == pytest.approx(6.0)
    assert [point.positions for point in combined.points] == [
        *(point.positions for point in first.points),
        transition.points[1].positions,
        transition.points[2].positions,
        second.points[1].positions,
    ]
    assert all(
        second.time_from_start > first.time_from_start
        for first, second in zip(combined.points, combined.points[1:])
    )


@pytest.mark.parametrize("progress", [-0.01, 1.01, float("nan")])
def test_sample_trajectory_rejects_invalid_progress(progress):
    with pytest.raises(ValueError, match="progress"):
        sample_trajectory(trajectory(), progress)
