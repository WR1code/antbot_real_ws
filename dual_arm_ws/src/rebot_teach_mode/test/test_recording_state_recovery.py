from pathlib import Path
import threading
from types import SimpleNamespace

import pytest

from rebot_teach_mode import teach_node as teach_node_module
from rebot_teach_mode.teach_node import TeachModeNode


class _Library:
    def __init__(self, root: Path) -> None:
        self._root = root

    def path_for(self, name: str) -> Path:
        return self._root / f"{name}.json"


class _Recorder:
    def __init__(self) -> None:
        self.started = None

    def start(self, now, positions, **_kwargs) -> None:
        self.started = (now, positions)


class _TrackingLock:
    def __init__(self) -> None:
        self.entered = False

    def __enter__(self):
        self.entered = True
        return self

    def __exit__(self, _exc_type, _exc_value, _traceback):
        self.entered = False


class _SampleRecorder:
    def __init__(self) -> None:
        self.sample = None

    def add(self, now, positions, **_kwargs) -> None:
        self.sample = (now, positions)
        return True


class _Logger:
    def __init__(self) -> None:
        self.warnings = []
        self.errors = []

    def warning(self, message) -> None:
        self.warnings.append(message)

    def error(self, message) -> None:
        self.errors.append(message)


class _DiscardingRecorder:
    def __init__(self) -> None:
        self.discarded = False

    def finish(self, *_args, **_kwargs):
        raise ValueError("recording needs at least 5 points")

    def discard(self) -> None:
        self.discarded = True


class _ImmediateFuture:
    def __init__(self, result) -> None:
        self._result = result

    def add_done_callback(self, callback) -> None:
        callback(self)

    def result(self):
        return self._result


class _ActionClient:
    def __init__(self, lock) -> None:
        self._lock = lock

    def wait_for_server(self, timeout_sec) -> bool:
        assert timeout_sec == 2.0
        assert not self._lock._is_owned()
        return True

    def send_goal_async(self, _goal, feedback_callback):
        assert not self._lock._is_owned()
        assert feedback_callback is not None
        return _ImmediateFuture(SimpleNamespace(accepted=True))


def _node(tmp_path: Path, driver_state: str):
    node = TeachModeNode.__new__(TeachModeNode)
    node._lock = threading.RLock()
    node._state = "IDLE"
    node._robot_model = "rebotarm_rs"
    node._teach_drag_mode = "passive_disabled"
    node._action_library = _Library(tmp_path)
    node._latest_arm_status = SimpleNamespace(
        state_machine=driver_state, enabled=driver_state != "DISABLED", error_codes=[]
    )
    node._allow_hardware = True
    node._latest_status_time = 12.5
    node._trajectory = None
    node._feedback_timeout = 0.3
    node._status_timeout = 0.5
    node._watchdog_suppressed_until = 0.0
    node._preflight = lambda *, expected_driver_states, require_enabled=True: (
        (0.0,) * 6
        if node._latest_arm_status.state_machine in expected_driver_states
        else (_ for _ in ()).throw(RuntimeError("unexpected driver state"))
    )
    node._now = lambda: 12.5
    node._recorder = _Recorder()
    node._pending_action_name = ""
    node._pending_action_description = ""
    node._pending_action_overwrite = False
    node._publish_status = lambda: None
    node.gravity_start_calls = 0

    def call_trigger(_client, _label):
        assert not node._lock._is_owned()
        node.gravity_start_calls += 1
        node._latest_arm_status.state_machine = "GRAVITY_COMP"

    node._call_trigger = call_trigger
    node._gravity_start = object()
    return node


def test_joint_limit_violations_report_each_invalid_current_joint():
    node = TeachModeNode.__new__(TeachModeNode)
    node._joint_names = tuple(f"joint{index}" for index in range(1, 7))
    node._lower_limits = (-2.618, 0.0, -2.96706, -2.356195, -1.56207, -3.14)
    node._upper_limits = (2.618, 3.14, 0.0, 2.356195, 1.56207, 3.14)
    node._joint_limit_margin = 0.0

    violations = node._joint_limit_violations(
        (-0.03, 2.94, -3.0, 2.4, -0.02, 2.19)
    )

    assert len(violations) == 2
    assert violations[0].startswith("joint3=-3.000000")
    assert violations[1].startswith("joint4=2.400000")


def test_recording_adopts_existing_gravity_compensation(tmp_path):
    node = _node(tmp_path, "GRAVITY_COMP")

    with node._lock:
        node._begin_recording_locked("demo", "drag", overwrite=False)

    assert node.gravity_start_calls == 0
    assert node._recorder.started == (12.5, (0.0,) * 6)
    assert node._state == "RECORDING"


def test_recording_starts_gravity_compensation_from_idle(tmp_path):
    node = _node(tmp_path, "IDLE")

    with node._lock:
        node._begin_recording_locked("demo", "drag", overwrite=False)

    assert node.gravity_start_calls == 1
    assert node._watchdog_suppressed_until == 13.5
    assert node._state == "RECORDING"


def test_standalone_gravity_mode_can_be_adopted_by_recording(tmp_path):
    node = _node(tmp_path, "IDLE")
    node._supports_gravity_compensation = True
    node._call_trigger_while_state_unlocked = lambda _client, _label: setattr(
        node._latest_arm_status, "state_machine", "GRAVITY_COMP"
    )
    node._wait_for_preflight_while_state_unlocked = lambda states: (
        (0.0,) * 6 if node._latest_arm_status.state_machine in states else
        (_ for _ in ()).throw(RuntimeError("wrong driver state"))
    )
    response = SimpleNamespace(success=False, message="")
    node._start_gravity_mode(None, response)
    assert response.success
    assert node._state == "IDLE"
    with node._lock:
        node._begin_recording_locked("standalone", "drag", overwrite=False)
    assert node.gravity_start_calls == 0
    assert node._state == "RECORDING"
    node._gravity_stop = object()
    node._stop_gravity_mode(None, response)
    assert not response.success
    assert "RECORDING" in response.message


def test_uncalibrated_gravity_start_can_be_retried_after_driver_stays_idle(tmp_path):
    node = _node(tmp_path, "IDLE")
    node._call_trigger_while_state_unlocked = lambda _client, _label: (
        (_ for _ in ()).throw(RuntimeError("MIT torque calibration is not confirmed"))
    )
    with node._lock, pytest.raises(RuntimeError, match="calibration is not confirmed"):
        node._begin_recording_locked("retry", "drag", overwrite=False)
    assert node._state == "IDLE"


def test_joint_sample_timestamp_is_taken_after_state_lock():
    node = TeachModeNode.__new__(TeachModeNode)
    node._lock = _TrackingLock()
    node._robot_model = "rebotarm_rs"
    node._joint_names = tuple(f"joint{index}" for index in range(1, 7))
    node._state = "RECORDING"
    node._recorder = _SampleRecorder()
    node._preview_active = False
    node._preview_hold_pending = False

    def now():
        assert node._lock.entered
        return 42.0

    node._now = now
    message = SimpleNamespace(
        name=list(node._joint_names),
        position=[float(index) for index in range(6)],
    )

    node._joint_state_callback(message)

    assert node._latest_joint_time == 42.0
    assert node._recorder.sample == (
        42.0,
        tuple(float(index) for index in range(6)),
    )


def test_second_stop_preserves_original_fault_message():
    node = TeachModeNode.__new__(TeachModeNode)
    node._lock = threading.RLock()
    node._state = "FAULT"
    node._message = "original hardware fault"
    node._publish_status = lambda: None
    response = SimpleNamespace(success=True, message="")

    node._stop_recording(None, response)

    assert not response.success
    assert response.message == "already in FAULT: original hardware fault"
    assert node._message == "original hardware fault"


def test_fault_reset_clears_failed_replay_progress():
    node = TeachModeNode.__new__(TeachModeNode)
    node._lock = threading.RLock()
    node._state = "FAULT"
    node._trajectory = object()
    node._message = "replay failed"
    node._replay_progress = 1.0
    node._replay_plan_duration_sec = 8.0
    node._replay_started_time = 10.0
    node._replay_sequence_name = "demo_sequence"
    node._replay_action_index = 2
    node._replay_total_actions = 2
    node._preflight = lambda *, expected_driver_states: (0.0,) * 6
    node._publish_status = lambda: None
    response = SimpleNamespace(success=False, message="")

    node._reset(None, response)

    assert response.success
    assert node._state == "READY"
    assert node._replay_progress == 0.0
    assert node._replay_plan_duration_sec == 0.0
    assert node._replay_started_time == 0.0
    assert node._replay_sequence_name == ""
    assert node._replay_action_index == 0
    assert node._replay_total_actions == 0


def test_short_recording_stops_gravity_for_rs():
    node = TeachModeNode.__new__(TeachModeNode)
    node._lock = threading.RLock()
    node._state = "RECORDING"
    node._robot_model = "rebotarm_rs"
    node._trajectory = None
    node._message = "recording"
    node._recorder = _DiscardingRecorder()
    node._pending_action_name = "demo"
    node._pending_action_description = ""
    node._pending_action_overwrite = False
    node._gravity_stop = object()
    node._latest_arm_status = SimpleNamespace(
        state_machine="GRAVITY_COMP", enabled=True, error_codes=[]
    )
    node._latest_status_time = 10.0
    node._status_timeout = 0.5
    node._require_xbox_locked = True
    node._xbox_armed = False
    node._joint_limit_violations = lambda _positions: ()
    node._feedback_timeout = 0.3
    node._preflight = lambda *, expected_driver_states: (0.0,) * 6
    node._call_trigger = lambda _client, _label: None
    wait_calls = []
    node._wait_for_preflight_while_state_unlocked = lambda states, **kwargs: (
        wait_calls.append((states, kwargs)) or (0.0,) * 6
    )
    node._now = lambda: 10.0
    node.get_parameter = lambda name: SimpleNamespace(
        value=5 if name == "minimum_recording_points" else 0.5
    )
    node._publish_status = lambda: None
    logger = _Logger()
    node.get_logger = lambda: logger
    response = SimpleNamespace(success=True, message="")

    node._stop_recording(None, response)

    assert not response.success
    assert node._state == "IDLE"
    assert node._recorder.discarded
    assert "recording needs at least 5 points" in response.message
    assert logger.warnings
    assert wait_calls == [({"IDLE"}, {})]


def test_piper_passive_recording_starts_without_motor_or_gravity_command(tmp_path):
    node = _node(tmp_path, "DISABLED")
    node._robot_model = "piperh"
    node._latest_arm_status.enabled = False
    node._latest_joint_time = 12.5
    node._latest_joint_feedback_stamp = 100.0
    node._latest_positions = (0.2,) * 6
    node._latest_teach_positions = (0.2,) * 6
    node._latest_teach_sample_stamp = 100.0
    node._latest_teach_arrival_time = 12.5
    node._latest_teach_metadata = {
        "timestamp_j12": 99.9998, "timestamp_j34": 99.9999,
        "timestamp_j56": 100.0, "callback_monotonic_time": 12.5,
        "intra_cycle_span": .0002, "timing_valid": True,
    }
    node._latest_teach_dropped_cycles = 0
    node._latest_teach_wide_span_cycles = 0
    node._preflight = lambda *, expected_driver_states, require_enabled: (
        node._latest_positions
        if expected_driver_states == {"DISABLED"} and not require_enabled
        else (_ for _ in ()).throw(RuntimeError("unexpected passive preflight"))
    )
    node._joint_limit_violations = lambda _positions: ()
    node._require_xbox_locked = True
    node._xbox_armed = False
    node._recording_dropped_gaps = 0
    node._recording_timed_out = False
    node._recorder.sample_period_sec = 0.02

    with node._lock:
        node._begin_recording_locked("passive", "manual drag", overwrite=False)

    assert node.gravity_start_calls == 0
    assert node._recording_source == "normal"
    assert node._recorder.started == (100.0, (0.2,) * 6)
    assert node._state == "RECORDING"


def test_piper_passive_recording_rejects_enabled_driver(tmp_path):
    node = _node(tmp_path, "DISABLED")
    node._robot_model = "piperh"
    node._latest_arm_status.enabled = True
    node._require_xbox_locked = True
    node._xbox_armed = False
    with node._lock, pytest.raises(RuntimeError, match="must be disabled"):
        node._begin_recording_locked("passive", "manual drag", overwrite=False)
    assert node.gravity_start_calls == 0


def test_piper_passive_records_grouped_can_feedback_event():
    node = TeachModeNode.__new__(TeachModeNode)
    node._lock = threading.RLock()
    node._joint_names = tuple(f"joint{index}" for index in range(1, 7))
    node._robot_model = "piperh"
    node._state = "RECORDING"
    node._recording_source = "normal"
    node._recording_passive_frames = 1
    node._recording_passive_first_time = 10.0
    node._recording_passive_last_time = 10.0
    node._latest_joint_feedback_stamp = 100.0
    node._recording_duplicate_feedback = 0
    node._recording_sample_ticks = 0
    node._recording_dropped_gaps = 0
    node._recording_gap_counts = {"30ms": 0, "50ms": 0, "100ms": 0}
    node._recording_timestamp_backwards = 0
    node._recording_timing_invalid = False
    node._feedback_timeout = 0.3
    node._recorder = _SampleRecorder()
    node._recorder.sample_period_sec = 0.02
    node._preview_active = False
    node._preview_hold_pending = False
    node._now = lambda: 10.02
    node._piper_recording_preflight = lambda: node._latest_teach_positions
    stamp = lambda sec, ns: SimpleNamespace(sec=sec, nanosec=ns)
    message = SimpleNamespace(
        sample_timestamp=stamp(100, 10_000_000),
        timestamp_j12=stamp(100, 9_800_000),
        timestamp_j34=stamp(100, 9_900_000),
        timestamp_j56=stamp(100, 10_000_000),
        callback_monotonic_time=10.019,
        intra_cycle_span=.0002,
        dropped_cycles=0,
        wide_span_cycles=0,
        timing_valid=True,
        name=list(node._joint_names),
        position=[float(index) for index in range(6)],
    )

    node._latest_teach_sample_stamp = 100.0
    node._teach_joint_state_callback(message)

    assert node._recorder.sample == (100.01, tuple(float(index) for index in range(6)))
    assert node._recording_passive_frames == 2
    assert node._recording_sample_ticks == 0


def test_short_piper_passive_recording_never_switches_gravity_mode():
    node = TeachModeNode.__new__(TeachModeNode)
    node._lock = threading.RLock()
    node._state = "RECORDING"
    node._robot_model = "piperh"
    node._recording_source = "normal"
    node._teach_drag_mode = "passive_disabled"
    node._trajectory = None
    node._recorder = _DiscardingRecorder()
    node._latest_arm_status = SimpleNamespace(state_machine="DISABLED", enabled=False)
    node._piper_recording_preflight = lambda: (0.0,) * 6
    node._call_trigger_while_state_unlocked = lambda *_args: pytest.fail(
        "passive recording must not call gravity service"
    )
    node._pending_action_name = "short"
    node._pending_action_description = ""
    node._pending_action_overwrite = False
    node._now = lambda: 10.0
    node.get_parameter = lambda name: SimpleNamespace(
        value=5 if name == "minimum_recording_points" else 0.5
    )
    node._publish_status = lambda: None
    logger = _Logger()
    node.get_logger = lambda: logger
    response = SimpleNamespace(success=True, message="")

    node._stop_recording(None, response)

    assert not response.success
    assert node._state == "IDLE"
    assert node._recorder.discarded


def test_cancel_stops_rviz_preview_without_hardware_command():
    node = TeachModeNode.__new__(TeachModeNode)
    node._lock = threading.RLock()
    node._state = "READY"
    node._preview_active = True
    node._preview_paused = False
    node._latest_positions = (0.1,) * 6
    node._trajectory = None
    node._message = "previewing"
    node._publish_status = lambda: None
    published = []

    def publish_pose(positions):
        published.append(positions)
        node._preview_active = False

    node._publish_preview_pose = publish_pose
    response = SimpleNamespace(success=False, message="")

    node._cancel(None, response)

    assert response.success
    assert published == [(0.1,) * 6]
    assert not node._preview_active
    assert "hardware unchanged" in response.message


def test_cancel_can_be_queued_while_trajectory_goal_is_starting():
    node = TeachModeNode.__new__(TeachModeNode)
    node._lock = threading.RLock()
    node._state = "REPLAY_STARTING"
    node._active_goal = None
    node._cancel_requested = False
    node._preview_active = False
    node._message = "starting"
    node._publish_status = lambda: None
    response = SimpleNamespace(success=False, message="")

    node._cancel(None, response)

    assert response.success
    assert node._cancel_requested
    assert node._state == "CANCELLING"
    assert "queued" in response.message


def test_cancel_during_sequence_planning_never_commands_hardware():
    node = TeachModeNode.__new__(TeachModeNode)
    node._lock = threading.RLock()
    node._state = "SEQUENCE_PLANNING"
    node._allow_hardware = True
    node._trajectory = object()
    node._cancel_requested = False
    node._preview_active = False
    node._message = "planning"
    node._publish_status = lambda: None
    node._clear_sequence_locked = lambda: None
    response = SimpleNamespace(success=False, message="")

    node._cancel(None, response)

    assert response.success
    assert node._cancel_requested
    assert node._state == "READY"
    assert "hardware unchanged" in response.message


def test_delete_selected_action_clears_loaded_trajectory(tmp_path):
    node = TeachModeNode.__new__(TeachModeNode)
    node._lock = threading.RLock()
    node._state = "READY"
    node._allow_hardware = True
    node._selected_action_name = "旧动作"
    node._trajectory = SimpleNamespace(
        points=[SimpleNamespace(positions=(0.2,) * 6)]
    )
    node._latest_positions = (0.1,) * 6
    deleted = SimpleNamespace(path=tmp_path / "旧动作.teach.json")
    node._action_library = SimpleNamespace(delete=lambda name: deleted)
    node._publish_status = lambda: None
    published_poses = []
    node._publish_preview_pose = published_poses.append
    response = SimpleNamespace(success=False, message="", path="")

    node._delete_action_group(SimpleNamespace(name="旧动作"), response)

    assert response.success
    assert response.path == str(deleted.path)
    assert node._selected_action_name == ""
    assert node._trajectory is None
    assert node._state == "IDLE"
    assert published_poses == [(0.1,) * 6]


def test_clear_action_selection_stops_preview_and_discards_old_selection():
    node = TeachModeNode.__new__(TeachModeNode)
    node._lock = threading.RLock()
    node._state = "READY"
    node._allow_hardware = True
    node._selected_action_name = "旧动作"
    node._trajectory = SimpleNamespace(
        points=[SimpleNamespace(positions=(0.2,) * 6)]
    )
    node._latest_positions = (0.1,) * 6
    node._publish_status = lambda: None
    node._clear_sequence_locked = lambda: None
    published_poses = []
    node._publish_preview_pose = published_poses.append
    response = SimpleNamespace(success=False, message="")

    node._clear_action_selection(None, response)

    assert response.success
    assert node._selected_action_name == ""
    assert node._trajectory is None
    assert node._state == "IDLE"
    assert published_poses == [(0.1,) * 6]


def test_startup_recovery_loads_trajectory_without_starting_preview(
    tmp_path, monkeypatch
):
    path = tmp_path / "latest.motion.json"
    path.write_text("{}", encoding="utf-8")
    trajectory = SimpleNamespace(
        points=[SimpleNamespace(positions=(0.2,) * 6)] * 3
    )
    monkeypatch.setattr(
        teach_node_module, "load_trajectory", lambda _path: trajectory
    )
    node = TeachModeNode.__new__(TeachModeNode)
    node._trajectory_path = path
    node._allow_hardware = True
    node._state = "IDLE"
    node._validate_trajectory = lambda _trajectory: None
    node._publish_recorded_preview = lambda _trajectory: (_ for _ in ()).throw(
        AssertionError("startup must not begin a loop preview")
    )
    published_poses = []
    node._publish_preview_pose = published_poses.append

    node._try_load_existing()

    assert node._trajectory is trajectory
    assert node._selected_action_name == ""
    assert node._state == "READY"
    assert published_poses == [(0.2,) * 6]
    assert "preview is stopped" in node._message


def test_invalid_startup_trajectory_does_not_block_new_recording(
    tmp_path, monkeypatch
):
    path = tmp_path / "latest.motion.json"
    path.write_text("{}", encoding="utf-8")
    monkeypatch.setattr(
        teach_node_module, "load_trajectory", lambda _path: object()
    )
    node = TeachModeNode.__new__(TeachModeNode)
    node._trajectory_path = path
    node._allow_hardware = True
    node._state = "IDLE"
    node._trajectory = object()
    node._selected_action_name = "old"
    node._validate_trajectory = lambda _trajectory: (_ for _ in ()).throw(
        ValueError("joint_limits_valid=false")
    )

    node._try_load_existing()

    assert node._trajectory is None
    assert node._selected_action_name == ""
    assert node._state == "IDLE"
    assert "new recording remains available" in node._message


def test_rviz_preview_frames_follow_the_preview_clock():
    from rebot_teach_mode.trajectory import RecordedPoint, RecordedTrajectory

    node = TeachModeNode.__new__(TeachModeNode)
    node._lock = threading.RLock()
    node._joint_names = tuple(f"joint{index}" for index in range(1, 7))
    node._preview_active = True
    node._preview_paused = False
    node._preview_duration_sec = 2.0
    node._preview_progress_offset = 0.25
    node._preview_started_time = 10.0
    node._now = lambda: 10.5
    node._rviz_preview_trajectory = RecordedTrajectory(
        node._joint_names,
        (
            RecordedPoint(0.0, (0.0,) * 6),
            RecordedPoint(2.0, (2.0,) * 6),
        ),
        "2026-08-30T00:00:00+00:00",
    )
    published = []
    published_trace_progress = []
    node._publish_display_robot_state = published.append
    node._publish_tcp_trace = published_trace_progress.append
    node._tcp_trace_progress = 0.0
    node._last_preview_trace_publish_time = 0.0

    node._advance_rviz_preview()

    assert published == [(1.0,) * 6]
    assert published_trace_progress == [pytest.approx(0.5)]


def test_hardware_replay_progress_uses_controller_desired_time():
    node = TeachModeNode.__new__(TeachModeNode)
    node._lock = threading.RLock()
    node._state = "REPLAYING"
    node._joint_names = tuple(f"joint{index}" for index in range(1, 7))
    node._replay_plan_duration_sec = 4.0
    node._replay_started_time = 9.0
    node._replay_progress = 0.0
    node._maximum_tracking_error = 0.25
    node._tracking_error_limit = 5
    node._tracking_error_count = 0
    node._last_replay_progress_publish = 0.0
    node._now = lambda: 10.0
    published = []
    node._publish_status = lambda: published.append(node._replay_progress)
    point = SimpleNamespace(
        positions=(0.0,) * 6,
        time_from_start=SimpleNamespace(sec=2, nanosec=0),
    )
    feedback_message = SimpleNamespace(
        feedback=SimpleNamespace(
            desired=point,
            actual=SimpleNamespace(positions=(0.0,) * 6),
        )
    )

    node._trajectory_feedback(feedback_message)

    assert node._replay_progress == 0.5
    assert published == [0.5]


def test_hardware_replay_progress_falls_back_when_driver_time_is_zero():
    node = TeachModeNode.__new__(TeachModeNode)
    node._lock = threading.RLock()
    node._state = "REPLAYING"
    node._joint_names = tuple(f"joint{index}" for index in range(1, 7))
    node._replay_plan_duration_sec = 4.0
    node._replay_started_time = 10.0
    node._replay_progress = 0.0
    node._maximum_tracking_error = 0.25
    node._tracking_error_limit = 5
    node._tracking_error_count = 0
    node._last_replay_progress_publish = 0.0
    node._now = lambda: 11.0
    node._publish_status = lambda: None
    feedback_message = SimpleNamespace(
        feedback=SimpleNamespace(
            desired=SimpleNamespace(
                positions=(0.0,) * 6,
                time_from_start=SimpleNamespace(sec=0, nanosec=0),
            ),
            actual=SimpleNamespace(positions=(0.0,) * 6),
        )
    )

    node._trajectory_feedback(feedback_message)

    assert node._replay_progress == 0.25


def test_sequence_playback_hides_single_action_shape_visualization():
    node = TeachModeNode.__new__(TeachModeNode)
    node._shape_marker_visible = True
    calls = []
    node._refresh_shape_marker_locked = lambda: calls.append("marker")
    node._publish_drawing_path = lambda: calls.append("drawing_path")

    node._hide_shape_visualization_locked()

    assert not node._shape_marker_visible
    assert calls == ["marker", "drawing_path"]


def test_robot_sequence_replay_uses_only_requested_actions_in_order():
    class Library:
        def load(self, name):
            return SimpleNamespace(name=name), SimpleNamespace(name=name)

        def load_sequence(self, _name):
            raise AssertionError("saved full sequence must not be loaded")

    node = TeachModeNode.__new__(TeachModeNode)
    node._lock = threading.RLock()
    node._state = "IDLE"
    node._action_library = Library()
    node._validate_trajectory = lambda _trajectory: None
    node._hide_shape_visualization_locked = lambda: None
    node._message = ""
    started = []
    planned = []
    transition = object()
    node._plan_sequence_transitions = lambda names, trajectories: (
        planned.append((tuple(names), tuple(item.name for item in trajectories)))
        or [transition]
    )
    node._preflight = lambda **_kwargs: (0.0,) * 6
    node._plan_sequence_entry_guarded = (
        lambda name, trajectory, current: planned.append(
            ("entry", name, trajectory.name, current)
        ) or None
    )
    node._begin_replay_locked = lambda speed: started.append(speed)
    node._publish_status = lambda: None
    request = SimpleNamespace(
        name="", speed_scale=0.7, action_names=["second", "fourth"]
    )
    response = SimpleNamespace(success=False, message="")

    result = node._replay_action_sequence(request, response)

    assert result.success
    assert node._selected_action_name == "second"
    assert node._active_sequence_name == "current editor"
    assert node._sequence_queue == ["fourth"]
    assert node._sequence_transitions == [transition]
    assert node._sequence_action_index == 1
    assert node._sequence_total_actions == 2
    assert planned == [
        ("entry", "second", "second", (0.0,) * 6),
        (("second", "fourth"), ("second", "fourth")),
    ]
    assert started == [0.7]


def test_sequence_transitions_are_preplanned_only_for_disconnected_boundaries():
    from rebot_teach_mode.trajectory import RecordedPoint, RecordedTrajectory

    names = tuple(f"joint{index}" for index in range(1, 7))

    def action(start, end):
        return RecordedTrajectory(
            names,
            (RecordedPoint(0.0, start), RecordedPoint(1.0, end)),
            "",
        )

    first = action((0.0,) * 6, (0.2,) * 6)
    second = action((0.2,) * 6, (0.4,) * 6)
    third = action((0.8,) * 6, (1.0,) * 6)
    transition = action((0.4,) * 6, (0.8,) * 6)
    node = TeachModeNode.__new__(TeachModeNode)
    node._cancel_requested = False
    node.get_parameter = lambda name: SimpleNamespace(
        value={
            "sequence_transition_direct_tolerance_rad": 0.01,
            "sequence_transition_planning_time_sec": 5.0,
            "sequence_transition_service_timeout_sec": 15.0,
            "sequence_transition_velocity_scaling_factor": 0.1,
        }[name]
    )
    node._publish_status = lambda: None
    planned = []

    def plan(start, goal, timeout, **kwargs):
        planned.append((start, goal, timeout, kwargs))
        return object()

    node._plan_joint_segment = plan
    node._recorded_from_joint_trajectory = lambda _trajectory, _label: transition

    result = node._plan_sequence_transitions(
        ("first", "second", "third"), (first, second, third)
    )

    assert result == [None, transition]
    assert len(planned) == 1
    assert planned[0][0] == second.points[-1].positions
    assert planned[0][1] == third.points[0].positions
    assert planned[0][2] == 15.0
    assert planned[0][3]["planning_time_sec"] == 5.0
    assert planned[0][3]["velocity_scaling_factor"] == 0.1


def test_sequence_entry_is_collision_planned_from_live_pose():
    from rebot_teach_mode.trajectory import RecordedPoint, RecordedTrajectory

    names = tuple(f"joint{index}" for index in range(1, 7))
    current = (0.0,) * 6
    goal = (0.3,) * 6
    action = RecordedTrajectory(
        names,
        (RecordedPoint(0.0, goal), RecordedPoint(3.0, goal)),
        "",
    )
    entry = RecordedTrajectory(
        names,
        (RecordedPoint(0.0, current), RecordedPoint(2.0, goal)),
        "",
    )
    node = TeachModeNode.__new__(TeachModeNode)
    node.get_parameter = lambda name: SimpleNamespace(
        value={
            "sequence_transition_direct_tolerance_rad": 0.01,
            "sequence_transition_planning_time_sec": 5.0,
            "sequence_transition_service_timeout_sec": 15.0,
            "sequence_transition_velocity_scaling_factor": 0.1,
        }[name]
    )
    node._publish_status = lambda: None
    planned = []

    def plan(start, target, timeout, **kwargs):
        planned.append((start, target, timeout, kwargs))
        return object()

    node._plan_joint_segment = plan
    node._recorded_from_joint_trajectory = lambda _trajectory, _label: entry

    result = node._plan_sequence_entry("pose", action, current)

    assert result == entry
    assert planned[0][0] == current
    assert planned[0][1] == goal
    assert planned[0][2] == 15.0
    assert planned[0][3]["planning_time_sec"] == 5.0
    assert planned[0][3]["velocity_scaling_factor"] == 0.1


def test_sequence_entry_is_skipped_when_live_pose_matches_target():
    from rebot_teach_mode.trajectory import RecordedPoint, RecordedTrajectory

    names = tuple(f"joint{index}" for index in range(1, 7))
    goal = (0.3,) * 6
    action = RecordedTrajectory(
        names,
        (RecordedPoint(0.0, goal), RecordedPoint(3.0, goal)),
        "",
    )
    node = TeachModeNode.__new__(TeachModeNode)
    node.get_parameter = lambda _name: SimpleNamespace(value=0.01)
    node._plan_joint_segment = lambda *_args, **_kwargs: pytest.fail(
        "matching entry must not invoke MoveIt"
    )

    assert node._plan_sequence_entry("pose", action, goal) is None


def test_trajectory_goal_response_is_waited_without_state_lock():
    node = TeachModeNode.__new__(TeachModeNode)
    node._lock = threading.RLock()
    node._trajectory_action = _ActionClient(node._lock)
    node._trajectory_feedback = lambda _message: None

    with node._lock:
        goal_handle = node._send_trajectory_goal_while_state_unlocked(object())
        assert node._lock._is_owned()

    assert goal_handle.accepted
