from geometry_msgs.msg import PointStamped
from types import SimpleNamespace
from tf2_ros import TransformException

from rebotarm_pulse.piper_pulse_align import PiperPulseAlign, PlanningState
import rebotarm_pulse.piper_pulse_align as pulse_align_module


def test_link6_hover_searches_equivalent_roll_until_ik_passes():
    operation = object.__new__(PiperPulseAlign)
    operation._context = {}
    operation._param = lambda name: {
        "link6_hover_roll_search_deg": [0.0, 30.0, -30.0],
    }[name]
    operation._pose = lambda position, orientation: SimpleNamespace(
        position=position, orientation=orientation, pose=object()
    )
    calls = []

    def valid_ik(pose, _seed, label, warn_only=False):
        calls.append((pose.orientation, label, warn_only))
        operation._context["last_ik_reason"] = "IK_FAILED"
        operation._context["last_ik_diagnostic"] = {"label": label}
        if len(calls) == 2:
            operation._context["last_ik_reason"] = "NONE"
            return [0.1] * 6
        return None

    operation._valid_ik = valid_ik
    operation.node = SimpleNamespace(
        get_logger=lambda: SimpleNamespace(info=lambda _message: None)
    )

    pose, joints, orientation = operation._select_hover_ik(
        (0.3, -0.5, 0.8), (0.0, 0.0, 0.0, 1.0), [0.0] * 6, True
    )

    assert pose.orientation == orientation
    assert joints == [0.1] * 6
    assert len(calls) == 2
    assert operation._context["hover_roll_deg"] == 30.0
    assert [item["result"] for item in operation._context["hover_ik_attempts"]] == [
        "FAIL", "PASS",
    ]


def test_target_already_in_planning_frame_is_preserved():
    operation = object.__new__(PiperPulseAlign)
    target = PointStamped()
    target.header.frame_id = "piperh_planning_world"
    target.point.x, target.point.y, target.point.z = 0.1, -0.2, 0.4
    operation._latest_target = target
    operation._param = lambda name: {
        "planning_frame": "piperh_planning_world",
        "tf_timeout_sec": 0.1,
    }[name]
    assert operation._target_in_planning_frame() is target


def test_tool_tf_lookup_spins_subscriptions_until_transform_arrives(monkeypatch):
    operation = object.__new__(PiperPulseAlign)
    operation._param = lambda name: {
        "planning_frame": "piperh_planning_world",
        "tool_frame": "piperh/Link6",
        "tf_timeout_sec": 8.0,
    }[name]
    operation._context = {}
    attempts = []
    spins = []
    transform = SimpleNamespace(transform=SimpleNamespace(
        translation=SimpleNamespace(x=1.0, y=2.0, z=3.0),
        rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
    ))

    def lookup(target, source, _stamp, timeout):
        attempts.append((target, source, timeout.nanoseconds))
        if len(attempts) == 1:
            raise TransformException("not discovered yet")
        return transform

    operation._tf_buffer = SimpleNamespace(lookup_transform=lookup)
    operation._spin_inputs = lambda duration: spins.append(duration)
    monkeypatch.setattr(pulse_align_module.rclpy, "ok", lambda: True)

    assert operation._current_tool_transform() is transform
    assert len(spins) == 1
    assert all(item == ("piperh_planning_world", "piperh/Link6", 0) for item in attempts)
    assert operation._context["Link6_tf"]["translation"] == (1.0, 2.0, 3.0)


def test_surface_normal_wait_drains_callbacks_until_window_is_stable(monkeypatch):
    operation = object.__new__(PiperPulseAlign)
    operation._param = lambda name: {"target_timeout_sec": 30.0}[name]
    operation._attempt_invalidated = False
    results = iter([
        (None, "surface normal is unstable: jitter=0.00deg samples=1"),
        (None, "surface normal is unstable: jitter=0.10deg samples=2"),
        ((0.0, 0.0, 1.0), "surface normal is ready"),
    ])
    spins = []
    operation._surface_normal_in_planning_frame = lambda: next(results)
    operation._spin_inputs = lambda duration: spins.append(duration)
    monkeypatch.setattr(pulse_align_module.rclpy, "ok", lambda: True)
    monkeypatch.setattr(pulse_align_module.time, "monotonic", lambda: 10.0)

    normal, reason = operation._wait_for_surface_normal_in_planning_frame()

    assert normal == (0.0, 0.0, 1.0)
    assert reason == "surface normal is ready"
    assert spins == [0.05, 0.05]


def test_pressure_graph_diagnostics_reject_duplicate_status_or_data(monkeypatch):
    operation = object.__new__(PiperPulseAlign)
    operation._param = lambda name: "/dev/tty-test" if name == "pressure_serial_device" else None
    monkeypatch.setattr(pulse_align_module, "serial_owner_pids", lambda _device: [1234])
    operation.node = SimpleNamespace(
        count_publishers=lambda topic: 2 if topic.endswith("serial_connected") else 1
    )
    status = operation._pressure_graph_snapshot()
    assert status["pressure_status_publisher_count"] == 2
    assert status["pressure_data_publisher_count"] == 1
    assert status["serial_owner_pid"] == 1234
    assert not status["pressure_source_unique"]

    operation.node = SimpleNamespace(count_publishers=lambda _topic: 1)
    assert operation._pressure_graph_snapshot()["pressure_source_unique"]


def test_live_execution_monitor_prioritizes_pressure_abort():
    operation = object.__new__(PiperPulseAlign)
    operation._xbox_known = True
    operation._xbox_locked = True
    operation._selected_robot = "piperh"
    operation._serial_connected = True
    operation._zero_calibrated = True
    operation._local_pressure_baseline = {"s1": 100.0, "s2": 100.0, "s3": 100.0}
    operation._pressure_received = {"s1": 9.9, "s2": 9.9, "s3": 9.9}
    operation._param = lambda name: {
        "maximum_pressure_age_sec": 0.25, "dummy_contact_mode": False,
        "link6_hover_mode": False,
    }[name]
    operation._pressure_filters = {
        "s1": SimpleNamespace(status=lambda _now: SimpleNamespace(abort=False, emergency_abort=False)),
        "s2": SimpleNamespace(status=lambda _now: SimpleNamespace(abort=True, emergency_abort=False)),
        "s3": SimpleNamespace(status=lambda _now: SimpleNamespace(abort=False, emergency_abort=False)),
    }

    assert operation._execution_safety_blocker(10.0) == "PRESSURE_DELTA_ABORT"


def test_live_execution_monitor_reports_emergency_pressure_abort():
    operation = object.__new__(PiperPulseAlign)
    operation._xbox_known = True
    operation._xbox_locked = True
    operation._selected_robot = "piperh"
    operation._serial_connected = True
    operation._zero_calibrated = True
    operation._local_pressure_baseline = {"s1": 100.0, "s2": 100.0, "s3": 100.0}
    operation._pressure_received = {"s1": 9.9, "s2": 9.9, "s3": 9.9}
    operation._param = lambda name: {
        "maximum_pressure_age_sec": 0.25, "dummy_contact_mode": False,
        "link6_hover_mode": False,
    }[name]
    operation._pressure_filters = {
        "s1": SimpleNamespace(status=lambda _now: SimpleNamespace(abort=False, emergency_abort=False)),
        "s2": SimpleNamespace(status=lambda _now: SimpleNamespace(abort=True, emergency_abort=True)),
        "s3": SimpleNamespace(status=lambda _now: SimpleNamespace(abort=False, emergency_abort=False)),
    }

    assert operation._execution_safety_blocker(10.0) == "PRESSURE_EMERGENCY_ABORT"


def test_live_execution_monitor_cancels_on_stale_pressure():
    operation = object.__new__(PiperPulseAlign)
    operation._xbox_known = True
    operation._xbox_locked = True
    operation._selected_robot = "piperh"
    operation._serial_connected = True
    operation._zero_calibrated = True
    operation._local_pressure_baseline = {"s1": 100.0, "s2": 100.0, "s3": 100.0}
    operation._pressure_received = {"s1": 9.9, "s2": 9.6, "s3": 9.9}
    operation._param = lambda name: {
        "maximum_pressure_age_sec": 0.25, "dummy_contact_mode": False,
        "link6_hover_mode": False,
    }[name]

    assert operation._execution_safety_blocker(10.0) == "PRESSURE_STALE"


def test_dummy_execution_ignores_camera_dropout_but_cancels_target_motion():
    operation = object.__new__(PiperPulseAlign)
    operation._param = lambda name: {
        "dummy_contact_mode": True, "maximum_pressure_age_sec": 0.25,
        "link6_hover_mode": False,
    }[name]
    operation._target_received = 7.0
    operation._attempt_invalidated = False
    operation._xbox_known = True
    operation._xbox_locked = True
    operation._selected_robot = "piperh"
    operation._serial_connected = True
    operation._zero_calibrated = True
    operation._local_pressure_baseline = {"s1": 100.0, "s2": 100.0, "s3": 100.0}
    operation._pressure_received = {"s1": 9.9, "s2": 9.9, "s3": 9.9}
    operation._pressure_filters = {
        channel: SimpleNamespace(
            status=lambda _now: SimpleNamespace(abort=False, emergency_abort=False)
        ) for channel in ("s1", "s2", "s3")
    }
    assert operation._execution_safety_blocker(10.0) is None
    operation._target_received = 9.9
    operation._attempt_invalidated = True
    assert operation._execution_safety_blocker(10.0) == "TARGET_MOVED"


def test_bare_link6_hover_does_not_require_removed_pressure_tool():
    operation = object.__new__(PiperPulseAlign)
    operation._param = lambda name: {
        "dummy_contact_mode": False,
        "link6_hover_mode": True,
        "surface_normal_max_age_sec": 1.5,
        "surface_normal_stable_deg": 5.0,
        "surface_normal_execution_dropout_grace_sec": 1.5,
        "link6_hover_continue_on_visual_dropout": True,
        "surface_normal_execution_max_deviation_deg": 8.0,
        "surface_normal_execution_hold_sec": 0.2,
    }[name]
    operation._attempt_invalidated = False
    operation._xbox_known = True
    operation._xbox_locked = True
    operation._selected_robot = "piperh"
    operation._serial_connected = False
    operation._zero_calibrated = False
    operation._local_pressure_baseline = None
    operation._latest_surface_normal = object()
    operation._surface_normal_received = 9.9
    operation._surface_normal_filter = SimpleNamespace(
        status=lambda: SimpleNamespace(
            stable=True, filtered=(0.0, 0.0, 1.0), jitter_deg=0.0, sample_count=3
        )
    )
    operation._context = {"surface_normal_planning_frame": (0.0, 0.0, 1.0)}
    operation._surface_normal_drift_since = None

    assert operation._execution_safety_blocker(10.0) is None


def _bare_link6_execution(normal_status, received=9.9, continue_on_dropout=True):
    operation = object.__new__(PiperPulseAlign)
    operation._param = lambda name: {
        "dummy_contact_mode": False,
        "link6_hover_mode": True,
        "surface_normal_max_age_sec": 1.5,
        "surface_normal_stable_deg": 5.0,
        "surface_normal_execution_dropout_grace_sec": 1.5,
        "link6_hover_continue_on_visual_dropout": continue_on_dropout,
        "surface_normal_execution_max_deviation_deg": 8.0,
        "surface_normal_execution_hold_sec": 0.2,
    }[name]
    operation._attempt_invalidated = False
    operation._xbox_known = True
    operation._xbox_locked = True
    operation._selected_robot = "piperh"
    operation._latest_surface_normal = object()
    operation._surface_normal_received = received
    operation._surface_normal_filter = SimpleNamespace(status=lambda: normal_status)
    operation._context = {"surface_normal_planning_frame": (0.0, 0.0, 1.0)}
    operation._surface_normal_drift_since = None
    return operation


def test_bare_link6_execution_tolerates_temporary_unstable_normal_window():
    operation = _bare_link6_execution(SimpleNamespace(
        stable=False, filtered=None, jitter_deg=float("inf"), sample_count=0
    ))

    assert operation._execution_safety_blocker(10.0) is None


def test_bare_link6_execution_keeps_latched_normal_after_camera_dropout():
    operation = _bare_link6_execution(SimpleNamespace(
        stable=False, filtered=None, jitter_deg=float("inf"), sample_count=0
    ), received=8.49)

    assert operation._execution_safety_blocker(10.0) is None


def test_bare_link6_execution_can_restore_stale_normal_interlock():
    operation = _bare_link6_execution(SimpleNamespace(
        stable=False, filtered=None, jitter_deg=float("inf"), sample_count=0
    ), received=8.49, continue_on_dropout=False)

    assert operation._execution_safety_blocker(10.0) == "SURFACE_NORMAL_STALE"


def test_bare_link6_execution_requires_persistent_reliable_normal_deviation():
    operation = _bare_link6_execution(SimpleNamespace(
        stable=True,
        filtered=(0.173648, 0.0, 0.984808),
        jitter_deg=0.1,
        sample_count=3,
    ), continue_on_dropout=False)

    assert operation._execution_safety_blocker(10.0) is None
    assert operation._execution_safety_blocker(10.19) is None
    assert operation._execution_safety_blocker(10.21) == "SURFACE_NORMAL_MOVED"


def test_bare_link6_endpoint_test_ignores_post_submit_visual_invalidation():
    operation = _bare_link6_execution(SimpleNamespace(
        stable=True,
        filtered=(0.173648, 0.0, 0.984808),
        jitter_deg=0.1,
        sample_count=3,
    ))
    operation._attempt_invalidated = True

    assert operation._execution_safety_blocker(10.0) is None
    assert operation._execution_safety_blocker(10.5) is None


def test_bare_link6_pre_execution_still_requires_stable_normal():
    operation = _bare_link6_execution(SimpleNamespace(
        stable=False, filtered=None, jitter_deg=float("inf"), sample_count=0
    ))

    assert operation._pre_execution_surface_normal_blocker(10.0) == (
        "SURFACE_NORMAL_UNSTABLE"
    )


def test_dummy_heartbeat_reports_dropout_without_invalidating_active_trajectory(monkeypatch):
    operation = object.__new__(PiperPulseAlign)
    operation._last_heartbeat = 0.0
    operation._heartbeat_period = 0.5
    operation._latest_target = object()
    operation._target_received = 7.0
    operation._target_lost_reset = False
    operation._attempt_invalidated = False
    operation._planning_state = PlanningState.PRECONTACT_EXECUTING
    operation._primary_blocker = "NONE"
    operation._diagnostics = SimpleNamespace(plan_id="dummy-plan")
    operation._target_filter = SimpleNamespace(reset=lambda _reason: None)
    operation._param = lambda name: {
        "target_dropout_grace_sec": 0.5,
        "target_soft_timeout_sec": 1.5,
        "target_hard_timeout_sec": 2.0,
        "target_stable_radius_m": 0.005,
        "dummy_contact_mode": True,
        "link6_hover_mode": False,
    }[name]
    operation.build_diagnostic_snapshot = lambda: {
        "state": "PRECONTACT_EXECUTING", "target_stable": False,
        "target_jitter_mm": 0.0, "target_motion_mm": 0.0,
        "target_gap_state": "TARGET_LOST", "target_age_ms": 3000.0,
        "stable_latched": False, "pressure": {"max_delta_pa": 0.0},
        "gates": {}, "serial_connected": True,
    }
    operation._publish_diagnostics = lambda _snapshot: None
    events = []
    operation._emit = lambda event_type, **_fields: events.append(event_type)
    operation.node = SimpleNamespace(
        get_logger=lambda: SimpleNamespace(info=lambda _message: None)
    )
    monkeypatch.setattr(pulse_align_module.time, "monotonic", lambda: 10.0)

    operation._heartbeat()

    assert operation._target_lost_reset is True
    assert operation._attempt_invalidated is False
    assert operation._planning_state == PlanningState.PRECONTACT_EXECUTING
    assert operation._primary_blocker == "NONE"
    assert "frozen_target_dropout" in events


def test_link6_heartbeat_keeps_frozen_goal_during_eye_in_hand_dropout(monkeypatch):
    operation = object.__new__(PiperPulseAlign)
    operation._last_heartbeat = 0.0
    operation._heartbeat_period = 0.5
    operation._latest_target = object()
    operation._target_received = 7.0
    operation._target_lost_reset = False
    operation._attempt_invalidated = False
    operation._planning_state = PlanningState.PRECONTACT_EXECUTING
    operation._primary_blocker = "NONE"
    operation._diagnostics = SimpleNamespace(plan_id="hover-plan")
    operation._target_filter = SimpleNamespace(reset=lambda _reason: None)
    operation._param = lambda name: {
        "target_dropout_grace_sec": 0.5,
        "target_soft_timeout_sec": 1.5,
        "target_hard_timeout_sec": 2.0,
        "target_stable_radius_m": 0.005,
        "dummy_contact_mode": False,
        "link6_hover_mode": True,
        "link6_hover_continue_on_visual_dropout": True,
    }[name]
    operation.build_diagnostic_snapshot = lambda: {
        "state": "PRECONTACT_EXECUTING", "target_stable": False,
        "target_jitter_mm": 0.0, "target_motion_mm": 0.0,
        "target_gap_state": "TARGET_LOST", "target_age_ms": 3000.0,
        "stable_latched": False, "pressure": {"max_delta_pa": 0.0},
        "gates": {}, "serial_connected": True,
    }
    operation._publish_diagnostics = lambda _snapshot: None
    events = []
    operation._emit = lambda event_type, **_fields: events.append(event_type)
    operation.node = SimpleNamespace(
        get_logger=lambda: SimpleNamespace(info=lambda _message: None)
    )
    monkeypatch.setattr(pulse_align_module.time, "monotonic", lambda: 10.0)

    operation._heartbeat()

    assert operation._target_lost_reset is True
    assert operation._attempt_invalidated is False
    assert operation._planning_state == PlanningState.PRECONTACT_EXECUTING
    assert operation._primary_blocker == "NONE"
    assert "frozen_target_dropout" in events
