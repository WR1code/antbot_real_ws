"""Offline checks; these never send a goal to hardware."""
import math
from types import SimpleNamespace
from unittest.mock import Mock
import time

import pytest
from sensor_msgs.msg import JointState

from rebotarm_pulse.anchor_repeatability import (
    ANCHOR, NAMES, AnchorRepeatability, error_degrees, joint_positions, summarize,
)


def test_named_real_feedback_accepts_gripper_and_reordered_names():
    msg = JointState(name=['gripper', *reversed(NAMES)], position=[0.0, *reversed(ANCHOR)])
    assert joint_positions(msg) == tuple(ANCHOR)
    assert error_degrees(joint_positions(msg)) == [0.0] * 6


@pytest.mark.parametrize('names,values', [
    (NAMES[:5], ANCHOR[:5]), (NAMES, ANCHOR[:5]),
    ([NAMES[0], *NAMES[:5]], ANCHOR),
    (NAMES, [math.nan, *ANCHOR[1:]]), (NAMES, [math.inf, *ANCHOR[1:]]),
])
def test_bad_feedback_has_no_fallback(names, values):
    assert joint_positions(JointState(name=names, position=values)) is None


def test_signed_error_population_statistics_and_worst_joint():
    positions = [a + math.radians(0.04 * (i + 1)) for i, a in enumerate(ANCHOR)]
    assert error_degrees(positions) == pytest.approx([0.04 * i for i in range(1, 7)])
    stats, worst, passed = summarize([[0.01] * 6, [-0.03] * 5 + [-0.2]])
    assert stats[0] == pytest.approx(dict(mean=-0.01, std=0.02, max_absolute=0.03, min=-0.03, max=0.01))
    assert worst == 5
    assert not passed


@pytest.mark.parametrize('value,passed', [(0.09999, True), (0.1, False), (-0.1, False)])
def test_strict_pass_threshold(value, passed):
    assert summarize([[value] * 6])[2] is passed
    assert summarize([[value] * 6])[0][0]['std'] == 0.0


def feedback_harness():
    test = object.__new__(AnchorRepeatability)
    test.node = Mock()
    test.node.get_clock.return_value.now.return_value.nanoseconds = 10_000_000_000
    test._param = lambda _: 0.75
    test._raw = None
    test._raw_stamp = 0
    test._raw_time = 0.0
    test._sequence = 0
    test._moveit_time = time.monotonic()
    test._moveit_stamp = 9_900_000_000
    return test


def feedback(stamp):
    msg = JointState(name=NAMES, position=ANCHOR)
    msg.header.stamp.sec = int(stamp)
    msg.header.stamp.nanosec = int((stamp % 1) * 1e9)
    return msg


def test_stale_future_zero_and_replayed_feedback_are_rejected():
    node = feedback_harness()
    for stamp in [0, 8, 11]:
        node._raw_cb(feedback(stamp))
    assert node._raw is None
    node._raw_cb(feedback(9.9))
    assert node._raw == tuple(ANCHOR)
    node._raw_cb(feedback(9.9))
    node._raw_cb(feedback(9.8))
    assert node._sequence == 1


def test_stale_feedback_and_xbox_unlock_abort(monkeypatch):
    monkeypatch.setattr('rebotarm_pulse.anchor_repeatability.rclpy.ok', lambda: True)
    node = feedback_harness()
    node._xbox_known, node._xbox_armed = True, False
    with pytest.raises(RuntimeError, match='missing or stale'):
        node._guard()
    node._raw_cb(feedback(9.9))
    node._guard()
    node._raw_time = time.monotonic() - 1
    with pytest.raises(RuntimeError, match='missing or stale'):
        node._guard()
    node._xbox_armed = True
    with pytest.raises(RuntimeError, match='LOCKED'):
        node._guard()


def test_action_result_timeout_cancels_existing_goal():
    node = feedback_harness()
    node._guard = Mock()
    node._execute = Mock()
    handle = Mock(accepted=True)
    node._execute.send_goal_async.return_value.result.return_value = handle
    node.wait = Mock(side_effect=[True, False])
    node.cancel_motion = Mock()
    with pytest.raises(RuntimeError, match='result timeout'):
        node.execute_trajectory(SimpleNamespace(), 0.01)
    node.cancel_motion.assert_called_once()
    assert node._active_goal is handle


def test_settling_requires_new_feedback_and_full_stationary_window(monkeypatch):
    node = feedback_harness()
    clock = [0.0]
    node._param = lambda name: {'settle_timeout_sec': 5.0, 'settle_sec': 2.0,
                               'stationary_range_deg': 0.02}[name]
    node._guard = Mock()
    monkeypatch.setattr('rebotarm_pulse.anchor_repeatability.time.monotonic', lambda: clock[0])

    def spin(*args, **kwargs):
        clock[0] += 0.25
        node._raw_time = clock[0]
        node._sequence += 1
        # Initially moving; final two seconds must contain only stationary samples.
        node._raw = tuple(a + (0.02 * clock[0] if clock[0] <= 1 else 0.0) for a in ANCHOR)
        node._raw_stamp = int(clock[0] * 1e9)

    monkeypatch.setattr('rebotarm_pulse.anchor_repeatability.rclpy.spin_once', spin)
    positions, stamp = node._settled_feedback()
    assert positions == tuple(ANCHOR)
    assert clock[0] >= 3.25
    assert stamp > 0


def test_settling_cannot_reuse_pre_return_sample(monkeypatch):
    node = feedback_harness()
    clock = [0.0]
    node._param = lambda name: {'settle_timeout_sec': 3.0, 'settle_sec': 2.0,
                               'stationary_range_deg': 0.02}[name]
    node._guard = Mock()
    node._raw = tuple(ANCHOR)
    monkeypatch.setattr('rebotarm_pulse.anchor_repeatability.time.monotonic', lambda: clock[0])
    monkeypatch.setattr('rebotarm_pulse.anchor_repeatability.rclpy.spin_once',
                        lambda *a, **kw: clock.__setitem__(0, clock[0] + 0.25))
    with pytest.raises(RuntimeError, match='did not settle'):
        node._settled_feedback()


def test_complete_run_csv_and_alternating_departures(tmp_path, monkeypatch):
    import csv
    from rebotarm_pulse.anchor_repeatability import DEFAULTS
    node = feedback_harness()
    node._param = lambda name: {'output_dir': str(tmp_path), 'iterations': 2}.get(name, DEFAULTS[name])
    node._xbox_known, node._xbox_armed = True, False
    node._raw = tuple(ANCHOR)
    node._guard = Mock()
    node._departures = [[ANCHOR[0] + 0.1, *ANCHOR[1:]], [ANCHOR[0] - 0.1, *ANCHOR[1:]]]
    node._move = Mock()
    node._settled_feedback = Mock(side_effect=[
        (ANCHOR, 10_000_000_000), (node._departures[0], 11_000_000_000),
        (ANCHOR, 12_000_000_000), (node._departures[1], 13_000_000_000),
        (ANCHOR, 14_000_000_000),
    ])
    monkeypatch.setattr('rebotarm_pulse.anchor_repeatability.rclpy.ok', lambda: True)
    monkeypatch.setattr('rebotarm_pulse.anchor_repeatability.rclpy.spin_once', lambda *a, **kw: None)
    assert node.run()
    assert [call.args[0] for call in node._move.call_args_list] == [
        ANCHOR, node._departures[0], ANCHOR, node._departures[1], ANCHOR]
    with node.csv_path.open() as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 2
    assert list(rows[0]) == ['iteration', *(f'j{i}_error_deg' for i in range(1, 7)), 'timestamp']
    assert rows[1]['iteration'] == '2'
    assert float(rows[0]['j6_error_deg']) == 0
    assert rows[0]['timestamp'].endswith('+08:00')


def test_fresh_raw_feedback_does_not_mask_stale_moveit_state(monkeypatch):
    monkeypatch.setattr('rebotarm_pulse.anchor_repeatability.rclpy.ok', lambda: True)
    node = feedback_harness()
    node._param = lambda name: '/piperh/joint_states' if name == 'moveit_feedback_topic' else 0.75
    node._xbox_known, node._xbox_armed = True, False
    node._raw_cb(feedback(9.9))
    node._moveit_time = time.monotonic() - 20 * 60
    with pytest.raises(RuntimeError, match='MOVEIT_STATE_STALE.*piperh/joint_states'):
        node._guard()
    node._moveit_cb(feedback(9.95))
    node._guard()
    # State monitoring never replaces the raw measurement with the MoveIt stream.
    assert node._raw_stamp == 9_900_000_000


def test_moveit_monitor_rejects_replayed_or_old_states():
    node = feedback_harness()
    node._moveit_time = 0
    node._moveit_cb(feedback(8))
    node._moveit_cb(feedback(9.9))
    assert node._moveit_time == 0
    node._moveit_cb(feedback(9.95))
    assert node._moveit_time > 0


@pytest.mark.parametrize('status,code', [(6, -4), (5, -4), (4, -4)])
def test_terminal_failed_action_does_not_request_cancellation(status, code):
    node = feedback_harness()
    node._guard = Mock()
    node._execute = Mock()
    handle = Mock(accepted=True)
    node._execute.send_goal_async.return_value.result.return_value = handle
    result = SimpleNamespace(status=status, result=SimpleNamespace(
        error_code=SimpleNamespace(val=code, message='', source='')))
    handle.get_result_async.return_value.result.return_value = result
    node.wait = Mock(return_value=True)
    node.cancel_motion = Mock()
    with pytest.raises(RuntimeError, match=r'CONTROL_FAILED \(-4\)'):
        node.execute_trajectory(SimpleNamespace(), 60)
    assert node._active_goal is None
    node.cancel_motion.assert_not_called()
