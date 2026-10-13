"""Offline arrival checks; no hardware goals are sent."""
from collections import deque
import math
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from rclpy.task import Future
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from rebotarm_pulse.handeye_arrival import HandeyeArrivalWindow, arrival_timeout_message
from rebotarm_pulse.handeye_observation import MotionSafetyError
from rebotarm_pulse.piperh_auto_handeye import JOINT_NAMES, PiperHAutoHandeye


def joints(deg=0.0):
    return [math.radians(deg)] * 6


def feed(window, values, start=0.0, count=12):
    return [window.add(values, start + i * .1, start + i * .1) for i in range(count)]


def test_action_success_position_alone_cannot_replace_full_stable_window():
    window = HandeyeArrivalWindow(joints())
    assert feed(window, joints(.092745), count=10) == [False] * 10
    assert window.add(joints(.092745), 1.1, 1.1)


@pytest.mark.parametrize('residual', [.1, -.1, .2, 1.15])
def test_all_joints_must_be_strictly_inside_tolerance(residual):
    for index in range(6):
        values = joints()
        values[index] = math.radians(residual)
        assert not any(feed(HandeyeArrivalWindow(joints()), values))


def test_variation_is_peak_to_peak_over_full_window_and_strict():
    window = HandeyeArrivalWindow(joints())
    for i in range(12):
        assert not window.add(joints(0 if i % 2 == 0 else .01), i * .1, i * .1)
    assert any(feed(window, joints(.005), start=1.2, count=13))


def test_out_of_tolerance_excursion_resets_window():
    window = HandeyeArrivalWindow(joints())
    assert not any(feed(window, joints(), count=8))
    assert not window.add(joints(.11), .8, .8)
    assert not any(feed(window, joints(), start=.9, count=10))
    assert window.add(joints(), 2.0, 2.0)


def test_duplicate_timestamp_and_feedback_gap_cannot_fill_window():
    window = HandeyeArrivalWindow(joints())
    assert not window.add(joints(), 1, 1)
    assert not window.add(joints(), 1, 3)
    assert not window.add(joints(), 3, 3)
    assert len(window.samples) == 1
    assert not window.add(joints(), 3.1, 4.1)  # receipt time alone is insufficient


def test_timeout_reports_all_signed_residuals_and_worst_joint():
    positions = joints(.01)
    positions[3] = math.radians(-.09)
    message = arrival_timeout_message('anchor_return', joints(), positions, .2)
    assert 'Handeye calibration arrival timeout' in message
    assert all(f'J{i}:' in message for i in range(1, 7))
    assert 'J4: -0.090000 deg' in message
    assert '最大误差关节: J4' in message
    assert 'command target:' in message
    assert 'planned final joint:' in message
    assert 'current feedback joint:' in message
    assert 'J4 error=-0.090000 deg' in message
    assert 'max error=0.090000 deg (J4)' in message
    assert 'feedback age: 0.200 s' in message


def arrival_node(monkeypatch, sample_factory):
    clock = [0.0]
    params = {'joint_arrival_tolerance_deg': .1, 'stable_duration': 1.0,
              'stable_position_variation_deg': .01, 'arrival_timeout_sec': 3.0}
    node = SimpleNamespace(
        _record_only=False, _arrival_feedback=deque(), _arrival_latest=None,
        _arrival_last_stamp=0, _stopping=threading.Event(),
        get_parameter=lambda name: SimpleNamespace(value=params[name]),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int((10 + clock[0])*1e9))),
        _motion_guard=Mock(), _publish=Mock(), get_logger=lambda: Mock(),
    )
    class Condition:
        def __enter__(self): return self
        def __exit__(self, *args): pass
        def notify_all(self): pass
        def wait(self, timeout):
            clock[0] += .1
            msg = sample_factory(clock[0])
            if msg is not None:
                PiperHAutoHandeye._arrival_joint_state(node, msg)
    node._arrival_condition = Condition()
    monkeypatch.setattr('rebotarm_pulse.piperh_auto_handeye.time.monotonic', lambda: clock[0])
    return node, clock


def message(elapsed, residual=.0):
    msg = JointState(name=list(JOINT_NAMES), position=joints(residual))
    stamp = int((10 + elapsed) * 1e9)
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 10**9)
    return msg


def test_checker_waits_for_fresh_stable_real_feedback_after_completion(monkeypatch):
    node, clock = arrival_node(monkeypatch, lambda elapsed: message(elapsed, .04))
    node._arrival_feedback.append((tuple(joints()), 9_000_000_000, -2))
    PiperHAutoHandeye._wait_for_arrival(node, joints(), 'return')
    assert clock[0] >= 1.1
    assert not node._stopping.is_set()


@pytest.mark.parametrize('factory', [
    lambda elapsed: message(elapsed, .12),
    lambda elapsed: message(.01),  # repeated timestamp cannot satisfy the window
    lambda elapsed: None,
    lambda elapsed: message(elapsed - 2),  # stale timestamps
])
def test_checker_timeout_aborts_without_arrival_success(monkeypatch, factory):
    node, clock = arrival_node(monkeypatch, factory)
    with pytest.raises(MotionSafetyError, match='Handeye calibration arrival timeout'):
        PiperHAutoHandeye._wait_for_arrival(node, joints(), 'return')
    assert node._stopping.is_set()
    assert clock[0] >= 3


def executed_node(arrival, endpoint=None):
    target = [-.15, 1.5, -1.5, -.26, 1.0, -1.93]
    endpoint = target if endpoint is None else endpoint
    trajectory = JointTrajectory(joint_names=list(JOINT_NAMES),
                                points=[JointTrajectoryPoint(positions=endpoint)])
    terminal = Future()
    terminal.set_result(SimpleNamespace(status=4, result=SimpleNamespace(error_code=0)))
    handle = SimpleNamespace(accepted=True, get_result_async=lambda: terminal)
    pending = Future()
    pending.set_result(handle)
    node = SimpleNamespace(
        _record_only=False, _motion_guard=Mock(), _lock=threading.Lock(),
        _stop_lock=threading.Lock(), _stopping=threading.Event(),
        _active_goal=None, _pending_goal=None, _active_result=None,
        _current_positions=lambda: target, _plan=Mock(return_value=trajectory),
        _trajectory=SimpleNamespace(send_goal_async=Mock(return_value=pending)),
        _goal_response=lambda _: None, _wait_for_arrival=arrival,
        get_logger=lambda: Mock(),
        get_parameter=lambda name: SimpleNamespace(
            value=(1 if name == 'bootstrap_convergence_retries' else
                   .1 if name == 'joint_arrival_tolerance_deg' else
                   .0005 if name == 'anchor_goal_tolerance_rad' else 35.0)
        ),
    )
    node._execute = lambda *a, **kw: PiperHAutoHandeye._execute(node, *a, **kw)
    return node, target


def test_terminal_trajectory_success_must_pass_arrival_gate():
    arrival = Mock()
    node, target = executed_node(arrival)
    node._execute(target, 'calibration')
    arrival.assert_called_once_with(target, 'calibration')
    node._plan.assert_called_once_with(target, target, goal_tolerance=.008)
    assert node._active_goal is None


def test_arrival_uses_planned_final_joint_not_original_command():
    arrival = Mock()
    planned = [-.146, 1.504, -1.496, -.256, 1.004, -1.93]
    node, target = executed_node(
        arrival, planned,
    )
    node._execute(target, 'calibration')
    checked_target = arrival.call_args.args[0]
    assert checked_target == pytest.approx(planned)
    assert checked_target != pytest.approx(target)


def test_bootstrap_timeout_replans_once_with_strict_scaling():
    arrival = Mock(side_effect=[False, True])
    node, target = executed_node(arrival)
    node._execute(
        target, 'bootstrap_01', goal_tolerance=.0005,
        velocity_scaling=.4, acceleration_scaling=.25,
        convergence_retries=1, command_target=target,
    )
    assert node._plan.call_count == 2
    for call in node._plan.call_args_list:
        assert call.kwargs == {
            'goal_tolerance': .0005,
            'velocity_scaling': .4,
            'acceleration_scaling': .25,
        }
    first, second = arrival.call_args_list
    assert first.kwargs['abort_on_timeout'] is False
    assert second.kwargs['abort_on_timeout'] is True


def test_anchor_arrival_timeout_prevents_sample_capture():
    arrival = Mock(side_effect=MotionSafetyError('Handeye calibration arrival timeout'))
    node, target = executed_node(arrival)
    node._capture = Mock()
    with pytest.raises(MotionSafetyError, match='arrival timeout'):
        PiperHAutoHandeye._return_anchor(node, target, 'anchor_return')
    node._capture.assert_not_called()
    node._plan.assert_called_once_with(target, target, goal_tolerance=.0005)


def test_existing_joint_callback_feeds_arrival_only_in_automatic_mode(monkeypatch):
    node, clock = arrival_node(monkeypatch, lambda _: None)
    node._lock = threading.Lock()
    node._joint_history = deque()
    node._arrival_joint_state = Mock()
    PiperHAutoHandeye._joint_state(node, message(.1, .02))
    assert node._positions == tuple(joints(.02))
    node._arrival_joint_state.assert_called_once()
    node._arrival_joint_state.reset_mock()
    node._record_only = True
    PiperHAutoHandeye._joint_state(node, message(.2, .03))
    assert node._positions == tuple(joints(.03))
    node._arrival_joint_state.assert_not_called()


@pytest.mark.parametrize('kind', ['missing', 'duplicate', 'nan', 'future'])
def test_arrival_rejects_malformed_or_future_feedback(monkeypatch, kind):
    node, clock = arrival_node(monkeypatch, lambda _: None)
    msg = message(.1)
    if kind == 'missing':
        msg.name = list(JOINT_NAMES[:5])
        msg.position = joints()[:5]
    elif kind == 'duplicate':
        msg.name = [JOINT_NAMES[0], *JOINT_NAMES[:5]]
    elif kind == 'nan':
        msg.position = [math.nan, *joints()[:5]]
    else:
        msg = message(2)
    PiperHAutoHandeye._arrival_joint_state(node, msg)
    assert node._arrival_latest is None
    assert not node._arrival_feedback
