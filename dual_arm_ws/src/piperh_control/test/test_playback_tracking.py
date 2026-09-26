import csv
import threading
import math

import pytest

from piperh_control.playback_tracking import (
    TrackingLog,
    bounded_trajectory_step,
    prepare_trajectory,
    wrap_to_pi,
)


LOWER = [-2.618, 0.0, -2.96706, -2.356195, -1.56207, -3.14]
UPPER = [2.618, 3.14, 0.0, 2.356195, 1.56207, 3.14]
Q0 = [0.0, 1.0, -1.0, 0.0, 0.0, 0.0]


def test_delayed_control_tick_cannot_jump_far_ahead():
    assert bounded_trajectory_step(0.01, 0.01) == pytest.approx(0.01)
    assert bounded_trajectory_step(0.38, 0.01) == pytest.approx(0.02)
    assert bounded_trajectory_step(-0.01, 0.01) == 0.0


def test_preparation_preserves_knots_and_retimes_fast_samples():
    q1 = [0.1, 1.0, -1.0, 0.0, 0.0, 0.0]
    q2 = [0.2, 1.0, -1.0, 0.0, 0.0, 0.0]
    plan = prepare_trajectory([0.0, 0.01, 0.02], [Q0, q1, q2],
                              velocity_limit=0.3, acceleration_limit=1.0,
                              rate_hz=100, lower=LOWER, upper=UPPER)
    assert plan.duration > 0.02
    assert plan.sample(0.0) == pytest.approx(Q0)
    assert plan.sample(0.01 * plan.scale) == pytest.approx(q1)
    assert plan.sample(plan.duration) == pytest.approx(q2)
    samples = [plan.sample(i * plan.duration / 1000)[0] for i in range(1001)]
    dt = plan.duration / 1000
    velocity = [(b-a)/dt for a, b in zip(samples, samples[1:])]
    assert max(abs(v) for v in velocity) <= 0.301
    assert max(abs(b-a)/dt for a, b in zip(velocity, velocity[1:])) <= 1.01


def test_preparation_rejects_duplicate_time_and_unwrapped_joint_limit():
    with pytest.raises(ValueError, match="strictly increasing"):
        prepare_trajectory([0.0, 0.0], [Q0, Q0], velocity_limit=.3,
                           acceleration_limit=1, rate_hz=100, lower=LOWER, upper=UPPER)
    end = list(Q0)
    end[5] = -3.13
    start = list(Q0)
    start[5] = 3.13
    with pytest.raises(ValueError, match="unwrapped"):
        prepare_trajectory([0.0, 1.0], [start, end], velocity_limit=.3,
                           acceleration_limit=1, rate_hz=100, lower=LOWER, upper=UPPER)


def test_tracking_log_records_signed_error_and_statistics(tmp_path):
    path = tmp_path / "playback.csv"
    tracking = TrackingLog(path)
    actual = list(Q0)
    actual[0] = -0.1
    assert tracking.add(0.0, Q0, actual, 0.01, "TRACK")[0] == pytest.approx(0.1)
    assert "J1: max=0.1000" in tracking.summary()
    tracking.close()
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 1
    assert float(rows[0]["j1_error"]) == pytest.approx(0.1)
    assert math.isclose(wrap_to_pi(-math.pi - 0.1), math.pi - 0.1)


@pytest.mark.parametrize("actual_offset,expected", [(0.0, "SUCCESS"), (0.1, "TRACKING_TIMEOUT")])
def test_action_waits_for_real_final_convergence(monkeypatch, tmp_path, actual_offset, expected):
    """Run the existing ActionServer execution method with an entirely fake clock/driver."""
    from types import SimpleNamespace
    from control_msgs.action import FollowJointTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    from piperh_control import hardware_adapter as adapter

    clock = SimpleNamespace(now=100.0)
    monkeypatch.setattr(adapter, "time", SimpleNamespace(
        monotonic=lambda: clock.now,
        sleep=lambda duration: setattr(clock, "now", clock.now + max(duration, 0.001)),
    ))
    monkeypatch.setattr(adapter.rclpy, "ok", lambda: True)
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = [f"joint{j}" for j in range(1, 7)]
    for nanoseconds in (0, 20_000_000):
        point = JointTrajectoryPoint()
        point.positions = Q0
        point.time_from_start.nanosec = nanoseconds
        goal.trajectory.points.append(point)
    calls = []
    handle = SimpleNamespace(
        request=goal, is_cancel_requested=False,
        succeed=lambda: calls.append("SUCCESS"),
        abort=lambda: calls.append("ABORTED"),
        canceled=lambda: calls.append("CANCELED"),
        publish_feedback=lambda feedback: calls.append(feedback),
    )
    stamp = SimpleNamespace(value=0)

    def feedback():
        stamp.value += 1
        q = list(Q0)
        q[0] += actual_offset
        return adapter.JointFeedback(tuple(q), float(stamp.value), 0.001,
                                     True, adapter.NORMAL_FEEDBACK_SOURCE)

    params = {"playback_tracking_dir": str(tmp_path)}
    node = SimpleNamespace(
        _playback_velocity=.3, _playback_acceleration=1.0,
        _playback_rate=100.0, _playback_feedback_timeout=.2,
        _playback_tolerance=.02, _playback_stable_cycles=3,
        _playback_settle_timeout=.04, _tracking_warn=.05,
        _tracking_abort=.2, _tracking_abort_duration=.5,
        _driver_speed=15, get_joint_feedback=feedback,
        _lock=threading.RLock(),
        _action_authority=object(),
        _authority_lease=1.0,
        _command_authority=SimpleNamespace(renew=lambda *args, **kwargs: True),
        _publish_driver=lambda q, authority: calls.append(tuple(q)) or True,
        _duration=adapter.HardwareAdapter._duration,
        get_parameter=lambda name: SimpleNamespace(value=params[name]),
        get_logger=lambda: SimpleNamespace(info=lambda _: None, warning=lambda _: None),
    )
    result = adapter.HardwareAdapter._execute_arm_locked(node, handle)
    assert calls.count("SUCCESS") == (1 if expected == "SUCCESS" else 0)
    assert ("ABORTED" in calls) == (expected == "TRACKING_TIMEOUT")
    assert (result.error_code == FollowJointTrajectory.Result.SUCCESSFUL) == (expected == "SUCCESS")
    if expected == "TRACKING_TIMEOUT":
        assert "TRACKING_TIMEOUT" in result.error_string
    assert len(list(tmp_path.glob("playback_tracking_*.csv"))) == 1
