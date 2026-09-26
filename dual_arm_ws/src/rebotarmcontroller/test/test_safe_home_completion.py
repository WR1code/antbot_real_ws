import threading

import numpy as np
import pytest

from rebotarmcontroller.hardware_config import _ensure_rebot_sdk_in_syspath
from rebotarmcontroller.hardware_manager import HardwareManager


_ensure_rebot_sdk_in_syspath()
from motorbridge import CallError  # noqa: E402
from reBotArm_control_py.actuator.rebotarm import JointGroup  # noqa: E402
from reBotArm_control_py.controllers.rebotarm_endpose_controller import (  # noqa: E402
    RebotArmEndPose,
)


class ConstantStateRobot:
    def __init__(self, position):
        self.position = np.asarray(position, dtype=np.float64)

    def get_state(self):
        zeros = np.zeros_like(self.position)
        return self.position.copy(), zeros, zeros


def bare_controller(position):
    controller = object.__new__(RebotArmEndPose)
    controller._running = True
    controller._n = len(position)
    controller._dt = 0.01
    controller._q_target = np.zeros(len(position), dtype=np.float64)
    controller._vlim_override = None
    controller.rebotarm = ConstantStateRobot(position)
    return controller


def test_safe_home_reports_failure_when_feedback_never_settles(monkeypatch):
    clock = [0.0]
    monkeypatch.setattr(
        "reBotArm_control_py.controllers.rebotarm_endpose_controller.time.monotonic",
        lambda: clock[0],
    )
    monkeypatch.setattr(
        "reBotArm_control_py.controllers.rebotarm_endpose_controller.time.sleep",
        lambda duration: clock.__setitem__(0, clock[0] + max(duration, 0.1)),
    )
    controller = bare_controller([0.02])
    with pytest.raises(TimeoutError, match="did not settle"):
        controller.safe_home(max_vel=1.0, settle_thresh=0.01)
    assert controller._vlim_override is None


def test_explicit_home_shutdown_disconnects_even_when_safe_home_fails():
    events = []

    class Robot:
        def stop_control_loop(self):
            events.append("stop")

        def disable_all(self):
            events.append("disable")

        def disconnect(self):
            events.append("disconnect")

    manager = object.__new__(HardwareManager)
    manager._connected = True
    manager._enabled = True
    manager._control_output_enabled = True
    manager._state_machine = "IDLE"
    manager._cmd_lock = threading.RLock()
    manager._robot = Robot()
    manager._endpos_ctrl = type("EndPose", (), {"_running": True})()
    manager.safe_home = lambda: (_ for _ in ()).throw(TimeoutError("not home"))

    with pytest.raises(RuntimeError, match="safe_home"):
        manager.shutdown(request_home=True)
    assert events == ["stop", "disable", "disconnect"]
    assert manager._connected is False


def test_shutdown_runs_disable_and_disconnect_after_stop_failure():
    events = []

    class Robot:
        def stop_control_loop(self):
            events.append("stop")
            raise KeyboardInterrupt()

        def disable_all(self):
            events.append("disable")

        def disconnect(self):
            events.append("disconnect")

    manager = object.__new__(HardwareManager)
    manager._connected = True
    manager._enabled = True
    manager._control_output_enabled = True
    manager._state_machine = "IDLE"
    manager._cmd_lock = threading.RLock()
    manager._robot = Robot()
    manager._endpos_ctrl = type("EndPose", (), {"_running": True})()
    manager.safe_home = lambda: None

    with pytest.raises(RuntimeError, match="stop_control_loop"):
        manager.shutdown()
    assert events == ["stop", "disable", "disconnect"]
    assert manager._connected is False


def test_enter_mode_retries_a_partial_group_failure(monkeypatch):
    attempts = []

    class Group:
        def mode_pos_vel(self):
            attempts.append("pos_vel")
            return len(attempts) == 3

    monkeypatch.setattr(
        "rebotarmcontroller.hardware_manager.time.sleep",
        lambda _duration: None,
    )

    HardwareManager._enter_mode(Group(), "pos_vel", "arm")

    assert attempts == ["pos_vel", "pos_vel", "pos_vel"]


def test_enter_mode_rejects_persistent_group_failure(monkeypatch):
    class Group:
        def mode_pos_vel(self):
            return False

    monkeypatch.setattr(
        "rebotarmcontroller.hardware_manager.time.sleep",
        lambda _duration: None,
    )

    with pytest.raises(RuntimeError, match="after 3 attempts"):
        HardwareManager._enter_mode(Group(), "pos_vel", "arm")


def test_pos_vel_send_reports_failed_joint():
    class FailedMotor:
        def send_pos_vel(self, _position, _velocity_limit):
            raise CallError("serial write failed")

    group = object.__new__(JointGroup)
    group._jcfgs = [type("Joint", (), {"name": "joint6"})()]
    group._mm = {"joint6": FailedMotor()}
    group._pv_vlim = np.array([3.0])

    with pytest.raises(RuntimeError, match="joint6: serial write failed"):
        group.send_pos_vel(np.array([0.1]))


def test_control_output_failure_stops_commands_and_records_error():
    class FailedEndPose:
        def _loop_cb(self, _robot, _dt):
            raise RuntimeError("POS_VEL command failed for joint6")

    manager = object.__new__(HardwareManager)
    manager._cmd_lock = threading.RLock()
    manager._control_output_enabled = True
    manager._error_codes = []
    manager._endpos_ctrl = FailedEndPose()

    with pytest.raises(RuntimeError, match="joint6"):
        manager._endpos_loop_cb(object(), 0.01)

    assert manager._control_output_enabled is False
    assert manager.error_codes == [
        "control_output_failed: POS_VEL command failed for joint6"
    ]
