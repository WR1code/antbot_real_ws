from types import SimpleNamespace

import pytest

from rebotarmcontroller.hardware_manager import HardwareManager, LifecycleState


class FakeRobot:
    def __init__(self):
        self.calls = []
        self.has_gripper = False

    def connect(self):
        self.calls.append("connect")

    def stop_control_loop(self):
        self.calls.append("stop_control_loop")

    def disable_all(self):
        self.calls.append("disable_all")

    def disconnect(self):
        self.calls.append("disconnect")


def manager():
    value = object.__new__(HardwareManager)
    value._robot = FakeRobot()
    value._connected = False
    value._initialized = False
    value._enabled = False
    value._control_output_enabled = False
    value._state_machine = "IDLE"
    value._lifecycle_state = LifecycleState.DISCONNECTED
    value._error_codes = []
    value._gravity_comp_active = False
    value._gripper_group = None
    value._gripper_target_position = None
    value._endpos_ctrl = SimpleNamespace(_running=False)
    value._cmd_lock = __import__("threading").RLock()
    value.safe_home_calls = 0
    value.safe_home = lambda: setattr(
        value, "safe_home_calls", value.safe_home_calls + 1
    )
    return value


def test_connect_only_connects_and_does_not_enable_or_start_control():
    value = manager()
    value.connect()
    assert value._robot.calls == ["connect"]
    assert value.connected
    assert not value.initialized
    assert not value.enabled
    assert value.lifecycle_state is LifecycleState.CONNECTED


def test_initialize_is_non_motion_and_enable_is_explicit(monkeypatch):
    value = manager()
    value.connect()
    value.initialize()
    assert value.lifecycle_state is LifecycleState.INITIALIZED
    assert not value.enabled
    assert value._robot.calls == ["connect"]

    monkeypatch.setattr(value, "start_endpos_control", lambda: setattr(value, "_enabled", True))
    value.enable()
    assert value.enabled


@pytest.mark.parametrize("reason", ["normal", "fault", "estop"])
def test_shutdown_never_homes_implicitly(reason):
    value = manager()
    value.connect()
    value.shutdown(reason=reason)
    assert value.safe_home_calls == 0
    assert value._robot.calls == [
        "connect", "stop_control_loop", "disable_all", "disconnect"
    ]
    assert value.lifecycle_state is LifecycleState.DISCONNECTED


def test_fault_and_estop_reject_even_explicit_home_request():
    for reason in ("fault", "estop"):
        value = manager()
        value.connect()
        with pytest.raises(ValueError, match="safe_home"):
            value.shutdown(reason=reason, request_home=True)
        assert value.safe_home_calls == 0
