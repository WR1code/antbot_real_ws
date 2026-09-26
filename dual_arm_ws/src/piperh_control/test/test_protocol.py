import threading
from types import SimpleNamespace

import pytest

from piperh_control.hardware_adapter import (
    HardwareAdapter, LOWER, NORMAL_FEEDBACK_SOURCE, UPPER,
    bounded_arm, command_gate_reason,
)
from piperh_control.joint_state_udp_bridge import encode_packet, ordered_positions


def test_joint_order_is_name_based():
    assert ordered_positions(["joint2", "joint1"], [2.0, 1.0],
                             ["joint1", "joint2"]) == [1.0, 2.0]


def test_malformed_joint_state_is_rejected():
    assert ordered_positions(["joint1"], [1.0, 2.0], ["joint1"]) is None
    assert ordered_positions(["joint1", "joint1"], [1.0, 2.0], ["joint1"]) is None


def test_packet_shape():
    packet = encode_packet(4, [0.0] * 6)
    assert b'"version":1' in packet and b'"sequence":4' in packet


def test_piper_h_hardware_limits():
    names = [f"joint{i}" for i in range(1, 7)]
    assert bounded_arm(names, [0.0, 1.0, -1.0, 0.0, 0.0, 0.0])
    assert bounded_arm(names, [0.0, -0.1, -1.0, 0.0, 0.0, 0.0]) is None
    assert LOWER[1] == 0.0 and UPPER[2] == 0.0


def test_driver_commands_require_feedback_and_enable():
    assert command_gate_reason(False, True) == "stale CAN feedback"
    assert command_gate_reason(True, False) == "motors are disabled"
    assert command_gate_reason(True, True) is None


def test_normal_feedback_remains_source_during_gravity(monkeypatch):
    from piperh_control import hardware_adapter as adapter
    monkeypatch.setattr(adapter.time, "monotonic", lambda: 20.01)
    node = SimpleNamespace(
        _lock=threading.Lock(), _last_joint_stamp=100.005,
        _last_feedback=20.0, _arm=[0.1] * 6,
        _playback_feedback_timeout=0.2, _gravity_active=True,
    )
    feedback = HardwareAdapter.get_joint_feedback(node)
    assert feedback.source == NORMAL_FEEDBACK_SOURCE
    assert feedback.feedback_timestamp == 100.005
    assert feedback.valid


def test_real_torque_precheck_fails_closed_by_default():
    values = {"gravity_real_torque_enabled": False}
    node = SimpleNamespace(get_parameter=lambda name: SimpleNamespace(
        value=values.get(name, "")
    ))
    with pytest.raises(RuntimeError, match="real torque output is disabled"):
        HardwareAdapter._gravity_parameters(node)
