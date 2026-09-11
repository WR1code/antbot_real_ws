"""Pure safety-gate tests for the H743 ROS bridge."""

import math
from types import SimpleNamespace

import pytest

from antbot_h743_bridge.bridge import (
    CmdVelUartBridge,
    JOINT_NAMES,
    ack_is_motion_ready,
    joint_positions_from_feedback,
)
from antbot_h743_bridge.chassis_uart_protocol import Ack, CONTROL_IDS


def ack(*, flags=0x04, faults=0):
    return Ack(
        sequence=1,
        status=0,
        chassis_state=4,
        reject_reason=0,
        fault_flags=faults,
        steering_flags=flags,
        can_flags=0,
        uart_valid_count=0,
        can_tx_count=0,
        can_rx_count=0,
        stm32_tick=1,
        steering_position_mrad=(0, 0, 0, 0),
    )


def test_motion_requires_ready_fault_free_ack():
    assert not ack_is_motion_ready(None)
    assert not ack_is_motion_ready(ack(flags=0x00))
    assert not ack_is_motion_ready(ack(flags=0x0C))
    assert not ack_is_motion_ready(ack(faults=1))
    assert ack_is_motion_ready(ack())


def test_joint_positions_follow_real_steering_and_drive_feedback():
    steering = ack()
    steering = Ack(
        **{
            **steering.__dict__,
            "steering_position_mrad": (1000, -500, 0, -32768),
        }
    )
    drive = Ack(
        **{
            **ack().__dict__,
            "detail_type": 4,
            "detail_values": (9000, -18000, 36000, 4500),
            "detail_valid_mask": 0b0111,
        }
    )

    positions = joint_positions_from_feedback(
        steering, drive, [0.25] * len(JOINT_NAMES)
    )

    assert len(JOINT_NAMES) == 8
    assert positions[0] == pytest.approx(1.0)
    assert positions[1] == pytest.approx(math.pi / 2)
    assert positions[2] == pytest.approx(-0.5)
    assert positions[3] == pytest.approx(-math.pi)
    assert positions[5] == pytest.approx(2 * math.pi)
    assert positions[6] == pytest.approx(0.25)  # invalid steering keeps last value
    assert positions[7] == pytest.approx(0.25)  # invalid drive keeps last value


def test_system_reset_locks_motion_and_sends_guarded_command():
    bridge = object.__new__(CmdVelUartBridge)
    bridge.operator_requested = True
    bridge.serial = object()
    bridge.latest_ack = ack()
    bridge.last_ack_monotonic = 123.0
    bridge.feedback = {2: ack()}
    calls = []
    bridge.send_stop_frames = lambda: calls.append(("stop",))
    bridge.send_control = lambda command, payload=b"": (
        calls.append((command, payload)) or True
    )
    bridge.publish_status = lambda: calls.append(("status",))
    response = SimpleNamespace(success=False, message="")

    result = bridge.reset_system(None, response)

    assert result is response
    assert response.success
    assert bridge.operator_requested is False
    assert bridge.latest_ack is None
    assert bridge.last_ack_monotonic == 0.0
    assert bridge.feedback == {}
    assert calls[:2] == [
        ("stop",),
        (CONTROL_IDS["SYSTEM_RESET"], b"RST!"),
    ]
