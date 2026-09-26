from pathlib import Path
from types import SimpleNamespace

from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String

from antbot_h743_bridge.command_authority import CommandAuthority, AuthoritativeWriter
from antbot_h743_bridge.bridge import CmdVelUartBridge


def test_only_current_base_owner_reaches_transport_and_lease_expires():
    frames = []
    gate = CommandAuthority()
    writer = AuthoritativeWriter(gate, frames.append)
    operator = gate.acquire("operator_manager", now=5.0, lease_seconds=0.25)
    assert operator is not None

    assert writer.write(operator, "forward", now=5.1)
    assert not writer.write(None, "bypass", now=5.1)
    assert not writer.write(operator, "stale", now=5.3)
    assert frames == ["forward"]


def test_disable_and_estop_revoke_base_owner():
    gate = CommandAuthority()
    token = gate.acquire("operator_manager", now=8.0, lease_seconds=1.0)
    assert token is not None
    gate.revoke("disabled")
    assert not gate.validate(token, now=8.1)
    replacement = gate.acquire("operator_manager", now=8.2, lease_seconds=1.0)
    assert replacement is not None
    gate.revoke("estop")
    assert not gate.validate(replacement, now=8.3)


def test_bridge_claim_and_release_are_explicit():
    bridge = object.__new__(CmdVelUartBridge)
    bridge._base_authority = CommandAuthority()
    bridge.operator_requested = True
    claim = String()
    claim.data = (
        '{"owner":"operator_manager:test-session",'
        '"lease_seconds":0.5}'
    )
    bridge.on_authority_claim(claim)
    assert bridge._base_authority.owner() == "operator_manager:test-session"
    release = String()
    release.data = (
        '{"owner":"operator_manager:test-session","release":true}'
    )
    bridge.on_authority_claim(release)
    assert bridge._base_authority.owner() is None


def test_disabled_bridge_does_not_reacquire_periodic_claim():
    bridge = object.__new__(CmdVelUartBridge)
    bridge._base_authority = CommandAuthority()
    bridge.operator_requested = False
    claim = String()
    claim.data = (
        '{"owner":"operator_manager:test-session",'
        '"lease_seconds":0.5}'
    )

    bridge.on_authority_claim(claim)

    assert bridge._base_authority.owner() is None


def test_uart_bridge_uses_internal_authorized_topic_not_global_cmd_vel():
    bridge = (
        Path(__file__).parents[1] / "antbot_h743_bridge" / "bridge.py"
    ).read_text(encoding="utf-8")
    launch = (
        Path(__file__).parents[2] / "antbot_real_bringup" / "launch" /
        "real_base.launch.py"
    ).read_text(encoding="utf-8")
    config = (
        Path(__file__).parents[2] / "antbot_real_bringup" / "config" /
        "real_base.yaml"
    ).read_text(encoding="utf-8")
    assert '"/antbot/base/authorized_cmd_vel"' in bridge
    assert "TwistStamped" in bridge
    assert "self._base_authority.validate" in bridge
    assert '"topic": "/antbot/base/authorized_cmd_vel"' in launch
    assert "topic: /antbot/base/authorized_cmd_vel" in config


def test_uart_callback_rejects_bypass_before_encoding_or_transport():
    bridge = object.__new__(CmdVelUartBridge)
    bridge._base_authority = CommandAuthority()
    frames = []
    bridge._base_writer = AuthoritativeWriter(
        bridge._base_authority, frames.append
    )
    bridge.get_logger = lambda: SimpleNamespace(error=lambda *args, **kwargs: None)
    bridge.motion_allowed = lambda authority: True
    bridge.max_linear_speed = 1.0
    bridge.sequence = 0
    command = TwistStamped()
    command.header.frame_id = "unowned-debug-node"
    command.twist.linear.x = 0.5

    bridge.on_cmd_vel(command)

    assert frames == []


def test_uart_callback_accepts_only_live_owner_at_final_writer():
    bridge = object.__new__(CmdVelUartBridge)
    bridge._base_authority = CommandAuthority()
    frames = []
    bridge._base_writer = AuthoritativeWriter(
        bridge._base_authority, frames.append
    )
    bridge.get_logger = lambda: SimpleNamespace(
        error=lambda *args, **kwargs: None,
        warning=lambda *args, **kwargs: None,
    )
    bridge.motion_allowed = lambda authority: bridge._base_authority.validate(
        authority
    )
    bridge.max_linear_speed = 1.0
    bridge.sequence = 0
    owner = "operator_manager:test-session"
    bridge._base_authority.acquire(owner, lease_seconds=1.0)
    command = TwistStamped()
    command.header.frame_id = owner
    command.twist.linear.x = 0.5

    bridge.on_cmd_vel(command)

    assert len(frames) == 1


def test_uart_callback_rejects_owner_when_safety_gate_is_locked():
    bridge = object.__new__(CmdVelUartBridge)
    bridge._base_authority = CommandAuthority()
    frames = []
    bridge._base_writer = AuthoritativeWriter(
        bridge._base_authority, frames.append
    )
    bridge.get_logger = lambda: SimpleNamespace(
        error=lambda *args, **kwargs: None,
        warning=lambda *args, **kwargs: None,
    )
    bridge.motion_allowed = lambda authority: False
    bridge.max_linear_speed = 1.0
    bridge.sequence = 0
    owner = "operator_manager:test-session"
    bridge._base_authority.acquire(owner, lease_seconds=1.0)
    command = TwistStamped()
    command.header.frame_id = owner
    command.twist.linear.x = 0.5

    bridge.on_cmd_vel(command)

    assert frames == []
