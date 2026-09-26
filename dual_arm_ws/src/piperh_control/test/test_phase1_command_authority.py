from pathlib import Path

import pytest

from piperh_control.command_authority import CommandAuthority, AuthoritativeWriter


def test_action_and_servo_are_mutually_exclusive_and_switch_explicitly():
    gate = CommandAuthority()
    action = gate.acquire("action", now=10.0, lease_seconds=1.0)

    assert gate.validate(action, now=10.5)
    assert gate.acquire("servo", now=10.5, lease_seconds=1.0) is None

    assert gate.release(action)
    servo = gate.acquire("servo", now=10.6, lease_seconds=1.0)
    assert servo is not None
    assert not gate.validate(action, now=10.6)
    assert gate.validate(servo, now=10.6)


def test_stale_released_and_non_owner_tokens_cannot_reach_transport():
    sent = []
    gate = CommandAuthority()
    writer = AuthoritativeWriter(gate, sent.append)
    owner = gate.acquire("servo", now=20.0, lease_seconds=0.5)
    assert owner is not None

    assert writer.write(owner, (1, 2, 3), now=20.1)
    assert not writer.write(owner, (4, 5, 6), now=20.6)
    replacement = gate.acquire("action", now=20.6, lease_seconds=1.0)
    assert replacement is not None
    assert not writer.write(owner, (7, 8, 9), now=20.7)
    assert gate.release(replacement)
    assert not writer.write(replacement, (10,), now=20.8)
    assert sent == [(1, 2, 3)]


def test_context_releases_authority_on_exception():
    gate = CommandAuthority()
    with pytest.raises(RuntimeError):
        with gate.hold("action", now=lambda: 30.0, lease_seconds=1.0) as token:
            assert gate.validate(token, now=30.1)
            raise RuntimeError("cancel")
    assert gate.owner(now=30.1) is None


def test_hardware_adapter_checks_authority_at_final_publish_boundary():
    source = (
        Path(__file__).parents[1] / "piperh_control" / "hardware_adapter.py"
    ).read_text(encoding="utf-8")
    assert "def _publish_driver(self, arm: Sequence[float], authority" in source
    assert "self._command_authority.validate(authority" in source


def test_gravity_sdk_writes_use_authoritative_writer():
    source = (
        Path(__file__).parents[1] / "piperh_control" / "hardware_adapter.py"
    ).read_text(encoding="utf-8")
    assert "mit_writer.write(" in source
    assert "self._servo_authority" in source
    assert "self._action_authority" in source
