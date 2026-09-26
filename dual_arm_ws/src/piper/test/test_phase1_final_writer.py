from pathlib import Path

from piper.command_writer import PiperCommandWriter
from piper.command_authority import CommandAuthority


class FakePiper:
    def __init__(self):
        self.calls = []

    def JointCtrl(self, *values):
        self.calls.append(("joint", values))

    def EndPoseCtrl(self, *values):
        self.calls.append(("pose", values))


def test_vendor_transport_rejects_non_owner_and_stale_owner():
    sdk = FakePiper()
    gate = CommandAuthority()
    writer = PiperCommandWriter(sdk, gate)
    token = gate.acquire("piperh_adapter", now=1.0, lease_seconds=0.5)

    assert writer.joints(token, (1, 2, 3, 4, 5, 6), now=1.1)
    assert not writer.joints(None, (6, 5, 4, 3, 2, 1), now=1.2)
    assert not writer.joints(token, (0, 0, 0, 0, 0, 0), now=1.6)
    assert sdk.calls == [("joint", (1, 2, 3, 4, 5, 6))]


def test_vendor_unsafe_topics_are_fail_closed_by_default():
    source = (
        Path(__file__).parents[1] / "piper" / "piper_ctrl_single_node.py"
    ).read_text(encoding="utf-8")
    assert "allow_unsafe_command_topics" in source
    assert "declare_parameter('allow_unsafe_command_topics', False)" in source
    assert "self._authorized_writer" in source
    assert "unsafe Piper command topics are permanently disabled" in source
