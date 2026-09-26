import pytest

from rebot_xbox_hardware.arm_ownership import ArmOwnership


def test_handoff_waits_until_both_sources_are_confirmed_locked():
    ownership = ArmOwnership()
    assert ownership.request("piperh") == "none"
    assert ownership.report_armed("rebotarm", False) == "none"
    assert ownership.report_armed("piperh", True) == "none"
    assert ownership.report_armed("piperh", False) == "piperh"
    assert ownership.pending == ""


def test_same_owner_is_a_noop_and_bad_request_is_rejected():
    ownership = ArmOwnership(selected="piperh")
    assert ownership.request("piperh") == "piperh"
    with pytest.raises(ValueError):
        ownership.request("other")

