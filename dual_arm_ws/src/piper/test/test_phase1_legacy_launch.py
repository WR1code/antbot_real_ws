from pathlib import Path


def test_legacy_dual_piper_launch_fails_closed_by_default():
    source = (
        Path(__file__).parents[1] / "launch" / "start_two_piper.launch.py"
    ).read_text(encoding="utf-8")
    assert "allow_legacy_unsafe_launch" in source
    assert "default_value='false'" in source
    assert "auto_enable" in source
    assert "default_value='false'" in source
    assert "can_left_port must be explicitly configured" in source
    assert "can_right_port must be explicitly configured" in source
    assert "must use different CAN interfaces" in source


def test_uninstalled_alternate_vendor_node_is_fail_closed():
    source = (
        Path(__file__).parents[1] / "piper" / "piper_ctrl_single_node_new.py"
    ).read_text(encoding="utf-8")
    assert "PIPER_ALLOW_LEGACY_UNGUARDED_NODE" not in source
    assert "legacy unguarded Piper node is permanently disabled" in source
