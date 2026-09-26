from pathlib import Path


LAUNCH_FILE = Path(__file__).parents[1] / "launch" / "offline_preview.launch.py"


def test_legacy_offline_preview_forwards_to_the_comprehensive_workbench():
    source = LAUNCH_FILE.read_text(encoding="utf-8")

    assert 'get_package_share_directory("rebot_xbox_hardware")' in source
    assert '"dual_arm_offline_preview.launch.py"' in source
    assert '"rebot_model": LaunchConfiguration("model")' in source
    assert "rebotarm_bringup" not in source
    assert "piperh_control" not in source


def test_offline_preview_supports_both_rebotarm_models():
    source = LAUNCH_FILE.read_text(encoding="utf-8")

    assert 'default_value="dm"' in source
    assert 'choices=["dm", "rs"]' in source
