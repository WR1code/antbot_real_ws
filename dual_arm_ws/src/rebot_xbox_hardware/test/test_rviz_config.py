from pathlib import Path

import yaml


def _robot_models():
    config = Path(__file__).parents[1] / "config" / "dual_arm.rviz"
    root = yaml.safe_load(config.read_text(encoding="utf-8"))
    displays = root["Visualization Manager"]["Displays"]
    return {
        display["Name"]: display
        for display in displays
        if display.get("Class") == "rviz_default_plugins/RobotModel"
    }


def _panels():
    config = Path(__file__).parents[1] / "config" / "dual_arm.rviz"
    root = yaml.safe_load(config.read_text(encoding="utf-8"))
    return root["Panels"]


def _displays():
    config = Path(__file__).parents[1] / "config" / "dual_arm.rviz"
    root = yaml.safe_load(config.read_text(encoding="utf-8"))
    return root["Visualization Manager"]["Displays"]


def test_dual_arm_tf_prefixes_have_no_trailing_separator():
    models = _robot_models()

    assert models["reBotArm 实机模型"]["TF Prefix"] == "rebotarm"
    assert models["Piper-H 实机模型"]["TF Prefix"] == "piperh"


def test_only_initially_selected_robot_model_is_visible():
    models = _robot_models()

    assert models["reBotArm 实机模型"]["Enabled"] is True
    assert models["reBotArm 实机模型"]["Value"] is True
    assert models["Piper-H 实机模型"]["Enabled"] is False
    assert models["Piper-H 实机模型"]["Value"] is False


def test_dual_arm_features_are_hosted_by_one_workbench():
    panels = _panels()
    names = {panel["Name"] for panel in panels}

    assert "双臂统一工作台" in names
    assert "机械臂控制" in names
    assert "双臂预设动作" in names
    assert "车辆控制中心" in names
    assert "航点与巡航组" in names
    assert "Piper-H 预设动作" not in names
    assert "Displays" not in names
    assert "Views" not in names


def test_integrated_chassis_model_and_waypoint_tools_are_preserved():
    models = _robot_models()
    assert models["ANTBot 底盘整机模型"]["Description Topic"]["Value"] == (
        "/antbot/robot_description"
    )

    config = Path(__file__).parents[1] / "config" / "dual_arm.rviz"
    root = yaml.safe_load(config.read_text(encoding="utf-8"))
    tool_classes = {tool["Class"] for tool in root["Visualization Manager"]["Tools"]}
    assert {
        "robotcar_navigation/AddWaypoint",
        "robotcar_navigation/AddCharger",
        "robotcar_navigation/AddKeepoutZone",
        "robotcar_navigation/AddSpeedZone",
    } <= tool_classes


def test_dual_arm_workbench_has_rebot_action_preview_display():
    previews = [
        display
        for display in _displays()
        if display.get("Class") == "moveit_rviz_plugin/RobotState"
    ]

    assert len(previews) == 1
    assert previews[0]["Robot State Topic"] == "/rebotarm/display_robot_state"
    assert previews[0]["Enabled"] is False


def test_piper_staged_precontact_markers_are_visible():
    displays = {display.get("Name"): display for display in _displays()}
    staged = displays["Piper-H 两阶段预接触路径"]
    assert staged["Enabled"] is True
    assert staged["Topic"]["Value"] == "/piperh/pulse/staged_precontact_markers"
