from pathlib import Path
import ast

import yaml


ROOT = Path(__file__).resolve().parents[2]
LAUNCH = (
    ROOT / "antbot" / "antbot_navigation" / "launch"
    / "offline_waypoint_editor.launch.py"
)
RVIZ = (
    ROOT / "antbot" / "antbot_navigation" / "rviz"
    / "offline_waypoint_editor_3d.rviz"
)


def test_offline_launch_parses_and_declares_3d_arguments():
    text = LAUNCH.read_text(encoding="utf-8")
    ast.parse(text)
    for argument in (
        "map_yaml", "pointcloud_path", "pointcloud_metadata",
        "show_3d_cloud", "waypoint_file",
    ):
        assert f"'{argument}'" in text


def test_show_3d_false_default_and_condition_guard_offline_node():
    text = LAUNCH.read_text(encoding="utf-8")
    assert "DeclareLaunchArgument('show_3d_cloud', default_value='false')" in text
    node_start = text.index("executable='offline_pointcloud_publisher'")
    condition = text.index("condition=IfCondition(show_3d_cloud)", node_start)
    parameters = text.index("'pointcloud_path':", node_start)
    assert node_start < condition < parameters


def test_rviz_overlays_map_cloud_tf_robot_and_waypoints():
    config = yaml.safe_load(RVIZ.read_text(encoding="utf-8"))
    panel_classes = [panel["Class"] for panel in config["Panels"]]
    assert "robotcar_navigation/WaypointManagerPanel" in panel_classes
    manager = config["Visualization Manager"]
    classes = [display["Class"] for display in manager["Displays"]]
    for required in (
        "rviz_default_plugins/Map",
        "rviz_default_plugins/PointCloud2",
        "rviz_default_plugins/TF",
        "rviz_default_plugins/RobotModel",
        "rviz_default_plugins/InteractiveMarkers",
    ):
        assert required in classes
    assert manager["Global Options"]["Fixed Frame"] == "map"
    cloud = next(
        item for item in manager["Displays"]
        if item["Class"] == "rviz_default_plugins/PointCloud2"
    )
    assert cloud["Topic"]["Value"] == "/antbot/offline_map_points"
    assert cloud["Style"] == "Points"
    assert cloud["Alpha"] < 0.8
