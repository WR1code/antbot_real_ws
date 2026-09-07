from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[4]


def test_existing_step2_waypoint_xml_remains_xy_yaw_ground_plane_compatible():
    waypoint_file = ROOT / "isaac" / "navigation_data" / "waypoints.xml"
    tree = ET.parse(waypoint_file)
    waypoints = tree.getroot().findall("waypoint")
    assert waypoints
    for waypoint in waypoints:
        position = waypoint.find("./pose/position")
        orientation = waypoint.find("./pose/orientation")
        assert position is not None and orientation is not None
        assert all(key in position.attrib for key in ("x", "y", "z"))
        assert float(position.attrib["z"]) == 0.0
        assert all(key in orientation.attrib for key in ("z", "w"))


def test_robotcar_add_waypoint_tool_still_forces_ground_z():
    source = (
        ROOT.parent / "robotcar" / "src" / "robotcar_navigation"
        / "src" / "add_waypoint_tool.cpp"
    ).read_text(encoding="utf-8")
    assert "waypoint.pose.position.z = 0.0;" in source
