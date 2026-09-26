from types import SimpleNamespace

from geometry_msgs.msg import Pose
from rebot_teach_msgs.srv import ConfigureForbiddenZone
import yaml

from rebot_xbox_hardware.forbidden_zone_config import load_zone_config
from rebot_xbox_hardware.forbidden_zone_manager import (
    collision_object,
    forbidden_object_color,
    ForbiddenZoneManager,
    interactive_marker,
)


def _manager(tmp_path):
    tmp_path.mkdir(parents=True, exist_ok=True)
    manager = ForbiddenZoneManager.__new__(ForbiddenZoneManager)
    manager._busy = False
    manager._apply_client = SimpleNamespace(service_is_ready=lambda: True)
    manager._get_client = SimpleNamespace(service_is_ready=lambda: True)
    manager._user_config_file = tmp_path / "user.yaml"
    manager._config_file = str(tmp_path / "base.yaml")
    (tmp_path / "base.yaml").write_text("areas: {}\ngroups: {}\n")
    manager._start_apply = lambda: None
    return manager


def test_configure_service_persists_and_removes_primitive(tmp_path):
    manager = _manager(tmp_path)
    request = ConfigureForbiddenZone.Request()
    request.operation = "upsert"
    request.name = "table_keepout"
    request.shape = "box"
    request.dimensions = [0.4, 0.3, 0.2]
    request.frame_id = "base_link"
    request.pose.orientation.w = 1.0
    response = manager._configure_zone(request, ConfigureForbiddenZone.Response())
    assert response.success
    stored = yaml.safe_load(manager._user_config_file.read_text())
    zone = load_zone_config(stored).areas["table_keepout"]
    assert zone.dimensions == (0.4, 0.3, 0.2)

    remove = ConfigureForbiddenZone.Request(operation="remove", name="table_keepout")
    response = manager._configure_zone(remove, ConfigureForbiddenZone.Response())
    assert response.success
    assert yaml.safe_load(manager._user_config_file.read_text())["areas"] == {}


def test_collision_object_contains_loaded_obj_mesh(tmp_path):
    path = tmp_path / "triangle.obj"
    path.write_text("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n")
    zone = load_zone_config({"areas": {"mesh_keepout": {
        "shape": "mesh", "mesh_path": str(path), "mesh_scale": 0.001,
    }}}).areas["mesh_keepout"]
    result = collision_object(zone)
    assert result.id == "forbidden_zone/mesh_keepout"
    assert len(result.meshes) == 1
    assert len(result.meshes[0].triangles) == 1


def test_configure_mesh_copies_upload_to_managed_storage(tmp_path):
    source = tmp_path / "upload.obj"
    source.write_text("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n")
    manager = _manager(tmp_path / "runtime")
    request = ConfigureForbiddenZone.Request()
    request.operation = "upsert"
    request.name = "uploaded_mesh"
    request.shape = "mesh"
    request.frame_id = "base_link"
    request.mesh_path = str(source)
    request.mesh_scale = 0.001
    request.pose.orientation.w = 1.0
    response = manager._configure_zone(request, ConfigureForbiddenZone.Response())
    assert response.success
    stored = yaml.safe_load(manager._user_config_file.read_text())
    managed = stored["areas"]["uploaded_mesh"]["mesh_path"]
    assert managed != str(source)
    assert "forbidden_zone_meshes" in managed


def test_user_zone_has_six_dof_interactive_marker():
    zone = load_zone_config({"areas": {"movable box": {
        "shape": "box", "dimensions": [0.4, 0.3, 0.2],
        "pose": {"position": [0.1, 0.2, 0.3]},
    }}}).areas["movable box"]
    marker = interactive_marker(zone)
    assert marker.header.frame_id == "base_link"
    assert marker.description.startswith("禁区：movable box")
    assert marker.controls[0].markers[0].scale.x == 0.4
    assert {control.name for control in marker.controls[1:]} == {
        "move_x", "move_y", "move_z", "rotate_x", "rotate_y", "rotate_z",
    }


def test_dragged_pose_is_persisted_without_changing_shape(tmp_path):
    manager = _manager(tmp_path)
    manager._user_config_file.write_text(yaml.safe_dump({"areas": {"drag_me": {
        "shape": "sphere", "dimensions": [0.12],
        "pose": {"position": [0.0, 0.0, 0.0]},
    }}}))
    zone = load_zone_config(yaml.safe_load(
        manager._user_config_file.read_text()
    )).areas["drag_me"]
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (0.4, -0.2, 0.8)
    pose.orientation.w = 1.0
    moved = manager._zone_with_pose(zone, pose)
    manager._persist_interactive_pose("drag_me", moved)
    stored = yaml.safe_load(manager._user_config_file.read_text())
    assert stored["areas"]["drag_me"]["dimensions"] == [0.12]
    assert stored["areas"]["drag_me"]["pose"]["position"] == [0.4, -0.2, 0.8]


def test_named_user_group_is_persisted_and_removed(tmp_path):
    manager = _manager(tmp_path)
    manager._user_config_file.write_text(yaml.safe_dump({"areas": {
        "left": {"shape": "sphere", "dimensions": [0.1]},
        "right": {"shape": "box", "dimensions": [0.1, 0.2, 0.3]},
    }}))
    request = ConfigureForbiddenZone.Request()
    request.operation = "upsert_group"
    request.name = "fixtures"
    request.members = ["left", "right"]
    response = manager._configure_zone(request, ConfigureForbiddenZone.Response())
    assert response.success
    stored = yaml.safe_load(manager._user_config_file.read_text())
    assert stored["groups"]["fixtures"]["areas"] == ["left", "right"]

    remove = ConfigureForbiddenZone.Request(
        operation="remove_group", name="fixtures"
    )
    response = manager._configure_zone(remove, ConfigureForbiddenZone.Response())
    assert response.success
    assert yaml.safe_load(manager._user_config_file.read_text())["groups"] == {}


def test_forbidden_zones_remain_red_without_a_reach_boundary():
    zone = load_zone_config({"areas": {"red": {
        "shape": "sphere", "dimensions": [0.1],
    }}}).areas["red"]
    color = forbidden_object_color(zone).color
    assert color.r > 0.8 and color.g < 0.1 and color.b < 0.1
