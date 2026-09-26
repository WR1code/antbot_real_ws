"""Load named 3-D forbidden zones into the MoveIt planning scene."""

from dataclasses import replace
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import tempfile
from typing import Iterable

from geometry_msgs.msg import Point, Pose
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from moveit_msgs.msg import (
    CollisionObject,
    ObjectColor,
    PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA, Header
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rebot_teach_msgs.srv import ConfigureForbiddenZone
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
)
import yaml

from .forbidden_zone_config import (
    load_zone_config,
    select_active_zones,
    Zone,
    ZoneConfigError,
)
from .mesh_loader import load_mesh, MeshLoadError


OBJECT_PREFIX = "forbidden_zone/"
SHAPE_TYPES = {
    "box": SolidPrimitive.BOX,
    "sphere": SolidPrimitive.SPHERE,
    "cylinder": SolidPrimitive.CYLINDER,
    "cone": SolidPrimitive.CONE,
}
SQRT_HALF = math.sqrt(0.5)
FORBIDDEN_RED = ColorRGBA(r=0.88, g=0.05, b=0.04, a=0.72)
SUPPORTED_MODELS = {"dm", "rs", "piperh"}


def quaternion_from_rpy(roll: float, pitch: float, yaw: float):
    """Return an (x, y, z, w) quaternion without adding a tf dependency."""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def collision_object(zone: Zone) -> CollisionObject:
    """Convert one validated zone into a MoveIt collision object."""
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = zone.position
    (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ) = quaternion_from_rpy(*zone.orientation_rpy)
    result = CollisionObject(
        header=Header(frame_id=zone.frame_id),
        id=OBJECT_PREFIX + zone.name,
        operation=CollisionObject.ADD,
    )
    if zone.shape == "mesh":
        result.meshes = [load_mesh(zone.mesh_path, zone.mesh_scale)]
        result.mesh_poses = [pose]
    else:
        result.primitives = [SolidPrimitive(
            type=SHAPE_TYPES[zone.shape], dimensions=list(zone.dimensions)
        )]
        result.primitive_poses = [pose]
    return result


def _marker_name(zone_name: str) -> str:
    """Return a stable InteractiveMarker-safe name for an arbitrary zone name."""
    digest = hashlib.sha256(zone_name.encode("utf-8")).hexdigest()[:20]
    return f"user_zone_{digest}"


def _cone_marker(height: float, radius: float) -> Marker:
    marker = Marker(type=Marker.TRIANGLE_LIST, action=Marker.ADD)
    marker.scale.x = marker.scale.y = marker.scale.z = 1.0
    segments = 32
    top = Point(z=height * 0.5)
    center = Point(z=-height * 0.5)
    for index in range(segments):
        first_angle = 2.0 * math.pi * index / segments
        second_angle = 2.0 * math.pi * (index + 1) / segments
        first = Point(
            x=radius * math.cos(first_angle),
            y=radius * math.sin(first_angle), z=-height * 0.5,
        )
        second = Point(
            x=radius * math.cos(second_angle),
            y=radius * math.sin(second_angle), z=-height * 0.5,
        )
        marker.points.extend((top, first, second, center, second, first))
    return marker


def interactive_marker(zone: Zone) -> InteractiveMarker:
    """Build a six-degree-of-freedom RViz handle for one user zone."""
    result = InteractiveMarker()
    result.header.frame_id = zone.frame_id
    result.name = _marker_name(zone.name)
    result.description = f"禁区：{zone.name}（松开鼠标保存）"
    result.pose.position.x, result.pose.position.y, result.pose.position.z = zone.position
    (
        result.pose.orientation.x,
        result.pose.orientation.y,
        result.pose.orientation.z,
        result.pose.orientation.w,
    ) = quaternion_from_rpy(*zone.orientation_rpy)

    visual = Marker(action=Marker.ADD)
    extents = (0.15, 0.15, 0.15)
    if zone.shape == "box":
        visual.type = Marker.CUBE
        visual.scale.x, visual.scale.y, visual.scale.z = zone.dimensions
        extents = zone.dimensions
    elif zone.shape == "sphere":
        visual.type = Marker.SPHERE
        diameter = zone.dimensions[0] * 2.0
        visual.scale.x = visual.scale.y = visual.scale.z = diameter
        extents = (diameter, diameter, diameter)
    elif zone.shape in ("cylinder", "cone"):
        height, radius = zone.dimensions
        if zone.shape == "cylinder":
            visual.type = Marker.CYLINDER
            visual.scale.x = visual.scale.y = radius * 2.0
            visual.scale.z = height
        else:
            visual = _cone_marker(height, radius)
        extents = (radius * 2.0, radius * 2.0, height)
    else:
        visual.type = Marker.MESH_RESOURCE
        visual.mesh_resource = Path(zone.mesh_path).expanduser().resolve().as_uri()
        visual.mesh_use_embedded_materials = False
        visual.scale.x = visual.scale.y = visual.scale.z = zone.mesh_scale
        try:
            mesh = load_mesh(zone.mesh_path, zone.mesh_scale)
            coordinates = [
                (vertex.x, vertex.y, vertex.z) for vertex in mesh.vertices
            ]
            extents = tuple(
                max(point[axis] for point in coordinates) -
                min(point[axis] for point in coordinates)
                for axis in range(3)
            )
        except (OSError, MeshLoadError, ValueError):
            extents = (0.15, 0.15, 0.15)
    visual.pose.orientation.w = 1.0
    visual.color = ColorRGBA(r=0.95, g=0.03, b=0.02, a=0.52)
    visual_control = InteractiveMarkerControl()
    visual_control.name = "zone_geometry"
    visual_control.always_visible = True
    visual_control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D
    visual_control.markers.append(visual)
    result.controls.append(visual_control)
    result.scale = max(0.12, min(1.0, max(extents) * 1.25))

    axes = (
        ("x", SQRT_HALF, 0.0, 0.0),
        ("y", 0.0, SQRT_HALF, 0.0),
        ("z", 0.0, 0.0, SQRT_HALF),
    )
    for axis, x, y, z in axes:
        for mode, prefix in (
            (InteractiveMarkerControl.MOVE_AXIS, "move"),
            (InteractiveMarkerControl.ROTATE_AXIS, "rotate"),
        ):
            control = InteractiveMarkerControl()
            control.name = f"{prefix}_{axis}"
            control.orientation.w = SQRT_HALF
            control.orientation.x = x
            control.orientation.y = y
            control.orientation.z = z
            control.interaction_mode = mode
            result.controls.append(control)
    return result


def forbidden_object_color(zone: Zone) -> ObjectColor:
    return ObjectColor(id=OBJECT_PREFIX + zone.name, color=FORBIDDEN_RED)


def rpy_from_quaternion(x: float, y: float, z: float, w: float):
    """Convert a normalized quaternion to roll, pitch, yaw."""
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1e-9:
        raise ZoneConfigError("zone pose orientation must be a valid quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch_value = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(pitch_value)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return (roll, pitch, yaw)


class ForbiddenZoneManager(Node):
    """Own and atomically refresh configured MoveIt world objects."""

    def __init__(self) -> None:
        super().__init__("forbidden_zone_manager")
        self.declare_parameter("config_file", "")
        self.declare_parameter("active_groups", "")
        self.declare_parameter("user_config_file", "")
        self.declare_parameter("model", "dm")
        self.declare_parameter("move_group_namespace", "")
        self.declare_parameter(
            "interactive_namespace", "/forbidden_zone_manager/interactive"
        )
        self._config_file = str(self.get_parameter("config_file").value)
        self._active_groups = tuple(
            item.strip()
            for item in str(self.get_parameter("active_groups").value).split(",")
            if item.strip()
        )
        self._model = str(self.get_parameter("model").value).strip().lower()
        if self._model not in SUPPORTED_MODELS:
            raise ValueError(
                f"unsupported model {self._model!r}; expected dm, rs, or piperh"
            )
        configured_user_file = str(self.get_parameter("user_config_file").value).strip()
        self._user_config_file = (
            Path(configured_user_file).expanduser() if configured_user_file else
            Path.home() / ".ros" / "rebotarm" / "forbidden_zones_user.yaml"
        )
        move_group_namespace = str(
            self.get_parameter("move_group_namespace").value
        ).strip("/")
        moveit_prefix = f"/{move_group_namespace}" if move_group_namespace else ""
        self._apply_client = self.create_client(
            ApplyPlanningScene, f"{moveit_prefix}/apply_planning_scene"
        )
        self._get_client = self.create_client(
            GetPlanningScene, f"{moveit_prefix}/get_planning_scene"
        )
        self._status = self.create_publisher(
            String,
            "~/status",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self._reload_service = self.create_service(Trigger, "~/reload", self._reload)
        self._configure_service = self.create_service(
            ConfigureForbiddenZone, "~/configure", self._configure_zone
        )
        self._interactive_server = InteractiveMarkerServer(
            self, str(self.get_parameter("interactive_namespace").value)
        )
        self._interactive_zone_names = {}
        self._interactive_zones = {}
        self._interactive_apply_running = False
        self._interactive_apply_pending = None
        self._interactive_server.applyChanges()
        self._busy = False
        self._loaded_once = False
        self._timer = self.create_timer(0.5, self._try_initial_load)

    def _try_initial_load(self) -> None:
        if self._loaded_once or self._busy:
            return
        if not self._apply_client.service_is_ready():
            self._apply_client.wait_for_service(timeout_sec=0.0)
            return
        if not self._get_client.service_is_ready():
            self._get_client.wait_for_service(timeout_sec=0.0)
            return
        try:
            self._start_apply()
        except (OSError, yaml.YAMLError, ZoneConfigError, MeshLoadError) as exc:
            # Keep the node alive so the operator can fix the file and call reload.
            self._loaded_once = True
            self.get_logger().error(f"forbidden-zone configuration rejected: {exc}")

    def _reload(self, _request, response):
        if self._busy:
            response.success = False
            response.message = "a planning-scene update is already running"
            return response
        if (
            not self._apply_client.service_is_ready()
            or not self._get_client.service_is_ready()
        ):
            response.success = False
            response.message = "MoveIt planning-scene services are not ready"
            return response
        try:
            self._start_apply()
        except (OSError, yaml.YAMLError, ZoneConfigError, MeshLoadError) as exc:
            response.success = False
            response.message = str(exc)
            return response
        response.success = True
        response.message = "forbidden-zone reload started"
        return response

    def _read_user_data(self):
        if not self._user_config_file.exists():
            return {"areas": {}, "groups": {}}
        with self._user_config_file.open(encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
        if (
            not isinstance(data, dict) or
            not isinstance(data.get("areas", {}), dict) or
            not isinstance(data.get("groups", {}), dict)
        ):
            raise ZoneConfigError(
                "user forbidden-zone file must contain areas and groups mappings"
            )
        return data

    def _write_user_data(self, data) -> None:
        self._user_config_file.parent.mkdir(parents=True, exist_ok=True)
        descriptor, temporary_name = tempfile.mkstemp(
            prefix=".forbidden_zones_", suffix=".yaml",
            dir=str(self._user_config_file.parent), text=True,
        )
        try:
            with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
                yaml.safe_dump(data, stream, allow_unicode=True, sort_keys=False)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary_name, self._user_config_file)
        except Exception:
            try:
                os.unlink(temporary_name)
            except OSError:
                pass
            raise

    def _store_mesh(self, source_value: str, scale: float) -> str:
        source = Path(source_value).expanduser().resolve()
        # Validate the source before copying it into managed persistent storage.
        load_mesh(str(source), scale)
        digest = hashlib.sha256()
        with source.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
        mesh_directory = self._user_config_file.parent / "forbidden_zone_meshes"
        mesh_directory.mkdir(parents=True, exist_ok=True)
        destination = mesh_directory / (digest.hexdigest()[:24] + source.suffix.lower())
        if not destination.exists():
            descriptor, temporary_name = tempfile.mkstemp(
                prefix=".mesh_", suffix=source.suffix.lower(), dir=str(mesh_directory)
            )
            os.close(descriptor)
            try:
                shutil.copyfile(source, temporary_name)
                os.replace(temporary_name, destination)
            except Exception:
                try:
                    os.unlink(temporary_name)
                except OSError:
                    pass
                raise
        load_mesh(str(destination), scale)
        return str(destination)

    def _merged_config_data(self):
        if not self._config_file:
            raise ZoneConfigError("config_file must not be empty")
        with Path(self._config_file).open(encoding="utf-8") as stream:
            base = yaml.safe_load(stream) or {}
        if not isinstance(base, dict):
            raise ZoneConfigError("base forbidden-zone configuration must be a mapping")
        merged = dict(base)
        merged["areas"] = dict(base.get("areas", {}))
        merged["groups"] = dict(base.get("groups", {}))
        user = self._read_user_data()
        for name, area in user.get("areas", {}).items():
            merged["areas"][name] = area
        for name, group in user.get("groups", {}).items():
            if name in merged["groups"]:
                raise ZoneConfigError(
                    f"user group {name!r} conflicts with a base safety group"
                )
            merged["groups"][name] = group
        return merged

    def _read_active_zones(self):
        config = load_zone_config(self._merged_config_data())
        return config, select_active_zones(config, self._active_groups)

    @staticmethod
    def _validate_user_name(name: str) -> str:
        result = name.strip()
        if not result or len(result) > 64:
            raise ZoneConfigError("zone name must contain 1 to 64 characters")
        if any(ord(character) < 32 or character in "/\\" for character in result):
            raise ZoneConfigError("zone name must not contain control characters or slashes")
        return result

    def _configure_zone(self, request, response):
        if self._busy:
            response.success = False
            response.message = "a planning-scene update is already running"
            return response
        if (
            not self._apply_client.service_is_ready()
            or not self._get_client.service_is_ready()
        ):
            response.success = False
            response.message = "MoveIt planning-scene services are not ready"
            return response
        try:
            operation = request.operation.strip().lower()
            name = self._validate_user_name(request.name)
            data = self._read_user_data()
            areas = dict(data.get("areas", {}))
            groups = dict(data.get("groups", {}))
            if operation == "remove":
                if name not in areas:
                    raise ZoneConfigError(f"user zone {name!r} does not exist")
                del areas[name]
                groups = {
                    group_name: {
                        **group,
                        "areas": [member for member in group.get("areas", [])
                                  if member != name],
                    }
                    for group_name, group in groups.items()
                    if any(member != name for member in group.get("areas", []))
                }
            elif operation == "upsert":
                shape = request.shape.strip().lower()
                pose = request.pose
                position = [pose.position.x, pose.position.y, pose.position.z]
                if not all(math.isfinite(value) for value in position):
                    raise ZoneConfigError("zone position must contain finite values")
                orientation = rpy_from_quaternion(
                    pose.orientation.x, pose.orientation.y,
                    pose.orientation.z, pose.orientation.w,
                )
                area = {
                    "shape": shape,
                    "pose": {
                        "position": position,
                        "orientation_rpy": list(orientation),
                    },
                    "frame_id": request.frame_id.strip() or "base_link",
                    "enabled": True,
                }
                if shape == "mesh":
                    mesh_path = self._store_mesh(request.mesh_path, request.mesh_scale)
                    area["mesh_path"] = mesh_path
                    area["mesh_scale"] = float(request.mesh_scale)
                else:
                    area["dimensions"] = list(request.dimensions)
                # Validate before committing anything to disk.
                load_zone_config({"areas": {name: area}})
                areas[name] = area
            elif operation == "upsert_group":
                members = [member.strip() for member in request.members if member.strip()]
                if not members:
                    raise ZoneConfigError("a zone group must contain at least one user zone")
                if len(set(members)) != len(members):
                    raise ZoneConfigError("zone group members must not contain duplicates")
                missing = [member for member in members if member not in areas]
                if missing:
                    raise ZoneConfigError(
                        "zone group references unknown user zones: " + ", ".join(missing)
                    )
                with Path(self._config_file).open(encoding="utf-8") as stream:
                    base = yaml.safe_load(stream) or {}
                if name in base.get("groups", {}):
                    raise ZoneConfigError(
                        f"group {name!r} is a fixed base safety group"
                    )
                groups[name] = {"areas": members, "enabled": True}
            elif operation == "remove_group":
                if name not in groups:
                    raise ZoneConfigError(f"user group {name!r} does not exist")
                del groups[name]
            else:
                raise ZoneConfigError(
                    "operation must be upsert, remove, upsert_group, or remove_group"
                )
            data = {"frame_id": "base_link", "areas": areas, "groups": groups}
            load_zone_config(data)
            self._write_user_data(data)
            self._start_apply()
            response.success = True
            response.message = (
                f"{operation.replace('_', ' ')} {name!r} saved; "
                "MoveIt planning-scene refresh started"
            )
        except (OSError, yaml.YAMLError, ZoneConfigError, MeshLoadError) as exc:
            response.success = False
            response.message = str(exc)
        return response

    @staticmethod
    def _zone_with_pose(zone: Zone, pose: Pose) -> Zone:
        position = (pose.position.x, pose.position.y, pose.position.z)
        if not all(math.isfinite(value) for value in position):
            raise ZoneConfigError("zone position must contain finite values")
        orientation = rpy_from_quaternion(
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w,
        )
        return replace(zone, position=position, orientation_rpy=orientation)

    def _refresh_interactive_markers(self, zones: Iterable[Zone]) -> None:
        user_names = set(self._read_user_data().get("areas", {}))
        draggable = [zone for zone in zones if zone.name in user_names]
        self._interactive_server.clear()
        self._interactive_zone_names = {}
        self._interactive_zones = {zone.name: zone for zone in draggable}
        for zone in draggable:
            message = interactive_marker(zone)
            self._interactive_zone_names[message.name] = zone.name
            self._interactive_server.insert(
                message, feedback_callback=self._interactive_feedback
            )
        self._interactive_server.applyChanges()

    def _queue_interactive_apply(self, zone: Zone) -> None:
        """Coalesce drag updates while keeping the collision object in motion."""
        self._interactive_apply_pending = zone
        if self._interactive_apply_running or not self._apply_client.service_is_ready():
            return
        self._send_interactive_apply()

    def _send_interactive_apply(self) -> None:
        zone = self._interactive_apply_pending
        if zone is None:
            return
        self._interactive_apply_pending = None
        self._interactive_apply_running = True
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects.append(collision_object(zone))
        scene.object_colors.append(forbidden_object_color(zone))
        future = self._apply_client.call_async(
            ApplyPlanningScene.Request(scene=scene)
        )
        future.add_done_callback(self._interactive_applied)

    def _interactive_applied(self, future) -> None:
        self._interactive_apply_running = False
        try:
            response = future.result()
            if not response.success:
                self.get_logger().error(
                    "MoveIt rejected a dragged forbidden-zone pose"
                )
        except Exception as exc:
            self.get_logger().error(
                f"could not update dragged forbidden zone: {exc}"
            )
        if self._interactive_apply_pending is not None:
            self._send_interactive_apply()

    def _persist_interactive_pose(self, name: str, zone: Zone) -> None:
        data = self._read_user_data()
        areas = dict(data.get("areas", {}))
        if name not in areas:
            raise ZoneConfigError(f"user zone {name!r} no longer exists")
        area = dict(areas[name])
        area["pose"] = {
            "position": list(zone.position),
            "orientation_rpy": list(zone.orientation_rpy),
        }
        areas[name] = area
        stored = {
            "frame_id": data.get("frame_id", "base_link"),
            "areas": areas,
            "groups": data.get("groups", {}),
        }
        load_zone_config(stored)
        self._write_user_data(stored)

    def _interactive_feedback(self, feedback: InteractiveMarkerFeedback) -> None:
        if feedback.event_type not in (
            InteractiveMarkerFeedback.POSE_UPDATE,
            InteractiveMarkerFeedback.MOUSE_UP,
        ):
            return
        name = self._interactive_zone_names.get(feedback.marker_name)
        previous = self._interactive_zones.get(name)
        if not name or previous is None:
            return
        if self._busy:
            stored_pose = collision_object(previous)
            self._interactive_server.setPose(
                feedback.marker_name,
                stored_pose.primitive_poses[0] if previous.shape != "mesh" else
                stored_pose.mesh_poses[0],
            )
            self._interactive_server.applyChanges()
            self.get_logger().warning(
                "forbidden-zone drag ignored while the planning scene is refreshing"
            )
            return
        try:
            moved = self._zone_with_pose(previous, feedback.pose)
            self._interactive_zones[name] = moved
            self._queue_interactive_apply(moved)
            if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
                self._persist_interactive_pose(name, moved)
                self.get_logger().info(
                    f"saved dragged forbidden zone {name!r} at "
                    f"({moved.position[0]:.3f}, {moved.position[1]:.3f}, "
                    f"{moved.position[2]:.3f})"
                )
                if not self._busy:
                    self._start_apply()
        except (OSError, yaml.YAMLError, ZoneConfigError, MeshLoadError) as exc:
            self.get_logger().error(
                f"rejecting dragged forbidden-zone pose: {exc}"
            )
            self._interactive_server.setPose(
                feedback.marker_name, collision_object(previous).primitive_poses[0]
                if previous.shape != "mesh" else
                collision_object(previous).mesh_poses[0]
            )
            self._interactive_server.applyChanges()

    def _start_apply(self) -> None:
        config, zones = self._read_active_zones()
        self._busy = True
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
        future = self._get_client.call_async(request)
        future.add_done_callback(
            lambda completed: self._got_scene(completed, config, zones)
        )

    def _got_scene(self, future, config, zones: Iterable[Zone]) -> None:
        try:
            response = future.result()
            old_ids = [
                item.id
                for item in response.scene.world.collision_objects
                if item.id.startswith(OBJECT_PREFIX)
            ]
            scene = PlanningScene(is_diff=True)
            scene.world.collision_objects.extend(
                CollisionObject(id=item_id, operation=CollisionObject.REMOVE)
                for item_id in old_ids
            )
            scene.world.collision_objects.extend(
                collision_object(zone) for zone in zones
            )
            scene.object_colors.extend(forbidden_object_color(zone) for zone in zones)
            apply_request = ApplyPlanningScene.Request(scene=scene)
            apply_future = self._apply_client.call_async(apply_request)
            apply_future.add_done_callback(
                lambda completed: self._applied(completed, config, zones)
            )
        except Exception as exc:  # rclpy futures surface service errors here
            self._failed(f"could not read the current planning scene: {exc}")

    def _applied(self, future, config, zones: Iterable[Zone]) -> None:
        try:
            response = future.result()
            if not response.success:
                self._failed("MoveIt rejected the forbidden-zone planning scene")
                return
            zones = tuple(zones)
            self._refresh_interactive_markers(zones)
            zone_names = [zone.name for zone in zones]
            effective_groups = list(self._active_groups) or [
                group.name for group in config.groups.values() if group.enabled
            ]
            user_data = self._read_user_data()
            status = {
                "config_file": self._config_file,
                "user_config_file": str(self._user_config_file),
                "active_groups": effective_groups,
                "loaded_areas": zone_names,
                "defined_areas": list(config.areas),
                "enabled_areas": [
                    area.name for area in config.areas.values() if area.enabled
                ],
                "defined_groups": list(config.groups),
                "user_areas": list(user_data.get("areas", {})),
                "user_groups": list(user_data.get("groups", {})),
                "user_group_members": {
                    name: list(group.get("areas", []))
                    for name, group in user_data.get("groups", {}).items()
                },
                "draggable_areas": list(self._interactive_zones),
                "interactive_namespace": "/forbidden_zone_manager/interactive",
            }
            self._status.publish(String(data=json.dumps(status, ensure_ascii=False)))
            self.get_logger().info(
                f"loaded {len(zone_names)} forbidden zone(s): "
                + (", ".join(zone_names) if zone_names else "none")
            )
            self._loaded_once = True
            self._busy = False
        except Exception as exc:
            self._failed(f"could not apply the planning scene: {exc}")

    def _failed(self, message: str) -> None:
        self._busy = False
        self.get_logger().error(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ForbiddenZoneManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
