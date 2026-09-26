"""Validation and selection for named MoveIt forbidden-zone configuration."""

from dataclasses import dataclass
import math
from typing import Any, Dict, Iterable, Mapping, Sequence, Tuple


class ZoneConfigError(ValueError):
    """Raised when a forbidden-zone configuration is unsafe or ambiguous."""


@dataclass(frozen=True)
class Zone:
    name: str
    shape: str
    dimensions: Tuple[float, ...]
    position: Tuple[float, float, float]
    orientation_rpy: Tuple[float, float, float]
    frame_id: str
    enabled: bool = True
    mesh_path: str = ""
    mesh_scale: float = 1.0


@dataclass(frozen=True)
class ZoneGroup:
    name: str
    areas: Tuple[str, ...]
    enabled: bool = True


@dataclass(frozen=True)
class ZoneConfig:
    frame_id: str
    areas: Mapping[str, Zone]
    groups: Mapping[str, ZoneGroup]


_DIMENSION_COUNTS = {
    "box": 3,
    "sphere": 1,
    "cylinder": 2,
    "cone": 2,
    "mesh": 0,
}


def _mapping(value: Any, field: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ZoneConfigError(f"{field} must be a mapping")
    return value


def _name(value: Any, field: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ZoneConfigError(f"{field} must be a non-empty string")
    return value.strip()


def _bool(value: Any, field: str) -> bool:
    if not isinstance(value, bool):
        raise ZoneConfigError(f"{field} must be true or false")
    return value


def _finite_vector(value: Any, size: int, field: str) -> Tuple[float, ...]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise ZoneConfigError(f"{field} must contain {size} numbers")
    if len(value) != size:
        raise ZoneConfigError(f"{field} must contain exactly {size} numbers")
    try:
        result = tuple(float(item) for item in value)
    except (TypeError, ValueError) as exc:
        raise ZoneConfigError(f"{field} must contain only numbers") from exc
    if not all(math.isfinite(item) for item in result):
        raise ZoneConfigError(f"{field} must contain only finite numbers")
    return result


def load_zone_config(data: Any) -> ZoneConfig:
    """Parse already-loaded YAML data and reject unsafe definitions."""
    root = _mapping(data, "configuration")
    frame_id = _name(root.get("frame_id", "base_link"), "frame_id")
    raw_areas = _mapping(root.get("areas", {}), "areas")
    raw_groups = _mapping(root.get("groups", {}), "groups")

    areas: Dict[str, Zone] = {}
    for raw_name, raw_area in raw_areas.items():
        name = _name(raw_name, "area name")
        area = _mapping(raw_area, f"area {name!r}")
        shape = _name(area.get("shape"), f"area {name!r}.shape").lower()
        if shape not in _DIMENSION_COUNTS:
            supported = ", ".join(sorted(_DIMENSION_COUNTS))
            raise ZoneConfigError(
                f"area {name!r}.shape must be one of: {supported}"
            )
        if shape == "mesh":
            dimensions = ()
            mesh_path = _name(area.get("mesh_path"), f"area {name!r}.mesh_path")
            try:
                mesh_scale = float(area.get("mesh_scale", 1.0))
            except (TypeError, ValueError) as exc:
                raise ZoneConfigError(
                    f"area {name!r}.mesh_scale must be a number"
                ) from exc
            if not math.isfinite(mesh_scale) or mesh_scale <= 0.0 or mesh_scale > 1000.0:
                raise ZoneConfigError(
                    f"area {name!r}.mesh_scale must be in (0, 1000]"
                )
        else:
            dimensions = _finite_vector(
                area.get("dimensions"),
                _DIMENSION_COUNTS[shape],
                f"area {name!r}.dimensions",
            )
            if any(item <= 0.0 for item in dimensions):
                raise ZoneConfigError(f"area {name!r}.dimensions must all be positive")
            mesh_path = ""
            mesh_scale = 1.0
        pose = _mapping(area.get("pose", {}), f"area {name!r}.pose")
        position = _finite_vector(
            pose.get("position", [0.0, 0.0, 0.0]),
            3,
            f"area {name!r}.pose.position",
        )
        orientation_rpy = _finite_vector(
            pose.get("orientation_rpy", [0.0, 0.0, 0.0]),
            3,
            f"area {name!r}.pose.orientation_rpy",
        )
        area_frame = _name(
            area.get("frame_id", frame_id), f"area {name!r}.frame_id"
        )
        enabled = _bool(area.get("enabled", True), f"area {name!r}.enabled")
        areas[name] = Zone(
            name=name,
            shape=shape,
            dimensions=dimensions,
            position=position,
            orientation_rpy=orientation_rpy,
            frame_id=area_frame,
            enabled=enabled,
            mesh_path=mesh_path,
            mesh_scale=mesh_scale,
        )

    groups: Dict[str, ZoneGroup] = {}
    for raw_name, raw_group in raw_groups.items():
        name = _name(raw_name, "group name")
        group = _mapping(raw_group, f"group {name!r}")
        raw_members = group.get("areas")
        if isinstance(raw_members, (str, bytes)) or not isinstance(
            raw_members, Sequence
        ):
            raise ZoneConfigError(f"group {name!r}.areas must be a list")
        members = tuple(
            _name(item, f"group {name!r}.areas") for item in raw_members
        )
        if len(set(members)) != len(members):
            raise ZoneConfigError(f"group {name!r}.areas contains duplicates")
        missing = [member for member in members if member not in areas]
        if missing:
            raise ZoneConfigError(
                f"group {name!r} references unknown areas: {', '.join(missing)}"
            )
        enabled = _bool(group.get("enabled", True), f"group {name!r}.enabled")
        groups[name] = ZoneGroup(name=name, areas=members, enabled=enabled)

    return ZoneConfig(frame_id=frame_id, areas=areas, groups=groups)


def select_active_zones(
    config: ZoneConfig, selected_groups: Iterable[str] = ()
) -> Tuple[Zone, ...]:
    """Return enabled zones in stable order for the selected named groups.

    With no explicit selection, all enabled groups are active. Enabled areas that
    do not belong to any group are always active. An explicit selection overrides
    group ``enabled`` flags, which is useful for choosing a site from launch.
    """
    requested = tuple(name.strip() for name in selected_groups if name.strip())
    unknown = [name for name in requested if name not in config.groups]
    if unknown:
        raise ZoneConfigError(f"unknown selected groups: {', '.join(unknown)}")

    grouped_names = {
        area_name for group in config.groups.values() for area_name in group.areas
    }
    active_names = {
        name
        for name, area in config.areas.items()
        if area.enabled and name not in grouped_names
    }
    active_groups = (
        [config.groups[name] for name in requested]
        if requested
        else [group for group in config.groups.values() if group.enabled]
    )
    for group in active_groups:
        active_names.update(
            area_name
            for area_name in group.areas
            if config.areas[area_name].enabled
        )
    return tuple(
        area for name, area in config.areas.items() if name in active_names
    )
