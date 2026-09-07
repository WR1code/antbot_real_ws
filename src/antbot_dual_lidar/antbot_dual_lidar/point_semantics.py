"""Pure semantic gates preventing native and world point-cloud aliasing."""

from dataclasses import dataclass


@dataclass(frozen=True)
class PointSemantics:
    coordinates: str
    motion_compensation: str
    route: str


def classify_gmo_semantics(frame_of_reference, motion_compensation):
    frame = str(frame_of_reference).upper()
    motion = str(motion_compensation).upper()
    if frame == "SENSOR" and motion == "NONCOMPENSATED":
        return PointSemantics("per_ray_sensor_frame", motion, "native_raw")
    if frame == "WORLD":
        return PointSemantics("world_endpoint", motion, "world_reference")
    raise ValueError(
        f"unsupported GMO point semantics: frame={frame} motion={motion}"
    )


def require_topic_route(topic, route):
    if topic.endswith("/points_raw_native") and route != "native_raw":
        raise ValueError("world/reference endpoints cannot be published as native raw")
    if "world_reference" in topic and route != "world_reference":
        raise ValueError("native sensor coordinates cannot be labelled world reference")
    return True
