"""Small, dependency-free STL/OBJ loader for forbidden-zone collision meshes."""

import math
from pathlib import Path
import struct

from geometry_msgs.msg import Point
from shape_msgs.msg import Mesh, MeshTriangle


MAX_FILE_BYTES = 50 * 1024 * 1024
MAX_TRIANGLES = 400_000
MAX_VERTICES = 1_200_000


class MeshLoadError(ValueError):
    """Raised when a mesh is unsupported, malformed, or unreasonably large."""


def _point(values, scale: float) -> Point:
    scaled = tuple(float(value) * scale for value in values)
    if not all(math.isfinite(value) for value in scaled):
        raise MeshLoadError("mesh contains non-finite vertices")
    return Point(x=scaled[0], y=scaled[1], z=scaled[2])


def _validate_path(path_value: str) -> Path:
    path = Path(path_value).expanduser().resolve()
    if path.suffix.lower() not in {".stl", ".obj"}:
        raise MeshLoadError("mesh file must use .stl or .obj")
    if not path.is_file():
        raise MeshLoadError(f"mesh file does not exist: {path}")
    if path.stat().st_size <= 0 or path.stat().st_size > MAX_FILE_BYTES:
        raise MeshLoadError("mesh file must be between 1 byte and 50 MiB")
    return path


def _load_binary_stl(data: bytes, scale: float) -> Mesh | None:
    if len(data) < 84:
        return None
    count = struct.unpack_from("<I", data, 80)[0]
    if len(data) != 84 + count * 50:
        return None
    if count == 0 or count > MAX_TRIANGLES:
        raise MeshLoadError("STL triangle count is empty or exceeds the safety limit")
    vertices = []
    triangles = []
    for index in range(count):
        offset = 84 + index * 50 + 12
        first = len(vertices)
        for vertex_index in range(3):
            values = struct.unpack_from("<fff", data, offset + 12 * vertex_index)
            vertices.append(_point(values, scale))
        triangles.append(MeshTriangle(vertex_indices=[first, first + 1, first + 2]))
    return Mesh(vertices=vertices, triangles=triangles)


def _load_ascii_stl(data: bytes, scale: float) -> Mesh:
    try:
        text = data.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise MeshLoadError("STL is neither valid binary nor UTF-8 ASCII") from exc
    coordinates = []
    for line in text.splitlines():
        words = line.strip().split()
        if len(words) == 4 and words[0].lower() == "vertex":
            try:
                coordinates.append(tuple(float(value) for value in words[1:]))
            except ValueError as exc:
                raise MeshLoadError("ASCII STL contains an invalid vertex") from exc
    if not coordinates or len(coordinates) % 3:
        raise MeshLoadError("ASCII STL contains no complete triangles")
    count = len(coordinates) // 3
    if count > MAX_TRIANGLES:
        raise MeshLoadError("STL triangle count exceeds the safety limit")
    vertices = [_point(value, scale) for value in coordinates]
    triangles = [
        MeshTriangle(vertex_indices=[index, index + 1, index + 2])
        for index in range(0, len(vertices), 3)
    ]
    return Mesh(vertices=vertices, triangles=triangles)


def _load_obj(data: bytes, scale: float) -> Mesh:
    try:
        text = data.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise MeshLoadError("OBJ must be UTF-8 text") from exc
    coordinates = []
    face_indices = []
    for line_number, line in enumerate(text.splitlines(), 1):
        words = line.strip().split()
        if not words or words[0].startswith("#"):
            continue
        if words[0] == "v" and len(words) >= 4:
            try:
                coordinates.append(tuple(float(value) for value in words[1:4]))
            except ValueError as exc:
                raise MeshLoadError(f"OBJ line {line_number} has an invalid vertex") from exc
            if len(coordinates) > MAX_VERTICES:
                raise MeshLoadError("OBJ vertex count exceeds the safety limit")
        elif words[0] == "f" and len(words) >= 4:
            polygon = []
            for token in words[1:]:
                try:
                    raw = int(token.split("/", 1)[0])
                except ValueError as exc:
                    raise MeshLoadError(f"OBJ line {line_number} has an invalid face") from exc
                resolved = raw - 1 if raw > 0 else len(coordinates) + raw
                if resolved < 0 or resolved >= len(coordinates):
                    raise MeshLoadError(f"OBJ line {line_number} references a missing vertex")
                polygon.append(resolved)
            for index in range(1, len(polygon) - 1):
                face_indices.append((polygon[0], polygon[index], polygon[index + 1]))
                if len(face_indices) > MAX_TRIANGLES:
                    raise MeshLoadError("OBJ triangle count exceeds the safety limit")
    if not coordinates or not face_indices:
        raise MeshLoadError("OBJ contains no vertices or faces")
    return Mesh(
        vertices=[_point(value, scale) for value in coordinates],
        triangles=[MeshTriangle(vertex_indices=list(face)) for face in face_indices],
    )


def load_mesh(path_value: str, scale: float = 1.0) -> Mesh:
    """Load an STL/OBJ collision mesh with a uniform unit conversion scale."""
    if not math.isfinite(scale) or scale <= 0.0 or scale > 1000.0:
        raise MeshLoadError("mesh scale must be finite and in (0, 1000]")
    path = _validate_path(path_value)
    data = path.read_bytes()
    if path.suffix.lower() == ".obj":
        return _load_obj(data, scale)
    return _load_binary_stl(data, scale) or _load_ascii_stl(data, scale)
