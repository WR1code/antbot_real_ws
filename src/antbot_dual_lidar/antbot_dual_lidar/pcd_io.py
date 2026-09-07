"""Binary PCD, metadata, transforms, and map-bundle helpers.

The PCD implementation deliberately keeps every numeric PointCloud2 field
instead of reducing a cloud to XYZ.  It writes packed PCD ``binary`` records;
the source message may itself contain padding or organized-row padding.
"""

from __future__ import annotations

from datetime import datetime, timezone
import os
from pathlib import Path
import re
import shutil
import tempfile

import numpy as np
import yaml
from sensor_msgs.msg import PointCloud2, PointField


FORMAT_VERSION = 1
ODOM_WARNING = (
    "当前点云由 odom 位姿累计得到，不是真正经过 LIO 闭环优化的三维 "
    "SLAM 地图，长距离运行时可能存在漂移。"
)

_POINTFIELD_DTYPES = {
    PointField.INT8: ("i1", "I", 1),
    PointField.UINT8: ("u1", "U", 1),
    PointField.INT16: ("i2", "I", 2),
    PointField.UINT16: ("u2", "U", 2),
    PointField.INT32: ("i4", "I", 4),
    PointField.UINT32: ("u4", "U", 4),
    PointField.FLOAT32: ("f4", "F", 4),
    PointField.FLOAT64: ("f8", "F", 8),
}
_PCD_TO_POINTFIELD = {
    ("I", 1): PointField.INT8,
    ("U", 1): PointField.UINT8,
    ("I", 2): PointField.INT16,
    ("U", 2): PointField.UINT16,
    ("I", 4): PointField.INT32,
    ("U", 4): PointField.UINT32,
    ("F", 4): PointField.FLOAT32,
    ("F", 8): PointField.FLOAT64,
}
_SAFE_NAME = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _atomic_bytes(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def atomic_yaml(path: Path, data: dict) -> None:
    payload = yaml.safe_dump(
        data, allow_unicode=True, sort_keys=False
    ).encode("utf-8")
    _atomic_bytes(Path(path), payload)


def _validated_fields(fields):
    result = []
    names = set()
    for field in fields:
        if not _SAFE_NAME.match(field.name):
            raise ValueError(f"unsupported PointCloud2 field name: {field.name!r}")
        if field.name in names:
            raise ValueError(f"duplicate PointCloud2 field: {field.name}")
        if field.datatype not in _POINTFIELD_DTYPES:
            raise ValueError(
                f"unsupported datatype {field.datatype} for field {field.name}"
            )
        count = int(field.count) or 1
        if count < 1:
            raise ValueError(f"invalid count for field {field.name}: {field.count}")
        result.append((field, count))
        names.add(field.name)
    if not result:
        raise ValueError("PointCloud2 has no fields")
    return result


def pointcloud2_to_array(message: PointCloud2):
    """Return a packed structured array and normalized PointField list."""
    fields = _validated_fields(message.fields)
    byte_order = ">" if message.is_bigendian else "<"
    names, formats, offsets = [], [], []
    normalized = []
    packed_offset = 0
    for field, count in fields:
        code, _pcd_type, size = _POINTFIELD_DTYPES[field.datatype]
        value_format = np.dtype(byte_order + code)
        if count > 1:
            value_format = np.dtype((value_format, (count,)))
        names.append(field.name)
        formats.append(value_format)
        offsets.append(int(field.offset))
        normalized.append(
            PointField(
                name=field.name,
                offset=packed_offset,
                datatype=field.datatype,
                count=count,
            )
        )
        packed_offset += size * count

    width, height = int(message.width), int(message.height)
    point_count = width * height
    expected = (height - 1) * int(message.row_step) + width * int(message.point_step)
    if len(message.data) < expected:
        raise ValueError(
            f"PointCloud2 data is truncated: {len(message.data)} < {expected}"
        )
    source_dtype = np.dtype(
        {
            "names": names,
            "formats": formats,
            "offsets": offsets,
            "itemsize": int(message.point_step),
        }
    )
    source = np.ndarray(
        shape=(height, width),
        dtype=source_dtype,
        buffer=message.data,
        strides=(int(message.row_step), int(message.point_step)),
    ).reshape(-1)

    packed_formats = []
    for field, count in fields:
        code, _pcd_type, _size = _POINTFIELD_DTYPES[field.datatype]
        value_format = np.dtype("<" + code)
        packed_formats.append(
            np.dtype((value_format, (count,))) if count > 1 else value_format
        )
    packed = np.empty(point_count, dtype=np.dtype(list(zip(names, packed_formats))))
    if point_count == 0:
        return packed, normalized
    for name in names:
        packed[name] = source[name]
    return packed, normalized


def save_pcd(message: PointCloud2, path, storage_format: str = "binary") -> dict:
    """Atomically save a PointCloud2 as a field-preserving binary PCD."""
    if storage_format != "binary":
        raise ValueError("only PCD binary storage is supported")
    array, fields = pointcloud2_to_array(message)
    field_parts, sizes, types, counts = [], [], [], []
    for field in fields:
        _code, pcd_type, size = _POINTFIELD_DTYPES[field.datatype]
        field_parts.append(field.name)
        sizes.append(str(size))
        types.append(pcd_type)
        counts.append(str(int(field.count) or 1))
    header = "\n".join(
        [
            "# .PCD v0.7 - Point Cloud Data file format",
            "VERSION 0.7",
            f"FIELDS {' '.join(field_parts)}",
            f"SIZE {' '.join(sizes)}",
            f"TYPE {' '.join(types)}",
            f"COUNT {' '.join(counts)}",
            f"WIDTH {len(array)}",
            "HEIGHT 1",
            "VIEWPOINT 0 0 0 1 0 0 0",
            f"POINTS {len(array)}",
            "DATA binary",
            "",
        ]
    ).encode("ascii")
    _atomic_bytes(Path(path), header + array.tobytes(order="C"))
    return {
        "point_count": len(array),
        "fields": [
            {
                "name": field.name,
                "offset": int(field.offset),
                "datatype": int(field.datatype),
                "count": int(field.count) or 1,
            }
            for field in fields
        ],
        "point_step": int(array.dtype.itemsize),
        "storage_format": storage_format,
    }


def _read_pcd_header(stream):
    parsed = {}
    while True:
        line = stream.readline()
        if not line:
            raise ValueError("PCD header ended before DATA")
        try:
            text = line.decode("ascii").strip()
        except UnicodeDecodeError as error:
            raise ValueError("PCD header is not ASCII") from error
        if not text or text.startswith("#"):
            continue
        key, *values = text.split()
        parsed[key.upper()] = values
        if key.upper() == "DATA":
            break
    required = ("FIELDS", "SIZE", "TYPE", "WIDTH", "HEIGHT", "POINTS", "DATA")
    missing = [key for key in required if key not in parsed]
    if missing:
        raise ValueError(f"PCD header missing: {', '.join(missing)}")
    return parsed


def load_pcd(path):
    """Load a binary PCD and return ``(structured_array, PointFields)``."""
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(f"point cloud file does not exist: {path}")
    with path.open("rb") as stream:
        header = _read_pcd_header(stream)
        data_mode = header["DATA"][0].lower()
        if data_mode != "binary":
            raise ValueError(
                f"unsupported PCD DATA mode {data_mode!r}; expected 'binary'"
            )
        names = header["FIELDS"]
        try:
            sizes = [int(value) for value in header["SIZE"]]
            types = [value.upper() for value in header["TYPE"]]
            counts = [
                int(value) for value in header.get("COUNT", ["1"] * len(names))
            ]
            points = int(header["POINTS"][0])
            width = int(header["WIDTH"][0])
            height = int(header["HEIGHT"][0])
        except (ValueError, IndexError) as error:
            raise ValueError(f"invalid numeric PCD header in {path}") from error
        if not (len(names) == len(sizes) == len(types) == len(counts)):
            raise ValueError("PCD FIELDS/SIZE/TYPE/COUNT lengths differ")
        if points != width * height or points < 0:
            raise ValueError("PCD POINTS does not match WIDTH * HEIGHT")

        formats, fields = [], []
        offset = 0
        for name, size, pcd_type, count in zip(names, sizes, types, counts):
            if not _SAFE_NAME.match(name) or count < 1:
                raise ValueError(f"unsupported PCD field declaration: {name}")
            datatype = _PCD_TO_POINTFIELD.get((pcd_type, size))
            if datatype is None:
                raise ValueError(f"unsupported PCD field type: {pcd_type}{size}")
            code = _POINTFIELD_DTYPES[datatype][0]
            value_format = np.dtype("<" + code)
            formats.append(
                np.dtype((value_format, (count,))) if count > 1 else value_format
            )
            fields.append(
                PointField(
                    name=name,
                    offset=offset,
                    datatype=datatype,
                    count=count,
                )
            )
            offset += size * count
        dtype = np.dtype(list(zip(names, formats)))
        payload = stream.read()
        expected = points * dtype.itemsize
        if len(payload) != expected:
            raise ValueError(
                f"PCD binary payload size mismatch: {len(payload)} != {expected}"
            )
        return np.frombuffer(payload, dtype=dtype).copy(), fields


def array_to_pointcloud2(array, fields, frame_id: str, stamp=None) -> PointCloud2:
    message = PointCloud2()
    message.header.frame_id = frame_id
    if stamp is not None:
        message.header.stamp = stamp
    message.height = 1
    message.width = len(array)
    message.fields = list(fields)
    message.is_bigendian = False
    message.point_step = int(array.dtype.itemsize)
    message.row_step = message.point_step * message.width
    message.data = array.tobytes(order="C")
    message.is_dense = all(
        name not in array.dtype.names or np.isfinite(array[name]).all()
        for name in ("x", "y", "z")
    )
    return message


def load_metadata(path) -> dict:
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(f"point cloud metadata does not exist: {path}")
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise ValueError(f"cannot parse point cloud metadata {path}: {error}") from error
    if not isinstance(data, dict):
        raise ValueError(f"point cloud metadata must be a mapping: {path}")
    for key in ("format_version", "frame_id", "point_count", "fields"):
        if key not in data:
            raise ValueError(f"point cloud metadata missing {key!r}: {path}")
    if not isinstance(data["frame_id"], str) or not data["frame_id"]:
        raise ValueError("metadata frame_id must be a non-empty string")
    return data


def _quaternion_matrix(rotation):
    quaternion = np.asarray(rotation, dtype=np.float64)
    if quaternion.shape != (4,) or not np.isfinite(quaternion).all():
        raise ValueError("rotation_xyzw must contain four finite numbers")
    norm = np.linalg.norm(quaternion)
    if norm < 1e-12:
        raise ValueError("rotation quaternion has zero length")
    x, y, z, w = quaternion / norm
    return np.array(
        [
            [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
            [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
            [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
        ],
        dtype=np.float64,
    )


def transform_cloud(array, transform: dict, source_frame: str, target_frame: str):
    """Numerically apply ``target <- source`` while retaining other fields."""
    if not isinstance(transform, dict):
        raise ValueError(
            f"no transform available from {source_frame!r} to {target_frame!r}"
        )
    if (
        transform.get("child_frame") != source_frame
        or transform.get("parent_frame") != target_frame
    ):
        raise ValueError(
            "stored transform frames do not match requested conversion: "
            f"{transform.get('parent_frame')} <- {transform.get('child_frame')}, "
            f"requested {target_frame} <- {source_frame}"
        )
    for coordinate in ("x", "y", "z"):
        if coordinate not in (array.dtype.names or ()):
            raise ValueError(f"PCD is missing required coordinate field {coordinate!r}")
    translation = np.asarray(transform.get("translation"), dtype=np.float64)
    if translation.shape != (3,) or not np.isfinite(translation).all():
        raise ValueError("transform translation must contain three finite numbers")
    matrix = _quaternion_matrix(transform.get("rotation_xyzw"))
    xyz = np.column_stack((array["x"], array["y"], array["z"])).astype(np.float64)
    transformed = xyz @ matrix.T + translation
    output = array.copy()
    output["x"], output["y"], output["z"] = transformed.T
    return output


def cloud_bounds(array):
    names = array.dtype.names or ()
    if not all(name in names for name in ("x", "y", "z")) or not len(array):
        return [None, None, None], [None, None, None]
    xyz = np.column_stack((array["x"], array["y"], array["z"])).astype(np.float64)
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    if not len(xyz):
        return [None, None, None], [None, None, None]
    return xyz.min(axis=0).tolist(), xyz.max(axis=0).tolist()


def write_cloud_metadata(path, message, pcd_summary, *, source_topic,
                         voxel_size, maximum_points, transform_to_map) -> dict:
    array, _fields = pointcloud2_to_array(message)
    bounds_min, bounds_max = cloud_bounds(array)
    stamp = message.header.stamp
    metadata = {
        "format_version": FORMAT_VERSION,
        "source_topic": source_topic,
        "frame_id": message.header.frame_id,
        "stamp": {"sec": int(stamp.sec), "nanosec": int(stamp.nanosec)},
        "point_count": int(pcd_summary["point_count"]),
        "fields": pcd_summary["fields"],
        "voxel_size": float(voxel_size),
        "maximum_points": int(maximum_points),
        "bounds_min": bounds_min,
        "bounds_max": bounds_max,
        "storage_format": pcd_summary["storage_format"],
        "saved_at": utc_now(),
        "transform_to_map": transform_to_map,
        "accumulation_method": (
            "bounded voxel accumulation of dual lidar points transformed by odom pose"
        ),
        "warning": ODOM_WARNING,
    }
    atomic_yaml(Path(path), metadata)
    return metadata


def validate_map_name(map_name: str) -> str:
    if not map_name or map_name in (".", "..") or "/" in map_name or "\\" in map_name:
        raise ValueError("map_name must be one non-empty directory name")
    return map_name


def write_manifest(directory, map_name: str, *, navigation_frame="map",
                   cloud_frame=None, source_topic=None, transform=None,
                   notes=None) -> dict:
    directory = Path(directory)
    validate_map_name(map_name)
    existing = {}
    manifest_path = directory / "manifest.yaml"
    if manifest_path.is_file():
        try:
            candidate = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
            if isinstance(candidate, dict):
                existing = candidate
        except yaml.YAMLError:
            pass
    def present(name):
        return name if (directory / name).is_file() else None

    occupancy = present("map.yaml")
    pointcloud = present("cloud.pcd")
    metadata = present("cloud_metadata.yaml")
    waypoint = present("waypoints.xml")
    manifest = {
        "format_version": FORMAT_VERSION,
        "map_name": map_name,
        "created_at": existing.get("created_at", utc_now()),
        "occupancy_map_yaml": occupancy,
        "pointcloud_file": pointcloud,
        "pointcloud_metadata_file": metadata,
        "waypoint_file": waypoint,
        "navigation_frame": navigation_frame,
        "cloud_frame": cloud_frame,
        "source_topic": source_topic,
        "map_to_cloud_transform": transform,
        "notes": notes or existing.get("notes", ""),
    }
    atomic_yaml(manifest_path, manifest)
    return manifest


def _atomic_copy(source: Path, destination: Path) -> None:
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{destination.name}.", suffix=".tmp", dir=destination.parent
    )
    os.close(descriptor)
    try:
        shutil.copy2(source, temporary)
        os.replace(temporary, destination)
    except BaseException:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def create_map_bundle(output, map_name, *, map_yaml=None, cloud=None,
                      metadata=None, waypoints=None, notes="") -> dict:
    """Copy supplied artifacts into a standard map directory and make manifest."""
    output = Path(output)
    validate_map_name(map_name)
    supplied = {
        "map.yaml": map_yaml,
        "cloud.pcd": cloud,
        "cloud_metadata.yaml": metadata,
        "waypoints.xml": waypoints,
    }
    for source in supplied.values():
        if source is not None and not Path(source).is_file():
            raise FileNotFoundError(f"bundle input does not exist: {source}")
    output.mkdir(parents=True, exist_ok=True)
    for destination_name, source in supplied.items():
        if source is not None:
            _atomic_copy(Path(source), output / destination_name)

    if map_yaml is not None:
        map_data = yaml.safe_load((output / "map.yaml").read_text(encoding="utf-8"))
        if not isinstance(map_data, dict) or "image" not in map_data:
            raise ValueError("occupancy map YAML must contain an image entry")
        source_image = Path(str(map_data["image"]))
        if not source_image.is_absolute():
            source_image = Path(map_yaml).parent / source_image
        if not source_image.is_file():
            raise FileNotFoundError(f"occupancy map image does not exist: {source_image}")
        image_name = "map" + source_image.suffix.lower()
        _atomic_copy(source_image, output / image_name)
        map_data["image"] = image_name
        atomic_yaml(output / "map.yaml", map_data)

    cloud_frame = source_topic = transform = None
    metadata_path = output / "cloud_metadata.yaml"
    if metadata_path.is_file():
        loaded = load_metadata(metadata_path)
        cloud_frame = loaded["frame_id"]
        source_topic = loaded.get("source_topic")
        transform = loaded.get("transform_to_map")
    return write_manifest(
        output, map_name, cloud_frame=cloud_frame, source_topic=source_topic,
        transform=transform, notes=notes
    )
