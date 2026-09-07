import numpy as np
import pytest
import yaml
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointField
from tf2_ros import TransformException

from antbot_dual_lidar.pcd_io import (
    array_to_pointcloud2,
    create_map_bundle,
    load_metadata,
    load_pcd,
    save_pcd,
    transform_cloud,
    write_cloud_metadata,
)
from antbot_dual_lidar.save_pointcloud import _stored_transform


def sample_cloud():
    dtype = np.dtype([
        ("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
        ("intensity", "<f4"), ("ring", "<u2"),
    ])
    array = np.array([
        (1.0, -2.0, 0.25, 12.5, 3),
        (2.5, 4.0, 1.75, 99.0, 8),
        (-1.0, 3.0, -0.5, 7.25, 15),
    ], dtype=dtype)
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name="ring", offset=16, datatype=PointField.UINT16, count=1),
    ]
    message = array_to_pointcloud2(array, fields, "odom")
    message.header.stamp.sec = 123
    message.header.stamp.nanosec = 456
    return message, array


def identity_transform():
    return {
        "parent_frame": "map",
        "child_frame": "odom",
        "translation": [0.0, 0.0, 0.0],
        "rotation_xyzw": [0.0, 0.0, 0.0, 1.0],
    }


class _TestLogger:
    def __init__(self):
        self.warnings = []
        self.errors = []

    def warning(self, message):
        self.warnings.append(message)

    def error(self, message):
        self.errors.append(message)


class _TestNode:
    def __init__(self, responses):
        self.responses = iter(responses)
        self.lookup_times = []
        self.logger = _TestLogger()
        self.tf_buffer = self

    def lookup_transform(self, _target, _source, lookup_time, timeout):
        del timeout
        self.lookup_times.append(lookup_time)
        response = next(self.responses)
        if isinstance(response, Exception):
            raise response
        return response

    def get_logger(self):
        return self.logger


def test_pointcloud_binary_pcd_round_trip_preserves_xyz_and_intensity(tmp_path):
    message, original = sample_cloud()
    summary = save_pcd(message, tmp_path / "cloud.pcd")
    loaded, fields = load_pcd(tmp_path / "cloud.pcd")

    assert summary["storage_format"] == "binary"
    assert len(loaded) == len(original)
    assert [field.name for field in fields] == [
        "x", "y", "z", "intensity", "ring"
    ]
    for name in ("x", "y", "z", "intensity"):
        assert np.allclose(loaded[name], original[name], atol=1e-6)
    assert np.array_equal(loaded["ring"], original["ring"])


def test_metadata_frame_count_bounds_and_identity_map_odom(tmp_path):
    message, original = sample_cloud()
    summary = save_pcd(message, tmp_path / "cloud.pcd")
    metadata = write_cloud_metadata(
        tmp_path / "cloud_metadata.yaml",
        message,
        summary,
        source_topic="/antbot/lidar/map_points",
        voxel_size=0.05,
        maximum_points=300000,
        transform_to_map=identity_transform(),
    )
    loaded = load_metadata(tmp_path / "cloud_metadata.yaml")

    assert loaded["frame_id"] == "odom"
    assert loaded["point_count"] == len(original)
    assert np.allclose(loaded["bounds_min"], [-1.0, -2.0, -0.5])
    assert np.allclose(loaded["bounds_max"], [2.5, 4.0, 1.75])
    assert loaded["transform_to_map"] == identity_transform()
    assert "LIO" in metadata["warning"]


def test_non_identity_transform_changes_xyz_but_preserves_intensity():
    _message, array = sample_cloud()
    transform = {
        "parent_frame": "map",
        "child_frame": "odom",
        "translation": [10.0, -1.0, 2.0],
        # +90 degrees about Z
        "rotation_xyzw": [0.0, 0.0, 2 ** -0.5, 2 ** -0.5],
    }
    result = transform_cloud(array, transform, "odom", "map")
    expected_first = [12.0, 0.0, 2.25]
    assert np.allclose(
        [result["x"][0], result["y"][0], result["z"][0]],
        expected_first,
        atol=1e-5,
    )
    assert np.array_equal(result["intensity"], array["intensity"])
    assert not np.allclose(result["z"], 0.0)


def test_missing_pcd_has_clear_error(tmp_path):
    with pytest.raises(FileNotFoundError, match="does not exist"):
        load_pcd(tmp_path / "missing.pcd")


def test_corrupt_metadata_has_clear_error(tmp_path):
    path = tmp_path / "cloud_metadata.yaml"
    path.write_text("frame_id: [unterminated", encoding="utf-8")
    with pytest.raises(ValueError, match="cannot parse"):
        load_metadata(path)


def test_transform_is_required_when_frames_differ():
    _message, array = sample_cloud()
    with pytest.raises(ValueError, match="no transform"):
        transform_cloud(array, None, "odom", "map")


def test_saver_falls_back_to_latest_tf_when_cloud_stamp_predates_buffer():
    message, _array = sample_cloud()
    stamped = TransformStamped()
    stamped.header.frame_id = "map"
    stamped.child_frame_id = "odom"
    stamped.transform.translation.x = 1.25
    stamped.transform.translation.y = -0.5
    stamped.transform.rotation.w = 1.0
    node = _TestNode([
        TransformException("cloud time is older than TF buffer"),
        stamped,
    ])

    transform = _stored_transform(node, message, "map")

    assert len(node.lookup_times) == 2
    assert node.lookup_times[0].nanoseconds == 123000000456
    assert node.lookup_times[1].nanoseconds == 0
    assert transform == {
        "parent_frame": "map",
        "child_frame": "odom",
        "translation": [1.25, -0.5, 0.0],
        "rotation_xyzw": [0.0, 0.0, 0.0, 1.0],
    }
    assert "falling back to the latest" in node.logger.warnings[0]
    assert not node.logger.errors


def test_map_bundle_copies_only_present_files_and_rewrites_image(tmp_path):
    source = tmp_path / "source"
    source.mkdir()
    (source / "original.pgm").write_bytes(b"P5\n1 1\n255\n\x00")
    (source / "original.yaml").write_text(
        "image: original.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n",
        encoding="utf-8",
    )
    (source / "waypoints.xml").write_text(
        '<waypoints><waypoint name="01" x="1" y="2" yaw="0"/></waypoints>',
        encoding="utf-8",
    )
    output = tmp_path / "home_01"
    manifest = create_map_bundle(
        output,
        "home_01",
        map_yaml=source / "original.yaml",
        waypoints=source / "waypoints.xml",
    )
    assert manifest["occupancy_map_yaml"] == "map.yaml"
    assert manifest["pointcloud_file"] is None
    assert manifest["waypoint_file"] == "waypoints.xml"
    assert (output / "map.pgm").is_file()
    assert yaml.safe_load((output / "map.yaml").read_text())["image"] == "map.pgm"
