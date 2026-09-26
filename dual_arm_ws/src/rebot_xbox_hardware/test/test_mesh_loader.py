import struct

import pytest

from rebot_xbox_hardware.mesh_loader import load_mesh, MeshLoadError


def test_load_obj_triangulates_polygon_and_scales(tmp_path):
    path = tmp_path / "zone.obj"
    path.write_text("v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nf 1 2 3 4\n")
    mesh = load_mesh(str(path), 0.001)
    assert len(mesh.vertices) == 4
    assert len(mesh.triangles) == 2
    assert mesh.vertices[1].x == pytest.approx(0.001)


def test_load_binary_stl(tmp_path):
    path = tmp_path / "zone.stl"
    triangle = struct.pack(
        "<12fH", 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0
    )
    path.write_bytes(bytes(80) + struct.pack("<I", 1) + triangle)
    mesh = load_mesh(str(path))
    assert len(mesh.vertices) == 3
    assert len(mesh.triangles) == 1


def test_rejects_unsupported_or_invalid_mesh(tmp_path):
    bad = tmp_path / "zone.ply"
    bad.write_text("x")
    with pytest.raises(MeshLoadError, match=".stl or .obj"):
        load_mesh(str(bad))
