import pytest

from rebot_xbox_hardware.forbidden_zone_config import (
    load_zone_config,
    select_active_zones,
    ZoneConfigError,
)


def _config():
    return load_zone_config(
        {
            "frame_id": "base_link",
            "areas": {
                "wall": {
                    "shape": "box",
                    "dimensions": [0.1, 1.0, 1.0],
                    "pose": {"position": [-0.4, 0.0, 0.5]},
                },
                "fixture": {
                    "shape": "cylinder",
                    "dimensions": [0.3, 0.08],
                    "enabled": False,
                },
                "ceiling": {"shape": "sphere", "dimensions": [0.2]},
            },
            "groups": {
                "cell": {"areas": ["wall", "fixture"]},
                "alternate": {"areas": ["ceiling"], "enabled": False},
            },
        }
    )


def test_enabled_groups_select_union_and_skip_disabled_area():
    config = _config()
    assert [zone.name for zone in select_active_zones(config)] == ["wall"]


def test_explicit_group_selection_overrides_group_enabled_flag():
    config = _config()
    assert [zone.name for zone in select_active_zones(config, ["alternate"])] == [
        "ceiling"
    ]


@pytest.mark.parametrize(
    "area, message",
    [
        ({"shape": "box", "dimensions": [1.0, 2.0]}, "exactly 3"),
        ({"shape": "sphere", "dimensions": [-1.0]}, "positive"),
        ({"shape": "mesh", "mesh_path": ""}, "non-empty string"),
    ],
)
def test_invalid_shape_definition_is_rejected(area, message):
    with pytest.raises(ZoneConfigError, match=message):
        load_zone_config({"areas": {"bad": area}})


def test_unknown_group_member_and_selected_group_are_rejected():
    with pytest.raises(ZoneConfigError, match="unknown areas"):
        load_zone_config(
            {
                "areas": {},
                "groups": {"cell": {"areas": ["missing"]}},
            }
        )
    with pytest.raises(ZoneConfigError, match="unknown selected groups"):
        select_active_zones(_config(), ["missing"])


def test_mesh_definition_records_path_and_scale():
    config = load_zone_config({"areas": {"fixture_mesh": {
        "shape": "mesh", "mesh_path": "/tmp/fixture.stl", "mesh_scale": 0.001,
    }}})
    zone = config.areas["fixture_mesh"]
    assert zone.dimensions == ()
    assert zone.mesh_path == "/tmp/fixture.stl"
    assert zone.mesh_scale == pytest.approx(0.001)
