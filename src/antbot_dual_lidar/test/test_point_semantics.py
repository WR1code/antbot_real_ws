import pytest

from antbot_dual_lidar.point_semantics import (
    classify_gmo_semantics,
    require_topic_route,
)


def test_sensor_noncompensated_is_the_only_native_route():
    native = classify_gmo_semantics("SENSOR", "NONCOMPENSATED")
    assert native.route == "native_raw"
    assert require_topic_route("/lidar/points_raw_native", native.route)
    with pytest.raises(ValueError):
        classify_gmo_semantics("SENSOR", "COMPENSATED")


def test_world_endpoint_cannot_be_published_as_raw():
    world = classify_gmo_semantics("WORLD", "NONCOMPENSATED")
    assert world.route == "world_reference"
    with pytest.raises(ValueError, match="cannot be published as native raw"):
        require_topic_route("/lidar/points_raw_native", world.route)
