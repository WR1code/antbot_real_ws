from geometry_msgs.msg import Point, Pose, Quaternion

from rebot_teach_mode.teach_node import (
    PREFLIGHT_COLLISION,
    PREFLIGHT_NEAR_LIMIT,
    PREFLIGHT_NO_IK,
    PREFLIGHT_REACHABLE,
    TeachModeNode,
)


def test_preflight_distinguishes_limit_collision_and_no_ik():
    node = TeachModeNode.__new__(TeachModeNode)
    node._joint_names = tuple(f"joint{index}" for index in range(1, 7))
    node._lower_limits = (-1.0,) * 6
    node._upper_limits = (1.0,) * 6
    node._preflight_yaw = lambda *_args, **_kwargs: ((0.0,) * 6, 0.0)

    def fake_ik(position, _seed, _yaw, _orientation, avoid_collisions, _timeout):
        category = round(position[0])
        if category == 0:
            return (0.0,) * 6, 1
        if category == 1:
            return (0.92,) * 6, 1
        if category == 2:
            return (None, -31) if avoid_collisions else ((0.2,) * 6, 1)
        return None, -31

    node._preflight_ik = fake_ik
    statuses, rows, yaw = node._preflight_samples(
        ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0),
         (2.0, 0.0, 0.0), (3.0, 0.0, 0.0)),
        (0.0,) * 6, Quaternion(w=1.0), 0.1, 0.15,
    )
    assert statuses == (
        PREFLIGHT_REACHABLE,
        PREFLIGHT_NEAR_LIMIT,
        PREFLIGHT_COLLISION,
        PREFLIGHT_NO_IK,
    )
    assert rows[-1] is None
    assert yaw == 0.0


def test_preflight_trace_is_rendered_on_both_sides_of_drawing_plane():
    node = TeachModeNode.__new__(TeachModeNode)
    node._shape_marker_shape = "rectangle"
    node._shape_marker_source = ""
    node._shape_marker_width = 0.17
    node._shape_marker_height = 0.08
    node._shape_pen_length_m = 0.081
    node._shape_pen_lift_m = 0.031
    node._shape_marker_pose = Pose(
        position=Point(x=0.4, y=0.0, z=0.25),
        orientation=Quaternion(w=1.0),
    )
    node._tcp_trace_line_width_m = 0.004
    node._shape_preflight = {
        "signature": node._shape_signature(),
        "local_tip_points": ((0.0, 0.0, 0.0), (0.1, 0.0, 0.0)),
        "statuses": (PREFLIGHT_REACHABLE, PREFLIGHT_REACHABLE),
    }

    markers = node._shape_preflight_markers()

    assert len(markers) == 2
    assert {round(marker.points[0].z, 4) for marker in markers} == {-0.0004, 0.0004}


def test_shape_signature_ignores_rviz_sub_micrometre_round_trip_noise():
    node = TeachModeNode.__new__(TeachModeNode)
    node._shape_marker_shape = "rectangle"
    node._shape_marker_source = ""
    node._shape_marker_width = 0.17
    node._shape_marker_height = 0.08
    node._shape_pen_length_m = 0.081
    node._shape_pen_lift_m = 0.031
    node._shape_marker_pose = Pose(
        position=Point(x=0.4, y=0.0, z=0.25),
        orientation=Quaternion(w=1.0),
    )
    signature = node._shape_signature()

    node._shape_marker_pose.position.x += 1e-9

    assert node._shape_signature() == signature
