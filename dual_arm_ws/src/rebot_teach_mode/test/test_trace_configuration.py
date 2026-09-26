import threading
from types import SimpleNamespace

import pytest

from rebot_teach_mode.teach_node import TeachModeNode


def _node():
    node = TeachModeNode.__new__(TeachModeNode)
    node._lock = threading.RLock()
    node._message = ""
    node._tcp_trace_line_width_m = 0.004
    node._tcp_trace_progress = 0.375
    node.refresh_count = 0
    node.published_progress = []
    node.status_count = 0
    node._refresh_shape_marker_locked = lambda: setattr(
        node, "refresh_count", node.refresh_count + 1
    )
    node._publish_tcp_trace = node.published_progress.append
    node._publish_status = lambda: setattr(
        node, "status_count", node.status_count + 1
    )
    return node


def test_trace_width_update_refreshes_current_visualization():
    node = _node()
    response = SimpleNamespace(success=False, message="", line_width_m=0.0)

    result = node._configure_trace(
        SimpleNamespace(set_line_width=True, line_width_m=0.012), response
    )

    assert result.success
    assert result.line_width_m == pytest.approx(0.012)
    assert node._tcp_trace_line_width_m == pytest.approx(0.012)
    assert node.refresh_count == 1
    assert node.published_progress == [pytest.approx(0.375)]
    assert node.status_count == 1


@pytest.mark.parametrize("line_width", [0.00049, 0.03001, float("nan")])
def test_trace_width_update_rejects_values_outside_safe_display_range(line_width):
    node = _node()
    response = SimpleNamespace(success=False, message="", line_width_m=0.0)

    result = node._configure_trace(
        SimpleNamespace(set_line_width=True, line_width_m=line_width), response
    )

    assert not result.success
    assert result.line_width_m == pytest.approx(0.004)
    assert node.refresh_count == 0
    assert node.published_progress == []
    assert node.status_count == 1


def test_preview_pen_starts_at_gripper_front_and_ends_at_exposed_tip():
    node = TeachModeNode.__new__(TeachModeNode)
    node._tcp_link_name = "gripper_tcp"
    node._shape_pen_mount_offset_m = 0.1064
    node._shape_pen_length_m = 0.060

    attached = node._virtual_pen_attached_object()

    assert attached is not None
    assert attached.link_name == "gripper_tcp"
    body_pose, tip_pose = attached.object.primitive_poses
    assert body_pose.position.x == pytest.approx(0.1364)
    assert body_pose.orientation.y > 0.0
    assert tip_pose.position.x == pytest.approx(0.1664)
    assert attached.object.primitives[0].dimensions[0] == pytest.approx(0.060)


def test_dual_arm_visual_markers_use_prefixed_tf_frames():
    node = TeachModeNode.__new__(TeachModeNode)
    node._visualization_base_frame = "piperh/base_link"
    node._visualization_tcp_link_name = "piperh/Link6"
    node._tcp_trace_line_width_m = 0.004
    node._shape_pen_mount_offset_m = 0.0
    node._shape_pen_length_m = 0.060
    node._shape_pen_lift_m = 0.012
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: SimpleNamespace())
    )

    trace = node._trace_marker(0, [(0.0, 0.0, 0.0)], completed=False)
    pen_markers = node._virtual_pen_markers()

    assert trace.header.frame_id == "piperh/base_link"
    assert {marker.header.frame_id for marker in pen_markers} == {"piperh/Link6"}
