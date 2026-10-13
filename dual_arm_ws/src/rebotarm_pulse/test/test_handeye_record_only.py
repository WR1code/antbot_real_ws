"""Synthetic callbacks only; no robot, calibration input, or motor command."""

from dataclasses import replace
from types import SimpleNamespace

import pytest
import yaml
from tf2_ros import TransformException

from rebotarm_pulse.handeye_observation import MotionSafetyError
from rebotarm_pulse.piperh_auto_handeye import PiperHAutoHandeye
from rebotarm_pulse.piperh_auto_handeye import _normalize_ros_arguments
from test_handeye_observation import capture_node, observation


def recorder(tmp_path, items, monkeypatch):
    node = capture_node(tmp_path, items)
    monkeypatch.setenv("ANTBOT_HAND_EYE_RUNS", str(tmp_path))
    previous_parameter = node.get_parameter
    extra = {"service_timeout_sec": 1.0, "min_detection_interval": 0.15, "max_detection_age": 0.10}
    node.get_parameter = lambda name: SimpleNamespace(value=extra[name]) if name in extra else previous_parameter(name)
    node._camera_info_document = {"width": 640, "height": 480}
    node._camera_info_changed = False
    node._camera = "camera_color_optical_frame"
    node._marker = "marker_frame"
    node._can_lookup = lambda *_args: True
    node._record_only = True
    node._anchor_robot_pose = None
    node._anchor_visual_pose = None
    node.get_logger = lambda: SimpleNamespace(info=lambda *_args: None)
    states = []
    node._publish = lambda state, *_args, **kwargs: states.append((state, kwargs))
    original_queue = node._observations

    class StopWhenDrained:
        def __bool__(self):
            if not original_queue:
                node._stopping.set()
                return False
            return True

        def popleft(self):
            return original_queue.popleft()

    node._observations = StopWhenDrained()
    return node, states


def test_record_only_saves_anchor_synchronized_frames_and_rejections(tmp_path, monkeypatch):
    a = observation("run:1", 1010000000)
    b = observation("run:2", 1160000000)
    bad = replace(observation("run:3", 1310000000), reprojection_error=9.0)
    node, states = recorder(tmp_path, [a, a, b, bad], monkeypatch)
    PiperHAutoHandeye._run_record_only(node)
    assert node._run_dir.name.startswith("record_session_")
    assert (node._run_dir / "anchor.yaml").exists()
    files = sorted((node._run_dir / "observations").glob("*.yaml"))
    assert len(files) == 4
    first = yaml.safe_load(files[0].read_text())
    assert first["valid"]
    assert first["joint_state"] and first["robot_pose"]
    assert first["robot_timestamp"] == a.image_timestamp
    assert first["robot_visual_time_offset"] == 0
    repeated = yaml.safe_load(files[1].read_text())
    assert repeated["reason"] == ["duplicate_detection_id"]
    low_quality = yaml.safe_load(files[3].read_text())
    assert low_quality["reason"] == ["HIGH_REPROJECTION_ERROR"]
    assert low_quality["robot_pose"] is not None
    summary = yaml.safe_load((node._run_dir / "session.yaml").read_text())
    assert summary["state"] == "stopped"
    assert summary["motion_enabled"] is False and summary["calibration_written"] is False
    assert summary["quality_summary"]["invalid"] == 1
    assert states[-1][0] == "record_stopped"
    assert not list(tmp_path.rglob("*.calib"))


def test_record_only_forbids_execution_and_full_workflow():
    node = SimpleNamespace(_record_only=True)
    with pytest.raises(MotionSafetyError, match="forbids"):
        PiperHAutoHandeye._execute(node, (), "blocked")
    with pytest.raises(MotionSafetyError, match="cannot enter"):
        PiperHAutoHandeye._run(node)


def test_guarded_record_only_never_enters_calibration():
    calls = []
    node = SimpleNamespace(_record_only=True, _run_record_only=lambda: calls.append("record"))
    PiperHAutoHandeye._run_guarded(node)
    assert calls == ["record"]


def test_robot_tf_unavailable_is_retained_without_latest_state_fallback(tmp_path, monkeypatch):
    item = observation(stamp=1010000000)
    node, _states = recorder(tmp_path, [item], monkeypatch)
    def unavailable(*_args, **_kwargs):
        raise TransformException("exact timestamp unavailable")
    node._buffer.lookup_transform = unavailable
    PiperHAutoHandeye._run_record_only(node)
    saved = yaml.safe_load((node._run_dir / "observations/000001.yaml").read_text())
    assert saved["reason"] == ["robot_tf_timestamp_unavailable"]
    assert saved["robot_pose"] is None
    assert not (node._run_dir / "anchor.yaml").exists()


@pytest.mark.parametrize("explicit_ros_args", [False, True])
def test_real_parameter_startup_record_only_without_motion_clients(monkeypatch, explicit_ros_args):
    import rclpy
    import rebotarm_pulse.piperh_auto_handeye as module
    from rclpy.node import Node

    arguments = ["-p", "record_only:=true", "-p", "motion_enabled:=false"]
    if explicit_ros_args:
        arguments.insert(0, "--ros-args")
    rclpy.init(args=_normalize_ros_arguments(arguments), domain_id=101)
    node = None
    calls = []
    def forbidden(*_args, **_kwargs):
        raise AssertionError("record-only called a motion interface")
    monkeypatch.setattr(Node, "create_client", forbidden)
    monkeypatch.setattr(module, "ActionClient", forbidden)
    monkeypatch.setattr(PiperHAutoHandeye, "_precheck", forbidden)
    monkeypatch.setattr(PiperHAutoHandeye, "_run", forbidden)
    monkeypatch.setattr(PiperHAutoHandeye, "_plan", forbidden)
    monkeypatch.setattr(PiperHAutoHandeye, "_execute", forbidden)
    monkeypatch.setattr(PiperHAutoHandeye, "_run_record_only", lambda self: calls.append("record"))
    try:
        node = PiperHAutoHandeye()
        assert node._record_only is True
        assert node.get_parameter("motion_enabled").value is False
        assert node._ik is None and node._planner is None and node._trajectory is None
        assert node._joint_history is not None and node._observations is not None
        node._run_guarded()
        assert calls == ["record"]
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_automatic_mode_disabled_permission_still_refuses_start(monkeypatch):
    import rclpy
    import rebotarm_pulse.piperh_auto_handeye as module
    from rclpy.node import Node
    rclpy.init(args=_normalize_ros_arguments([
        "-p", "record_only:=false", "-p", "motion_enabled:=false",
    ]), domain_id=101)
    node = None
    monkeypatch.setattr(Node, "create_client", lambda *_args, **_kwargs: object())
    monkeypatch.setattr(module, "ActionClient", lambda *_args, **_kwargs: object())
    try:
        node = PiperHAutoHandeye()
        assert node._record_only is False
        with pytest.raises(RuntimeError, match="automatic motion is disabled"):
            node._precheck()
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_standard_ros_arguments_remain_unchanged():
    arguments = ["node", "--ros-args", "-p", "record_only:=true"]
    assert _normalize_ros_arguments(arguments) == arguments


def test_program_name_and_bare_parameter_flags_are_normalized():
    arguments = ["node", "-p", "record_only:=true", "-p", "motion_enabled:=false"]
    assert _normalize_ros_arguments(arguments) == ["node", "--ros-args", *arguments[1:]]
