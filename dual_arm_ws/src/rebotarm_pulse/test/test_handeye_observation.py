"""Regression tests for fresh detections, synchronized capture, and safe stop."""

from collections import deque
from concurrent.futures import Future
from dataclasses import replace
import json
import math
import threading
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from rebotarm_pulse.adaptive_handeye import HemispherePose, Transform, rotation_exp
from rebotarm_pulse.handeye_observation import (
    DetectionIdentity, FrameAdmission, MotionSafetyError, Observation,
    anchor_drift, confirm_trajectory_stop, detection_observation, nearest_joint_state,
)
from rebotarm_pulse.piperh_auto_handeye import PiperHAutoHandeye
from rebotarm_pulse.handeye_quality import QualityConfig


def observation(identity="run:1", stamp=1000000000):
    return Observation(
        identity, stamp, "camera_color_optical_frame",
        {"rotation": np.eye(3).tolist(), "translation": [0.0, 0.0, 0.4]},
        0.2, 10000.0, [[0.0, 0.0], [100.0, 0.0], [100.0, 100.0], [0.0, 100.0]],
        image_width=640, image_height=480,
    )


def test_workspace_constraint_uses_predicted_camera_target_pose():
    node = PiperHAutoHandeye.__new__(PiperHAutoHandeye)
    node._base_T_target = Transform.identity()
    node._workspace_mode = "local"

    # base_T_camera.inverse() places the target 0.47 m along optical Z.
    camera = Transform(np.eye(3), [0.0, 0.0, -0.47])
    tilt, z_m = node._candidate_workspace_metrics(camera)
    assert tilt == pytest.approx(0.0)
    assert z_m == pytest.approx(0.47)
    assert node._candidate_in_workspace(camera)

    too_far = Transform(np.eye(3), [0.0, 0.0, -0.48])
    assert not node._candidate_in_workspace(too_far)
    tilted = Transform(rotation_exp([0.0, math.radians(25.0), 0.0]), [0.0, 0.0, -0.47])
    assert not node._candidate_in_workspace(tilted)


def test_observed_workspace_constraint_rechecks_measured_pose():
    accepted = Transform(np.eye(3), [0.0, 0.0, 0.479])
    assert PiperHAutoHandeye._observed_in_local_workspace(accepted)
    assert not PiperHAutoHandeye._observed_in_local_workspace(
        Transform(np.eye(3), [0.0, 0.0, 0.48])
    )
    assert not PiperHAutoHandeye._observed_in_local_workspace(
        Transform(rotation_exp([0.0, math.radians(25.0), 0.0]), [0.0, 0.0, 0.47])
    )


def test_global_workspace_does_not_filter_candidates():
    node = PiperHAutoHandeye.__new__(PiperHAutoHandeye)
    node._workspace_mode = "global"
    node._base_T_target = None
    assert node._candidate_in_workspace(Transform.identity())


def test_final_candidate_refresh_recomputes_geometry_and_filters_before_registration():
    node = PiperHAutoHandeye.__new__(PiperHAutoHandeye)
    node._base_T_target = Transform.identity()
    poses = {
        0.0: Transform(np.eye(3), [0.0, 0.0, 1.0]),
        1.0: Transform(np.eye(3), [0.1, 0.0, 1.0]),
        2.0: Transform(np.eye(3), [1.0, 0.0, 1.0]),
    }
    node._fk_for_link5 = lambda joints: poses[float(joints[0])]
    node._candidate_in_fov = lambda pose: float(pose.translation[0]) < 0.5
    node._candidate_in_workspace = lambda _pose: True
    candidates = [
        HemispherePose(9.0, 9.0, 9.0, 9.0, Transform.identity(), (value, 0, 0, 0, 0, 0))
        for value in (1.0, 2.0)
    ]

    refreshed, rejected = PiperHAutoHandeye._refresh_joint_candidates(
        node, candidates, Transform.identity(), Transform.identity(),
        (0.0, 0.0, 0.0), (0.0, 0, 0, 0, 0, 0),
    )

    assert len(refreshed) == 1
    np.testing.assert_allclose(refreshed[0].base_T_camera.translation, [0.1, 0.0, 1.0])
    assert refreshed[0].radius_m == pytest.approx(math.sqrt(1.01))
    assert rejected == {"predicted_fov_or_projected_area": 1}


def test_duplicate_detection_id_cannot_count_again():
    gate = FrameAdmission()
    first = observation()
    assert gate.reason(first, first.image_timestamp + 10000000, 0) is None
    gate.accept(first)
    second = replace(first, image_timestamp=first.image_timestamp + 200000000)
    assert gate.reason(second, second.image_timestamp, 0) == "duplicate_detection_id"


@pytest.mark.parametrize("offset", [0, -1000000])
def test_same_or_backwards_timestamp_is_rejected(offset):
    gate = FrameAdmission()
    first = observation()
    assert gate.reason(first, first.image_timestamp, 0) is None
    second = observation("run:2", first.image_timestamp + offset)
    assert gate.reason(second, first.image_timestamp, 0) == "non_increasing_timestamp"


def test_old_future_and_pre_window_frames_are_rejected():
    item = observation()
    assert FrameAdmission().reason(item, item.image_timestamp + 250000001, 0) == "expired_frame"
    assert FrameAdmission().reason(item, item.image_timestamp - 1, 0) == "future_timestamp"
    assert FrameAdmission().reason(item, item.image_timestamp, item.image_timestamp + 1) == "before_capture_window"


def test_minimum_interval_accepts_exact_boundary():
    gate = FrameAdmission()
    first = observation()
    assert gate.reason(first, first.image_timestamp, 0) is None
    gate.accept(first)
    fast = observation("run:2", first.image_timestamp + 149999999)
    assert gate.reason(fast, fast.image_timestamp, 0) == "detection_interval"
    boundary = observation("run:3", first.image_timestamp + 150000000)
    assert gate.reason(boundary, boundary.image_timestamp, 0) is None


def test_detector_restart_separates_ids_but_preserves_timestamp_guard():
    old, new = DetectionIdentity(), DetectionIdentity()
    assert old.next() != new.next()
    assert new.sequence == 1
    gate = FrameAdmission()
    a = observation(old.next())
    assert gate.reason(a, a.image_timestamp, 0) is None
    gate.accept(a)
    b = observation(new.next(), a.image_timestamp + 150000000)
    assert gate.reason(b, b.image_timestamp, 0) is None


def test_observation_serialization_preserves_image_pose_corners_and_receive_time():
    identity = DetectionIdentity()
    header = SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=22), frame_id="camera")
    item = detection_observation(
        identity, header, "marker_frame", np.zeros(3), [0, 0, 0.4], 0.3,
        [[0, 0], [10, 0], [10, 10], [0, 10]],
    )
    decoded = Observation.from_json(json.dumps(item.to_dict()), 1050000000)
    assert decoded.image_timestamp == 1000000022
    assert decoded.receive_time == 1050000000
    assert decoded.board_area == 100.0
    assert decoded.reprojection_error == 0.3
    assert len(decoded.corners) == 4


def test_joint_history_matches_image_time_and_rejects_excess_offset():
    history = [{"timestamp": 1000000000}, {"timestamp": 1040000000}]
    assert nearest_joint_state(history, 1030000000, 0.05) is history[1]
    assert nearest_joint_state(history, 1100000000, 0.05) is None
    assert nearest_joint_state([], 1000000000, 0.05) is None


@pytest.mark.parametrize("translation,angle", [([0.00111, 0, 0], 0), ([0, 0, 0], 0.251)])
def test_anchor_translation_or_rotation_exceed_threshold(translation, angle):
    result = anchor_drift(
        Transform.identity(), Transform(rotation_exp([0, 0, np.deg2rad(angle)]), translation),
    )
    assert result["exceeded"]


def test_anchor_within_threshold_is_accepted():
    result = anchor_drift(
        Transform.identity(), Transform(rotation_exp([0, 0, np.deg2rad(0.24)]), [0.0009, 0, 0]),
    )
    assert not result["exceeded"]


class Handle:
    def __init__(self, cancel_error=False):
        self.cancel_error = cancel_error
        self.calls = 0

    def cancel_goal_async(self):
        self.calls += 1
        if self.cancel_error:
            raise RuntimeError("cancel transport failed")
        response = Future()
        response.set_result(SimpleNamespace(goals_canceling=[1]))
        return response


@pytest.mark.parametrize("cancel_error", [False, True])
def test_cancel_ack_or_exception_without_terminal_result_is_not_stop(cancel_error):
    with pytest.raises(MotionSafetyError, match="MOTION_STOP_UNCONFIRMED"):
        confirm_trajectory_stop(Handle(cancel_error), Future(), 0.001)


def test_cancel_exception_with_terminal_result_still_confirms_stop():
    result = Future()
    result.set_result(SimpleNamespace(status=5))
    assert confirm_trajectory_stop(Handle(True), result, 0.001).status == 5


def test_unconfirmed_stop_preserves_active_handle():
    handle = Handle()
    node = SimpleNamespace(
        _stop_lock=threading.Lock(), _lock=threading.Lock(), _pending_goal=None,
        _active_goal=handle, _active_result=Future(),
        get_parameter=lambda _name: SimpleNamespace(value=0.001),
    )
    with pytest.raises(MotionSafetyError, match="MOTION_STOP_UNCONFIRMED"):
        PiperHAutoHandeye._confirm_motion_stopped(node)
    assert node._active_goal is handle


def test_late_goal_acceptance_after_abort_is_canceled_and_retained():
    result = Future()
    handle = Handle()
    handle.accepted = True
    handle.get_result_async = lambda: result
    future = Future()
    future.set_result(handle)
    stopping = threading.Event()
    stopping.set()
    node = SimpleNamespace(
        _lock=threading.Lock(), _stopping=stopping, _active_goal=None,
        _active_result=None, _pending_goal=future,
        _cancel_active_goal=lambda: handle.cancel_goal_async(),
    )
    PiperHAutoHandeye._goal_response(node, future)
    assert node._active_goal is handle
    assert node._active_result is result
    assert handle.calls == 1


def test_delayed_goal_callback_does_not_resurrect_finished_handle():
    handle = Handle()
    handle.accepted = True
    future = Future()
    future.set_result(handle)
    node = SimpleNamespace(
        _lock=threading.Lock(), _pending_goal=None, _active_goal=None,
        _active_result=None,
    )
    PiperHAutoHandeye._goal_response(node, future)
    assert node._active_goal is None
    assert handle.calls == 0


def test_unresolved_goal_acceptance_cannot_be_reported_stopped():
    node = SimpleNamespace(
        _stop_lock=threading.Lock(), _lock=threading.Lock(), _pending_goal=Future(),
        _active_goal=None, _active_result=None,
        get_parameter=lambda _name: SimpleNamespace(value=0.001),
    )
    with pytest.raises(MotionSafetyError, match="MOTION_STOP_UNCONFIRMED"):
        PiperHAutoHandeye._confirm_motion_stopped(node)
    assert node._pending_goal is not None


def test_anchor_failure_sets_stop_and_saves_both_poses(tmp_path):
    node = SimpleNamespace(
        _anchor_robot_pose=Transform.identity(), _run_dir=tmp_path,
        _stopping=threading.Event(), _confirm_motion_stopped=lambda: None,
        _transform_dict=PiperHAutoHandeye._transform_dict,
        get_parameter=lambda name: SimpleNamespace(
            value=(0.001 if "translation" in name else
                   0.0005 if name == "anchor_goal_tolerance_rad" else 0.25)
        ),
    )
    with pytest.raises(MotionSafetyError, match="ANCHOR_DRIFT_ERROR"):
        PiperHAutoHandeye._check_anchor_robot(node, "return", Transform(np.eye(3), [0.0011, 0, 0]))
    assert node._stopping.is_set()
    saved = yaml.safe_load((tmp_path / "anchor_robot_return.yaml").read_text())
    assert saved["exceeded"]
    assert "T_before" in saved and "T_after" in saved


@pytest.mark.parametrize("translation,should_fail", [(0.0, False), (0.002127, True)])
def test_anchor_return_checks_synchronized_capture_instead_of_latest_tf(tmp_path, translation, should_fail):
    from rebotarm_pulse.adaptive_handeye import PosePair

    calls = []
    robot = Transform(np.eye(3), [translation, 0, 0])
    pair = PosePair(robot, Transform.identity())
    node = SimpleNamespace(
        _anchor_robot_pose=Transform.identity(), _anchor_visual_pose=None,
        _run_dir=tmp_path, _stopping=threading.Event(),
        _confirm_motion_stopped=lambda: None,
        _transform_dict=PiperHAutoHandeye._transform_dict,
        get_parameter=lambda name: SimpleNamespace(
            value=(0.001 if "translation" in name else
                   0.0005 if name == "anchor_goal_tolerance_rad" else 0.25)
        ),
        _execute=lambda goal, label, **kwargs: calls.append(("execute", kwargs)),
    )

    def capture(label):
        calls.append(("capture", label))
        return pair

    def latest_tf(*args):
        pytest.fail("Anchor return must not read potentially lagging latest TF")

    node._capture = capture
    node._lookup = latest_tf
    node._check_anchor_robot = lambda label, pose: PiperHAutoHandeye._check_anchor_robot(node, label, pose)
    if should_fail:
        with pytest.raises(MotionSafetyError, match="ANCHOR_DRIFT_ERROR"):
            PiperHAutoHandeye._return_anchor(node, [0.0] * 6, "return")
        assert node._stopping.is_set()
    else:
        assert PiperHAutoHandeye._return_anchor(node, [0.0] * 6, "return") is pair
        assert not node._stopping.is_set()
    assert calls == [("execute", {"goal_tolerance": 0.0005}), ("capture", "anchor_check_return")]


def test_anchor_plan_endpoint_outside_tolerance_is_rejected_before_execution():
    from rebotarm_pulse.piperh_auto_handeye import JOINT_NAMES

    trajectory = SimpleNamespace(
        joint_names=list(JOINT_NAMES),
        points=[SimpleNamespace(positions=[0.001, 0, 0, 0, 0, 0])],
    )
    node = SimpleNamespace(
        _record_only=False, _motion_guard=lambda: None, _lock=threading.Lock(),
        _active_goal=None, _pending_goal=None, _current_positions=lambda: [0.0] * 6,
        _plan=lambda *args, **kwargs: trajectory,
    )
    with pytest.raises(MotionSafetyError, match="planned endpoint exceeds goal tolerance"):
        PiperHAutoHandeye._execute(node, [0.0] * 6, "return", goal_tolerance=0.0001)


@pytest.mark.parametrize(
    ("j4_deg", "excluded"),
    [(0.0, True), (0.299, True), (-0.299, True), (0.30, False), (-0.30, False), (1.0, False)],
)
def test_j4_zero_exclusion_is_symmetric_and_has_an_open_boundary(j4_deg, excluded):
    node = SimpleNamespace(get_parameter=lambda _name: SimpleNamespace(value=0.30))
    joints = [0.0, 0.0, 0.0, math.radians(j4_deg), 0.0, 0.0]
    assert PiperHAutoHandeye._j4_in_zero_exclusion(node, joints) is excluded


def test_candidate_planned_endpoint_in_j4_zero_zone_is_rejected_before_execution():
    from rebotarm_pulse.piperh_auto_handeye import JOINT_NAMES

    trajectory = SimpleNamespace(
        joint_names=list(JOINT_NAMES),
        points=[SimpleNamespace(positions=[0.0, 0.0, 0.0, math.radians(0.2), 0.0, 0.0])],
    )
    node = SimpleNamespace(
        _record_only=False, _motion_guard=lambda: None, _lock=threading.Lock(),
        _active_goal=None, _pending_goal=None, _current_positions=lambda: [0.0] * 6,
        _plan=lambda *args, **kwargs: trajectory,
        get_parameter=lambda name: SimpleNamespace(
            value=0.30 if name == "j4_zero_exclusion_deg" else 35.0
        ),
    )
    goal = [0.0, 0.0, 0.0, math.radians(0.2), 0.0, 0.0]
    with pytest.raises(RuntimeError, match="inside the configured zero exclusion zone"):
        PiperHAutoHandeye._execute(
            node, goal, "selection_001", enforce_j4_zero_exclusion=True,
        )


@pytest.mark.parametrize("tolerance", [0.0001, 0.0005, 0.008])
def test_controller_uses_formal_arrival_tolerance_not_moveit_tolerance(tolerance):
    from rebotarm_pulse.piperh_auto_handeye import JOINT_NAMES

    sent = []
    future = Future()
    future.set_result(SimpleNamespace(accepted=False))
    # Use a ROS trajectory message so the action goal's type checking applies.
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    trajectory = JointTrajectory(
        joint_names=list(JOINT_NAMES), points=[JointTrajectoryPoint(positions=[0.0] * 6)],
    )
    node = SimpleNamespace(
        _record_only=False, _motion_guard=lambda: None, _lock=threading.Lock(),
        _stop_lock=threading.Lock(), _active_goal=None, _pending_goal=None,
        _current_positions=lambda: [0.0] * 6,
        _plan=lambda *args, **kwargs: trajectory, _goal_response=lambda _: None,
        _trajectory=SimpleNamespace(send_goal_async=lambda goal: sent.append(goal) or future),
        get_parameter=lambda name: SimpleNamespace(
            value=.1 if name == "joint_arrival_tolerance_deg" else 35.0
        ),
    )
    with pytest.raises(RuntimeError, match="controller rejected"):
        PiperHAutoHandeye._execute(node, [0.0] * 6, "return", goal_tolerance=tolerance)
    assert {item.name: item.position for item in sent[0].goal_tolerance} == dict.fromkeys(
        JOINT_NAMES, math.radians(.1)
    )


class CaptureQueue:
    """Feed callbacks arriving after capture clears its previous queue."""

    def __init__(self, items, clock):
        self.items = deque(items)
        self.clock = clock

    def clear(self):
        pass

    def __bool__(self):
        return bool(self.items)

    def popleft(self):
        item = self.items.popleft()
        self.clock.stamp = item.image_timestamp + 10000000
        return item


def capture_node(tmp_path, items, required=15):
    clock = SimpleNamespace(stamp=1000000000)
    clock.now = lambda: SimpleNamespace(nanoseconds=clock.stamp)
    params = {
        "settle_sec": 0.0, "sample_frames": required, "vision_frame_timeout": 0.1,
        "robot_visual_sync_threshold": 0.05, "max_future_timestamp_wait_sec": 0.5,
        "robot_tf_wait_sec": 0.1,
    }
    transform = SimpleNamespace(
        rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        translation=SimpleNamespace(x=0.0, y=0.0, z=0.0),
    )
    node = SimpleNamespace(
        _lock=threading.Lock(), _stopping=threading.Event(),
        _motion_guard=lambda: None, get_clock=lambda: clock,
        get_parameter=lambda name: SimpleNamespace(value=params[name]),
        _observations=CaptureQueue(items, clock),
        _joint_history=deque({
            "timestamp": item.image_timestamp, "positions": [0.0] * 6,
            "velocities": [0.0] * 6,
        } for item in items),
        _frame_admission=FrameAdmission(), _current_positions=lambda: (0.0,) * 6,
        _buffer=SimpleNamespace(lookup_transform=lambda *_args, **_kwargs: transform),
        _base="piperh/base_link", _link5="piperh/Link5", _run_dir=tmp_path,
        _sample_count=0, _camera_info_hash="test", _transform_dict=PiperHAutoHandeye._transform_dict,
        _quality_config=QualityConfig(), _quality_results=[],
        get_logger=lambda: SimpleNamespace(info=lambda *_args: None),
    )
    node._capture_window = lambda label, audit: PiperHAutoHandeye._capture_window(node, label, audit)
    node._synchronize_observation = lambda item, history: PiperHAutoHandeye._synchronize_observation(node, item, history)
    node._wait_for_observation_timestamp = lambda item, deadline: PiperHAutoHandeye._wait_for_observation_timestamp(
        node, item, deadline,
    )
    return node


def test_timestamped_robot_tf_lookup_waits_for_late_buffer_update(tmp_path):
    item = observation("late-tf:1", 1010000000)
    node = capture_node(tmp_path, [item], required=1)
    calls = []
    transform = node._buffer.lookup_transform()

    def lookup(parent, child, stamp, *, timeout):
        calls.append((parent, child, stamp.nanoseconds, timeout.nanoseconds))
        return transform

    node._buffer.lookup_transform = lookup
    record, pair, quality, reason = PiperHAutoHandeye._synchronize_observation(
        node, item, list(node._joint_history),
    )
    assert reason is None and record is not None and pair is not None and quality.valid
    assert calls == [("piperh/base_link", "piperh/Link5", item.image_timestamp, 100000000)]


def test_capture_waits_for_small_future_stamp_then_keeps_sync_gates(tmp_path):
    items = [observation(f"future:{index}", 1020000000 + index * 150000000) for index in range(3)]
    node = capture_node(tmp_path, items, required=3)
    clock = node.get_clock()

    class AdvancingStopEvent:
        def wait(self, seconds):
            clock.stamp += int(seconds * 1e9)
            return False

        def is_set(self):
            return False

    node._stopping = AdvancingStopEvent()
    original_pop = node._observations.popleft

    def pop_future_item():
        value = original_pop()
        clock.stamp = value.image_timestamp - 20000000
        return value

    node._observations.popleft = pop_future_item
    PiperHAutoHandeye._capture(node, "future_wait")
    saved = yaml.safe_load((tmp_path / "capture_future_wait.yaml").read_text())
    assert saved["accepted"] == 3
    assert saved["timestamp_wait"]["waited_frames"] == 3
    assert saved["timestamp_wait"]["max_initial_future_sec"] == pytest.approx(0.02)


def test_capture_counts_15_distinct_detections_and_saves_rejections(tmp_path):
    valid = [observation(f"run:{index}", 1010000000 + index * 150000000) for index in range(15)]
    items = [valid[0], valid[0]] + valid[1:]
    node = capture_node(tmp_path, items)
    PiperHAutoHandeye._capture(node, "test")
    saved = yaml.safe_load((tmp_path / "capture_test.yaml").read_text())
    assert saved["accepted"] == 15
    assert len({item["detection_id"] for item in saved["observations"]}) == 15
    assert saved["rejected_reasons"]["duplicate_detection_id"] == 1
    assert all(item["robot_visual_time_offset"] == 0.0 for item in saved["observations"])


def test_capture_cannot_fill_sample_count_with_repeated_tf_or_frames(tmp_path):
    first = observation(stamp=1010000000)
    node = capture_node(tmp_path, [first] * 15)
    with pytest.raises(RuntimeError, match="VISION_FRAME_TIMEOUT"):
        PiperHAutoHandeye._capture(node, "timeout")
    saved = yaml.safe_load((tmp_path / "capture_timeout.yaml").read_text())
    assert saved["accepted"] == 1
    assert saved["rejected_reasons"]["duplicate_detection_id"] == 14


def test_interrupted_capture_preserves_partial_observations(tmp_path):
    node = capture_node(tmp_path, [observation(stamp=1010000000)])

    def guard():
        if node._frame_admission.accepted_timestamp:
            raise MotionSafetyError("operator canceled")

    node._motion_guard = guard
    with pytest.raises(MotionSafetyError, match="operator canceled"):
        PiperHAutoHandeye._capture(node, "interrupted")
    saved = yaml.safe_load((tmp_path / "capture_interrupted.yaml").read_text())
    assert saved["accepted"] == 1


def test_low_quality_observation_is_saved_but_not_aggregated(tmp_path):
    bad = replace(observation("run:bad", 1010000000), reprojection_error=9.0,
                  camera_to_board_pose={"rotation": np.eye(3).tolist(), "translation": [10, 0, 0.4]})
    valid = [observation(f"run:{index}", 1160000000 + index * 150000000) for index in range(15)]
    node = capture_node(tmp_path, [bad] + valid)
    pair = PiperHAutoHandeye._capture(node, "quality")
    np.testing.assert_allclose(pair.camera_T_target.translation, [0, 0, 0.4])
    saved = yaml.safe_load((tmp_path / "capture_quality.yaml").read_text())
    assert saved["accepted"] == 15
    assert len(saved["observations"]) == 16
    assert saved["observations"][0]["valid"] is False
    assert saved["observations"][0]["reason"] == ["HIGH_REPROJECTION_ERROR"]
    assert saved["quality_summary"]["invalid"] == 1
    session = yaml.safe_load((tmp_path / "quality_summary.yaml").read_text())
    assert session["quality_summary"]["total"] == 16


def test_all_low_quality_frames_timeout_and_are_retained(tmp_path):
    bad = [replace(observation(f"bad:{index}", 1010000000 + index * 150000000),
                   reprojection_error=9.0) for index in range(15)]
    node = capture_node(tmp_path, bad)
    with pytest.raises(RuntimeError, match="VISION_FRAME_TIMEOUT"):
        PiperHAutoHandeye._capture(node, "all_bad")
    saved = yaml.safe_load((tmp_path / "capture_all_bad.yaml").read_text())
    assert saved["accepted"] == 0
    assert len(saved["observations"]) == 15
    assert saved["quality_summary"]["reject_reason_count"]["HIGH_REPROJECTION_ERROR"] == 15


def test_link5_fk_uses_namespaced_base_and_requested_joint_state():
    captured = {}
    pose = SimpleNamespace(
        position=SimpleNamespace(x=0.3, y=-0.1, z=0.4),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    response = SimpleNamespace(
        error_code=SimpleNamespace(val=1),
        pose_stamped=[SimpleNamespace(pose=pose)],
    )

    def call_async(request):
        captured["request"] = request
        return object()

    node = SimpleNamespace(
        _moveit_base="piperh/base_link", _moveit_link="Link5",
        _fk=SimpleNamespace(call_async=call_async),
        _robot_state=lambda joints: SimpleNamespace(joints=tuple(joints)),
        _wait_future=lambda _future, _label, _timeout: response,
    )
    result = PiperHAutoHandeye._fk_for_link5(node, [0.0] * 6)

    np.testing.assert_allclose(result.translation, (0.3, -0.1, 0.4))
    request = captured["request"]
    assert request.header.frame_id == "piperh/base_link"
    assert request.fk_link_names == ["Link5"]


def test_stratified_candidates_fall_back_when_reachable_buckets_are_sparse():
    candidates = [
        HemispherePose(
            0.4, 10.0, float(phi), 0.0, Transform.identity(), tuple([0.0] * 6),
        )
        for phi in range(0, 360, 30)
    ]
    node = PiperHAutoHandeye.__new__(PiperHAutoHandeye)

    selected = PiperHAutoHandeye._stratified_candidates(node, candidates, 8, set())

    assert len(selected) == 8
    assert len({PiperHAutoHandeye._candidate_key(item) for item in selected}) == 8
