import ast
import inspect
import math
from pathlib import Path

import pytest
from geometry_msgs.msg import PointStamped, Vector3, Vector3Stamped
from std_msgs.msg import String

from rebotarm_pulse import piper_pulse_target
from rebotarm_pulse.piper_pulse_target import (
    load_saved_calibration,
    nearest_stamp_ns,
    PiperPulseTarget,
    quaternion_from_z_axis,
    timing_distribution,
)


def _rotate_local_z(quaternion):
    x, y, z, w = quaternion
    return (
        2.0 * (x * z + w * y),
        2.0 * (y * z - w * x),
        1.0 - 2.0 * (x * x + y * y),
    )


@pytest.mark.parametrize(
    "normal",
    [(0.0, 0.0, 1.0), (0.0, 1.0, 0.0), (1.0, 0.0, 0.0), (-0.2, 0.9, 0.3)],
)
def test_board_target_tf_local_z_follows_detected_normal(normal) -> None:
    quaternion = quaternion_from_z_axis(Vector3(x=normal[0], y=normal[1], z=normal[2]))
    rotated = _rotate_local_z(quaternion)
    length = math.sqrt(sum(value * value for value in normal))
    assert rotated == pytest.approx(tuple(value / length for value in normal))


def test_saved_eye_in_hand_calibration_bypasses_duplicate_camera_tf(
    monkeypatch, tmp_path: Path
) -> None:
    calibration_dir = tmp_path / ".ros2/easy_handeye2/calibrations"
    calibration_dir.mkdir(parents=True)
    (calibration_dir / "test_camera.calib").write_text(
        """parameters:
  calibration_type: eye_in_hand
  robot_effector_frame: piperh/Link5
  tracking_base_frame: camera_color_optical_frame
transform:
  translation: {x: 0.1, y: -0.2, z: 0.3}
  rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
""",
        encoding="utf-8",
    )
    monkeypatch.setattr(Path, "home", classmethod(lambda cls: tmp_path))
    monkeypatch.setenv("EASY_HANDEYE2_CALIBRATIONS_DIRECTORY", str(calibration_dir))
    calibration = load_saved_calibration("test_camera")
    assert calibration is not None

    operation = object.__new__(PiperPulseTarget)
    operation.calibration_transform = calibration
    operation._calibration_translation = operation._translation_tuple(calibration)
    operation.planning_frame = "piperh/Link5"
    operation.calibration_bridge_frame = "piperh/Link5"
    operation.tf_timeout_sec = 0.5
    operation.get_logger = lambda: type(
        "Logger", (), {"info": lambda self, _message: None,
                       "error": lambda self, _message: None}
    )()
    message = PointStamped()
    message.header.frame_id = "camera_color_optical_frame"
    message.point.x, message.point.y, message.point.z = 1.0, 2.0, 3.0

    transformed = operation._transform_point(message)
    assert transformed.header.frame_id == "piperh/Link5"
    assert transformed.point.x == pytest.approx(1.1)
    assert transformed.point.y == pytest.approx(1.8)
    assert transformed.point.z == pytest.approx(3.3)


def test_vector_transform_cannot_erase_handeye_translation_before_point_transform(
    monkeypatch, tmp_path: Path
) -> None:
    calibration_dir = tmp_path / "calibrations"
    calibration_dir.mkdir()
    calibration_dir.joinpath("immutable_handeye.calib").write_text(
        """parameters:
  calibration_type: eye_in_hand
  robot_effector_frame: piperh/Link5
  tracking_base_frame: camera_color_optical_frame
transform:
  translation: {x: -0.067992333, y: -0.118081997, z: -0.028696742}
  rotation: {x: 0.0, y: 0.0, z: 0.7071067811865475, w: 0.7071067811865476}
""",
        encoding="utf-8",
    )
    monkeypatch.setenv(
        "EASY_HANDEYE2_CALIBRATIONS_DIRECTORY", str(calibration_dir)
    )
    calibration = load_saved_calibration("immutable_handeye")
    assert calibration is not None

    class Logger:
        def info(self, _message):
            pass

        def error(self, _message):
            pass

    operation = object.__new__(PiperPulseTarget)
    operation.calibration_transform = calibration
    operation._calibration_translation = operation._translation_tuple(calibration)
    operation.planning_frame = "piperh/Link5"
    operation.calibration_bridge_frame = "piperh/Link5"
    operation.tf_timeout_sec = 0.5
    operation.get_logger = lambda: Logger()

    normal = Vector3Stamped()
    normal.header.frame_id = "camera_color_optical_frame"
    normal.vector.x = 1.0
    operation._transform_vector(normal)

    preserved = operation._translation_tuple(operation.calibration_transform)
    assert preserved == pytest.approx(
        (-0.067992333, -0.118081997, -0.028696742)
    )

    point = PointStamped()
    point.header.frame_id = "camera_color_optical_frame"
    point.point.x, point.point.y, point.point.z = 1.0, 2.0, 3.0
    transformed = operation._transform_point(point)

    # R is +90 degrees about Z, so R*p=(-2, 1, 3).  The assertion requires
    # the full R*p+t result and rejects the historical R*p-only corruption.
    assert transformed.point.x == pytest.approx(-2.0 - 0.067992333)
    assert transformed.point.y == pytest.approx(1.0 - 0.118081997)
    assert transformed.point.z == pytest.approx(3.0 - 0.028696742)

    operation.calibration_transform.transform.translation.x = 0.0
    operation.calibration_transform.transform.translation.y = 0.0
    operation.calibration_transform.transform.translation.z = 0.0
    with pytest.raises(
        RuntimeError, match="calibration transform translation was modified"
    ):
        operation._transform_point(point)


@pytest.mark.parametrize("name", ["", "../bad", "folder/name"])
def test_saved_calibration_rejects_unsafe_names(name: str) -> None:
    assert load_saved_calibration(name) is None


def test_target_node_uses_multithreaded_executor_for_tf_callbacks() -> None:
    """Prevent blocking TF lookups from starving TransformListener callbacks."""
    tree = ast.parse(inspect.getsource(piper_pulse_target.main))
    executor_calls = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "MultiThreadedExecutor"
    ]
    assert len(executor_calls) == 1
    threads = next(
        keyword.value for keyword in executor_calls[0].keywords
        if keyword.arg == "num_threads"
    )
    assert isinstance(threads, ast.Constant)
    assert threads.value >= 2


def test_timing_helpers_choose_nearest_and_report_p95() -> None:
    assert nearest_stamp_ns([900, 1010, 1200], 1000) == 1010
    assert nearest_stamp_ns([], 1000) is None
    summary = timing_distribution([1.0, 2.0, 3.0, 40.0])
    assert summary == {
        "count": 4,
        "mean_ms": 11.5,
        "p95_ms": 40.0,
        "max_ms": 40.0,
    }


def test_moving_target_tf_rejects_latest_time() -> None:
    operation = object.__new__(PiperPulseTarget)
    with pytest.raises(Exception, match="latest TF lookup is forbidden"):
        operation._lookup_transform("base", "camera", piper_pulse_target.Time())


def test_runtime_calibration_selection_reloads_transform_and_clears_target(
    monkeypatch, tmp_path: Path
) -> None:
    calibration_dir = tmp_path / "calibrations"
    calibration_dir.mkdir()
    (calibration_dir / "pass_snapshot.calib").write_text(
        """parameters:
  calibration_type: eye_in_hand
  robot_effector_frame: piperh/Link5
  tracking_base_frame: camera_color_optical_frame
transform:
  translation: {x: 0.4, y: 0.5, z: 0.6}
  rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
""",
        encoding="utf-8",
    )
    monkeypatch.setenv("EASY_HANDEYE2_CALIBRATIONS_DIRECTORY", str(calibration_dir))

    class Logger:
        def info(self, _message):
            pass

        def error(self, _message):
            pass

    operation = object.__new__(PiperPulseTarget)
    operation.calibration_name = "old"
    operation.calibration_transform = None
    operation._last_valid_target = PointStamped()
    operation._last_valid_target_rx = 1.0
    operation._last_valid_target_stamp_ns = 123
    operation._last_target_input_rx = 1.0
    operation._last_tf_ok = True
    operation._last_blocker = "NONE"
    operation.get_logger = lambda: Logger()

    operation.select_calibration(String(data="pass_snapshot"))

    assert operation.calibration_name == "pass_snapshot"
    assert operation.calibration_transform.transform.translation.x == pytest.approx(0.4)
    assert operation._last_valid_target is None
    assert operation._last_valid_target_stamp_ns == 0
    assert operation._last_blocker == "CALIBRATION_CHANGED"
