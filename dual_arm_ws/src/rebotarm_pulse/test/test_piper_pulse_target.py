from pathlib import Path

import pytest
from geometry_msgs.msg import PointStamped

from rebotarm_pulse.piper_pulse_target import PiperPulseTarget, load_saved_calibration


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
    calibration = load_saved_calibration("test_camera")
    assert calibration is not None

    operation = object.__new__(PiperPulseTarget)
    operation.calibration_transform = calibration
    operation.planning_frame = "piperh/Link5"
    operation.calibration_bridge_frame = "piperh/Link5"
    operation.tf_timeout_sec = 0.5
    message = PointStamped()
    message.header.frame_id = "camera_color_optical_frame"
    message.point.x, message.point.y, message.point.z = 1.0, 2.0, 3.0

    transformed = operation._transform_point(message)
    assert transformed.header.frame_id == "piperh/Link5"
    assert transformed.point.x == pytest.approx(1.1)
    assert transformed.point.y == pytest.approx(1.8)
    assert transformed.point.z == pytest.approx(3.3)


@pytest.mark.parametrize("name", ["", "../bad", "folder/name"])
def test_saved_calibration_rejects_unsafe_names(name: str) -> None:
    assert load_saved_calibration(name) is None
