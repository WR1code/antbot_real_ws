import yaml

import easy_handeye2.handeye_calibration as calibration_module


def test_load_calibration_ignores_compatible_metadata(tmp_path, monkeypatch):
    monkeypatch.setattr(calibration_module, "CALIBRATIONS_DIRECTORY", tmp_path)
    document = {
        "parameters": {
            "name": "metadata_test",
            "calibration_type": "eye_in_hand",
            "robot_base_frame": "base",
            "robot_effector_frame": "link5",
            "tracking_base_frame": "camera",
            "tracking_marker_frame": "target",
            "freehand_robot_movement": True,
        },
        "transform": {
            "translation": {"x": 0.1, "y": 0.2, "z": 0.3},
            "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        },
        "metadata": {
            "workspace_mode": "local",
            "workspace_constraints": {"tilt_max_deg": 25, "z_max": 0.48},
        },
    }
    (tmp_path / "metadata_test.calib").write_text(
        yaml.safe_dump(document), encoding="utf-8"
    )

    loaded = calibration_module.load_calibration("metadata_test")

    assert loaded.parameters.name == "metadata_test"
    assert loaded.transform.translation.z == 0.3
