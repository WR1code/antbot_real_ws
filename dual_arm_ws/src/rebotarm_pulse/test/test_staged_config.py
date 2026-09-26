from pathlib import Path

import yaml


def test_staged_precontact_execution_keeps_existing_safety_distances():
    path = Path(__file__).parents[1] / "config" / "piper_pulse_align.yaml"
    params = yaml.safe_load(path.read_text())["piper_pulse_align"]["ros__parameters"]

    assert params["execute_motion"] is True
    assert params["enable_auto_sensor_axis_alignment"] is False
    assert params["sensor_array_axis_local"] == [0.0, 1.0, 0.0]
    assert params["standoff_m"] == 0.060
    assert params["dummy_contact_mode"] is False
    assert params["dummy_max_cartesian_speed_m_s"] <= 0.005
    assert params["dummy_execution_timeout_sec"] <= 90.0
    assert params["maximum_horizontal_translation_m"] == 0.300
    assert params["maximum_vertical_translation_m"] == 0.200
    assert params["target_filter_window_sec"] == 0.6
    assert params["target_stable_duration_sec"] == 0.0
    assert params["target_stable_min_samples"] == 3
    assert params["target_stable_radius_m"] == 0.005
    assert params["target_ema_alpha"] == 0.35
    assert params["target_stable_inlier_fraction"] == 0.60
    assert params["target_soft_drift_m"] == 0.008
    assert params["target_hard_drift_m"] == 0.015
    assert params["target_jump_reject_m"] == 0.015
    assert params["target_hard_drift_hold_sec"] == 0.20
    assert params["target_soft_timeout_sec"] == 1.5
    assert params["target_hard_timeout_sec"] == 2.0
    assert params["target_dropout_grace_sec"] == 0.5
    assert params["arm_direction_max_age_sec"] == 1.5
    assert params["pressure_precontact_delta_pa"] == 100.0
    assert params["pressure_abort_hold_sec"] == 0.15
    assert params["pressure_emergency_delta_pa"] == 1000.0


def test_target_pipeline_diagnostics_are_fast_and_do_not_mask_long_gaps():
    path = Path(__file__).parents[1] / "config" / "piper_pulse_target.yaml"
    params = yaml.safe_load(path.read_text())["piper_pulse_target"]["ros__parameters"]

    assert params["input_hard_age_sec"] == 1.5
    assert params["tf_timeout_sec"] <= 0.1
    assert params["heartbeat_hz"] >= 2.0
    assert params["statistics_period_sec"] == 5.0
    assert params["gap_warning_sec"] == [0.5, 1.0, 2.0]
