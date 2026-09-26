from pathlib import Path

import yaml


def load_mapping() -> dict:
    config_path = Path(__file__).parents[1] / "config" / "xbox_mapping.yaml"
    return yaml.safe_load(config_path.read_text(encoding="utf-8"))


def test_piperh_speed_tiers_are_doubled() -> None:
    parameters = load_mapping()["rebot_xbox_twist"]["ros__parameters"]

    assert parameters["speed"]["levels"] == [0.20, 0.50, 1.00]
    assert parameters["limits"]["linear_speed"] == 0.10
    assert parameters["limits"]["maximum_linear_speed"] == 0.10
    assert parameters["limits"]["angular_speed"] == 0.60
    assert parameters["limits"]["maximum_angular_speed"] == 0.60


def test_no_gripper_controls_are_configured() -> None:
    config = load_mapping()
    parameters = config["rebot_xbox_twist"]["ros__parameters"]

    assert parameters["frames"]["end_effector"] == "Link6"
    assert "triggers" not in parameters
    assert parameters["safety"]["require_released_triggers_to_arm"] is False
    assert "rebot_xbox_sim_gripper" not in config


def test_hardware_driver_recovers_after_can_reconnect() -> None:
    launch_path = Path(__file__).parents[1] / "launch" / "hardware.launch.py"
    source = launch_path.read_text(encoding="utf-8")

    assert 'executable="piper_single_ctrl"' in source
    assert source.count("respawn=True") == 2
    assert source.count("respawn_delay=2.0") == 2


def test_hardware_adapter_exposes_standard_motor_enable_service() -> None:
    adapter_path = Path(__file__).parents[1] / "piperh_control" / "hardware_adapter.py"
    source = adapter_path.read_text(encoding="utf-8")

    assert 'self.declare_parameter("driver_enable_service", "/piperh/enable_srv")' in source
    assert (
        'self.declare_parameter("motor_enable_service", "/piperh/motor/set_enabled")'
        in source
    )
    assert "self._driver_enable_client.call_async(vendor_request)" in source
    assert "response.success = bool(vendor_response.enable_response)" in source


def test_motor_enable_latch_outlives_the_immediate_feedback_gate() -> None:
    adapter_path = Path(__file__).parents[1] / "piperh_control" / "hardware_adapter.py"
    source = adapter_path.read_text(encoding="utf-8")

    assert 'self.declare_parameter("feedback_timeout_sec", 0.75)' in source
    assert 'self.declare_parameter("motor_enable_latch_timeout_sec", 3.0)' in source
    assert "motor_enable_latch_timeout_sec must exceed feedback_timeout_sec" in source


def test_hardware_adapter_exposes_shared_teach_safety_status() -> None:
    adapter_path = Path(__file__).parents[1] / "piperh_control" / "hardware_adapter.py"
    source = adapter_path.read_text(encoding="utf-8")

    assert 'self.declare_parameter("arm_status_topic", "/piperh/arm_status")' in source
    assert "message = ArmStatus()" in source
    assert '"GRAVITY_COMPENSATION_ACTIVE" if gravity_active else' in source
    assert '"/piperh/control_state/dump"' in source
    assert '"IDLE" if enabled else "DISABLED"' in source


def test_default_gravity_configuration_cannot_output_torque() -> None:
    path = Path(__file__).parents[1] / "config" / "gravity_compensation.yaml"
    parameters = yaml.safe_load(path.read_text(encoding="utf-8"))["/**"]["ros__parameters"]
    assert parameters["gravity_real_torque_enabled"] is False
    assert parameters["gravity_mit_support_confirmed"] is False
    assert parameters["gravity_base_mount_confirmed"] is False
    assert parameters["gravity_expected_firmware"] == "S-V1.9-0"
    assert parameters["gravity_torque_scale"] == 0.10
    assert parameters["gravity_max_torque_nm"] == [0.0] * 6


def test_selected_arm_interlock_is_applied_to_adapter() -> None:
    launch_path = Path(__file__).parents[1] / "launch" / "hardware.launch.py"
    source = launch_path.read_text(encoding="utf-8")
    adapter_start = source.index('package="piperh_control"')
    assert '"gravity_require_selected_robot": LaunchConfiguration(' in source[adapter_start:]
