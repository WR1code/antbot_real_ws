from pathlib import Path
import xml.etree.ElementTree as ET

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


def test_official_piper_h_kinematics_and_physical_limits_are_installed() -> None:
    path = (
        Path(__file__).parents[2]
        / "piper_h_description"
        / "urdf"
        / "piper_h_description.urdf"
    )
    robot = ET.parse(path).getroot()
    expected = {
        "joint2": ("0 0 0", "1.5707963 -0.1357866 -3.1415926", "0", "3.1415926"),
        "joint3": ("0.28503 0 0", "0 0 -1.7938494", "-2.9670597", "0"),
        "joint4": ("-0.02198 -0.25575 0", "1.5707963 0 0", "-2.3561944", "2.3561944"),
        "joint5": ("0 0 0", "-1.5707963 0 0", "-1.5620696", "1.5620696"),
        "joint6": ("0 -0.0963 0", "1.5707963 0 0", "-2.0943951", "2.0943951"),
    }
    for name, (xyz, rpy, lower, upper) in expected.items():
        joint = robot.find(f"./joint[@name='{name}']")
        assert joint is not None
        assert joint.find("origin").attrib == {"xyz": xyz, "rpy": rpy}
        assert joint.find("axis").attrib["xyz"] == "0 0 1"
        assert joint.find("limit").attrib["lower"] == lower
        assert joint.find("limit").attrib["upper"] == upper


def test_moveit_tolerance_does_not_expand_hardware_limits() -> None:
    path = (
        Path(__file__).parents[2]
        / "piperh_moveit_config"
        / "config"
        / "joint_limits.yaml"
    )
    limits = yaml.safe_load(path.read_text(encoding="utf-8"))["joint_limits"]
    assert limits["joint2"]["min_position"] == -0.1
    assert limits["joint3"]["max_position"] == 0.1

    from piperh_control.hardware_adapter import LOWER, UPPER

    assert LOWER[1] == 0.0
    assert UPPER[2] == 0.0
