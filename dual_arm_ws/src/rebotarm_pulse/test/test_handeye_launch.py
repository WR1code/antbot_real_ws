import importlib.util
from pathlib import Path

import pytest


LAUNCH_FILE = Path(__file__).parents[1] / "launch" / "handeye_calibrate.launch.py"
SPEC = importlib.util.spec_from_file_location("handeye_calibrate_launch", LAUNCH_FILE)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def test_all_supported_tag_families_accept_their_last_id():
    for family, code_count in MODULE.TAG_FAMILY_CODE_COUNTS.items():
        MODULE._validate_tag_selection(family, code_count - 1)


@pytest.mark.parametrize(
    ("family", "tag_id"),
    [("not-a-family", 0), ("16h5", -1), ("16h5", 30), ("36h11", 587)],
)
def test_invalid_tag_selection_is_rejected(family, tag_id):
    with pytest.raises(ValueError):
        MODULE._validate_tag_selection(family, tag_id)


def test_both_calibration_layouts_are_supported_and_have_matching_sequences():
    for calibration_type in ("eye_on_base", "eye_in_hand"):
        MODULE._validate_calibration_type(calibration_type)
        assert MODULE.AUTO_SEQUENCE_BY_TYPE[calibration_type]
        assert MODULE.AUTO_ACTION_PREFIX_BY_TYPE[calibration_type]

    with pytest.raises(ValueError, match="unsupported calibration_type"):
        MODULE._validate_calibration_type("eye_on_hand")


def test_easy_handeye_and_moveit_calibration_backends_are_supported():
    MODULE._validate_calibration_backend("easy_handeye2")
    MODULE._validate_calibration_backend("moveit_calibration")

    with pytest.raises(ValueError, match="unsupported calibration_backend"):
        MODULE._validate_calibration_backend("unknown")


def test_single_tag_and_a4_board_targets_are_supported():
    MODULE._validate_target_type("single_tag", "36h11")
    MODULE._validate_target_type("a4_4tag_board", "36h11")
    MODULE._validate_target_type("charuco_a4_5x7", "36h11")

    with pytest.raises(ValueError, match="unsupported target_type"):
        MODULE._validate_target_type("tag_bundle", "36h11")
    with pytest.raises(ValueError, match="requires tag_family '36h11'"):
        MODULE._validate_target_type("a4_4tag_board", "25h9")


def test_automatic_sequence_topics_are_configurable_for_namespaced_robots():
    source = LAUNCH_FILE.read_text(encoding="utf-8")
    assert '"teach_status_topic": value("teach_status_topic")' in source
    assert '"joint_state_topic": value("joint_state_topic")' in source
    assert '"teach_cancel_service": value("teach_cancel_service")' in source


def test_handeye_launch_declares_automatic_sequence_node():
    source = LAUNCH_FILE.read_text(encoding="utf-8")

    assert 'executable="auto_handeye_sequence"' in source
    assert '"calibration_type": calibration_type' in source
    assert '"action_name_prefix": auto_action_prefix' in source
    assert '"eye_in_hand": "自动手眼标定_DM_眼在手上_12姿态"' in source
    assert 'default_value="12"' in source
    assert 'condition=IfCondition(LaunchConfiguration("use_auto_sequence"))' in source
    assert 'DeclareLaunchArgument("use_auto_sequence", default_value="true")' in source
    assert 'if calibration_backend == "easy_handeye2":' in source


def test_handeye_launch_disables_easy_handeye_dummy_camera_transform():
    source = LAUNCH_FILE.read_text(encoding="utf-8")

    assert '"publish_dummy_transform": "false"' in source


def test_handeye_launch_declares_four_tag_board_fusion_node():
    source = LAUNCH_FILE.read_text(encoding="utf-8")

    assert 'executable="apriltag_board_pose"' in source
    assert 'detected_ids = [0, 1, 2, 3] if board_mode else [tag_id]' in source
    assert 'DeclareLaunchArgument("board_spacing_x_m", default_value="0.100")' in source
    assert 'DeclareLaunchArgument("board_spacing_y_m", default_value="0.120")' in source


def test_handeye_launch_declares_fixed_charuco_board_node():
    source = LAUNCH_FILE.read_text(encoding="utf-8")

    assert 'executable="charuco_board_pose"' in source
    assert '"squares_x": 5' in source
    assert '"squares_y": 7' in source
    assert '"square_length_m": 0.035' in source
    assert '"marker_length_m": 0.026' in source
    assert 'DeclareLaunchArgument("charuco_minimum_corners", default_value="6")' in source
