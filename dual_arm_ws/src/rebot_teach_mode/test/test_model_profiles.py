import pytest

from rebot_teach_mode.model_profiles import teaching_parameters_for_model


def test_dm_profile_uses_dm_limits_and_isolated_storage():
    profile = teaching_parameters_for_model(" DM ")
    assert profile["robot_model"] == "rebotarm_dm"
    assert profile["driver_service_timeout_sec"] == 15.0
    assert profile["feedback_timeout_sec"] == 0.75
    assert profile["status_timeout_sec"] == 0.75
    assert profile["replay_speed_scale"] == 1.0
    assert profile["joint_lower_limits"] == [
        -2.8,
        -3.14,
        -3.14,
        -1.87,
        -1.57,
        -3.14,
    ]
    assert profile["joint_upper_limits"] == [
        2.8,
        0.005,
        0.005,
        1.57,
        1.57,
        3.14,
    ]
    assert profile["trajectory_path"].endswith("latest_dm.motion.json")
    assert profile["action_library_dir"].endswith("action_groups/dm")
    assert profile["shape_pen_mount_offset_m"] == pytest.approx(0.1064)


def test_profiles_are_returned_as_independent_mappings():
    first = teaching_parameters_for_model("dm")
    first["joint_lower_limits"][0] = 123.0
    assert teaching_parameters_for_model("dm")["joint_lower_limits"][0] == -2.8


def test_rs_profile_uses_its_gripper_front_offset():
    assert teaching_parameters_for_model("rs")[
        "shape_pen_mount_offset_m"
    ] == pytest.approx(0.1572)


def test_piperh_profile_has_independent_storage_frames_and_limits():
    profile = teaching_parameters_for_model("piperh")
    assert profile["robot_model"] == "piperh"
    assert profile["feedback_timeout_sec"] == 2.0
    assert profile["tcp_link_name"] == "Link6"
    assert profile["trajectory_path"].startswith("/home/w/project/piperh/")
    assert profile["action_library_dir"].startswith("/home/w/project/piperh/")
    assert profile["joint_lower_limits"] == [
        -2.618, 0.0, -2.96706, -2.356195, -1.56207, -3.14
    ]
    assert profile["joint_upper_limits"] == [
        2.618, 3.14, 0.0, 2.356195, 1.56207, 3.14
    ]


def test_unknown_model_is_rejected():
    with pytest.raises(ValueError, match="expected dm, rs, or piperh"):
        teaching_parameters_for_model("unknown")
