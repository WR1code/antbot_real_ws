import pytest

from rebot_xbox_hardware.launcher_command import (
    ARM_PROFILES,
    build_launch_arguments,
    profile_for,
)


def test_selector_exposes_all_three_real_arm_profiles():
    assert [profile.key for profile in ARM_PROFILES] == [
        "rebotarm_dm",
        "rebotarm_rs",
        "piperh",
    ]


@pytest.mark.parametrize(
    "key,expected",
    [
        ("rebotarm_dm", ("robot:=rebotarm", "model:=dm", "channel:=/dev/rebot_can")),
        ("rebotarm_rs", ("robot:=rebotarm", "model:=rs", "channel:=can0")),
        ("piperh", ("robot:=piperh", "model:=piperh", "channel:=can0")),
    ],
)
def test_command_matches_selected_driver_and_model(key, expected):
    profile = profile_for(key)
    command = build_launch_arguments(
        profile,
        channel=profile.default_channel,
        joy_device="/dev/input/js0",
    )
    assert command[:3] == [
        "launch",
        "rebot_xbox_hardware",
        "xbox_hardware_servo.launch.py",
    ]
    assert all(value in command for value in expected)
    assert f"arm_namespace:={profile.namespace}" in command


def test_piperh_never_starts_unsupported_drag_teaching():
    command = build_launch_arguments(
        profile_for("piperh"),
        channel="can0",
        joy_device="/dev/input/js0",
        use_teach=True,
    )
    assert "use_teach:=false" in command


@pytest.mark.parametrize("channel,joy", [("", "/dev/input/js0"), ("can 0", "/dev/input/js0"), ("can0", "")])
def test_invalid_device_arguments_are_rejected(channel, joy):
    with pytest.raises(ValueError):
        build_launch_arguments(
            profile_for("piperh"), channel=channel, joy_device=joy
        )


def test_unknown_profile_is_rejected():
    with pytest.raises(ValueError):
        profile_for("unknown")
