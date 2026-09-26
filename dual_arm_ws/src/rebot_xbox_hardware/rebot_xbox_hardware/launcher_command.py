"""Pure command construction for the graphical real-arm selector."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ArmProfile:
    key: str
    label: str
    robot: str
    model: str
    default_channel: str
    namespace: str
    supports_gripper: bool
    supports_drag_teach: bool
    detail: str


ARM_PROFILES = (
    ArmProfile(
        key="rebotarm_dm",
        label="reBotArm（DM 串口版）",
        robot="rebotarm",
        model="dm",
        default_channel="/dev/rebot_can",
        namespace="rebotarm",
        supports_gripper=True,
        supports_drag_teach=True,
        detail="DM 模型 · gripper_tcp · 串口驱动 · 综合控制面板",
    ),
    ArmProfile(
        key="rebotarm_rs",
        label="reBotArm（RS CAN 版）",
        robot="rebotarm",
        model="rs",
        default_channel="can0",
        namespace="rebotarm",
        supports_gripper=True,
        supports_drag_teach=True,
        detail="RS 模型 · gripper_tcp · CAN 驱动 · 综合控制面板",
    ),
    ArmProfile(
        key="piperh",
        label="Piper-H（CAN 六轴版）",
        robot="piperh",
        model="piperh",
        default_channel="can0",
        namespace="piperh",
        supports_gripper=False,
        supports_drag_teach=False,
        detail="Piper-H 模型 · Link6 · CAN 驱动 · 预设动作面板",
    ),
)

_PROFILES_BY_KEY = {profile.key: profile for profile in ARM_PROFILES}


def profile_for(key: str) -> ArmProfile:
    """Look up a supported GUI selection."""
    try:
        return _PROFILES_BY_KEY[str(key)]
    except KeyError as error:
        raise ValueError(f"unsupported arm profile {key!r}") from error


def _bool_arg(value: bool) -> str:
    return "true" if value else "false"


def build_launch_arguments(
    profile: ArmProfile,
    *,
    channel: str,
    joy_device: str,
    use_rviz: bool = True,
    use_forbidden_zones: bool = True,
    use_teach: bool = True,
) -> list[str]:
    """Build an argv list without a shell or command interpolation."""
    channel = str(channel).strip()
    joy_device = str(joy_device).strip()
    if not channel or any(character.isspace() for character in channel):
        raise ValueError("通信通道不能为空或包含空白字符")
    if not joy_device or any(character.isspace() for character in joy_device):
        raise ValueError("手柄设备不能为空或包含空白字符")
    effective_teach = bool(use_teach and profile.supports_drag_teach)
    return [
        "launch",
        "rebot_xbox_hardware",
        "xbox_hardware_servo.launch.py",
        f"robot:={profile.robot}",
        f"model:={profile.model}",
        f"channel:={channel}",
        f"arm_namespace:={profile.namespace}",
        f"joy_device:={joy_device}",
        f"use_rviz:={_bool_arg(use_rviz)}",
        f"use_forbidden_zones:={_bool_arg(use_forbidden_zones)}",
        f"use_teach:={_bool_arg(effective_teach)}",
    ]
