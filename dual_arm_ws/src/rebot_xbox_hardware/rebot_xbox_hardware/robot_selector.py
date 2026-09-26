"""Validate the robot selected by the unified real-hardware launch."""

from __future__ import annotations

from dataclasses import dataclass


_ROBOT_ALIASES = {
    "rebotarm": "rebotarm",
    "rebot": "rebotarm",
    "piperh": "piperh",
    "piper-h": "piperh",
    "piper_h": "piperh",
}


@dataclass(frozen=True)
class RobotSelection:
    """Normalized launch selection used before any hardware node is started."""

    robot: str
    model: str
    namespace: str


def select_robot(robot: str, model: str, namespace: str = "") -> RobotSelection:
    """Return a coherent robot/model/namespace tuple or reject it early."""
    requested_robot = str(robot).strip().lower()
    try:
        normalized_robot = _ROBOT_ALIASES[requested_robot]
    except KeyError as error:
        raise ValueError(
            f"unsupported robot {robot!r}; expected rebotarm or piperh"
        ) from error

    requested_namespace = str(namespace).strip("/")
    if normalized_robot == "rebotarm":
        normalized_model = str(model).strip().lower()
        if normalized_model not in {"dm", "rs"}:
            raise ValueError(
                f"unsupported reBotArm model {model!r}; expected dm or rs"
            )
        return RobotSelection(
            robot="rebotarm",
            model=normalized_model,
            namespace=requested_namespace or "rebotarm",
        )

    # Piper-H is a single, six-axis, no-gripper model.  Its current official
    # driver adapter intentionally exposes fixed /piperh and MoveIt endpoints,
    # so accepting another namespace would make the displayed and controlled
    # robot disagree.
    if requested_namespace and requested_namespace != "piperh":
        raise ValueError(
            "Piper-H currently requires arm_namespace:=piperh"
        )
    return RobotSelection(robot="piperh", model="piperh", namespace="piperh")
