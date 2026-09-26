import pytest

from rebot_xbox_hardware.robot_selector import select_robot


@pytest.mark.parametrize("model", ["dm", "rs", "DM", "RS"])
def test_rebotarm_keeps_selected_hardware_model(model):
    result = select_robot("rebotarm", model)
    assert result.robot == "rebotarm"
    assert result.model == model.lower()
    assert result.namespace == "rebotarm"


@pytest.mark.parametrize("alias", ["piperh", "piper-h", "piper_h"])
def test_piper_aliases_select_the_piper_model_and_namespace(alias):
    result = select_robot(alias, "dm")
    assert result.robot == "piperh"
    assert result.model == "piperh"
    assert result.namespace == "piperh"


def test_custom_rebotarm_namespace_is_preserved():
    assert select_robot("rebotarm", "dm", "/arm_a/").namespace == "arm_a"


@pytest.mark.parametrize(
    "robot,model,namespace",
    [
        ("unknown", "dm", ""),
        ("rebotarm", "piperh", ""),
        ("piperh", "dm", "another_arm"),
    ],
)
def test_incoherent_selections_are_rejected(robot, model, namespace):
    with pytest.raises(ValueError):
        select_robot(robot, model, namespace)
