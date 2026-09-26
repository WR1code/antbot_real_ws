from types import SimpleNamespace

from piper.piper_ctrl_single_node import PiperRosNode, joint_speed_percent


def test_joint_speed_percent_reads_adapter_field_and_clamps():
    assert joint_speed_percent(SimpleNamespace(velocity=[0.0] * 6 + [15.0])) == 15
    assert joint_speed_percent(SimpleNamespace(velocity=[0.0] * 6 + [150.0])) == 100
    assert joint_speed_percent(SimpleNamespace(velocity=[])) == 100


def test_motion_mode_is_only_sent_when_mode_or_speed_changes():
    calls = []
    node = object.__new__(PiperRosNode)
    node._last_motion_command = None
    node.piper = SimpleNamespace(
        MotionCtrl_2=lambda control, mode, speed: calls.append((control, mode, speed))
    )
    node.get_logger = lambda: SimpleNamespace(info=lambda message: None)

    node._set_motion_mode(0x01, 15)
    node._set_motion_mode(0x01, 15)
    node._set_motion_mode(0x01, 10)
    node._set_motion_mode(0x00, 50)

    assert calls == [(0x01, 0x01, 15), (0x01, 0x01, 10), (0x01, 0x00, 50)]
