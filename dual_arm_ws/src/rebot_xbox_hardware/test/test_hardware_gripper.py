from rebot_xbox_hardware.hardware_gripper import gripper_command


def test_trigger_amount_scales_real_gripper_velocity():
    assert gripper_command(0.5, 0.0, 1.0, 5.0, 0.0, 2.0, 0.1) == (5.0, 1.0)
    assert gripper_command(0.0, 0.25, 4.0, 5.0, 0.0, 2.0, 0.1) == (0.0, 0.5)


def test_release_conflict_timeout_and_torque_all_stop():
    args = (0.0, 0.0, 2.0, 5.0, 0.0, 2.0, 0.1)
    assert gripper_command(*args) is None
    assert gripper_command(1.0, 1.0, *args[2:]) is None
    assert gripper_command(1.0, 0.0, *args[2:], stale=True) is None
    assert gripper_command(0.0, 1.0, *args[2:], torque_latched=True) is None
