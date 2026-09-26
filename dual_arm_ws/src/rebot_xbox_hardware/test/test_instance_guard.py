from pathlib import Path

import pytest

from rebot_xbox_hardware.instance_guard import (
    ProcessInfo,
    acquire_lock,
    conflicting_processes,
    conflicting_node_names,
    lock_path,
    process_conflict_reason,
)


def test_lock_path_is_namespace_specific_and_sanitized(tmp_path):
    assert lock_path("/shop/arm 1", str(tmp_path)) == Path(
        tmp_path, "rebotarm-xbox-hardware-shop_arm_1.lock"
    )


def test_second_guard_cannot_acquire_same_lock(tmp_path):
    path = lock_path("rebotarm", str(tmp_path))
    first = acquire_lock(path)
    try:
        with pytest.raises(BlockingIOError):
            acquire_lock(path)
    finally:
        # Closing the owning descriptor releases the advisory lock.
        import os

        os.close(first)


def test_legacy_core_nodes_are_rejected_without_matching_probe_itself():
    assert conflicting_node_names(
        ["rebotarm_single_instance_probe", "move_group", "/camera"]
    ) == ["/move_group"]


def test_existing_piper_driver_is_rejected_before_opening_can_again():
    assert conflicting_node_names(
        ["/piper_ctrl_single_node", "/piperh_hardware_adapter", "/unrelated"]
    ) == ["/piper_ctrl_single_node", "/piperh_hardware_adapter"]


def _process(pid, command, *arguments):
    return ProcessInfo(pid, 1, command, tuple(arguments))


def test_residual_native_driver_and_rviz_processes_are_detected():
    driver = _process(101, "reBotArmContro", "/opt/robot/reBotArmController")
    rviz = _process(102, "rviz2", "/opt/ros/jazzy/lib/rviz2/rviz2", "-d", "view.rviz")

    assert process_conflict_reason(driver) == "executable=reBotArmController"
    assert process_conflict_reason(rviz) == "executable=rviz2"


def test_python_console_script_and_old_launch_are_detected():
    adapter = _process(
        201,
        "python3",
        "/usr/bin/python3",
        "/workspace/install/piperh_control/lib/piperh_control/hardware_adapter",
    )
    launch = _process(
        202,
        "ros2",
        "/usr/bin/python3",
        "/opt/ros/jazzy/bin/ros2",
        "launch",
        "rebot_xbox_hardware",
        "dual_arm_hardware.launch.py",
    )

    assert process_conflict_reason(adapter) == "executable=hardware_adapter"
    assert process_conflict_reason(launch) == "launch=dual_arm_hardware.launch.py"


def test_preflight_ignores_its_current_launch_ancestry_and_unrelated_processes():
    parent_launch = _process(
        301, "ros2", "ros2", "launch", "rebot_xbox_hardware",
        "dual_arm_hardware.launch.py"
    )
    unrelated = _process(302, "camera_node", "/opt/camera/camera_node")
    stale_servo = _process(303, "servo_node", "/opt/ros/lib/servo_node")

    assert conflicting_processes(
        [parent_launch, unrelated, stale_servo], excluded_pids={301}
    ) == [(stale_servo, "executable=servo_node")]
