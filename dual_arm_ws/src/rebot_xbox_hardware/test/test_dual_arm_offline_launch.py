from pathlib import Path


def _source():
    path = (
        Path(__file__).parents[1]
        / "launch"
        / "dual_arm_offline_preview.launch.py"
    )
    return path.read_text(encoding="utf-8")


def test_offline_launch_exposes_the_two_robot_selector_models():
    source = _source()

    assert 'choices=["rebotarm", "piperh"]' in source
    assert '_moveit_config("rebotarm"' in source
    assert '_moveit_config("piperh")' in source
    assert '"dual_arm.offline_preview": True' in source


def test_offline_launch_uses_mock_control_and_never_starts_real_drivers():
    source = _source()

    assert 'executable="ros2_control_node"' in source
    assert '"allow_hardware": "false"' in source
    assert '"preview_only": "true"' in source
    assert 'package="rebotarm_bringup"' not in source
    assert 'package="piperh_control"' not in source
    assert 'package="joy_linux"' not in source


def test_offline_launch_embeds_moveit_in_the_shared_free_planning_page():
    assert '"rebot_demo.integrate_motion_planning": True' in _source()


def test_real_launch_embeds_moveit_in_the_shared_free_planning_page():
    path = Path(__file__).parents[1] / "launch" / "dual_arm_hardware.launch.py"
    source = path.read_text(encoding="utf-8")

    assert '"rebot_demo.integrate_motion_planning": True' in source
    assert '"rebot_demo.integrate_motion_planning": False' not in source


def test_real_launch_integrates_complete_antbot_operator_without_second_rviz():
    path = Path(__file__).parents[1] / "launch" / "dual_arm_hardware.launch.py"
    source = path.read_text(encoding="utf-8")

    assert 'get_package_share_directory("antbot_real_bringup")' in source
    assert '"use_operator_rviz": "false"' in source
    assert '"start_pulse": "false"' in source
    assert '"publish_placeholder_pose": "false"' in source
    assert '"topics.control_target": "/xbox/control_target"' in source


def test_real_launch_freezes_its_rviz_flag_before_nested_chassis_launches():
    path = Path(__file__).parents[1] / "launch" / "dual_arm_hardware.launch.py"
    source = path.read_text(encoding="utf-8")

    assert 'use_rviz = LaunchConfiguration("use_rviz").perform(context)' in source
    assert "condition=IfCondition(use_rviz)" in source


def test_real_launch_does_not_reemit_shutdown_when_already_stopping():
    path = Path(__file__).parents[1] / "launch" / "dual_arm_hardware.launch.py"
    source = path.read_text(encoding="utf-8")

    assert "def shutdown_if_guard_exits(event, context):" in source
    assert "if context.is_shutdown:" in source
    assert "on_exit=shutdown_if_guard_exits" in source
    assert 'sigterm_timeout="2.0"' in source
    assert 'sigkill_timeout="2.0"' in source


def test_real_launch_mounts_live_piper_at_inverted_cad_pose():
    path = Path(__file__).parents[1] / "launch" / "dual_arm_hardware.launch.py"
    source = path.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("piper_x", default_value="-0.0025")' in source
    assert 'DeclareLaunchArgument("piper_y", default_value="-0.1350")' in source
    assert 'DeclareLaunchArgument("piper_z", default_value="0.9543")' in source
    assert 'DeclareLaunchArgument("piper_roll", default_value="1.57079632679")' in source
    assert 'DeclareLaunchArgument("piper_pitch", default_value="0.0")' in source
    assert 'DeclareLaunchArgument("piper_yaw", default_value="0.0")' in source


def test_real_launch_integrates_read_only_piper_pulse_nodes():
    path = Path(__file__).parents[1] / "launch" / "dual_arm_hardware.launch.py"
    source = path.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("piper_pulse_serial_port", default_value="")' in source
    assert 'executable="piper_tool_geometry"' in source
    assert 'executable="piper_pulse_target"' in source
    assert 'executable="pressure_serial_bridge"' in source
    assert 'parameters=[pulse_zero_config, {"port": piper_pulse_serial_port}]' in source
    assert '"mount_pitch": LaunchConfiguration("piper_pitch")' in source
    assert '"mount_roll": LaunchConfiguration("piper_roll")' in source


def test_real_launch_uses_one_mount_for_live_and_planned_piper_models():
    path = Path(__file__).parents[1] / "launch" / "dual_arm_hardware.launch.py"
    source = path.read_text(encoding="utf-8")

    assert '"root_frame": "piperh_planning_world"' in source
    assert '"base_link", "piperh/piperh_planning_world"' in source
    assert '"base_link", "piperh_planning_world"' in source
    assert 'name="piperh_moveit_mount_tf"' in source
    assert '"base_link", "piperh/world"' not in source


def test_dual_launches_namespace_rviz_markers_for_each_robot():
    offline_source = _source()
    real_path = Path(__file__).parents[1] / "launch" / "dual_arm_hardware.launch.py"
    real_source = real_path.read_text(encoding="utf-8")

    for source in (offline_source, real_source):
        assert '"visualization_frame_prefix": "rebotarm"' in source
        assert '"visualization_frame_prefix": "piperh"' in source
