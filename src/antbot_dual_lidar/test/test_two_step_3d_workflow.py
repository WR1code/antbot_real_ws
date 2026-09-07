from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
PROJECT = ROOT.parent
ISAAC_SCRIPTS = PROJECT / "isaac" / "scripts"


def read(name):
    return (ISAAC_SCRIPTS / name).read_text(encoding="utf-8")


def test_step1_runs_3d_lidar_and_saves_both_maps():
    script = read("step1_mapping.sh")
    assert "--lidar-mode 3d" in script
    assert "--lidar-profile mapping" in script
    assert '--control-usd "$SCENE_USD"' in script
    assert "--test-layout empty" in script
    assert "workflow_save_map" in script
    assert "workflow_save_pointcloud" in script
    assert "workflow_create_map_bundle" in script
    assert "isaac_mapping_stable.launch.py" in script
    assert "/antbot/lidar/map_points" in script
    assert "phase4b_capture_node" in script
    assert 'preview_output_frame:=map' in script
    assert 'preview_voxel_size_m:=0.03' in script
    assert 'preview_pixel_stride:=2' in script
    assert 'preview_min_observations:=2' in script
    assert "/antbot/rgbd/accumulated_cloud" in script
    assert "step1_runs" in script
    assert "promote_step1_run.sh" in script


def test_mapping_candidates_are_separate_from_official_map_bundle():
    common = read("workflow_common.sh")
    promote = read("promote_step1_run.sh")
    assert 'MAP_OUTPUT_DIR="${ANTBOT_MAP_OUTPUT_DIR:-$MAP_PROJECT_DIR}"' in common
    assert '--output-directory "$MAP_OUTPUT_DIR"' in common
    assert '--output "$MAP_OUTPUT_DIR"' in common
    assert 'RUNS_ROOT="$MAP_PROJECT_DIR/step1_runs"' in promote
    assert 'BACKUP_DIR="$MAP_PROJECT_DIR/official_backups/$PROMOTION_ID"' in promote


def test_repeated_promotion_is_idempotent():
    promote = read("promote_step1_run.sh")
    assert "OFFICIAL_IS_IDENTICAL=true" in promote
    assert 'cmp -s -- "$RUN_DIR/$artifact" "$MAP_PROJECT_DIR/$artifact"' in promote
    assert "无需重复发布" in promote


def test_mapping_and_unified_navigation_use_fail_fast_bounded_waits():
    common = read("workflow_common.sh")
    for step in ("step1_mapping.sh", "unified_edit_navigation.sh"):
        assert 'source "$SCRIPT_DIR/workflow_common.sh"' in read(step)
    assert 'exec bash "$SCRIPT_DIR/unified_edit_navigation.sh" "$@"' in read(
        "step2_waypoints.sh"
    )

    assert "WORKFLOW_GROUP_LOGS" in common
    assert "workflow_check_groups_alive" in common
    assert 'tail -n "${ANTBOT_FAILURE_LOG_LINES:-30}"' in common
    assert 'wait_for_ros.py' in common
    assert '--watch-pid "$pid"' in common
    assert 'workflow_wait_for_topic()' in common
    assert 'workflow_wait_for_service()' in common
    assert 'workflow_wait_for_action()' in common


def test_mapping_and_unified_navigation_are_locked_and_clean_stale_groups():
    common = read("workflow_common.sh")
    assert "flock -n" in common
    assert "workflow_lock_owner_is_active" in common
    assert "workflow_write_lock_metadata" in common
    assert 'exec {WORKFLOW_LOCK_FD}>&-' in common
    assert "workflow_run_without_lock" in common
    assert "workflow_start_without_lock" in common
    assert "遗留工作流锁已清理" in common
    assert "workflow_prepare_processes()" in common
    for step in ("step1_mapping.sh", "unified_edit_navigation.sh"):
        script = read(step)
        assert "workflow_acquire_lock" in script
        assert "workflow_prepare_processes" in script


def test_stable_mapping_launch_allows_slow_isaac_bond_startup():
    launch_file = (
        Path(__file__).resolve().parents[1]
        / "launch"
        / "isaac_mapping_stable.launch.py"
    ).read_text(encoding="utf-8")
    assert '"bond_timeout": 15.0' in launch_file
    assert '"node_names": ["slam_toolbox"]' in launch_file


def test_heavy_cloud_nodes_keep_tf_current_during_processing():
    package = Path(__file__).resolve().parents[1] / "antbot_dual_lidar"
    for filename in ("fixed_frame_mapper.py", "dynamic_obstacle_monitor.py"):
        source = (package / filename).read_text(encoding="utf-8")
        assert "MultiThreadedExecutor(num_threads=2)" in source
        assert "executor.add_node(node)" in source
        assert "executor.spin()" in source
        assert "rclpy.spin(node)" not in source


def test_description_overlay_is_checked_before_each_step_starts():
    common = read("workflow_common.sh")
    assert "payload_tower.xacro" in common
    assert "isaac_mapping_stable.launch.py" in common
    assert "colcon build --symlink-install --packages-select antbot_description" in common
    for step in ("step1_mapping.sh", "unified_edit_navigation.sh"):
        assert "workflow_require_builds" in read(step)
    assert "workflow_require_stable_mapping_launch" in read("step1_mapping.sh")


def test_step2_is_the_only_live_edit_navigation_workflow():
    script = read("step2_waypoints.sh")
    unified = read("unified_edit_navigation.sh")
    assert 'exec bash "$SCRIPT_DIR/unified_edit_navigation.sh" "$@"' in script
    assert not (ISAAC_SCRIPTS / "step3_navigation.sh").exists()
    assert "第二阶段：地图编辑 + 实时感知 + 航点巡航" in unified
    assert "waypoint_navigation.launch.py" in unified
    assert 'waypoints_file:="$WAYPOINT_FILE"' in unified
    assert 'speed_zone_file:="$MAP_PROJECT_DIR/speeds.speed.json"' in unified
    assert 'stuck_history_file:="$MAP_PROJECT_DIR/stuck_history.json"' in unified


def test_offline_editor_joint_states_use_the_published_robot_description():
    launch_file = (
        Path(__file__).resolve().parents[2]
        / "antbot"
        / "antbot_navigation"
        / "launch"
        / "offline_waypoint_editor.launch.py"
    ).read_text(encoding="utf-8")
    assert launch_file.count(
        "('robot_description', '/antbot/robot_description')"
    ) == 2


def test_step2_uses_3d_obstacles_amcl_projection_and_xml_waypoints():
    script = read("unified_edit_navigation.sh")
    assert "--lidar-mode 3d" in script
    assert "--lidar-profile navigation" in script
    assert '--control-usd "$SCENE_USD"' in script
    assert "--test-layout empty" in script
    assert "isaac_3d_navigation.launch.py" in script
    assert "use_amcl:=true" in script
    assert "waypoint_navigation.launch.py" in script
    assert "start_nav2:=false" in script
    assert "wr_navigate.py" in script
    assert "offline_preview_publisher" in script
    assert "route_file" in script
    assert "routes" in script


def test_step2_route_runner_publishes_progress_for_rviz_panel():
    runner = (
        ROOT / "src" / "robotcar_navigation" / "scripts" / "wr_navigate.py"
    ).read_text(encoding="utf-8")
    panel = (
        ROOT / "src" / "robotcar_navigation" / "src"
        / "waypoint_manager_panel.cpp"
    ).read_text(encoding="utf-8")
    assert '"/waterplus/route_progress"' in runner
    assert '"current_index"' in runner
    assert '"completed"' in runner
    assert '"total"' in runner
    assert '"/waterplus/route_progress"' in panel
    assert "QProgressBar" in panel
    assert "showRouteProgress" in panel


def test_step1_and_step2_rviz_show_actual_speed_and_live_camera():
    navigation = ROOT / "src" / "antbot" / "antbot_navigation"
    mapping_rviz = (navigation / "rviz" / "mapping_3d.rviz").read_text(
        encoding="utf-8"
    )
    waypoint_rviz = (
        navigation / "rviz" / "waypoint_navigation.rviz"
    ).read_text(encoding="utf-8")
    status_panel = (
        ROOT / "src" / "robotcar_navigation" / "src"
        / "vehicle_status_panel.cpp"
    ).read_text(encoding="utf-8")

    for rviz_config in (mapping_rviz, waypoint_rviz):
        assert "robotcar_navigation/VehicleStatusPanel" in rviz_config
        assert "车辆状态与相机" in rviz_config

    assert '"/odometry/filtered"' in status_panel
    assert '"/antbot/camera/color/image_raw"' in status_panel
    assert "std::hypot(vx, vy)" in status_panel
    assert "m/s" in status_panel
    assert "rad/s" in status_panel
    assert "rclcpp::SensorDataQoS()" in status_panel

    step1 = read("step1_mapping.sh")
    step2 = read("unified_edit_navigation.sh")
    assert "--topic /odometry/filtered" in step1
    assert "--topic /odometry/filtered" in step2
    assert "--publisher /antbot/camera/color/image_raw" in step1
    assert "--publisher /antbot/camera/color/image_raw" in step2


def test_robot_intent_battery_and_led_interfaces_are_wired_into_rviz():
    robotcar = ROOT / "src" / "robotcar_navigation"
    intent = (robotcar / "scripts" / "robot_intent_monitor.py").read_text(
        encoding="utf-8"
    )
    vehicle_panel = (
        robotcar / "src" / "vehicle_status_panel.cpp"
    ).read_text(encoding="utf-8")
    cmake = (robotcar / "CMakeLists.txt").read_text(encoding="utf-8")

    assert '"/antbot/robot_intent"' in intent
    assert '"/antbot/led_intent"' in intent
    assert '"准备向左转"' in intent
    assert '"passing_right"' in intent
    assert '"/antbot/passing_request"' in intent
    assert '"/battery"' in vehicle_panel
    assert '"/antbot/vehicle_status"' in vehicle_panel
    assert "robot_intent_monitor.py" in cmake
    assert "robot_intent_monitor.py" in read("step1_mapping.sh")


def test_named_keepouts_are_editable_saved_and_applied_to_nav2():
    robotcar = ROOT / "src" / "robotcar_navigation"
    manager = (robotcar / "scripts" / "keepout_zone_manager.py").read_text(
        encoding="utf-8"
    )
    plugin = (robotcar / "robotcar_navigation_plugin.xml").read_text(
        encoding="utf-8"
    )
    nav_params = (
        ROOT / "src" / "antbot_dual_lidar" / "config"
        / "isaac_3d_nav2_params.yaml"
    ).read_text(encoding="utf-8")
    rviz = (
        ROOT / "src" / "antbot" / "antbot_navigation" / "rviz"
        / "waypoint_navigation.rviz"
    ).read_text(encoding="utf-8")

    assert "/keepout_filter_mask" in manager
    assert "/costmap_filter_info" in manager
    assert "/waterplus/save_keepout_group" in manager
    assert "/waterplus/rename_keepout" in manager
    assert "def rename_zone(" in manager
    assert "os.replace" in manager
    assert "robotcar_navigation/AddKeepoutZone" in plugin
    assert nav_params.count('plugin: "nav2_costmap_2d::KeepoutFilter"') == 2
    assert "keepout_filter" in nav_params
    assert "robotcar_navigation/AddKeepoutZone" in rviz
    assert "/waterplus/keepout_markers" in rviz
    assert "unified_edit_navigation.sh" in read("step2_waypoints.sh")
    assert "keepout_file" in read("unified_edit_navigation.sh")


def test_speed_zones_are_named_saved_and_enforced_by_nav2():
    robotcar = ROOT / "src" / "robotcar_navigation"
    manager = (robotcar / "scripts" / "speed_zone_manager.py").read_text(
        encoding="utf-8"
    )
    panel = (robotcar / "src" / "waypoint_manager_panel.cpp").read_text(
        encoding="utf-8"
    )
    nav_params = (
        ROOT / "src" / "antbot_dual_lidar" / "config"
        / "isaac_3d_nav2_params.yaml"
    ).read_text(encoding="utf-8")
    assert 'info.type = 2' in manager
    assert 'info.multiplier = 0.01' in manager
    assert '"/speed_filter_mask"' in manager
    assert '"/waterplus/save_speed_zone_group"' in manager
    assert '"/waterplus/rename_speed_zone"' in manager
    assert "def rename_zone(" in manager
    assert '"/waterplus/get_speed_zone_names"' in panel
    assert 'plugin: "nav2_costmap_2d::SpeedFilter"' in nav_params
    assert 'filter_info_topic: /speed_costmap_filter_info' in nav_params


def test_zone_boundaries_are_user_sized_and_manager_panel_scrolls():
    robotcar = ROOT / "src" / "robotcar_navigation"
    keepout_tool = (robotcar / "src" / "add_keepout_zone_tool.cpp").read_text(
        encoding="utf-8"
    )
    speed_tool = (robotcar / "src" / "add_speed_zone_tool.cpp").read_text(
        encoding="utf-8"
    )
    panel = (robotcar / "src" / "waypoint_manager_panel.cpp").read_text(
        encoding="utf-8"
    )
    for tool in (keepout_tool, speed_tool):
        assert '"形状"' in tool
        assert 'addOption("矩形"' in tool
        assert 'addOption("正方形"' in tool
        assert 'addOption("椭圆"' in tool
        assert 'addOption("圆形"' in tool
        assert "updatePreview" in tool
        assert "BillboardLine" in tool
        assert "constrainedEnd" in tool
        assert "zone.shape = shapeName()" in tool
        assert "processMouseEvent" in tool
    assert "QScrollArea" in panel
    assert "setWidgetResizable(true)" in panel
    assert "Qt::ScrollBarAlwaysOff" in panel
    assert "outer->addWidget(status_label_)" in panel
    assert 'tr("输入并绘制禁区")' in panel
    assert 'tr("输入并绘制限速区")' in panel
    assert 'tr("新禁区形状：")' in panel
    assert 'tr("新限速区形状：")' in panel
    assert 'subProp("形状")->setValue(shape)' in panel
    assert 'tool_manager->setCurrentTool(tool)' in panel
    assert 'tr("输入选中禁区的新名称")' in panel
    assert 'tr("输入选中限速区的新名称")' in panel
    assert '"/waterplus/rename_keepout"' in panel
    assert '"/waterplus/rename_speed_zone"' in panel


def test_zone_shapes_are_persisted_rasterized_and_rendered():
    robotcar = ROOT / "src" / "robotcar_navigation"
    keepout_manager = (robotcar / "scripts" / "keepout_zone_manager.py").read_text(
        encoding="utf-8"
    )
    speed_manager = (robotcar / "scripts" / "speed_zone_manager.py").read_text(
        encoding="utf-8"
    )
    for manager in (keepout_manager, speed_manager):
        assert 'SUPPORTED_SHAPES = {"rectangle", "square", "ellipse", "circle"}' in manager
        assert 'raw.get("shape", "rectangle")' in manager
        assert 'zone.get("shape", "rectangle") in {"ellipse", "circle"}' in manager
        assert "Marker.CYLINDER" in manager
        assert '"version": 2' in manager


def test_dynamic_obstacles_fuse_lidar_and_rgbd_against_saved_clouds():
    monitor = (
        ROOT / "src" / "antbot_dual_lidar" / "antbot_dual_lidar"
        / "dynamic_obstacle_monitor.py"
    ).read_text(encoding="utf-8")
    rviz = (
        ROOT / "src" / "antbot" / "antbot_navigation" / "rviz"
        / "waypoint_navigation.rviz"
    ).read_text(encoding="utf-8")
    assert '"/antbot/offline_map_points"' in monitor
    assert '"/antbot/rgbd/offline_cloud"' in monitor
    assert '"/antbot/dynamic_obstacles/lidar_points"' in monitor
    assert '"/antbot/dynamic_obstacles/visual_points"' in monitor
    assert "confirmation_hits" in monitor
    assert "/antbot/dynamic_obstacles/lidar_points" in rviz
    assert "/antbot/dynamic_obstacles/visual_points" in rviz


def test_view_stuck_history_and_dynamic_footprint_are_published():
    awareness = (
        ROOT / "src" / "robotcar_navigation" / "scripts"
        / "robot_awareness_monitor.py"
    ).read_text(encoding="utf-8")
    assert '"机器人现在正在看这里"' in awareness
    assert 'f"这里过去发生过 {score} 次卡死"' in awareness
    assert '"/spin/_action/status"' not in awareness  # generated from action name
    assert 'f"/{action_name}/_action/status"' in awareness
    assert '"/local_costmap/footprint"' in awareness
    assert '"/global_costmap/footprint"' in awareness
    assert "speed_margin_gain" in awareness
    assert "os.replace" in awareness


def test_charging_pause_and_task_history_are_available_in_stage3_panel():
    robotcar = ROOT / "src" / "robotcar_navigation"
    panel = (robotcar / "src" / "waypoint_manager_panel.cpp").read_text(
        encoding="utf-8"
    )
    navigator = (robotcar / "src" / "wp_navi_server.cpp").read_text(
        encoding="utf-8"
    )
    route_runner = (robotcar / "scripts" / "wr_navigate.py").read_text(
        encoding="utf-8"
    )

    assert '"/waterplus/navi_charger"' in panel
    assert '"/waterplus/charge_result"' in panel
    assert "pauseOrResumeRoute" in panel
    assert "task_history.jsonl" in panel
    assert "exportTaskHistory" in panel
    assert '"/antbot/dock_request"' in navigator
    assert "get_charger_name" in navigator
    assert '"/waterplus/route_control"' in route_runner
    assert '"paused"' in route_runner
    assert '"resume"' in route_runner


def test_legacy_step4_is_only_a_step1_compatibility_alias():
    script = read("step4_3d_nav_baseline.sh")
    assert 'exec "$SCRIPT_DIR/step1_mapping.sh" "$@"' in script
    assert "isaac_3d_navigation.launch.py" not in script


def test_3d_isaac_writer_also_publishes_projected_planar_scans():
    script = read("run_antbot_dual_lidar.py")
    assert "project_xyz_to_laserscan" in script
    assert '"/scan_0"' in script
    assert '"/scan_1"' in script
