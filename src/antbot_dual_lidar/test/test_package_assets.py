from pathlib import Path
import ast

import yaml


ROOT = Path(__file__).resolve().parents[1]


def test_yaml_is_parseable_and_has_both_streams():
    parsed = yaml.safe_load((ROOT / "config" / "dual_lidar.yaml").read_text())
    params = parsed["antbot_dual_lidar_preprocessor"]["ros__parameters"]
    assert params["front_left"]["input_topic"] != params["rear_right"]["input_topic"]
    assert params["front_left"]["output_topic"] != params["rear_right"]["output_topic"]


def test_launch_files_parse_and_required_arguments_exist():
    for name in (
        "dual_lidar_bringup.launch.py",
        "dual_lidar_replay.launch.py",
        "isaac_3d_navigation.launch.py",
    ):
        ast.parse((ROOT / "launch" / name).read_text())
    text = (ROOT / "launch" / "dual_lidar_bringup.launch.py").read_text()
    for argument in (
        "use_sim_time", "use_rviz", "start_preprocessing", "start_diagnostics",
        "front_lidar_topic", "rear_lidar_topic", "target_frame", "record_bag",
        "rviz_config", "lidar_profile", "start_synchronizer",
        "publish_synchronized", "sync_strategy",
        "start_deskew_validator", "point_time_convention",
    ):
        assert f'"{argument}"' in text


def test_navigation_uses_live_clouds_and_rear_local_costmap():
    launch_text = (ROOT / "launch" / "isaac_3d_navigation.launch.py").read_text()
    nav_params = yaml.safe_load(
        (ROOT / "config" / "isaac_3d_nav2_params.yaml").read_text()
    )
    local_params = nav_params["local_costmap"]["local_costmap"]["ros__parameters"]
    rear_params = local_params["dual_3d_obstacles"]["rear_3d"]

    for forbidden in ("points_deskew_truth", "world_reference", "ground_truth"):
        assert forbidden not in launch_text
    assert "rear_3d" in local_params["dual_3d_obstacles"]["observation_sources"]
    assert rear_params["topic"] == "/antbot/lidar/rear_right/points_filtered"
    assert rear_params["data_type"] == "PointCloud2"
    assert "'use_amcl'" in launch_text
    assert "'use_amcl', default_value='true'" in launch_text


def test_recording_topic_contract():
    text = (ROOT / "scripts" / "record_dual_lidar_bag.sh").read_text()
    for topic in (
        "/clock", "/antbot/lidar/front_left/points_raw_native",
        "/antbot/lidar/rear_right/points_raw_native",
        "/antbot/lidar/front_left/points_deskew_truth",
        "/antbot/lidar/rear_right/points_deskew_truth",
        "/antbot/imu/data_raw", "/antbot/imu/data",
        "/antbot/ground_truth/odom",
        "/tf", "/tf_static", "/odom", "/joint_states", "/cmd_vel",
    ):
        assert topic in text
    assert "--storage mcap" in text


def test_phase_2a_contract_and_reports():
    contract = yaml.safe_load((ROOT / "config" / "lio_input_contract.yaml").read_text())
    assert contract["front_lidar"]["topic"] != contract["rear_lidar"]["topic"]
    assert contract["point_fields"]["per_point_time"] == "sensor_native_always_available_in_3d_mode"
    assert contract["imu"]["available"] is True
    assert contract["imu"]["raw_topic"] == "/antbot/imu/data_raw"
    assert contract["imu"]["topic"] == "/antbot/imu/data"
    assert contract["imu"]["primary_for_phase_2b"] is True
    assert contract["auxiliary_imu"]["fused_with_primary"] is False
    assert contract["point_time"]["datatype"] == "int32"
    for name in (
        "PERFORMANCE_BASELINE.md", "QOS_DESIGN.md",
        "LIO_INPUT_COMPATIBILITY.md", "STATE_INPUT_REPORT.md",
        "GUI_MOTION_TEST_REPORT.md", "OCCLUSION_TEST_REPORT.md",
        "BLIND_SPOT_REPORT.md", "STABILITY_TEST_REPORT.md",
        "LIO_INPUT_CONTRACT.md",
        "IMU_VALIDATION_REPORT.md", "IMU_GROUND_TRUTH_COMPARISON.md",
        "POINT_TIME_SEMANTICS_REPORT.md", "DATASET_VALIDATION_REPORT.md",
        "LIO_IMPLEMENTATION_DECISION.md",
        "GMO_POINT_SEMANTICS_AUDIT.md",
    ):
        assert (ROOT / name).is_file()
