from pathlib import Path
import ast
import json


SRC_ROOT = Path(__file__).resolve().parents[2]
BRINGUP = SRC_ROOT / "antbot" / "antbot_bringup"
DUAL_LIDAR = SRC_ROOT / "antbot_dual_lidar"


def test_mid360s_config_has_two_unique_devices():
    config = json.loads(
        (BRINGUP / "config" / "mid360s_dual.json").read_text(encoding="utf-8")
    )
    assert config["lidar_summary_info"]["lidar_type"] == 8
    assert "Mid360s" in config
    assert len(config["Mid360s"]["host_net_info"]) == 1
    devices = config["lidar_configs"]
    assert len(devices) == 2
    assert len({device["ip"] for device in devices}) == 2
    assert all(device["pcl_data_type"] == 1 for device in devices)


def test_mid360s_launch_uses_multi_topic_and_lossless_frame_relay():
    launch_path = BRINGUP / "launch" / "lidar_3d.launch.py"
    launch_text = launch_path.read_text(encoding="utf-8")
    ast.parse(launch_text)
    assert "'multi_topic': 1" in launch_text
    assert "'xfer_format': 0" in launch_text
    assert "executable='livox_frame_relay'" in launch_text
    assert "/antbot/lidar/front_left/points_raw_native" in launch_text
    assert "/antbot/lidar/rear_right/points_raw_native" in launch_text

    relay_text = (
        DUAL_LIDAR / "antbot_dual_lidar" / "livox_frame_relay.py"
    ).read_text(encoding="utf-8")
    ast.parse(relay_text)
    assert "message.header.frame_id = stream['frame_id']" in relay_text
    assert "stream['publisher'].publish(message)" in relay_text


def test_full_bringup_selects_mid360s_by_default():
    launch_text = (
        BRINGUP / "launch" / "bringup.launch.py"
    ).read_text(encoding="utf-8")
    ast.parse(launch_text)
    assert "'start_legacy_2d_lidars', default_value='false'" in launch_text
    assert "'start_mid360s_lidars', default_value='true'" in launch_text
