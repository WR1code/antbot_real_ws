"""Stage 1 only: existing Livox driver and lossless per-device cloud frame relay.

This launch never changes network settings and never starts SLAM, Nav2, RViz,
fusion, or an additional robot_state_publisher. Keep the existing chassis
description publisher as the sole owner of LiDAR extrinsic TF.
"""

import json
import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _network_preflight(interface, host_ip):
    """Require a secondary sensor address without disturbing the base profile."""
    address = json.loads(subprocess.check_output(
        ["ip", "-j", "addr", "show", "dev", interface], text=True
    ))
    ipv4 = {
        entry["local"]
        for link in address
        for entry in link.get("addr_info", [])
        if entry.get("family") == "inet"
    }
    if host_ip not in ipv4 or "10.42.0.1" not in ipv4:
        raise RuntimeError(
            f"{interface} must have BOTH 10.42.0.1 and {host_ip} before "
            "stage 1. This launch does not alter NetworkManager or addresses."
        )
    routes = json.loads(subprocess.check_output(
        ["ip", "-j", "route", "show", "default"], text=True
    ))
    if not routes or any(route.get("dev") == interface for route in routes):
        raise RuntimeError(
            f"Default route is missing or uses {interface}; stage 1 requires "
            "the existing Wi-Fi default route."
        )


def _start(context):
    config_path = LaunchConfiguration("config").perform(context)
    interface = LaunchConfiguration("lidar_interface").perform(context)
    front_ip = LaunchConfiguration("front_ip").perform(context)
    rear_ip = LaunchConfiguration("rear_ip").perform(context)
    if front_ip == rear_ip:
        raise RuntimeError("front_ip and rear_ip must be distinct")
    with open(config_path, encoding="utf-8") as stream:
        config = json.load(stream)
    host_ips = config["Mid360s"]["host_net_info"]
    devices = {item["ip"] for item in config["lidar_configs"]}
    if len(host_ips) != 1 or devices != {front_ip, rear_ip}:
        raise RuntimeError(
            "front_ip/rear_ip must match the two devices in the supplied "
            "Livox runtime JSON; the mapping is test-only"
        )
    host_ip = host_ips[0]["host_ip"]
    _network_preflight(interface, host_ip)

    return [
        Node(
            package="livox_ros_driver2",
            executable="livox_ros_driver2_node",
            name="antbot_mid360_stage1_driver",
            output="screen",
            parameters=[{
                "xfer_format": 0,
                "multi_topic": 1,
                "data_src": 0,
                "publish_freq": 10.0,
                "output_data_type": 0,
                "frame_id": "livox_frame",
                "user_config_path": config_path,
            }],
        ),
        Node(
            package="antbot_dual_lidar",
            executable="livox_imu_relay",
            name="antbot_mid360_stage1_imu_relay",
            output="screen",
            condition=IfCondition(LaunchConfiguration("normalize_imu")),
            parameters=[{
                "front_left.input_topic": "/livox/imu_" + front_ip.replace(".", "_"),
                "rear_right.input_topic": "/livox/imu_" + rear_ip.replace(".", "_"),
                "front_left.output_topic": "/antbot/lidar/front_left/imu_raw_si",
                "rear_right.output_topic": "/antbot/lidar/rear_right/imu_raw_si",
                "front_left.frame_id": "mid360_front_imu",
                "rear_right.frame_id": "mid360_rear_imu",
                "acceleration_scale": 9.80665,
            }],
        ),
        Node(
            package="antbot_dual_lidar",
            executable="livox_frame_relay",
            name="antbot_mid360_stage1_frame_relay",
            output="screen",
            parameters=[{
                "front_left.input_topic": "/livox/lidar_" + front_ip.replace(".", "_"),
                "rear_right.input_topic": "/livox/lidar_" + rear_ip.replace(".", "_"),
                "front_left.output_topic": "/antbot/lidar/front_left/points_raw_native",
                "rear_right.output_topic": "/antbot/lidar/rear_right/points_raw_native",
                "front_left.frame_id": "lidar_2d_front_scan",
                "rear_right.frame_id": "lidar_2d_back_scan",
            }],
        ),
    ]


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("antbot_mapping"),
        "config",
        "dual_mid360_actual.json",
    )
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config),
        DeclareLaunchArgument("lidar_interface", default_value="eno1"),
        DeclareLaunchArgument("normalize_imu", default_value="true"),
        DeclareLaunchArgument(
            "front_ip", description="Test-only IP mapped to front_left"
        ),
        DeclareLaunchArgument(
            "rear_ip", description="Test-only IP mapped to rear_right"
        ),
        OpaqueFunction(function=_start),
    ])
