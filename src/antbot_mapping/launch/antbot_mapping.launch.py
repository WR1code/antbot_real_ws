"""Preflight-gated single-MID360S FAST-LIO baseline.

This launch does not alter NetworkManager. It starts FAST-LIO only after the
sensor/time/TF/dependency preflight exits successfully.
"""

import json
import os
import subprocess
from pathlib import Path

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _network_preflight(interface, host_ip):
    address = json.loads(subprocess.check_output(
        ["ip", "-j", "addr", "show", "dev", interface], text=True
    ))
    ipv4 = {
        entry["local"] for link in address for entry in link.get("addr_info", [])
        if entry.get("family") == "inet"
    }
    if not {"10.42.0.1", host_ip}.issubset(ipv4):
        raise RuntimeError(
            f"{interface} must retain 10.42.0.1 and also have {host_ip}; "
            "this launch never changes network configuration"
        )
    routes = json.loads(subprocess.check_output(
        ["ip", "-j", "route", "show", "default"], text=True
    ))
    if not routes or any(route.get("dev") == interface for route in routes):
        raise RuntimeError("the default route must remain on Wi-Fi")


def _start(context):
    package_share = get_package_share_directory("antbot_mapping")
    config_file = LaunchConfiguration("lio_config").perform(context)
    driver_config = LaunchConfiguration("driver_config").perform(context)
    interface = LaunchConfiguration("lidar_interface").perform(context)
    front_ip = LaunchConfiguration("front_ip").perform(context)
    rear_ip = LaunchConfiguration("rear_ip").perform(context)
    if front_ip == rear_ip:
        raise RuntimeError("front_ip and rear_ip must differ")
    with open(driver_config, encoding="utf-8") as stream:
        driver_data = json.load(stream)
    devices = {entry["ip"] for entry in driver_data["lidar_configs"]}
    host_entries = driver_data["Mid360s"]["host_net_info"]
    if devices != {front_ip, rear_ip} or len(host_entries) != 1:
        raise RuntimeError("launch IP mapping must match the two-device driver JSON")
    _network_preflight(interface, host_entries[0]["host_ip"])
    if not os.path.isfile(config_file):
        raise RuntimeError(f"FAST-LIO config does not exist: {config_file}")

    driver = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="antbot_mapping_mid360_driver",
        output="screen",
        parameters=[{
            "xfer_format": 1,
            "multi_topic": 1,
            "data_src": 0,
            "publish_freq": 10.0,
            "output_data_type": 0,
            "frame_id": "livox_frame",
            "user_config_path": driver_config,
        }],
    )
    imu_relay = Node(
        package="antbot_dual_lidar",
        executable="livox_imu_relay",
        name="antbot_mapping_imu_relay",
        output="screen",
        parameters=[{
            "front_left.input_topic": "/livox/imu_" + front_ip.replace(".", "_"),
            "rear_right.input_topic": "/livox/imu_" + rear_ip.replace(".", "_"),
            "front_left.frame_id": "mid360_front_imu",
            "rear_right.frame_id": "mid360_rear_imu",
        }],
    )
    imu_transforms = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"antbot_{name}_lidar_to_imu_tf",
            arguments=[
                "--x", "0.011", "--y", "0.02329", "--z", "-0.04412",
                "--roll", "0", "--pitch", "0", "--yaw", "0",
                "--frame-id", lidar_frame, "--child-frame-id", imu_frame,
            ],
        )
        for name, lidar_frame, imu_frame in (
            ("front", "lidar_2d_front_scan", "mid360_front_imu"),
            ("rear", "lidar_2d_back_scan", "mid360_rear_imu"),
        )
    ]
    fast_lio = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        name="antbot_front_mid360_fast_lio",
        output="screen",
        parameters=[config_file, {
            "map_file_path": LaunchConfiguration("map_file_path"),
            "pcd_save.pcd_save_en": ParameterValue(
                LaunchConfiguration("pcd_save_en"), value_type=bool
            ),
        }],
    )
    odom_adapter = Node(
        package="antbot_mapping",
        executable="lio_odom_adapter",
        output="screen",
        parameters=[{
            "base_to_imu_translation": [0.333, 0.24529, 0.36988],
        }],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        output="screen",
    )
    preflight = Node(
        package="antbot_mapping",
        executable="mapping_preflight",
        name="antbot_mapping_preflight",
        output="screen",
        parameters=[{
            "front_ip": front_ip,
            "rear_ip": rear_ip,
            "config_file": config_file,
        }],
    )

    def _after_preflight(event, _context):
        if event.returncode == 0:
            return [
                LogInfo(msg="Preflight passed; starting FAST-LIO baseline"),
                fast_lio,
                odom_adapter,
                rviz,
            ]
        raise RuntimeError(
            f"mapping preflight failed with code {event.returncode}; mapping denied"
        )

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("antbot_description"),
            "launch", "description.launch.py"
        )),
        condition=IfCondition(LaunchConfiguration("start_robot_description")),
        launch_arguments={
            "use_rviz": "false",
            "use_joint_state_publisher": "false",
            "use_joint_state_publisher_gui": "false",
        }.items(),
    )
    return [
        description,
        driver,
        imu_relay,
        *imu_transforms,
        TimerAction(period=5.0, actions=[preflight]),
        RegisterEventHandler(OnProcessExit(
            target_action=preflight, on_exit=_after_preflight
        )),
    ]


def generate_launch_description():
    package_share = get_package_share_directory("antbot_mapping")
    workspace_root = Path(get_package_prefix("antbot_mapping")).parents[1]
    return LaunchDescription([
        DeclareLaunchArgument("front_ip"),
        DeclareLaunchArgument("rear_ip"),
        DeclareLaunchArgument("lidar_interface", default_value="eno1"),
        DeclareLaunchArgument(
            "driver_config",
            default_value=os.path.join(
                package_share, "config", "dual_mid360_actual.json"
            ),
        ),
        DeclareLaunchArgument(
            "lio_config",
            default_value=os.path.join(package_share, "config", "front_mid360_lio.yaml"),
        ),
        DeclareLaunchArgument(
            "map_file_path",
            default_value=str(
                workspace_root / "artifacts" / "maps" /
                "mapping_runs" / "manual" / "map.pcd"
            ),
        ),
        DeclareLaunchArgument("pcd_save_en", default_value="false"),
        DeclareLaunchArgument("start_robot_description", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        OpaqueFunction(function=_start),
    ])
