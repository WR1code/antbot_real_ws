"""Start the Isaac mapping stack with startup-tolerant lifecycle settings."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    navigation_dir = get_package_share_directory("antbot_navigation")
    lidar_dir = get_package_share_directory("antbot_dual_lidar")
    params_file = os.path.join(
        navigation_dir, "config", "sim", "slam_toolbox_params.yaml"
    )
    rviz_default = os.path.join(navigation_dir, "rviz", "mapping_3d.rviz")
    lidar_params = os.path.join(lidar_dir, "config", "dual_lidar.yaml")
    enable_3d_mapping = LaunchConfiguration("enable_3d_mapping")

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("enable_3d_mapping", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=rviz_default),
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[
                params_file,
                {"use_sim_time": True, "scan_topic": "/scan_0"},
            ],
        ),
        TimerAction(
            period=1.0,
            actions=[
                Node(
                    package="nav2_lifecycle_manager",
                    executable="lifecycle_manager",
                    name="lifecycle_manager_slam",
                    output="screen",
                    parameters=[{
                        "autostart": True,
                        # Isaac's first RTX frames briefly saturate the host;
                        # four seconds produced false bond failures in logs.
                        "bond_timeout": 15.0,
                        "node_names": ["slam_toolbox"],
                        "use_sim_time": True,
                    }],
                ),
            ],
        ),
        Node(
            package="antbot_dual_lidar",
            executable="cloud_preprocessor",
            name="antbot_dual_lidar_preprocessor",
            output="screen",
            condition=IfCondition(enable_3d_mapping),
            parameters=[
                lidar_params,
                {
                    "use_sim_time": True,
                    "lidar_profile": "mapping",
                    "target_frame": "base_link",
                    "front_left.input_topic": (
                        "/antbot/lidar/front_left/points_raw_native"
                    ),
                    "rear_right.input_topic": (
                        "/antbot/lidar/rear_right/points_raw_native"
                    ),
                },
            ],
        ),
        Node(
            package="antbot_dual_lidar",
            executable="fixed_frame_mapper",
            name="antbot_fixed_frame_mapper",
            output="screen",
            condition=IfCondition(enable_3d_mapping),
            parameters=[{"use_sim_time": True, "fixed_frame": "odom"}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_mapping",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            arguments=["-d", LaunchConfiguration("rviz_config")],
            parameters=[{"use_sim_time": True}],
        ),
    ])
