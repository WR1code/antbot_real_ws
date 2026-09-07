"""Replay a recorded MCAP bag through the same preprocessing/visualization path."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("antbot_dual_lidar")
    config = os.path.join(share, "config", "dual_lidar.yaml")
    default_rviz = os.path.join(share, "config", "rviz", "dual_lidar.rviz")
    bag_path = LaunchConfiguration("bag_path")
    use_rviz = LaunchConfiguration("use_rviz")
    start_preprocessing = LaunchConfiguration("start_preprocessing")
    start_diagnostics = LaunchConfiguration("start_diagnostics")
    rviz_config = LaunchConfiguration("rviz_config")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "bag_path", description="Path to an existing rosbag2 directory"
            ),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            # Recorded bags contain both raw and filtered topics. Keep this off
            # by default to avoid duplicate publishers; enable it for raw-only bags.
            DeclareLaunchArgument("start_preprocessing", default_value="false"),
            DeclareLaunchArgument("start_diagnostics", default_value="true"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz),
            Node(
                package="antbot_dual_lidar",
                executable="cloud_preprocessor",
                name="antbot_dual_lidar_preprocessor",
                parameters=[config, {"use_sim_time": True}],
                output="screen",
                condition=IfCondition(start_preprocessing),
            ),
            Node(
                package="antbot_dual_lidar",
                executable="dual_lidar_diagnostics",
                name="antbot_dual_lidar_diagnostics",
                parameters=[config, {"use_sim_time": True}],
                output="screen",
                condition=IfCondition(start_diagnostics),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="dual_lidar_rviz",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": True}],
                output="screen",
                condition=IfCondition(use_rviz),
            ),
            ExecuteProcess(
                # The standard bag already records Isaac's /clock payload.
                # Do not use rosbag2 --clock here: it would synthesize clock
                # from wall-time receipt stamps and disagree with cloud headers.
                cmd=["ros2", "bag", "play", bag_path, "--delay", "1.0"],
                output="screen",
            ),
        ]
    )
