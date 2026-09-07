"""Launch the real-robot-first Phase 4B capture node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    default_config = str(
        Path(get_package_share_directory("antbot_rgbd_dataset"))
        / "config" / "rgbd_real_robot.yaml"
    )
    default_rviz_config = str(
        Path(get_package_share_directory("antbot_rgbd_dataset"))
        / "rviz" / "rgbd_live_scan.rviz"
    )
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
        Node(
            package="antbot_rgbd_dataset",
            executable="phase4b_capture_node",
            name="phase4b_capture_node",
            output="screen",
            parameters=[default_config, LaunchConfiguration("config")],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rgbd_scan_rviz",
            output="screen",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
    ])
