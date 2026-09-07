"""Bring up preprocessing, diagnostics, RViz, and optional MCAP recording."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("antbot_dual_lidar")
    defaults = os.path.join(share, "config", "dual_lidar.yaml")
    default_rviz = os.path.join(share, "config", "rviz", "dual_lidar.rviz")
    record_script = os.path.join(share, "scripts", "record_dual_lidar_bag.sh")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    start_preprocessing = LaunchConfiguration("start_preprocessing")
    start_diagnostics = LaunchConfiguration("start_diagnostics")
    start_synchronizer = LaunchConfiguration("start_synchronizer")
    start_deskew_validator = LaunchConfiguration("start_deskew_validator")
    point_time_convention = LaunchConfiguration("point_time_convention")
    publish_synchronized = LaunchConfiguration("publish_synchronized")
    sync_strategy = LaunchConfiguration("sync_strategy")
    lidar_profile = LaunchConfiguration("lidar_profile")
    front_topic = LaunchConfiguration("front_lidar_topic")
    rear_topic = LaunchConfiguration("rear_lidar_topic")
    target_frame = LaunchConfiguration("target_frame")
    record_bag = LaunchConfiguration("record_bag")
    bag_root = LaunchConfiguration("bag_root")
    rviz_config = LaunchConfiguration("rviz_config")
    config_file = LaunchConfiguration("config_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("start_preprocessing", default_value="true"),
            DeclareLaunchArgument("start_diagnostics", default_value="true"),
            DeclareLaunchArgument("start_synchronizer", default_value="true"),
            DeclareLaunchArgument("start_deskew_validator", default_value="false"),
            DeclareLaunchArgument(
                "point_time_convention", default_value="header_plus_offset"
            ),
            DeclareLaunchArgument("publish_synchronized", default_value="false"),
            DeclareLaunchArgument("sync_strategy", default_value="approximate"),
            DeclareLaunchArgument("lidar_profile", default_value="mapping"),
            DeclareLaunchArgument(
                "front_lidar_topic",
                default_value="/antbot/lidar/front_left/points_raw_native",
            ),
            DeclareLaunchArgument(
                "rear_lidar_topic",
                default_value="/antbot/lidar/rear_right/points_raw_native",
            ),
            DeclareLaunchArgument("target_frame", default_value="base_link"),
            DeclareLaunchArgument("record_bag", default_value="false"),
            DeclareLaunchArgument("bag_root", default_value="bags"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz),
            DeclareLaunchArgument("config_file", default_value=defaults),
            Node(
                package="antbot_dual_lidar",
                executable="cloud_preprocessor",
                name="antbot_dual_lidar_preprocessor",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "use_sim_time": use_sim_time,
                        "lidar_profile": lidar_profile,
                        "target_frame": target_frame,
                        "front_left.input_topic": front_topic,
                        "rear_right.input_topic": rear_topic,
                    },
                ],
                condition=IfCondition(start_preprocessing),
            ),
            Node(
                package="antbot_dual_lidar",
                executable="dual_lidar_diagnostics",
                name="antbot_dual_lidar_diagnostics",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "use_sim_time": use_sim_time,
                        "front_topic": front_topic,
                        "rear_topic": rear_topic,
                    },
                ],
                condition=IfCondition(start_diagnostics),
            ),
            Node(
                package="antbot_dual_lidar",
                executable="dual_cloud_synchronizer",
                name="antbot_dual_lidar_synchronizer",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "use_sim_time": use_sim_time,
                        "front_topic": front_topic,
                        "rear_topic": rear_topic,
                        "strategy": sync_strategy,
                        "publish_synchronized": publish_synchronized,
                    },
                ],
                condition=IfCondition(start_synchronizer),
            ),
            Node(
                package="antbot_dual_lidar",
                executable="ground_truth_deskew_validator",
                name="ground_truth_deskew_validator",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "point_time_convention": point_time_convention,
                    }
                ],
                condition=IfCondition(start_deskew_validator),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="dual_lidar_rviz",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
                condition=IfCondition(use_rviz),
            ),
            ExecuteProcess(
                cmd=[record_script, bag_root],
                output="screen",
                condition=IfCondition(record_bag),
            ),
        ]
    )
