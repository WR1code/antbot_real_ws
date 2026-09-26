from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("antbot_mid360_fusion"))
    config = LaunchConfiguration("config")
    stage = LaunchConfiguration("stage")
    front_topic = LaunchConfiguration("front_topic")
    rear_topic = LaunchConfiguration("rear_topic")
    target_frame = LaunchConfiguration("target_frame")
    metrics_csv = LaunchConfiguration("metrics_csv")
    use_rviz = LaunchConfiguration("use_rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config", default_value=str(share / "config" / "dual_mid360_fusion.yaml")
            ),
            DeclareLaunchArgument("stage", default_value="merge"),
            DeclareLaunchArgument(
                "front_topic", default_value="/antbot/lidar/front_left/points_raw_native"
            ),
            DeclareLaunchArgument(
                "rear_topic", default_value="/antbot/lidar/rear_right/points_raw_native"
            ),
            DeclareLaunchArgument("target_frame", default_value="base_link"),
            DeclareLaunchArgument("metrics_csv", default_value=""),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            Node(
                package="antbot_mid360_fusion",
                executable="dual_mid360_fusion_node",
                name="dual_mid360_fusion",
                output="screen",
                parameters=[
                    config,
                    {
                        "processing_stage": stage,
                        "front.input_topic": front_topic,
                        "rear.input_topic": rear_topic,
                        "target_frame": target_frame,
                        "metrics_csv_path": metrics_csv,
                    },
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="dual_mid360_fusion_rviz",
                arguments=["-d", str(share / "rviz" / "dual_mid360_fusion.rviz")],
                condition=IfCondition(use_rviz),
                output="screen",
            ),
        ]
    )
