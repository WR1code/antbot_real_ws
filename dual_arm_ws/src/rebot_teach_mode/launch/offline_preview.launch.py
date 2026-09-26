"""Compatibility alias for the single comprehensive offline workbench."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    comprehensive_launch = os.path.join(
        get_package_share_directory("rebot_xbox_hardware"),
        "launch",
        "dual_arm_offline_preview.launch.py",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model",
                default_value="dm",
                choices=["dm", "rs"],
                description="reBotArm model used by the comprehensive workbench",
            ),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            LogInfo(
                msg=(
                    "offline_preview.launch.py now opens the comprehensive "
                    "reBotArm/Piper-H offline workbench."
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(comprehensive_launch),
                launch_arguments={
                    "rebot_model": LaunchConfiguration("model"),
                    "use_rviz": LaunchConfiguration("use_rviz"),
                }.items(),
            ),
        ]
    )
