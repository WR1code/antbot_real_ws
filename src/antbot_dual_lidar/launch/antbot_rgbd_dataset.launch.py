"""Launch the independent Phase 2 RGB-D keyframe recorder.

This launch intentionally does not create or publish an RGB-D camera.  It consumes the
single Phase 1A publisher already owned by the Isaac process.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config = (
        get_package_share_directory("antbot_rgbd_dataset")
        + "/config/rgbd_dataset.yaml"
    )
    arguments = [
        DeclareLaunchArgument("config_file", default_value=config),
        DeclareLaunchArgument("dataset_name", default_value="home_rgbd_01"),
        DeclareLaunchArgument(
            "output_root",
            default_value=os.path.expanduser("~/antbot_data/rgbd_datasets"),
        ),
        DeclareLaunchArgument("fixed_frame", default_value="odom"),
        DeclareLaunchArgument(
            "camera_frame", default_value="camera_color_optical_frame"
        ),
        DeclareLaunchArgument(
            "rgb_topic", default_value="/antbot/camera/color/image_raw"
        ),
        DeclareLaunchArgument(
            "depth_topic", default_value="/antbot/camera/depth/image_raw"
        ),
        DeclareLaunchArgument(
            "color_info_topic",
            default_value="/antbot/camera/color/camera_info",
        ),
        DeclareLaunchArgument(
            "depth_info_topic",
            default_value="/antbot/camera/depth/camera_info",
        ),
        DeclareLaunchArgument("auto_start", default_value="true"),
        DeclareLaunchArgument("overwrite_existing", default_value="false"),
    ]
    recorder = Node(
        package="antbot_rgbd_dataset",
        executable="rgbd_keyframe_recorder",
        name="rgbd_keyframe_recorder",
        output="screen",
        parameters=[
            LaunchConfiguration("config_file"),
            {
                "dataset_name": LaunchConfiguration("dataset_name"),
                "output_root": LaunchConfiguration("output_root"),
                "fixed_frame": LaunchConfiguration("fixed_frame"),
                "camera_frame": LaunchConfiguration("camera_frame"),
                "rgb_topic": LaunchConfiguration("rgb_topic"),
                "depth_topic": LaunchConfiguration("depth_topic"),
                "color_info_topic": LaunchConfiguration("color_info_topic"),
                "depth_info_topic": LaunchConfiguration("depth_info_topic"),
                "auto_start": LaunchConfiguration("auto_start"),
                "overwrite_existing": LaunchConfiguration("overwrite_existing"),
            },
        ],
    )
    return LaunchDescription(arguments + [recorder])
