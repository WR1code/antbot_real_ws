"""Run the complete Step 2 operator UI with the real H743 lower layer."""

import os

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Combine Step 2 editing tools with the reconnecting real base bridge."""
    real_dir = get_package_share_directory("antbot_real_bringup")
    navigation_dir = get_package_share_directory("antbot_navigation")
    map_yaml = LaunchConfiguration("map")
    waypoint_file = LaunchConfiguration("waypoints_file")

    real_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(real_dir, "launch", "real_base.launch.py")
        ),
        launch_arguments={
            "port": LaunchConfiguration("port"),
            "start_description": "true",
            "start_xbox": "true",
            "start_joy": LaunchConfiguration("start_joy"),
            "joy_device": LaunchConfiguration("joy_device"),
            "max_linear_speed": LaunchConfiguration("max_linear_speed"),
            "allow_disconnected": "true",
            "default_teleop_mode": LaunchConfiguration(
                "default_teleop_mode"
            ),
            "mapping_output_prefix": LaunchConfiguration(
                "mapping_output_prefix"
            ),
        }.items(),
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{"yaml_filename": map_yaml, "use_sim_time": False}],
    )
    map_lifecycle = TimerAction(
        period=1.0,
        actions=[Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_operator_map",
            output="screen",
            parameters=[{
                "autostart": True,
                "node_names": ["map_server"],
                "use_sim_time": False,
            }],
        )],
    )

    waypoint_tools = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            navigation_dir, "launch", "waypoint_navigation.launch.py"
        )),
        launch_arguments={
            "mode": "real",
            # H743 currently has no odom and rejects angular.z. Keep autonomous
            # motion disabled while preserving all map/waypoint/zone UI tools.
            "start_nav2": "false",
            "map": map_yaml,
            "waypoints_file": waypoint_file,
            "start_waypoint_saver": "false",
            "keepout_file": LaunchConfiguration("keepout_file"),
            "speed_zone_file": LaunchConfiguration("speed_zone_file"),
            "stuck_history_file": LaunchConfiguration("stuck_history_file"),
            "use_rviz": "false",
        }.items(),
    )

    offline_cloud = Node(
        package="antbot_dual_lidar",
        executable="offline_pointcloud_publisher",
        name="antbot_offline_pointcloud_publisher",
        output="screen",
        condition=IfCondition(LaunchConfiguration("show_3d_cloud")),
        parameters=[{
            "pointcloud_path": LaunchConfiguration("pointcloud_path"),
            "metadata_path": LaunchConfiguration("pointcloud_metadata"),
            "publish_topic": "/antbot/offline_map_points",
            "target_frame": "map",
            "publish_rate": 0.5,
            "use_sim_time": False,
        }],
    )
    offline_rgbd = Node(
        package="antbot_rgbd_dataset",
        executable="offline_preview_publisher",
        name="antbot_rgbd_offline_cloud_publisher",
        output="screen",
        condition=IfCondition(LaunchConfiguration("show_rgbd_cloud")),
        parameters=[{
            "preview_path": LaunchConfiguration("rgbd_preview_path"),
            "publish_topic": "/antbot/rgbd/offline_cloud",
            "publish_rate": 0.5,
            "use_sim_time": False,
        }],
    )

    # Placeholder only: the current H743 protocol has no odometry. It keeps
    # the saved map and robot model visible at the origin without pretending
    # that live localization exists.
    placeholder_pose = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="operator_map_to_base_placeholder",
        output="screen",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "map", "--child-frame-id", "base_link",
        ],
    )

    unicode_preload = os.path.join(
        get_package_prefix("robotcar_navigation"),
        "lib",
        "librobotcar_rviz_unicode.so",
    )
    inherited_preload = os.environ.get("LD_PRELOAD", "")
    rviz_preload = ":".join(
        item for item in (unicode_preload, inherited_preload) if item
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_waypoints",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_operator_rviz")),
        arguments=["-d", LaunchConfiguration("rviz_config")],
        additional_env={
            "LANG": "zh_CN.UTF-8",
            "LANGUAGE": "zh_CN:zh",
            "LC_ALL": "zh_CN.UTF-8",
            "LD_PRELOAD": rviz_preload,
        },
        parameters=[{"use_sim_time": False}],
    )
    shutdown_with_rviz = RegisterEventHandler(OnProcessExit(
        target_action=rviz,
        on_exit=[EmitEvent(event=Shutdown(
            reason="Step 2 operator RViz exited"
        ))],
    ))

    return LaunchDescription([
        DeclareLaunchArgument("port", default_value=os.environ.get(
            "RS00_UART_PORT",
            "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5CE6063665-if00",
        )),
        DeclareLaunchArgument("start_joy", default_value="false"),
        DeclareLaunchArgument("joy_device", default_value="/dev/input/js0"),
        DeclareLaunchArgument("max_linear_speed", default_value="0.10"),
        DeclareLaunchArgument(
            "default_teleop_mode", default_value="xbox",
            choices=["xbox", "keyboard"],
        ),
        DeclareLaunchArgument(
            "mapping_output_prefix",
            default_value="/tmp/antbot_mapping/map",
        ),
        DeclareLaunchArgument("map"),
        DeclareLaunchArgument("waypoints_file"),
        DeclareLaunchArgument("keepout_file", default_value=""),
        DeclareLaunchArgument("speed_zone_file", default_value=""),
        DeclareLaunchArgument("stuck_history_file", default_value=""),
        DeclareLaunchArgument("pointcloud_path", default_value=""),
        DeclareLaunchArgument("pointcloud_metadata", default_value=""),
        DeclareLaunchArgument("rgbd_preview_path", default_value=""),
        DeclareLaunchArgument("show_3d_cloud", default_value="false"),
        DeclareLaunchArgument("show_rgbd_cloud", default_value="false"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(
                navigation_dir, "rviz", "waypoint_navigation.rviz"
            ),
        ),
        DeclareLaunchArgument("use_operator_rviz", default_value="true"),
        real_base,
        map_server,
        map_lifecycle,
        waypoint_tools,
        offline_cloud,
        offline_rgbd,
        placeholder_pose,
        rviz,
        shutdown_with_rviz,
    ])
