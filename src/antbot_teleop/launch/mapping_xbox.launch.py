"""Start the measured Generic Xbox pad and ANTBot mapping controller."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Return the Xbox mapping launch description."""
    start_joy = LaunchConfiguration('start_joy')
    joy_device = LaunchConfiguration('joy_device')
    map_prefix = LaunchConfiguration('map_prefix')
    use_sim_time = LaunchConfiguration('use_sim_time')
    max_linear_speed = LaunchConfiguration('max_linear_speed')
    max_angular_speed = LaunchConfiguration('max_angular_speed')

    joy_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='antbot_xbox_joy_node',
        output='screen',
        condition=IfCondition(start_joy),
        parameters=[{
            'dev': joy_device,
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
            'sticky_buttons': False,
        }],
    )
    xbox_node = Node(
        package='antbot_teleop',
        executable='mapping_xbox',
        name='antbot_mapping_xbox',
        output='screen',
        parameters=[{
            'map_prefix': map_prefix,
            'map_use_sim_time': use_sim_time,
            'use_sim_time': use_sim_time,
            'max_linear_vel': max_linear_speed,
            'max_angular_vel': max_angular_speed,
        }],
    )
    exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=xbox_node,
            on_exit=[
                EmitEvent(event=Shutdown(
                    reason='ANTBot Xbox mapping controller exited'))
            ],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('start_joy', default_value='true'),
        DeclareLaunchArgument(
            'joy_device', default_value='/dev/input/js0'),
        DeclareLaunchArgument('map_prefix', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('max_linear_speed', default_value='1.50'),
        DeclareLaunchArgument('max_angular_speed', default_value='1.00'),
        joy_node,
        xbox_node,
        exit_handler,
    ])
