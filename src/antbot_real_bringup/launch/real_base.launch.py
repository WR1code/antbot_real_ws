"""Start the real AntBot model and H743 UART bridge without simulation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.actions import EmitEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


DEFAULT_UART_PORT = (
    "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5CE6063665-if00"
)


def generate_launch_description():
    port = LaunchConfiguration("port")
    start_description = LaunchConfiguration("start_description")
    start_xbox = LaunchConfiguration("start_xbox")
    start_joy = LaunchConfiguration("start_joy")
    joy_device = LaunchConfiguration("joy_device")
    max_linear_speed = LaunchConfiguration("max_linear_speed")
    telemetry_period = LaunchConfiguration("telemetry_period")

    description_launch = os.path.join(
        get_package_share_directory("antbot_description"),
        "launch",
        "description.launch.py",
    )
    model = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(description_launch),
        launch_arguments={
            "use_sim_time": "false",
            "use_joint_state_publisher": "false",
            "use_joint_state_publisher_gui": "false",
            "use_rviz": "false",
        }.items(),
        condition=IfCondition(start_description),
    )

    bridge = Node(
        package="antbot_h743_bridge",
        executable="h743_cmd_vel_bridge",
        name="h743_cmd_vel_bridge",
        output="screen",
        parameters=[{
            "port": port,
            "baud": 115200,
            "topic": "/cmd_vel",
            "status_topic": "/rs00/motor_status",
            "max_linear_speed": ParameterValue(
                max_linear_speed, value_type=float
            ),
            "telemetry_period": ParameterValue(
                telemetry_period, value_type=float
            ),
        }],
    )

    joy_condition = IfCondition(PythonExpression([
        "'", start_xbox, "'.lower() == 'true' and '",
        start_joy, "'.lower() == 'true'",
    ]))
    joy = Node(
        package="joy_linux",
        executable="joy_linux_node",
        name="antbot_real_joy",
        output="screen",
        condition=joy_condition,
        parameters=[{
            "dev": joy_device,
            "deadzone": 0.05,
            "autorepeat_rate": 20.0,
            "sticky_buttons": False,
        }],
    )
    xbox = Node(
        package="antbot_teleop",
        executable="mapping_xbox",
        name="antbot_real_xbox",
        output="screen",
        condition=IfCondition(start_xbox),
        parameters=[{
            "map_prefix": "/tmp/antbot_real_unused_map",
            "map_use_sim_time": False,
            "use_sim_time": False,
            "max_linear_vel": ParameterValue(
                max_linear_speed, value_type=float
            ),
            "max_angular_vel": 0.0,
            "topics.cmd_vel": "/cmd_vel",
        }],
    )

    shutdown_if_bridge_exits = RegisterEventHandler(
        OnProcessExit(
            target_action=bridge,
            on_exit=[EmitEvent(event=Shutdown(
                reason="H743 UART bridge exited"
            ))],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "port",
            default_value=os.environ.get("RS00_UART_PORT", DEFAULT_UART_PORT),
            description="Stable /dev/serial/by-id path for the H743 USB-TTL",
        ),
        DeclareLaunchArgument("start_description", default_value="true"),
        DeclareLaunchArgument(
            "start_xbox",
            default_value="false",
            description="Requires completed calibration and a physical E-stop",
        ),
        DeclareLaunchArgument("start_joy", default_value="true"),
        DeclareLaunchArgument("joy_device", default_value="/dev/input/js0"),
        DeclareLaunchArgument("max_linear_speed", default_value="0.10"),
        DeclareLaunchArgument("telemetry_period", default_value="0.20"),
        model,
        bridge,
        joy,
        xbox,
        shutdown_if_bridge_exits,
    ])
