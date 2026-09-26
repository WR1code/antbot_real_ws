"""Open the desktop selector that owns one real-arm launch at a time."""

from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    selector = Node(
        package="rebot_xbox_hardware",
        executable="arm_selector",
        name="real_arm_selector",
        output="screen",
    )
    return LaunchDescription(
        [
            selector,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=selector,
                    on_exit=[
                        EmitEvent(
                            event=Shutdown(reason="real-arm selector closed")
                        )
                    ],
                )
            ),
        ]
    )
