"""Run real-robot SLAM without opening a second RViz window."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    share = get_package_share_directory("antbot_real_bringup")
    params = os.path.join(share, "config", "slam_toolbox_operator.yaml")
    scan_topic = LaunchConfiguration("scan_topic")
    map_topic = LaunchConfiguration("map_topic")

    slam = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="antbot_operator_slam",
        namespace="",
        output="screen",
        parameters=[params, {
            "use_sim_time": False,
            "use_lifecycle_manager": False,
            "scan_topic": scan_topic,
        }],
        remappings=[("/map", map_topic), ("map", map_topic)],
    )
    configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(slam),
        transition_id=Transition.TRANSITION_CONFIGURE,
    ))
    activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[AntBot] SLAM Toolbox activating"),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )
    return LaunchDescription([
        DeclareLaunchArgument("scan_topic", default_value="/scan_0"),
        DeclareLaunchArgument(
            "map_topic", default_value="/antbot/mapping/map"
        ),
        slam,
        configure,
        activate,
    ])
