"""Start Piper-H MoveIt, Isaac UDP bridge and optional guarded Xbox input."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    control_share = get_package_share_directory("piperh_control")
    moveit_share = get_package_share_directory("piperh_moveit_config")
    mapping = os.path.join(control_share, "config", "xbox_mapping.yaml")
    with open(os.path.join(control_share, "config", "servo_config.yaml"), encoding="utf-8") as stream:
        servo = {"moveit_servo": yaml.safe_load(stream)}
    moveit = (
        MoveItConfigsBuilder("piperh", package_name="piperh_moveit_config")
        .robot_description(file_path="config/piperh.urdf.xacro")
        .robot_description_semantic(file_path="config/piperh.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    use_xbox = LaunchConfiguration("use_xbox")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_xbox", default_value="false"),
            DeclareLaunchArgument("joy_device", default_value="/dev/input/js0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(moveit_share, "launch", "demo.launch.py"))
            ),
            Node(
                package="piperh_control",
                executable="joint_state_udp_bridge",
                output="screen",
            ),
            Node(
                package="moveit_servo",
                executable="servo_node",
                name="servo_node",
                output="screen",
                condition=IfCondition(use_xbox),
                parameters=[
                    servo,
                    {"update_period": 0.01, "planning_group_name": "arm"},
                    moveit.robot_description,
                    moveit.robot_description_semantic,
                    moveit.robot_description_kinematics,
                    moveit.joint_limits,
                ],
            ),
            Node(
                package="joy_linux",
                executable="joy_linux_node",
                name="piperh_joy",
                output="screen",
                condition=IfCondition(use_xbox),
                parameters=[{"dev": LaunchConfiguration("joy_device"), "deadzone": 0.05, "autorepeat_rate": 20.0}],
            ),
            Node(
                package="rebot_xbox_servo",
                executable="rebot_xbox_twist",
                name="rebot_xbox_twist",
                output="screen",
                condition=IfCondition(use_xbox),
                parameters=[mapping],
            ),
        ]
    )
