"""Start the simulation MoveIt stack, Servo, joy_linux and Xbox guard node."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("rebot_xbox_servo")
    default_mapping = os.path.join(
        package_share, "config", "xbox_mapping.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("mapping", default_value=default_mapping),
            DeclareLaunchArgument("joy_device", default_value=""),
            DeclareLaunchArgument("start_joy", default_value="true"),
            DeclareLaunchArgument("start_moveit", default_value="true"),
            DeclareLaunchArgument("start_servo", default_value="true"),
            DeclareLaunchArgument("start_initializer", default_value="true"),
            DeclareLaunchArgument("start_isaac_bridge", default_value="true"),
            DeclareLaunchArgument("start_sim_gripper", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            OpaqueFunction(function=_launch_setup),
        ]
    )


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    package_share = get_package_share_directory("rebot_xbox_servo")
    mapping = LaunchConfiguration("mapping").perform(context)
    with open(mapping, encoding="utf-8") as mapping_file:
        mapping_parameters = yaml.safe_load(mapping_file)[
            "rebot_xbox_twist"
        ]["ros__parameters"]
    joy_device = LaunchConfiguration("joy_device").perform(context)
    if not joy_device:
        joy_device = mapping_parameters["device"]
    with open(
        os.path.join(package_share, "config", "servo_config.yaml"),
        encoding="utf-8",
    ) as servo_file:
        servo_parameters = {"moveit_servo": yaml.safe_load(servo_file)}

    moveit_config = (
        MoveItConfigsBuilder("rebotarm", package_name="rebotarm_moveit_config")
        .robot_description(file_path="config/rebotarm_rs.urdf.xacro")
        .robot_description_semantic(file_path="config/rebotarm_rs.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    simulation_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rebotarm_moveit_config"),
                "launch",
                "demo.launch.py",
            )
        ),
        condition=IfCondition(LaunchConfiguration("start_moveit")),
        launch_arguments={
            "model": "rs",
            "use_rviz": LaunchConfiguration("use_rviz"),
        }.items(),
    )
    isaac_bridge = Node(
        package="rebotarm_isaac_bridge",
        executable="joint_state_udp_bridge",
        name="rebotarm_isaac_joint_state_udp_bridge",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_isaac_bridge")),
        parameters=[
            {
                "joint_state_topic": "/joint_states",
                "host": "127.0.0.1",
                "port": 5005,
                "send_rate_hz": 60.0,
                "stale_timeout_sec": 1.0,
                "invert_joint_sign": False,
            }
        ],
    )

    def make_servo_node():
        return Node(
            package="moveit_servo",
            executable="servo_node",
            name="servo_node",
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_servo")),
            parameters=[
                servo_parameters,
                {"update_period": 0.01, "planning_group_name": "arm"},
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
            ],
        )
    joy_node = Node(
        package="joy_linux",
        executable="joy_linux_node",
        name="rebot_xbox_joy_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_joy")),
        parameters=[
            {
                "dev": joy_device,
                "deadzone": 0.05,
                "autorepeat_rate": 20.0,
                "sticky_buttons": False,
            }
        ],
    )

    def make_xbox_node():
        return Node(
            package="rebot_xbox_servo",
            executable="rebot_xbox_twist",
            name="rebot_xbox_twist",
            output="screen",
            parameters=[mapping],
        )

    arm_initializer_node = Node(
        package="rebot_xbox_servo",
        executable="arm_initializer",
        name="rebot_xbox_arm_initializer",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_initializer")),
        parameters=[mapping],
    )
    delayed_servo_node = make_servo_node()
    delayed_xbox_node = make_xbox_node()
    direct_control_nodes = GroupAction(
        condition=UnlessCondition(LaunchConfiguration("start_initializer")),
        actions=[make_servo_node(), make_xbox_node()],
    )
    initializer_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=arm_initializer_node,
            on_exit=lambda event, context: _after_initializer(
                event,
                context,
                delayed_servo_node,
                delayed_xbox_node,
            ),
        )
    )
    sim_gripper_node = Node(
        package="rebot_xbox_servo",
        executable="sim_gripper",
        name="rebot_xbox_sim_gripper",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_sim_gripper")),
        parameters=[mapping],
    )
    return [
        simulation_stack,
        isaac_bridge,
        joy_node,
        sim_gripper_node,
        arm_initializer_node,
        initializer_exit_handler,
        direct_control_nodes,
    ]


def _after_initializer(event, context, servo_node, xbox_node):
    """Start user control only after initialization, or stop on failure."""
    del context
    if event.returncode == 0:
        return [servo_node, xbox_node]
    return [
        EmitEvent(
            event=Shutdown(
                reason=(
                    "arm initialization failed with code "
                    f"{event.returncode}"
                )
            )
        )
    ]
