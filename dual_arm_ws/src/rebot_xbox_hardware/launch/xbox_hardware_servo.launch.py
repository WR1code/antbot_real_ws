"""Start the selected real arm, its matching model, and shared Xbox stack."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit, OnProcessIO
from launch.events import Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

from rebot_xbox_hardware.robot_selector import select_robot


def generate_launch_description():
    package_share = get_package_share_directory("rebot_xbox_hardware")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot",
                default_value="rebotarm",
                description="Controlled robot: rebotarm or piperh",
            ),
            DeclareLaunchArgument(
                "model",
                default_value="rs",
                description="reBotArm actuator model: dm or rs; ignored for Piper-H",
            ),
            DeclareLaunchArgument("channel", default_value="can0"),
            DeclareLaunchArgument(
                "arm_namespace",
                default_value="",
                description="Empty selects rebotarm or piperh to match robot",
            ),
            DeclareLaunchArgument(
                "mapping",
                default_value="",
                description="Optional Xbox mapping YAML; empty selects per robot",
            ),
            DeclareLaunchArgument("joy_device", default_value=""),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "use_forbidden_zones",
                default_value="true",
                description="Load named 3-D forbidden zones into MoveIt",
            ),
            DeclareLaunchArgument(
                "forbidden_zones_config",
                default_value=os.path.join(
                    package_share, "config", "forbidden_zones.yaml"
                ),
                description="YAML file containing named areas and area groups",
            ),
            DeclareLaunchArgument(
                "active_zone_groups",
                default_value="",
                description="Comma-separated group names; empty uses enabled groups",
            ),
            DeclareLaunchArgument(
                "forbidden_zones_user_config",
                default_value="",
                description=(
                    "Persistent user-zone YAML; empty uses "
                    "~/.ros/rebotarm/forbidden_zones_user.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "use_teach",
                default_value="true",
                description="Start guarded drag teaching and action replay",
            ),
            DeclareLaunchArgument(
                "teach_use_xbox",
                default_value="true",
                description="Use X to teach, B to replay, and Start to cancel",
            ),
            OpaqueFunction(function=_setup),
        ]
    )


def _setup(context, *args, **kwargs):
    del args, kwargs
    selection = select_robot(
        LaunchConfiguration("robot").perform(context),
        LaunchConfiguration("model").perform(context),
        LaunchConfiguration("arm_namespace").perform(context),
    )
    if selection.robot == "piperh":
        return _guarded_stack(selection.namespace, _piperh_stack(context))
    return _guarded_stack(
        selection.namespace,
        _rebotarm_stack(context, selection.model, selection.namespace),
    )


def _selected_mapping(context, robot):
    configured = LaunchConfiguration("mapping").perform(context).strip()
    if configured:
        return configured
    package = "piperh_control" if robot == "piperh" else "rebot_xbox_servo"
    return os.path.join(
        get_package_share_directory(package), "config", "xbox_mapping.yaml"
    )


def _selected_joy_device(context, mapping):
    with open(mapping, encoding="utf-8") as stream:
        mapping_params = yaml.safe_load(stream)["rebot_xbox_twist"]["ros__parameters"]
    return (
        LaunchConfiguration("joy_device").perform(context)
        or mapping_params["device"]
    )


def _forbidden_zones_node(context, model):
    user_config = LaunchConfiguration("forbidden_zones_user_config").perform(
        context
    ).strip()
    if not user_config and model == "piperh":
        user_config = os.path.expanduser(
            "~/.ros/piperh/forbidden_zones_user.yaml"
        )
    return Node(
        package="rebot_xbox_hardware",
        executable="forbidden_zone_manager",
        name="forbidden_zone_manager",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_forbidden_zones")),
        parameters=[
            {
                "config_file": LaunchConfiguration("forbidden_zones_config"),
                "active_groups": LaunchConfiguration("active_zone_groups"),
                "user_config_file": user_config,
                "model": model,
            }
        ],
    )


def _piperh_stack(context):
    mapping = _selected_mapping(context, "piperh")
    joy_device = _selected_joy_device(context, mapping)
    piper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("piperh_control"),
                "launch",
                "hardware.launch.py",
            )
        ),
        launch_arguments={
            "can_port": LaunchConfiguration("channel"),
            "use_xbox": "true",
            "joy_device": joy_device,
            "mapping": mapping,
            "use_rviz": LaunchConfiguration("use_rviz"),
        }.items(),
    )
    return [
        LogInfo(
            msg=(
                "Selected Piper-H: loading the Piper-H URDF/SRDF, CAN driver, "
                "MoveIt, Servo, Xbox safety controls, motion presets, and zones."
            )
        ),
        piper_launch,
        _forbidden_zones_node(context, "piperh"),
    ]


def _rebotarm_stack(context, model, namespace):
    mapping = _selected_mapping(context, "rebotarm")
    joy_device = _selected_joy_device(context, mapping)

    with open(
        os.path.join(
            get_package_share_directory("rebot_xbox_servo"),
            "config",
            "servo_config.yaml",
        ),
        encoding="utf-8",
    ) as stream:
        servo = yaml.safe_load(stream)
    command_topic = f"/{namespace}/xbox_servo/joint_trajectory"
    servo["joint_topic"] = f"/{namespace}/joint_states"
    servo["command_out_topic"] = command_topic
    servo_parameters = {"moveit_servo": servo}

    moveit_config = (
        MoveItConfigsBuilder("rebotarm", package_name="rebotarm_moveit_config")
        .robot_description(
            file_path=(
                "config/rebotarm_rs.urdf.xacro"
                if model == "rs"
                else "config/rebotarm.urdf.xacro"
            )
        )
        .robot_description_semantic(
            file_path=(
                "config/rebotarm_rs.srdf"
                if model == "rs"
                else "config/rebotarm.srdf"
            )
        )
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rebotarm_bringup"),
                "launch",
                "driver.launch.py",
            )
        ),
        launch_arguments={
            "model": model,
            "channel": LaunchConfiguration("channel"),
            "arm_namespace": namespace,
            "servo_joint_trajectory_topic": command_topic,
            "servo_command_timeout": "0.15",
        }.items(),
    )
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rebotarm_moveit_config"),
                "launch",
                "hardware.launch.py",
            )
        ),
        launch_arguments={
            "model": model,
            "arm_namespace": namespace,
            "use_rviz": LaunchConfiguration("use_rviz"),
        }.items(),
    )
    teach = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rebot_teach_mode"),
                "launch",
                "teach_mode.launch.py",
            )
        ),
        condition=IfCondition(LaunchConfiguration("use_teach")),
        launch_arguments={
            "model": model,
            "arm_namespace": namespace,
            # This is already the explicit real-hardware launch entry.  The
            # teach node still requires fresh feedback, IDLE driver state, and
            # an Xbox LOCKED report before either gravity compensation or
            # trajectory playback can start.
            "allow_hardware": "true",
            "use_xbox": LaunchConfiguration("teach_use_xbox"),
            "require_xbox_locked": "true",
        }.items(),
    )
    return [
        LogInfo(
            msg=f"Selected reBotArm {model.upper()}: loading matching driver and model."
        ),
        driver,
        moveit,
        _forbidden_zones_node(context, model),
        teach,
        Node(
            package="moveit_servo",
            executable="servo_node",
            name="servo_node",
            output="screen",
            parameters=[
                servo_parameters,
                {"update_period": 0.01, "planning_group_name": "arm"},
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
            ],
        ),
        Node(
            package="joy_linux",
            executable="joy_linux_node",
            name="rebot_xbox_joy_node",
            output="screen",
            parameters=[{"dev": joy_device, "deadzone": 0.05, "autorepeat_rate": 20.0}],
        ),
        Node(
            package="rebot_xbox_servo",
            executable="rebot_xbox_twist",
            name="rebot_xbox_twist",
            output="screen",
            parameters=[mapping],
        ),
        Node(
            package="rebot_xbox_hardware",
            executable="hardware_gripper",
            name="rebot_xbox_hardware_gripper",
            output="screen",
            parameters=[
                {
                    "state_topic": f"/{namespace}/gripper/state",
                    "command_topic": f"/{namespace}/gripper/cmd/pos_vel",
                    "open_position": 5.0 if model == "rs" else -5.0,
                    "closed_position": 0.0,
                    "maximum_velocity": 2.0,
                    "maximum_closing_torque": 0.80,
                }
            ],
        ),
    ]


def _guarded_stack(namespace, stack):
    guard = Node(
        package="rebot_xbox_hardware",
        executable="instance_guard",
        name="rebotarm_single_instance_guard",
        output="screen",
        arguments=["--lock-name", namespace],
    )

    def start_stack_when_guard_is_ready(event):
        text = (
            event.text.decode(errors="replace")
            if isinstance(event.text, bytes)
            else event.text
        )
        if "single-instance guard ready:" in text:
            return stack
        return []

    return [
        guard,
        RegisterEventHandler(
            OnProcessIO(
                target_action=guard,
                on_stdout=start_stack_when_guard_is_ready,
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=guard,
                on_exit=[
                    EmitEvent(
                        event=Shutdown(
                            reason="single-instance guard exited; refusing duplicate stack"
                        )
                    )
                ],
            )
        ),
    ]
