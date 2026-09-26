"""Preview reBotArm and Piper-H with mock hardware in the unified RViz workbench."""

import os
from importlib.machinery import SourceFileLoader

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
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessIO
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rebot_model",
                default_value="dm",
                choices=["dm", "rs"],
                description="reBotArm model used by the offline workbench",
            ),
            DeclareLaunchArgument(
                "initial_robot",
                default_value="rebotarm",
                choices=["rebotarm", "piperh"],
                description="Robot initially shown in the unified workbench",
            ),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("rebot_y", default_value="-0.40"),
            DeclareLaunchArgument("piper_y", default_value="0.40"),
            OpaqueFunction(function=_setup),
        ]
    )


def _moveit_config(robot, rebot_model="dm"):
    if robot == "piperh":
        return (
            MoveItConfigsBuilder("piperh", package_name="piperh_moveit_config")
            .robot_description(file_path="config/piperh.urdf.xacro")
            .robot_description_semantic(file_path="config/piperh.srdf")
            .robot_description_kinematics(file_path="config/kinematics.yaml")
            .joint_limits(file_path="config/joint_limits.yaml")
            .trajectory_execution(file_path="config/moveit_controllers.yaml")
            .planning_pipelines(pipelines=["ompl"])
            .to_moveit_configs()
        )
    suffix = "_rs" if rebot_model == "rs" else ""
    return (
        MoveItConfigsBuilder("rebotarm", package_name="rebotarm_moveit_config")
        .robot_description(file_path=f"config/rebotarm{suffix}.urdf.xacro")
        .robot_description_semantic(file_path=f"config/rebotarm{suffix}.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )


def _mock_stack(robot, moveit_config, controller_config, controllers):
    namespace = f"/{robot}"
    joint_states = f"{namespace}/joint_states"
    moveit_common = SourceFileLoader(
        f"{robot}_offline_moveit_launch_common",
        os.path.join(
            get_package_share_directory("rebotarm_moveit_config"),
            "launch",
            "moveit_launch_common.py",
        ),
    ).load_module()
    moveit_params = moveit_common.moveit_parameters(moveit_config)

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=robot,
            name="robot_state_publisher",
            output="both",
            parameters=[
                moveit_config.robot_description,
                {"frame_prefix": f"{robot}/"},
            ],
            remappings=[("/joint_states", joint_states)],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            namespace=robot,
            name="controller_manager",
            output="screen",
            parameters=[moveit_config.robot_description, controller_config],
            remappings=[("/joint_states", joint_states)],
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            namespace=robot,
            name="move_group",
            output="screen",
            parameters=[moveit_params],
            remappings=[("/joint_states", joint_states)],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            namespace=robot,
            name="joint_state_broadcaster_spawner",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager",
                f"{namespace}/controller_manager",
            ],
        ),
    ]
    for controller in controllers:
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                namespace=robot,
                name=f"{controller}_spawner",
                arguments=[
                    controller,
                    "--controller-manager",
                    f"{namespace}/controller_manager",
                ],
            )
        )
    return nodes, moveit_params


def _setup(context, *args, **kwargs):
    del args, kwargs
    rebot_model = LaunchConfiguration("rebot_model").perform(context).strip().lower()
    initial_robot = LaunchConfiguration("initial_robot").perform(context).strip().lower()
    if rebot_model not in ("dm", "rs"):
        raise ValueError("rebot_model must be dm or rs")
    if initial_robot not in ("rebotarm", "piperh"):
        raise ValueError("initial_robot must be rebotarm or piperh")

    rebot = _moveit_config("rebotarm", rebot_model)
    piper = _moveit_config("piperh")
    rebot_controllers = os.path.join(
        get_package_share_directory("rebot_xbox_hardware"),
        "config",
        "offline_rebotarm_controllers.yaml",
    )
    piper_controllers = os.path.join(
        get_package_share_directory("rebot_xbox_hardware"),
        "config",
        "offline_piperh_controllers.yaml",
    )
    rebot_nodes, rebot_moveit_params = _mock_stack(
        "rebotarm",
        rebot,
        rebot_controllers,
        ("rebotarm_controller", "gripper_controller"),
    )
    piper_nodes, _piper_moveit_params = _mock_stack(
        "piperh", piper, piper_controllers, ("arm_controller",)
    )

    rviz_config = os.path.join(
        get_package_share_directory("rebot_xbox_hardware"),
        "config",
        "dual_arm.rviz",
    )
    piper_rviz_model_params = {
        "piperh_robot_description": piper.robot_description["robot_description"],
        "piperh_robot_description_semantic": piper.robot_description_semantic[
            "robot_description_semantic"
        ],
        "piperh_robot_description_kinematics": piper.robot_description_kinematics[
            "robot_description_kinematics"
        ],
        "piperh_robot_description_planning": piper.joint_limits[
            "robot_description_planning"
        ],
    }
    teach_launch = os.path.join(
        get_package_share_directory("rebot_teach_mode"),
        "launch",
        "teach_mode.launch.py",
    )
    stack = [
        LogInfo(
            msg=(
                "DUAL-ARM OFFLINE PREVIEW: reBotArm and Piper-H use mock "
                "ros2_control; no serial/CAN driver or Xbox process is started."
            )
        ),
        *rebot_nodes,
        *piper_nodes,
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="rebotarm_world_tf",
            arguments=[
                "0", LaunchConfiguration("rebot_y"), "0", "0", "0", "0",
                "world", "rebotarm/base_link",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="rebotarm_moveit_world_tf",
            arguments=[
                "0", LaunchConfiguration("rebot_y"), "0", "0", "0", "0",
                "world", "base_link",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="piperh_world_tf",
            arguments=[
                "0", LaunchConfiguration("piper_y"), "0", "0", "0", "0",
                "world", "piperh/world",
            ],
        ),
        Node(
            package="rebot_xbox_hardware",
            executable="active_arm_manager",
            name="active_arm_manager",
            output="screen",
            parameters=[
                {"initial_robot": initial_robot, "offline_preview": True}
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(teach_launch),
            launch_arguments={
                "model": rebot_model,
                "arm_namespace": "rebotarm",
                "moveit_namespace": "rebotarm",
                "visualization_frame_prefix": "rebotarm",
                "allow_hardware": "false",
                "preview_only": "true",
                "use_xbox": "false",
                "require_xbox_locked": "false",
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(teach_launch),
            launch_arguments={
                "model": "piperh",
                "arm_namespace": "piperh",
                "moveit_namespace": "piperh",
                "visualization_frame_prefix": "piperh",
                "allow_hardware": "false",
                "preview_only": "true",
                "use_xbox": "false",
                "require_xbox_locked": "false",
            }.items(),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="dual_arm_offline_rviz",
            output="screen",
            arguments=["-d", rviz_config],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            parameters=[
                rebot_moveit_params,
                piper_rviz_model_params,
                {
                    "motion_preset.initial_robot": initial_robot,
                    "motion_preset.rebot_model": rebot_model,
                    "motion_preset.dual_arm": True,
                    "dual_arm.offline_preview": True,
                    # Keep the same nested planning layout as the real stack.
                    "rebot_demo.integrate_motion_planning": True,
                },
            ],
            remappings=[
                ("/joint_states", "/rebotarm/joint_states"),
                ("/check_state_validity", "/rebotarm/check_state_validity"),
                ("/execute_trajectory", "/rebotarm/execute_trajectory"),
                ("/display_planned_path", "/rebotarm/display_planned_path"),
            ],
            respawn=True,
            respawn_delay=2.0,
        ),
    ]

    guard = Node(
        package="rebot_xbox_hardware",
        executable="instance_guard",
        name="dual_arm_offline_single_instance_guard",
        output="screen",
        arguments=["--lock-name", "dual_arm"],
    )

    def start_stack_when_ready(event):
        output = (
            event.text.decode(errors="replace")
            if isinstance(event.text, bytes)
            else event.text
        )
        return stack if "single-instance guard ready:" in output else []

    return [
        guard,
        RegisterEventHandler(
            OnProcessIO(target_action=guard, on_stdout=start_stack_when_ready)
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=guard,
                on_exit=[
                    EmitEvent(event=Shutdown(reason="dual-arm preview guard exited"))
                ],
            )
        ),
    ]
