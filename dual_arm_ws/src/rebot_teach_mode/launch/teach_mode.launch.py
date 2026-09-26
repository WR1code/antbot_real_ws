"""Start model-aware guarded teaching and optional Xbox controls."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from rebot_teach_mode.model_profiles import teaching_parameters_for_model


def generate_launch_description() -> LaunchDescription:
    config = os.path.join(
        get_package_share_directory("rebot_teach_mode"),
        "config",
        "teach_mode.yaml",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=config),
            DeclareLaunchArgument("model", default_value="rs"),
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            DeclareLaunchArgument("moveit_namespace", default_value=""),
            DeclareLaunchArgument(
                "visualization_frame_prefix",
                default_value="",
                description=(
                    "TF prefix used only by RViz markers; MoveIt requests keep "
                    "using the unprefixed robot-model frames"
                ),
            ),
            DeclareLaunchArgument(
                "xbox_armed_topic", default_value="/rebot_xbox/armed"
            ),
            DeclareLaunchArgument("allow_hardware", default_value="false"),
            DeclareLaunchArgument(
                "preview_only",
                default_value="false",
                description="Label this hardware-disabled node as an offline preview stack",
            ),
            DeclareLaunchArgument("use_xbox", default_value="false"),
            DeclareLaunchArgument("require_xbox_locked", default_value="true"),
            OpaqueFunction(function=_setup),
        ]
    )


def _setup(context, *args, **kwargs):
    del args, kwargs
    model = LaunchConfiguration("model").perform(context).strip().lower()
    namespace = (
        LaunchConfiguration("arm_namespace").perform(context).strip("/")
    )
    if not namespace:
        raise ValueError("arm_namespace must not be empty")
    model_parameters = teaching_parameters_for_model(model)
    if model == "piperh":
        description_package = "piperh_moveit_config"
        description_file = "piperh.urdf.xacro"
    else:
        description_package = "rebotarm_moveit_config"
        description_file = "rebotarm_rs.urdf.xacro" if model == "rs" else "rebotarm.urdf.xacro"
    common_parameters = {
        "arm_namespace": namespace,
        "moveit_namespace": LaunchConfiguration("moveit_namespace"),
        "visualization_frame_prefix": LaunchConfiguration(
            "visualization_frame_prefix"
        ),
        "xbox_armed_topic": LaunchConfiguration("xbox_armed_topic"),
        "allow_hardware": ParameterValue(
            LaunchConfiguration("allow_hardware"), value_type=bool
        ),
        "preview_only": ParameterValue(
            LaunchConfiguration("preview_only"), value_type=bool
        ),
        "require_xbox_locked": ParameterValue(
            LaunchConfiguration("require_xbox_locked"), value_type=bool
        ),
        **model_parameters,
        "robot_description": ParameterValue(
            Command(
                [
                    "xacro ",
                    PathJoinSubstitution(
                        [
                            FindPackageShare(description_package),
                            "config",
                            description_file,
                        ]
                    ),
                ]
            ),
            value_type=str,
        ),
    }
    return [
        Node(
            package="rebot_teach_mode",
            executable="teach_mode_node",
            namespace=namespace,
            name="rebot_teach_mode",
            output="screen",
            parameters=[LaunchConfiguration("config"), common_parameters],
            respawn=True,
            respawn_delay=2.0,
        ),
        Node(
            package="rebot_teach_mode",
            executable="teach_xbox_bridge",
            namespace=namespace,
            name="rebot_teach_xbox_bridge",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_xbox")),
            parameters=[
                LaunchConfiguration("config"),
                {
                    "arm_namespace": namespace,
                    "teach_status_topic": f"/{namespace}/teach/status",
                },
            ],
        ),
    ]
