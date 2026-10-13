"""Run the test against the existing Piper-H hardware/MoveIt stack."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("iterations", default_value="20"),
        DeclareLaunchArgument("settle_sec", default_value="2.0"),
        DeclareLaunchArgument("output_dir", default_value="."),
        Node(
            package="rebotarm_pulse", executable="anchor_repeatability",
            name="piperh_anchor_repeatability", output="screen",
            parameters=[{
                "iterations": ParameterValue(LaunchConfiguration("iterations"), value_type=int),
                "settle_sec": ParameterValue(LaunchConfiguration("settle_sec"), value_type=float),
                "output_dir": ParameterValue(LaunchConfiguration("output_dir"), value_type=str),
                "use_sim_time": False,
            }],
            remappings=[
                ("/joint_states", "/piperh/driver_joint_states"),
                ("/rebotarm/joint_states", "/piperh/driver_joint_states"),
                ("/rebot_xbox/armed", "/piperh/xbox/armed"),
                ("/execute_trajectory", "/piperh/execute_trajectory"),
                ("/compute_ik", "/piperh/compute_ik"),
                ("/plan_kinematic_path", "/piperh/plan_kinematic_path"),
                ("/display_planned_path", "/piperh/display_planned_path"),
            ],
        ),
    ])
