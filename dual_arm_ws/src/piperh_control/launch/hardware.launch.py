"""Start the official Piper CAN driver with MoveIt and guarded adapters."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    control_share = get_package_share_directory("piperh_control")
    gravity_config = os.path.join(control_share, "config", "gravity_compensation.yaml")
    if not os.path.isfile(gravity_config):
        gravity_config = os.path.abspath(os.path.join(
            os.path.dirname(os.path.realpath(__file__)), "..", "config",
            "gravity_compensation.yaml",
        ))
    default_mapping = os.path.join(control_share, "config", "xbox_mapping.yaml")
    with open(os.path.join(control_share, "config", "servo_config.yaml"), encoding="utf-8") as stream:
        servo_values = yaml.safe_load(stream)
    servo_values["command_out_topic"] = "/piperh/servo_joint_trajectory"
    servo_values["joint_topic"] = LaunchConfiguration("joint_states_topic")
    servo = {"moveit_servo": servo_values}
    root_frame = LaunchConfiguration("root_frame")
    mount_x = LaunchConfiguration("mount_x")
    mount_y = LaunchConfiguration("mount_y")
    mount_z = LaunchConfiguration("mount_z")
    mount_roll = LaunchConfiguration("mount_roll")
    mount_pitch = LaunchConfiguration("mount_pitch")
    mount_yaw = LaunchConfiguration("mount_yaw")
    moveit = (
        MoveItConfigsBuilder("piperh", package_name="piperh_moveit_config")
        .robot_description(
            file_path="config/piperh.urdf.xacro",
            mappings={
                "root_frame": root_frame,
                "mount_x": mount_x,
                "mount_y": mount_y,
                "mount_z": mount_z,
                "mount_roll": mount_roll,
                "mount_pitch": mount_pitch,
                "mount_yaw": mount_yaw,
            },
        )
        .robot_description_semantic(file_path="config/piperh.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    description = moveit.robot_description["robot_description"]
    use_xbox = LaunchConfiguration("use_xbox")
    use_servo = LaunchConfiguration("use_servo")
    use_rviz = LaunchConfiguration("use_rviz")
    launch_description = generate_move_group_launch(moveit)
    launch_description.add_action(DeclareLaunchArgument("can_port", default_value="can0"))
    launch_description.add_action(
        DeclareLaunchArgument("driver_speed_percent", default_value="25")
    )
    launch_description.add_action(
        DeclareLaunchArgument("playback_max_velocity_rad_s", default_value="0.40")
    )
    launch_description.add_action(
        DeclareLaunchArgument("playback_max_acceleration_rad_s2", default_value="1.0")
    )
    launch_description.add_action(
        DeclareLaunchArgument("gravity_require_selected_robot", default_value="false")
    )
    launch_description.add_action(
        DeclareLaunchArgument(
            "gravity_config_file",
            default_value=gravity_config,
        )
    )
    launch_description.add_action(DeclareLaunchArgument("use_xbox", default_value="false"))
    launch_description.add_action(
        DeclareLaunchArgument("use_servo", default_value=use_xbox)
    )
    launch_description.add_action(DeclareLaunchArgument("use_rviz", default_value="true"))
    launch_description.add_action(
        DeclareLaunchArgument("joint_states_topic", default_value="/joint_states")
    )
    launch_description.add_action(
        DeclareLaunchArgument(
            "trajectory_action",
            default_value="/arm_controller/follow_joint_trajectory",
        )
    )
    launch_description.add_action(
        DeclareLaunchArgument("tf_prefix", default_value="")
    )
    launch_description.add_action(DeclareLaunchArgument("root_frame", default_value="world"))
    launch_description.add_action(DeclareLaunchArgument("mount_x", default_value="0.0"))
    launch_description.add_action(DeclareLaunchArgument("mount_y", default_value="0.0"))
    launch_description.add_action(DeclareLaunchArgument("mount_z", default_value="0.0"))
    launch_description.add_action(DeclareLaunchArgument("mount_roll", default_value="0.0"))
    launch_description.add_action(DeclareLaunchArgument("mount_pitch", default_value="0.0"))
    launch_description.add_action(DeclareLaunchArgument("mount_yaw", default_value="0.0"))
    launch_description.add_action(DeclareLaunchArgument("joy_device", default_value="/dev/input/js0"))
    launch_description.add_action(
        DeclareLaunchArgument("mapping", default_value=default_mapping)
    )
    launch_description.add_action(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[
                {
                    "robot_description": description,
                    "frame_prefix": LaunchConfiguration("tf_prefix"),
                }
            ],
        )
    )
    launch_description.add_action(
        Node(
            package="piper",
            executable="piper_single_ctrl",
            name="piper_ctrl_single_node",
            output="screen",
            # The vendor process exits when SocketCAN is not ready or is
            # unplugged. Keep retrying so udev/systemd recovery can reconnect
            # without restarting the complete dual-arm workbench.
            respawn=True,
            respawn_delay=2.0,
            parameters=[
                {
                    "can_port": LaunchConfiguration("can_port"),
                    "auto_enable": False,
                    "gripper_exist": False,
                }
            ],
            remappings=[
                ("joint_ctrl_single", "/piperh/driver_joint_command"),
                ("joint_states_single", "/piperh/driver_joint_states"),
                ("joint_states_feedback", "/piperh/driver_joint_states_diagnostic"),
                ("teach_joint_states_raw", "/piperh/teach_joint_states_raw"),
                ("arm_status", "/piperh/driver_arm_status"),
            ],
        )
    )
    launch_description.add_action(
        Node(
            package="piperh_control",
            executable="hardware_adapter",
            output="screen",
            respawn=True,
            respawn_delay=2.0,
            parameters=[
                LaunchConfiguration("gravity_config_file"),
                {
                    "feedback_topic": LaunchConfiguration("joint_states_topic"),
                    "trajectory_action": LaunchConfiguration("trajectory_action"),
                    "can_port": LaunchConfiguration("can_port"),
                    "driver_speed_percent": ParameterValue(
                        LaunchConfiguration("driver_speed_percent"), value_type=int
                    ),
                    "playback_max_velocity_rad_s": ParameterValue(
                        LaunchConfiguration("playback_max_velocity_rad_s"), value_type=float
                    ),
                    "playback_max_acceleration_rad_s2": ParameterValue(
                        LaunchConfiguration("playback_max_acceleration_rad_s2"), value_type=float
                    ),
                    "gravity_require_selected_robot": LaunchConfiguration(
                        "gravity_require_selected_robot"
                    ),
                    "gravity_mount_roll": mount_roll,
                    "gravity_mount_pitch": mount_pitch,
                    "gravity_mount_yaw": mount_yaw,
                }
            ],
        )
    )
    launch_description.add_action(
        Node(
            package="rviz2",
            executable="rviz2",
            output="log",
            arguments=["-d", os.path.join(get_package_share_directory("piperh_moveit_config"), "config", "moveit.rviz")],
            condition=IfCondition(use_rviz),
            parameters=[
                moveit.planning_pipelines,
                moveit.robot_description_kinematics,
                moveit.robot_description,
                moveit.robot_description_semantic,
                {"motion_preset.piper_driver_speed_percent": ParameterValue(
                    LaunchConfiguration("driver_speed_percent"), value_type=int
                )},
            ],
        )
    )
    launch_description.add_action(
        Node(
            package="moveit_servo",
            executable="servo_node",
            name="servo_node",
            output="screen",
            condition=IfCondition(use_servo),
            parameters=[
                servo,
                {"update_period": 0.01, "planning_group_name": "arm"},
                moveit.robot_description,
                moveit.robot_description_semantic,
                moveit.robot_description_kinematics,
                moveit.joint_limits,
            ],
        )
    )
    launch_description.add_action(
        Node(
            package="joy_linux",
            executable="joy_linux_node",
            name="piperh_joy",
            output="screen",
            condition=IfCondition(use_xbox),
            parameters=[{"dev": LaunchConfiguration("joy_device"), "deadzone": 0.05, "autorepeat_rate": 20.0}],
        )
    )
    launch_description.add_action(
        Node(
            package="rebot_xbox_servo",
            executable="rebot_xbox_twist",
            name="rebot_xbox_twist",
            output="screen",
            condition=IfCondition(use_xbox),
            parameters=[LaunchConfiguration("mapping")],
        )
    )
    return launch_description
