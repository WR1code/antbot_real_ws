from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory("antbot_lidar_fusion")
    description_share = get_package_share_directory("antbot_description")
    use_rviz = LaunchConfiguration("use_rviz")
    publish_description = LaunchConfiguration("publish_description")
    use_sim_time = LaunchConfiguration("use_sim_time")
    front_topic = LaunchConfiguration("front_scan_topic")
    rear_topic = LaunchConfiguration("rear_scan_topic")
    target_frame = LaunchConfiguration("target_frame")
    robot_description = ParameterValue(
        Command(["xacro ", os.path.join(description_share, "urdf", "antbot.xacro")]),
        value_type=str,
    )
    common = [{"use_sim_time": use_sim_time}]
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("publish_description", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("front_scan_topic", default_value="/scan_0"),
            DeclareLaunchArgument("rear_scan_topic", default_value="/scan_1"),
            DeclareLaunchArgument("target_frame", default_value="base_link"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="antbot_robot_state_publisher",
                parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}],
                remappings=[("robot_description", "/antbot/robot_description")],
                condition=IfCondition(publish_description),
                output="screen",
            ),
            Node(
                package="antbot_lidar_fusion",
                executable="lidar_fusion_node",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "front_scan_topic": front_topic,
                        "rear_scan_topic": rear_topic,
                        "target_frame": target_frame,
                    }
                ],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", os.path.join(share, "config", "antbot_dual_lidar.rviz")],
                parameters=common,
                condition=IfCondition(use_rviz),
                output="screen",
            ),
        ]
    )
