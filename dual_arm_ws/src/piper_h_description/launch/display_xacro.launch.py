# Copyright 2026 The antbot_real_ws contributors
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Display the pinned official Piper-H model."""
    package_path = get_package_share_path('piper_h_description')
    # The historical camera/gripper xacro uses an older kinematic chain.
    default_model = package_path / 'urdf/piper_h_description.urdf'
    default_rviz = package_path / 'rviz/piper_ctrl.rviz'

    gui_arg = DeclareLaunchArgument(
        name='gui', default_value='true', choices=['true', 'false'],
        description='Flag to enable joint_state_publisher_gui')
    model_arg = DeclareLaunchArgument(
        name='model', default_value=str(default_model),
        description='Absolute path to robot URDF or xacro file')
    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig', default_value=str(default_rviz),
        description='Absolute path to RViz config file')

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]), value_type=str)
    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}])
    joint_state_publisher = Node(
        package='joint_state_publisher', executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui')))
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui')))
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')])

    return LaunchDescription([
        gui_arg, model_arg, rviz_arg, joint_state_publisher,
        joint_state_publisher_gui, robot_state_publisher, rviz,
    ])
