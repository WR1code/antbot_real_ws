# Copyright 2026 ROBOTIS AI CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Start ANTBot Nav2 and reuse robotcar's complete RViz waypoint toolchain."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('antbot_navigation')
    robotcar_navigation_dir = get_package_share_directory(
        'robotcar_navigation')
    mode = LaunchConfiguration('mode')
    start_nav2 = LaunchConfiguration('start_nav2')
    start_waypoint_saver = LaunchConfiguration('start_waypoint_saver')
    use_rviz = LaunchConfiguration('use_rviz')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'launch', 'navigation.launch.py')),
        condition=IfCondition(start_nav2),
        launch_arguments={
            'mode': mode,
            'world': LaunchConfiguration('world'),
            'map': LaunchConfiguration('map'),
        }.items())

    editor = Node(
        package='robotcar_navigation',
        executable='wp_edit_node',
        name='wp_edit_node',
        output='screen',
        parameters=[{
            'use_sim_time': PythonExpression(["'", mode, "' == 'sim'"]),
            'load': LaunchConfiguration('waypoints_file'),
        }])

    waypoint_navigator = Node(
        package='robotcar_navigation',
        executable='wp_navi_server',
        name='wp_navi_server',
        output='screen',
        parameters=[{
            'use_sim_time': PythonExpression(["'", mode, "' == 'sim'"]),
            'save_file': LaunchConfiguration('waypoints_file'),
        }])

    waypoint_set_pose = Node(
        package='robotcar_navigation',
        executable='wp_set_pose',
        name='wp_set_pose',
        output='screen',
        parameters=[{
            'use_sim_time': PythonExpression(["'", mode, "' == 'sim'"]),
        }])

    waypoint_saver = Node(
        package='robotcar_navigation',
        executable='wp_saver',
        name='wp_saver',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(start_waypoint_saver),
        parameters=[{
            'use_sim_time': PythonExpression(["'", mode, "' == 'sim'"]),
        }])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_waypoints',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{
            'use_sim_time': PythonExpression(["'", mode, "' == 'sim'"]),
        }])

    keepout_manager = Node(
        package='robotcar_navigation',
        executable='keepout_zone_manager.py',
        name='keepout_zone_manager',
        output='screen',
        parameters=[{
            'use_sim_time': PythonExpression(["'", mode, "' == 'sim'"]),
            'keepout_file': LaunchConfiguration('keepout_file'),
        }])

    speed_zone_manager = Node(
        package='robotcar_navigation',
        executable='speed_zone_manager.py',
        name='speed_zone_manager',
        output='screen',
        parameters=[{
            'use_sim_time': PythonExpression(["'", mode, "' == 'sim'"]),
            'speed_zone_file': LaunchConfiguration('speed_zone_file'),
        }])

    intent_monitor = Node(
        package='robotcar_navigation',
        executable='robot_intent_monitor.py',
        name='robot_intent_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': PythonExpression(["'", mode, "' == 'sim'"]),
        }])

    awareness_monitor = Node(
        package='robotcar_navigation',
        executable='robot_awareness_monitor.py',
        name='robot_awareness_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': PythonExpression(["'", mode, "' == 'sim'"]),
            'stuck_history_file': LaunchConfiguration('stuck_history_file'),
        }])

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='sim', choices=['sim', 'real'],
            description='sim uses /clock; real uses wall time'),
        DeclareLaunchArgument(
            'start_nav2', default_value='true',
            description='Start the existing ANTBot Nav2 stack'),
        DeclareLaunchArgument(
            'world', default_value='',
            description='World key from maps/worlds.yaml'),
        DeclareLaunchArgument(
            'map', default_value='',
            description='Map YAML path; overrides world'),
        DeclareLaunchArgument(
            'waypoints_file',
            default_value=os.path.join(
                robotcar_navigation_dir, 'config', 'waypoints.xml'),
            description='robotcar waypoint XML loaded by wp_edit_node'),
        DeclareLaunchArgument(
            'start_waypoint_saver', default_value='true',
            description='Start robotcar keyboard save/action utility'),
        DeclareLaunchArgument(
            'keepout_file', default_value='',
            description='Optional named keepout-zone JSON file'),
        DeclareLaunchArgument(
            'speed_zone_file', default_value='',
            description='Optional named speed-zone JSON file'),
        DeclareLaunchArgument(
            'stuck_history_file', default_value='',
            description='Persistent historical recovery-point JSON file'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(
                package_dir, 'rviz', 'waypoint_navigation.rviz')),
        nav2,
        editor,
        waypoint_navigator,
        waypoint_set_pose,
        waypoint_saver,
        keepout_manager,
        speed_zone_manager,
        intent_monitor,
        awareness_monitor,
        rviz,
    ])
