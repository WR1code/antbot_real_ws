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

"""Isaac dual-3D-lidar AMCL localization, cloud obstacles, and Nav2."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    lidar_share = get_package_share_directory('antbot_dual_lidar')
    params = os.path.join(
        lidar_share, 'config', 'isaac_3d_nav2_params.yaml')
    lidar_params = os.path.join(
        lidar_share, 'config', 'dual_lidar.yaml')
    rviz_default = os.path.join(
        lidar_share, 'config', 'rviz', 'isaac_3d_navigation.rviz')
    use_rviz = LaunchConfiguration('use_rviz')
    use_amcl = LaunchConfiguration('use_amcl')
    map_yaml = LaunchConfiguration('map')

    nav_nodes = [
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[params, {'yaml_filename': map_yaml}],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_isaac_map',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[params],
        ),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'map',
                description=(
                    'Saved 2D map YAML aligned with Isaac odom origin')),
            DeclareLaunchArgument(
                'use_rviz', default_value='true'),
            DeclareLaunchArgument(
                'use_amcl', default_value='true',
                description=(
                    'Use AMCL with the 3D-cloud horizontal /scan_0 projection '
                    'instead of an identity map->odom transform')),
            DeclareLaunchArgument(
                'rviz_config', default_value=rviz_default),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='map_to_odom_identity',
                condition=UnlessCondition(use_amcl),
                arguments=[
                    '--x', '0', '--y', '0', '--z', '0',
                    '--roll', '0', '--pitch', '0', '--yaw', '0',
                    '--frame-id', 'map',
                    '--child-frame-id', 'odom',
                ],
                output='screen',
            ),
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                condition=IfCondition(use_amcl),
                parameters=[
                    params,
                    {'use_sim_time': True, 'scan_topic': '/scan_0'},
                ],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_3d_amcl',
                output='screen',
                condition=IfCondition(use_amcl),
                parameters=[{
                    'autostart': True,
                    'bond_timeout': 15.0,
                    'node_names': ['amcl'],
                    'use_sim_time': True,
                }],
            ),
            Node(
                package='antbot_dual_lidar',
                executable='cloud_preprocessor',
                name='antbot_dual_lidar_preprocessor',
                output='screen',
                parameters=[
                    lidar_params,
                    {
                        'use_sim_time': True,
                        'lidar_profile': 'navigation',
                        'target_frame': 'base_link',
                        'front_left.input_topic': (
                            '/antbot/lidar/front_left/'
                            'points_raw_native'),
                        'rear_right.input_topic': (
                            '/antbot/lidar/rear_right/'
                            'points_raw_native'),
                    },
                ],
            ),
            Node(
                package='antbot_dual_lidar',
                executable='fixed_frame_mapper',
                name='antbot_fixed_frame_mapper',
                output='screen',
                parameters=[
                    {'use_sim_time': True, 'fixed_frame': 'odom'}],
            ),
            Node(
                package='antbot_dual_lidar',
                executable='dynamic_obstacle_monitor',
                name='antbot_dynamic_obstacle_monitor',
                output='screen',
                parameters=[{'use_sim_time': True}],
            ),
            *nav_nodes,
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2_3d_navigation',
                arguments=[
                    '-d', LaunchConfiguration('rviz_config')],
                parameters=[{'use_sim_time': True}],
                output='screen',
                condition=IfCondition(use_rviz),
            ),
        ]
    )
