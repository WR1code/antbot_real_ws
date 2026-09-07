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

"""Load a saved map and robotcar waypoint editor without a simulator."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    """Create the wall-time offline waypoint editing stack."""
    robotcar_dir = get_package_share_directory('robotcar_navigation')
    package_dir = get_package_share_directory('antbot_navigation')
    description_dir = get_package_share_directory('antbot_description')
    robot_description = xacro.process_file(
        os.path.join(description_dir, 'urdf', 'antbot.xacro')).toxml()
    map_file = LaunchConfiguration('map_yaml')
    waypoint_file = LaunchConfiguration('waypoint_file')
    use_rviz = LaunchConfiguration('use_rviz')
    show_3d_cloud = LaunchConfiguration('show_3d_cloud')
    show_rgbd_cloud = LaunchConfiguration('show_rgbd_cloud')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value='',
            description='Compatibility alias used as map_yaml default'),
        DeclareLaunchArgument(
            'map_yaml', default_value=LaunchConfiguration('map'),
            description='Absolute path to the saved occupancy map YAML'),
        DeclareLaunchArgument(
            'waypoints_file',
            default_value='',
            description='Compatibility alias used as waypoint_file default'),
        DeclareLaunchArgument(
            'waypoint_file', default_value=LaunchConfiguration('waypoints_file'),
            description='Absolute path to robotcar waypoint XML'),
        DeclareLaunchArgument(
            'keepout_file', default_value='',
            description='Optional named keepout-zone JSON file'),
        DeclareLaunchArgument(
            'speed_zone_file', default_value='',
            description='Optional named speed-zone JSON file'),
        DeclareLaunchArgument('pointcloud_path', default_value=''),
        DeclareLaunchArgument('pointcloud_metadata', default_value=''),
        DeclareLaunchArgument('show_3d_cloud', default_value='false'),
        DeclareLaunchArgument('show_rgbd_cloud', default_value='false'),
        DeclareLaunchArgument('rgbd_preview_path', default_value=''),
        DeclareLaunchArgument(
            'rgbd_cloud_topic', default_value='/antbot/rgbd/offline_cloud'),
        DeclareLaunchArgument(
            'pointcloud_topic', default_value='/antbot/offline_map_points'),
        DeclareLaunchArgument('pointcloud_target_frame', default_value='map'),
        DeclareLaunchArgument('pointcloud_publish_rate', default_value='0.5'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(
                robotcar_dir, 'rviz', 'editwaypoints.rviz')),
        DeclareLaunchArgument(
            'rviz_3d_config',
            default_value=os.path.join(
                package_dir, 'rviz', 'offline_waypoint_editor_3d.rviz')),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': map_file,
                'use_sim_time': False,
            }],
        ),
        TimerAction(
            period=1.0,
            actions=[
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_offline_map',
                    output='screen',
                    parameters=[{
                        'autostart': True,
                        'node_names': ['map_server'],
                        'use_sim_time': False,
                    }],
                ),
            ],
        ),
        Node(
            package='robotcar_navigation',
            executable='wp_edit_node',
            name='wp_edit_node',
            output='screen',
            parameters=[{
                'load': waypoint_file,
                'use_sim_time': False,
            }],
        ),
        Node(
            package='robotcar_navigation',
            executable='keepout_zone_manager.py',
            name='keepout_zone_manager',
            output='screen',
            parameters=[{
                'keepout_file': LaunchConfiguration('keepout_file'),
                'use_sim_time': False,
            }],
        ),
        Node(
            package='robotcar_navigation',
            executable='speed_zone_manager.py',
            name='speed_zone_manager',
            output='screen',
            parameters=[{
                'speed_zone_file': LaunchConfiguration('speed_zone_file'),
                'use_sim_time': False,
            }],
        ),
        Node(
            package='antbot_dual_lidar',
            executable='offline_pointcloud_publisher',
            name='antbot_offline_pointcloud_publisher',
            output='screen',
            condition=IfCondition(show_3d_cloud),
            parameters=[{
                'pointcloud_path': LaunchConfiguration('pointcloud_path'),
                'metadata_path': LaunchConfiguration('pointcloud_metadata'),
                'publish_topic': LaunchConfiguration('pointcloud_topic'),
                'target_frame': LaunchConfiguration('pointcloud_target_frame'),
                'publish_rate': LaunchConfiguration('pointcloud_publish_rate'),
                'use_sim_time': False,
            }],
        ),
        Node(
            package='antbot_rgbd_dataset',
            executable='offline_preview_publisher',
            name='antbot_rgbd_offline_cloud_publisher',
            output='screen',
            condition=IfCondition(show_rgbd_cloud),
            parameters=[{
                'preview_path': LaunchConfiguration('rgbd_preview_path'),
                'publish_topic': LaunchConfiguration('rgbd_cloud_topic'),
                'publish_rate': 0.5,
                'use_sim_time': False,
            }],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='antbot_robot_state_publisher_offline_editor',
            output='screen',
            condition=IfCondition(show_3d_cloud),
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': False,
            }],
            remappings=[
                ('robot_description', '/antbot/robot_description'),
            ],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher_offline_editor',
            output='screen',
            condition=IfCondition(show_3d_cloud),
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': False,
            }],
            remappings=[
                ('robot_description', '/antbot/robot_description'),
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='offline_map_to_robot',
            output='screen',
            condition=IfCondition(show_3d_cloud),
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'map', '--child-frame-id', 'base_link',
            ],
        ),
        GroupAction(
            condition=IfCondition(use_rviz),
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2_waypoint_editor',
                    output='screen',
                    condition=IfCondition(show_3d_cloud),
                    arguments=[
                        '-d', LaunchConfiguration('rviz_3d_config')],
                    parameters=[{'use_sim_time': False}],
                ),
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2_waypoint_editor',
                    output='screen',
                    condition=UnlessCondition(show_3d_cloud),
                    arguments=['-d', LaunchConfiguration('rviz_config')],
                    parameters=[{'use_sim_time': False}],
                ),
            ],
        ),
    ])
