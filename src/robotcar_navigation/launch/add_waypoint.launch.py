#!/usr/bin/env python3

from pathlib import Path

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _share(package_name: str) -> Path:
    """
    优先从 install/share 里面找包路径。
    如果包还没有安装，则回退到源码空间下查找。
    """
    try:
        return Path(get_package_share_directory(package_name))
    except PackageNotFoundError:
        source_pkg = Path(__file__).resolve().parents[2] / package_name
        if source_pkg.exists():
            return source_pkg
        raise


def generate_launch_description():
    robotcar_navigation = _share("robotcar_navigation")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    map_file = LaunchConfiguration("map")
    rviz_config = LaunchConfiguration("rviz_config")
    waypoints_file = LaunchConfiguration("waypoints_file")
    publish_floor = LaunchConfiguration("publish_floor")
    floor_model = LaunchConfiguration("floor_model")
    floor_width = LaunchConfiguration("floor_width")
    floor_height = LaunchConfiguration("floor_height")
    floor_x = LaunchConfiguration("floor_x")
    floor_y = LaunchConfiguration("floor_y")
    floor_z = LaunchConfiguration("floor_z")

    return LaunchDescription(
        [
            # =========================
            # Launch 参数
            # =========================
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="是否使用仿真时间。真实车一般 false，Gazebo 仿真一般 true。",
            ),

            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="是否启动 RViz2。",
            ),

            DeclareLaunchArgument(
                "map",
                default_value=str(robotcar_navigation / "maps" / "map.yaml"),
                description="地图 yaml 文件路径。",
            ),

            DeclareLaunchArgument(
                "rviz_config",
                default_value=str(robotcar_navigation / "rviz" / "editwaypoints.rviz"),
                description="RViz 配置文件路径。",
            ),

            DeclareLaunchArgument(
                "waypoints_file",
                default_value=str(robotcar_navigation / "config" / "waypoints.xml"),
                description="航点 XML 文件路径。",
            ),

            DeclareLaunchArgument(
                "publish_floor",
                default_value="true",
                description="是否在 RViz 中发布纹理地板 mesh。",
            ),

            DeclareLaunchArgument(
                "floor_model",
                default_value="package://robotcar_navigation/meshes/plane_textured.dae",
                description="地板 mesh 资源路径。",
            ),

            DeclareLaunchArgument(
                "floor_width",
                default_value="4.19",
                description="地板 mesh 在 map 坐标系下的 X 尺寸。",
            ),

            DeclareLaunchArgument(
                "floor_height",
                default_value="4.18",
                description="地板 mesh 在 map 坐标系下的 Y 尺寸。",
            ),

            DeclareLaunchArgument(
                "floor_x",
                default_value="-0.004",
                description="地板 mesh 中心点 X。",
            ),

            DeclareLaunchArgument(
                "floor_y",
                default_value="0.001",
                description="地板 mesh 中心点 Y。",
            ),

            DeclareLaunchArgument(
                "floor_z",
                default_value="-0.02",
                description="地板 mesh Z，低于航点交互控件以免遮挡旋转圈。",
            ),

            # =========================
            # 地图服务器
            # ROS1: map_server
            # ROS2: nav2_map_server + lifecycle_manager
            # =========================
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[
                    {
                        "yaml_filename": map_file,
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),

            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_map",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": True,
                        "node_names": ["map_server"],
                    }
                ],
            ),

            # =========================
            # 航点编辑节点
            # 作用：读取 waypoints.xml，并在 RViz 中显示/编辑航点
            # =========================
            Node(
                package="robotcar_navigation",
                executable="wp_edit_node",
                name="wp_edit_node",
                output="screen",
                parameters=[
                    {
                        "load": waypoints_file,
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),

            # =========================
            # 航点保存节点
            # 作用：保存编辑后的航点
            # =========================
            Node(
                package="robotcar_navigation",
                executable="wp_saver",
                name="wp_saver",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "save_file": waypoints_file,
                    }
                ],
            ),

            Node(
                package="robotcar_navigation",
                executable="floor_texture_publisher",
                name="floor_texture_publisher",
                output="screen",
                condition=IfCondition(publish_floor),
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "model": floor_model,
                        "width": ParameterValue(floor_width, value_type=float),
                        "height": ParameterValue(floor_height, value_type=float),
                        "pos_x": ParameterValue(floor_x, value_type=float),
                        "pos_y": ParameterValue(floor_y, value_type=float),
                        "pos_z": ParameterValue(floor_z, value_type=float),
                    }
                ],
            ),

            # =========================
            # RViz2
            # 注意：RViz 里面 Fixed Frame 应该设为 map
            # =========================
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                    }
                ],
                output="screen",
            ),
        ]
    )
