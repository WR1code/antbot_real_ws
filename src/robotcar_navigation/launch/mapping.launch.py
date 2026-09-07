from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _share(package_name: str) -> Path:
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
    autostart = LaunchConfiguration("autostart")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "params_file",
                default_value=str(robotcar_navigation / "config" / "slam_toolbox_params.yaml"),
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=str(robotcar_navigation / "rviz" / "mapping.rviz"),
            ),
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_slam",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": autostart,
                        "node_names": ["slam_toolbox"],
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", LaunchConfiguration("rviz_config")],
                condition=IfCondition(LaunchConfiguration("use_rviz")),
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            ),
        ]
    )
