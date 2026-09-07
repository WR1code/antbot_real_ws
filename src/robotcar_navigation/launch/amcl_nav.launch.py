from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    nav2_bringup = Path(get_package_share_directory("nav2_bringup"))
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "map",
                default_value=str(robotcar_navigation / "maps" / "map.yaml"),
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=str(robotcar_navigation / "config" / "nav2_params.yaml"),
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=str(robotcar_navigation / "rviz" / "nav.rviz"),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(nav2_bringup / "launch" / "bringup_launch.py")),
                launch_arguments={
                    "map": LaunchConfiguration("map"),
                    "use_sim_time": use_sim_time,
                    "params_file": LaunchConfiguration("params_file"),
                    "autostart": LaunchConfiguration("autostart"),
                }.items(),
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
