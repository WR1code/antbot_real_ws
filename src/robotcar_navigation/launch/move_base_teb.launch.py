from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _share(package_name: str) -> Path:
    try:
        return Path(get_package_share_directory(package_name))
    except PackageNotFoundError:
        source_pkg = Path(__file__).resolve().parents[2] / package_name
        if source_pkg.exists():
            return source_pkg
        raise


def generate_launch_description():
    pkg = _share("robotcar_navigation")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(pkg / "launch" / "amcl_nav.launch.py")),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
            ),
        ]
    )
