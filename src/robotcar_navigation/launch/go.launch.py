from pathlib import Path
import uuid

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
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


def _optional_rosbridge(context):
    if LaunchConfiguration("use_rosbridge").perform(context).lower() not in ("true", "1", "yes"):
        return []
    try:
        rosbridge = Path(get_package_share_directory("rosbridge_server"))
    except PackageNotFoundError:
        return []
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(rosbridge / "launch" / "rosbridge_websocket_launch.xml"))
        )
    ]


def generate_launch_description():
    robotcar_navigation = _share("robotcar_navigation")
    robotcar_gazebo = _share("robotcar_gazebo")
    nav2_bringup = Path(get_package_share_directory("nav2_bringup"))

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    rviz_config = LaunchConfiguration("rviz_config")
    world_file = LaunchConfiguration("world_file")
    waypoints_file = LaunchConfiguration("waypoints_file")
    initial_pose_waypoint = LaunchConfiguration("initial_pose_waypoint")
    gz_partition = LaunchConfiguration("gz_partition")
    default_gz_partition = f"robotcar_{uuid.uuid4().hex[:8]}"

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_rosbridge", default_value="false"),
            DeclareLaunchArgument("set_initial_pose", default_value="true"),
            DeclareLaunchArgument("initial_pose_waypoint", default_value="12"),
            DeclareLaunchArgument("gz_partition", default_value=default_gz_partition),
            SetEnvironmentVariable("GZ_PARTITION", gz_partition),
            DeclareLaunchArgument("x_pos", default_value="1.7"),
            DeclareLaunchArgument("y_pos", default_value="1.7"),
            DeclareLaunchArgument("z_pos", default_value="0.001"),
            DeclareLaunchArgument("yaw_pos", default_value="3.14159"),
            DeclareLaunchArgument(
                "world_file",
                default_value=str(robotcar_gazebo / "worlds" / "map_world.world"),
            ),
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
            DeclareLaunchArgument(
                "waypoints_file",
                default_value=str(robotcar_navigation / "config" / "waypoints.xml"),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(robotcar_gazebo / "launch" / "robotcar_slam.launch.py")
                ),
                launch_arguments={
                    "world_file": world_file,
                    "x_pos": LaunchConfiguration("x_pos"),
                    "y_pos": LaunchConfiguration("y_pos"),
                    "z_pos": LaunchConfiguration("z_pos"),
                    "yaw_pos": LaunchConfiguration("yaw_pos"),
                    "use_sim_time": use_sim_time,
                    "gz_partition": gz_partition,
                    "bridge_robot_topics": "false",
                    "start_teleop": "false",
                    "start_slam": "false",
                    "use_rviz": "false",
                }.items(),
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="robotcar_gz_bridge",
                output="screen",
                arguments=[
                    "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                    "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                    "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
                    "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
                    "/camera/rgb/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
                    "/camera/rgb/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(nav2_bringup / "launch" / "bringup_launch.py")),
                launch_arguments={
                    "map": map_file,
                    "use_sim_time": use_sim_time,
                    "params_file": params_file,
                    "autostart": LaunchConfiguration("autostart"),
                }.items(),
            ),
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
            Node(
                package="robotcar_navigation",
                executable="wp_set_pose",
                name="wp_set_pose",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            TimerAction(
                period=8.0,
                actions=[
                    Node(
                        package="robotcar_navigation",
                        executable="set_pose_from_waypoint",
                        name="set_initial_pose_from_waypoint",
                        output="screen",
                        arguments=[initial_pose_waypoint],
                        parameters=[{"use_sim_time": use_sim_time}],
                    )
                ],
                condition=IfCondition(LaunchConfiguration("set_initial_pose")),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                condition=IfCondition(LaunchConfiguration("use_rviz")),
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            ),
            OpaqueFunction(function=_optional_rosbridge),
        ]
    )
