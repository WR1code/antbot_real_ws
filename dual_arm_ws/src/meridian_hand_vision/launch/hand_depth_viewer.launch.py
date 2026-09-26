from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackagePrefix


def generate_launch_description() -> LaunchDescription:
    runner = PathJoinSubstitution(
        [FindPackagePrefix("meridian_hand_vision"), "lib", "meridian_hand_vision", "run_hand_depth_viewer.sh"]
    )
    return LaunchDescription(
        [ExecuteProcess(cmd=[runner], output="screen", sigterm_timeout="5")]
    )
