"""Run isolated red, green, and cyan RGB-D detectors for the sorting demo."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Create three detector instances with non-colliding names and outputs."""
    arguments = [
        DeclareLaunchArgument("image_topic", default_value="/camera/color/image_raw"),
        DeclareLaunchArgument("depth_topic", default_value="/camera/depth/image_raw"),
        DeclareLaunchArgument(
            "camera_info_topic", default_value="/camera/color/camera_info"
        ),
        DeclareLaunchArgument("min_area", default_value="500.0"),
    ]
    common = {
        "image_topic": LaunchConfiguration("image_topic"),
        "depth_topic": LaunchConfiguration("depth_topic"),
        "camera_info_topic": LaunchConfiguration("camera_info_topic"),
        "min_area": ParameterValue(LaunchConfiguration("min_area"), value_type=float),
        "depth_window_size": 7,
        "min_valid_depth_count": 5,
        "min_depth_m": 0.10,
        "max_depth_m": 2.50,
        "sync_slop_sec": 0.08,
        "depth_scale_16u": 0.001,
    }
    detectors = []
    for color in ("red", "green", "cyan"):
        parameters = dict(common)
        parameters.update(
            {
                "color_mode": color,
                "debug_topic": f"/demo/color/{color}/debug_image",
                "pixel_topic": f"/demo/color/{color}/pixel",
                "camera_point_topic": f"/demo/color/{color}/camera_point",
            }
        )
        detectors.append(
            Node(
                package="red_point_localizer",
                executable="red_point_detector",
                name=f"{color}_target_detector",
                output="screen",
                parameters=[parameters],
            )
        )
    return LaunchDescription(arguments + detectors)
