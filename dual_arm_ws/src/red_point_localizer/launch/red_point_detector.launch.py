"""Launch the aligned-RGBD color point detector."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Create the detector launch description."""
    defaults = {
        'image_topic': '/camera/color/image_raw',
        'depth_topic': '/camera/depth/image_raw',
        'camera_info_topic': '/camera/color/camera_info',
        'debug_topic': '/red_point/debug_image',
        'pixel_topic': '/red_point/pixel',
        'camera_point_topic': '/red_point/camera_point',
        'color_mode': 'red',
        'min_area': '100.0',
        'depth_window_size': '7',
        'min_valid_depth_count': '5',
        'min_depth_m': '0.10',
        'max_depth_m': '2.50',
        'sync_slop_sec': '0.08',
        'depth_scale_16u': '0.001',
        'sample_csv_path': '',
        'debug_image_path': '',
    }
    arguments = [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in defaults.items()
    ]
    float_parameters = {
        'min_area', 'min_depth_m', 'max_depth_m', 'sync_slop_sec',
        'depth_scale_16u',
    }
    integer_parameters = {'depth_window_size', 'min_valid_depth_count'}
    parameters = {}
    for name in defaults:
        value = LaunchConfiguration(name)
        if name in float_parameters:
            value = ParameterValue(value, value_type=float)
        elif name in integer_parameters:
            value = ParameterValue(value, value_type=int)
        parameters[name] = value

    detector = Node(
        package='red_point_localizer',
        executable='red_point_detector',
        name='red_point_detector',
        output='screen',
        parameters=[parameters],
    )
    return LaunchDescription(arguments + [detector])
