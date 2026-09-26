"""Start AprilTag tracking and easy_handeye2 in either hand-eye layout."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


TAG_FAMILY_CODE_COUNTS = {
    "16h5": 30,
    "25h9": 35,
    "36h11": 587,
    "Circle21h7": 38,
    "Circle49h12": 65535,
    "Custom48h12": 42211,
    "Standard41h12": 2115,
    "Standard52h13": 48714,
}

CALIBRATION_TYPES = {"eye_on_base", "eye_in_hand"}
CALIBRATION_BACKENDS = {"easy_handeye2", "moveit_calibration"}
TARGET_TYPES = {"single_tag", "a4_4tag_board", "charuco_a4_5x7"}
AUTO_SEQUENCE_BY_TYPE = {
    "eye_on_base": "自动手眼标定_DM_12姿态",
    "eye_in_hand": "自动手眼标定_DM_眼在手上_12姿态",
}
AUTO_ACTION_PREFIX_BY_TYPE = {
    "eye_on_base": "手眼标定姿态_",
    "eye_in_hand": "眼在手上标定姿态_",
}


def _validate_tag_selection(family: str, tag_id: int) -> None:
    if family not in TAG_FAMILY_CODE_COUNTS:
        supported = ", ".join(TAG_FAMILY_CODE_COUNTS)
        raise ValueError(f"unsupported tag_family '{family}'; choose one of: {supported}")
    maximum_id = TAG_FAMILY_CODE_COUNTS[family] - 1
    if not 0 <= tag_id <= maximum_id:
        raise ValueError(
            f"tag_id {tag_id} is outside the valid range 0..{maximum_id} "
            f"for tag_family '{family}'"
        )


def _validate_calibration_type(calibration_type: str) -> None:
    if calibration_type not in CALIBRATION_TYPES:
        supported = ", ".join(sorted(CALIBRATION_TYPES))
        raise ValueError(
            f"unsupported calibration_type '{calibration_type}'; choose one of: "
            f"{supported}"
        )


def _validate_calibration_backend(calibration_backend: str) -> None:
    if calibration_backend not in CALIBRATION_BACKENDS:
        supported = ", ".join(sorted(CALIBRATION_BACKENDS))
        raise ValueError(
            f"unsupported calibration_backend '{calibration_backend}'; choose one of: "
            f"{supported}"
        )


def _validate_target_type(target_type: str, tag_family: str) -> None:
    if target_type not in TARGET_TYPES:
        supported = ", ".join(sorted(TARGET_TYPES))
        raise ValueError(
            f"unsupported target_type '{target_type}'; choose one of: {supported}"
        )
    if target_type == "a4_4tag_board" and tag_family != "36h11":
        raise ValueError("the bundled A4 four-tag board requires tag_family '36h11'")


def _start_calibration(context):
    def value(name: str) -> str:
        return LaunchConfiguration(name).perform(context)

    target_type = value("target_type")
    tag_family = value("tag_family")
    _validate_target_type(target_type, tag_family)
    tag_id = int(value("tag_id"))
    if target_type != "charuco_a4_5x7":
        _validate_tag_selection(tag_family, tag_id)
    calibration_type = value("calibration_type")
    _validate_calibration_type(calibration_type)
    calibration_backend = value("calibration_backend")
    _validate_calibration_backend(calibration_backend)
    tag_size = float(value("tag_size_m"))
    tracking_marker = value("tracking_marker_frame")
    auto_sequence_name = value("auto_sequence_name").strip()
    if not auto_sequence_name:
        auto_sequence_name = AUTO_SEQUENCE_BY_TYPE[calibration_type]
    auto_action_prefix = value("auto_action_prefix").strip()
    if not auto_action_prefix:
        auto_action_prefix = AUTO_ACTION_PREFIX_BY_TYPE[calibration_type]
    camera_info_topic = value("camera_info_topic")
    board_mode = target_type == "a4_4tag_board"
    charuco_mode = target_type == "charuco_a4_5x7"
    detected_ids = [0, 1, 2, 3] if board_mode else [tag_id]
    detected_frames = (
        [f"handeye_board_tag_{tag_id}" for tag_id in detected_ids]
        if board_mode
        else [tracking_marker]
    )
    detected_sizes = [tag_size] * len(detected_ids)
    config = os.path.join(
        get_package_share_directory("rebotarm_pulse"),
        "config",
        "apriltag_handeye.yaml",
    )
    easy_handeye_launch = os.path.join(
        get_package_share_directory("easy_handeye2"),
        "launch",
        "calibrate.launch.py",
    )
    actions = [
        Node(
            package="image_proc",
            executable="rectify_node",
            name="handeye_rectify",
            namespace="rebotarm_handeye",
            output="screen",
            parameters=[
                {
                    "queue_size": 1,
                    "qos_overrides./rebotarm_handeye/image_rect.publisher.reliability":
                        "best_effort",
                    "qos_overrides./rebotarm_handeye/image_rect.publisher.history":
                        "keep_last",
                    "qos_overrides./rebotarm_handeye/image_rect.publisher.depth": 1,
                }
            ],
            remappings=[
                ("image", value("color_topic")),
                ("camera_info", camera_info_topic),
                ("image_rect", "/rebotarm_handeye/image_rect"),
            ],
        ),
    ]
    if charuco_mode:
        actions.append(
            Node(
                package="rebotarm_pulse",
                executable="charuco_board_pose",
                name="charuco_board_pose",
                namespace="rebotarm_handeye",
                output="screen",
                parameters=[
                    {
                        "squares_x": 5,
                        "squares_y": 7,
                        "square_length_m": 0.035,
                        "marker_length_m": 0.026,
                        "minimum_charuco_corners": int(value("charuco_minimum_corners")),
                        "marker_frame": tracking_marker,
                    }
                ],
                remappings=[
                    ("image", "/rebotarm_handeye/image_rect"),
                    ("camera_info", camera_info_topic),
                ],
            )
        )
    else:
        actions.append(
            Node(
                package="apriltag_ros",
                executable="apriltag_node",
                name="handeye_apriltag",
                namespace="rebotarm_handeye",
                output="screen",
                parameters=[
                    config,
                    {
                        "family": tag_family,
                        "tag.ids": detected_ids,
                        "tag.frames": detected_frames,
                        "tag.sizes": detected_sizes,
                    },
                ],
                remappings=[
                    ("image_rect", "/rebotarm_handeye/image_rect"),
                    ("camera_info", camera_info_topic),
                ],
            )
        )
    if board_mode:
        actions.append(
            Node(
                package="rebotarm_pulse",
                executable="apriltag_board_pose",
                name="apriltag_board_pose",
                namespace="rebotarm_handeye",
                output="screen",
                parameters=[
                    {
                        "tag_ids": detected_ids,
                        "tag_size_m": tag_size,
                        "center_spacing_x_m": float(value("board_spacing_x_m")),
                        "center_spacing_y_m": float(value("board_spacing_y_m")),
                        "minimum_visible_tags": int(value("board_minimum_visible_tags")),
                        "marker_frame": tracking_marker,
                    }
                ],
                remappings=[
                    ("detections", "/rebotarm_handeye/detections"),
                    ("camera_info", camera_info_topic),
                ],
            )
        )
    if calibration_backend == "easy_handeye2":
        actions.extend(
            [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(easy_handeye_launch),
                    launch_arguments={
                        "name": value("name"),
                        "calibration_type": calibration_type,
                        "robot_base_frame": value("robot_base_frame"),
                        "robot_effector_frame": value("robot_effector_frame"),
                        "tracking_base_frame": value("tracking_base_frame"),
                        "tracking_marker_frame": tracking_marker,
                        # The camera driver already owns tracking_base_frame.
                        # easy_handeye2's example transform would give that frame
                        # a second parent and make the TF tree ambiguous.
                        "publish_dummy_transform": "false",
                    }.items(),
                ),
                Node(
                    package="rebotarm_pulse",
                    executable="auto_handeye_sequence",
                    name="auto_handeye_sequence",
                    output="screen",
                    condition=IfCondition(LaunchConfiguration("use_auto_sequence")),
                    parameters=[
                        {
                            "sequence_name": auto_sequence_name,
                            "action_name_prefix": auto_action_prefix,
                            "calibration_type": calibration_type,
                            "tracking_marker_frame": tracking_marker,
                            "minimum_samples": int(value("auto_minimum_samples")),
                            "teach_status_topic": value("teach_status_topic"),
                            "joint_state_topic": value("joint_state_topic"),
                            "teach_cancel_service": value("teach_cancel_service"),
                        }
                    ],
                ),
            ]
        )
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("name", default_value="rebotarm_camera"),
            DeclareLaunchArgument(
                "calibration_type",
                default_value="eye_on_base",
                choices=sorted(CALIBRATION_TYPES),
                description="eye_on_base: fixed camera; eye_in_hand: camera on robot",
            ),
            DeclareLaunchArgument(
                "calibration_backend",
                default_value="easy_handeye2",
                choices=sorted(CALIBRATION_BACKENDS),
                description="hand-eye calibration UI and solver backend",
            ),
            DeclareLaunchArgument("robot_base_frame", default_value="base_link"),
            DeclareLaunchArgument("robot_effector_frame", default_value="gripper_tcp"),
            DeclareLaunchArgument(
                "tracking_base_frame", default_value="camera_color_optical_frame"
            ),
            DeclareLaunchArgument("tracking_marker_frame", default_value="marker_frame"),
            DeclareLaunchArgument("tag_family", default_value="36h11"),
            DeclareLaunchArgument("tag_id", default_value="0"),
            DeclareLaunchArgument("tag_size_m", default_value="0.040"),
            DeclareLaunchArgument(
                "target_type",
                default_value="single_tag",
                choices=sorted(TARGET_TYPES),
                description="single AprilTag, four-tag A4 board, or bundled 5x7 ChArUco A4 board",
            ),
            DeclareLaunchArgument("board_spacing_x_m", default_value="0.100"),
            DeclareLaunchArgument("board_spacing_y_m", default_value="0.120"),
            DeclareLaunchArgument("board_minimum_visible_tags", default_value="2"),
            DeclareLaunchArgument("charuco_minimum_corners", default_value="6"),
            DeclareLaunchArgument(
                "auto_sequence_name",
                default_value="",
                description="empty selects the matching built-in sequence",
            ),
            DeclareLaunchArgument(
                "auto_action_prefix",
                default_value="",
                description="empty selects the matching built-in action prefix",
            ),
            DeclareLaunchArgument("auto_minimum_samples", default_value="12"),
            DeclareLaunchArgument(
                "teach_status_topic", default_value="/rebotarm/teach/status"
            ),
            DeclareLaunchArgument("joint_state_topic", default_value="/joint_states"),
            DeclareLaunchArgument(
                "teach_cancel_service", default_value="/rebotarm/teach/cancel"
            ),
            DeclareLaunchArgument("use_auto_sequence", default_value="true"),
            DeclareLaunchArgument("color_topic", default_value="/camera/color/image_raw"),
            DeclareLaunchArgument(
                "camera_info_topic", default_value="/camera/color/camera_info"
            ),
            OpaqueFunction(function=_start_calibration),
        ]
    )
