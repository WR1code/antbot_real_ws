"""Run reBotArm and Piper-H concurrently and select Xbox ownership in RViz."""

import os
from importlib.machinery import SourceFileLoader

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessIO
from launch.events import Shutdown
from launch.logging import get_logger
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
from rebotarm_pulse.pressure_serial_owner import serial_device, serial_owner_pids


def generate_launch_description():
    gravity_config = os.path.join(
        get_package_share_directory("piperh_control"), "config", "gravity_compensation.yaml"
    )
    if not os.path.isfile(gravity_config):
        raise FileNotFoundError(f"Missing installed gravity configuration: {gravity_config}; rebuild this workspace")
    return LaunchDescription(
        [
            DeclareLaunchArgument("rebot_model", default_value="dm"),
            DeclareLaunchArgument("rebot_channel", default_value="/dev/rebot_can"),
            DeclareLaunchArgument("piper_channel", default_value="can0"),
            DeclareLaunchArgument("piper_driver_speed_percent", default_value="25"),
            DeclareLaunchArgument(
                "piper_playback_max_velocity_rad_s", default_value="0.40"
            ),
            DeclareLaunchArgument(
                "piper_playback_max_acceleration_rad_s2", default_value="1.0"
            ),
            DeclareLaunchArgument(
                "piper_gravity_config_file", default_value=gravity_config
            ),
            DeclareLaunchArgument("joy_device", default_value="/dev/input/js0"),
            DeclareLaunchArgument("initial_robot", default_value="rebotarm"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_teach", default_value="true"),
            DeclareLaunchArgument("use_forbidden_zones", default_value="true"),
            DeclareLaunchArgument("use_piper_pulse_observer", default_value="true"),
            DeclareLaunchArgument("piper_pulse_serial_port", default_value=""),
            DeclareLaunchArgument("use_chassis", default_value="true"),
            DeclareLaunchArgument(
                "use_mapping", default_value="false",
                description=(
                    "Enable RViz-controlled MID360S FAST-LIO mode. The sensor "
                    "and LIO child starts only when the RViz button is pressed; "
                    "legacy world/map parents of the mobile base are suppressed."
                ),
            ),
            DeclareLaunchArgument("mapping_front_ip", default_value="192.168.1.116"),
            DeclareLaunchArgument("mapping_rear_ip", default_value="192.168.1.139"),
            DeclareLaunchArgument("mapping_lidar_interface", default_value="eno1"),
            DeclareLaunchArgument(
                "publish_world_to_base", default_value="true",
                description=(
                    "Keep the legacy static world->base_link for the workbench; "
                    "set false in mapping mode so odom->base_link has one parent"
                ),
            ),
            DeclareLaunchArgument("chassis_port", default_value=os.environ.get(
                "RS00_UART_PORT",
                "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5CE6063665-if00",
            )),
            DeclareLaunchArgument("chassis_max_linear_speed", default_value="1.50"),
            DeclareLaunchArgument("chassis_teleop_mode", default_value="xbox"),
            DeclareLaunchArgument(
                "chassis_map_dir",
                default_value=os.path.join(os.environ.get("ANTBOT_REAL_WS", "."), "artifacts/maps/home_01"),
            ),
            DeclareLaunchArgument("rebot_x", default_value="0.0"),
            DeclareLaunchArgument("rebot_y", default_value="-0.20"),
            DeclareLaunchArgument("rebot_z", default_value="1.286"),
            # Measured from the Piper base flange in the production CAD and
            # expressed in ANTBot base_link. The flange position is unchanged
            # after rotating the physical arm to the official right-side mount.
            DeclareLaunchArgument("piper_x", default_value="-0.0025"),
            DeclareLaunchArgument("piper_y", default_value="-0.1350"),
            DeclareLaunchArgument("piper_z", default_value="0.9543"),
            # Right-side mount with the rear cable facing backward. The previous
            # roll=+90, pitch=+90 pose represented the connector-up rotation;
            # removing that extra base-axis turn keeps the flange normal and
            # moves the connector side from up to the robot's rear.
            DeclareLaunchArgument("piper_roll", default_value="1.57079632679"),
            DeclareLaunchArgument("piper_pitch", default_value="0.0"),
            DeclareLaunchArgument("piper_yaw", default_value="0.0"),
            OpaqueFunction(function=_setup),
        ]
    )


def _mapping(package):
    path = os.path.join(get_package_share_directory(package), "config", "xbox_mapping.yaml")
    with open(path, encoding="utf-8") as stream:
        return yaml.safe_load(stream)["rebot_xbox_twist"]["ros__parameters"]


def _moveit_config(robot, model="", mappings=None):
    if robot == "piperh":
        return (
            MoveItConfigsBuilder("piperh", package_name="piperh_moveit_config")
            .robot_description(
                file_path="config/piperh.urdf.xacro", mappings=mappings
            )
            .robot_description_semantic(file_path="config/piperh.srdf")
            .robot_description_kinematics(file_path="config/kinematics.yaml")
            .joint_limits(file_path="config/joint_limits.yaml")
            .trajectory_execution(file_path="config/moveit_controllers.yaml")
            .planning_pipelines(pipelines=["ompl"])
            .to_moveit_configs()
        )
    suffix = "_rs" if model == "rs" else ""
    return (
        MoveItConfigsBuilder("rebotarm", package_name="rebotarm_moveit_config")
        .robot_description(file_path=f"config/rebotarm{suffix}.urdf.xacro")
        .robot_description_semantic(file_path=f"config/rebotarm{suffix}.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_hardware_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )


def _xbox(robot, mapping, frames):
    values = dict(mapping)
    values.update(
        {
            "robot_name": robot,
            "frames.base": frames[0],
            "frames.end_effector": frames[1],
            "topics.joy": "/joy",
            "topics.twist": f"/{robot}/servo_node/delta_twist_cmds",
            "topics.armed": f"/{robot}/xbox/armed",
            "topics.armed_service": f"/{robot}/xbox/set_armed",
            "topics.gripper_open": f"/{robot}/xbox/gripper_open",
            "topics.gripper_close": f"/{robot}/xbox/gripper_close",
            # ANTBot owns the system-wide base/arm selector. Both arm drivers
            # listen to the same transient-local topic and still apply their
            # independent selected-arm and armed safety gates.
            "topics.control_target": "/xbox/control_target",
            "topics.preset_service": f"/{robot}/xbox/go_to_preset",
            "topics.servo_switch_service": f"/{robot}/servo_node/switch_command_type",
            "topics.servo_pause_service": f"/{robot}/servo_node/pause_servo",
            "topics.selected_robot": "/dual_arm/selected",
        }
    )
    return Node(
        package="rebot_xbox_servo",
        executable="rebot_xbox_twist",
        namespace=robot,
        name="rebot_xbox_twist",
        output="screen",
        parameters=[values],
    )


def _zone_node(robot, model):
    share = get_package_share_directory("rebot_xbox_hardware")
    return Node(
        package="rebot_xbox_hardware",
        executable="forbidden_zone_manager",
        namespace=robot,
        name="forbidden_zone_manager",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_forbidden_zones")),
        parameters=[
            {
                "config_file": os.path.join(share, "config", "forbidden_zones.yaml"),
                "user_config_file": os.path.join(os.environ.get("ROBOT_ZONE_ROOT", os.path.expanduser("~/.ros")), robot, "forbidden_zones_user.yaml"),
                "model": model,
                "move_group_namespace": robot,
                "interactive_namespace": f"/{robot}/forbidden_zones/interactive",
            }
        ],
    )


def _setup(context, *args, **kwargs):
    del args, kwargs
    model = LaunchConfiguration("rebot_model").perform(context).strip().lower()
    # Resolve this before visiting the ANTBot includes.  Their nested waypoint
    # launch also uses a ``use_rviz`` launch configuration and sets it false to
    # suppress its own RViz.  Keeping our condition lazy lets that child value
    # leak into this node and silently suppresses the unified workbench.
    use_rviz = LaunchConfiguration("use_rviz").perform(context)
    use_piper_pulse_observer = LaunchConfiguration(
        "use_piper_pulse_observer"
    ).perform(context)
    if model not in ("dm", "rs"):
        raise ValueError("rebot_model must be dm or rs")
    rebot_channel = LaunchConfiguration("rebot_channel").perform(context).strip()
    piper_channel = LaunchConfiguration("piper_channel").perform(context).strip()
    piper_pulse_serial_port = LaunchConfiguration(
        "piper_pulse_serial_port"
    ).perform(context).strip()
    if not rebot_channel or not piper_channel:
        raise ValueError("both CAN channels must be provided")
    if model == "rs" and rebot_channel == piper_channel:
        raise ValueError("reBotArm and Piper-H must use different CAN interfaces")

    rebot = _moveit_config("rebotarm", model)
    piper_mount = {
        "root_frame": "piperh_planning_world",
        "mount_x": LaunchConfiguration("piper_x").perform(context),
        "mount_y": LaunchConfiguration("piper_y").perform(context),
        "mount_z": LaunchConfiguration("piper_z").perform(context),
        "mount_roll": LaunchConfiguration("piper_roll").perform(context),
        "mount_pitch": LaunchConfiguration("piper_pitch").perform(context),
        "mount_yaw": LaunchConfiguration("piper_yaw").perform(context),
    }
    piper_moveit = _moveit_config("piperh", mappings=piper_mount)
    piper_rviz_model_params = {
        "piperh_robot_description_kinematics":
            piper_moveit.robot_description_kinematics["robot_description_kinematics"],
        "piperh_robot_description_planning":
            piper_moveit.joint_limits["robot_description_planning"],
    }
    moveit_common = SourceFileLoader(
        "dual_arm_moveit_launch_common",
        os.path.join(
            get_package_share_directory("rebotarm_moveit_config"),
            "launch",
            "moveit_launch_common.py",
        ),
    ).load_module()
    rebot_moveit_params = moveit_common.moveit_parameters(rebot)

    with open(
        os.path.join(
            get_package_share_directory("rebot_xbox_servo"),
            "config",
            "servo_config.yaml",
        ),
        encoding="utf-8",
    ) as stream:
        rebot_servo = yaml.safe_load(stream)
    rebot_servo["joint_topic"] = "/rebotarm/joint_states"
    rebot_servo["command_out_topic"] = "/rebotarm/xbox_servo/joint_trajectory"

    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rebotarm_bringup"),
                "launch",
                "driver.launch.py",
            )
        ),
        launch_arguments={
            "model": model,
            "channel": rebot_channel,
            "arm_namespace": "rebotarm",
            "servo_joint_trajectory_topic": "/rebotarm/xbox_servo/joint_trajectory",
            "servo_command_timeout": "0.15",
        }.items(),
    )
    piper = GroupAction(
        [
            PushRosNamespace("piperh"),
            SetRemap(src="/joint_states", dst="/piperh/joint_states"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("piperh_control"),
                        "launch",
                        "hardware.launch.py",
                    )
                ),
                launch_arguments={
                    "can_port": piper_channel,
                    "driver_speed_percent": LaunchConfiguration(
                        "piper_driver_speed_percent"
                    ),
                    "playback_max_velocity_rad_s": LaunchConfiguration(
                        "piper_playback_max_velocity_rad_s"
                    ),
                    "playback_max_acceleration_rad_s2": LaunchConfiguration(
                        "piper_playback_max_acceleration_rad_s2"
                    ),
                    "gravity_config_file": LaunchConfiguration("piper_gravity_config_file"),
                    "gravity_require_selected_robot": "true",
                    "use_xbox": "false",
                    "use_servo": "true",
                    "use_rviz": "false",
                    "joint_states_topic": "/piperh/joint_states",
                    "trajectory_action": "/piperh/arm_controller/follow_joint_trajectory",
                    "tf_prefix": "piperh/",
                    "root_frame": "piperh_planning_world",
                    "mount_x": LaunchConfiguration("piper_x"),
                    "mount_y": LaunchConfiguration("piper_y"),
                    "mount_z": LaunchConfiguration("piper_z"),
                    "mount_roll": LaunchConfiguration("piper_roll"),
                    "mount_pitch": LaunchConfiguration("piper_pitch"),
                    "mount_yaw": LaunchConfiguration("piper_yaw"),
                }.items(),
            ),
        ]
    )
    teach = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rebot_teach_mode"),
                "launch",
                "teach_mode.launch.py",
            )
        ),
        condition=IfCondition(LaunchConfiguration("use_teach")),
        launch_arguments={
            "model": model,
            "arm_namespace": "rebotarm",
            "moveit_namespace": "rebotarm",
            "visualization_frame_prefix": "rebotarm",
            "xbox_armed_topic": "/rebotarm/xbox/armed",
            "allow_hardware": "true",
            "use_xbox": "false",
            "require_xbox_locked": "true",
        }.items(),
    )
    piper_teach = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rebot_teach_mode"),
                "launch",
                "teach_mode.launch.py",
            )
        ),
        condition=IfCondition(LaunchConfiguration("use_teach")),
        launch_arguments={
            "model": "piperh",
            "arm_namespace": "piperh",
            "moveit_namespace": "piperh",
            "visualization_frame_prefix": "piperh",
            "xbox_armed_topic": "/piperh/xbox/armed",
            "allow_hardware": "true",
            "use_xbox": "false",
            "require_xbox_locked": "true",
        }.items(),
    )

    chassis_map_dir = LaunchConfiguration("chassis_map_dir")
    chassis_operator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("antbot_real_bringup"),
                "launch",
                "operator_step2.launch.py",
            )
        ),
        condition=IfCondition(LaunchConfiguration("use_chassis")),
        launch_arguments={
            "port": LaunchConfiguration("chassis_port"),
            # dual_arm_joy is the one physical /joy owner for the whole robot.
            "start_joy": "false",
            "joy_device": LaunchConfiguration("joy_device"),
            "max_linear_speed": LaunchConfiguration("chassis_max_linear_speed"),
            "default_teleop_mode": LaunchConfiguration("chassis_teleop_mode"),
            "mapping_output_prefix": [chassis_map_dir, "/mapping_runs/current/map"],
            "mapping_backend": PythonExpression([
                "'fast_lio' if '", LaunchConfiguration("use_mapping"),
                "' == 'true' else 'slam_toolbox'",
            ]),
            "mapping_front_ip": LaunchConfiguration("mapping_front_ip"),
            "mapping_rear_ip": LaunchConfiguration("mapping_rear_ip"),
            "mapping_lidar_interface": LaunchConfiguration(
                "mapping_lidar_interface"
            ),
            "manage_mapping_network": LaunchConfiguration("use_mapping"),
            "publish_mapping_placeholder_pose": LaunchConfiguration("use_mapping"),
            "map": [chassis_map_dir, "/map.yaml"],
            "waypoints_file": [chassis_map_dir, "/waypoints.xml"],
            "keepout_file": [chassis_map_dir, "/keepouts.keepout.json"],
            "speed_zone_file": [chassis_map_dir, "/speeds.speed.json"],
            "stuck_history_file": [chassis_map_dir, "/stuck_history.json"],
            "pointcloud_path": [chassis_map_dir, "/cloud.pcd"],
            "pointcloud_metadata": [chassis_map_dir, "/cloud_metadata.yaml"],
            "rgbd_preview_path": [chassis_map_dir, "/rgbd_preview.npz"],
            "show_3d_cloud": "true",
            "show_rgbd_cloud": "true",
            "publish_placeholder_pose": "false",
            "use_operator_rviz": "false",
            # This parent already owns the pressure bridge and HTTP gateway.
            "start_pulse": "false",
        }.items(),
    )
    rviz_config = os.path.join(
        get_package_share_directory("rebot_xbox_hardware"), "config", "dual_arm.rviz"
    )
    unicode_preload = os.path.join(
        get_package_prefix("robotcar_navigation"),
        "lib",
        "librobotcar_rviz_unicode.so",
    )
    inherited_preload = os.environ.get("LD_PRELOAD", "")
    rviz_preload = ":".join(
        item for item in (unicode_preload, inherited_preload) if item
    )
    pulse_geometry_config = os.path.join(
        get_package_share_directory("rebotarm_pulse"),
        "config",
        "piper_tool_geometry.yaml",
    )
    pulse_target_config = os.path.join(
        get_package_share_directory("rebotarm_pulse"),
        "config",
        "piper_pulse_target.yaml",
    )
    pulse_nodes = [
        Node(
            package="rebotarm_pulse",
            executable="piper_tool_geometry",
            name="piper_tool_geometry",
            output="screen",
            parameters=[pulse_geometry_config],
        ),
        Node(
            package="rebotarm_pulse",
            executable="piper_pulse_target",
            name="piper_pulse_target",
            output="screen",
            parameters=[pulse_target_config],
            condition=IfCondition(use_piper_pulse_observer),
        ),
    ]
    if piper_pulse_serial_port:
        pulse_zero_config = os.path.join(
            get_package_share_directory("rebotarm_pulse"),
            "config",
            "piper_pressure_zero.yaml",
        )
        owners = serial_owner_pids(piper_pulse_serial_port)
        same_as_arm_port = (
            serial_device(piper_pulse_serial_port) == serial_device(rebot_channel)
        )
        if owners or same_as_arm_port:
            issue = (
                f"serial owner PID(s)={owners}" if owners else
                f"same device as rebot_channel={rebot_channel}"
            )
            get_logger("dual_arm_hardware").error(
                "Refusing duplicate pressure_serial_bridge for "
                f"{piper_pulse_serial_port} ({serial_device(piper_pulse_serial_port)}); "
                f"{issue}. Other hardware nodes remain available."
            )
        else:
            pulse_nodes.append(
                Node(
                    package="rebotarm_pulse",
                    executable="pressure_serial_bridge",
                    name="pressure_serial_bridge",
                    output="screen",
                    parameters=[pulse_zero_config, {"port": piper_pulse_serial_port}],
                )
            )
        pulse_nodes.append(
            Node(
                package="rebotarm_pulse",
                executable="pulse_web_gateway",
                name="pulse_web_gateway",
                output="screen",
            )
        )
    stack = [
        driver,
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace="rebotarm",
            name="robot_state_publisher",
            output="both",
            parameters=[rebot.robot_description, {"frame_prefix": "rebotarm/"}],
            remappings=[("/joint_states", "/rebotarm/joint_states")],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="rebotarm_world_tf",
            arguments=[
                LaunchConfiguration("rebot_x"), LaunchConfiguration("rebot_y"),
                LaunchConfiguration("rebot_z"), "0", "0", "0",
                "base_link", "rebotarm/base_link",
            ],
        ),
        # MoveIt's internal reBotArm planning frame remains base_link; this
        # alias lets its interactive markers coexist with the prefixed live TF.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="rebotarm_moveit_world_tf",
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration("publish_world_to_base"), "' == 'true' and '",
                LaunchConfiguration("use_mapping"), "' != 'true'",
            ])),
            arguments=["0", LaunchConfiguration("rebot_y"), "0", "0", "0", "0", "world", "base_link"],
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            namespace="rebotarm",
            name="move_group",
            output="screen",
            parameters=[rebot_moveit_params],
            remappings=[
                ("/joint_states", "/rebotarm/joint_states"),
                ("/rebotarm/rebotarm/follow_joint_trajectory", "/rebotarm/follow_joint_trajectory"),
            ],
        ),
        Node(
            package="moveit_servo",
            executable="servo_node",
            namespace="rebotarm",
            name="servo_node",
            output="screen",
            # MoveIt Servo can remain in its robot-state wait loop after ROS
            # shutdown. Do not make the whole workbench wait 15 seconds for it.
            sigterm_timeout="2.0",
            sigkill_timeout="2.0",
            parameters=[
                {"moveit_servo": rebot_servo},
                {"update_period": 0.01, "planning_group_name": "arm"},
                rebot.robot_description,
                rebot.robot_description_semantic,
                rebot.robot_description_kinematics,
                rebot.joint_limits,
            ],
        ),
        piper,
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="piperh_world_tf",
            arguments=[
                "0", "0", "0", "0", "0", "0",
                "base_link", "piperh/piperh_planning_world",
            ],
        ),
        # The unprefixed planning model and the prefixed live model share this
        # chassis attachment. The mount pose itself lives in world_to_base in
        # both robot descriptions, so orange goals overlay the physical arm.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="piperh_moveit_mount_tf",
            arguments=[
                "0", "0", "0", "0", "0", "0",
                "base_link", "piperh_planning_world",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="integrated_world_to_map_tf",
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration("use_chassis"), "' == 'true' and '",
                LaunchConfiguration("use_mapping"), "' != 'true'",
            ])),
            arguments=["0", "0", "0", "0", "0", "0", "world", "map"],
        ),
        Node(
            package="joy_linux",
            executable="joy_linux_node",
            name="dual_arm_joy",
            output="screen",
            parameters=[
                {
                    "dev": LaunchConfiguration("joy_device"),
                    "deadzone": 0.05,
                    "autorepeat_rate": 20.0,
                }
            ],
        ),
        _xbox("rebotarm", _mapping("rebot_xbox_servo"), ("base_link", "gripper_tcp")),
        _xbox("piperh", _mapping("piperh_control"), ("base_link", "Link6")),
        Node(
            package="rebot_xbox_hardware",
            executable="active_arm_manager",
            name="active_arm_manager",
            output="screen",
            parameters=[{"initial_robot": LaunchConfiguration("initial_robot")}],
        ),
        Node(
            package="rebot_xbox_hardware",
            executable="hardware_gripper",
            namespace="rebotarm",
            name="rebot_xbox_hardware_gripper",
            output="screen",
            parameters=[
                {
                    "open_topic": "/rebotarm/xbox/gripper_open",
                    "close_topic": "/rebotarm/xbox/gripper_close",
                    "state_topic": "/rebotarm/gripper/state",
                    "command_topic": "/rebotarm/gripper/cmd/pos_vel",
                    "open_position": 5.0 if model == "rs" else -5.0,
                    "closed_position": 0.0,
                    "maximum_velocity": 2.0,
                    "maximum_closing_torque": 0.80,
                }
            ],
        ),
        _zone_node("rebotarm", model),
        _zone_node("piperh", "piperh"),
        *pulse_nodes,
        teach,
        piper_teach,
        chassis_operator,
        Node(
            package="rviz2",
            executable="rviz2",
            name="dual_arm_rviz",
            output="screen",
            arguments=["-d", rviz_config],
            condition=IfCondition(use_rviz),
            additional_env={
                "LANG": "zh_CN.UTF-8",
                "LANGUAGE": "zh_CN:zh",
                "LC_ALL": "zh_CN.UTF-8",
                "LD_PRELOAD": rviz_preload,
            },
            parameters=[
                rebot_moveit_params,
                {
                    "piperh_robot_description":
                        piper_moveit.robot_description["robot_description"],
                    "piperh_robot_description_semantic":
                        piper_moveit.robot_description_semantic[
                            "robot_description_semantic"
                        ],
                    "motion_preset.initial_robot": LaunchConfiguration("initial_robot"),
                    "motion_preset.rebot_model": model,
                    "motion_preset.dual_arm": True,
                    "motion_preset.piper_driver_speed_percent": ParameterValue(
                        LaunchConfiguration("piper_driver_speed_percent"), value_type=int
                    ),
                    # DemoPanel is the sole owner of MoveIt's panel and embeds
                    # it in 演示与动作 -> 自由运动规划.
                    "rebot_demo.integrate_motion_planning": True,
                },
                piper_rviz_model_params,
            ],
            remappings=[
                ("/joint_states", "/rebotarm/joint_states"),
                ("/rebot_xbox/armed", "/rebotarm/xbox/armed"),
                ("/rebot_xbox/set_armed", "/rebotarm/xbox/set_armed"),
                ("/check_state_validity", "/rebotarm/check_state_validity"),
                ("/execute_trajectory", "/rebotarm/execute_trajectory"),
                ("/display_planned_path", "/rebotarm/display_planned_path"),
                ("/forbidden_zone_manager/reload", "/rebotarm/forbidden_zone_manager/reload"),
                ("/forbidden_zone_manager/configure", "/rebotarm/forbidden_zone_manager/configure"),
                ("/forbidden_zone_manager/status", "/rebotarm/forbidden_zone_manager/status"),
            ],
            respawn=True,
            respawn_delay=2.0,
        ),
    ]
    guard = Node(
        package="rebot_xbox_hardware",
        executable="instance_guard",
        name="dual_arm_single_instance_guard",
        output="screen",
        arguments=["--lock-name", "dual_arm"],
    )

    def start_stack_when_ready(event):
        output = event.text.decode(errors="replace") if isinstance(event.text, bytes) else event.text
        return stack if "single-instance guard ready:" in output else []

    def shutdown_if_guard_exits(event, context):
        del event
        # SIGINT also stops the guard. Emitting another Shutdown while launch
        # is already tearing down can leave the launch event loop alive after
        # every child process has gone.
        if context.is_shutdown:
            return []
        return [EmitEvent(event=Shutdown(reason="dual-arm instance guard exited"))]

    return [
        guard,
        RegisterEventHandler(
            OnProcessIO(target_action=guard, on_stdout=start_stack_when_ready)
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=guard,
                on_exit=shutdown_if_guard_exits,
            )
        ),
    ]
