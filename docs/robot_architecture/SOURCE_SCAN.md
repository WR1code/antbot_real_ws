# 源码扫描范围与复查索引

2026-09-15；当前工作树含已有未提交修改。本轮只新增审计文档；没有 source 环境、启动节点、打开设备、使能或运动硬件。

CONFIRMED 表示源码/配置存在，不表示设备已接线或运行正常；LIKELY 表示厂商/用途有间接证据；UNKNOWN / NEED_CONFIRMATION 表示实机、安装或接口尚待确认。

扫描排除 build/install/log/logs、虚拟环境及 .git；保留全部源码包、第三方源码、固件、历史仿真配置。完整文件清单、AST 调用、C++ 调用上下文、launch 实例和入口点见 [source_inventory.json](source_inventory.json)。不把历史仿真接口当作真机证据。没有执行 PDF/Excel 的版面审计，硬件结论来自对应源码配置和文本说明。

## 包清单

| Package | Source | Build type |
|---|---|---|
| red_point_localizer | `dual_arm_ws/src/red_point_localizer/package.xml` | ament_python |
| rebotarm_pulse | `dual_arm_ws/src/rebotarm_pulse/package.xml` | ament_python |
| rviz_visual_tools | `dual_arm_ws/src/rviz_visual_tools/package.xml` | ament_cmake |
| rebot_teach_mode | `dual_arm_ws/src/rebot_teach_mode/package.xml` | ament_python |
| piperh_motion_rviz | `dual_arm_ws/src/piperh_motion_rviz/package.xml` | ament_cmake |
| rebotarm_moveit_config | `dual_arm_ws/src/rebotarm_moveit_config/package.xml` | ament_cmake |
| rebotarm_moveit_demos | `dual_arm_ws/src/rebotarm_moveit_demos/package.xml` | ament_python |
| piper | `dual_arm_ws/src/piper/package.xml` | ament_python |
| graph_msgs | `dual_arm_ws/src/graph_msgs/package.xml` | ament_cmake |
| rebot_teach_msgs | `dual_arm_ws/src/rebot_teach_msgs/package.xml` | ament_cmake |
| moveit_visual_tools | `dual_arm_ws/src/moveit_visual_tools/package.xml` | ament_cmake |
| rebotarm_description | `dual_arm_ws/src/rebotarm_description/package.xml` | ament_cmake |
| piper_h_description | `dual_arm_ws/src/piper_h_description/package.xml` | ament_cmake |
| rebotarm_msgs | `dual_arm_ws/src/rebotarm_msgs/package.xml` | ament_cmake |
| piperh_moveit_config | `dual_arm_ws/src/piperh_moveit_config/package.xml` | ament_cmake |
| rebotarmcontroller | `dual_arm_ws/src/rebotarmcontroller/package.xml` | ament_python |
| piperh_control | `dual_arm_ws/src/piperh_control/package.xml` | ament_python |
| meridian_hand_vision | `dual_arm_ws/src/meridian_hand_vision/package.xml` | ament_python |
| rebotarm_demo_rviz | `dual_arm_ws/src/rebotarm_demo_rviz/package.xml` | ament_cmake |
| rebotarm_bringup | `dual_arm_ws/src/rebotarm_bringup/package.xml` | ament_python |
| rebot_xbox_hardware | `dual_arm_ws/src/rebot_xbox_hardware/package.xml` | ament_python |
| piper_msgs | `dual_arm_ws/src/piper_msgs/package.xml` | ament_cmake |
| rebot_xbox_servo | `dual_arm_ws/src/rebot_xbox_servo/package.xml` | ament_python |
| moveit_calibration_plugins | `dual_arm_ws/src/moveit_calibration/moveit_calibration_plugins/package.xml` | ament_cmake |
| moveit_calibration_gui | `dual_arm_ws/src/moveit_calibration/moveit_calibration_gui/package.xml` | ament_cmake |
| orbbec_description | `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_description/package.xml` | ament_cmake |
| orbbec_camera_msgs | `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera_msgs/package.xml` | ament_cmake |
| orbbec_camera | `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/package.xml` | ament_cmake |
| easy_handeye2_msgs | `dual_arm_ws/src/easy_handeye2/easy_handeye2_msgs/package.xml` | ament_cmake |
| easy_handeye2 | `dual_arm_ws/src/easy_handeye2/easy_handeye2/package.xml` | ament_python |
| antbot_dual_lidar | `src/antbot_dual_lidar/package.xml` | ament_python |
| antbot_imu | `src/antbot_imu/package.xml` | ament_cmake |
| antbot_camera | `src/antbot_camera/package.xml` | ament_cmake |
| rebotarm_pulse | `src/rebotarm_pulse/package.xml` | ament_python |
| antbot_real_bringup | `src/antbot_real_bringup/package.xml` | ament_cmake |
| robotcar_navigation | `src/robotcar_navigation/package.xml` | ament_cmake |
| vanjee_lidar_msg | `src/vanjee_lidar_msg/package.xml` | ament_cmake |
| antbot_lidar_fusion | `src/antbot_lidar_fusion/package.xml` | ament_python |
| antbot_libs | `src/antbot_libs/package.xml` | ament_cmake |
| antbot_teleop | `src/antbot_teleop/package.xml` | ament_python |
| antbot_interfaces | `src/antbot_interfaces/package.xml` | ament_cmake |
| antbot_h743_bridge | `src/antbot_h743_bridge/package.xml` | ament_python |
| antbot_rgbd_dataset | `src/antbot_rgbd_dataset/package.xml` | ament_python |
| vanjee_lidar_sdk | `src/vanjee_lidar_sdk/package.xml` | ament_cmake |
| antbot_navigation | `src/antbot_navigation/package.xml` | ament_cmake |
| antbot_description | `src/antbot_description/package.xml` | ament_cmake |
| vanjee_lidar_msg | `src/vanjee_lidar_sdk/src/vanjee_lidar_msg/package.xml` | None |

## Python 可执行入口

| Package | Executable | Target | Evidence |
|---|---|---|---|
| red_point_localizer | red_point_detector | `red_point_localizer.red_point_detector:main` | `dual_arm_ws/src/red_point_localizer/setup.py:32` |
| rebotarm_pulse | apriltag_board_pose | `rebotarm_pulse.apriltag_board_pose:main` | `dual_arm_ws/src/rebotarm_pulse/setup.py:29` |
| rebotarm_pulse | charuco_board_pose | `rebotarm_pulse.charuco_board_pose:main` | `dual_arm_ws/src/rebotarm_pulse/setup.py:30` |
| rebotarm_pulse | auto_handeye_sequence | `rebotarm_pulse.auto_handeye_sequence:main` | `dual_arm_ws/src/rebotarm_pulse/setup.py:31` |
| rebotarm_pulse | joint_target_move | `rebotarm_pulse.joint_target_move:main` | `dual_arm_ws/src/rebotarm_pulse/setup.py:32` |
| rebotarm_pulse | pulse_approach | `rebotarm_pulse.pulse_approach:main` | `dual_arm_ws/src/rebotarm_pulse/setup.py:33` |
| rebotarm_pulse | pressure_serial_bridge | `rebotarm_pulse.pressure_serial_bridge:main` | `dual_arm_ws/src/rebotarm_pulse/setup.py:34` |
| rebotarm_pulse | pulse_web_gateway | `rebotarm_pulse.pulse_web_gateway:main` | `dual_arm_ws/src/rebotarm_pulse/setup.py:35` |
| rebotarm_pulse | piper_pulse_target | `rebotarm_pulse.piper_pulse_target:main` | `dual_arm_ws/src/rebotarm_pulse/setup.py:36` |
| rebotarm_pulse | piper_pulse_align | `rebotarm_pulse.piper_pulse_align:main` | `dual_arm_ws/src/rebotarm_pulse/setup.py:37` |
| rebotarm_pulse | piper_tool_geometry | `rebotarm_pulse.piper_tool_geometry:main` | `dual_arm_ws/src/rebotarm_pulse/setup.py:38` |
| rebot_teach_mode | teach_mode_node | `rebot_teach_mode.teach_node:main` | `dual_arm_ws/src/rebot_teach_mode/setup.py:39` |
| rebot_teach_mode | teach_xbox_bridge | `rebot_teach_mode.xbox_bridge:main` | `dual_arm_ws/src/rebot_teach_mode/setup.py:40` |
| rebotarm_moveit_demos | pick_place | `rebotarm_moveit_demos.pick_place:main` | `dual_arm_ws/src/rebotarm_moveit_demos/setup.py:25` |
| rebotarm_moveit_demos | draw_square | `rebotarm_moveit_demos.draw_square:main` | `dual_arm_ws/src/rebotarm_moveit_demos/setup.py:26` |
| rebotarm_moveit_demos | go_home | `rebotarm_moveit_demos.go_home:main` | `dual_arm_ws/src/rebotarm_moveit_demos/setup.py:27` |
| piper | piper_single_ctrl | `piper.piper_ctrl_single_node:main` | `dual_arm_ws/src/piper/setup.py:30` |
| piper | piper_read_slave_joint | `piper.piper_read_slave_joint:main` | `dual_arm_ws/src/piper/setup.py:31` |
| rebotarmcontroller | reBotArmController | `rebotarmcontroller.rebotarm_controller:main` | `dual_arm_ws/src/rebotarmcontroller/setup.py:22` |
| rebotarmcontroller | GravityCompensation | `rebotarmcontroller.examples.gravity_compensation:main` | `dual_arm_ws/src/rebotarmcontroller/setup.py:23` |
| rebotarmcontroller | GripperControl | `rebotarmcontroller.examples.gripper_control:main` | `dual_arm_ws/src/rebotarmcontroller/setup.py:24` |
| rebotarmcontroller | MoveTo | `rebotarmcontroller.examples.move_to:main` | `dual_arm_ws/src/rebotarmcontroller/setup.py:25` |
| rebotarmcontroller | MoveToPose | `rebotarmcontroller.examples.move_to_pose:main` | `dual_arm_ws/src/rebotarmcontroller/setup.py:26` |
| piperh_control | joint_state_udp_bridge | `piperh_control.joint_state_udp_bridge:main` | `dual_arm_ws/src/piperh_control/setup.py:22` |
| piperh_control | hardware_adapter | `piperh_control.hardware_adapter:main` | `dual_arm_ws/src/piperh_control/setup.py:23` |
| piperh_control | gravity_audit | `piperh_control.gravity_audit:main` | `dual_arm_ws/src/piperh_control/setup.py:24` |
| meridian_hand_vision | hand_depth_viewer | `meridian_hand_vision.hand_depth_viewer:main` | `dual_arm_ws/src/meridian_hand_vision/setup.py:34` |
| rebot_xbox_hardware | arm_selector | `rebot_xbox_hardware.arm_selector:main` | `dual_arm_ws/src/rebot_xbox_hardware/setup.py:28` |
| rebot_xbox_hardware | active_arm_manager | `rebot_xbox_hardware.active_arm_manager:main` | `dual_arm_ws/src/rebot_xbox_hardware/setup.py:29` |
| rebot_xbox_hardware | hardware_gripper | `rebot_xbox_hardware.hardware_gripper:main` | `dual_arm_ws/src/rebot_xbox_hardware/setup.py:30` |
| rebot_xbox_hardware | instance_guard | `rebot_xbox_hardware.instance_guard:main` | `dual_arm_ws/src/rebot_xbox_hardware/setup.py:31` |
| rebot_xbox_hardware | forbidden_zone_manager | `rebot_xbox_hardware.forbidden_zone_manager:main` | `dual_arm_ws/src/rebot_xbox_hardware/setup.py:32` |
| rebot_xbox_servo | rebot_xbox_twist | `rebot_xbox_servo.xbox_twist:main` | `dual_arm_ws/src/rebot_xbox_servo/setup.py:35` |
| rebot_xbox_servo | inspect_joy | `rebot_xbox_servo.inspect_joy:main` | `dual_arm_ws/src/rebot_xbox_servo/setup.py:36` |
| rebot_xbox_servo | sim_gripper | `rebot_xbox_servo.sim_gripper:main` | `dual_arm_ws/src/rebot_xbox_servo/setup.py:37` |
| rebot_xbox_servo | arm_initializer | `rebot_xbox_servo.arm_initializer:main` | `dual_arm_ws/src/rebot_xbox_servo/setup.py:38` |
| easy_handeye2 | handeye_server | `easy_handeye2.handeye_server:main` | `dual_arm_ws/src/easy_handeye2/easy_handeye2/setup.py:28` |
| easy_handeye2 | handeye_server_robot | `easy_handeye2.handeye_server_robot:main` | `dual_arm_ws/src/easy_handeye2/easy_handeye2/setup.py:29` |
| easy_handeye2 | handeye_publisher | `easy_handeye2.handeye_publisher:main` | `dual_arm_ws/src/easy_handeye2/easy_handeye2/setup.py:30` |
| easy_handeye2 | handeye_calibration_commander | `easy_handeye2.handeye_calibration_commander:main` | `dual_arm_ws/src/easy_handeye2/easy_handeye2/setup.py:31` |
| antbot_dual_lidar | cloud_preprocessor | `antbot_dual_lidar.cloud_preprocessor:main` | `src/antbot_dual_lidar/setup.py:32` |
| antbot_dual_lidar | livox_frame_relay | `antbot_dual_lidar.livox_frame_relay:main` | `src/antbot_dual_lidar/setup.py:33` |
| antbot_dual_lidar | dual_lidar_diagnostics | `antbot_dual_lidar.diagnostics:main` | `src/antbot_dual_lidar/setup.py:34` |
| antbot_dual_lidar | dual_cloud_synchronizer | `antbot_dual_lidar.synchronizer:main` | `src/antbot_dual_lidar/setup.py:35` |
| antbot_dual_lidar | performance_probe | `antbot_dual_lidar.performance_probe:main` | `src/antbot_dual_lidar/setup.py:36` |
| antbot_dual_lidar | coverage_probe | `antbot_dual_lidar.coverage_probe:main` | `src/antbot_dual_lidar/setup.py:37` |
| antbot_dual_lidar | ground_truth_deskew_validator | `antbot_dual_lidar.ground_truth_deskew_validator:main` | `src/antbot_dual_lidar/setup.py:38` |
| antbot_dual_lidar | validate_lio_dataset | `antbot_dual_lidar.dataset_validation:main` | `src/antbot_dual_lidar/setup.py:39` |
| antbot_dual_lidar | phase2a_motion_experiment | `antbot_dual_lidar.phase2a_motion_experiment:main` | `src/antbot_dual_lidar/setup.py:40` |
| antbot_dual_lidar | phase2a_analyze_bag | `antbot_dual_lidar.phase2a_bag_analysis:main` | `src/antbot_dual_lidar/setup.py:41` |
| antbot_dual_lidar | phase2a_minimal_manifest | `antbot_dual_lidar.phase2a_minimal_manifest:main` | `src/antbot_dual_lidar/setup.py:42` |
| antbot_dual_lidar | fixed_frame_mapper | `antbot_dual_lidar.fixed_frame_mapper:main` | `src/antbot_dual_lidar/setup.py:43` |
| antbot_dual_lidar | save_pointcloud | `antbot_dual_lidar.save_pointcloud:main` | `src/antbot_dual_lidar/setup.py:44` |
| antbot_dual_lidar | offline_pointcloud_publisher | `antbot_dual_lidar.offline_pointcloud_publisher:main` | `src/antbot_dual_lidar/setup.py:45` |
| antbot_dual_lidar | create_map_bundle | `antbot_dual_lidar.create_map_bundle:main` | `src/antbot_dual_lidar/setup.py:46` |
| antbot_dual_lidar | dynamic_obstacle_monitor | `antbot_dual_lidar.dynamic_obstacle_monitor:main` | `src/antbot_dual_lidar/setup.py:47` |
| rebotarm_pulse | apriltag_board_pose | `rebotarm_pulse.apriltag_board_pose:main` | `src/rebotarm_pulse/setup.py:29` |
| rebotarm_pulse | charuco_board_pose | `rebotarm_pulse.charuco_board_pose:main` | `src/rebotarm_pulse/setup.py:30` |
| rebotarm_pulse | auto_handeye_sequence | `rebotarm_pulse.auto_handeye_sequence:main` | `src/rebotarm_pulse/setup.py:31` |
| rebotarm_pulse | joint_target_move | `rebotarm_pulse.joint_target_move:main` | `src/rebotarm_pulse/setup.py:32` |
| rebotarm_pulse | pulse_approach | `rebotarm_pulse.pulse_approach:main` | `src/rebotarm_pulse/setup.py:33` |
| rebotarm_pulse | pressure_serial_bridge | `rebotarm_pulse.pressure_serial_bridge:main` | `src/rebotarm_pulse/setup.py:34` |
| rebotarm_pulse | pulse_web_gateway | `rebotarm_pulse.pulse_web_gateway:main` | `src/rebotarm_pulse/setup.py:35` |
| rebotarm_pulse | piper_pulse_target | `rebotarm_pulse.piper_pulse_target:main` | `src/rebotarm_pulse/setup.py:36` |
| rebotarm_pulse | piper_pulse_align | `rebotarm_pulse.piper_pulse_align:main` | `src/rebotarm_pulse/setup.py:37` |
| rebotarm_pulse | piper_tool_geometry | `rebotarm_pulse.piper_tool_geometry:main` | `src/rebotarm_pulse/setup.py:38` |
| antbot_lidar_fusion | lidar_fusion_node | `antbot_lidar_fusion.lidar_fusion_node:main` | `src/antbot_lidar_fusion/setup.py:26` |
| antbot_lidar_fusion | topic_check_node | `antbot_lidar_fusion.topic_check_node:main` | `src/antbot_lidar_fusion/setup.py:27` |
| antbot_teleop | teleop_keyboard | `antbot_teleop.teleop_keyboard:main` | `src/antbot_teleop/setup.py:28` |
| antbot_teleop | teleop_smooth | `antbot_teleop.teleop_smooth:main` | `src/antbot_teleop/setup.py:29` |
| antbot_teleop | mapping_keyboard | `antbot_teleop.mapping_keyboard:main` | `src/antbot_teleop/setup.py:30` |
| antbot_teleop | mapping_xbox | `antbot_teleop.mapping_xbox:main` | `src/antbot_teleop/setup.py:31` |
| antbot_teleop | teleop_joystick | `antbot_teleop.teleop_joystick:main` | `src/antbot_teleop/setup.py:32` |
| antbot_teleop | swerve_sim | `antbot_teleop.swerve_sim:main` | `src/antbot_teleop/setup.py:33` |
| antbot_h743_bridge | h743_cmd_vel_bridge | `antbot_h743_bridge.bridge:main` | `src/antbot_h743_bridge/setup.py:23` |
| antbot_h743_bridge | h743_control | `antbot_h743_bridge.control_tool:main` | `src/antbot_h743_bridge/setup.py:24` |
| antbot_h743_bridge | h743_dashboard | `antbot_h743_bridge.chassis_dashboard:main` | `src/antbot_h743_bridge/setup.py:25` |
| antbot_h743_bridge | h743_uart_debug | `antbot_h743_bridge.uart_debug_tool:main` | `src/antbot_h743_bridge/setup.py:26` |
| antbot_h743_bridge | antbot_operator_manager | `antbot_h743_bridge.operator_manager:main` | `src/antbot_h743_bridge/setup.py:27` |
| antbot_rgbd_dataset | rgbd_keyframe_recorder | `antbot_rgbd_dataset.recorder_node:main` | `src/antbot_rgbd_dataset/setup.py:29` |
| antbot_rgbd_dataset | phase4b_capture_node | `antbot_rgbd_dataset.phase4b_capture_node:main` | `src/antbot_rgbd_dataset/setup.py:30` |
| antbot_rgbd_dataset | rebuild_rgbd_preview | `antbot_rgbd_dataset.rebuild_preview:main` | `src/antbot_rgbd_dataset/setup.py:33` |

## 解析限制

[]
补充：本机安装SDK已只读追到Piper编码与CAN发送、MotorBridge native传输选择；详见[SDK及安装证据](SDK_AND_INSTALL_EVIDENCE.md)。供应商内部UNKNOWN仅指Piper控制器电机路由及MotorBridge native/桥固件后的协议，不再指Python SDK发送实现。
