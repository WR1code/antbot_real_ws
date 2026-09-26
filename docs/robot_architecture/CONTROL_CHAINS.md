# 完整控制链追踪

审计日期：2026-09-15。静态源码审计；CONFIRMED 指实现/配置已发现，不代表真机在线。UNKNOWN / NEED_CONFIRMATION 不作为可执行配置。详见 [扫描范围](SOURCE_SCAN.md)。

## 1. 当前底盘链（不是传统 Nav2→底盘CAN直连）

`/joy → MappingXbox / RViz键盘 → /antbot/cmd_vel/{xbox,keyboard} (Twist) → AntbotOperatorManager.forward_command → /cmd_vel (Twist) → CmdVelUartBridge.on_cmd_vel → encode_cmd_vel → Serial.write → USB-TTL USART1 → HostCmdVelUart → ChassisTranslation → SteeringController + DriveController → MCU FDCAN1 RS00 IDs1..4 / FDCAN2 MINI IDs5..8 → 舵轮`

证据：[`src/antbot_teleop/antbot_teleop/mapping_xbox.py:114`](../../src/antbot_teleop/antbot_teleop/mapping_xbox.py#L114)；[`src/robotcar_navigation/src/vehicle_status_panel.cpp:580`](../../src/robotcar_navigation/src/vehicle_status_panel.cpp#L580)；[`src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py:186`](../../src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py#L186)；[`src/antbot_h743_bridge/antbot_h743_bridge/bridge.py:354`](../../src/antbot_h743_bridge/antbot_h743_bridge/bridge.py#L354)；[`src/antbot_h743_bridge/antbot_h743_bridge/chassis_uart_protocol.py:131`](../../src/antbot_h743_bridge/antbot_h743_bridge/chassis_uart_protocol.py#L131)；[`firmware/rs00_fk743_test/firmware/App/Src/host_cmd_vel_uart.c:246`](../../firmware/rs00_fk743_test/firmware/App/Src/host_cmd_vel_uart.c#L246)；[`firmware/rs00_fk743_test/firmware/App/Src/chassis_translation_controller.c:220`](../../firmware/rs00_fk743_test/firmware/App/Src/chassis_translation_controller.c#L220)；[`firmware/rs00_fk743_test/firmware/App/Src/rs00_stm32_fdcan.c:1`](../../firmware/rs00_fk743_test/firmware/App/Src/rs00_stm32_fdcan.c#L1)。

桥把同时含wz的指令改为原地旋转（vx=vy=0），不是完整同时平移转弯的全向接口。当前操作 launch `start_nav2=false`；导航任务链在 `/odom`/odom TF 缺失处中断。不能用 MCU 单轮状态代替已验证轮式里程计。

## 2. reBotArm 规划/示教链

`RViz MoveIt / Teach / MoveItDemoBase → MoveIt /rebotarm/execute_trajectory 或直接 /rebotarm/follow_joint_trajectory → ArmActions → HardwareManager.begin_trajectory_stream / set_joint_position_target → ReBotArm / endpose controller → JointGroup / motorbridge.Controller → DM串口921600（下层桥CAN）→ motors1..7`

证据：[`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/ros_actions.py:5`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/ros_actions.py#L5)；[`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/hardware_manager.py:423`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/hardware_manager.py#L423)；[`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/hardware_manager.py:244`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/hardware_manager.py#L244)；[`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/hardware_manager.py:784`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/hardware_manager.py#L784)；[`dual_arm_ws/third_party/reBotArm_control_py/reBotArm_control_py/actuator/rebotarm.py:490`](../../dual_arm_ws/third_party/reBotArm_control_py/reBotArm_control_py/actuator/rebotarm.py#L490)。MotorBridge 的实际桥固件/CAN比特率未在本仓库追到：UNKNOWN / NEED_CONFIRMATION。RS型号走 Controller(can channel)→RobStride，不能沿用DM串口结论。

## 3. reBotArm Servo 和低层旁路

`XboxTwist → TwistStamped → moveit_servo → /rebotarm/xbox_servo/joint_trajectory → ServoTrajectoryInput._command → HardwareManager.set_servo_joint_position_target → 同一SDK/串口/电机`

`/rebotarm/joints/{joint}/cmd/mit 或 cmd/pos_vel → MotorPassthrough._can_send_lowlevel → HardwareManager.send_joint_* → JointGroup→MotorBridge`。

证据：[`dual_arm_ws/src/rebot_xbox_hardware/launch/dual_arm_hardware.launch.py:240`](../../dual_arm_ws/src/rebot_xbox_hardware/launch/dual_arm_hardware.launch.py#L240)；[`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/servo_trajectory_input.py:49`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/servo_trajectory_input.py#L49)；[`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/motor_passthrough.py:128`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/motor_passthrough.py#L128)。局部state/RLock是模式和线程互斥，尚不是caller ownership。

## 4. Piper-H 轨迹/Servo链

`MoveIt / Teach / Pulse → /piperh/arm_controller/follow_joint_trajectory → HardwareAdapter._execute_arm_locked → prepare_trajectory + tracking → _publish_driver → /piperh/driver_joint_command (JointState) → PiperRosNode.joint_callback → MotionCtrl_2 + JointCtrl → piper_sdk C_PiperInterface → Linux SocketCAN can0 → Piper控制器/关节`

`Xbox → moveit_servo → /piperh/servo_joint_trajectory → HardwareAdapter._stream → 同一_publish_driver`。

证据：[`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:757`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L757)；[`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:699`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L699)；[`dual_arm_ws/src/piper/piper/piper_ctrl_single_node.py:395`](../../dual_arm_ws/src/piper/piper/piper_ctrl_single_node.py#L395)；[`dual_arm_ws/src/piper/piper/piper_ctrl_single_node.py:83`](../../dual_arm_ws/src/piper/piper/piper_ctrl_single_node.py#L83)。位置rad→厂商毫度；velocity[6]是全局速度百分比，不能按各关节速度解释。反馈 `can0 0x2A5..0x2A7 → TeachCanGrouper → HardwareAdapter._direct_can_feedback_loop → /piperh/joint_states`；只读监听不是第二个控制源。SDK底层电机路由未在仓库直接实现，UNKNOWN / NEED_CONFIRMATION。

## 5. Piper Leader/手拖的直接SDK链

`Teach gravity start → HardwareAdapter._start_gravity_compensation → OfficialMitBackend → PiperSDK Leader角色切换/或MIT → can0 → Piper`；退出执行恢复follower/保持等局部清理。

证据：[`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:1033`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L1033)；[`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:1203`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L1203)；[`dual_arm_ws/src/piperh_control/piperh_control/mit_backend.py:15`](../../dual_arm_ws/src/piperh_control/piperh_control/mit_backend.py#L15)。adapter 内 gravity flags 抑制 position，但 vendor pos_cmd/enable_flag 等ROS旁路仍需封口，不能把多SocketCAN读者误判为多writer。

## 6. 视觉→3D→TF→规划→真实机械臂

`Orbbec USB → orbbec_camera Image/Depth/CameraInfo → HandDepthViewer/RedPointDetector → PointStamped/方向向量 → TF Buffer.lookup_transform → PiperPulseTarget/PiperPulseAlign或PulseApproach → MoveIt IK/GetMotionPlan/GetCartesianPath/ExecuteTrajectory → 上述臂轨迹链`

证据：[`src/antbot_camera/launch/camera.launch.py:62`](../../src/antbot_camera/launch/camera.launch.py#L62)；[`dual_arm_ws/src/meridian_hand_vision/meridian_hand_vision/hand_depth_viewer.py:131`](../../dual_arm_ws/src/meridian_hand_vision/meridian_hand_vision/hand_depth_viewer.py#L131)；[`dual_arm_ws/src/red_point_localizer/red_point_localizer/red_point_detector.py:114`](../../dual_arm_ws/src/red_point_localizer/red_point_localizer/red_point_detector.py#L114)；[`src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py:166`](../../src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py#L166)；[`src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py:1669`](../../src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py#L1669)；[`src/rebotarm_pulse/rebotarm_pulse/pulse_approach.py:188`](../../src/rebotarm_pulse/rebotarm_pulse/pulse_approach.py#L188)。完整话题表达式见矩阵。总入口只启动 pulse observer/工具几何，不等于默认启动相机和人体检测或自动执行把脉；实际手眼文件与光学frame须现场验证。

## 7. 压力安全/网页链

`压力芯片（型号未知）→ ESP32-S3样本文本 → USB串口 → PressureSerialBridge.poll → parse_pressure_line → s1/s2/s3 raw FluidPressure(Pa)/Temperature → zeroed压力 → PiperPulseAlign._pressure_safe / dummy流程 + PulseWebGateway → HTTP8765`

证据：[`src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py:162`](../../src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py#L162)；[`src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py:732`](../../src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py#L732)；[`src/rebotarm_pulse/rebotarm_pulse/pulse_web_gateway.py:1`](../../src/rebotarm_pulse/rebotarm_pulse/pulse_web_gateway.py#L1)。不是已验证完整接触/诊断治疗任务；网页可在线而压力采集被跳过，不能作为READY判据。

## 8. LiDAR/IMU/导航候选链

`Vanjee UDP/TCP → DriverManager/source driver → PointCloud2 / IMU packets`；`真实双2D /scan_0、/scan_1 → LidarFusionNode → 融合scan → SLAM/AMCL候选`；`IMU串口→antbot_libs Communicator→ImuNode calibration/filter→Imu`。

证据：[`src/vanjee_lidar_sdk/node/vanjee_lidar_sdk_node.cpp:1`](../../src/vanjee_lidar_sdk/node/vanjee_lidar_sdk_node.cpp#L1)；[`src/antbot_lidar_fusion/antbot_lidar_fusion/lidar_fusion_node.py:1`](../../src/antbot_lidar_fusion/antbot_lidar_fusion/lidar_fusion_node.py#L1)；[`src/antbot_imu/src/imu_node.cpp:1`](../../src/antbot_imu/src/imu_node.cpp#L1)。`antbot_dual_lidar`的GMO/truth/LIO配置明确含仿真时间和真值字段，不能据其`odom.available=true`宣称H743发布里程计。

## 9. 固件调试/主机脚本旁路

`control_tool / uart_debug_tool / chassis_dashboard / firmware host脚本 → Serial.write → 同H743`，独立打开串口绕过ROS门禁；固件仍有自己的保护。启动脚本/README要求单持有串口，但全入口统一排他锁未确认。路径索引见SOURCE_SCAN，安全评估见CONTROL_OWNERSHIP。

补充：本机安装SDK已只读追到Piper编码与CAN发送、MotorBridge native传输选择；详见[SDK及安装证据](SDK_AND_INSTALL_EVIDENCE.md)。供应商内部UNKNOWN仅指Piper控制器电机路由及MotorBridge native/桥固件后的协议，不再指Python SDK发送实现。
