# ROS Interface Matrix

审计日期：2026-09-15。静态源码审计；CONFIRMED 指实现/配置已发现，不代表真机在线。UNKNOWN / NEED_CONFIRMATION 不作为可执行配置。详见 [扫描范围](SOURCE_SCAN.md)。

这里列出每个 Python Node 类及 ROS helper 对象的所有静态声明；helper 不重复计作节点。C++ 节点/插件的全部调用上下文单列附录。表达式是源码表达式，不伪造动态解析后的名称。实际 launch 的 namespace、参数覆盖和 remap 优先于构造默认值；同包副本、例子、测试、仿真均保留但不声称同时运行。外部 Nav2/MoveIt 接口来自 launch/config，供应商内部接口未逐一运行验证。

## 当前整机关键有效接口

| Node/对象 | 类型 | Interface | Message Type | Direction | 对象 |
|---|---|---|---|---|---|
| antbot_operator_manager | Topic | /antbot/cmd_vel/xbox、/antbot/cmd_vel/keyboard | geometry_msgs/msg/Twist | SUB | 底盘遥控源 |
| antbot_operator_manager | Topic | /cmd_vel | geometry_msgs/msg/Twist | PUB | H743桥 |
| h743_cmd_vel_bridge | Topic | /cmd_vel | geometry_msgs/msg/Twist | SUB | 底盘 |
| h743_cmd_vel_bridge | Topic | /rs00/motor_status、/antbot/vehicle_status | std_msgs/msg/String（JSON） | PUB | 健康 |
| h743_cmd_vel_bridge | Topic | /joint_states、/battery | sensor_msgs/msg/JointState、BatteryState | PUB | 底盘 |
| h743_cmd_vel_bridge | Service | /antbot/operator_enable、/antbot/system_reset | std_srvs/srv/SetBool、Trigger | SERVER | 安全门禁 |
| rebotarm_controller | Action | /rebotarm/follow_joint_trajectory | control_msgs/action/FollowJointTrajectory | SERVER | reBotArm |
| rebotarm_controller | Topic | /rebotarm/xbox_servo/joint_trajectory | trajectory_msgs/msg/JointTrajectory | SUB | reBotArm |
| rebotarm_controller | Topic | /rebotarm/joints/{joint}/cmd/mit、cmd/pos_vel | rebotarm_msgs/msg/JointMitCmd、JointPosVelCmd | SUB | 低层命令 |
| hardware_adapter | Action | /piperh/arm_controller/follow_joint_trajectory | control_msgs/action/FollowJointTrajectory | SERVER | Piper |
| hardware_adapter | Topic | /piperh/servo_joint_trajectory | trajectory_msgs/msg/JointTrajectory | SUB | Piper |
| hardware_adapter → piper_single_ctrl | Topic | /piperh/driver_joint_command | sensor_msgs/msg/JointState | PUB → SUB | Piper最终ROS命令 |
| hardware_adapter | Topic | /piperh/joint_states、/piperh/leader_joint_states、/piperh/arm_status | JointState、JointState、rebotarm_msgs/msg/ArmStatus | PUB | Piper |
| piper_single_ctrl | Service | /piperh/enable_srv | piper_msgs/srv/Enable | SERVER | 厂商使能 |
| active_arm_manager | Topic | /dual_arm/select_request、/dual_arm/selected、/dual_arm/status | std_msgs/msg/String | SUB、PUB、PUB | Xbox选择 |
| active_arm_manager | Topic | /{rebotarm,piperh}/xbox/armed | std_msgs/msg/Bool | SUB | 锁定反馈 |
| perception / pulse | Topic | PointStamped、PoseStamped、Marker（名称见各对象） | geometry_msgs、visualization_msgs | PUB/SUB | 视觉到规划 |
| TF对象 | TF | /tf、/tf_static | tf2_msgs/msg/TFMessage | PUB/SUB | Buffer/TransformBroadcaster/静态TF |
| Nav2（默认关闭） | Topic/Action | /cmd_vel、/odom、navigate_to_pose 等 | Twist、Odometry、nav2_msgs actions | PUB/SUB/SERVER | 缺真机odom |

证据：[`src/antbot_h743_bridge/antbot_h743_bridge/bridge.py:173`](../../src/antbot_h743_bridge/antbot_h743_bridge/bridge.py#L173)；[`src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py:100`](../../src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py#L100)；[`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:141`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L141)；[`dual_arm_ws/src/rebot_xbox_hardware/launch/dual_arm_hardware.launch.py:254`](../../dual_arm_ws/src/rebot_xbox_hardware/launch/dual_arm_hardware.launch.py#L254)。

## 按源码对象的完整 Python 声明

### NON_ROS / CmdVelUartBridge

Node Name：`cmd_vel_uart_bridge`；Executable：见 SOURCE_SCAN 入口和 launch 附录；UNKNOWN / NEED_CONFIRMATION；Source：[`firmware/rs00_fk743_test/host/ros2_cmd_vel_uart.py:1`](../../firmware/rs00_fk743_test/host/ros2_cmd_vel_uart.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'port'` | `resolve_uart_port(),` | DEFAULT | `__init__` | [`firmware/rs00_fk743_test/host/ros2_cmd_vel_uart.py:30`](../../firmware/rs00_fk743_test/host/ros2_cmd_vel_uart.py#L30) |
| Parameter | `'baud'` | `115200` | DEFAULT | `__init__` | [`firmware/rs00_fk743_test/host/ros2_cmd_vel_uart.py:34`](../../firmware/rs00_fk743_test/host/ros2_cmd_vel_uart.py#L34) |
| Parameter | `'topic'` | `"/cmd_vel"` | DEFAULT | `__init__` | [`firmware/rs00_fk743_test/host/ros2_cmd_vel_uart.py:35`](../../firmware/rs00_fk743_test/host/ros2_cmd_vel_uart.py#L35) |
| Parameter | `'max_linear_speed'` | `0.5` | DEFAULT | `__init__` | [`firmware/rs00_fk743_test/host/ros2_cmd_vel_uart.py:36`](../../firmware/rs00_fk743_test/host/ros2_cmd_vel_uart.py#L36) |
| Parameter | `'status_topic'` | `"/rs00/motor_status"` | DEFAULT | `__init__` | [`firmware/rs00_fk743_test/host/ros2_cmd_vel_uart.py:37`](../../firmware/rs00_fk743_test/host/ros2_cmd_vel_uart.py#L37) |
| Parameter | `'telemetry_period'` | `0.2` | DEFAULT | `__init__` | [`firmware/rs00_fk743_test/host/ros2_cmd_vel_uart.py:38`](../../firmware/rs00_fk743_test/host/ros2_cmd_vel_uart.py#L38) |
| Topic | `topic` | `geometry_msgs.msg.Twist` | SUB | `__init__` | [`firmware/rs00_fk743_test/host/ros2_cmd_vel_uart.py:73`](../../firmware/rs00_fk743_test/host/ros2_cmd_vel_uart.py#L73) |
| Topic | `status_topic` | `std_msgs.msg.String` | PUB | `__init__` | [`firmware/rs00_fk743_test/host/ros2_cmd_vel_uart.py:76`](../../firmware/rs00_fk743_test/host/ros2_cmd_vel_uart.py#L76) |
| Timer | `0.02` | `` | CALLBACK | `__init__` | [`firmware/rs00_fk743_test/host/ros2_cmd_vel_uart.py:77`](../../firmware/rs00_fk743_test/host/ros2_cmd_vel_uart.py#L77) |
| Timer | `telemetry_period` | `` | CALLBACK | `__init__` | [`firmware/rs00_fk743_test/host/ros2_cmd_vel_uart.py:78`](../../firmware/rs00_fk743_test/host/ros2_cmd_vel_uart.py#L78) |
| Timer | `1.0` | `` | CALLBACK | `__init__` | [`firmware/rs00_fk743_test/host/ros2_cmd_vel_uart.py:81`](../../firmware/rs00_fk743_test/host/ros2_cmd_vel_uart.py#L81) |

### antbot_dual_lidar / CloudPreprocessor

Node Name：`antbot_dual_lidar_preprocessor`；Executable：cloud_preprocessor；Source：[`src/antbot_dual_lidar/antbot_dual_lidar/cloud_preprocessor.py:1`](../../src/antbot_dual_lidar/antbot_dual_lidar/cloud_preprocessor.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'lidar_profile'` | `"mapping"` | DEFAULT | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/cloud_preprocessor.py:23`](../../src/antbot_dual_lidar/antbot_dual_lidar/cloud_preprocessor.py#L23) |
| Parameter | `'target_frame'` | `"base_link"` | DEFAULT | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/cloud_preprocessor.py:32`](../../src/antbot_dual_lidar/antbot_dual_lidar/cloud_preprocessor.py#L32) |
| Parameter | `'tf_timeout_sec'` | `0.08` | DEFAULT | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/cloud_preprocessor.py:33`](../../src/antbot_dual_lidar/antbot_dual_lidar/cloud_preprocessor.py#L33) |
| Parameter | `'input_timeout_sec'` | `1.5` | DEFAULT | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/cloud_preprocessor.py:34`](../../src/antbot_dual_lidar/antbot_dual_lidar/cloud_preprocessor.py#L34) |
| Parameter | `'front_left.input_topic'` | `"/antbot/lidar/front_left/points"` | DEFAULT | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/cloud_preprocessor.py:35`](../../src/antbot_dual_lidar/antbot_dual_lidar/cloud_preprocessor.py#L35) |
| Parameter | `'front_left.output_topic'` | `"/antbot/lidar/front_left/points_filtered"` | DEFAULT | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/cloud_preprocessor.py:36`](../../src/antbot_dual_lidar/antbot_dual_lidar/cloud_preprocessor.py#L36) |
| Parameter | `'rear_right.input_topic'` | `"/antbot/lidar/rear_right/points"` | DEFAULT | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/cloud_preprocessor.py:37`](../../src/antbot_dual_lidar/antbot_dual_lidar/cloud_preprocessor.py#L37) |
| Parameter | `'rear_right.output_topic'` | `"/antbot/lidar/rear_right/points_filtered"` | DEFAULT | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/cloud_preprocessor.py:38`](../../src/antbot_dual_lidar/antbot_dual_lidar/cloud_preprocessor.py#L38) |
| Parameter | `f'{prefix}.{name}'` | `default` | DEFAULT | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/cloud_preprocessor.py:45`](../../src/antbot_dual_lidar/antbot_dual_lidar/cloud_preprocessor.py#L45) |
| Parameter | `f'body_filter.{name}'` | `default` | DEFAULT | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/cloud_preprocessor.py:52`](../../src/antbot_dual_lidar/antbot_dual_lidar/cloud_preprocessor.py#L52) |
| Topic | `output` | `sensor_msgs.msg.PointCloud2` | PUB | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/cloud_preprocessor.py:68`](../../src/antbot_dual_lidar/antbot_dual_lidar/cloud_preprocessor.py#L68) |
| Topic | `input_topic` | `sensor_msgs.msg.PointCloud2` | SUB | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/cloud_preprocessor.py:69`](../../src/antbot_dual_lidar/antbot_dual_lidar/cloud_preprocessor.py#L69) |
| Timer | `max(0.1, self.input_timeout / 2.0)` | `` | CALLBACK | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/cloud_preprocessor.py:80`](../../src/antbot_dual_lidar/antbot_dual_lidar/cloud_preprocessor.py#L80) |

### antbot_dual_lidar / MODULE

Node Name：`helper / MODULE / derived node；NEED_CONFIRMATION`；Executable：coverage_probe；Source：[`src/antbot_dual_lidar/antbot_dual_lidar/coverage_probe.py:1`](../../src/antbot_dual_lidar/antbot_dual_lidar/coverage_probe.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Topic | `'/antbot/lidar/front_left/points'` | `sensor_msgs.msg.PointCloud2` | SUB | `main` | [`src/antbot_dual_lidar/antbot_dual_lidar/coverage_probe.py:76`](../../src/antbot_dual_lidar/antbot_dual_lidar/coverage_probe.py#L76) |
| Topic | `'/antbot/lidar/rear_right/points'` | `sensor_msgs.msg.PointCloud2` | SUB | `main` | [`src/antbot_dual_lidar/antbot_dual_lidar/coverage_probe.py:82`](../../src/antbot_dual_lidar/antbot_dual_lidar/coverage_probe.py#L82) |

### antbot_dual_lidar / DualLidarDiagnostics

Node Name：`antbot_dual_lidar_diagnostics`；Executable：dual_lidar_diagnostics；Source：[`src/antbot_dual_lidar/antbot_dual_lidar/diagnostics.py:1`](../../src/antbot_dual_lidar/antbot_dual_lidar/diagnostics.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'front_topic'` | `"/antbot/lidar/front_left/points_raw_native"` | DEFAULT | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/diagnostics.py:17`](../../src/antbot_dual_lidar/antbot_dual_lidar/diagnostics.py#L17) |
| Parameter | `'rear_topic'` | `"/antbot/lidar/rear_right/points_raw_native"` | DEFAULT | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/diagnostics.py:20`](../../src/antbot_dual_lidar/antbot_dual_lidar/diagnostics.py#L20) |
| Parameter | `'max_cloud_time_difference_sec'` | `0.05` | DEFAULT | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/diagnostics.py:23`](../../src/antbot_dual_lidar/antbot_dual_lidar/diagnostics.py#L23) |
| Parameter | `'cloud_timeout_sec'` | `1.5` | DEFAULT | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/diagnostics.py:24`](../../src/antbot_dual_lidar/antbot_dual_lidar/diagnostics.py#L24) |
| Parameter | `'minimum_expected_frequency'` | `5.0` | DEFAULT | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/diagnostics.py:25`](../../src/antbot_dual_lidar/antbot_dual_lidar/diagnostics.py#L25) |
| Parameter | `'report_period_sec'` | `2.0` | DEFAULT | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/diagnostics.py:26`](../../src/antbot_dual_lidar/antbot_dual_lidar/diagnostics.py#L26) |
| Topic | `'/clock'` | `rosgraph_msgs.msg.Clock` | SUB | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/diagnostics.py:37`](../../src/antbot_dual_lidar/antbot_dual_lidar/diagnostics.py#L37) |
| Topic | `str(self.get_parameter('front_topic').value)` | `sensor_msgs.msg.PointCloud2` | SUB | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/diagnostics.py:40`](../../src/antbot_dual_lidar/antbot_dual_lidar/diagnostics.py#L40) |
| Topic | `str(self.get_parameter('rear_topic').value)` | `sensor_msgs.msg.PointCloud2` | SUB | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/diagnostics.py:44`](../../src/antbot_dual_lidar/antbot_dual_lidar/diagnostics.py#L44) |
| Timer | `period` | `` | CALLBACK | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/diagnostics.py:48`](../../src/antbot_dual_lidar/antbot_dual_lidar/diagnostics.py#L48) |

### antbot_dual_lidar / DynamicObstacleMonitor

Node Name：`antbot_dynamic_obstacle_monitor`；Executable：dynamic_obstacle_monitor；Source：[`src/antbot_dual_lidar/antbot_dual_lidar/dynamic_obstacle_monitor.py:1`](../../src/antbot_dual_lidar/antbot_dual_lidar/dynamic_obstacle_monitor.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'voxel_size'` | `0.12` | DEFAULT | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/dynamic_obstacle_monitor.py:43`](../../src/antbot_dual_lidar/antbot_dual_lidar/dynamic_obstacle_monitor.py#L43) |
| Parameter | `'confirmation_hits'` | `3` | DEFAULT | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/dynamic_obstacle_monitor.py:44`](../../src/antbot_dual_lidar/antbot_dual_lidar/dynamic_obstacle_monitor.py#L44) |
| Parameter | `'persistence_sec'` | `2.5` | DEFAULT | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/dynamic_obstacle_monitor.py:45`](../../src/antbot_dual_lidar/antbot_dual_lidar/dynamic_obstacle_monitor.py#L45) |
| Parameter | `'visual_stride'` | `6` | DEFAULT | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/dynamic_obstacle_monitor.py:46`](../../src/antbot_dual_lidar/antbot_dual_lidar/dynamic_obstacle_monitor.py#L46) |
| Parameter | `'min_height'` | `0.06` | DEFAULT | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/dynamic_obstacle_monitor.py:47`](../../src/antbot_dual_lidar/antbot_dual_lidar/dynamic_obstacle_monitor.py#L47) |
| Parameter | `'max_height'` | `1.9` | DEFAULT | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/dynamic_obstacle_monitor.py:48`](../../src/antbot_dual_lidar/antbot_dual_lidar/dynamic_obstacle_monitor.py#L48) |
| Parameter | `'max_range'` | `6.0` | DEFAULT | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/dynamic_obstacle_monitor.py:49`](../../src/antbot_dual_lidar/antbot_dual_lidar/dynamic_obstacle_monitor.py#L49) |
| Topic | `'/antbot/dynamic_obstacles/lidar_points'` | `sensor_msgs.msg.PointCloud2` | PUB | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/dynamic_obstacle_monitor.py:70`](../../src/antbot_dual_lidar/antbot_dual_lidar/dynamic_obstacle_monitor.py#L70) |
| Topic | `'/antbot/dynamic_obstacles/visual_points'` | `sensor_msgs.msg.PointCloud2` | PUB | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/dynamic_obstacle_monitor.py:73`](../../src/antbot_dual_lidar/antbot_dual_lidar/dynamic_obstacle_monitor.py#L73) |
| Topic | `'/antbot/dynamic_obstacles/status'` | `std_msgs.msg.String` | PUB | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/dynamic_obstacle_monitor.py:76`](../../src/antbot_dual_lidar/antbot_dual_lidar/dynamic_obstacle_monitor.py#L76) |
| Topic | `'/map'` | `nav_msgs.msg.OccupancyGrid` | SUB | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/dynamic_obstacle_monitor.py:79`](../../src/antbot_dual_lidar/antbot_dual_lidar/dynamic_obstacle_monitor.py#L79) |
| Topic | `'/antbot/offline_map_points'` | `sensor_msgs.msg.PointCloud2` | SUB | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/dynamic_obstacle_monitor.py:80`](../../src/antbot_dual_lidar/antbot_dual_lidar/dynamic_obstacle_monitor.py#L80) |
| Topic | `'/antbot/rgbd/offline_cloud'` | `sensor_msgs.msg.PointCloud2` | SUB | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/dynamic_obstacle_monitor.py:84`](../../src/antbot_dual_lidar/antbot_dual_lidar/dynamic_obstacle_monitor.py#L84) |
| Topic | `topic` | `sensor_msgs.msg.PointCloud2` | SUB | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/dynamic_obstacle_monitor.py:92`](../../src/antbot_dual_lidar/antbot_dual_lidar/dynamic_obstacle_monitor.py#L92) |
| Topic | `'/antbot/camera/color/image_raw'` | `sensor_msgs.msg.Image` | SUB | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/dynamic_obstacle_monitor.py:95`](../../src/antbot_dual_lidar/antbot_dual_lidar/dynamic_obstacle_monitor.py#L95) |
| Topic | `'/antbot/camera/depth/camera_info'` | `sensor_msgs.msg.CameraInfo` | SUB | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/dynamic_obstacle_monitor.py:99`](../../src/antbot_dual_lidar/antbot_dual_lidar/dynamic_obstacle_monitor.py#L99) |
| Topic | `'/antbot/camera/depth/image_raw'` | `sensor_msgs.msg.Image` | SUB | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/dynamic_obstacle_monitor.py:103`](../../src/antbot_dual_lidar/antbot_dual_lidar/dynamic_obstacle_monitor.py#L103) |
| Timer | `1.0` | `` | CALLBACK | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/dynamic_obstacle_monitor.py:107`](../../src/antbot_dual_lidar/antbot_dual_lidar/dynamic_obstacle_monitor.py#L107) |

### antbot_dual_lidar / FixedFrameMapper

Node Name：`antbot_fixed_frame_mapper`；Executable：fixed_frame_mapper；Source：[`src/antbot_dual_lidar/antbot_dual_lidar/fixed_frame_mapper.py:1`](../../src/antbot_dual_lidar/antbot_dual_lidar/fixed_frame_mapper.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'fixed_frame'` | `"odom"` | DEFAULT | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/fixed_frame_mapper.py:27`](../../src/antbot_dual_lidar/antbot_dual_lidar/fixed_frame_mapper.py#L27) |
| Parameter | `'front_topic'` | `"/antbot/lidar/front_left/points_filtered"` | DEFAULT | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/fixed_frame_mapper.py:28`](../../src/antbot_dual_lidar/antbot_dual_lidar/fixed_frame_mapper.py#L28) |
| Parameter | `'rear_topic'` | `"/antbot/lidar/rear_right/points_filtered"` | DEFAULT | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/fixed_frame_mapper.py:31`](../../src/antbot_dual_lidar/antbot_dual_lidar/fixed_frame_mapper.py#L31) |
| Parameter | `'odom_topic'` | `"/odom"` | DEFAULT | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/fixed_frame_mapper.py:34`](../../src/antbot_dual_lidar/antbot_dual_lidar/fixed_frame_mapper.py#L34) |
| Parameter | `'map_topic'` | `"/antbot/lidar/map_points"` | DEFAULT | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/fixed_frame_mapper.py:35`](../../src/antbot_dual_lidar/antbot_dual_lidar/fixed_frame_mapper.py#L35) |
| Parameter | `'trajectory_topic'` | `"/antbot/trajectory"` | DEFAULT | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/fixed_frame_mapper.py:36`](../../src/antbot_dual_lidar/antbot_dual_lidar/fixed_frame_mapper.py#L36) |
| Parameter | `'voxel_leaf_size'` | `0.05` | DEFAULT | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/fixed_frame_mapper.py:37`](../../src/antbot_dual_lidar/antbot_dual_lidar/fixed_frame_mapper.py#L37) |
| Parameter | `'max_map_points'` | `300000` | DEFAULT | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/fixed_frame_mapper.py:38`](../../src/antbot_dual_lidar/antbot_dual_lidar/fixed_frame_mapper.py#L38) |
| Parameter | `'publish_frequency'` | `2.0` | DEFAULT | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/fixed_frame_mapper.py:39`](../../src/antbot_dual_lidar/antbot_dual_lidar/fixed_frame_mapper.py#L39) |
| Parameter | `'trajectory_min_distance'` | `0.05` | DEFAULT | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/fixed_frame_mapper.py:40`](../../src/antbot_dual_lidar/antbot_dual_lidar/fixed_frame_mapper.py#L40) |
| Parameter | `'trajectory_max_poses'` | `10000` | DEFAULT | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/fixed_frame_mapper.py:41`](../../src/antbot_dual_lidar/antbot_dual_lidar/fixed_frame_mapper.py#L41) |
| Parameter | `'tf_timeout_sec'` | `0.10` | DEFAULT | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/fixed_frame_mapper.py:42`](../../src/antbot_dual_lidar/antbot_dual_lidar/fixed_frame_mapper.py#L42) |
| Topic | `str(self.get_parameter('map_topic').value)` | `sensor_msgs.msg.PointCloud2` | PUB | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/fixed_frame_mapper.py:72`](../../src/antbot_dual_lidar/antbot_dual_lidar/fixed_frame_mapper.py#L72) |
| Topic | `str(self.get_parameter('trajectory_topic').value)` | `nav_msgs.msg.Path` | PUB | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/fixed_frame_mapper.py:75`](../../src/antbot_dual_lidar/antbot_dual_lidar/fixed_frame_mapper.py#L75) |
| Topic | `str(self.get_parameter(parameter).value)` | `sensor_msgs.msg.PointCloud2` | SUB | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/fixed_frame_mapper.py:88`](../../src/antbot_dual_lidar/antbot_dual_lidar/fixed_frame_mapper.py#L88) |
| Topic | `str(self.get_parameter('odom_topic').value)` | `nav_msgs.msg.Odometry` | SUB | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/fixed_frame_mapper.py:94`](../../src/antbot_dual_lidar/antbot_dual_lidar/fixed_frame_mapper.py#L94) |
| Timer | `1.0 / frequency` | `` | CALLBACK | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/fixed_frame_mapper.py:100`](../../src/antbot_dual_lidar/antbot_dual_lidar/fixed_frame_mapper.py#L100) |

### antbot_dual_lidar / GroundTruthDeskewValidator

Node Name：`ground_truth_deskew_validator`；Executable：ground_truth_deskew_validator；Source：[`src/antbot_dual_lidar/antbot_dual_lidar/ground_truth_deskew_validator.py:1`](../../src/antbot_dual_lidar/antbot_dual_lidar/ground_truth_deskew_validator.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'ground_truth_topic'` | `"/antbot/ground_truth/odom"` | DEFAULT | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/ground_truth_deskew_validator.py:45`](../../src/antbot_dual_lidar/antbot_dual_lidar/ground_truth_deskew_validator.py#L45) |
| Parameter | `'point_time_convention'` | `"header_plus_offset"` | DEFAULT | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/ground_truth_deskew_validator.py:46`](../../src/antbot_dual_lidar/antbot_dual_lidar/ground_truth_deskew_validator.py#L46) |
| Parameter | `'scan_period_sec'` | `0.1` | DEFAULT | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/ground_truth_deskew_validator.py:47`](../../src/antbot_dual_lidar/antbot_dual_lidar/ground_truth_deskew_validator.py#L47) |
| Parameter | `'pose_buffer_sec'` | `10.0` | DEFAULT | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/ground_truth_deskew_validator.py:48`](../../src/antbot_dual_lidar/antbot_dual_lidar/ground_truth_deskew_validator.py#L48) |
| Topic | `truth_topic` | `nav_msgs.msg.Odometry` | SUB | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/ground_truth_deskew_validator.py:61`](../../src/antbot_dual_lidar/antbot_dual_lidar/ground_truth_deskew_validator.py#L61) |
| Topic | `config['output']` | `sensor_msgs.msg.PointCloud2` | PUB | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/ground_truth_deskew_validator.py:66`](../../src/antbot_dual_lidar/antbot_dual_lidar/ground_truth_deskew_validator.py#L66) |
| Topic | `config['world']` | `sensor_msgs.msg.PointCloud2` | PUB | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/ground_truth_deskew_validator.py:69`](../../src/antbot_dual_lidar/antbot_dual_lidar/ground_truth_deskew_validator.py#L69) |
| Topic | `config['input']` | `sensor_msgs.msg.PointCloud2` | SUB | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/ground_truth_deskew_validator.py:73`](../../src/antbot_dual_lidar/antbot_dual_lidar/ground_truth_deskew_validator.py#L73) |
| Timer | `10.0` | `` | CALLBACK | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/ground_truth_deskew_validator.py:80`](../../src/antbot_dual_lidar/antbot_dual_lidar/ground_truth_deskew_validator.py#L80) |

### antbot_dual_lidar / LivoxFrameRelay

Node Name：`antbot_livox_frame_relay`；Executable：livox_frame_relay；Source：[`src/antbot_dual_lidar/antbot_dual_lidar/livox_frame_relay.py:1`](../../src/antbot_dual_lidar/antbot_dual_lidar/livox_frame_relay.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `f'{name}.{parameter}'` | `default` | DEFAULT | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/livox_frame_relay.py:39`](../../src/antbot_dual_lidar/antbot_dual_lidar/livox_frame_relay.py#L39) |
| Topic | `output_topic` | `sensor_msgs.msg.PointCloud2` | PUB | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/livox_frame_relay.py:53`](../../src/antbot_dual_lidar/antbot_dual_lidar/livox_frame_relay.py#L53) |
| Topic | `input_topic` | `sensor_msgs.msg.PointCloud2` | SUB | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/livox_frame_relay.py:56`](../../src/antbot_dual_lidar/antbot_dual_lidar/livox_frame_relay.py#L56) |

### antbot_dual_lidar / OfflinePointCloudPublisher

Node Name：`antbot_offline_pointcloud_publisher`；Executable：offline_pointcloud_publisher；Source：[`src/antbot_dual_lidar/antbot_dual_lidar/offline_pointcloud_publisher.py:1`](../../src/antbot_dual_lidar/antbot_dual_lidar/offline_pointcloud_publisher.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'pointcloud_path'` | `""` | DEFAULT | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/offline_pointcloud_publisher.py:19`](../../src/antbot_dual_lidar/antbot_dual_lidar/offline_pointcloud_publisher.py#L19) |
| Parameter | `'metadata_path'` | `""` | DEFAULT | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/offline_pointcloud_publisher.py:20`](../../src/antbot_dual_lidar/antbot_dual_lidar/offline_pointcloud_publisher.py#L20) |
| Parameter | `'publish_topic'` | `"/antbot/offline_map_points"` | DEFAULT | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/offline_pointcloud_publisher.py:21`](../../src/antbot_dual_lidar/antbot_dual_lidar/offline_pointcloud_publisher.py#L21) |
| Parameter | `'target_frame'` | `"map"` | DEFAULT | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/offline_pointcloud_publisher.py:22`](../../src/antbot_dual_lidar/antbot_dual_lidar/offline_pointcloud_publisher.py#L22) |
| Parameter | `'publish_rate'` | `0.5` | DEFAULT | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/offline_pointcloud_publisher.py:23`](../../src/antbot_dual_lidar/antbot_dual_lidar/offline_pointcloud_publisher.py#L23) |
| Topic | `topic` | `sensor_msgs.msg.PointCloud2` | PUB | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/offline_pointcloud_publisher.py:58`](../../src/antbot_dual_lidar/antbot_dual_lidar/offline_pointcloud_publisher.py#L58) |
| Timer | `1.0 / rate if rate > 0 else 0.1` | `` | CALLBACK | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/offline_pointcloud_publisher.py:65`](../../src/antbot_dual_lidar/antbot_dual_lidar/offline_pointcloud_publisher.py#L65) |

### antbot_dual_lidar / MODULE

Node Name：`helper / MODULE / derived node；NEED_CONFIRMATION`；Executable：performance_probe；Source：[`src/antbot_dual_lidar/antbot_dual_lidar/performance_probe.py:1`](../../src/antbot_dual_lidar/antbot_dual_lidar/performance_probe.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Topic | `topic` | `sensor_msgs.msg.PointCloud2` | SUB | `main` | [`src/antbot_dual_lidar/antbot_dual_lidar/performance_probe.py:144`](../../src/antbot_dual_lidar/antbot_dual_lidar/performance_probe.py#L144) |
| Topic | `'/clock'` | `rosgraph_msgs.msg.Clock` | SUB | `main` | [`src/antbot_dual_lidar/antbot_dual_lidar/performance_probe.py:159`](../../src/antbot_dual_lidar/antbot_dual_lidar/performance_probe.py#L159) |

### antbot_dual_lidar / MotionExperiment

Node Name：`phase2a_motion_experiment`；Executable：phase2a_motion_experiment；Source：[`src/antbot_dual_lidar/antbot_dual_lidar/phase2a_motion_experiment.py:1`](../../src/antbot_dual_lidar/antbot_dual_lidar/phase2a_motion_experiment.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Topic | `'/cmd_vel'` | `geometry_msgs.msg.Twist` | PUB | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/phase2a_motion_experiment.py:60`](../../src/antbot_dual_lidar/antbot_dual_lidar/phase2a_motion_experiment.py#L60) |
| Topic | `'/clock'` | `rosgraph_msgs.msg.Clock` | SUB | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/phase2a_motion_experiment.py:61`](../../src/antbot_dual_lidar/antbot_dual_lidar/phase2a_motion_experiment.py#L61) |
| Topic | `'/antbot/imu/data'` | `sensor_msgs.msg.Imu` | SUB | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/phase2a_motion_experiment.py:62`](../../src/antbot_dual_lidar/antbot_dual_lidar/phase2a_motion_experiment.py#L62) |
| Topic | `'/antbot/ground_truth/odom'` | `nav_msgs.msg.Odometry` | SUB | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/phase2a_motion_experiment.py:63`](../../src/antbot_dual_lidar/antbot_dual_lidar/phase2a_motion_experiment.py#L63) |
| Timer | `0.05` | `` | CALLBACK | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/phase2a_motion_experiment.py:74`](../../src/antbot_dual_lidar/antbot_dual_lidar/phase2a_motion_experiment.py#L74) |

### antbot_dual_lidar / LatestCloudSaver

Node Name：`antbot_pointcloud_saver`；Executable：save_pointcloud；Source：[`src/antbot_dual_lidar/antbot_dual_lidar/save_pointcloud.py:1`](../../src/antbot_dual_lidar/antbot_dual_lidar/save_pointcloud.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Topic | `topic` | `sensor_msgs.msg.PointCloud2` | SUB | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/save_pointcloud.py:45`](../../src/antbot_dual_lidar/antbot_dual_lidar/save_pointcloud.py#L45) |

### antbot_dual_lidar / DualCloudSynchronizer

Node Name：`antbot_dual_lidar_synchronizer`；Executable：dual_cloud_synchronizer；Source：[`src/antbot_dual_lidar/antbot_dual_lidar/synchronizer.py:1`](../../src/antbot_dual_lidar/antbot_dual_lidar/synchronizer.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'front_topic'` | `"/antbot/lidar/front_left/points"` | DEFAULT | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/synchronizer.py:16`](../../src/antbot_dual_lidar/antbot_dual_lidar/synchronizer.py#L16) |
| Parameter | `'rear_topic'` | `"/antbot/lidar/rear_right/points"` | DEFAULT | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/synchronizer.py:17`](../../src/antbot_dual_lidar/antbot_dual_lidar/synchronizer.py#L17) |
| Parameter | `'front_output_topic'` | `"/antbot/lidar/synchronized/front_left"` | DEFAULT | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/synchronizer.py:18`](../../src/antbot_dual_lidar/antbot_dual_lidar/synchronizer.py#L18) |
| Parameter | `'rear_output_topic'` | `"/antbot/lidar/synchronized/rear_right"` | DEFAULT | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/synchronizer.py:21`](../../src/antbot_dual_lidar/antbot_dual_lidar/synchronizer.py#L21) |
| Parameter | `'strategy'` | `"approximate"` | DEFAULT | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/synchronizer.py:24`](../../src/antbot_dual_lidar/antbot_dual_lidar/synchronizer.py#L24) |
| Parameter | `'maximum_pair_delta_sec'` | `0.02` | DEFAULT | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/synchronizer.py:25`](../../src/antbot_dual_lidar/antbot_dual_lidar/synchronizer.py#L25) |
| Parameter | `'expected_period_sec'` | `0.10` | DEFAULT | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/synchronizer.py:26`](../../src/antbot_dual_lidar/antbot_dual_lidar/synchronizer.py#L26) |
| Parameter | `'queue_size'` | `20` | DEFAULT | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/synchronizer.py:27`](../../src/antbot_dual_lidar/antbot_dual_lidar/synchronizer.py#L27) |
| Parameter | `'publish_synchronized'` | `False` | DEFAULT | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/synchronizer.py:28`](../../src/antbot_dual_lidar/antbot_dual_lidar/synchronizer.py#L28) |
| Parameter | `'report_period_sec'` | `5.0` | DEFAULT | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/synchronizer.py:29`](../../src/antbot_dual_lidar/antbot_dual_lidar/synchronizer.py#L29) |
| Topic | `str(self.get_parameter('front_output_topic').value)` | `sensor_msgs.msg.PointCloud2` | PUB | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/synchronizer.py:44`](../../src/antbot_dual_lidar/antbot_dual_lidar/synchronizer.py#L44) |
| Topic | `str(self.get_parameter('rear_output_topic').value)` | `sensor_msgs.msg.PointCloud2` | PUB | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/synchronizer.py:49`](../../src/antbot_dual_lidar/antbot_dual_lidar/synchronizer.py#L49) |
| Topic | `str(self.get_parameter('front_topic').value)` | `sensor_msgs.msg.PointCloud2` | SUB | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/synchronizer.py:54`](../../src/antbot_dual_lidar/antbot_dual_lidar/synchronizer.py#L54) |
| Topic | `str(self.get_parameter('rear_topic').value)` | `sensor_msgs.msg.PointCloud2` | SUB | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/synchronizer.py:60`](../../src/antbot_dual_lidar/antbot_dual_lidar/synchronizer.py#L60) |
| Timer | `float(self.get_parameter('report_period_sec').value)` | `` | CALLBACK | `__init__` | [`src/antbot_dual_lidar/antbot_dual_lidar/synchronizer.py:66`](../../src/antbot_dual_lidar/antbot_dual_lidar/synchronizer.py#L66) |

### antbot_h743_bridge / CmdVelUartBridge

Node Name：`cmd_vel_uart_bridge`；Executable：h743_cmd_vel_bridge；Source：[`src/antbot_h743_bridge/antbot_h743_bridge/bridge.py:1`](../../src/antbot_h743_bridge/antbot_h743_bridge/bridge.py#L1)。

Hardware=H743 UART；Startup=串口/ACK，允许离线参数决定是否退出；Target=底盘；Update=serial50Hz、joint10Hz、status1Hz；Fault=撤销门禁/重连/退出零帧。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'port'` | `resolve_uart_port(),` | DEFAULT | `__init__` | [`src/antbot_h743_bridge/antbot_h743_bridge/bridge.py:87`](../../src/antbot_h743_bridge/antbot_h743_bridge/bridge.py#L87) |
| Parameter | `'baud'` | `115200` | DEFAULT | `__init__` | [`src/antbot_h743_bridge/antbot_h743_bridge/bridge.py:91`](../../src/antbot_h743_bridge/antbot_h743_bridge/bridge.py#L91) |
| Parameter | `'topic'` | `"/cmd_vel"` | DEFAULT | `__init__` | [`src/antbot_h743_bridge/antbot_h743_bridge/bridge.py:92`](../../src/antbot_h743_bridge/antbot_h743_bridge/bridge.py#L92) |
| Parameter | `'max_linear_speed'` | `0.5` | DEFAULT | `__init__` | [`src/antbot_h743_bridge/antbot_h743_bridge/bridge.py:93`](../../src/antbot_h743_bridge/antbot_h743_bridge/bridge.py#L93) |
| Parameter | `'status_topic'` | `"/rs00/motor_status"` | DEFAULT | `__init__` | [`src/antbot_h743_bridge/antbot_h743_bridge/bridge.py:94`](../../src/antbot_h743_bridge/antbot_h743_bridge/bridge.py#L94) |
| Parameter | `'vehicle_status_topic'` | `"/antbot/vehicle_status"` | DEFAULT | `__init__` | [`src/antbot_h743_bridge/antbot_h743_bridge/bridge.py:95`](../../src/antbot_h743_bridge/antbot_h743_bridge/bridge.py#L95) |
| Parameter | `'joint_state_topic'` | `"/joint_states"` | DEFAULT | `__init__` | [`src/antbot_h743_bridge/antbot_h743_bridge/bridge.py:96`](../../src/antbot_h743_bridge/antbot_h743_bridge/bridge.py#L96) |
| Parameter | `'battery_topic'` | `"/battery"` | DEFAULT | `__init__` | [`src/antbot_h743_bridge/antbot_h743_bridge/bridge.py:97`](../../src/antbot_h743_bridge/antbot_h743_bridge/bridge.py#L97) |
| Parameter | `'battery_empty_voltage'` | `18.0` | DEFAULT | `__init__` | [`src/antbot_h743_bridge/antbot_h743_bridge/bridge.py:98`](../../src/antbot_h743_bridge/antbot_h743_bridge/bridge.py#L98) |
| Parameter | `'battery_full_voltage'` | `30.0` | DEFAULT | `__init__` | [`src/antbot_h743_bridge/antbot_h743_bridge/bridge.py:99`](../../src/antbot_h743_bridge/antbot_h743_bridge/bridge.py#L99) |
| Parameter | `'telemetry_period'` | `0.2` | DEFAULT | `__init__` | [`src/antbot_h743_bridge/antbot_h743_bridge/bridge.py:100`](../../src/antbot_h743_bridge/antbot_h743_bridge/bridge.py#L100) |
| Parameter | `'allow_disconnected'` | `False` | DEFAULT | `__init__` | [`src/antbot_h743_bridge/antbot_h743_bridge/bridge.py:101`](../../src/antbot_h743_bridge/antbot_h743_bridge/bridge.py#L101) |
| Parameter | `'reconnect_period'` | `1.0` | DEFAULT | `__init__` | [`src/antbot_h743_bridge/antbot_h743_bridge/bridge.py:102`](../../src/antbot_h743_bridge/antbot_h743_bridge/bridge.py#L102) |
| Parameter | `'telemetry_timeout'` | `1.5` | DEFAULT | `__init__` | [`src/antbot_h743_bridge/antbot_h743_bridge/bridge.py:103`](../../src/antbot_h743_bridge/antbot_h743_bridge/bridge.py#L103) |
| Parameter | `'operator_enable_service'` | `"/antbot/operator_enable"` | DEFAULT | `__init__` | [`src/antbot_h743_bridge/antbot_h743_bridge/bridge.py:104`](../../src/antbot_h743_bridge/antbot_h743_bridge/bridge.py#L104) |
| Parameter | `'system_reset_service'` | `"/antbot/system_reset"` | DEFAULT | `__init__` | [`src/antbot_h743_bridge/antbot_h743_bridge/bridge.py:107`](../../src/antbot_h743_bridge/antbot_h743_bridge/bridge.py#L107) |
| Topic | `topic` | `geometry_msgs.msg.Twist` | SUB | `__init__` | [`src/antbot_h743_bridge/antbot_h743_bridge/bridge.py:173`](../../src/antbot_h743_bridge/antbot_h743_bridge/bridge.py#L173) |
| Topic | `status_topic` | `std_msgs.msg.String` | PUB | `__init__` | [`src/antbot_h743_bridge/antbot_h743_bridge/bridge.py:176`](../../src/antbot_h743_bridge/antbot_h743_bridge/bridge.py#L176) |
| Topic | `vehicle_status_topic` | `std_msgs.msg.String` | PUB | `__init__` | [`src/antbot_h743_bridge/antbot_h743_bridge/bridge.py:179`](../../src/antbot_h743_bridge/antbot_h743_bridge/bridge.py#L179) |
| Topic | `joint_state_topic` | `sensor_msgs.msg.JointState` | PUB | `__init__` | [`src/antbot_h743_bridge/antbot_h743_bridge/bridge.py:181`](../../src/antbot_h743_bridge/antbot_h743_bridge/bridge.py#L181) |
| Topic | `battery_topic` | `sensor_msgs.msg.BatteryState` | PUB | `__init__` | [`src/antbot_h743_bridge/antbot_h743_bridge/bridge.py:184`](../../src/antbot_h743_bridge/antbot_h743_bridge/bridge.py#L184) |
| Service | `str(self.get_parameter('operator_enable_service').value)` | `std_srvs.srv.SetBool` | SERVER | `__init__` | [`src/antbot_h743_bridge/antbot_h743_bridge/bridge.py:187`](../../src/antbot_h743_bridge/antbot_h743_bridge/bridge.py#L187) |
| Service | `str(self.get_parameter('system_reset_service').value)` | `std_srvs.srv.Trigger` | SERVER | `__init__` | [`src/antbot_h743_bridge/antbot_h743_bridge/bridge.py:192`](../../src/antbot_h743_bridge/antbot_h743_bridge/bridge.py#L192) |
| Timer | `0.02` | `` | CALLBACK | `__init__` | [`src/antbot_h743_bridge/antbot_h743_bridge/bridge.py:197`](../../src/antbot_h743_bridge/antbot_h743_bridge/bridge.py#L197) |
| Timer | `telemetry_period` | `` | CALLBACK | `__init__` | [`src/antbot_h743_bridge/antbot_h743_bridge/bridge.py:198`](../../src/antbot_h743_bridge/antbot_h743_bridge/bridge.py#L198) |
| Timer | `1.0` | `` | CALLBACK | `__init__` | [`src/antbot_h743_bridge/antbot_h743_bridge/bridge.py:201`](../../src/antbot_h743_bridge/antbot_h743_bridge/bridge.py#L201) |
| Timer | `0.1` | `` | CALLBACK | `__init__` | [`src/antbot_h743_bridge/antbot_h743_bridge/bridge.py:202`](../../src/antbot_h743_bridge/antbot_h743_bridge/bridge.py#L202) |
| Timer | `reconnect_period` | `` | CALLBACK | `__init__` | [`src/antbot_h743_bridge/antbot_h743_bridge/bridge.py:203`](../../src/antbot_h743_bridge/antbot_h743_bridge/bridge.py#L203) |

### antbot_h743_bridge / OperatorManager

Node Name：`antbot_operator_manager`；Executable：antbot_operator_manager；Source：[`src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py:1`](../../src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'default_teleop_mode'` | `"xbox"` | DEFAULT | `__init__` | [`src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py:94`](../../src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py#L94) |
| Parameter | `'xbox_cmd_topic'` | `"/antbot/cmd_vel/xbox"` | DEFAULT | `__init__` | [`src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py:95`](../../src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py#L95) |
| Parameter | `'keyboard_cmd_topic'` | `"/antbot/cmd_vel/keyboard"` | DEFAULT | `__init__` | [`src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py:96`](../../src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py#L96) |
| Parameter | `'joy_topic'` | `"/joy"` | DEFAULT | `__init__` | [`src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py:97`](../../src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py#L97) |
| Parameter | `'manage_joy'` | `True` | DEFAULT | `__init__` | [`src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py:98`](../../src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py#L98) |
| Parameter | `'joy_device'` | `"auto"` | DEFAULT | `__init__` | [`src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py:99`](../../src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py#L99) |
| Parameter | `'output_cmd_topic'` | `"/cmd_vel"` | DEFAULT | `__init__` | [`src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py:100`](../../src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py#L100) |
| Parameter | `'max_linear_speed'` | `0.10` | DEFAULT | `__init__` | [`src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py:101`](../../src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py#L101) |
| Parameter | `'command_timeout'` | `0.35` | DEFAULT | `__init__` | [`src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py:102`](../../src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py#L102) |
| Parameter | `'scan_topic'` | `"/scan_0"` | DEFAULT | `__init__` | [`src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py:103`](../../src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py#L103) |
| Parameter | `'mapping_topic'` | `"/antbot/mapping/map"` | DEFAULT | `__init__` | [`src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py:104`](../../src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py#L104) |
| Parameter | `'mapping_output_prefix'` | `"/tmp/antbot_mapping/map"` | DEFAULT | `__init__` | [`src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py:105`](../../src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py#L105) |
| Topic | `str(self.get_parameter('output_cmd_topic').value)` | `geometry_msgs.msg.Twist` | PUB | `__init__` | [`src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py:146`](../../src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py#L146) |
| Topic | `'/antbot/operator_ui_status'` | `std_msgs.msg.String` | PUB | `__init__` | [`src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py:149`](../../src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py#L149) |
| Topic | `str(self.get_parameter('xbox_cmd_topic').value)` | `geometry_msgs.msg.Twist` | SUB | `__init__` | [`src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py:152`](../../src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py#L152) |
| Topic | `str(self.get_parameter('keyboard_cmd_topic').value)` | `geometry_msgs.msg.Twist` | SUB | `__init__` | [`src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py:158`](../../src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py#L158) |
| Topic | `'/antbot_xbox/armed'` | `std_msgs.msg.Bool` | SUB | `__init__` | [`src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py:164`](../../src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py#L164) |
| Topic | `'/antbot_xbox/status'` | `std_msgs.msg.String` | SUB | `__init__` | [`src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py:167`](../../src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py#L167) |
| Service | `'/antbot/teleop/use_xbox'` | `std_srvs.srv.SetBool` | SERVER | `__init__` | [`src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py:170`](../../src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py#L170) |
| Service | `'/antbot/mapping/set_enabled'` | `std_srvs.srv.SetBool` | SERVER | `__init__` | [`src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py:173`](../../src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py#L173) |
| Service | `'/antbot/mapping/save'` | `std_srvs.srv.Trigger` | SERVER | `__init__` | [`src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py:176`](../../src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py#L176) |
| Timer | `0.05` | `` | CALLBACK | `__init__` | [`src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py:179`](../../src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py#L179) |
| Timer | `0.5` | `` | CALLBACK | `__init__` | [`src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py:180`](../../src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py#L180) |
| Timer | `1.0` | `` | CALLBACK | `__init__` | [`src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py:181`](../../src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py#L181) |

### antbot_lidar_fusion / LidarFusionNode

Node Name：`lidar_fusion_node`；Executable：lidar_fusion_node；Source：[`src/antbot_lidar_fusion/antbot_lidar_fusion/lidar_fusion_node.py:1`](../../src/antbot_lidar_fusion/antbot_lidar_fusion/lidar_fusion_node.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'front_scan_topic'` | `"/scan_0"` | DEFAULT | `__init__` | [`src/antbot_lidar_fusion/antbot_lidar_fusion/lidar_fusion_node.py:57`](../../src/antbot_lidar_fusion/antbot_lidar_fusion/lidar_fusion_node.py#L57) |
| Parameter | `'rear_scan_topic'` | `"/scan_1"` | DEFAULT | `__init__` | [`src/antbot_lidar_fusion/antbot_lidar_fusion/lidar_fusion_node.py:58`](../../src/antbot_lidar_fusion/antbot_lidar_fusion/lidar_fusion_node.py#L58) |
| Parameter | `'target_frame'` | `"base_link"` | DEFAULT | `__init__` | [`src/antbot_lidar_fusion/antbot_lidar_fusion/lidar_fusion_node.py:59`](../../src/antbot_lidar_fusion/antbot_lidar_fusion/lidar_fusion_node.py#L59) |
| Parameter | `'body_min_x'` | `-0.43` | DEFAULT | `__init__` | [`src/antbot_lidar_fusion/antbot_lidar_fusion/lidar_fusion_node.py:60`](../../src/antbot_lidar_fusion/antbot_lidar_fusion/lidar_fusion_node.py#L60) |
| Parameter | `'body_max_x'` | `0.43` | DEFAULT | `__init__` | [`src/antbot_lidar_fusion/antbot_lidar_fusion/lidar_fusion_node.py:61`](../../src/antbot_lidar_fusion/antbot_lidar_fusion/lidar_fusion_node.py#L61) |
| Parameter | `'body_min_y'` | `-0.29` | DEFAULT | `__init__` | [`src/antbot_lidar_fusion/antbot_lidar_fusion/lidar_fusion_node.py:62`](../../src/antbot_lidar_fusion/antbot_lidar_fusion/lidar_fusion_node.py#L62) |
| Parameter | `'body_max_y'` | `0.29` | DEFAULT | `__init__` | [`src/antbot_lidar_fusion/antbot_lidar_fusion/lidar_fusion_node.py:63`](../../src/antbot_lidar_fusion/antbot_lidar_fusion/lidar_fusion_node.py#L63) |
| Topic | `'/antbot/lidar/combined_points'` | `sensor_msgs.msg.PointCloud2` | PUB | `__init__` | [`src/antbot_lidar_fusion/antbot_lidar_fusion/lidar_fusion_node.py:72`](../../src/antbot_lidar_fusion/antbot_lidar_fusion/lidar_fusion_node.py#L72) |

### antbot_lidar_fusion / TopicCheckNode

Node Name：`dual_lidar_topic_check`；Executable：topic_check_node；Source：[`src/antbot_lidar_fusion/antbot_lidar_fusion/topic_check_node.py:1`](../../src/antbot_lidar_fusion/antbot_lidar_fusion/topic_check_node.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Topic | `'/clock'` | `rosgraph_msgs.msg.Clock` | SUB | `__init__` | [`src/antbot_lidar_fusion/antbot_lidar_fusion/topic_check_node.py:32`](../../src/antbot_lidar_fusion/antbot_lidar_fusion/topic_check_node.py#L32) |
| Topic | `'/scan_0'` | `sensor_msgs.msg.LaserScan` | SUB | `__init__` | [`src/antbot_lidar_fusion/antbot_lidar_fusion/topic_check_node.py:33`](../../src/antbot_lidar_fusion/antbot_lidar_fusion/topic_check_node.py#L33) |
| Topic | `'/scan_1'` | `sensor_msgs.msg.LaserScan` | SUB | `__init__` | [`src/antbot_lidar_fusion/antbot_lidar_fusion/topic_check_node.py:36`](../../src/antbot_lidar_fusion/antbot_lidar_fusion/topic_check_node.py#L36) |
| Topic | `'/antbot/lidar/combined_points'` | `sensor_msgs.msg.PointCloud2` | SUB | `__init__` | [`src/antbot_lidar_fusion/antbot_lidar_fusion/topic_check_node.py:39`](../../src/antbot_lidar_fusion/antbot_lidar_fusion/topic_check_node.py#L39) |

### antbot_navigation / ScanFixRelay

Node Name：`scan_fix_relay`；Executable：见 SOURCE_SCAN 入口和 launch 附录；UNKNOWN / NEED_CONFIRMATION；Source：[`src/antbot_navigation/scripts/scan_fix_relay.py:1`](../../src/antbot_navigation/scripts/scan_fix_relay.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'input_topic'` | `'/scan_0'` | DEFAULT | `__init__` | [`src/antbot_navigation/scripts/scan_fix_relay.py:40`](../../src/antbot_navigation/scripts/scan_fix_relay.py#L40) |
| Parameter | `'output_topic'` | `'/scan_0_fixed'` | DEFAULT | `__init__` | [`src/antbot_navigation/scripts/scan_fix_relay.py:41`](../../src/antbot_navigation/scripts/scan_fix_relay.py#L41) |
| Parameter | `'num_ranges'` | `400` | DEFAULT | `__init__` | [`src/antbot_navigation/scripts/scan_fix_relay.py:42`](../../src/antbot_navigation/scripts/scan_fix_relay.py#L42) |
| Topic | `output_topic` | `sensor_msgs.msg.LaserScan` | PUB | `__init__` | [`src/antbot_navigation/scripts/scan_fix_relay.py:54`](../../src/antbot_navigation/scripts/scan_fix_relay.py#L54) |
| Topic | `input_topic` | `sensor_msgs.msg.LaserScan` | SUB | `__init__` | [`src/antbot_navigation/scripts/scan_fix_relay.py:55`](../../src/antbot_navigation/scripts/scan_fix_relay.py#L55) |

### antbot_navigation / WaypointEditor

Node Name：`antbot_waypoint_editor`；Executable：见 SOURCE_SCAN 入口和 launch 附录；UNKNOWN / NEED_CONFIRMATION；Source：[`src/antbot_navigation/scripts/waypoint_editor.py:1`](../../src/antbot_navigation/scripts/waypoint_editor.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'waypoints_file'` | `''` | DEFAULT | `__init__` | [`src/antbot_navigation/scripts/waypoint_editor.py:52`](../../src/antbot_navigation/scripts/waypoint_editor.py#L52) |
| Parameter | `'frame_id'` | `'map'` | DEFAULT | `__init__` | [`src/antbot_navigation/scripts/waypoint_editor.py:53`](../../src/antbot_navigation/scripts/waypoint_editor.py#L53) |
| Parameter | `'autoload'` | `True` | DEFAULT | `__init__` | [`src/antbot_navigation/scripts/waypoint_editor.py:54`](../../src/antbot_navigation/scripts/waypoint_editor.py#L54) |
| Topic | `'/clicked_point'` | `geometry_msgs.msg.PointStamped` | SUB | `__init__` | [`src/antbot_navigation/scripts/waypoint_editor.py:69`](../../src/antbot_navigation/scripts/waypoint_editor.py#L69) |
| Action | `'navigate_through_poses'` | `nav2_msgs.action.NavigateThroughPoses` | CLIENT | `__init__` | [`src/antbot_navigation/scripts/waypoint_editor.py:71`](../../src/antbot_navigation/scripts/waypoint_editor.py#L71) |
| Action | `'navigate_to_pose'` | `nav2_msgs.action.NavigateToPose` | CLIENT | `__init__` | [`src/antbot_navigation/scripts/waypoint_editor.py:73`](../../src/antbot_navigation/scripts/waypoint_editor.py#L73) |
| Service | `'~/save'` | `std_srvs.srv.Trigger` | SERVER | `__init__` | [`src/antbot_navigation/scripts/waypoint_editor.py:76`](../../src/antbot_navigation/scripts/waypoint_editor.py#L76) |
| Service | `'~/start'` | `std_srvs.srv.Trigger` | SERVER | `__init__` | [`src/antbot_navigation/scripts/waypoint_editor.py:77`](../../src/antbot_navigation/scripts/waypoint_editor.py#L77) |
| Service | `'~/cancel'` | `std_srvs.srv.Trigger` | SERVER | `__init__` | [`src/antbot_navigation/scripts/waypoint_editor.py:78`](../../src/antbot_navigation/scripts/waypoint_editor.py#L78) |
| Service | `'~/clear'` | `std_srvs.srv.Empty` | SERVER | `__init__` | [`src/antbot_navigation/scripts/waypoint_editor.py:79`](../../src/antbot_navigation/scripts/waypoint_editor.py#L79) |
| Service | `'~/reload'` | `std_srvs.srv.Trigger` | SERVER | `__init__` | [`src/antbot_navigation/scripts/waypoint_editor.py:80`](../../src/antbot_navigation/scripts/waypoint_editor.py#L80) |

### antbot_rgbd_dataset / OfflinePreviewPublisher

Node Name：`antbot_rgbd_offline_cloud_publisher`；Executable：见 SOURCE_SCAN 入口和 launch 附录；UNKNOWN / NEED_CONFIRMATION；Source：[`src/antbot_rgbd_dataset/antbot_rgbd_dataset/offline_preview_node.py:1`](../../src/antbot_rgbd_dataset/antbot_rgbd_dataset/offline_preview_node.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'preview_path'` | `""` | DEFAULT | `__init__` | [`src/antbot_rgbd_dataset/antbot_rgbd_dataset/offline_preview_node.py:17`](../../src/antbot_rgbd_dataset/antbot_rgbd_dataset/offline_preview_node.py#L17) |
| Parameter | `'publish_topic'` | `"/antbot/rgbd/offline_cloud"` | DEFAULT | `__init__` | [`src/antbot_rgbd_dataset/antbot_rgbd_dataset/offline_preview_node.py:18`](../../src/antbot_rgbd_dataset/antbot_rgbd_dataset/offline_preview_node.py#L18) |
| Parameter | `'publish_rate'` | `0.5` | DEFAULT | `__init__` | [`src/antbot_rgbd_dataset/antbot_rgbd_dataset/offline_preview_node.py:19`](../../src/antbot_rgbd_dataset/antbot_rgbd_dataset/offline_preview_node.py#L19) |
| Topic | `topic` | `sensor_msgs.msg.PointCloud2` | PUB | `__init__` | [`src/antbot_rgbd_dataset/antbot_rgbd_dataset/offline_preview_node.py:30`](../../src/antbot_rgbd_dataset/antbot_rgbd_dataset/offline_preview_node.py#L30) |
| Timer | `1.0 / rate` | `` | CALLBACK | `__init__` | [`src/antbot_rgbd_dataset/antbot_rgbd_dataset/offline_preview_node.py:34`](../../src/antbot_rgbd_dataset/antbot_rgbd_dataset/offline_preview_node.py#L34) |

### antbot_rgbd_dataset / Phase4BCaptureNode

Node Name：`phase4b_capture_node`；Executable：phase4b_capture_node；Source：[`src/antbot_rgbd_dataset/antbot_rgbd_dataset/phase4b_capture_node.py:1`](../../src/antbot_rgbd_dataset/antbot_rgbd_dataset/phase4b_capture_node.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Topic | `'/antbot/rgbd/live_cloud'` | `sensor_msgs.msg.PointCloud2` | PUB | `__init__` | [`src/antbot_rgbd_dataset/antbot_rgbd_dataset/phase4b_capture_node.py:81`](../../src/antbot_rgbd_dataset/antbot_rgbd_dataset/phase4b_capture_node.py#L81) |
| Topic | `'/antbot/rgbd/accumulated_cloud'` | `sensor_msgs.msg.PointCloud2` | PUB | `__init__` | [`src/antbot_rgbd_dataset/antbot_rgbd_dataset/phase4b_capture_node.py:84`](../../src/antbot_rgbd_dataset/antbot_rgbd_dataset/phase4b_capture_node.py#L84) |
| Topic | `'/antbot/rgbd/camera_path'` | `nav_msgs.msg.Path` | PUB | `__init__` | [`src/antbot_rgbd_dataset/antbot_rgbd_dataset/phase4b_capture_node.py:87`](../../src/antbot_rgbd_dataset/antbot_rgbd_dataset/phase4b_capture_node.py#L87) |
| Topic | `'/antbot/rgbd/coverage_markers'` | `visualization_msgs.msg.MarkerArray` | PUB | `__init__` | [`src/antbot_rgbd_dataset/antbot_rgbd_dataset/phase4b_capture_node.py:90`](../../src/antbot_rgbd_dataset/antbot_rgbd_dataset/phase4b_capture_node.py#L90) |
| Topic | `'/antbot/rgbd/coverage_marker_array'` | `visualization_msgs.msg.MarkerArray` | PUB | `__init__` | [`src/antbot_rgbd_dataset/antbot_rgbd_dataset/phase4b_capture_node.py:93`](../../src/antbot_rgbd_dataset/antbot_rgbd_dataset/phase4b_capture_node.py#L93) |
| Topic | `'/antbot/rgbd/status'` | `std_msgs.msg.String` | PUB | `__init__` | [`src/antbot_rgbd_dataset/antbot_rgbd_dataset/phase4b_capture_node.py:96`](../../src/antbot_rgbd_dataset/antbot_rgbd_dataset/phase4b_capture_node.py#L96) |
| Topic | `self.parameters['color_topic']` | `sensor_msgs.msg.Image` | SUB | `__init__` | [`src/antbot_rgbd_dataset/antbot_rgbd_dataset/phase4b_capture_node.py:116`](../../src/antbot_rgbd_dataset/antbot_rgbd_dataset/phase4b_capture_node.py#L116) |
| Topic | `self.parameters['depth_topic']` | `sensor_msgs.msg.Image` | SUB | `__init__` | [`src/antbot_rgbd_dataset/antbot_rgbd_dataset/phase4b_capture_node.py:119`](../../src/antbot_rgbd_dataset/antbot_rgbd_dataset/phase4b_capture_node.py#L119) |
| Topic | `self.parameters['camera_info_topic']` | `sensor_msgs.msg.CameraInfo` | SUB | `__init__` | [`src/antbot_rgbd_dataset/antbot_rgbd_dataset/phase4b_capture_node.py:122`](../../src/antbot_rgbd_dataset/antbot_rgbd_dataset/phase4b_capture_node.py#L122) |
| Topic | `self.parameters['pose_topic']` | `nav_msgs.msg.Odometry` | SUB | `__init__` | [`src/antbot_rgbd_dataset/antbot_rgbd_dataset/phase4b_capture_node.py:128`](../../src/antbot_rgbd_dataset/antbot_rgbd_dataset/phase4b_capture_node.py#L128) |
| Topic | `self.parameters['pose_topic']` | `geometry_msgs.msg.PoseStamped` | SUB | `__init__` | [`src/antbot_rgbd_dataset/antbot_rgbd_dataset/phase4b_capture_node.py:133`](../../src/antbot_rgbd_dataset/antbot_rgbd_dataset/phase4b_capture_node.py#L133) |
| Parameter | `name` | `default` | DEFAULT | `_declare_parameters` | [`src/antbot_rgbd_dataset/antbot_rgbd_dataset/phase4b_capture_node.py:200`](../../src/antbot_rgbd_dataset/antbot_rgbd_dataset/phase4b_capture_node.py#L200) |

### antbot_rgbd_dataset / RgbdKeyframeRecorder

Node Name：`rgbd_keyframe_recorder`；Executable：rgbd_keyframe_recorder；Source：[`src/antbot_rgbd_dataset/antbot_rgbd_dataset/recorder_node.py:1`](../../src/antbot_rgbd_dataset/antbot_rgbd_dataset/recorder_node.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Topic | `self.rgb_topic` | `sensor_msgs.msg.Image` | SUB | `__init__` | [`src/antbot_rgbd_dataset/antbot_rgbd_dataset/recorder_node.py:88`](../../src/antbot_rgbd_dataset/antbot_rgbd_dataset/recorder_node.py#L88) |
| Topic | `self.depth_topic` | `sensor_msgs.msg.Image` | SUB | `__init__` | [`src/antbot_rgbd_dataset/antbot_rgbd_dataset/recorder_node.py:89`](../../src/antbot_rgbd_dataset/antbot_rgbd_dataset/recorder_node.py#L89) |
| Topic | `self.color_info_topic` | `sensor_msgs.msg.CameraInfo` | SUB | `__init__` | [`src/antbot_rgbd_dataset/antbot_rgbd_dataset/recorder_node.py:90`](../../src/antbot_rgbd_dataset/antbot_rgbd_dataset/recorder_node.py#L90) |
| Topic | `self.depth_info_topic` | `sensor_msgs.msg.CameraInfo` | SUB | `__init__` | [`src/antbot_rgbd_dataset/antbot_rgbd_dataset/recorder_node.py:93`](../../src/antbot_rgbd_dataset/antbot_rgbd_dataset/recorder_node.py#L93) |
| Topic | `self.status_topic` | `std_msgs.msg.String` | PUB | `__init__` | [`src/antbot_rgbd_dataset/antbot_rgbd_dataset/recorder_node.py:96`](../../src/antbot_rgbd_dataset/antbot_rgbd_dataset/recorder_node.py#L96) |
| Topic | `self.path_topic` | `nav_msgs.msg.Path` | PUB | `__init__` | [`src/antbot_rgbd_dataset/antbot_rgbd_dataset/recorder_node.py:97`](../../src/antbot_rgbd_dataset/antbot_rgbd_dataset/recorder_node.py#L97) |
| Topic | `self.marker_topic` | `visualization_msgs.msg.MarkerArray` | PUB | `__init__` | [`src/antbot_rgbd_dataset/antbot_rgbd_dataset/recorder_node.py:98`](../../src/antbot_rgbd_dataset/antbot_rgbd_dataset/recorder_node.py#L98) |
| Service | `self.start_service` | `std_srvs.srv.Trigger` | SERVER | `__init__` | [`src/antbot_rgbd_dataset/antbot_rgbd_dataset/recorder_node.py:99`](../../src/antbot_rgbd_dataset/antbot_rgbd_dataset/recorder_node.py#L99) |
| Service | `self.stop_service` | `std_srvs.srv.Trigger` | SERVER | `__init__` | [`src/antbot_rgbd_dataset/antbot_rgbd_dataset/recorder_node.py:100`](../../src/antbot_rgbd_dataset/antbot_rgbd_dataset/recorder_node.py#L100) |
| Service | `self.capture_service` | `std_srvs.srv.Trigger` | SERVER | `__init__` | [`src/antbot_rgbd_dataset/antbot_rgbd_dataset/recorder_node.py:101`](../../src/antbot_rgbd_dataset/antbot_rgbd_dataset/recorder_node.py#L101) |
| Service | `self.status_service` | `std_srvs.srv.Trigger` | SERVER | `__init__` | [`src/antbot_rgbd_dataset/antbot_rgbd_dataset/recorder_node.py:102`](../../src/antbot_rgbd_dataset/antbot_rgbd_dataset/recorder_node.py#L102) |
| Timer | `0.01` | `` | CALLBACK | `__init__` | [`src/antbot_rgbd_dataset/antbot_rgbd_dataset/recorder_node.py:103`](../../src/antbot_rgbd_dataset/antbot_rgbd_dataset/recorder_node.py#L103) |
| Timer | `1.0` | `` | CALLBACK | `__init__` | [`src/antbot_rgbd_dataset/antbot_rgbd_dataset/recorder_node.py:104`](../../src/antbot_rgbd_dataset/antbot_rgbd_dataset/recorder_node.py#L104) |
| Parameter | `name` | `value` | DEFAULT | `_declare_parameters` | [`src/antbot_rgbd_dataset/antbot_rgbd_dataset/recorder_node.py:147`](../../src/antbot_rgbd_dataset/antbot_rgbd_dataset/recorder_node.py#L147) |

### antbot_teleop / MappingKeyboard

Node Name：`antbot_mapping_keyboard`；Executable：mapping_keyboard；Source：[`src/antbot_teleop/antbot_teleop/mapping_keyboard.py:1`](../../src/antbot_teleop/antbot_teleop/mapping_keyboard.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'cmd_vel_topic'` | `'/cmd_vel'` | DEFAULT | `__init__` | [`src/antbot_teleop/antbot_teleop/mapping_keyboard.py:80`](../../src/antbot_teleop/antbot_teleop/mapping_keyboard.py#L80) |
| Parameter | `'map_prefix'` | `''` | DEFAULT | `__init__` | [`src/antbot_teleop/antbot_teleop/mapping_keyboard.py:81`](../../src/antbot_teleop/antbot_teleop/mapping_keyboard.py#L81) |
| Parameter | `'max_linear_vel'` | `0.60` | DEFAULT | `__init__` | [`src/antbot_teleop/antbot_teleop/mapping_keyboard.py:82`](../../src/antbot_teleop/antbot_teleop/mapping_keyboard.py#L82) |
| Parameter | `'max_angular_vel'` | `1.00` | DEFAULT | `__init__` | [`src/antbot_teleop/antbot_teleop/mapping_keyboard.py:83`](../../src/antbot_teleop/antbot_teleop/mapping_keyboard.py#L83) |
| Parameter | `'linear_accel'` | `1.20` | DEFAULT | `__init__` | [`src/antbot_teleop/antbot_teleop/mapping_keyboard.py:84`](../../src/antbot_teleop/antbot_teleop/mapping_keyboard.py#L84) |
| Parameter | `'angular_accel'` | `2.40` | DEFAULT | `__init__` | [`src/antbot_teleop/antbot_teleop/mapping_keyboard.py:85`](../../src/antbot_teleop/antbot_teleop/mapping_keyboard.py#L85) |
| Parameter | `'publish_rate'` | `20.0` | DEFAULT | `__init__` | [`src/antbot_teleop/antbot_teleop/mapping_keyboard.py:86`](../../src/antbot_teleop/antbot_teleop/mapping_keyboard.py#L86) |
| Parameter | `'speed_level'` | `5` | DEFAULT | `__init__` | [`src/antbot_teleop/antbot_teleop/mapping_keyboard.py:87`](../../src/antbot_teleop/antbot_teleop/mapping_keyboard.py#L87) |
| Parameter | `'key_timeout'` | `0.45` | DEFAULT | `__init__` | [`src/antbot_teleop/antbot_teleop/mapping_keyboard.py:88`](../../src/antbot_teleop/antbot_teleop/mapping_keyboard.py#L88) |
| Topic | `topic` | `geometry_msgs.msg.Twist` | PUB | `__init__` | [`src/antbot_teleop/antbot_teleop/mapping_keyboard.py:102`](../../src/antbot_teleop/antbot_teleop/mapping_keyboard.py#L102) |

### antbot_teleop / MappingXbox

Node Name：`antbot_mapping_xbox`；Executable：mapping_xbox；Source：[`src/antbot_teleop/antbot_teleop/mapping_xbox.py:1`](../../src/antbot_teleop/antbot_teleop/mapping_xbox.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Topic | `str(self.get_parameter('topics.cmd_vel').value)` | `geometry_msgs.msg.Twist` | PUB | `__init__` | [`src/antbot_teleop/antbot_teleop/mapping_xbox.py:114`](../../src/antbot_teleop/antbot_teleop/mapping_xbox.py#L114) |
| Topic | `str(self.get_parameter('topics.armed').value)` | `std_msgs.msg.Bool` | PUB | `__init__` | [`src/antbot_teleop/antbot_teleop/mapping_xbox.py:116`](../../src/antbot_teleop/antbot_teleop/mapping_xbox.py#L116) |
| Topic | `str(self.get_parameter('topics.control_target').value)` | `std_msgs.msg.String` | PUB | `__init__` | [`src/antbot_teleop/antbot_teleop/mapping_xbox.py:118`](../../src/antbot_teleop/antbot_teleop/mapping_xbox.py#L118) |
| Topic | `str(self.get_parameter('topics.status').value)` | `std_msgs.msg.String` | PUB | `__init__` | [`src/antbot_teleop/antbot_teleop/mapping_xbox.py:123`](../../src/antbot_teleop/antbot_teleop/mapping_xbox.py#L123) |
| Topic | `str(self.get_parameter('topics.joy').value)` | `sensor_msgs.msg.Joy` | SUB | `__init__` | [`src/antbot_teleop/antbot_teleop/mapping_xbox.py:128`](../../src/antbot_teleop/antbot_teleop/mapping_xbox.py#L128) |
| Service | `str(self.get_parameter('topics.speed_down_service').value)` | `std_srvs.srv.Trigger` | SERVER | `__init__` | [`src/antbot_teleop/antbot_teleop/mapping_xbox.py:134`](../../src/antbot_teleop/antbot_teleop/mapping_xbox.py#L134) |
| Service | `str(self.get_parameter('topics.speed_up_service').value)` | `std_srvs.srv.Trigger` | SERVER | `__init__` | [`src/antbot_teleop/antbot_teleop/mapping_xbox.py:139`](../../src/antbot_teleop/antbot_teleop/mapping_xbox.py#L139) |
| Timer | `1.0 / self.publish_rate` | `` | CALLBACK | `__init__` | [`src/antbot_teleop/antbot_teleop/mapping_xbox.py:155`](../../src/antbot_teleop/antbot_teleop/mapping_xbox.py#L155) |
| Timer | `0.2` | `` | CALLBACK | `__init__` | [`src/antbot_teleop/antbot_teleop/mapping_xbox.py:157`](../../src/antbot_teleop/antbot_teleop/mapping_xbox.py#L157) |
| Parameter | `'axes.left_x'` | `0` | DEFAULT | `_declare_parameters` | [`src/antbot_teleop/antbot_teleop/mapping_xbox.py:167`](../../src/antbot_teleop/antbot_teleop/mapping_xbox.py#L167) |
| Parameter | `'axes.left_y'` | `1` | DEFAULT | `_declare_parameters` | [`src/antbot_teleop/antbot_teleop/mapping_xbox.py:168`](../../src/antbot_teleop/antbot_teleop/mapping_xbox.py#L168) |
| Parameter | `'axes.lt'` | `2` | DEFAULT | `_declare_parameters` | [`src/antbot_teleop/antbot_teleop/mapping_xbox.py:169`](../../src/antbot_teleop/antbot_teleop/mapping_xbox.py#L169) |
| Parameter | `'axes.rt'` | `5` | DEFAULT | `_declare_parameters` | [`src/antbot_teleop/antbot_teleop/mapping_xbox.py:170`](../../src/antbot_teleop/antbot_teleop/mapping_xbox.py#L170) |
| Parameter | `'buttons.safety_lock'` | `0` | DEFAULT | `_declare_parameters` | [`src/antbot_teleop/antbot_teleop/mapping_xbox.py:171`](../../src/antbot_teleop/antbot_teleop/mapping_xbox.py#L171) |
| Parameter | `'buttons.save_map'` | `1` | DEFAULT | `_declare_parameters` | [`src/antbot_teleop/antbot_teleop/mapping_xbox.py:172`](../../src/antbot_teleop/antbot_teleop/mapping_xbox.py#L172) |
| Parameter | `'buttons.save_and_exit'` | `2` | DEFAULT | `_declare_parameters` | [`src/antbot_teleop/antbot_teleop/mapping_xbox.py:173`](../../src/antbot_teleop/antbot_teleop/mapping_xbox.py#L173) |
| Parameter | `'buttons.control_switch'` | `8` | DEFAULT | `_declare_parameters` | [`src/antbot_teleop/antbot_teleop/mapping_xbox.py:174`](../../src/antbot_teleop/antbot_teleop/mapping_xbox.py#L174) |
| Parameter | `'buttons.speed_down'` | `9` | DEFAULT | `_declare_parameters` | [`src/antbot_teleop/antbot_teleop/mapping_xbox.py:175`](../../src/antbot_teleop/antbot_teleop/mapping_xbox.py#L175) |
| Parameter | `'buttons.speed_up'` | `10` | DEFAULT | `_declare_parameters` | [`src/antbot_teleop/antbot_teleop/mapping_xbox.py:176`](../../src/antbot_teleop/antbot_teleop/mapping_xbox.py#L176) |
| Parameter | `'deadzone'` | `0.10` | DEFAULT | `_declare_parameters` | [`src/antbot_teleop/antbot_teleop/mapping_xbox.py:177`](../../src/antbot_teleop/antbot_teleop/mapping_xbox.py#L177) |
| Parameter | `'trigger_deadzone'` | `0.05` | DEFAULT | `_declare_parameters` | [`src/antbot_teleop/antbot_teleop/mapping_xbox.py:178`](../../src/antbot_teleop/antbot_teleop/mapping_xbox.py#L178) |
| Parameter | `'joy_timeout'` | `0.50` | DEFAULT | `_declare_parameters` | [`src/antbot_teleop/antbot_teleop/mapping_xbox.py:179`](../../src/antbot_teleop/antbot_teleop/mapping_xbox.py#L179) |
| Parameter | `'publish_rate'` | `20.0` | DEFAULT | `_declare_parameters` | [`src/antbot_teleop/antbot_teleop/mapping_xbox.py:180`](../../src/antbot_teleop/antbot_teleop/mapping_xbox.py#L180) |
| Parameter | `'max_linear_vel'` | `1.50` | DEFAULT | `_declare_parameters` | [`src/antbot_teleop/antbot_teleop/mapping_xbox.py:181`](../../src/antbot_teleop/antbot_teleop/mapping_xbox.py#L181) |
| Parameter | `'max_angular_vel'` | `1.00` | DEFAULT | `_declare_parameters` | [`src/antbot_teleop/antbot_teleop/mapping_xbox.py:182`](../../src/antbot_teleop/antbot_teleop/mapping_xbox.py#L182) |
| Parameter | `'speed_levels'` | `[0.10, 0.25, 0.50, 0.75, 1.00]` | DEFAULT | `_declare_parameters` | [`src/antbot_teleop/antbot_teleop/mapping_xbox.py:183`](../../src/antbot_teleop/antbot_teleop/mapping_xbox.py#L183) |
| Parameter | `'initial_speed_level'` | `1` | DEFAULT | `_declare_parameters` | [`src/antbot_teleop/antbot_teleop/mapping_xbox.py:184`](../../src/antbot_teleop/antbot_teleop/mapping_xbox.py#L184) |
| Parameter | `'map_prefix'` | `default_map` | DEFAULT | `_declare_parameters` | [`src/antbot_teleop/antbot_teleop/mapping_xbox.py:185`](../../src/antbot_teleop/antbot_teleop/mapping_xbox.py#L185) |
| Parameter | `'map_use_sim_time'` | `True` | DEFAULT | `_declare_parameters` | [`src/antbot_teleop/antbot_teleop/mapping_xbox.py:186`](../../src/antbot_teleop/antbot_teleop/mapping_xbox.py#L186) |
| Parameter | `'shutdown_zero_frames'` | `5` | DEFAULT | `_declare_parameters` | [`src/antbot_teleop/antbot_teleop/mapping_xbox.py:187`](../../src/antbot_teleop/antbot_teleop/mapping_xbox.py#L187) |
| Parameter | `'topics.joy'` | `'/joy'` | DEFAULT | `_declare_parameters` | [`src/antbot_teleop/antbot_teleop/mapping_xbox.py:188`](../../src/antbot_teleop/antbot_teleop/mapping_xbox.py#L188) |
| Parameter | `'topics.cmd_vel'` | `'/cmd_vel'` | DEFAULT | `_declare_parameters` | [`src/antbot_teleop/antbot_teleop/mapping_xbox.py:189`](../../src/antbot_teleop/antbot_teleop/mapping_xbox.py#L189) |
| Parameter | `'topics.armed'` | `'/antbot_xbox/armed'` | DEFAULT | `_declare_parameters` | [`src/antbot_teleop/antbot_teleop/mapping_xbox.py:190`](../../src/antbot_teleop/antbot_teleop/mapping_xbox.py#L190) |
| Parameter | `'topics.status'` | `'/antbot_xbox/status'` | DEFAULT | `_declare_parameters` | [`src/antbot_teleop/antbot_teleop/mapping_xbox.py:191`](../../src/antbot_teleop/antbot_teleop/mapping_xbox.py#L191) |
| Parameter | `'topics.speed_down_service'` | `'/antbot_xbox/speed_down'` | DEFAULT | `_declare_parameters` | [`src/antbot_teleop/antbot_teleop/mapping_xbox.py:192`](../../src/antbot_teleop/antbot_teleop/mapping_xbox.py#L192) |
| Parameter | `'topics.speed_up_service'` | `'/antbot_xbox/speed_up'` | DEFAULT | `_declare_parameters` | [`src/antbot_teleop/antbot_teleop/mapping_xbox.py:194`](../../src/antbot_teleop/antbot_teleop/mapping_xbox.py#L194) |
| Parameter | `'topics.control_target'` | `'/xbox/control_target'` | DEFAULT | `_declare_parameters` | [`src/antbot_teleop/antbot_teleop/mapping_xbox.py:196`](../../src/antbot_teleop/antbot_teleop/mapping_xbox.py#L196) |

### antbot_teleop / SwerveSimNode

Node Name：`swerve_sim`；Executable：swerve_sim；Source：[`src/antbot_teleop/antbot_teleop/swerve_sim.py:1`](../../src/antbot_teleop/antbot_teleop/swerve_sim.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Topic | `'cmd_vel'` | `geometry_msgs.msg.Twist` | SUB | `__init__` | [`src/antbot_teleop/antbot_teleop/swerve_sim.py:64`](../../src/antbot_teleop/antbot_teleop/swerve_sim.py#L64) |

### antbot_teleop / TeleopJoystickNode

Node Name：`teleop_joystick`；Executable：teleop_joystick；Source：[`src/antbot_teleop/antbot_teleop/teleop_joystick.py:1`](../../src/antbot_teleop/antbot_teleop/teleop_joystick.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'max_linear_vel'` | `1.5` | DEFAULT | `__init__` | [`src/antbot_teleop/antbot_teleop/teleop_joystick.py:57`](../../src/antbot_teleop/antbot_teleop/teleop_joystick.py#L57) |
| Parameter | `'max_spin_vel'` | `1.5` | DEFAULT | `__init__` | [`src/antbot_teleop/antbot_teleop/teleop_joystick.py:58`](../../src/antbot_teleop/antbot_teleop/teleop_joystick.py#L58) |
| Parameter | `'speed_level'` | `3` | DEFAULT | `__init__` | [`src/antbot_teleop/antbot_teleop/teleop_joystick.py:59`](../../src/antbot_teleop/antbot_teleop/teleop_joystick.py#L59) |
| Parameter | `'deadzone'` | `0.1` | DEFAULT | `__init__` | [`src/antbot_teleop/antbot_teleop/teleop_joystick.py:60`](../../src/antbot_teleop/antbot_teleop/teleop_joystick.py#L60) |
| Parameter | `'module_x'` | `0.265` | DEFAULT | `__init__` | [`src/antbot_teleop/antbot_teleop/teleop_joystick.py:61`](../../src/antbot_teleop/antbot_teleop/teleop_joystick.py#L61) |
| Parameter | `'module_y'` | `0.256` | DEFAULT | `__init__` | [`src/antbot_teleop/antbot_teleop/teleop_joystick.py:62`](../../src/antbot_teleop/antbot_teleop/teleop_joystick.py#L62) |
| Parameter | `'steering_limit_deg'` | `60.0` | DEFAULT | `__init__` | [`src/antbot_teleop/antbot_teleop/teleop_joystick.py:63`](../../src/antbot_teleop/antbot_teleop/teleop_joystick.py#L63) |
| Parameter | `'safety_factor'` | `0.95` | DEFAULT | `__init__` | [`src/antbot_teleop/antbot_teleop/teleop_joystick.py:64`](../../src/antbot_teleop/antbot_teleop/teleop_joystick.py#L64) |
| Parameter | `'w_abs_max'` | `2.0` | DEFAULT | `__init__` | [`src/antbot_teleop/antbot_teleop/teleop_joystick.py:65`](../../src/antbot_teleop/antbot_teleop/teleop_joystick.py#L65) |
| Topic | `'cmd_vel'` | `geometry_msgs.msg.Twist` | PUB | `__init__` | [`src/antbot_teleop/antbot_teleop/teleop_joystick.py:110`](../../src/antbot_teleop/antbot_teleop/teleop_joystick.py#L110) |
| Service | `'cargo/command'` | `antbot_interfaces.srv.CargoCommand` | CLIENT | `__init__` | [`src/antbot_teleop/antbot_teleop/teleop_joystick.py:111`](../../src/antbot_teleop/antbot_teleop/teleop_joystick.py#L111) |
| Service | `'headlight/operation'` | `std_srvs.srv.SetBool` | CLIENT | `__init__` | [`src/antbot_teleop/antbot_teleop/teleop_joystick.py:112`](../../src/antbot_teleop/antbot_teleop/teleop_joystick.py#L112) |
| Service | `'wiper/operation'` | `antbot_interfaces.srv.WiperOperation` | CLIENT | `__init__` | [`src/antbot_teleop/antbot_teleop/teleop_joystick.py:114`](../../src/antbot_teleop/antbot_teleop/teleop_joystick.py#L114) |
| Topic | `'joy'` | `sensor_msgs.msg.Joy` | SUB | `__init__` | [`src/antbot_teleop/antbot_teleop/teleop_joystick.py:116`](../../src/antbot_teleop/antbot_teleop/teleop_joystick.py#L116) |

### antbot_teleop / TeleopKeyboardNode

Node Name：`teleop_keyboard`；Executable：teleop_keyboard；Source：[`src/antbot_teleop/antbot_teleop/teleop_keyboard.py:1`](../../src/antbot_teleop/antbot_teleop/teleop_keyboard.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'max_linear_vel'` | `1.0` | DEFAULT | `__init__` | [`src/antbot_teleop/antbot_teleop/teleop_keyboard.py:49`](../../src/antbot_teleop/antbot_teleop/teleop_keyboard.py#L49) |
| Parameter | `'max_angular_vel'` | `1.0` | DEFAULT | `__init__` | [`src/antbot_teleop/antbot_teleop/teleop_keyboard.py:50`](../../src/antbot_teleop/antbot_teleop/teleop_keyboard.py#L50) |
| Parameter | `'speed_level'` | `5` | DEFAULT | `__init__` | [`src/antbot_teleop/antbot_teleop/teleop_keyboard.py:51`](../../src/antbot_teleop/antbot_teleop/teleop_keyboard.py#L51) |
| Parameter | `'publish_rate'` | `10.0` | DEFAULT | `__init__` | [`src/antbot_teleop/antbot_teleop/teleop_keyboard.py:52`](../../src/antbot_teleop/antbot_teleop/teleop_keyboard.py#L52) |
| Topic | `'cmd_vel'` | `geometry_msgs.msg.Twist` | PUB | `__init__` | [`src/antbot_teleop/antbot_teleop/teleop_keyboard.py:63`](../../src/antbot_teleop/antbot_teleop/teleop_keyboard.py#L63) |

### antbot_teleop / SmoothTeleop

Node Name：`antbot_teleop_smooth`；Executable：teleop_smooth；Source：[`src/antbot_teleop/antbot_teleop/teleop_smooth.py:1`](../../src/antbot_teleop/antbot_teleop/teleop_smooth.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'cmd_vel_topic'` | `'/cmd_vel'` | DEFAULT | `__init__` | [`src/antbot_teleop/antbot_teleop/teleop_smooth.py:56`](../../src/antbot_teleop/antbot_teleop/teleop_smooth.py#L56) |
| Parameter | `'max_linear_vel'` | `0.5` | DEFAULT | `__init__` | [`src/antbot_teleop/antbot_teleop/teleop_smooth.py:57`](../../src/antbot_teleop/antbot_teleop/teleop_smooth.py#L57) |
| Parameter | `'max_angular_vel'` | `1.0` | DEFAULT | `__init__` | [`src/antbot_teleop/antbot_teleop/teleop_smooth.py:58`](../../src/antbot_teleop/antbot_teleop/teleop_smooth.py#L58) |
| Parameter | `'linear_accel'` | `0.8` | DEFAULT | `__init__` | [`src/antbot_teleop/antbot_teleop/teleop_smooth.py:59`](../../src/antbot_teleop/antbot_teleop/teleop_smooth.py#L59) |
| Parameter | `'angular_accel'` | `1.8` | DEFAULT | `__init__` | [`src/antbot_teleop/antbot_teleop/teleop_smooth.py:60`](../../src/antbot_teleop/antbot_teleop/teleop_smooth.py#L60) |
| Parameter | `'publish_rate'` | `20.0` | DEFAULT | `__init__` | [`src/antbot_teleop/antbot_teleop/teleop_smooth.py:61`](../../src/antbot_teleop/antbot_teleop/teleop_smooth.py#L61) |
| Parameter | `'speed_level'` | `5` | DEFAULT | `__init__` | [`src/antbot_teleop/antbot_teleop/teleop_smooth.py:62`](../../src/antbot_teleop/antbot_teleop/teleop_smooth.py#L62) |
| Parameter | `'key_timeout'` | `0.45` | DEFAULT | `__init__` | [`src/antbot_teleop/antbot_teleop/teleop_smooth.py:63`](../../src/antbot_teleop/antbot_teleop/teleop_smooth.py#L63) |
| Topic | `topic` | `geometry_msgs.msg.Twist` | PUB | `__init__` | [`src/antbot_teleop/antbot_teleop/teleop_smooth.py:74`](../../src/antbot_teleop/antbot_teleop/teleop_smooth.py#L74) |

### easy_handeye2 / HandeyeCalibrationParametersProvider

Node Name：`helper / MODULE / derived node；NEED_CONFIRMATION`；Executable：见 SOURCE_SCAN 入口和 launch 附录；UNKNOWN / NEED_CONFIRMATION；Source：[`dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_calibration.py:1`](../../dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_calibration.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'name'` | `'', descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING)` | DEFAULT | `__init__` | [`dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_calibration.py:20`](../../dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_calibration.py#L20) |
| Parameter | `'calibration_type'` | `'', descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING)` | DEFAULT | `__init__` | [`dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_calibration.py:21`](../../dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_calibration.py#L21) |
| Parameter | `'robot_base_frame'` | `'', descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING)` | DEFAULT | `__init__` | [`dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_calibration.py:22`](../../dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_calibration.py#L22) |
| Parameter | `'robot_effector_frame'` | `'', descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING)` | DEFAULT | `__init__` | [`dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_calibration.py:23`](../../dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_calibration.py#L23) |
| Parameter | `'tracking_base_frame'` | `'', descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING)` | DEFAULT | `__init__` | [`dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_calibration.py:24`](../../dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_calibration.py#L24) |
| Parameter | `'tracking_marker_frame'` | `'', descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING)` | DEFAULT | `__init__` | [`dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_calibration.py:25`](../../dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_calibration.py#L25) |
| Parameter | `'freehand_robot_movement'` | `True` | DEFAULT | `__init__` | [`dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_calibration.py:26`](../../dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_calibration.py#L26) |

### easy_handeye2 / HandeyeClient

Node Name：`helper / MODULE / derived node；NEED_CONFIRMATION`；Executable：见 SOURCE_SCAN 入口和 launch 附录；UNKNOWN / NEED_CONFIRMATION；Source：[`dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_client.py:1`](../../dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_client.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Service | `hec.GET_SAMPLE_LIST_TOPIC` | `ehm.srv.TakeSample` | CLIENT | `__init__` | [`dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_client.py:19`](../../dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_client.py#L19) |
| Service | `hec.GET_CURRENT_TRANSFORMS_TOPIC` | `ehm.srv.TakeSample` | CLIENT | `__init__` | [`dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_client.py:21`](../../dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_client.py#L21) |
| Service | `hec.TAKE_SAMPLE_TOPIC` | `ehm.srv.TakeSample` | CLIENT | `__init__` | [`dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_client.py:23`](../../dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_client.py#L23) |
| Service | `hec.REMOVE_SAMPLE_TOPIC` | `ehm.srv.RemoveSample` | CLIENT | `__init__` | [`dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_client.py:25`](../../dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_client.py#L25) |
| Service | `hec.LIST_ALGORITHMS_TOPIC` | `ehm.srv.ListAlgorithms` | CLIENT | `__init__` | [`dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_client.py:30`](../../dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_client.py#L30) |
| Service | `hec.SET_ALGORITHM_TOPIC` | `ehm.srv.SetAlgorithm` | CLIENT | `__init__` | [`dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_client.py:32`](../../dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_client.py#L32) |
| Service | `hec.COMPUTE_CALIBRATION_TOPIC` | `ehm.srv.ComputeCalibration` | CLIENT | `__init__` | [`dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_client.py:34`](../../dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_client.py#L34) |
| Service | `hec.SAVE_CALIBRATION_TOPIC` | `ehm.srv.SaveCalibration` | CLIENT | `__init__` | [`dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_client.py:37`](../../dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_client.py#L37) |
| Service | `hec.CHECK_STARTING_POSE_TOPIC` | `ehm.srv.CheckStartingPose` | CLIENT | `__init__` | [`dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_client.py:44`](../../dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_client.py#L44) |
| Service | `hec.ENUMERATE_TARGET_POSES_TOPIC` | `ehm.srv.EnumerateTargetPoses` | CLIENT | `__init__` | [`dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_client.py:48`](../../dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_client.py#L48) |
| Service | `hec.SELECT_TARGET_POSE_TOPIC` | `ehm.srv.SelectTargetPose` | CLIENT | `__init__` | [`dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_client.py:52`](../../dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_client.py#L52) |
| Service | `hec.PLAN_TO_SELECTED_TARGET_POSE_TOPIC` | `ehm.srv.PlanToSelectedTargetPose` | CLIENT | `__init__` | [`dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_client.py:56`](../../dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_client.py#L56) |
| Service | `hec.EXECUTE_PLAN_TOPIC` | `ehm.srv.ExecutePlan` | CLIENT | `__init__` | [`dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_client.py:60`](../../dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_client.py#L60) |

### easy_handeye2 / HandeyePublisher

Node Name：`handeye_publisher`；Executable：handeye_publisher；Source：[`dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_publisher.py:1`](../../dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_publisher.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'name'` | `''` | DEFAULT | `__init__` | [`dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_publisher.py:15`](../../dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_publisher.py#L15) |

### easy_handeye2 / RqtHandeyeEvaluatorWidget

Node Name：`helper / MODULE / derived node；NEED_CONFIRMATION`；Executable：见 SOURCE_SCAN 入口和 launch 附录；UNKNOWN / NEED_CONFIRMATION；Source：[`dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_rqt_evaluator_widget.py:1`](../../dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_rqt_evaluator_widget.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'name'` | `''` | DEFAULT | `__init__` | [`dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_rqt_evaluator_widget.py:30`](../../dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_rqt_evaluator_widget.py#L30) |

### easy_handeye2 / HandeyeServer

Node Name：`handeye_server`；Executable：handeye_server；Source：[`dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_server.py:1`](../../dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_server.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Timer | `2.0` | `` | CALLBACK | `__init__` | [`dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_server.py:27`](../../dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_server.py#L27) |
| Service | `hec.LIST_ALGORITHMS_TOPIC` | `ehm.srv.ListAlgorithms` | SERVER | `setup_services_and_topics` | [`dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_server.py:53`](../../dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_server.py#L53) |
| Service | `hec.SET_ALGORITHM_TOPIC` | `ehm.srv.SetAlgorithm` | SERVER | `setup_services_and_topics` | [`dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_server.py:55`](../../dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_server.py#L55) |
| Service | `hec.GET_CURRENT_TRANSFORMS_TOPIC` | `ehm.srv.TakeSample` | SERVER | `setup_services_and_topics` | [`dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_server.py:57`](../../dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_server.py#L57) |
| Service | `hec.GET_SAMPLE_LIST_TOPIC` | `ehm.srv.TakeSample` | SERVER | `setup_services_and_topics` | [`dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_server.py:59`](../../dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_server.py#L59) |
| Service | `hec.TAKE_SAMPLE_TOPIC` | `ehm.srv.TakeSample` | SERVER | `setup_services_and_topics` | [`dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_server.py:61`](../../dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_server.py#L61) |
| Service | `hec.REMOVE_SAMPLE_TOPIC` | `ehm.srv.RemoveSample` | SERVER | `setup_services_and_topics` | [`dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_server.py:62`](../../dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_server.py#L62) |
| Service | `hec.SAVE_SAMPLES_TOPIC` | `ehm.srv.SaveSamples` | SERVER | `setup_services_and_topics` | [`dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_server.py:64`](../../dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_server.py#L64) |
| Service | `hec.LOAD_SAMPLES_TOPIC` | `ehm.srv.LoadSamples` | SERVER | `setup_services_and_topics` | [`dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_server.py:66`](../../dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_server.py#L66) |
| Service | `hec.COMPUTE_CALIBRATION_TOPIC` | `ehm.srv.ComputeCalibration` | SERVER | `setup_services_and_topics` | [`dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_server.py:68`](../../dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_server.py#L68) |
| Service | `hec.SAVE_CALIBRATION_TOPIC` | `ehm.srv.SaveCalibration` | SERVER | `setup_services_and_topics` | [`dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_server.py:70`](../../dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_server.py#L70) |
| Topic | `hec.TAKE_SAMPLE_TOPIC` | `std_msgs.msg.Empty` | SUB | `setup_services_and_topics` | [`dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_server.py:74`](../../dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_server.py#L74) |
| Topic | `hec.REMOVE_SAMPLE_TOPIC` | `std_msgs.msg.Empty` | SUB | `setup_services_and_topics` | [`dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_server.py:76`](../../dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_server.py#L76) |

### easy_handeye2 / HandeyeServerRobot

Node Name：`handeye_server_robot`；Executable：handeye_server_robot；Source：[`dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_server_robot.py:1`](../../dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_server_robot.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'rotation_delta_degrees'` | `25` | DEFAULT | `__init__` | [`dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_server_robot.py:21`](../../dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_server_robot.py#L21) |
| Parameter | `'translation_delta_meters'` | `0.1` | DEFAULT | `__init__` | [`dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_server_robot.py:22`](../../dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_server_robot.py#L22) |
| Parameter | `'max_velocity_scaling'` | `0.5` | DEFAULT | `__init__` | [`dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_server_robot.py:23`](../../dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_server_robot.py#L23) |
| Parameter | `'max_acceleration_scaling'` | `0.5` | DEFAULT | `__init__` | [`dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_server_robot.py:24`](../../dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_server_robot.py#L24) |
| Service | `hec.CHECK_STARTING_POSE_TOPIC` | `ehm.srv.CheckStartingPose` | SERVER | `__init__` | [`dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_server_robot.py:37`](../../dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_server_robot.py#L37) |
| Service | `hec.ENUMERATE_TARGET_POSES_TOPIC` | `ehm.srv.EnumerateTargetPoses` | SERVER | `__init__` | [`dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_server_robot.py:38`](../../dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_server_robot.py#L38) |
| Service | `hec.SELECT_TARGET_POSE_TOPIC` | `ehm.srv.SelectTargetPose` | SERVER | `__init__` | [`dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_server_robot.py:39`](../../dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_server_robot.py#L39) |
| Service | `hec.PLAN_TO_SELECTED_TARGET_POSE_TOPIC` | `ehm.srv.PlanToSelectedTargetPose` | SERVER | `__init__` | [`dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_server_robot.py:40`](../../dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_server_robot.py#L40) |
| Service | `hec.EXECUTE_PLAN_TOPIC` | `ehm.srv.ExecutePlan` | SERVER | `__init__` | [`dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_server_robot.py:41`](../../dual_arm_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/handeye_server_robot.py#L41) |

### meridian_hand_vision / RgbDepthSubscriber

Node Name：`meridian_hand_depth_viewer`；Executable：hand_depth_viewer；Source：[`dual_arm_ws/src/meridian_hand_vision/meridian_hand_vision/hand_depth_viewer.py:1`](../../dual_arm_ws/src/meridian_hand_vision/meridian_hand_vision/hand_depth_viewer.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Topic | `color_topic` | `sensor_msgs.msg.Image` | SUB | `__init__` | [`dual_arm_ws/src/meridian_hand_vision/meridian_hand_vision/hand_depth_viewer.py:121`](../../dual_arm_ws/src/meridian_hand_vision/meridian_hand_vision/hand_depth_viewer.py#L121) |
| Topic | `depth_topic` | `sensor_msgs.msg.Image` | SUB | `__init__` | [`dual_arm_ws/src/meridian_hand_vision/meridian_hand_vision/hand_depth_viewer.py:124`](../../dual_arm_ws/src/meridian_hand_vision/meridian_hand_vision/hand_depth_viewer.py#L124) |
| Topic | `camera_info_topic` | `sensor_msgs.msg.CameraInfo` | SUB | `__init__` | [`dual_arm_ws/src/meridian_hand_vision/meridian_hand_vision/hand_depth_viewer.py:127`](../../dual_arm_ws/src/meridian_hand_vision/meridian_hand_vision/hand_depth_viewer.py#L127) |
| Topic | `gesture_topic` | `std_msgs.msg.String` | PUB | `__init__` | [`dual_arm_ws/src/meridian_hand_vision/meridian_hand_vision/hand_depth_viewer.py:131`](../../dual_arm_ws/src/meridian_hand_vision/meridian_hand_vision/hand_depth_viewer.py#L131) |
| Topic | `annotated_topic` | `sensor_msgs.msg.Image` | PUB | `__init__` | [`dual_arm_ws/src/meridian_hand_vision/meridian_hand_vision/hand_depth_viewer.py:132`](../../dual_arm_ws/src/meridian_hand_vision/meridian_hand_vision/hand_depth_viewer.py#L132) |
| Topic | `pulse_topic` | `geometry_msgs.msg.PointStamped` | PUB | `__init__` | [`dual_arm_ws/src/meridian_hand_vision/meridian_hand_vision/hand_depth_viewer.py:133`](../../dual_arm_ws/src/meridian_hand_vision/meridian_hand_vision/hand_depth_viewer.py#L133) |
| Topic | `pulse_status_topic` | `std_msgs.msg.String` | PUB | `__init__` | [`dual_arm_ws/src/meridian_hand_vision/meridian_hand_vision/hand_depth_viewer.py:134`](../../dual_arm_ws/src/meridian_hand_vision/meridian_hand_vision/hand_depth_viewer.py#L134) |
| Topic | `pulse_marker_topic` | `visualization_msgs.msg.Marker` | PUB | `__init__` | [`dual_arm_ws/src/meridian_hand_vision/meridian_hand_vision/hand_depth_viewer.py:135`](../../dual_arm_ws/src/meridian_hand_vision/meridian_hand_vision/hand_depth_viewer.py#L135) |
| Topic | `arm_direction_topic` | `geometry_msgs.msg.Vector3Stamped` | PUB | `__init__` | [`dual_arm_ws/src/meridian_hand_vision/meridian_hand_vision/hand_depth_viewer.py:136`](../../dual_arm_ws/src/meridian_hand_vision/meridian_hand_vision/hand_depth_viewer.py#L136) |

### orbbec_camera / CameraMonitorNode

Node Name：`camera_monitor_node`；Executable：见 SOURCE_SCAN 入口和 launch 附录；UNKNOWN / NEED_CONFIRMATION；Source：[`dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts/common_benchmark_node.py:1`](../../dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts/common_benchmark_node.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Topic | `f'{ns}/device_status'` | `orbbec_camera_msgs.msg.DeviceStatus` | SUB | `__init__` | [`dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts/common_benchmark_node.py:156`](../../dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts/common_benchmark_node.py#L156) |
| Topic | `f'{ns}/color/image_raw'` | `sensor_msgs.msg.Image` | SUB | `__init__` | [`dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts/common_benchmark_node.py:162`](../../dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts/common_benchmark_node.py#L162) |
| Topic | `f'{ns}/depth/image_raw'` | `sensor_msgs.msg.Image` | SUB | `__init__` | [`dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts/common_benchmark_node.py:168`](../../dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts/common_benchmark_node.py#L168) |
| Timer | `1.0` | `` | CALLBACK | `__init__` | [`dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts/common_benchmark_node.py:176`](../../dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts/common_benchmark_node.py#L176) |

### orbbec_camera / TestNode

Node Name：`test_node`；Executable：见 SOURCE_SCAN 入口和 launch 附录；UNKNOWN / NEED_CONFIRMATION；Source：[`dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts/receive_pc.py:1`](../../dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts/receive_pc.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Topic | `'/camera/depth/points'` | `sensor_msgs.msg.PointCloud2` | SUB | `__init__` | [`dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts/receive_pc.py:11`](../../dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts/receive_pc.py#L11) |

### orbbec_camera / ServiceBenchmark

Node Name：`helper / MODULE / derived node；NEED_CONFIRMATION`；Executable：见 SOURCE_SCAN 入口和 launch 附录；UNKNOWN / NEED_CONFIRMATION；Source：[`dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts/service_benchmark_node.py:1`](../../dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts/service_benchmark_node.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Service | `self.service_name` | `self.ServiceClass` | CLIENT | `__init__` | [`dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts/service_benchmark_node.py:42`](../../dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts/service_benchmark_node.py#L42) |

### piper / PiperRosNode

Node Name：`piper_ctrl_single_node`；Executable：piper_single_ctrl；Source：[`dual_arm_ws/src/piper/piper/piper_ctrl_single_node.py:1`](../../dual_arm_ws/src/piper/piper/piper_ctrl_single_node.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'can_port'` | `'can0'` | DEFAULT | `__init__` | [`dual_arm_ws/src/piper/piper/piper_ctrl_single_node.py:36`](../../dual_arm_ws/src/piper/piper/piper_ctrl_single_node.py#L36) |
| Parameter | `'auto_enable'` | `False` | DEFAULT | `__init__` | [`dual_arm_ws/src/piper/piper/piper_ctrl_single_node.py:37`](../../dual_arm_ws/src/piper/piper/piper_ctrl_single_node.py#L37) |
| Parameter | `'gripper_exist'` | `True` | DEFAULT | `__init__` | [`dual_arm_ws/src/piper/piper/piper_ctrl_single_node.py:38`](../../dual_arm_ws/src/piper/piper/piper_ctrl_single_node.py#L38) |
| Parameter | `'gripper_val_mutiple'` | `1` | DEFAULT | `__init__` | [`dual_arm_ws/src/piper/piper/piper_ctrl_single_node.py:39`](../../dual_arm_ws/src/piper/piper/piper_ctrl_single_node.py#L39) |
| Topic | `'joint_states_single'` | `sensor_msgs.msg.JointState` | PUB | `__init__` | [`dual_arm_ws/src/piper/piper/piper_ctrl_single_node.py:52`](../../dual_arm_ws/src/piper/piper/piper_ctrl_single_node.py#L52) |
| Topic | `'joint_states_feedback'` | `sensor_msgs.msg.JointState` | PUB | `__init__` | [`dual_arm_ws/src/piper/piper/piper_ctrl_single_node.py:53`](../../dual_arm_ws/src/piper/piper/piper_ctrl_single_node.py#L53) |
| Topic | `'joint_ctrl'` | `sensor_msgs.msg.JointState` | PUB | `__init__` | [`dual_arm_ws/src/piper/piper/piper_ctrl_single_node.py:54`](../../dual_arm_ws/src/piper/piper/piper_ctrl_single_node.py#L54) |
| Topic | `'arm_status'` | `piper_msgs.msg.PiperStatusMsg` | PUB | `__init__` | [`dual_arm_ws/src/piper/piper/piper_ctrl_single_node.py:55`](../../dual_arm_ws/src/piper/piper/piper_ctrl_single_node.py#L55) |
| Topic | `'end_pose'` | `geometry_msgs.msg.Pose` | PUB | `__init__` | [`dual_arm_ws/src/piper/piper/piper_ctrl_single_node.py:56`](../../dual_arm_ws/src/piper/piper/piper_ctrl_single_node.py#L56) |
| Topic | `'end_pose_stamped'` | `geometry_msgs.msg.PoseStamped` | PUB | `__init__` | [`dual_arm_ws/src/piper/piper/piper_ctrl_single_node.py:57`](../../dual_arm_ws/src/piper/piper/piper_ctrl_single_node.py#L57) |
| Topic | `'teach_joint_states_raw'` | `piper_msgs.msg.PiperTeachJointState` | PUB | `__init__` | [`dual_arm_ws/src/piper/piper/piper_ctrl_single_node.py:58`](../../dual_arm_ws/src/piper/piper/piper_ctrl_single_node.py#L58) |
| Service | `'enable_srv'` | `piper_msgs.srv.Enable` | SERVER | `__init__` | [`dual_arm_ws/src/piper/piper/piper_ctrl_single_node.py:61`](../../dual_arm_ws/src/piper/piper/piper_ctrl_single_node.py#L61) |
| Topic | `'pos_cmd'` | `piper_msgs.msg.PosCmd` | SUB | `__init__` | [`dual_arm_ws/src/piper/piper/piper_ctrl_single_node.py:101`](../../dual_arm_ws/src/piper/piper/piper_ctrl_single_node.py#L101) |
| Topic | `'joint_ctrl_single'` | `sensor_msgs.msg.JointState` | SUB | `__init__` | [`dual_arm_ws/src/piper/piper/piper_ctrl_single_node.py:102`](../../dual_arm_ws/src/piper/piper/piper_ctrl_single_node.py#L102) |
| Topic | `'enable_flag'` | `std_msgs.msg.Bool` | SUB | `__init__` | [`dual_arm_ws/src/piper/piper/piper_ctrl_single_node.py:103`](../../dual_arm_ws/src/piper/piper/piper_ctrl_single_node.py#L103) |

### piper / PiperRosNode

Node Name：`piper_ctrl_single_node`；Executable：见 SOURCE_SCAN 入口和 launch 附录；UNKNOWN / NEED_CONFIRMATION；Source：[`dual_arm_ws/src/piper/piper/piper_ctrl_single_node_new.py:1`](../../dual_arm_ws/src/piper/piper/piper_ctrl_single_node_new.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'can_port'` | `'can0'` | DEFAULT | `__init__` | [`dual_arm_ws/src/piper/piper/piper_ctrl_single_node_new.py:27`](../../dual_arm_ws/src/piper/piper/piper_ctrl_single_node_new.py#L27) |
| Parameter | `'auto_enable'` | `False` | DEFAULT | `__init__` | [`dual_arm_ws/src/piper/piper/piper_ctrl_single_node_new.py:28`](../../dual_arm_ws/src/piper/piper/piper_ctrl_single_node_new.py#L28) |
| Parameter | `'gripper_exist'` | `True` | DEFAULT | `__init__` | [`dual_arm_ws/src/piper/piper/piper_ctrl_single_node_new.py:29`](../../dual_arm_ws/src/piper/piper/piper_ctrl_single_node_new.py#L29) |
| Parameter | `'gripper_val_mutiple'` | `1` | DEFAULT | `__init__` | [`dual_arm_ws/src/piper/piper/piper_ctrl_single_node_new.py:30`](../../dual_arm_ws/src/piper/piper/piper_ctrl_single_node_new.py#L30) |
| Topic | `'joint_states_feedback'` | `sensor_msgs.msg.JointState` | PUB | `__init__` | [`dual_arm_ws/src/piper/piper/piper_ctrl_single_node_new.py:43`](../../dual_arm_ws/src/piper/piper/piper_ctrl_single_node_new.py#L43) |
| Topic | `'joint_ctrl_states_feedback'` | `sensor_msgs.msg.JointState` | PUB | `__init__` | [`dual_arm_ws/src/piper/piper/piper_ctrl_single_node_new.py:44`](../../dual_arm_ws/src/piper/piper/piper_ctrl_single_node_new.py#L44) |
| Topic | `'arm_status'` | `piper_msgs.msg.PiperStatusMsg` | PUB | `__init__` | [`dual_arm_ws/src/piper/piper/piper_ctrl_single_node_new.py:45`](../../dual_arm_ws/src/piper/piper/piper_ctrl_single_node_new.py#L45) |
| Topic | `'end_pose'` | `geometry_msgs.msg.Pose` | PUB | `__init__` | [`dual_arm_ws/src/piper/piper/piper_ctrl_single_node_new.py:46`](../../dual_arm_ws/src/piper/piper/piper_ctrl_single_node_new.py#L46) |
| Topic | `'end_pose_stamped'` | `geometry_msgs.msg.PoseStamped` | PUB | `__init__` | [`dual_arm_ws/src/piper/piper/piper_ctrl_single_node_new.py:47`](../../dual_arm_ws/src/piper/piper/piper_ctrl_single_node_new.py#L47) |
| Service | `'enable_srv'` | `piper_msgs.srv.Enable` | SERVER | `__init__` | [`dual_arm_ws/src/piper/piper/piper_ctrl_single_node_new.py:49`](../../dual_arm_ws/src/piper/piper/piper_ctrl_single_node_new.py#L49) |
| Topic | `'pos_cmd'` | `piper_msgs.msg.PosCmd` | SUB | `__init__` | [`dual_arm_ws/src/piper/piper/piper_ctrl_single_node_new.py:75`](../../dual_arm_ws/src/piper/piper/piper_ctrl_single_node_new.py#L75) |
| Topic | `'joint_ctrl_cmd'` | `sensor_msgs.msg.JointState` | SUB | `__init__` | [`dual_arm_ws/src/piper/piper/piper_ctrl_single_node_new.py:76`](../../dual_arm_ws/src/piper/piper/piper_ctrl_single_node_new.py#L76) |
| Topic | `'enable_cmd'` | `std_msgs.msg.Bool` | SUB | `__init__` | [`dual_arm_ws/src/piper/piper/piper_ctrl_single_node_new.py:77`](../../dual_arm_ws/src/piper/piper/piper_ctrl_single_node_new.py#L77) |

### piper / PiperRosNode

Node Name：`piper_ctrl_single_node`；Executable：piper_read_slave_joint；Source：[`dual_arm_ws/src/piper/piper/piper_read_slave_joint.py:1`](../../dual_arm_ws/src/piper/piper/piper_read_slave_joint.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'can_port'` | `'can0'` | DEFAULT | `__init__` | [`dual_arm_ws/src/piper/piper/piper_read_slave_joint.py:25`](../../dual_arm_ws/src/piper/piper/piper_read_slave_joint.py#L25) |
| Parameter | `'gripper_exist'` | `True` | DEFAULT | `__init__` | [`dual_arm_ws/src/piper/piper/piper_read_slave_joint.py:26`](../../dual_arm_ws/src/piper/piper/piper_read_slave_joint.py#L26) |
| Topic | `'joint_states'` | `sensor_msgs.msg.JointState` | PUB | `__init__` | [`dual_arm_ws/src/piper/piper/piper_read_slave_joint.py:38`](../../dual_arm_ws/src/piper/piper/piper_read_slave_joint.py#L38) |

### piperh_control / HardwareAdapter

Node Name：`piperh_hardware_adapter`；Executable：hardware_adapter；Source：[`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:1`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L1)。

Hardware=Piper can0+vendor enable；Startup=SocketCAN/新鲜反馈/人工使能；Target=Piper；Update=反馈最高100Hz、轨迹100Hz、status5Hz；Fault=抑制目标/abort，不能等同物理断使能。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'raw_feedback_topic'` | `"/piperh/driver_joint_states"` | DEFAULT | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:129`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L129) |
| Parameter | `'grouped_feedback_topic'` | `"/piperh/teach_joint_states_raw"` | DEFAULT | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:130`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L130) |
| Parameter | `'driver_command_topic'` | `"/piperh/driver_joint_command"` | DEFAULT | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:133`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L133) |
| Parameter | `'driver_enable_service'` | `"/piperh/enable_srv"` | DEFAULT | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:134`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L134) |
| Parameter | `'motor_enable_service'` | `"/piperh/motor/set_enabled"` | DEFAULT | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:135`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L135) |
| Parameter | `'stream_command_topic'` | `"/piperh/servo_joint_trajectory"` | DEFAULT | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:136`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L136) |
| Parameter | `'feedback_topic'` | `"/joint_states"` | DEFAULT | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:137`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L137) |
| Parameter | `'leader_feedback_topic'` | `"/piperh/leader_joint_states"` | DEFAULT | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:138`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L138) |
| Parameter | `'arm_status_topic'` | `"/piperh/arm_status"` | DEFAULT | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:139`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L139) |
| Parameter | `'trajectory_action'` | `"/arm_controller/follow_joint_trajectory"` | DEFAULT | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:140`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L140) |
| Parameter | `'feedback_timeout_sec'` | `0.75` | DEFAULT | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:143`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L143) |
| Parameter | `'motor_enable_latch_timeout_sec'` | `3.0` | DEFAULT | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:144`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L144) |
| Parameter | `'feedback_publish_rate_hz'` | `100.0` | DEFAULT | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:145`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L145) |
| Parameter | `'command_rate_hz'` | `50.0` | DEFAULT | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:146`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L146) |
| Parameter | `'driver_speed_percent'` | `25` | DEFAULT | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:147`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L147) |
| Parameter | `'playback_rate_hz'` | `100.0` | DEFAULT | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:148`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L148) |
| Parameter | `'playback_max_velocity_rad_s'` | `0.40` | DEFAULT | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:149`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L149) |
| Parameter | `'playback_max_acceleration_rad_s2'` | `1.0` | DEFAULT | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:150`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L150) |
| Parameter | `'playback_feedback_timeout_sec'` | `0.20` | DEFAULT | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:151`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L151) |
| Parameter | `'playback_joint_tolerance_rad'` | `0.02` | DEFAULT | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:152`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L152) |
| Parameter | `'playback_stable_cycles'` | `10` | DEFAULT | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:153`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L153) |
| Parameter | `'playback_settle_timeout_sec'` | `5.0` | DEFAULT | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:154`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L154) |
| Parameter | `'tracking_warn_threshold_rad'` | `0.05` | DEFAULT | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:155`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L155) |
| Parameter | `'tracking_abort_threshold_rad'` | `0.20` | DEFAULT | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:156`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L156) |
| Parameter | `'tracking_abort_duration_sec'` | `0.5` | DEFAULT | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:157`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L157) |
| Parameter | `'playback_tracking_dir'` | `str(Path(os.environ.get("REBOTARM_WORKSPACE", ".")) / "log/piperh_tracking")` | DEFAULT | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:158`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L158) |
| Parameter | `'can_port'` | `"can0"` | DEFAULT | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:159`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L159) |
| Parameter | `'gravity_require_selected_robot'` | `False` | DEFAULT | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:160`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L160) |
| Parameter | `'gravity_compensation_calibrated'` | `False` | DEFAULT | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:161`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L161) |
| Parameter | `'gravity_backend'` | `"model_mit"` | DEFAULT | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:162`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L162) |
| Parameter | `'gravity_expected_firmware'` | `""` | DEFAULT | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:163`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L163) |
| Parameter | `'gravity_mount_roll'` | `0.0` | DEFAULT | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:164`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L164) |
| Parameter | `'gravity_mount_pitch'` | `0.0` | DEFAULT | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:165`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L165) |
| Parameter | `'gravity_mount_yaw'` | `0.0` | DEFAULT | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:166`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L166) |
| Parameter | `'gravity_payload_mass_kg'` | `0.0` | DEFAULT | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:167`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L167) |
| Parameter | `'gravity_payload_com_xyz_m'` | `[0.0, 0.0, 0.0]` | DEFAULT | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:168`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L168) |
| Parameter | `'gravity_official_assistance'` | `[0] * 6` | DEFAULT | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:169`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L169) |
| Parameter | `'gravity_official_payload'` | `"unchanged"` | DEFAULT | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:170`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L170) |
| Parameter | `'gravity_official_installation'` | `"unchanged"` | DEFAULT | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:171`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L171) |
| Parameter | `'gravity_tau_scale'` | `[1.0] * 6` | DEFAULT | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:172`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L172) |
| Parameter | `'gravity_kp'` | `[0.0] * 6` | DEFAULT | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:173`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L173) |
| Parameter | `'gravity_kd'` | `[0.8] * 6` | DEFAULT | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:174`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L174) |
| Parameter | `'gravity_max_torque_nm'` | `[4.0] * 6` | DEFAULT | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:175`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L175) |
| Parameter | `'gravity_rate_hz'` | `50.0` | DEFAULT | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:176`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L176) |
| Parameter | `'gravity_max_velocity_rad_s'` | `0.6` | DEFAULT | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:177`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L177) |
| Parameter | `'gravity_max_duration_sec'` | `0.0` | DEFAULT | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:178`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L178) |
| Topic | `str(self.get_parameter('driver_command_topic').value)` | `sensor_msgs.msg.JointState` | PUB | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:257`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L257) |
| Topic | `str(self.get_parameter('feedback_topic').value)` | `sensor_msgs.msg.JointState` | PUB | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:260`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L260) |
| Topic | `str(self.get_parameter('leader_feedback_topic').value)` | `sensor_msgs.msg.JointState` | PUB | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:263`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L263) |
| Topic | `str(self.get_parameter('arm_status_topic').value)` | `rebotarm_msgs.msg.ArmStatus` | PUB | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:269`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L269) |
| Timer | `0.2` | `` | CALLBACK | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:272`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L272) |
| Service | `str(self.get_parameter('driver_enable_service').value)` | `piper_msgs.srv.Enable` | CLIENT | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:273`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L273) |
| Service | `str(self.get_parameter('motor_enable_service').value)` | `std_srvs.srv.SetBool` | SERVER | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:278`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L278) |
| Service | `'/piperh/gravity_compensation/start'` | `std_srvs.srv.Trigger` | SERVER | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:284`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L284) |
| Service | `'/piperh/gravity_compensation/stop'` | `std_srvs.srv.Trigger` | SERVER | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:288`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L288) |
| Service | `'/piperh/control_state/dump'` | `std_srvs.srv.Trigger` | SERVER | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:292`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L292) |
| Topic | `'/piperh/xbox/armed'` | `std_msgs.msg.Bool` | SUB | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:299`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L299) |
| Topic | `'/dual_arm/selected'` | `std_msgs.msg.String` | SUB | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:303`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L303) |
| Topic | `'/piperh/driver_arm_status'` | `piper_msgs.msg.PiperStatusMsg` | SUB | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:307`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L307) |
| Topic | `str(self.get_parameter('raw_feedback_topic').value)` | `sensor_msgs.msg.JointState` | SUB | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:311`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L311) |
| Topic | `str(self.get_parameter('grouped_feedback_topic').value)` | `piper_msgs.msg.PiperTeachJointState` | SUB | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:318`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L318) |
| Topic | `str(self.get_parameter('stream_command_topic').value)` | `trajectory_msgs.msg.JointTrajectory` | SUB | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:325`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L325) |
| Action | `str(self.get_parameter('trajectory_action').value)` | `control_msgs.action.FollowJointTrajectory` | SERVER | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:332`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L332) |

### piperh_control / JointStateUdpBridge

Node Name：`piperh_joint_state_udp_bridge`；Executable：joint_state_udp_bridge；Source：[`dual_arm_ws/src/piperh_control/piperh_control/joint_state_udp_bridge.py:1`](../../dual_arm_ws/src/piperh_control/piperh_control/joint_state_udp_bridge.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'joint_state_topic'` | `"/joint_states"` | DEFAULT | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/joint_state_udp_bridge.py:48`](../../dual_arm_ws/src/piperh_control/piperh_control/joint_state_udp_bridge.py#L48) |
| Parameter | `'host'` | `"127.0.0.1"` | DEFAULT | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/joint_state_udp_bridge.py:49`](../../dual_arm_ws/src/piperh_control/piperh_control/joint_state_udp_bridge.py#L49) |
| Parameter | `'port'` | `5015` | DEFAULT | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/joint_state_udp_bridge.py:50`](../../dual_arm_ws/src/piperh_control/piperh_control/joint_state_udp_bridge.py#L50) |
| Parameter | `'send_rate_hz'` | `60.0` | DEFAULT | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/joint_state_udp_bridge.py:51`](../../dual_arm_ws/src/piperh_control/piperh_control/joint_state_udp_bridge.py#L51) |
| Parameter | `'stale_timeout_sec'` | `0.5` | DEFAULT | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/joint_state_udp_bridge.py:52`](../../dual_arm_ws/src/piperh_control/piperh_control/joint_state_udp_bridge.py#L52) |
| Topic | `str(self.get_parameter('joint_state_topic').value)` | `sensor_msgs.msg.JointState` | SUB | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/joint_state_udp_bridge.py:65`](../../dual_arm_ws/src/piperh_control/piperh_control/joint_state_udp_bridge.py#L65) |
| Timer | `1.0 / rate` | `` | CALLBACK | `__init__` | [`dual_arm_ws/src/piperh_control/piperh_control/joint_state_udp_bridge.py:71`](../../dual_arm_ws/src/piperh_control/piperh_control/joint_state_udp_bridge.py#L71) |

### rebot_teach_mode / TeachModeNode

Node Name：`rebot_teach_mode`；Executable：teach_mode_node；Source：[`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:1`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Topic | `f'/{self._namespace}/teach/status'` | `std_msgs.msg.String` | PUB | `__init__` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:389`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L389) |
| Topic | `f'/{self._namespace}/display_planned_path'` | `moveit_msgs.msg.DisplayTrajectory` | PUB | `__init__` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:395`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L395) |
| Topic | `f'/{self._namespace}/display_robot_state'` | `moveit_msgs.msg.DisplayRobotState` | PUB | `__init__` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:400`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L400) |
| Topic | `f'/{self._namespace}/teach/tcp_trajectory'` | `visualization_msgs.msg.MarkerArray` | PUB | `__init__` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:405`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L405) |
| Topic | `f'/{self._namespace}/teach/drawing_path'` | `rebot_teach_msgs.msg.DrawingPath` | PUB | `__init__` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:408`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L408) |
| Topic | `f'/{self._namespace}/joint_states'` | `sensor_msgs.msg.JointState` | SUB | `__init__` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:413`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L413) |
| Topic | `f'/{self._namespace}/teach_joint_states_raw'` | `piper_msgs.msg.PiperTeachJointState` | SUB | `__init__` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:423`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L423) |
| Topic | `f'/{self._namespace}/arm_status'` | `rebotarm_msgs.msg.ArmStatus` | SUB | `__init__` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:430`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L430) |
| Topic | `str(self.get_parameter('xbox_armed_topic').value)` | `std_msgs.msg.Bool` | SUB | `__init__` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:437`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L437) |
| Service | `f'/{self._namespace}/gravity_compensation/start'` | `std_srvs.srv.Trigger` | CLIENT | `__init__` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:445`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L445) |
| Service | `f'/{self._namespace}/gravity_compensation/stop'` | `std_srvs.srv.Trigger` | CLIENT | `__init__` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:450`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L450) |
| Action | `trajectory_action` | `control_msgs.action.FollowJointTrajectory` | CLIENT | `__init__` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:458`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L458) |
| Service | `f'{self._moveit_prefix}/compute_ik'` | `moveit_msgs.srv.GetPositionIK` | CLIENT | `__init__` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:464`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L464) |
| Service | `f'{self._moveit_prefix}/compute_cartesian_path'` | `moveit_msgs.srv.GetCartesianPath` | CLIENT | `__init__` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:467`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L467) |
| Service | `f'{self._moveit_prefix}/plan_kinematic_path'` | `moveit_msgs.srv.GetMotionPlan` | CLIENT | `__init__` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:472`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L472) |
| Service | `f'/{self._namespace}/teach/{name}'` | `std_srvs.srv.Trigger` | SERVER | `__init__` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:489`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L489) |
| Service | `f'/{self._namespace}/teach/create_shape_action'` | `rebot_teach_msgs.srv.CreateShapeAction` | SERVER | `__init__` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:495`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L495) |
| Service | `f'/{self._namespace}/teach/configure_shape_marker'` | `rebot_teach_msgs.srv.ConfigureShapeMarker` | SERVER | `__init__` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:501`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L501) |
| Service | `f'/{self._namespace}/teach/check_shape_reachability'` | `rebot_teach_msgs.srv.CheckShapeReachability` | SERVER | `__init__` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:507`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L507) |
| Service | `f'/{self._namespace}/teach/configure_trace'` | `rebot_teach_msgs.srv.ConfigureTrace` | SERVER | `__init__` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:513`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L513) |
| Service | `f'/{self._namespace}/teach/start_action_group'` | `rebot_teach_msgs.srv.StartActionGroup` | SERVER | `__init__` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:519`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L519) |
| Service | `f'/{self._namespace}/teach/save_action_group'` | `rebot_teach_msgs.srv.SaveActionGroup` | SERVER | `__init__` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:525`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L525) |
| Service | `f'/{self._namespace}/teach/select_action_group'` | `rebot_teach_msgs.srv.SelectActionGroup` | SERVER | `__init__` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:531`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L531) |
| Service | `f'/{self._namespace}/teach/list_action_groups'` | `rebot_teach_msgs.srv.ListActionGroups` | SERVER | `__init__` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:537`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L537) |
| Service | `f'/{self._namespace}/teach/replay_action_group'` | `rebot_teach_msgs.srv.ReplayActionGroup` | SERVER | `__init__` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:543`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L543) |
| Service | `f'/{self._namespace}/teach/preview_action_group'` | `rebot_teach_msgs.srv.PreviewActionGroup` | SERVER | `__init__` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:549`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L549) |
| Service | `f'/{self._namespace}/teach/preview_action_sequence'` | `rebot_teach_msgs.srv.PreviewActionSequence` | SERVER | `__init__` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:555`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L555) |
| Service | `f'/{self._namespace}/teach/rename_action_group'` | `rebot_teach_msgs.srv.RenameActionGroup` | SERVER | `__init__` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:561`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L561) |
| Service | `f'/{self._namespace}/teach/delete_action_group'` | `rebot_teach_msgs.srv.DeleteActionGroup` | SERVER | `__init__` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:567`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L567) |
| Service | `f'/{self._namespace}/teach/copy_action_group'` | `rebot_teach_msgs.srv.CopyActionGroup` | SERVER | `__init__` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:573`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L573) |
| Service | `f'/{self._namespace}/teach/list_action_sequences'` | `rebot_teach_msgs.srv.ListActionSequences` | SERVER | `__init__` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:579`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L579) |
| Service | `f'/{self._namespace}/teach/save_action_sequence'` | `rebot_teach_msgs.srv.SaveActionSequence` | SERVER | `__init__` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:585`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L585) |
| Service | `f'/{self._namespace}/teach/replay_action_sequence'` | `rebot_teach_msgs.srv.ReplayActionSequence` | SERVER | `__init__` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:591`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L591) |
| Timer | `0.1` | `` | CALLBACK | `__init__` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:597`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L597) |
| Timer | `0.5` | `` | CALLBACK | `__init__` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:598`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L598) |
| Timer | `self._preview_frame_period` | `` | CALLBACK | `__init__` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:599`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L599) |
| Parameter | `'arm_namespace'` | `"rebotarm"` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:608`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L608) |
| Parameter | `'moveit_namespace'` | `""` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:609`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L609) |
| Parameter | `'xbox_armed_topic'` | `"/rebot_xbox/armed"` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:610`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L610) |
| Parameter | `'allow_hardware'` | `False` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:611`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L611) |
| Parameter | `'preview_only'` | `False` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:612`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L612) |
| Parameter | `'joint_names'` | `["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:613`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L613) |
| Parameter | `'trajectory_path'` | `str(Path(os.environ.get("ROBOT_MOTION_ROOT", "motions")) / "rs/teach/latest.motion.json"),` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:617`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L617) |
| Parameter | `'action_library_dir'` | `str(Path(os.environ.get("ROBOT_MOTION_ROOT", "motions")) / "rs/action_groups"),` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:621`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L621) |
| Parameter | `'robot_model'` | `"rebotarm_rs"` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:625`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L625) |
| Parameter | `'trajectory_action'` | `""` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:626`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L626) |
| Parameter | `'sample_period_sec'` | `0.02` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:627`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L627) |
| Parameter | `'minimum_position_delta_rad'` | `0.0005` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:628`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L628) |
| Parameter | `'minimum_recording_duration_sec'` | `0.5` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:629`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L629) |
| Parameter | `'minimum_recording_points'` | `5` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:630`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L630) |
| Parameter | `'maximum_recording_duration_sec'` | `120.0` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:631`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L631) |
| Parameter | `'maximum_samples'` | `6000` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:632`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L632) |
| Parameter | `'feedback_timeout_sec'` | `0.30` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:633`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L633) |
| Parameter | `'leader_joint_states_topic'` | `"/piperh/leader_joint_states"` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:634`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L634) |
| Parameter | `'status_timeout_sec'` | `0.50` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:637`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L637) |
| Parameter | `'driver_service_timeout_sec'` | `5.0` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:638`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L638) |
| Parameter | `'preview_frame_period_sec'` | `0.02` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:639`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L639) |
| Parameter | `'preview_maximum_idle_sec'` | `0.20` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:640`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L640) |
| Parameter | `'preview_idle_position_delta_rad'` | `0.002` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:641`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L641) |
| Parameter | `'require_xbox_locked'` | `True` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:642`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L642) |
| Parameter | `'supports_gravity_compensation'` | `True` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:643`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L643) |
| Parameter | `'endpoint_tolerance_rad'` | `0.08` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:644`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L644) |
| Parameter | `'replay_speed_scale'` | `1.0` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:645`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L645) |
| Parameter | `'maximum_joint_velocity_rad_s'` | `0.30` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:646`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L646) |
| Parameter | `'reverse_return'` | `True` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:647`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L647) |
| Parameter | `'start_dwell_sec'` | `0.5` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:648`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L648) |
| Parameter | `'sequence_transition_direct_tolerance_rad'` | `0.01` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:649`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L649) |
| Parameter | `'sequence_transition_planning_time_sec'` | `5.0` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:650`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L650) |
| Parameter | `'sequence_transition_service_timeout_sec'` | `15.0` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:651`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L651) |
| Parameter | `'sequence_transition_velocity_scaling_factor'` | `0.1` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:652`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L652) |
| Parameter | `'maximum_tracking_error_rad'` | `0.25` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:653`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L653) |
| Parameter | `'tracking_error_consecutive_samples'` | `5` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:654`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L654) |
| Parameter | `'joint_lower_limits'` | `[-2.8, 0.0, 0.0, -1.57, -1.57, -3.14]` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:655`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L655) |
| Parameter | `'joint_upper_limits'` | `[2.8, 3.14, 3.14, 1.57, 1.57, 3.14]` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:658`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L658) |
| Parameter | `'joint_limit_margin_rad'` | `0.0` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:661`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L661) |
| Parameter | `'robot_description'` | `""` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:662`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L662) |
| Parameter | `'tcp_base_frame'` | `"base_link"` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:663`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L663) |
| Parameter | `'tcp_link_name'` | `"gripper_tcp"` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:664`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L664) |
| Parameter | `'visualization_frame_prefix'` | `""` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:665`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L665) |
| Parameter | `'shape_center'` | `[0.28, 0.0, 0.12]` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:666`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L666) |
| Parameter | `'shape_width'` | `0.06` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:667`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L667) |
| Parameter | `'shape_height'` | `0.08` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:668`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L668) |
| Parameter | `'shape_tcp_rpy'` | `[0.0, 1.57, 0.0]` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:669`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L669) |
| Parameter | `'shape_tcp_yaw_offsets'` | `[0.0, math.pi, -math.pi]` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:670`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L670) |
| Parameter | `'shape_cartesian_step_m'` | `0.003` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:673`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L673) |
| Parameter | `'shape_jump_threshold'` | `0.0` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:674`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L674) |
| Parameter | `'shape_planning_time_sec'` | `5.0` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:675`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L675) |
| Parameter | `'shape_service_timeout_sec'` | `15.0` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:676`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L676) |
| Parameter | `'shape_maximum_joint_velocity_rad_s'` | `0.15` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:677`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L677) |
| Parameter | `'shape_maximum_start_drift_rad'` | `0.02` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:678`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L678) |
| Parameter | `'shape_marker_scale_m'` | `0.09` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:679`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L679) |
| Parameter | `'shape_pen_length_m'` | `0.0` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:680`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L680) |
| Parameter | `'shape_pen_mount_offset_m'` | `0.0` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:681`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L681) |
| Parameter | `'shape_pen_lift_m'` | `0.012` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:682`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L682) |
| Parameter | `'shape_preflight_maximum_points'` | `60` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:683`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L683) |
| Parameter | `'shape_preflight_joint_warning_margin_rad'` | `0.15` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:684`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L684) |
| Parameter | `'shape_preflight_ik_timeout_sec'` | `0.08` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:685`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L685) |
| Parameter | `'shape_preflight_suggestion_step_m'` | `0.02` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:686`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L686) |
| Parameter | `'shape_preflight_suggestion_steps'` | `3` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:687`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L687) |
| Parameter | `'image_maximum_strokes'` | `24` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:688`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L688) |
| Parameter | `'image_maximum_points'` | `480` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:689`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L689) |
| Parameter | `'image_minimum_contour_length_px'` | `24.0` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:690`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L690) |
| Parameter | `'image_simplify_epsilon_px'` | `2.0` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:691`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L691) |
| Parameter | `'tcp_trace_line_width_m'` | `0.004` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:692`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L692) |

### rebot_teach_mode / TeachXboxBridge

Node Name：`rebot_teach_xbox_bridge`；Executable：teach_xbox_bridge；Source：[`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/xbox_bridge.py:1`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/xbox_bridge.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'arm_namespace'` | `"rebotarm"` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/xbox_bridge.py:21`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/xbox_bridge.py#L21) |
| Parameter | `'joy_topic'` | `"/joy"` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/xbox_bridge.py:22`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/xbox_bridge.py#L22) |
| Parameter | `'armed_topic'` | `"/rebot_xbox/armed"` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/xbox_bridge.py:23`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/xbox_bridge.py#L23) |
| Parameter | `'control_target_topic'` | `"/xbox/control_target"` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/xbox_bridge.py:24`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/xbox_bridge.py#L24) |
| Parameter | `'teach_status_topic'` | `"/rebotarm/teach/status"` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/xbox_bridge.py:25`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/xbox_bridge.py#L25) |
| Parameter | `'record_button'` | `2` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/xbox_bridge.py:26`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/xbox_bridge.py#L26) |
| Parameter | `'replay_button'` | `1` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/xbox_bridge.py:27`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/xbox_bridge.py#L27) |
| Parameter | `'cancel_button'` | `7` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/xbox_bridge.py:28`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/xbox_bridge.py#L28) |
| Parameter | `'joy_timeout_sec'` | `0.30` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/xbox_bridge.py:29`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/xbox_bridge.py#L29) |
| Service | `f'/{namespace}/teach/start_recording'` | `std_srvs.srv.Trigger` | CLIENT | `__init__` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/xbox_bridge.py:43`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/xbox_bridge.py#L43) |
| Service | `f'/{namespace}/teach/stop_recording'` | `std_srvs.srv.Trigger` | CLIENT | `__init__` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/xbox_bridge.py:46`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/xbox_bridge.py#L46) |
| Service | `f'/{namespace}/teach/replay'` | `std_srvs.srv.Trigger` | CLIENT | `__init__` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/xbox_bridge.py:49`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/xbox_bridge.py#L49) |
| Service | `f'/{namespace}/teach/cancel'` | `std_srvs.srv.Trigger` | CLIENT | `__init__` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/xbox_bridge.py:50`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/xbox_bridge.py#L50) |
| Topic | `str(self.get_parameter('joy_topic').value)` | `sensor_msgs.msg.Joy` | SUB | `__init__` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/xbox_bridge.py:51`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/xbox_bridge.py#L51) |
| Topic | `str(self.get_parameter('armed_topic').value)` | `std_msgs.msg.Bool` | SUB | `__init__` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/xbox_bridge.py:57`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/xbox_bridge.py#L57) |
| Topic | `str(self.get_parameter('control_target_topic').value)` | `std_msgs.msg.String` | SUB | `__init__` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/xbox_bridge.py:63`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/xbox_bridge.py#L63) |
| Topic | `str(self.get_parameter('teach_status_topic').value)` | `std_msgs.msg.String` | SUB | `__init__` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/xbox_bridge.py:69`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/xbox_bridge.py#L69) |
| Timer | `0.1` | `` | CALLBACK | `__init__` | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/xbox_bridge.py:84`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/xbox_bridge.py#L84) |

### rebot_xbox_hardware / ActiveArmManager

Node Name：`active_arm_manager`；Executable：active_arm_manager；Source：[`dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/active_arm_manager.py:1`](../../dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/active_arm_manager.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'initial_robot'` | `"rebotarm"` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/active_arm_manager.py:15`](../../dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/active_arm_manager.py#L15) |
| Parameter | `'offline_preview'` | `False` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/active_arm_manager.py:16`](../../dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/active_arm_manager.py#L16) |
| Topic | `'/dual_arm/selected'` | `std_msgs.msg.String` | PUB | `__init__` | [`dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/active_arm_manager.py:25`](../../dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/active_arm_manager.py#L25) |
| Topic | `'/dual_arm/status'` | `std_msgs.msg.String` | PUB | `__init__` | [`dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/active_arm_manager.py:26`](../../dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/active_arm_manager.py#L26) |
| Service | `name` | `action_msgs.srv.CancelGoal` | CLIENT | `__init__` | [`dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/active_arm_manager.py:28`](../../dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/active_arm_manager.py#L28) |
| Topic | `'/dual_arm/select_request'` | `std_msgs.msg.String` | SUB | `__init__` | [`dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/active_arm_manager.py:38`](../../dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/active_arm_manager.py#L38) |
| Topic | `f'/{robot}/xbox/armed'` | `std_msgs.msg.Bool` | SUB | `__init__` | [`dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/active_arm_manager.py:42`](../../dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/active_arm_manager.py#L42) |

### rebot_xbox_hardware / ForbiddenZoneManager

Node Name：`forbidden_zone_manager`；Executable：forbidden_zone_manager；Source：[`dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/forbidden_zone_manager.py:1`](../../dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/forbidden_zone_manager.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'config_file'` | `""` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/forbidden_zone_manager.py:229`](../../dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/forbidden_zone_manager.py#L229) |
| Parameter | `'active_groups'` | `""` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/forbidden_zone_manager.py:230`](../../dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/forbidden_zone_manager.py#L230) |
| Parameter | `'user_config_file'` | `""` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/forbidden_zone_manager.py:231`](../../dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/forbidden_zone_manager.py#L231) |
| Parameter | `'model'` | `"dm"` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/forbidden_zone_manager.py:232`](../../dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/forbidden_zone_manager.py#L232) |
| Parameter | `'move_group_namespace'` | `""` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/forbidden_zone_manager.py:233`](../../dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/forbidden_zone_manager.py#L233) |
| Parameter | `'interactive_namespace'` | `"/forbidden_zone_manager/interactive"` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/forbidden_zone_manager.py:234`](../../dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/forbidden_zone_manager.py#L234) |
| Service | `f'{moveit_prefix}/apply_planning_scene'` | `moveit_msgs.srv.ApplyPlanningScene` | CLIENT | `__init__` | [`dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/forbidden_zone_manager.py:257`](../../dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/forbidden_zone_manager.py#L257) |
| Service | `f'{moveit_prefix}/get_planning_scene'` | `moveit_msgs.srv.GetPlanningScene` | CLIENT | `__init__` | [`dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/forbidden_zone_manager.py:260`](../../dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/forbidden_zone_manager.py#L260) |
| Topic | `'~/status'` | `std_msgs.msg.String` | PUB | `__init__` | [`dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/forbidden_zone_manager.py:263`](../../dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/forbidden_zone_manager.py#L263) |
| Service | `'~/reload'` | `std_srvs.srv.Trigger` | SERVER | `__init__` | [`dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/forbidden_zone_manager.py:268`](../../dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/forbidden_zone_manager.py#L268) |
| Service | `'~/configure'` | `rebot_teach_msgs.srv.ConfigureForbiddenZone` | SERVER | `__init__` | [`dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/forbidden_zone_manager.py:269`](../../dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/forbidden_zone_manager.py#L269) |
| Timer | `0.5` | `` | CALLBACK | `__init__` | [`dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/forbidden_zone_manager.py:282`](../../dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/forbidden_zone_manager.py#L282) |

### rebot_xbox_hardware / HardwareGripper

Node Name：`rebot_xbox_hardware_gripper`；Executable：hardware_gripper；Source：[`dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/hardware_gripper.py:1`](../../dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/hardware_gripper.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'open_topic'` | `"/rebot_xbox/gripper_open"` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/hardware_gripper.py:38`](../../dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/hardware_gripper.py#L38) |
| Parameter | `'close_topic'` | `"/rebot_xbox/gripper_close"` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/hardware_gripper.py:39`](../../dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/hardware_gripper.py#L39) |
| Parameter | `'state_topic'` | `"/rebotarm/gripper/state"` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/hardware_gripper.py:40`](../../dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/hardware_gripper.py#L40) |
| Parameter | `'command_topic'` | `"/rebotarm/gripper/cmd/pos_vel"` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/hardware_gripper.py:41`](../../dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/hardware_gripper.py#L41) |
| Parameter | `'open_position'` | `5.0` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/hardware_gripper.py:42`](../../dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/hardware_gripper.py#L42) |
| Parameter | `'closed_position'` | `0.0` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/hardware_gripper.py:43`](../../dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/hardware_gripper.py#L43) |
| Parameter | `'maximum_velocity'` | `2.0` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/hardware_gripper.py:44`](../../dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/hardware_gripper.py#L44) |
| Parameter | `'minimum_velocity'` | `0.10` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/hardware_gripper.py:45`](../../dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/hardware_gripper.py#L45) |
| Parameter | `'command_timeout'` | `0.30` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/hardware_gripper.py:46`](../../dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/hardware_gripper.py#L46) |
| Parameter | `'maximum_closing_torque'` | `0.80` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/hardware_gripper.py:47`](../../dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/hardware_gripper.py#L47) |
| Topic | `str(self.get_parameter('command_topic').value)` | `rebotarm_msgs.msg.JointPosVelCmd` | PUB | `__init__` | [`dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/hardware_gripper.py:64`](../../dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/hardware_gripper.py#L64) |
| Topic | `str(self.get_parameter('open_topic').value)` | `std_msgs.msg.Float64` | SUB | `__init__` | [`dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/hardware_gripper.py:67`](../../dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/hardware_gripper.py#L67) |
| Topic | `str(self.get_parameter('close_topic').value)` | `std_msgs.msg.Float64` | SUB | `__init__` | [`dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/hardware_gripper.py:70`](../../dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/hardware_gripper.py#L70) |
| Topic | `str(self.get_parameter('state_topic').value)` | `rebotarm_msgs.msg.JointMotorState` | SUB | `__init__` | [`dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/hardware_gripper.py:73`](../../dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/hardware_gripper.py#L73) |
| Timer | `0.05` | `` | CALLBACK | `__init__` | [`dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/hardware_gripper.py:79`](../../dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/hardware_gripper.py#L79) |

### rebot_xbox_servo / ArmInitializer

Node Name：`rebot_xbox_arm_initializer`；Executable：arm_initializer；Source：[`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/arm_initializer.py:1`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/arm_initializer.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'joint_names'` | `[f"joint{index}" for index in range(1, 7)]` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/arm_initializer.py:70`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/arm_initializer.py#L70) |
| Parameter | `'target_positions'` | `[0.0, 1.75, 0.7, -0.7, 0.0, 0.0]` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/arm_initializer.py:73`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/arm_initializer.py#L73) |
| Parameter | `'move_duration'` | `6.0` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/arm_initializer.py:76`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/arm_initializer.py#L76) |
| Parameter | `'position_tolerance'` | `0.01` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/arm_initializer.py:77`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/arm_initializer.py#L77) |
| Parameter | `'controller_name'` | `"rebotarm_controller"` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/arm_initializer.py:78`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/arm_initializer.py#L78) |
| Parameter | `'controller_wait_timeout'` | `30.0` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/arm_initializer.py:79`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/arm_initializer.py#L79) |
| Parameter | `'completion_timeout'` | `12.0` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/arm_initializer.py:80`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/arm_initializer.py#L80) |
| Parameter | `'topics.joint_states'` | `"/joint_states"` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/arm_initializer.py:81`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/arm_initializer.py#L81) |
| Parameter | `'topics.armed'` | `"/rebot_xbox/armed"` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/arm_initializer.py:82`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/arm_initializer.py#L82) |
| Parameter | `'topics.command'` | `"/rebotarm_controller/joint_trajectory"` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/arm_initializer.py:83`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/arm_initializer.py#L83) |
| Parameter | `'services.list_controllers'` | `"/controller_manager/list_controllers"` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/arm_initializer.py:86`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/arm_initializer.py#L86) |
| Topic | `str(self.get_parameter('topics.command').value)` | `trajectory_msgs.msg.JointTrajectory` | PUB | `__init__` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/arm_initializer.py:118`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/arm_initializer.py#L118) |
| Topic | `str(self.get_parameter('topics.armed').value)` | `std_msgs.msg.Bool` | PUB | `__init__` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/arm_initializer.py:128`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/arm_initializer.py#L128) |
| Topic | `str(self.get_parameter('topics.joint_states').value)` | `sensor_msgs.msg.JointState` | SUB | `__init__` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/arm_initializer.py:134`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/arm_initializer.py#L134) |
| Service | `str(self.get_parameter('services.list_controllers').value)` | `controller_manager_msgs.srv.ListControllers` | CLIENT | `__init__` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/arm_initializer.py:140`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/arm_initializer.py#L140) |

### rebot_xbox_servo / JoyInspector

Node Name：`inspect_joy`；Executable：inspect_joy；Source：[`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/inspect_joy.py:1`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/inspect_joy.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'joy_topic'` | `"/joy"` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/inspect_joy.py:11`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/inspect_joy.py#L11) |
| Parameter | `'noise_threshold'` | `0.05` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/inspect_joy.py:12`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/inspect_joy.py#L12) |
| Topic | `str(self.get_parameter('joy_topic').value)` | `sensor_msgs.msg.Joy` | SUB | `__init__` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/inspect_joy.py:18`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/inspect_joy.py#L18) |

### rebot_xbox_servo / SimGripper

Node Name：`rebot_xbox_sim_gripper`；Executable：sim_gripper；Source：[`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/sim_gripper.py:1`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/sim_gripper.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'topics.open_event'` | `"/rebot_xbox/gripper_open"` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/sim_gripper.py:70`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/sim_gripper.py#L70) |
| Parameter | `'topics.close_event'` | `"/rebot_xbox/gripper_close"` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/sim_gripper.py:71`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/sim_gripper.py#L71) |
| Parameter | `'topics.joint_states'` | `"/joint_states"` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/sim_gripper.py:72`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/sim_gripper.py#L72) |
| Parameter | `'topics.command'` | `"/gripper_controller/joint_trajectory"` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/sim_gripper.py:73`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/sim_gripper.py#L73) |
| Parameter | `'joint_names'` | `["gripper_joint1", "gripper_joint2"]` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/sim_gripper.py:76`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/sim_gripper.py#L76) |
| Parameter | `'open_positions'` | `[0.045, 0.045]` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/sim_gripper.py:79`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/sim_gripper.py#L79) |
| Parameter | `'closed_positions'` | `[0.0, 0.0]` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/sim_gripper.py:80`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/sim_gripper.py#L80) |
| Parameter | `'maximum_speed'` | `0.045` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/sim_gripper.py:81`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/sim_gripper.py#L81) |
| Parameter | `'control_rate'` | `20.0` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/sim_gripper.py:82`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/sim_gripper.py#L82) |
| Parameter | `'command_duration'` | `0.05` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/sim_gripper.py:83`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/sim_gripper.py#L83) |
| Parameter | `'command_timeout'` | `0.30` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/sim_gripper.py:84`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/sim_gripper.py#L84) |
| Topic | `str(self.get_parameter('topics.command').value)` | `trajectory_msgs.msg.JointTrajectory` | PUB | `__init__` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/sim_gripper.py:116`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/sim_gripper.py#L116) |
| Topic | `str(self.get_parameter('topics.open_event').value)` | `std_msgs.msg.Float64` | SUB | `__init__` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/sim_gripper.py:121`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/sim_gripper.py#L121) |
| Topic | `str(self.get_parameter('topics.close_event').value)` | `std_msgs.msg.Float64` | SUB | `__init__` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/sim_gripper.py:127`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/sim_gripper.py#L127) |
| Topic | `str(self.get_parameter('topics.joint_states').value)` | `sensor_msgs.msg.JointState` | SUB | `__init__` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/sim_gripper.py:133`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/sim_gripper.py#L133) |
| Timer | `1.0 / self._control_rate` | `` | CALLBACK | `__init__` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/sim_gripper.py:139`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/sim_gripper.py#L139) |

### rebot_xbox_servo / RebotXboxTwist

Node Name：`rebot_xbox_twist`；Executable：rebot_xbox_twist；Source：[`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py:1`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Topic | `str(self.get_parameter('topics.twist').value)` | `geometry_msgs.msg.TwistStamped` | PUB | `__init__` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py:99`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py#L99) |
| Topic | `str(self.get_parameter('topics.armed').value)` | `std_msgs.msg.Bool` | PUB | `__init__` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py:104`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py#L104) |
| Topic | `str(self.get_parameter('topics.gripper_open').value)` | `std_msgs.msg.Float64` | PUB | `__init__` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py:109`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py#L109) |
| Topic | `str(self.get_parameter('topics.gripper_close').value)` | `std_msgs.msg.Float64` | PUB | `__init__` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py:114`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py#L114) |
| Topic | `str(self.get_parameter('topics.joy').value)` | `sensor_msgs.msg.Joy` | SUB | `__init__` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py:119`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py#L119) |
| Topic | `str(self.get_parameter('topics.control_target').value)` | `std_msgs.msg.String` | SUB | `__init__` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py:125`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py#L125) |
| Topic | `str(self.get_parameter('topics.selected_robot').value)` | `std_msgs.msg.String` | SUB | `__init__` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py:133`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py#L133) |
| Service | `str(self.get_parameter('topics.preset_service').value)` | `std_srvs.srv.Trigger` | SERVER | `__init__` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py:139`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py#L139) |
| Service | `str(self.get_parameter('topics.armed_service').value)` | `std_srvs.srv.SetBool` | SERVER | `__init__` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py:144`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py#L144) |
| Service | `str(self.get_parameter('topics.servo_switch_service').value)` | `moveit_msgs.srv.ServoCommandType` | CLIENT | `__init__` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py:149`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py#L149) |
| Service | `str(self.get_parameter('topics.servo_pause_service').value)` | `std_srvs.srv.SetBool` | CLIENT | `__init__` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py:153`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py#L153) |
| Timer | `1.0 / self.publish_rate` | `` | CALLBACK | `__init__` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py:172`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py#L172) |
| Parameter | `'device'` | `"/dev/input/js0"` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py:184`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py#L184) |
| Parameter | `f'axes.{name}'` | `-1` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py:186`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py#L186) |
| Parameter | `f'invert.{name}'` | `False` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py:187`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py#L187) |
| Parameter | `f'buttons.{name}'` | `-1` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py:189`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py#L189) |
| Parameter | `f'triggers.{name}_axis'` | `-1` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py:191`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py#L191) |
| Parameter | `'triggers.deadzone'` | `0.05` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py:192`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py#L192) |
| Parameter | `'limits.linear_speed'` | `0.08` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py:194`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py#L194) |
| Parameter | `'limits.angular_speed'` | `0.40` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py:195`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py#L195) |
| Parameter | `'limits.minimum_linear_speed'` | `0.005` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py:196`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py#L196) |
| Parameter | `'limits.maximum_linear_speed'` | `0.08` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py:197`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py#L197) |
| Parameter | `'limits.minimum_angular_speed'` | `0.05` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py:198`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py#L198) |
| Parameter | `'limits.maximum_angular_speed'` | `0.50` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py:199`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py#L199) |
| Parameter | `'speed.levels'` | `[0.25, 0.50, 1.00]` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py:200`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py#L200) |
| Parameter | `'safety.deadzone'` | `0.10` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py:202`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py#L202) |
| Parameter | `'safety.joy_timeout'` | `0.30` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py:203`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py#L203) |
| Parameter | `'safety.require_centered_sticks_to_arm'` | `True` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py:204`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py#L204) |
| Parameter | `'safety.require_released_triggers_to_arm'` | `True` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py:205`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py#L205) |
| Parameter | `'safety.require_armed_for_gripper'` | `True` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py:206`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py#L206) |
| Parameter | `'safety.publish_rate'` | `20.0` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py:207`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py#L207) |
| Parameter | `'safety.shutdown_zero_frames'` | `5` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py:208`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py#L208) |
| Parameter | `'frames.base'` | `"base_link"` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py:210`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py#L210) |
| Parameter | `'frames.end_effector'` | `"gripper_tcp"` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py:211`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py#L211) |
| Parameter | `'robot_name'` | `""` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py:214`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py#L214) |
| Parameter | `'topics.joy'` | `"/joy"` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py:215`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py#L215) |
| Parameter | `'topics.twist'` | `"/servo_node/delta_twist_cmds"` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py:216`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py#L216) |
| Parameter | `'topics.armed'` | `"/rebot_xbox/armed"` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py:217`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py#L217) |
| Parameter | `'topics.armed_service'` | `"/rebot_xbox/set_armed"` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py:218`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py#L218) |
| Parameter | `'topics.gripper_open'` | `"/rebot_xbox/gripper_open"` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py:219`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py#L219) |
| Parameter | `'topics.gripper_close'` | `"/rebot_xbox/gripper_close"` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py:220`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py#L220) |
| Parameter | `'topics.control_target'` | `"/xbox/control_target"` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py:221`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py#L221) |
| Parameter | `'topics.selected_robot'` | `"/dual_arm/selected"` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py:222`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py#L222) |
| Parameter | `'topics.preset_service'` | `"/rebot_xbox/go_to_preset"` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py:223`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py#L223) |
| Parameter | `'topics.servo_switch_service'` | `"/servo_node/switch_command_type"` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py:224`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py#L224) |
| Parameter | `'topics.servo_pause_service'` | `"/servo_node/pause_servo"` | DEFAULT | `_declare_parameters` | [`dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py:227`](../../dual_arm_ws/src/rebot_xbox_servo/rebot_xbox_servo/xbox_twist.py#L227) |

### rebotarm_moveit_demos / MoveItDemoBase

Node Name：`helper / MODULE / derived node；NEED_CONFIRMATION`；Executable：见 SOURCE_SCAN 入口和 launch 附录；UNKNOWN / NEED_CONFIRMATION；Source：[`dual_arm_ws/src/rebotarm_moveit_demos/rebotarm_moveit_demos/demo_common.py:1`](../../dual_arm_ws/src/rebotarm_moveit_demos/rebotarm_moveit_demos/demo_common.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'preview_delay_sec'` | `2.0` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_moveit_demos/rebotarm_moveit_demos/demo_common.py:33`](../../dual_arm_ws/src/rebotarm_moveit_demos/rebotarm_moveit_demos/demo_common.py#L33) |
| Action | `'/execute_trajectory'` | `moveit_msgs.action.ExecuteTrajectory` | CLIENT | `__init__` | [`dual_arm_ws/src/rebotarm_moveit_demos/rebotarm_moveit_demos/demo_common.py:40`](../../dual_arm_ws/src/rebotarm_moveit_demos/rebotarm_moveit_demos/demo_common.py#L40) |
| Service | `'/compute_ik'` | `moveit_msgs.srv.GetPositionIK` | CLIENT | `__init__` | [`dual_arm_ws/src/rebotarm_moveit_demos/rebotarm_moveit_demos/demo_common.py:41`](../../dual_arm_ws/src/rebotarm_moveit_demos/rebotarm_moveit_demos/demo_common.py#L41) |
| Topic | `'/display_planned_path'` | `moveit_msgs.msg.DisplayTrajectory` | PUB | `__init__` | [`dual_arm_ws/src/rebotarm_moveit_demos/rebotarm_moveit_demos/demo_common.py:42`](../../dual_arm_ws/src/rebotarm_moveit_demos/rebotarm_moveit_demos/demo_common.py#L42) |
| Topic | `'/joint_states'` | `sensor_msgs.msg.JointState` | SUB | `__init__` | [`dual_arm_ws/src/rebotarm_moveit_demos/rebotarm_moveit_demos/demo_common.py:47`](../../dual_arm_ws/src/rebotarm_moveit_demos/rebotarm_moveit_demos/demo_common.py#L47) |
| Topic | `'/rebotarm/joint_states'` | `sensor_msgs.msg.JointState` | SUB | `__init__` | [`dual_arm_ws/src/rebotarm_moveit_demos/rebotarm_moveit_demos/demo_common.py:53`](../../dual_arm_ws/src/rebotarm_moveit_demos/rebotarm_moveit_demos/demo_common.py#L53) |

### rebotarm_moveit_demos / DrawSquare

Node Name：`helper / MODULE / derived node；NEED_CONFIRMATION`；Executable：draw_square；Source：[`dual_arm_ws/src/rebotarm_moveit_demos/rebotarm_moveit_demos/draw_square.py:1`](../../dual_arm_ws/src/rebotarm_moveit_demos/rebotarm_moveit_demos/draw_square.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Service | `'/plan_kinematic_path'` | `moveit_msgs.srv.GetMotionPlan` | CLIENT | `__init__` | [`dual_arm_ws/src/rebotarm_moveit_demos/rebotarm_moveit_demos/draw_square.py:33`](../../dual_arm_ws/src/rebotarm_moveit_demos/rebotarm_moveit_demos/draw_square.py#L33) |

### rebotarm_moveit_demos / GoHome

Node Name：`helper / MODULE / derived node；NEED_CONFIRMATION`；Executable：go_home；Source：[`dual_arm_ws/src/rebotarm_moveit_demos/rebotarm_moveit_demos/go_home.py:1`](../../dual_arm_ws/src/rebotarm_moveit_demos/rebotarm_moveit_demos/go_home.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Service | `'/plan_kinematic_path'` | `moveit_msgs.srv.GetMotionPlan` | CLIENT | `__init__` | [`dual_arm_ws/src/rebotarm_moveit_demos/rebotarm_moveit_demos/go_home.py:20`](../../dual_arm_ws/src/rebotarm_moveit_demos/rebotarm_moveit_demos/go_home.py#L20) |

### rebotarm_moveit_demos / PickPlace

Node Name：`helper / MODULE / derived node；NEED_CONFIRMATION`；Executable：pick_place；Source：[`dual_arm_ws/src/rebotarm_moveit_demos/rebotarm_moveit_demos/pick_place.py:1`](../../dual_arm_ws/src/rebotarm_moveit_demos/rebotarm_moveit_demos/pick_place.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Service | `'/plan_kinematic_path'` | `moveit_msgs.srv.GetMotionPlan` | CLIENT | `__init__` | [`dual_arm_ws/src/rebotarm_moveit_demos/rebotarm_moveit_demos/pick_place.py:36`](../../dual_arm_ws/src/rebotarm_moveit_demos/rebotarm_moveit_demos/pick_place.py#L36) |
| Service | `'/apply_planning_scene'` | `moveit_msgs.srv.ApplyPlanningScene` | CLIENT | `__init__` | [`dual_arm_ws/src/rebotarm_moveit_demos/rebotarm_moveit_demos/pick_place.py:37`](../../dual_arm_ws/src/rebotarm_moveit_demos/rebotarm_moveit_demos/pick_place.py#L37) |
| Service | `'/get_planning_scene'` | `moveit_msgs.srv.GetPlanningScene` | CLIENT | `__init__` | [`dual_arm_ws/src/rebotarm_moveit_demos/rebotarm_moveit_demos/pick_place.py:41`](../../dual_arm_ws/src/rebotarm_moveit_demos/rebotarm_moveit_demos/pick_place.py#L41) |
| Action | `str(self._param('gripper_action_name'))` | `control_msgs.action.FollowJointTrajectory` | CLIENT | `__init__` | [`dual_arm_ws/src/rebotarm_moveit_demos/rebotarm_moveit_demos/pick_place.py:45`](../../dual_arm_ws/src/rebotarm_moveit_demos/rebotarm_moveit_demos/pick_place.py#L45) |
| Action | `str(self._param('hardware_gripper_action_name'))` | `control_msgs.action.GripperCommand` | CLIENT | `__init__` | [`dual_arm_ws/src/rebotarm_moveit_demos/rebotarm_moveit_demos/pick_place.py:50`](../../dual_arm_ws/src/rebotarm_moveit_demos/rebotarm_moveit_demos/pick_place.py#L50) |

### rebotarm_pulse / AprilTagBoardPoseNode

Node Name：`apriltag_board_pose`；Executable：apriltag_board_pose, apriltag_board_pose；Source：[`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/apriltag_board_pose.py:1`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/apriltag_board_pose.py#L1)。

Hardware=依对象为串口压力/相机TF/机械臂反馈；Startup=配置与服务检查；Target=把脉/诊断；Update=以下Timer与参数；Fault=对象局部处理，是否整机互锁 UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'tag_ids'` | `list(DEFAULT_TAG_IDS)` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/apriltag_board_pose.py:204`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/apriltag_board_pose.py#L204) |
| Parameter | `'tag_size_m'` | `0.040` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/apriltag_board_pose.py:205`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/apriltag_board_pose.py#L205) |
| Parameter | `'center_spacing_x_m'` | `0.100` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/apriltag_board_pose.py:206`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/apriltag_board_pose.py#L206) |
| Parameter | `'center_spacing_y_m'` | `0.120` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/apriltag_board_pose.py:207`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/apriltag_board_pose.py#L207) |
| Parameter | `'minimum_visible_tags'` | `2` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/apriltag_board_pose.py:208`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/apriltag_board_pose.py#L208) |
| Parameter | `'max_reprojection_error_px'` | `3.0` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/apriltag_board_pose.py:209`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/apriltag_board_pose.py#L209) |
| Parameter | `'marker_frame'` | `"marker_frame"` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/apriltag_board_pose.py:210`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/apriltag_board_pose.py#L210) |
| Topic | `'camera_info'` | `sensor_msgs.msg.CameraInfo` | SUB | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/apriltag_board_pose.py:225`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/apriltag_board_pose.py#L225) |
| Topic | `'detections'` | `apriltag_msgs.msg.AprilTagDetectionArray` | SUB | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/apriltag_board_pose.py:226`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/apriltag_board_pose.py#L226) |

### rebotarm_pulse / AutoHandeyeSequence

Node Name：`auto_handeye_sequence`；Executable：auto_handeye_sequence, auto_handeye_sequence；Source：[`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:1`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L1)。

Hardware=依对象为串口压力/相机TF/机械臂反馈；Startup=配置与服务检查；Target=把脉/诊断；Update=以下Timer与参数；Fault=对象局部处理，是否整机互锁 UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'sequence_name'` | `"自动手眼标定_DM_12姿态"` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:147`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L147) |
| Parameter | `'action_name_prefix'` | `"手眼标定姿态_"` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:148`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L148) |
| Parameter | `'calibration_type'` | `"eye_on_base"` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:149`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L149) |
| Parameter | `'teach_status_topic'` | `"/rebotarm/teach/status"` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:150`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L150) |
| Parameter | `'joint_state_topic'` | `"/joint_states"` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:151`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L151) |
| Parameter | `'teach_cancel_service'` | `"/rebotarm/teach/cancel"` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:152`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L152) |
| Parameter | `'tracking_marker_frame'` | `"marker_frame"` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:153`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L153) |
| Parameter | `'minimum_samples'` | `12` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:154`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L154) |
| Parameter | `'settle_sec'` | `1.0` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:155`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L155) |
| Parameter | `'target_tolerance_rad'` | `0.015` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:156`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L156) |
| Parameter | `'feedback_timeout_sec'` | `0.75` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:157`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L157) |
| Parameter | `'service_timeout_sec'` | `2.0` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:158`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L158) |
| Parameter | `'startup_timeout_sec'` | `8.0` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:159`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L159) |
| Parameter | `'marker_freshness_sec'` | `0.35` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:160`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L160) |
| Parameter | `'marker_window_sec'` | `1.0` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:161`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L161) |
| Parameter | `'marker_min_frames'` | `5` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:162`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L162) |
| Parameter | `'marker_max_translation_jitter_m'` | `0.006` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:163`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L163) |
| Parameter | `'marker_max_rotation_jitter_deg'` | `2.0` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:164`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L164) |
| Parameter | `'tf_retry_timeout_sec'` | `5.0` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:165`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L165) |
| Parameter | `'max_translation_rms_m'` | `0.015` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:166`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L166) |
| Parameter | `'max_translation_error_m'` | `0.030` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:167`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L167) |
| Parameter | `'max_rotation_rms_deg'` | `2.0` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:168`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L168) |
| Parameter | `'max_rotation_error_deg'` | `4.0` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:169`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L169) |
| Topic | `str(self.get_parameter('teach_status_topic').value)` | `std_msgs.msg.String` | SUB | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:279`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L279) |
| Topic | `str(self.get_parameter('joint_state_topic').value)` | `sensor_msgs.msg.JointState` | SUB | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:285`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L285) |
| Topic | `'/tf'` | `tf2_msgs.msg.TFMessage` | SUB | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:291`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L291) |
| Topic | `'/rebotarm/handeye/auto_status'` | `std_msgs.msg.String` | PUB | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:297`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L297) |
| Service | `'/easy_handeye2/calibration/get_current_transforms'` | `easy_handeye2_msgs.srv.TakeSample` | CLIENT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:300`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L300) |
| Service | `'/easy_handeye2/calibration/get_sample_list'` | `easy_handeye2_msgs.srv.TakeSample` | CLIENT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:303`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L303) |
| Service | `'/easy_handeye2/calibration/take_sample'` | `easy_handeye2_msgs.srv.TakeSample` | CLIENT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:306`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L306) |
| Service | `'/easy_handeye2/calibration/remove_sample'` | `easy_handeye2_msgs.srv.RemoveSample` | CLIENT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:309`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L309) |
| Service | `'/easy_handeye2/calibration/compute_calibration'` | `easy_handeye2_msgs.srv.ComputeCalibration` | CLIENT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:312`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L312) |
| Service | `'/easy_handeye2/calibration/save_calibration'` | `easy_handeye2_msgs.srv.SaveCalibration` | CLIENT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:315`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L315) |
| Service | `str(self.get_parameter('teach_cancel_service').value)` | `std_srvs.srv.Trigger` | CLIENT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:318`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L318) |
| Timer | `0.05` | `` | CALLBACK | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:321`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L321) |

### rebotarm_pulse / CharucoBoardPoseNode

Node Name：`charuco_board_pose`；Executable：charuco_board_pose, charuco_board_pose；Source：[`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/charuco_board_pose.py:1`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/charuco_board_pose.py#L1)。

Hardware=依对象为串口压力/相机TF/机械臂反馈；Startup=配置与服务检查；Target=把脉/诊断；Update=以下Timer与参数；Fault=对象局部处理，是否整机互锁 UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'squares_x'` | `CHARUCO_SQUARES_X` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/charuco_board_pose.py:165`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/charuco_board_pose.py#L165) |
| Parameter | `'squares_y'` | `CHARUCO_SQUARES_Y` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/charuco_board_pose.py:166`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/charuco_board_pose.py#L166) |
| Parameter | `'square_length_m'` | `CHARUCO_SQUARE_LENGTH_M` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/charuco_board_pose.py:167`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/charuco_board_pose.py#L167) |
| Parameter | `'marker_length_m'` | `CHARUCO_MARKER_LENGTH_M` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/charuco_board_pose.py:168`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/charuco_board_pose.py#L168) |
| Parameter | `'minimum_charuco_corners'` | `6` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/charuco_board_pose.py:169`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/charuco_board_pose.py#L169) |
| Parameter | `'max_reprojection_error_px'` | `3.0` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/charuco_board_pose.py:170`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/charuco_board_pose.py#L170) |
| Parameter | `'marker_frame'` | `"marker_frame"` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/charuco_board_pose.py:171`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/charuco_board_pose.py#L171) |
| Topic | `'camera_info'` | `sensor_msgs.msg.CameraInfo` | SUB | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/charuco_board_pose.py:198`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/charuco_board_pose.py#L198) |
| Topic | `'image'` | `sensor_msgs.msg.Image` | SUB | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/charuco_board_pose.py:201`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/charuco_board_pose.py#L201) |

### rebotarm_pulse / JointTargetMove

Node Name：`helper / MODULE / derived node；NEED_CONFIRMATION`；Executable：joint_target_move, joint_target_move；Source：[`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/joint_target_move.py:1`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/joint_target_move.py#L1)。

Hardware=依对象为串口压力/相机TF/机械臂反馈；Startup=配置与服务检查；Target=把脉/诊断；Update=以下Timer与参数；Fault=对象局部处理，是否整机互锁 UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Service | `'/plan_kinematic_path'` | `moveit_msgs.srv.GetMotionPlan` | CLIENT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/joint_target_move.py:32`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/joint_target_move.py#L32) |
| Topic | `'/rebot_xbox/armed'` | `std_msgs.msg.Bool` | SUB | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/joint_target_move.py:38`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/joint_target_move.py#L38) |

### rebotarm_pulse / PiperPulseAlign

Node Name：`helper / MODULE / derived node；NEED_CONFIRMATION`；Executable：piper_pulse_align, piper_pulse_align；Source：[`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py:1`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py#L1)。

Hardware=依对象为串口压力/相机TF/机械臂反馈；Startup=配置与服务检查；Target=把脉/诊断；Update=以下Timer与参数；Fault=对象局部处理，是否整机互锁 UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Service | `'/compute_cartesian_path'` | `moveit_msgs.srv.GetCartesianPath` | CLIENT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py:95`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py#L95) |
| Service | `'/move_group/get_parameters'` | `rcl_interfaces.srv.GetParameters` | CLIENT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py:98`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py#L98) |
| Service | `'/check_state_validity'` | `moveit_msgs.srv.GetStateValidity` | CLIENT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py:101`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py#L101) |
| Topic | `str(self._param('target_topic'))` | `geometry_msgs.msg.PointStamped` | SUB | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py:164`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py#L164) |
| Topic | `str(self._param('arm_direction_topic'))` | `geometry_msgs.msg.Vector3Stamped` | SUB | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py:168`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py#L168) |
| Topic | `'/rebot_xbox/armed'` | `std_msgs.msg.Bool` | SUB | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py:175`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py#L175) |
| Topic | `'/dual_arm/selected'` | `std_msgs.msg.String` | SUB | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py:178`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py#L178) |
| Topic | `'/piperh/pulse/serial_connected'` | `std_msgs.msg.Bool` | SUB | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py:181`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py#L181) |
| Topic | `'/piperh/pulse/zero_calibrated'` | `std_msgs.msg.Bool` | SUB | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py:184`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py#L184) |
| Topic | `f'/piperh/pulse/zeroed/{channel}/pressure'` | `sensor_msgs.msg.FluidPressure` | SUB | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py:188`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py#L188) |
| Topic | `'/piperh/pulse/precontact'` | `geometry_msgs.msg.PointStamped` | PUB | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py:193`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py#L193) |
| Topic | `'/piperh/pulse/axis_alignment_markers'` | `visualization_msgs.msg.MarkerArray` | PUB | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py:199`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py#L199) |
| Topic | `'/piperh/pulse/staged_precontact_markers'` | `visualization_msgs.msg.MarkerArray` | PUB | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py:202`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py#L202) |
| Topic | `'/piperh/pulse/precontact_diagnostics'` | `diagnostic_msgs.msg.DiagnosticArray` | PUB | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py:205`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py#L205) |
| Service | `'/piperh/pulse/dump_precontact_diagnostics'` | `std_srvs.srv.Trigger` | SERVER | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py:208`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py#L208) |
| Timer | `self._heartbeat_period` | `` | CALLBACK | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py:213`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py#L213) |

### rebotarm_pulse / PiperPulseTarget

Node Name：`piper_pulse_target`；Executable：piper_pulse_target, piper_pulse_target；Source：[`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py:1`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py#L1)。

Hardware=依对象为串口压力/相机TF/机械臂反馈；Startup=配置与服务检查；Target=把脉/诊断；Update=以下Timer与参数；Fault=对象局部处理，是否整机互锁 UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'target_topic'` | `"/meridian_hand_vision/pulse_point"` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py:86`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py#L86) |
| Parameter | `'arm_direction_topic'` | `"/meridian_hand_vision/arm_direction"` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py:87`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py#L87) |
| Parameter | `'planning_frame'` | `"piperh_planning_world"` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py:90`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py#L90) |
| Parameter | `'calibration_name'` | `"piperh_camera_eye_in_hand"` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py:91`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py#L91) |
| Parameter | `'calibration_bridge_frame'` | `"piperh/Link5"` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py:95`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py#L95) |
| Parameter | `'tf_timeout_sec'` | `0.1` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py:96`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py#L96) |
| Parameter | `'input_hard_age_sec'` | `1.5` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py:97`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py#L97) |
| Parameter | `'heartbeat_hz'` | `2.0` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py:98`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py#L98) |
| Parameter | `'statistics_period_sec'` | `5.0` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py:99`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py#L99) |
| Parameter | `'gap_warning_sec'` | `[0.5, 1.0, 2.0]` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py:100`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py#L100) |
| Parameter | `'target_range_min_m'` | `[-2.0, -2.0, 0.0]` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py:101`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py#L101) |
| Parameter | `'target_range_max_m'` | `[2.0, 2.0, 3.0]` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py:102`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py#L102) |
| Parameter | `'diagnostic_log_directory'` | `"logs/pulse_precontact"` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py:103`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py#L103) |
| Parameter | `'source_status_topic'` | `"/meridian_hand_vision/pulse_status"` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py:104`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py#L104) |
| Topic | `'/piperh/pulse/target'` | `geometry_msgs.msg.PointStamped` | PUB | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py:154`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py#L154) |
| Topic | `'/piperh/pulse/target_marker'` | `visualization_msgs.msg.Marker` | PUB | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py:157`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py#L157) |
| Topic | `'/piperh/pulse/arm_direction'` | `geometry_msgs.msg.Vector3Stamped` | PUB | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py:160`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py#L160) |
| Topic | `'/piperh/pulse/target_diagnostics'` | `diagnostic_msgs.msg.DiagnosticArray` | PUB | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py:163`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py#L163) |
| Topic | `str(self.get_parameter('target_topic').value)` | `geometry_msgs.msg.PointStamped` | SUB | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py:166`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py#L166) |
| Topic | `str(self.get_parameter('source_status_topic').value)` | `std_msgs.msg.String` | SUB | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py:172`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py#L172) |
| Timer | `1.0 / heartbeat_hz` | `` | CALLBACK | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py:179`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py#L179) |
| Timer | `max(1.0, float(self.get_parameter('statistics_period_sec').value))` | `` | CALLBACK | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py:180`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py#L180) |
| Topic | `str(self.get_parameter('arm_direction_topic').value)` | `geometry_msgs.msg.Vector3Stamped` | SUB | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py:187`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py#L187) |

### rebotarm_pulse / PiperToolGeometry

Node Name：`piper_tool_geometry`；Executable：piper_tool_geometry, piper_tool_geometry；Source：[`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_tool_geometry.py:1`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_tool_geometry.py#L1)。

Hardware=依对象为串口压力/相机TF/机械臂反馈；Startup=配置与服务检查；Target=把脉/诊断；Update=以下Timer与参数；Fault=对象局部处理，是否整机互锁 UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'parent_frame'` | `"piperh/pulse_tool_envelope"` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_tool_geometry.py:52`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_tool_geometry.py#L52) |
| Parameter | `'geometry_calibrated'` | `False` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_tool_geometry.py:53`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_tool_geometry.py#L53) |
| Parameter | `'support_length_m'` | `0.11` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_tool_geometry.py:54`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_tool_geometry.py#L54) |
| Parameter | `'support_width_m'` | `0.025` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_tool_geometry.py:55`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_tool_geometry.py#L55) |
| Parameter | `'support_depth_m'` | `0.018` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_tool_geometry.py:56`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_tool_geometry.py#L56) |
| Parameter | `'approx_support_center_xyz'` | `[0.0, 0.0, -0.055]` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_tool_geometry.py:57`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_tool_geometry.py#L57) |
| Parameter | `'approx_support_rpy'` | `[0.0, 0.0, 0.0]` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_tool_geometry.py:58`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_tool_geometry.py#L58) |
| Parameter | `'approx_sensor_offsets_m'` | `[0.0, 0.010, -0.025, 0.0, 0.010, -0.055, 0.0, 0.010, -0.085],` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_tool_geometry.py:59`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_tool_geometry.py#L59) |
| Parameter | `'calibrated_sensor_offsets_m'` | `[0.0] * 9` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_tool_geometry.py:63`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_tool_geometry.py#L63) |
| Parameter | `'contact_normal_xyz'` | `[0.0, 0.0, 0.0]` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_tool_geometry.py:64`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_tool_geometry.py#L64) |
| Topic | `'/piperh/pulse/tool_markers'` | `visualization_msgs.msg.MarkerArray` | PUB | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_tool_geometry.py:94`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_tool_geometry.py#L94) |
| Timer | `1.0` | `` | CALLBACK | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_tool_geometry.py:105`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/piper_tool_geometry.py#L105) |

### rebotarm_pulse / PressureSerialBridge

Node Name：`pressure_serial_bridge`；Executable：pressure_serial_bridge, pressure_serial_bridge；Source：[`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py:1`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py#L1)。

Hardware=依对象为串口压力/相机TF/机械臂反馈；Startup=配置与服务检查；Target=把脉/诊断；Update=以下Timer与参数；Fault=对象局部处理，是否整机互锁 UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'port'` | `""` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py:28`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py#L28) |
| Parameter | `'baud_rate'` | `921600` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py:29`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py#L29) |
| Parameter | `'topic_prefix'` | `"/piperh/pulse"` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py:30`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py#L30) |
| Parameter | `'sensor_frame'` | `""` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py:31`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py#L31) |
| Parameter | `'zero_calibrated'` | `False` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py:32`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py#L32) |
| Parameter | `'zero_baselines_pa'` | `[0.0, 0.0, 0.0]` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py:33`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py#L33) |
| Parameter | `'zero_window_seconds'` | `3.0` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py:34`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py#L34) |
| Parameter | `'zero_min_samples'` | `30` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py:35`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py#L35) |
| Topic | `f'{prefix}/raw/{channel.lower()}/pressure'` | `sensor_msgs.msg.FluidPressure` | PUB | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py:74`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py#L74) |
| Topic | `f'{prefix}/raw/{channel.lower()}/temperature'` | `sensor_msgs.msg.Temperature` | PUB | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py:80`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py#L80) |
| Topic | `f'{prefix}/zeroed/{channel.lower()}/pressure'` | `sensor_msgs.msg.FluidPressure` | PUB | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py:86`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py#L86) |
| Topic | `f'{prefix}/serial_connected'` | `std_msgs.msg.Bool` | PUB | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py:94`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py#L94) |
| Topic | `f'{prefix}/zero_calibrated'` | `std_msgs.msg.Bool` | PUB | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py:97`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py#L97) |
| Service | `f'{prefix}/calibrate_zero'` | `std_srvs.srv.Trigger` | SERVER | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py:103`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py#L103) |
| Timer | `0.01` | `` | CALLBACK | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py:112`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py#L112) |

### rebotarm_pulse / PulseApproach

Node Name：`helper / MODULE / derived node；NEED_CONFIRMATION`；Executable：pulse_approach, pulse_approach；Source：[`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pulse_approach.py:1`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pulse_approach.py#L1)。

Hardware=依对象为串口压力/相机TF/机械臂反馈；Startup=配置与服务检查；Target=把脉/诊断；Update=以下Timer与参数；Fault=对象局部处理，是否整机互锁 UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Service | `'/plan_kinematic_path'` | `moveit_msgs.srv.GetMotionPlan` | CLIENT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pulse_approach.py:37`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pulse_approach.py#L37) |
| Topic | `str(self._param('target_topic'))` | `geometry_msgs.msg.PointStamped` | SUB | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pulse_approach.py:44`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pulse_approach.py#L44) |
| Topic | `'/rebot_xbox/armed'` | `std_msgs.msg.Bool` | SUB | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pulse_approach.py:53`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pulse_approach.py#L53) |
| Topic | `'/rebotarm/pulse/target'` | `geometry_msgs.msg.PointStamped` | PUB | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pulse_approach.py:54`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pulse_approach.py#L54) |
| Topic | `'/rebotarm/pulse/precontact'` | `geometry_msgs.msg.PointStamped` | PUB | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pulse_approach.py:57`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pulse_approach.py#L57) |
| Topic | `'/rebotarm/pulse/markers'` | `visualization_msgs.msg.Marker` | PUB | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pulse_approach.py:60`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pulse_approach.py#L60) |

### rebotarm_pulse / PulseWebGateway

Node Name：`pulse_web_gateway`；Executable：pulse_web_gateway, pulse_web_gateway；Source：[`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pulse_web_gateway.py:1`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pulse_web_gateway.py#L1)。

Hardware=依对象为串口压力/相机TF/机械臂反馈；Startup=配置与服务检查；Target=把脉/诊断；Update=以下Timer与参数；Fault=对象局部处理，是否整机互锁 UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'host'` | `"127.0.0.1"` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pulse_web_gateway.py:122`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pulse_web_gateway.py#L122) |
| Parameter | `'port'` | `8765` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pulse_web_gateway.py:123`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pulse_web_gateway.py#L123) |
| Parameter | `'topic_prefix'` | `"/piperh/pulse"` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pulse_web_gateway.py:124`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pulse_web_gateway.py#L124) |
| Parameter | `'site_directory'` | `""` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pulse_web_gateway.py:125`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pulse_web_gateway.py#L125) |
| Topic | `f'{prefix}/raw/{lower}/temperature'` | `sensor_msgs.msg.Temperature` | SUB | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pulse_web_gateway.py:143`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pulse_web_gateway.py#L143) |
| Topic | `f'{prefix}/raw/{lower}/pressure'` | `sensor_msgs.msg.FluidPressure` | SUB | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pulse_web_gateway.py:149`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pulse_web_gateway.py#L149) |
| Topic | `f'{prefix}/serial_connected'` | `std_msgs.msg.Bool` | SUB | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pulse_web_gateway.py:159`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pulse_web_gateway.py#L159) |
| Topic | `f'{prefix}/zero_calibrated'` | `std_msgs.msg.Bool` | SUB | `__init__` | [`dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pulse_web_gateway.py:167`](../../dual_arm_ws/src/rebotarm_pulse/rebotarm_pulse/pulse_web_gateway.py#L167) |

### rebotarm_pulse / AprilTagBoardPoseNode

Node Name：`apriltag_board_pose`；Executable：apriltag_board_pose, apriltag_board_pose；Source：[`src/rebotarm_pulse/rebotarm_pulse/apriltag_board_pose.py:1`](../../src/rebotarm_pulse/rebotarm_pulse/apriltag_board_pose.py#L1)。

Hardware=依对象为串口压力/相机TF/机械臂反馈；Startup=配置与服务检查；Target=把脉/诊断；Update=以下Timer与参数；Fault=对象局部处理，是否整机互锁 UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'tag_ids'` | `list(DEFAULT_TAG_IDS)` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/apriltag_board_pose.py:204`](../../src/rebotarm_pulse/rebotarm_pulse/apriltag_board_pose.py#L204) |
| Parameter | `'tag_size_m'` | `0.040` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/apriltag_board_pose.py:205`](../../src/rebotarm_pulse/rebotarm_pulse/apriltag_board_pose.py#L205) |
| Parameter | `'center_spacing_x_m'` | `0.100` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/apriltag_board_pose.py:206`](../../src/rebotarm_pulse/rebotarm_pulse/apriltag_board_pose.py#L206) |
| Parameter | `'center_spacing_y_m'` | `0.120` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/apriltag_board_pose.py:207`](../../src/rebotarm_pulse/rebotarm_pulse/apriltag_board_pose.py#L207) |
| Parameter | `'minimum_visible_tags'` | `2` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/apriltag_board_pose.py:208`](../../src/rebotarm_pulse/rebotarm_pulse/apriltag_board_pose.py#L208) |
| Parameter | `'max_reprojection_error_px'` | `3.0` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/apriltag_board_pose.py:209`](../../src/rebotarm_pulse/rebotarm_pulse/apriltag_board_pose.py#L209) |
| Parameter | `'marker_frame'` | `"marker_frame"` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/apriltag_board_pose.py:210`](../../src/rebotarm_pulse/rebotarm_pulse/apriltag_board_pose.py#L210) |
| Topic | `'camera_info'` | `sensor_msgs.msg.CameraInfo` | SUB | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/apriltag_board_pose.py:225`](../../src/rebotarm_pulse/rebotarm_pulse/apriltag_board_pose.py#L225) |
| Topic | `'detections'` | `apriltag_msgs.msg.AprilTagDetectionArray` | SUB | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/apriltag_board_pose.py:226`](../../src/rebotarm_pulse/rebotarm_pulse/apriltag_board_pose.py#L226) |

### rebotarm_pulse / AutoHandeyeSequence

Node Name：`auto_handeye_sequence`；Executable：auto_handeye_sequence, auto_handeye_sequence；Source：[`src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:1`](../../src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L1)。

Hardware=依对象为串口压力/相机TF/机械臂反馈；Startup=配置与服务检查；Target=把脉/诊断；Update=以下Timer与参数；Fault=对象局部处理，是否整机互锁 UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'sequence_name'` | `"自动手眼标定_DM_12姿态"` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:147`](../../src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L147) |
| Parameter | `'action_name_prefix'` | `"手眼标定姿态_"` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:148`](../../src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L148) |
| Parameter | `'calibration_type'` | `"eye_on_base"` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:149`](../../src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L149) |
| Parameter | `'teach_status_topic'` | `"/rebotarm/teach/status"` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:150`](../../src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L150) |
| Parameter | `'joint_state_topic'` | `"/joint_states"` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:151`](../../src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L151) |
| Parameter | `'teach_cancel_service'` | `"/rebotarm/teach/cancel"` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:152`](../../src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L152) |
| Parameter | `'tracking_marker_frame'` | `"marker_frame"` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:153`](../../src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L153) |
| Parameter | `'minimum_samples'` | `12` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:154`](../../src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L154) |
| Parameter | `'settle_sec'` | `1.0` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:155`](../../src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L155) |
| Parameter | `'target_tolerance_rad'` | `0.015` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:156`](../../src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L156) |
| Parameter | `'feedback_timeout_sec'` | `0.75` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:157`](../../src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L157) |
| Parameter | `'service_timeout_sec'` | `2.0` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:158`](../../src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L158) |
| Parameter | `'startup_timeout_sec'` | `8.0` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:159`](../../src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L159) |
| Parameter | `'marker_freshness_sec'` | `0.35` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:160`](../../src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L160) |
| Parameter | `'marker_window_sec'` | `1.0` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:161`](../../src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L161) |
| Parameter | `'marker_min_frames'` | `5` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:162`](../../src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L162) |
| Parameter | `'marker_max_translation_jitter_m'` | `0.006` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:163`](../../src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L163) |
| Parameter | `'marker_max_rotation_jitter_deg'` | `2.0` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:164`](../../src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L164) |
| Parameter | `'tf_retry_timeout_sec'` | `5.0` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:165`](../../src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L165) |
| Parameter | `'max_translation_rms_m'` | `0.015` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:166`](../../src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L166) |
| Parameter | `'max_translation_error_m'` | `0.030` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:167`](../../src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L167) |
| Parameter | `'max_rotation_rms_deg'` | `2.0` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:168`](../../src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L168) |
| Parameter | `'max_rotation_error_deg'` | `4.0` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:169`](../../src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L169) |
| Topic | `str(self.get_parameter('teach_status_topic').value)` | `std_msgs.msg.String` | SUB | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:279`](../../src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L279) |
| Topic | `str(self.get_parameter('joint_state_topic').value)` | `sensor_msgs.msg.JointState` | SUB | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:285`](../../src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L285) |
| Topic | `'/tf'` | `tf2_msgs.msg.TFMessage` | SUB | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:291`](../../src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L291) |
| Topic | `'/rebotarm/handeye/auto_status'` | `std_msgs.msg.String` | PUB | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:297`](../../src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L297) |
| Service | `'/easy_handeye2/calibration/get_current_transforms'` | `easy_handeye2_msgs.srv.TakeSample` | CLIENT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:300`](../../src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L300) |
| Service | `'/easy_handeye2/calibration/get_sample_list'` | `easy_handeye2_msgs.srv.TakeSample` | CLIENT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:303`](../../src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L303) |
| Service | `'/easy_handeye2/calibration/take_sample'` | `easy_handeye2_msgs.srv.TakeSample` | CLIENT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:306`](../../src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L306) |
| Service | `'/easy_handeye2/calibration/remove_sample'` | `easy_handeye2_msgs.srv.RemoveSample` | CLIENT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:309`](../../src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L309) |
| Service | `'/easy_handeye2/calibration/compute_calibration'` | `easy_handeye2_msgs.srv.ComputeCalibration` | CLIENT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:312`](../../src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L312) |
| Service | `'/easy_handeye2/calibration/save_calibration'` | `easy_handeye2_msgs.srv.SaveCalibration` | CLIENT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:315`](../../src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L315) |
| Service | `str(self.get_parameter('teach_cancel_service').value)` | `std_srvs.srv.Trigger` | CLIENT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:318`](../../src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L318) |
| Timer | `0.05` | `` | CALLBACK | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py:321`](../../src/rebotarm_pulse/rebotarm_pulse/auto_handeye_sequence.py#L321) |

### rebotarm_pulse / CharucoBoardPoseNode

Node Name：`charuco_board_pose`；Executable：charuco_board_pose, charuco_board_pose；Source：[`src/rebotarm_pulse/rebotarm_pulse/charuco_board_pose.py:1`](../../src/rebotarm_pulse/rebotarm_pulse/charuco_board_pose.py#L1)。

Hardware=依对象为串口压力/相机TF/机械臂反馈；Startup=配置与服务检查；Target=把脉/诊断；Update=以下Timer与参数；Fault=对象局部处理，是否整机互锁 UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'squares_x'` | `CHARUCO_SQUARES_X` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/charuco_board_pose.py:165`](../../src/rebotarm_pulse/rebotarm_pulse/charuco_board_pose.py#L165) |
| Parameter | `'squares_y'` | `CHARUCO_SQUARES_Y` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/charuco_board_pose.py:166`](../../src/rebotarm_pulse/rebotarm_pulse/charuco_board_pose.py#L166) |
| Parameter | `'square_length_m'` | `CHARUCO_SQUARE_LENGTH_M` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/charuco_board_pose.py:167`](../../src/rebotarm_pulse/rebotarm_pulse/charuco_board_pose.py#L167) |
| Parameter | `'marker_length_m'` | `CHARUCO_MARKER_LENGTH_M` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/charuco_board_pose.py:168`](../../src/rebotarm_pulse/rebotarm_pulse/charuco_board_pose.py#L168) |
| Parameter | `'minimum_charuco_corners'` | `6` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/charuco_board_pose.py:169`](../../src/rebotarm_pulse/rebotarm_pulse/charuco_board_pose.py#L169) |
| Parameter | `'max_reprojection_error_px'` | `3.0` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/charuco_board_pose.py:170`](../../src/rebotarm_pulse/rebotarm_pulse/charuco_board_pose.py#L170) |
| Parameter | `'marker_frame'` | `"marker_frame"` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/charuco_board_pose.py:171`](../../src/rebotarm_pulse/rebotarm_pulse/charuco_board_pose.py#L171) |
| Topic | `'camera_info'` | `sensor_msgs.msg.CameraInfo` | SUB | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/charuco_board_pose.py:198`](../../src/rebotarm_pulse/rebotarm_pulse/charuco_board_pose.py#L198) |
| Topic | `'image'` | `sensor_msgs.msg.Image` | SUB | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/charuco_board_pose.py:201`](../../src/rebotarm_pulse/rebotarm_pulse/charuco_board_pose.py#L201) |

### rebotarm_pulse / JointTargetMove

Node Name：`helper / MODULE / derived node；NEED_CONFIRMATION`；Executable：joint_target_move, joint_target_move；Source：[`src/rebotarm_pulse/rebotarm_pulse/joint_target_move.py:1`](../../src/rebotarm_pulse/rebotarm_pulse/joint_target_move.py#L1)。

Hardware=依对象为串口压力/相机TF/机械臂反馈；Startup=配置与服务检查；Target=把脉/诊断；Update=以下Timer与参数；Fault=对象局部处理，是否整机互锁 UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Service | `'/plan_kinematic_path'` | `moveit_msgs.srv.GetMotionPlan` | CLIENT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/joint_target_move.py:32`](../../src/rebotarm_pulse/rebotarm_pulse/joint_target_move.py#L32) |
| Topic | `'/rebot_xbox/armed'` | `std_msgs.msg.Bool` | SUB | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/joint_target_move.py:38`](../../src/rebotarm_pulse/rebotarm_pulse/joint_target_move.py#L38) |

### rebotarm_pulse / PiperPulseAlign

Node Name：`helper / MODULE / derived node；NEED_CONFIRMATION`；Executable：piper_pulse_align, piper_pulse_align；Source：[`src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py:1`](../../src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py#L1)。

Hardware=依对象为串口压力/相机TF/机械臂反馈；Startup=配置与服务检查；Target=把脉/诊断；Update=以下Timer与参数；Fault=对象局部处理，是否整机互锁 UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Service | `'/compute_cartesian_path'` | `moveit_msgs.srv.GetCartesianPath` | CLIENT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py:95`](../../src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py#L95) |
| Service | `'/move_group/get_parameters'` | `rcl_interfaces.srv.GetParameters` | CLIENT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py:98`](../../src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py#L98) |
| Service | `'/check_state_validity'` | `moveit_msgs.srv.GetStateValidity` | CLIENT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py:101`](../../src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py#L101) |
| Topic | `str(self._param('target_topic'))` | `geometry_msgs.msg.PointStamped` | SUB | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py:164`](../../src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py#L164) |
| Topic | `str(self._param('arm_direction_topic'))` | `geometry_msgs.msg.Vector3Stamped` | SUB | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py:168`](../../src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py#L168) |
| Topic | `'/rebot_xbox/armed'` | `std_msgs.msg.Bool` | SUB | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py:175`](../../src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py#L175) |
| Topic | `'/dual_arm/selected'` | `std_msgs.msg.String` | SUB | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py:178`](../../src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py#L178) |
| Topic | `'/piperh/pulse/serial_connected'` | `std_msgs.msg.Bool` | SUB | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py:181`](../../src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py#L181) |
| Topic | `'/piperh/pulse/zero_calibrated'` | `std_msgs.msg.Bool` | SUB | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py:184`](../../src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py#L184) |
| Topic | `f'/piperh/pulse/zeroed/{channel}/pressure'` | `sensor_msgs.msg.FluidPressure` | SUB | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py:188`](../../src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py#L188) |
| Topic | `'/piperh/pulse/precontact'` | `geometry_msgs.msg.PointStamped` | PUB | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py:193`](../../src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py#L193) |
| Topic | `'/piperh/pulse/axis_alignment_markers'` | `visualization_msgs.msg.MarkerArray` | PUB | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py:199`](../../src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py#L199) |
| Topic | `'/piperh/pulse/staged_precontact_markers'` | `visualization_msgs.msg.MarkerArray` | PUB | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py:202`](../../src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py#L202) |
| Topic | `'/piperh/pulse/precontact_diagnostics'` | `diagnostic_msgs.msg.DiagnosticArray` | PUB | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py:205`](../../src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py#L205) |
| Service | `'/piperh/pulse/dump_precontact_diagnostics'` | `std_srvs.srv.Trigger` | SERVER | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py:208`](../../src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py#L208) |
| Timer | `self._heartbeat_period` | `` | CALLBACK | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py:213`](../../src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py#L213) |

### rebotarm_pulse / PiperPulseTarget

Node Name：`piper_pulse_target`；Executable：piper_pulse_target, piper_pulse_target；Source：[`src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py:1`](../../src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py#L1)。

Hardware=依对象为串口压力/相机TF/机械臂反馈；Startup=配置与服务检查；Target=把脉/诊断；Update=以下Timer与参数；Fault=对象局部处理，是否整机互锁 UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'target_topic'` | `"/meridian_hand_vision/pulse_point"` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py:86`](../../src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py#L86) |
| Parameter | `'arm_direction_topic'` | `"/meridian_hand_vision/arm_direction"` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py:87`](../../src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py#L87) |
| Parameter | `'planning_frame'` | `"piperh_planning_world"` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py:90`](../../src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py#L90) |
| Parameter | `'calibration_name'` | `"piperh_camera_eye_in_hand"` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py:91`](../../src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py#L91) |
| Parameter | `'calibration_bridge_frame'` | `"piperh/Link5"` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py:95`](../../src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py#L95) |
| Parameter | `'tf_timeout_sec'` | `0.1` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py:96`](../../src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py#L96) |
| Parameter | `'input_hard_age_sec'` | `1.5` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py:97`](../../src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py#L97) |
| Parameter | `'heartbeat_hz'` | `2.0` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py:98`](../../src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py#L98) |
| Parameter | `'statistics_period_sec'` | `5.0` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py:99`](../../src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py#L99) |
| Parameter | `'gap_warning_sec'` | `[0.5, 1.0, 2.0]` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py:100`](../../src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py#L100) |
| Parameter | `'target_range_min_m'` | `[-2.0, -2.0, 0.0]` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py:101`](../../src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py#L101) |
| Parameter | `'target_range_max_m'` | `[2.0, 2.0, 3.0]` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py:102`](../../src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py#L102) |
| Parameter | `'diagnostic_log_directory'` | `"logs/pulse_precontact"` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py:103`](../../src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py#L103) |
| Parameter | `'source_status_topic'` | `"/meridian_hand_vision/pulse_status"` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py:104`](../../src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py#L104) |
| Topic | `'/piperh/pulse/target'` | `geometry_msgs.msg.PointStamped` | PUB | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py:154`](../../src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py#L154) |
| Topic | `'/piperh/pulse/target_marker'` | `visualization_msgs.msg.Marker` | PUB | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py:157`](../../src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py#L157) |
| Topic | `'/piperh/pulse/arm_direction'` | `geometry_msgs.msg.Vector3Stamped` | PUB | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py:160`](../../src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py#L160) |
| Topic | `'/piperh/pulse/target_diagnostics'` | `diagnostic_msgs.msg.DiagnosticArray` | PUB | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py:163`](../../src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py#L163) |
| Topic | `str(self.get_parameter('target_topic').value)` | `geometry_msgs.msg.PointStamped` | SUB | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py:166`](../../src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py#L166) |
| Topic | `str(self.get_parameter('source_status_topic').value)` | `std_msgs.msg.String` | SUB | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py:172`](../../src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py#L172) |
| Timer | `1.0 / heartbeat_hz` | `` | CALLBACK | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py:179`](../../src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py#L179) |
| Timer | `max(1.0, float(self.get_parameter('statistics_period_sec').value))` | `` | CALLBACK | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py:180`](../../src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py#L180) |
| Topic | `str(self.get_parameter('arm_direction_topic').value)` | `geometry_msgs.msg.Vector3Stamped` | SUB | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py:187`](../../src/rebotarm_pulse/rebotarm_pulse/piper_pulse_target.py#L187) |

### rebotarm_pulse / PiperToolGeometry

Node Name：`piper_tool_geometry`；Executable：piper_tool_geometry, piper_tool_geometry；Source：[`src/rebotarm_pulse/rebotarm_pulse/piper_tool_geometry.py:1`](../../src/rebotarm_pulse/rebotarm_pulse/piper_tool_geometry.py#L1)。

Hardware=依对象为串口压力/相机TF/机械臂反馈；Startup=配置与服务检查；Target=把脉/诊断；Update=以下Timer与参数；Fault=对象局部处理，是否整机互锁 UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'parent_frame'` | `"piperh/pulse_tool_envelope"` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/piper_tool_geometry.py:52`](../../src/rebotarm_pulse/rebotarm_pulse/piper_tool_geometry.py#L52) |
| Parameter | `'geometry_calibrated'` | `False` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/piper_tool_geometry.py:53`](../../src/rebotarm_pulse/rebotarm_pulse/piper_tool_geometry.py#L53) |
| Parameter | `'support_length_m'` | `0.11` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/piper_tool_geometry.py:54`](../../src/rebotarm_pulse/rebotarm_pulse/piper_tool_geometry.py#L54) |
| Parameter | `'support_width_m'` | `0.025` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/piper_tool_geometry.py:55`](../../src/rebotarm_pulse/rebotarm_pulse/piper_tool_geometry.py#L55) |
| Parameter | `'support_depth_m'` | `0.018` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/piper_tool_geometry.py:56`](../../src/rebotarm_pulse/rebotarm_pulse/piper_tool_geometry.py#L56) |
| Parameter | `'approx_support_center_xyz'` | `[0.0, 0.0, -0.055]` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/piper_tool_geometry.py:57`](../../src/rebotarm_pulse/rebotarm_pulse/piper_tool_geometry.py#L57) |
| Parameter | `'approx_support_rpy'` | `[0.0, 0.0, 0.0]` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/piper_tool_geometry.py:58`](../../src/rebotarm_pulse/rebotarm_pulse/piper_tool_geometry.py#L58) |
| Parameter | `'approx_sensor_offsets_m'` | `[0.0, 0.010, -0.025, 0.0, 0.010, -0.055, 0.0, 0.010, -0.085],` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/piper_tool_geometry.py:59`](../../src/rebotarm_pulse/rebotarm_pulse/piper_tool_geometry.py#L59) |
| Parameter | `'calibrated_sensor_offsets_m'` | `[0.0] * 9` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/piper_tool_geometry.py:63`](../../src/rebotarm_pulse/rebotarm_pulse/piper_tool_geometry.py#L63) |
| Parameter | `'contact_normal_xyz'` | `[0.0, 0.0, 0.0]` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/piper_tool_geometry.py:64`](../../src/rebotarm_pulse/rebotarm_pulse/piper_tool_geometry.py#L64) |
| Topic | `'/piperh/pulse/tool_markers'` | `visualization_msgs.msg.MarkerArray` | PUB | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/piper_tool_geometry.py:94`](../../src/rebotarm_pulse/rebotarm_pulse/piper_tool_geometry.py#L94) |
| Timer | `1.0` | `` | CALLBACK | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/piper_tool_geometry.py:105`](../../src/rebotarm_pulse/rebotarm_pulse/piper_tool_geometry.py#L105) |

### rebotarm_pulse / PressureSerialBridge

Node Name：`pressure_serial_bridge`；Executable：pressure_serial_bridge, pressure_serial_bridge；Source：[`src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py:1`](../../src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py#L1)。

Hardware=依对象为串口压力/相机TF/机械臂反馈；Startup=配置与服务检查；Target=把脉/诊断；Update=以下Timer与参数；Fault=对象局部处理，是否整机互锁 UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'port'` | `""` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py:28`](../../src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py#L28) |
| Parameter | `'baud_rate'` | `921600` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py:29`](../../src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py#L29) |
| Parameter | `'topic_prefix'` | `"/piperh/pulse"` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py:30`](../../src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py#L30) |
| Parameter | `'sensor_frame'` | `""` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py:31`](../../src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py#L31) |
| Parameter | `'zero_calibrated'` | `False` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py:32`](../../src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py#L32) |
| Parameter | `'zero_baselines_pa'` | `[0.0, 0.0, 0.0]` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py:33`](../../src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py#L33) |
| Parameter | `'zero_window_seconds'` | `3.0` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py:34`](../../src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py#L34) |
| Parameter | `'zero_min_samples'` | `30` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py:35`](../../src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py#L35) |
| Topic | `f'{prefix}/raw/{channel.lower()}/pressure'` | `sensor_msgs.msg.FluidPressure` | PUB | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py:74`](../../src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py#L74) |
| Topic | `f'{prefix}/raw/{channel.lower()}/temperature'` | `sensor_msgs.msg.Temperature` | PUB | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py:80`](../../src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py#L80) |
| Topic | `f'{prefix}/zeroed/{channel.lower()}/pressure'` | `sensor_msgs.msg.FluidPressure` | PUB | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py:86`](../../src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py#L86) |
| Topic | `f'{prefix}/serial_connected'` | `std_msgs.msg.Bool` | PUB | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py:94`](../../src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py#L94) |
| Topic | `f'{prefix}/zero_calibrated'` | `std_msgs.msg.Bool` | PUB | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py:97`](../../src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py#L97) |
| Service | `f'{prefix}/calibrate_zero'` | `std_srvs.srv.Trigger` | SERVER | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py:103`](../../src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py#L103) |
| Timer | `0.01` | `` | CALLBACK | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py:112`](../../src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py#L112) |

### rebotarm_pulse / PulseApproach

Node Name：`helper / MODULE / derived node；NEED_CONFIRMATION`；Executable：pulse_approach, pulse_approach；Source：[`src/rebotarm_pulse/rebotarm_pulse/pulse_approach.py:1`](../../src/rebotarm_pulse/rebotarm_pulse/pulse_approach.py#L1)。

Hardware=依对象为串口压力/相机TF/机械臂反馈；Startup=配置与服务检查；Target=把脉/诊断；Update=以下Timer与参数；Fault=对象局部处理，是否整机互锁 UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Service | `'/plan_kinematic_path'` | `moveit_msgs.srv.GetMotionPlan` | CLIENT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/pulse_approach.py:36`](../../src/rebotarm_pulse/rebotarm_pulse/pulse_approach.py#L36) |
| Topic | `str(self._param('target_topic'))` | `geometry_msgs.msg.PointStamped` | SUB | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/pulse_approach.py:43`](../../src/rebotarm_pulse/rebotarm_pulse/pulse_approach.py#L43) |
| Topic | `'/rebot_xbox/armed'` | `std_msgs.msg.Bool` | SUB | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/pulse_approach.py:52`](../../src/rebotarm_pulse/rebotarm_pulse/pulse_approach.py#L52) |
| Topic | `'/rebotarm/pulse/target'` | `geometry_msgs.msg.PointStamped` | PUB | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/pulse_approach.py:53`](../../src/rebotarm_pulse/rebotarm_pulse/pulse_approach.py#L53) |
| Topic | `'/rebotarm/pulse/precontact'` | `geometry_msgs.msg.PointStamped` | PUB | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/pulse_approach.py:56`](../../src/rebotarm_pulse/rebotarm_pulse/pulse_approach.py#L56) |
| Topic | `'/rebotarm/pulse/markers'` | `visualization_msgs.msg.Marker` | PUB | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/pulse_approach.py:59`](../../src/rebotarm_pulse/rebotarm_pulse/pulse_approach.py#L59) |

### rebotarm_pulse / PulseWebGateway

Node Name：`pulse_web_gateway`；Executable：pulse_web_gateway, pulse_web_gateway；Source：[`src/rebotarm_pulse/rebotarm_pulse/pulse_web_gateway.py:1`](../../src/rebotarm_pulse/rebotarm_pulse/pulse_web_gateway.py#L1)。

Hardware=依对象为串口压力/相机TF/机械臂反馈；Startup=配置与服务检查；Target=把脉/诊断；Update=以下Timer与参数；Fault=对象局部处理，是否整机互锁 UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'host'` | `"127.0.0.1"` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/pulse_web_gateway.py:122`](../../src/rebotarm_pulse/rebotarm_pulse/pulse_web_gateway.py#L122) |
| Parameter | `'port'` | `8765` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/pulse_web_gateway.py:123`](../../src/rebotarm_pulse/rebotarm_pulse/pulse_web_gateway.py#L123) |
| Parameter | `'topic_prefix'` | `"/piperh/pulse"` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/pulse_web_gateway.py:124`](../../src/rebotarm_pulse/rebotarm_pulse/pulse_web_gateway.py#L124) |
| Parameter | `'site_directory'` | `""` | DEFAULT | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/pulse_web_gateway.py:125`](../../src/rebotarm_pulse/rebotarm_pulse/pulse_web_gateway.py#L125) |
| Topic | `f'{prefix}/raw/{lower}/temperature'` | `sensor_msgs.msg.Temperature` | SUB | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/pulse_web_gateway.py:143`](../../src/rebotarm_pulse/rebotarm_pulse/pulse_web_gateway.py#L143) |
| Topic | `f'{prefix}/raw/{lower}/pressure'` | `sensor_msgs.msg.FluidPressure` | SUB | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/pulse_web_gateway.py:149`](../../src/rebotarm_pulse/rebotarm_pulse/pulse_web_gateway.py#L149) |
| Topic | `f'{prefix}/serial_connected'` | `std_msgs.msg.Bool` | SUB | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/pulse_web_gateway.py:159`](../../src/rebotarm_pulse/rebotarm_pulse/pulse_web_gateway.py#L159) |
| Topic | `f'{prefix}/zero_calibrated'` | `std_msgs.msg.Bool` | SUB | `__init__` | [`src/rebotarm_pulse/rebotarm_pulse/pulse_web_gateway.py:167`](../../src/rebotarm_pulse/rebotarm_pulse/pulse_web_gateway.py#L167) |

### rebotarmcontroller / MODULE

Node Name：`helper / MODULE / derived node；NEED_CONFIRMATION`；Executable：GravityCompensation；Source：[`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/examples/gravity_compensation.py:1`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/examples/gravity_compensation.py#L1)。

Hardware=reBotArm SDK/MotorBridge；Startup=hardware.connect；Target=reBotArm；Update=joint_state_rate/SDK rate，见参数；Fault=局部状态拒绝/hold/清理，整机传播未发现。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Service | `f'/{_NAMESPACE}/enable'` | `std_srvs.srv.Trigger` | CLIENT | `main` | [`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/examples/gravity_compensation.py:55`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/examples/gravity_compensation.py#L55) |
| Service | `f'/{_NAMESPACE}/gravity_compensation/start'` | `std_srvs.srv.Trigger` | CLIENT | `main` | [`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/examples/gravity_compensation.py:59`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/examples/gravity_compensation.py#L59) |
| Service | `f'/{_NAMESPACE}/safe_home'` | `std_srvs.srv.Trigger` | CLIENT | `main` | [`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/examples/gravity_compensation.py:63`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/examples/gravity_compensation.py#L63) |
| Service | `f'/{_NAMESPACE}/disable'` | `std_srvs.srv.Trigger` | CLIENT | `main` | [`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/examples/gravity_compensation.py:67`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/examples/gravity_compensation.py#L67) |

### rebotarmcontroller / DemoGripperControl

Node Name：`gripper_control`；Executable：GripperControl；Source：[`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/examples/gripper_control.py:1`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/examples/gripper_control.py#L1)。

Hardware=reBotArm SDK/MotorBridge；Startup=hardware.connect；Target=reBotArm；Update=joint_state_rate/SDK rate，见参数；Fault=局部状态拒绝/hold/清理，整机传播未发现。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Service | `f'/{_NAMESPACE}/enable'` | `std_srvs.srv.Trigger` | CLIENT | `__init__` | [`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/examples/gripper_control.py:18`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/examples/gripper_control.py#L18) |
| Service | `f'/{_NAMESPACE}/disable'` | `std_srvs.srv.Trigger` | CLIENT | `__init__` | [`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/examples/gripper_control.py:19`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/examples/gripper_control.py#L19) |
| Service | `f'/{_NAMESPACE}/gripper/open'` | `rebotarm_msgs.srv.GripperCommand` | CLIENT | `__init__` | [`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/examples/gripper_control.py:20`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/examples/gripper_control.py#L20) |
| Service | `f'/{_NAMESPACE}/gripper/close'` | `rebotarm_msgs.srv.GripperCommand` | CLIENT | `__init__` | [`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/examples/gripper_control.py:21`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/examples/gripper_control.py#L21) |

### rebotarmcontroller / DemoMoveTo

Node Name：`move_to`；Executable：MoveTo；Source：[`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/examples/move_to.py:1`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/examples/move_to.py#L1)。

Hardware=reBotArm SDK/MotorBridge；Startup=hardware.connect；Target=reBotArm；Update=joint_state_rate/SDK rate，见参数；Fault=局部状态拒绝/hold/清理，整机传播未发现。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Topic | `f'/{_NAMESPACE}/joint_states'` | `sensor_msgs.msg.JointState` | SUB | `__init__` | [`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/examples/move_to.py:44`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/examples/move_to.py#L44) |
| Action | `f'/{_NAMESPACE}/follow_joint_trajectory'` | `control_msgs.action.FollowJointTrajectory` | CLIENT | `__init__` | [`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/examples/move_to.py:50`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/examples/move_to.py#L50) |

### rebotarmcontroller / DemoMoveToPose

Node Name：`move_to_pose`；Executable：MoveToPose；Source：[`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/examples/move_to_pose.py:1`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/examples/move_to_pose.py#L1)。

Hardware=reBotArm SDK/MotorBridge；Startup=hardware.connect；Target=reBotArm；Update=joint_state_rate/SDK rate，见参数；Fault=局部状态拒绝/hold/清理，整机传播未发现。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Topic | `f'/{self._namespace}/joint_states'` | `sensor_msgs.msg.JointState` | SUB | `__init__` | [`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/examples/move_to_pose.py:24`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/examples/move_to_pose.py#L24) |
| Action | `f'/{self._namespace}/move_to_pose'` | `rebotarm_msgs.action.MoveToPose` | CLIENT | `__init__` | [`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/examples/move_to_pose.py:30`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/examples/move_to_pose.py#L30) |

### rebotarmcontroller / MotorPassthrough

Node Name：`helper / MODULE / derived node；NEED_CONFIRMATION`；Executable：见 SOURCE_SCAN 入口和 launch 附录；UNKNOWN / NEED_CONFIRMATION；Source：[`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/motor_passthrough.py:1`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/motor_passthrough.py#L1)。

Hardware=reBotArm SDK/MotorBridge；Startup=hardware.connect；Target=reBotArm；Update=joint_state_rate/SDK rate，见参数；Fault=局部状态拒绝/hold/清理，整机传播未发现。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Topic | `topic` | `msg_type` | SUB | `_subscribe` | [`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/motor_passthrough.py:83`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/motor_passthrough.py#L83) |

### rebotarmcontroller / reBotArmController

Node Name：`reBotArmController`；Executable：reBotArmController；Source：[`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/rebotarm_controller.py:1`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/rebotarm_controller.py#L1)。

Hardware=reBotArm SDK/MotorBridge；Startup=hardware.connect；Target=reBotArm；Update=joint_state_rate/SDK rate，见参数；Fault=局部状态拒绝/hold/清理，整机传播未发现。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'hardware_config'` | `""` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/rebotarm_controller.py:27`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/rebotarm_controller.py#L27) |
| Parameter | `'model'` | `""` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/rebotarm_controller.py:28`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/rebotarm_controller.py#L28) |
| Parameter | `'channel'` | `""` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/rebotarm_controller.py:29`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/rebotarm_controller.py#L29) |
| Parameter | `'joint_state_rate'` | `100.0` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/rebotarm_controller.py:30`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/rebotarm_controller.py#L30) |
| Parameter | `'arm_namespace'` | `"rebotarm"` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/rebotarm_controller.py:31`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/rebotarm_controller.py#L31) |
| Parameter | `'cmd_arbitration'` | `"reject"` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/rebotarm_controller.py:32`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/rebotarm_controller.py#L32) |
| Parameter | `'frame_id'` | `"base_link"` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/rebotarm_controller.py:33`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/rebotarm_controller.py#L33) |
| Parameter | `'ee_frame_id'` | `"end_link"` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/rebotarm_controller.py:34`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/rebotarm_controller.py#L34) |
| Parameter | `'disable_after_safe_home'` | `True` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/rebotarm_controller.py:35`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/rebotarm_controller.py#L35) |
| Parameter | `'servo_joint_trajectory_topic'` | `""` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/rebotarm_controller.py:36`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/rebotarm_controller.py#L36) |
| Parameter | `'servo_command_timeout'` | `0.15` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/rebotarm_controller.py:37`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/rebotarm_controller.py#L37) |
| Parameter | `'trajectory_path_tolerance'` | `0.03` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/rebotarm_controller.py:38`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/rebotarm_controller.py#L38) |
| Parameter | `'trajectory_goal_tolerance'` | `0.005` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/rebotarm_controller.py:39`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/rebotarm_controller.py#L39) |
| Parameter | `'trajectory_goal_time_tolerance'` | `1.5` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/rebotarm_controller.py:40`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/rebotarm_controller.py#L40) |
| Parameter | `'trajectory_stopped_velocity_tolerance'` | `0.02` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/rebotarm_controller.py:41`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/rebotarm_controller.py#L41) |
| Parameter | `'trajectory_tracking_slowdown_ratio'` | `0.5` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/rebotarm_controller.py:42`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/rebotarm_controller.py#L42) |
| Parameter | `'trajectory_tracking_stop_ratio'` | `0.75` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/rebotarm_controller.py:43`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/rebotarm_controller.py#L43) |
| Parameter | `'trajectory_execution_timeout_scaling'` | `4.0` | DEFAULT | `__init__` | [`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/rebotarm_controller.py:44`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/rebotarm_controller.py#L44) |

### rebotarmcontroller / ArmActions

Node Name：`helper / MODULE / derived node；NEED_CONFIRMATION`；Executable：见 SOURCE_SCAN 入口和 launch 附录；UNKNOWN / NEED_CONFIRMATION；Source：[`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/ros_actions.py:1`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/ros_actions.py#L1)。

Hardware=reBotArm SDK/MotorBridge；Startup=hardware.connect；Target=reBotArm；Update=joint_state_rate/SDK rate，见参数；Fault=局部状态拒绝/hold/清理，整机传播未发现。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Action | `f'/{namespace}/move_to_pose'` | `rebotarm_msgs.action.MoveToPose` | SERVER | `__init__` | [`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/ros_actions.py:128`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/ros_actions.py#L128) |
| Action | `f'/{namespace}/follow_joint_trajectory'` | `control_msgs.action.FollowJointTrajectory` | SERVER | `__init__` | [`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/ros_actions.py:137`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/ros_actions.py#L137) |
| Action | `f'/{namespace}/gripper/command'` | `control_msgs.action.GripperCommand` | SERVER | `__init__` | [`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/ros_actions.py:146`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/ros_actions.py#L146) |

### rebotarmcontroller / JointStatePublisher

Node Name：`helper / MODULE / derived node；NEED_CONFIRMATION`；Executable：见 SOURCE_SCAN 入口和 launch 附录；UNKNOWN / NEED_CONFIRMATION；Source：[`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/ros_publishers.py:1`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/ros_publishers.py#L1)。

Hardware=reBotArm SDK/MotorBridge；Startup=hardware.connect；Target=reBotArm；Update=joint_state_rate/SDK rate，见参数；Fault=局部状态拒绝/hold/清理，整机传播未发现。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Topic | `f'/{namespace}/joint_states'` | `sensor_msgs.msg.JointState` | PUB | `__init__` | [`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/ros_publishers.py:26`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/ros_publishers.py#L26) |
| Topic | `f'/{namespace}/joints/{name}/state'` | `rebotarm_msgs.msg.JointMotorState` | PUB | `__init__` | [`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/ros_publishers.py:33`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/ros_publishers.py#L33) |
| Topic | `f'/{namespace}/arm_status'` | `rebotarm_msgs.msg.ArmStatus` | PUB | `__init__` | [`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/ros_publishers.py:46`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/ros_publishers.py#L46) |
| Topic | `f'/{namespace}/gripper/state'` | `rebotarm_msgs.msg.JointMotorState` | PUB | `__init__` | [`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/ros_publishers.py:59`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/ros_publishers.py#L59) |
| Timer | `period` | `` | CALLBACK | `__init__` | [`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/ros_publishers.py:66`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/ros_publishers.py#L66) |

### rebotarmcontroller / ArmServices

Node Name：`helper / MODULE / derived node；NEED_CONFIRMATION`；Executable：见 SOURCE_SCAN 入口和 launch 附录；UNKNOWN / NEED_CONFIRMATION；Source：[`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/ros_services.py:1`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/ros_services.py#L1)。

Hardware=reBotArm SDK/MotorBridge；Startup=hardware.connect；Target=reBotArm；Update=joint_state_rate/SDK rate，见参数；Fault=局部状态拒绝/hold/清理，整机传播未发现。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Service | `f'/{namespace}/{name}'` | `srv_type` | SERVER | `__init__` | [`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/ros_services.py:44`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/ros_services.py#L44) |

### rebotarmcontroller / ServoTrajectoryInput

Node Name：`helper / MODULE / derived node；NEED_CONFIRMATION`；Executable：见 SOURCE_SCAN 入口和 launch 附录；UNKNOWN / NEED_CONFIRMATION；Source：[`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/servo_trajectory_input.py:1`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/servo_trajectory_input.py#L1)。

Hardware=reBotArm SDK/MotorBridge；Startup=hardware.connect；Target=reBotArm；Update=joint_state_rate/SDK rate，见参数；Fault=局部状态拒绝/hold/清理，整机传播未发现。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Topic | `topic` | `trajectory_msgs.msg.JointTrajectory` | SUB | `__init__` | [`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/servo_trajectory_input.py:36`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/servo_trajectory_input.py#L36) |
| Timer | `min(timeout / 2.0, 0.05)` | `` | CALLBACK | `__init__` | [`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/servo_trajectory_input.py:43`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/servo_trajectory_input.py#L43) |

### red_point_localizer / RedPointDetector

Node Name：`red_point_detector`；Executable：red_point_detector；Source：[`dual_arm_ws/src/red_point_localizer/red_point_localizer/red_point_detector.py:1`](../../dual_arm_ws/src/red_point_localizer/red_point_localizer/red_point_detector.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'image_topic'` | `'/camera/color/image_raw'` | DEFAULT | `__init__` | [`dual_arm_ws/src/red_point_localizer/red_point_localizer/red_point_detector.py:43`](../../dual_arm_ws/src/red_point_localizer/red_point_localizer/red_point_detector.py#L43) |
| Parameter | `'depth_topic'` | `'/camera/depth/image_raw'` | DEFAULT | `__init__` | [`dual_arm_ws/src/red_point_localizer/red_point_localizer/red_point_detector.py:46`](../../dual_arm_ws/src/red_point_localizer/red_point_localizer/red_point_detector.py#L46) |
| Parameter | `'camera_info_topic'` | `'/camera/color/camera_info'` | DEFAULT | `__init__` | [`dual_arm_ws/src/red_point_localizer/red_point_localizer/red_point_detector.py:49`](../../dual_arm_ws/src/red_point_localizer/red_point_localizer/red_point_detector.py#L49) |
| Parameter | `'debug_topic'` | `'/red_point/debug_image'` | DEFAULT | `__init__` | [`dual_arm_ws/src/red_point_localizer/red_point_localizer/red_point_detector.py:54`](../../dual_arm_ws/src/red_point_localizer/red_point_localizer/red_point_detector.py#L54) |
| Parameter | `'pixel_topic'` | `'/red_point/pixel'` | DEFAULT | `__init__` | [`dual_arm_ws/src/red_point_localizer/red_point_localizer/red_point_detector.py:57`](../../dual_arm_ws/src/red_point_localizer/red_point_localizer/red_point_detector.py#L57) |
| Parameter | `'camera_point_topic'` | `'/red_point/camera_point'` | DEFAULT | `__init__` | [`dual_arm_ws/src/red_point_localizer/red_point_localizer/red_point_detector.py:60`](../../dual_arm_ws/src/red_point_localizer/red_point_localizer/red_point_detector.py#L60) |
| Parameter | `'min_area'` | `100.0` | DEFAULT | `__init__` | [`dual_arm_ws/src/red_point_localizer/red_point_localizer/red_point_detector.py:64`](../../dual_arm_ws/src/red_point_localizer/red_point_localizer/red_point_detector.py#L64) |
| Parameter | `'depth_window_size'` | `7` | DEFAULT | `__init__` | [`dual_arm_ws/src/red_point_localizer/red_point_localizer/red_point_detector.py:66`](../../dual_arm_ws/src/red_point_localizer/red_point_localizer/red_point_detector.py#L66) |
| Parameter | `'min_valid_depth_count'` | `5` | DEFAULT | `__init__` | [`dual_arm_ws/src/red_point_localizer/red_point_localizer/red_point_detector.py:74`](../../dual_arm_ws/src/red_point_localizer/red_point_localizer/red_point_detector.py#L74) |
| Parameter | `'min_depth_m'` | `0.10` | DEFAULT | `__init__` | [`dual_arm_ws/src/red_point_localizer/red_point_localizer/red_point_detector.py:77`](../../dual_arm_ws/src/red_point_localizer/red_point_localizer/red_point_detector.py#L77) |
| Parameter | `'max_depth_m'` | `2.50` | DEFAULT | `__init__` | [`dual_arm_ws/src/red_point_localizer/red_point_localizer/red_point_detector.py:80`](../../dual_arm_ws/src/red_point_localizer/red_point_localizer/red_point_detector.py#L80) |
| Parameter | `'sync_slop_sec'` | `0.08` | DEFAULT | `__init__` | [`dual_arm_ws/src/red_point_localizer/red_point_localizer/red_point_detector.py:83`](../../dual_arm_ws/src/red_point_localizer/red_point_localizer/red_point_detector.py#L83) |
| Parameter | `'depth_scale_16u'` | `0.001` | DEFAULT | `__init__` | [`dual_arm_ws/src/red_point_localizer/red_point_localizer/red_point_detector.py:86`](../../dual_arm_ws/src/red_point_localizer/red_point_localizer/red_point_detector.py#L86) |
| Parameter | `'sample_csv_path'` | `''` | DEFAULT | `__init__` | [`dual_arm_ws/src/red_point_localizer/red_point_localizer/red_point_detector.py:89`](../../dual_arm_ws/src/red_point_localizer/red_point_localizer/red_point_detector.py#L89) |
| Parameter | `'debug_image_path'` | `''` | DEFAULT | `__init__` | [`dual_arm_ws/src/red_point_localizer/red_point_localizer/red_point_detector.py:92`](../../dual_arm_ws/src/red_point_localizer/red_point_localizer/red_point_detector.py#L92) |
| Parameter | `'color_mode'` | `'red'` | DEFAULT | `__init__` | [`dual_arm_ws/src/red_point_localizer/red_point_localizer/red_point_detector.py:96`](../../dual_arm_ws/src/red_point_localizer/red_point_localizer/red_point_detector.py#L96) |
| Topic | `debug_topic` | `sensor_msgs.msg.Image` | PUB | `__init__` | [`dual_arm_ws/src/red_point_localizer/red_point_localizer/red_point_detector.py:114`](../../dual_arm_ws/src/red_point_localizer/red_point_localizer/red_point_detector.py#L114) |
| Topic | `pixel_topic` | `geometry_msgs.msg.PointStamped` | PUB | `__init__` | [`dual_arm_ws/src/red_point_localizer/red_point_localizer/red_point_detector.py:115`](../../dual_arm_ws/src/red_point_localizer/red_point_localizer/red_point_detector.py#L115) |
| Topic | `camera_point_topic` | `geometry_msgs.msg.PointStamped` | PUB | `__init__` | [`dual_arm_ws/src/red_point_localizer/red_point_localizer/red_point_detector.py:116`](../../dual_arm_ws/src/red_point_localizer/red_point_localizer/red_point_detector.py#L116) |
| Topic | `self.camera_info_topic` | `sensor_msgs.msg.CameraInfo` | SUB | `__init__` | [`dual_arm_ws/src/red_point_localizer/red_point_localizer/red_point_detector.py:119`](../../dual_arm_ws/src/red_point_localizer/red_point_localizer/red_point_detector.py#L119) |

### robotcar_navigation / KeepoutZoneManager

Node Name：`keepout_zone_manager`；Executable：见 SOURCE_SCAN 入口和 launch 附录；UNKNOWN / NEED_CONFIRMATION；Source：[`src/robotcar_navigation/scripts/keepout_zone_manager.py:1`](../../src/robotcar_navigation/scripts/keepout_zone_manager.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'keepout_file'` | `""` | DEFAULT | `__init__` | [`src/robotcar_navigation/scripts/keepout_zone_manager.py:63`](../../src/robotcar_navigation/scripts/keepout_zone_manager.py#L63) |
| Topic | `'/keepout_filter_mask'` | `nav_msgs.msg.OccupancyGrid` | PUB | `__init__` | [`src/robotcar_navigation/scripts/keepout_zone_manager.py:69`](../../src/robotcar_navigation/scripts/keepout_zone_manager.py#L69) |
| Topic | `'/costmap_filter_info'` | `nav2_msgs.msg.CostmapFilterInfo` | PUB | `__init__` | [`src/robotcar_navigation/scripts/keepout_zone_manager.py:72`](../../src/robotcar_navigation/scripts/keepout_zone_manager.py#L72) |
| Topic | `'/waterplus/keepout_markers'` | `visualization_msgs.msg.MarkerArray` | PUB | `__init__` | [`src/robotcar_navigation/scripts/keepout_zone_manager.py:75`](../../src/robotcar_navigation/scripts/keepout_zone_manager.py#L75) |
| Topic | `'/waterplus/keepout_status'` | `std_msgs.msg.String` | PUB | `__init__` | [`src/robotcar_navigation/scripts/keepout_zone_manager.py:78`](../../src/robotcar_navigation/scripts/keepout_zone_manager.py#L78) |
| Topic | `'/map'` | `nav_msgs.msg.OccupancyGrid` | SUB | `__init__` | [`src/robotcar_navigation/scripts/keepout_zone_manager.py:81`](../../src/robotcar_navigation/scripts/keepout_zone_manager.py#L81) |
| Topic | `'/waterplus/add_keepout'` | `robotcar_navigation.msg.KeepoutZone` | SUB | `__init__` | [`src/robotcar_navigation/scripts/keepout_zone_manager.py:84`](../../src/robotcar_navigation/scripts/keepout_zone_manager.py#L84) |
| Topic | `'/waterplus/delete_keepout'` | `std_msgs.msg.String` | SUB | `__init__` | [`src/robotcar_navigation/scripts/keepout_zone_manager.py:87`](../../src/robotcar_navigation/scripts/keepout_zone_manager.py#L87) |
| Service | `'/waterplus/get_keepout_names'` | `robotcar_navigation.srv.GetWaypointNames` | SERVER | `__init__` | [`src/robotcar_navigation/scripts/keepout_zone_manager.py:90`](../../src/robotcar_navigation/scripts/keepout_zone_manager.py#L90) |
| Service | `'/waterplus/rename_keepout'` | `robotcar_navigation.srv.RenameWaypoint` | SERVER | `__init__` | [`src/robotcar_navigation/scripts/keepout_zone_manager.py:93`](../../src/robotcar_navigation/scripts/keepout_zone_manager.py#L93) |
| Service | `'/waterplus/save_keepout_group'` | `robotcar_navigation.srv.WaypointFile` | SERVER | `__init__` | [`src/robotcar_navigation/scripts/keepout_zone_manager.py:96`](../../src/robotcar_navigation/scripts/keepout_zone_manager.py#L96) |
| Service | `'/waterplus/load_keepout_group'` | `robotcar_navigation.srv.WaypointFile` | SERVER | `__init__` | [`src/robotcar_navigation/scripts/keepout_zone_manager.py:99`](../../src/robotcar_navigation/scripts/keepout_zone_manager.py#L99) |
| Service | `'/waterplus/enable_keepouts'` | `std_srvs.srv.SetBool` | SERVER | `__init__` | [`src/robotcar_navigation/scripts/keepout_zone_manager.py:102`](../../src/robotcar_navigation/scripts/keepout_zone_manager.py#L102) |

### robotcar_navigation / PurePursuitPlanner

Node Name：`pure_pursuit_planner`；Executable：见 SOURCE_SCAN 入口和 launch 附录；UNKNOWN / NEED_CONFIRMATION；Source：[`src/robotcar_navigation/scripts/pure_pursuit_planner.py:1`](../../src/robotcar_navigation/scripts/pure_pursuit_planner.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'lookahead_distance'` | `0.6` | DEFAULT | `__init__` | [`src/robotcar_navigation/scripts/pure_pursuit_planner.py:18`](../../src/robotcar_navigation/scripts/pure_pursuit_planner.py#L18) |
| Parameter | `'target_speed'` | `0.3` | DEFAULT | `__init__` | [`src/robotcar_navigation/scripts/pure_pursuit_planner.py:19`](../../src/robotcar_navigation/scripts/pure_pursuit_planner.py#L19) |
| Parameter | `'max_omega'` | `1.0` | DEFAULT | `__init__` | [`src/robotcar_navigation/scripts/pure_pursuit_planner.py:20`](../../src/robotcar_navigation/scripts/pure_pursuit_planner.py#L20) |
| Parameter | `'path_topic'` | `"/plan"` | DEFAULT | `__init__` | [`src/robotcar_navigation/scripts/pure_pursuit_planner.py:21`](../../src/robotcar_navigation/scripts/pure_pursuit_planner.py#L21) |
| Parameter | `'cmd_vel_topic'` | `"/cmd_vel"` | DEFAULT | `__init__` | [`src/robotcar_navigation/scripts/pure_pursuit_planner.py:22`](../../src/robotcar_navigation/scripts/pure_pursuit_planner.py#L22) |
| Parameter | `'base_frame'` | `"base_footprint"` | DEFAULT | `__init__` | [`src/robotcar_navigation/scripts/pure_pursuit_planner.py:23`](../../src/robotcar_navigation/scripts/pure_pursuit_planner.py#L23) |
| Topic | `self.get_parameter('path_topic').value` | `nav_msgs.msg.Path` | SUB | `__init__` | [`src/robotcar_navigation/scripts/pure_pursuit_planner.py:42`](../../src/robotcar_navigation/scripts/pure_pursuit_planner.py#L42) |
| Topic | `self.get_parameter('cmd_vel_topic').value` | `geometry_msgs.msg.Twist` | PUB | `__init__` | [`src/robotcar_navigation/scripts/pure_pursuit_planner.py:45`](../../src/robotcar_navigation/scripts/pure_pursuit_planner.py#L45) |
| Timer | `0.05` | `` | CALLBACK | `__init__` | [`src/robotcar_navigation/scripts/pure_pursuit_planner.py:46`](../../src/robotcar_navigation/scripts/pure_pursuit_planner.py#L46) |

### robotcar_navigation / RobotAwarenessMonitor

Node Name：`robot_awareness_monitor`；Executable：见 SOURCE_SCAN 入口和 launch 附录；UNKNOWN / NEED_CONFIRMATION；Source：[`src/robotcar_navigation/scripts/robot_awareness_monitor.py:1`](../../src/robotcar_navigation/scripts/robot_awareness_monitor.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'stuck_history_file'` | `""` | DEFAULT | `__init__` | [`src/robotcar_navigation/scripts/robot_awareness_monitor.py:46`](../../src/robotcar_navigation/scripts/robot_awareness_monitor.py#L46) |
| Parameter | `'base_half_length'` | `0.43` | DEFAULT | `__init__` | [`src/robotcar_navigation/scripts/robot_awareness_monitor.py:47`](../../src/robotcar_navigation/scripts/robot_awareness_monitor.py#L47) |
| Parameter | `'base_half_width'` | `0.29` | DEFAULT | `__init__` | [`src/robotcar_navigation/scripts/robot_awareness_monitor.py:48`](../../src/robotcar_navigation/scripts/robot_awareness_monitor.py#L48) |
| Parameter | `'base_margin'` | `0.08` | DEFAULT | `__init__` | [`src/robotcar_navigation/scripts/robot_awareness_monitor.py:49`](../../src/robotcar_navigation/scripts/robot_awareness_monitor.py#L49) |
| Parameter | `'speed_margin_gain'` | `0.45` | DEFAULT | `__init__` | [`src/robotcar_navigation/scripts/robot_awareness_monitor.py:50`](../../src/robotcar_navigation/scripts/robot_awareness_monitor.py#L50) |
| Parameter | `'view_range'` | `3.5` | DEFAULT | `__init__` | [`src/robotcar_navigation/scripts/robot_awareness_monitor.py:51`](../../src/robotcar_navigation/scripts/robot_awareness_monitor.py#L51) |
| Parameter | `'view_fov_deg'` | `70.0` | DEFAULT | `__init__` | [`src/robotcar_navigation/scripts/robot_awareness_monitor.py:52`](../../src/robotcar_navigation/scripts/robot_awareness_monitor.py#L52) |
| Topic | `'/antbot/awareness_markers'` | `visualization_msgs.msg.MarkerArray` | PUB | `__init__` | [`src/robotcar_navigation/scripts/robot_awareness_monitor.py:68`](../../src/robotcar_navigation/scripts/robot_awareness_monitor.py#L68) |
| Topic | `'/antbot/safety_envelope'` | `geometry_msgs.msg.PolygonStamped` | PUB | `__init__` | [`src/robotcar_navigation/scripts/robot_awareness_monitor.py:71`](../../src/robotcar_navigation/scripts/robot_awareness_monitor.py#L71) |
| Topic | `'/local_costmap/footprint'` | `geometry_msgs.msg.Polygon` | PUB | `__init__` | [`src/robotcar_navigation/scripts/robot_awareness_monitor.py:74`](../../src/robotcar_navigation/scripts/robot_awareness_monitor.py#L74) |
| Topic | `'/global_costmap/footprint'` | `geometry_msgs.msg.Polygon` | PUB | `__init__` | [`src/robotcar_navigation/scripts/robot_awareness_monitor.py:77`](../../src/robotcar_navigation/scripts/robot_awareness_monitor.py#L77) |
| Topic | `'/antbot/awareness_status'` | `std_msgs.msg.String` | PUB | `__init__` | [`src/robotcar_navigation/scripts/robot_awareness_monitor.py:80`](../../src/robotcar_navigation/scripts/robot_awareness_monitor.py#L80) |
| Topic | `'/odometry/filtered'` | `nav_msgs.msg.Odometry` | SUB | `__init__` | [`src/robotcar_navigation/scripts/robot_awareness_monitor.py:83`](../../src/robotcar_navigation/scripts/robot_awareness_monitor.py#L83) |
| Topic | `'/amcl_pose'` | `geometry_msgs.msg.PoseWithCovarianceStamped` | SUB | `__init__` | [`src/robotcar_navigation/scripts/robot_awareness_monitor.py:86`](../../src/robotcar_navigation/scripts/robot_awareness_monitor.py#L86) |
| Topic | `f'/{action_name}/_action/status'` | `action_msgs.msg.GoalStatusArray` | SUB | `__init__` | [`src/robotcar_navigation/scripts/robot_awareness_monitor.py:90`](../../src/robotcar_navigation/scripts/robot_awareness_monitor.py#L90) |
| Timer | `0.2` | `` | CALLBACK | `__init__` | [`src/robotcar_navigation/scripts/robot_awareness_monitor.py:97`](../../src/robotcar_navigation/scripts/robot_awareness_monitor.py#L97) |

### robotcar_navigation / RobotIntentMonitor

Node Name：`robot_intent_monitor`；Executable：见 SOURCE_SCAN 入口和 launch 附录；UNKNOWN / NEED_CONFIRMATION；Source：[`src/robotcar_navigation/scripts/robot_intent_monitor.py:1`](../../src/robotcar_navigation/scripts/robot_intent_monitor.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Topic | `'/antbot/robot_intent'` | `std_msgs.msg.String` | PUB | `__init__` | [`src/robotcar_navigation/scripts/robot_intent_monitor.py:29`](../../src/robotcar_navigation/scripts/robot_intent_monitor.py#L29) |
| Topic | `'/antbot/led_intent'` | `std_msgs.msg.String` | PUB | `__init__` | [`src/robotcar_navigation/scripts/robot_intent_monitor.py:32`](../../src/robotcar_navigation/scripts/robot_intent_monitor.py#L32) |
| Topic | `'/cmd_vel'` | `geometry_msgs.msg.Twist` | SUB | `__init__` | [`src/robotcar_navigation/scripts/robot_intent_monitor.py:35`](../../src/robotcar_navigation/scripts/robot_intent_monitor.py#L35) |
| Topic | `'/odometry/filtered'` | `nav_msgs.msg.Odometry` | SUB | `__init__` | [`src/robotcar_navigation/scripts/robot_intent_monitor.py:36`](../../src/robotcar_navigation/scripts/robot_intent_monitor.py#L36) |
| Topic | `'/plan'` | `nav_msgs.msg.Path` | SUB | `__init__` | [`src/robotcar_navigation/scripts/robot_intent_monitor.py:39`](../../src/robotcar_navigation/scripts/robot_intent_monitor.py#L39) |
| Topic | `'/antbot/passing_request'` | `std_msgs.msg.String` | SUB | `__init__` | [`src/robotcar_navigation/scripts/robot_intent_monitor.py:40`](../../src/robotcar_navigation/scripts/robot_intent_monitor.py#L40) |
| Topic | `'/antbot/dock_request'` | `std_msgs.msg.String` | SUB | `__init__` | [`src/robotcar_navigation/scripts/robot_intent_monitor.py:43`](../../src/robotcar_navigation/scripts/robot_intent_monitor.py#L43) |
| Topic | `'/waterplus/route_progress'` | `std_msgs.msg.String` | SUB | `__init__` | [`src/robotcar_navigation/scripts/robot_intent_monitor.py:46`](../../src/robotcar_navigation/scripts/robot_intent_monitor.py#L46) |
| Topic | `'/battery'` | `sensor_msgs.msg.BatteryState` | SUB | `__init__` | [`src/robotcar_navigation/scripts/robot_intent_monitor.py:49`](../../src/robotcar_navigation/scripts/robot_intent_monitor.py#L49) |
| Timer | `0.2` | `` | CALLBACK | `__init__` | [`src/robotcar_navigation/scripts/robot_intent_monitor.py:61`](../../src/robotcar_navigation/scripts/robot_intent_monitor.py#L61) |

### robotcar_navigation / SpeedZoneManager

Node Name：`speed_zone_manager`；Executable：见 SOURCE_SCAN 入口和 launch 附录；UNKNOWN / NEED_CONFIRMATION；Source：[`src/robotcar_navigation/scripts/speed_zone_manager.py:1`](../../src/robotcar_navigation/scripts/speed_zone_manager.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'speed_zone_file'` | `""` | DEFAULT | `__init__` | [`src/robotcar_navigation/scripts/speed_zone_manager.py:62`](../../src/robotcar_navigation/scripts/speed_zone_manager.py#L62) |
| Topic | `'/speed_filter_mask'` | `nav_msgs.msg.OccupancyGrid` | PUB | `__init__` | [`src/robotcar_navigation/scripts/speed_zone_manager.py:68`](../../src/robotcar_navigation/scripts/speed_zone_manager.py#L68) |
| Topic | `'/speed_costmap_filter_info'` | `nav2_msgs.msg.CostmapFilterInfo` | PUB | `__init__` | [`src/robotcar_navigation/scripts/speed_zone_manager.py:71`](../../src/robotcar_navigation/scripts/speed_zone_manager.py#L71) |
| Topic | `'/waterplus/speed_zone_markers'` | `visualization_msgs.msg.MarkerArray` | PUB | `__init__` | [`src/robotcar_navigation/scripts/speed_zone_manager.py:74`](../../src/robotcar_navigation/scripts/speed_zone_manager.py#L74) |
| Topic | `'/waterplus/speed_zone_status'` | `std_msgs.msg.String` | PUB | `__init__` | [`src/robotcar_navigation/scripts/speed_zone_manager.py:77`](../../src/robotcar_navigation/scripts/speed_zone_manager.py#L77) |
| Topic | `'/map'` | `nav_msgs.msg.OccupancyGrid` | SUB | `__init__` | [`src/robotcar_navigation/scripts/speed_zone_manager.py:80`](../../src/robotcar_navigation/scripts/speed_zone_manager.py#L80) |
| Topic | `'/waterplus/add_speed_zone'` | `robotcar_navigation.msg.SpeedZone` | SUB | `__init__` | [`src/robotcar_navigation/scripts/speed_zone_manager.py:81`](../../src/robotcar_navigation/scripts/speed_zone_manager.py#L81) |
| Topic | `'/waterplus/delete_speed_zone'` | `std_msgs.msg.String` | SUB | `__init__` | [`src/robotcar_navigation/scripts/speed_zone_manager.py:82`](../../src/robotcar_navigation/scripts/speed_zone_manager.py#L82) |
| Service | `'/waterplus/get_speed_zone_names'` | `robotcar_navigation.srv.GetWaypointNames` | SERVER | `__init__` | [`src/robotcar_navigation/scripts/speed_zone_manager.py:83`](../../src/robotcar_navigation/scripts/speed_zone_manager.py#L83) |
| Service | `'/waterplus/rename_speed_zone'` | `robotcar_navigation.srv.RenameWaypoint` | SERVER | `__init__` | [`src/robotcar_navigation/scripts/speed_zone_manager.py:86`](../../src/robotcar_navigation/scripts/speed_zone_manager.py#L86) |
| Service | `'/waterplus/save_speed_zone_group'` | `robotcar_navigation.srv.WaypointFile` | SERVER | `__init__` | [`src/robotcar_navigation/scripts/speed_zone_manager.py:89`](../../src/robotcar_navigation/scripts/speed_zone_manager.py#L89) |
| Service | `'/waterplus/load_speed_zone_group'` | `robotcar_navigation.srv.WaypointFile` | SERVER | `__init__` | [`src/robotcar_navigation/scripts/speed_zone_manager.py:92`](../../src/robotcar_navigation/scripts/speed_zone_manager.py#L92) |
| Service | `'/waterplus/enable_speed_zones'` | `std_srvs.srv.SetBool` | SERVER | `__init__` | [`src/robotcar_navigation/scripts/speed_zone_manager.py:95`](../../src/robotcar_navigation/scripts/speed_zone_manager.py#L95) |

### robotcar_navigation / NavigateWaypoints

Node Name：`wr_navigate_py`；Executable：见 SOURCE_SCAN 入口和 launch 附录；UNKNOWN / NEED_CONFIRMATION；Source：[`src/robotcar_navigation/scripts/wr_navigate.py:1`](../../src/robotcar_navigation/scripts/wr_navigate.py#L1)。

Hardware/Startup/Control target：按下列调用与 launch/config 复查，未解析的运行依赖 UNKNOWN / NEED_CONFIRMATION；Update=以下Timer/频率参数；Fault behavior=未做逐路径运行验证，UNKNOWN / NEED_CONFIRMATION。

| 类型 | Interface / period / parameter | Message Type / default | Direction | Function | Evidence |
|---|---|---|---|---|---|
| Parameter | `'waypoints_file'` | `""` | DEFAULT | `__init__` | [`src/robotcar_navigation/scripts/wr_navigate.py:27`](../../src/robotcar_navigation/scripts/wr_navigate.py#L27) |
| Parameter | `'route_file'` | `""` | DEFAULT | `__init__` | [`src/robotcar_navigation/scripts/wr_navigate.py:28`](../../src/robotcar_navigation/scripts/wr_navigate.py#L28) |
| Topic | `'/waterplus/route_progress'` | `std_msgs.msg.String` | PUB | `__init__` | [`src/robotcar_navigation/scripts/wr_navigate.py:38`](../../src/robotcar_navigation/scripts/wr_navigate.py#L38) |
| Action | `'navigate_to_pose'` | `nav2_msgs.action.NavigateToPose` | CLIENT | `__init__` | [`src/robotcar_navigation/scripts/wr_navigate.py:52`](../../src/robotcar_navigation/scripts/wr_navigate.py#L52) |
| Topic | `'/yolo_trigger'` | `std_msgs.msg.String` | PUB | `__init__` | [`src/robotcar_navigation/scripts/wr_navigate.py:53`](../../src/robotcar_navigation/scripts/wr_navigate.py#L53) |
| Topic | `'/waypoint_reached'` | `std_msgs.msg.String` | PUB | `__init__` | [`src/robotcar_navigation/scripts/wr_navigate.py:54`](../../src/robotcar_navigation/scripts/wr_navigate.py#L54) |
| Topic | `'/yolo_result'` | `std_msgs.msg.String` | SUB | `__init__` | [`src/robotcar_navigation/scripts/wr_navigate.py:55`](../../src/robotcar_navigation/scripts/wr_navigate.py#L55) |
| Topic | `'/waterplus/route_control'` | `std_msgs.msg.String` | SUB | `__init__` | [`src/robotcar_navigation/scripts/wr_navigate.py:58`](../../src/robotcar_navigation/scripts/wr_navigate.py#L58) |

## C++ 节点与 RViz 插件完整接口调用证据

C++ 动态成员/topic 表达式未做语义求值；下列源文件与调用上下文用于复查。RViz插件的节点是嵌入进程对象，不单独计作可执行ROS节点。定时器/故障/参数仍须对应类实现核实，UNKNOWN / NEED_CONFIRMATION。

- `moveit_calibration_gui` / `dual_arm_ws/src/moveit_calibration/moveit_calibration_gui/handeye_calibration_rviz_plugin/src/handeye_target_widget.cpp:146`

```cpp
advertise("/handeye_calibration/target_detection", 1)
```

- `moveit_calibration_gui` / `dual_arm_ws/src/moveit_calibration/moveit_calibration_gui/handeye_calibration_rviz_plugin/src/handeye_target_widget.cpp:421`

```cpp
sendTransform(tf2_msg)
```

- `moveit_calibration_gui` / `dual_arm_ws/src/moveit_calibration/moveit_calibration_gui/handeye_calibration_rviz_plugin/src/handeye_target_widget.cpp:560`

```cpp
subscribe to image topic.")
```

- `rviz_visual_tools` / `dual_arm_ws/src/rviz_visual_tools/src/remote_control.cpp:59`

```cpp
create_subscription<sensor_msgs::msg::Joy>(
      topics_interface_, rviz_dashboard_topic, rclcpp::SystemDefaultsQoS(),
      std::bind(&RemoteControl::rvizDashboardCallback, this, std::placeholders::_1))
```

- `rviz_visual_tools` / `dual_arm_ws/src/rviz_visual_tools/src/rviz_visual_tools.cpp:331`

```cpp
create_publisher<visualization_msgs::msg::MarkerArray>(
      topics_interface_, marker_topic_, feedback_pub_qos)
```

- `rviz_visual_tools` / `dual_arm_ws/src/rviz_visual_tools/src/tf_visual_tools.cpp:124`

```cpp
sendTransform(transforms_)
```

- `rviz_visual_tools` / `dual_arm_ws/src/rviz_visual_tools/include/rviz_visual_tools/remote_reciever.hpp:46`

```cpp
create_publisher<sensor_msgs::msg::Joy>("/rviz_visual_tools_gui", rclcpp::QoS(100))
```

- `piperh_motion_rviz` / `dual_arm_ws/src/piperh_motion_rviz/src/motion_preset_panel.cpp:278`

```cpp
create_subscription<rebotarm_msgs::msg::ArmStatus>(
    "/piperh/arm_status", status_qos,
    [this](const rebotarm_msgs::msg::ArmStatus::SharedPtr message) {
      const bool enabled = message->enabled
```

- `piperh_motion_rviz` / `dual_arm_ws/src/piperh_motion_rviz/src/motion_preset_panel.cpp:314`

```cpp
create_subscription<std_msgs::msg::String>(
      "/dual_arm/selected", selected_qos,
      [this](const std_msgs::msg::String::SharedPtr message) {selectedRobotCallback(message)
```

- `piperh_motion_rviz` / `dual_arm_ws/src/piperh_motion_rviz/src/motion_preset_panel.cpp:381`

```cpp
create_subscription<sensor_msgs::msg::JointState>(
    joint_states_topic, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::JointState::SharedPtr message) {
      jointStateCallback(message)
```

- `piperh_motion_rviz` / `dual_arm_ws/src/piperh_motion_rviz/src/motion_preset_panel.cpp:386`

```cpp
create_publisher<std_msgs::msg::String>(prefix + "/motion_progress", 10)
```

- `piperh_motion_rviz` / `dual_arm_ws/src/piperh_motion_rviz/src/motion_preset_panel.cpp:387`

```cpp
create_client<MoveGroup>(node_, move_action)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts/service_benchmark_node.cpp:91`

```cpp
create_client<ServiceT>(service_name_)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/tools/topic_statistics.hpp:38`

```cpp
create_subscription<sensor_msgs::msg::Image>(
        image_topic_, 10, std::bind(&TopicStatistics::image_callback, this, std::placeholders::_1),
        sub_opt)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/tools/topic_statistics.hpp:43`

```cpp
create_subscription<statistics_msgs::msg::MetricsMessage>(
        statistics_topic_, 10,
        std::bind(&TopicStatistics::statistics_callback, this, std::placeholders::_1))
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/tools/frame_latency.cpp:55`

```cpp
create_wall_timer(1s, [this, topic_name = topic_name]() {
    // print fps
    RCLCPP_INFO_STREAM(logger_, "topic: " << topic_name << " fps: " << frame_count_ / 1.0)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/tools/frame_latency.cpp:60`

```cpp
create_subscription<MsgType>(
      topic_name, rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile), qos_profile),
      [&, this](const std::shared_ptr<MsgType> msg) {
        rclcpp::Time curr_time = this->get_clock()->now()
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/tools/frame_latency.cpp:76`

```cpp
create_subscription<tf2_msgs::msg::TFMessage>(
      topic_name, rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile), qos_profile),
      [&, this](const std::shared_ptr<tf2_msgs::msg::TFMessage> msg) {
        rclcpp::Time curr_time = this->get_clock()->now()
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/tools/multi_save_rgbir.cpp:74`

```cpp
create_service<orbbec_camera_msgs::srv::SetInt32>(
        "start_capture", std::bind(&MultiCameraSubscriber::controlCaptureCallback, this,
                                   std::placeholders::_1, std::placeholders::_2))
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/tools/multi_save_rgbir.cpp:151`

```cpp
create_subscription<sensor_msgs::msg::Image>(
          left_ir_topics_[i], custom_qos,
          [this, i](std::shared_ptr<const sensor_msgs::msg::Image> msg) {
            this->irCallback(msg, i)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/tools/multi_save_rgbir.cpp:158`

```cpp
create_subscription<orbbec_camera_msgs::msg::Metadata>(
          left_ir_metadata_topic_[i], custom_qos,
          [this, i](std::shared_ptr<const orbbec_camera_msgs::msg::Metadata> msg) {
            this->ir_meta_Callback(msg, i)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/tools/multi_save_rgbir.cpp:164`

```cpp
create_subscription<sensor_msgs::msg::Image>(
          color_topics_[i], custom_qos,
          [this, i](std::shared_ptr<const sensor_msgs::msg::Image> msg) {
            this->colorCallback(msg, i)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/tools/multi_save_rgbir.cpp:171`

```cpp
create_subscription<orbbec_camera_msgs::msg::Metadata>(
          color_metadata_topic_[i], custom_qos,
          [this, i](std::shared_ptr<const orbbec_camera_msgs::msg::Metadata> msg) {
            this->color_meta_Callback(msg, i)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/tools/ob_benchmark.cpp:19`

```cpp
create_wall_timer(std::chrono::seconds(test_cycle_),
                                        std::bind(&ObBenchmark::functionCallback, this))
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/tools/ob_benchmark.cpp:21`

```cpp
create_wall_timer(std::chrono::seconds(switch_cycle_),
                                      std::bind(&ObBenchmark::configCallback, this))
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/tools/start_benchmark.cpp:20`

```cpp
create_subscription<sensor_msgs::msg::Image>(
          color_topics_[i], custom_qos,
          [this, i](std::shared_ptr<const sensor_msgs::msg::Image> msg) {
            this->color_Callback(msg, i)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/tools/start_benchmark.cpp:25`

```cpp
create_subscription<sensor_msgs::msg::Image>(
          depth_topics_[i], custom_qos,
          [this, i](std::shared_ptr<const sensor_msgs::msg::Image> msg) {
            this->depth_Callback(msg, i)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/tools/start_benchmark.cpp:30`

```cpp
create_subscription<sensor_msgs::msg::Image>(
          left_ir_topics_[i], custom_qos,
          [this, i](std::shared_ptr<const sensor_msgs::msg::Image> msg) {
            this->left_ir_Callback(msg, i)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/tools/start_benchmark.cpp:35`

```cpp
create_subscription<sensor_msgs::msg::Image>(
          right_ir_topics_[i], custom_qos,
          [this, i](std::shared_ptr<const sensor_msgs::msg::Image> msg) {
            this->right_ir_Callback(msg, i)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/tools/start_benchmark.cpp:40`

```cpp
create_subscription<sensor_msgs::msg::PointCloud2>(
          depth_point_cloud_topics_[i], custom_qos,
          [this, i](std::shared_ptr<const sensor_msgs::msg::PointCloud2> msg) {
            this->depth_point_cloud_Callback(msg, i)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/tools/start_benchmark.cpp:45`

```cpp
create_subscription<sensor_msgs::msg::PointCloud2>(
          color_point_cloud_topics_[i], custom_qos,
          [this, i](std::shared_ptr<const sensor_msgs::msg::PointCloud2> msg) {
            this->color_point_cloud_Callback(msg, i)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ob_camera_node_driver.cpp:294`

```cpp
create_service<std_srvs::srv::Empty>(
      "reboot_device", std::bind(&OBCameraNodeDriver::rebootDeviceCallback, this,
                                 std::placeholders::_1, std::placeholders::_2))
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ob_camera_node_driver.cpp:335`

```cpp
create_wall_timer(std::chrono::milliseconds(1000), [this]() { checkConnectTimer()
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ob_camera_node_driver.cpp:339`

```cpp
create_wall_timer(std::chrono::milliseconds(1000 / device_status_interval_hz),
                                [this]() { deviceStatusTimer()
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ob_camera_node_driver.cpp:346`

```cpp
create_publisher<orbbec_camera_msgs::msg::DeviceStatus>("device_status", qos)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ob_camera_node_driver.cpp:1013`

```cpp
create_wall_timer(time_sync_period_, [this]() {
        // Multiple safety checks before attempting time sync
        if (!device_) {
          RCLCPP_DEBUG_STREAM(logger_, "sync_host_time_timer_: device is null, skip time sync")
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/image_publisher.cpp:22`

```cpp
create_publisher<sensor_msgs::msg::Image>(
      topic_name, rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos), qos))
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/image_publisher.cpp:39`

```cpp
create_publisher(&node, topic_name, qos))
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ob_camera_node.cpp:837`

```cpp
create_wall_timer(software_trigger_period_, [this]() {
        if (software_trigger_enabled_) {
          TRY_EXECUTE_BLOCK(device_->triggerCapture())
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ob_camera_node.cpp:1829`

```cpp
create_service<SetFilter>(
      "set_filter", [this](const std::shared_ptr<SetFilter ::Request> request,
                           std::shared_ptr<SetFilter ::Response> response) {
        setFilterCallback(request, response)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ob_camera_node.cpp:2922`

```cpp
create_wall_timer(std::chrono::seconds(int(diagnostic_period_)), [this]() {
          try {
            // Check if we're still running and all components are valid
            if (!is_running_.load() || !diagnostic_updater_ ||
                !is_camera_node_initialized_.load() || !device_) {
              return
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ob_camera_node.cpp:3047`

```cpp
create_publisher<PointCloud2>(
        "depth_registered/points",
        rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(point_cloud_qos_profile),
                    point_cloud_qos_profile))
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ob_camera_node.cpp:3053`

```cpp
create_publisher<PointCloud2>(
        "depth/points", rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(point_cloud_qos_profile),
                                    point_cloud_qos_profile))
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ob_camera_node.cpp:3084`

```cpp
create_publisher<CameraInfo>(
        topic, rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(camera_info_qos_profile),
                           camera_info_qos_profile))
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ob_camera_node.cpp:3089`

```cpp
create_publisher<orbbec_camera_msgs::msg::Metadata>(
              name + "/metadata",
              rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(camera_info_qos_profile),
                          camera_info_qos_profile))
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ob_camera_node.cpp:3095`

```cpp
create_publisher<CameraInfo>(
          "color/camera_info_undistorted",
          rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(camera_info_qos_profile),
                      camera_info_qos_profile))
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ob_camera_node.cpp:3129`

```cpp
create_publisher<sensor_msgs::msg::Imu>(
        topic_name, rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(data_qos), data_qos))
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ob_camera_node.cpp:3132`

```cpp
create_publisher<orbbec_camera_msgs::msg::IMUInfo>(
        topic_name, rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(data_qos), data_qos))
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ob_camera_node.cpp:3135`

```cpp
create_publisher<orbbec_camera_msgs::msg::IMUInfo>(
        topic_name, rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(data_qos), data_qos))
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ob_camera_node.cpp:3147`

```cpp
create_publisher<sensor_msgs::msg::Imu>(
          data_topic_name, rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(data_qos), data_qos))
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ob_camera_node.cpp:3151`

```cpp
create_publisher<orbbec_camera_msgs::msg::IMUInfo>(
              data_topic_name,
              rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(data_qos), data_qos))
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ob_camera_node.cpp:3162`

```cpp
create_publisher<orbbec_camera_msgs::msg::Extrinsics>("depth_to_ir", extrinsics_qos)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ob_camera_node.cpp:3166`

```cpp
create_publisher<orbbec_camera_msgs::msg::Extrinsics>("depth_to_color",
                                                                     extrinsics_qos)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ob_camera_node.cpp:3171`

```cpp
create_publisher<orbbec_camera_msgs::msg::Extrinsics>("depth_to_left_ir",
                                                                     extrinsics_qos)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ob_camera_node.cpp:3176`

```cpp
create_publisher<orbbec_camera_msgs::msg::Extrinsics>("depth_to_right_ir",
                                                                     extrinsics_qos)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ob_camera_node.cpp:3181`

```cpp
create_publisher<orbbec_camera_msgs::msg::Extrinsics>("depth_to_accel",
                                                                     extrinsics_qos)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ob_camera_node.cpp:3186`

```cpp
create_publisher<orbbec_camera_msgs::msg::Extrinsics>("depth_to_gyro",
                                                                     extrinsics_qos)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ob_camera_node.cpp:3191`

```cpp
create_publisher<orbbec_camera_msgs::msg::Extrinsics>("left_color_to_right_color",
                                                                     extrinsics_qos)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ob_camera_node.cpp:3195`

```cpp
create_publisher<std_msgs::msg::String>("depth_filter_status", extrinsics_qos)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ob_camera_node.cpp:3200`

```cpp
create_publisher<DepthFiltersStatus>("depth_filters/status", extrinsics_qos)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ob_camera_node.cpp:4848`

```cpp
sendTransform(static_tf_msgs_)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ob_camera_node.cpp:4864`

```cpp
sendTransform(static_tf_msgs_)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/d2c_viewer.cpp:49`

```cpp
create_publisher<sensor_msgs::msg::Image>("depth_to_color/image_raw", qos)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ob_lidar_node.cpp:397`

```cpp
create_publisher<sensor_msgs::msg::LaserScan>(
        "scan/points", rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(point_cloud_qos_profile),
                                   point_cloud_qos_profile))
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ob_lidar_node.cpp:402`

```cpp
create_publisher<sensor_msgs::msg::PointCloud2>(
        "cloud/points", rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(point_cloud_qos_profile),
                                    point_cloud_qos_profile))
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ob_lidar_node.cpp:412`

```cpp
create_publisher<sensor_msgs::msg::Imu>(
        topic_name, rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(data_qos), data_qos))
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ob_lidar_node.cpp:421`

```cpp
create_publisher<orbbec_camera_msgs::msg::Extrinsics>(
            "/" + camera_name_ + "/lidar_to_imu", extrinsics_qos)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ob_lidar_node.cpp:1172`

```cpp
sendTransform(static_tf_msgs_)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ob_lidar_node.cpp:1379`

```cpp
sendTransform(static_tf_msgs_)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ros_service.cpp:64`

```cpp
create_service<GetInt32>(
        service_name,
        [this, stream_index = stream_index](const std::shared_ptr<GetInt32::Request> request,
                                            std::shared_ptr<GetInt32::Response> response) {
          getExposureCallback(request, response, stream_index)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ros_service.cpp:72`

```cpp
create_service<SetInt32>(
        service_name,
        [this, stream_index = stream_index](const std::shared_ptr<SetInt32::Request> request,
                                            std::shared_ptr<SetInt32::Response> response) {
          setExposureCallback(request, response, stream_index)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ros_service.cpp:79`

```cpp
create_service<GetInt32>(
        service_name,
        [this, stream_index = stream_index](const std::shared_ptr<GetInt32::Request> request,
                                            std::shared_ptr<GetInt32::Response> response) {
          getGainCallback(request, response, stream_index)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ros_service.cpp:87`

```cpp
create_service<SetInt32>(
        service_name,
        [this, stream_index = stream_index](const std::shared_ptr<SetInt32::Request> request,
                                            std::shared_ptr<SetInt32::Response> response) {
          setGainCallback(request, response, stream_index)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ros_service.cpp:94`

```cpp
create_service<SetBool>(
        service_name,
        [this, stream_index = stream_index](const std::shared_ptr<SetBool::Request> request,
                                            std::shared_ptr<SetBool::Response> response) {
          setAutoExposureCallback(request, response, stream_index)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ros_service.cpp:102`

```cpp
create_service<SetArrays>(
        service_name,
        [this, stream_index = stream_index](const std::shared_ptr<SetArrays::Request> request,
                                            std::shared_ptr<SetArrays::Response> response) {
          setAeRoiCallback(request, response, stream_index)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ros_service.cpp:111`

```cpp
create_service<SetBool>(
        service_name,
        [this, stream_index = stream_index](const std::shared_ptr<SetBool::Request> request,
                                            std::shared_ptr<SetBool::Response> response) {
          toggleSensorCallback(request, response, stream_index)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ros_service.cpp:118`

```cpp
create_service<SetBool>(
        service_name,
        [this, stream_index = stream_index](const std::shared_ptr<SetBool::Request> request,
                                            std::shared_ptr<SetBool::Response> response) {
          setMirrorCallback(request, response, stream_index)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ros_service.cpp:125`

```cpp
create_service<SetBool>(
        service_name,
        [this, stream_index = stream_index](const std::shared_ptr<SetBool::Request> request,
                                            std::shared_ptr<SetBool::Response> response) {
          setFlipCallback(request, response, stream_index)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ros_service.cpp:132`

```cpp
create_service<SetInt32>(
        service_name,
        [this, stream_index = stream_index](const std::shared_ptr<SetInt32::Request> request,
                                            std::shared_ptr<SetInt32::Response> response) {
          setRotationCallback(request, response, stream_index)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ros_service.cpp:139`

```cpp
create_service<SetInt32>(
      "set_fan_work_mode", [this](const std::shared_ptr<SetInt32::Request> request,
                                  std::shared_ptr<SetInt32::Response> response) {
        setFanWorkModeCallback(request, response)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ros_service.cpp:144`

```cpp
create_service<SetBool>(
      "set_floor_enable", [this](const std::shared_ptr<rmw_request_id_t> request_header,
                                 const std::shared_ptr<SetBool::Request> request,
                                 std::shared_ptr<SetBool::Response> response) {
        setFloorEnableCallback(request_header, request, response)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ros_service.cpp:150`

```cpp
create_service<SetBool>(
      "set_laser_enable", [this](const std::shared_ptr<rmw_request_id_t> request_header,
                                 const std::shared_ptr<SetBool::Request> request,
                                 std::shared_ptr<SetBool::Response> response) {
        setLaserEnableCallback(request_header, request, response)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ros_service.cpp:156`

```cpp
create_service<SetBool>(
      "set_ldp_enable", [this](const std::shared_ptr<rmw_request_id_t> request_header,
                               const std::shared_ptr<SetBool::Request> request,
                               std::shared_ptr<SetBool::Response> response) {
        setLdpEnableCallback(request_header, request, response)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ros_service.cpp:162`

```cpp
create_service<GetBool>(
      "get_ldp_status", [this](const std::shared_ptr<rmw_request_id_t> request_header,
                               const std::shared_ptr<GetBool::Request> request,
                               std::shared_ptr<GetBool::Response> response) {
        (void)request_header
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ros_service.cpp:169`

```cpp
create_service<GetBool>(
      "get_laser_status", [this](const std::shared_ptr<rmw_request_id_t> request_header,
                                 const std::shared_ptr<GetBool::Request> request,
                                 std::shared_ptr<GetBool::Response> response) {
        (void)request_header
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ros_service.cpp:176`

```cpp
create_service<SetBool>(
      "set_ptp_config", [this](const std::shared_ptr<rmw_request_id_t> request_header,
                               const std::shared_ptr<SetBool::Request> request,
                               std::shared_ptr<SetBool::Response> response) {
        setPtpConfigCallback(request_header, request, response)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ros_service.cpp:182`

```cpp
create_service<GetBool>(
      "get_ptp_config", [this](const std::shared_ptr<rmw_request_id_t> request_header,
                               const std::shared_ptr<GetBool::Request> request,
                               std::shared_ptr<GetBool::Response> response) {
        (void)request_header
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ros_service.cpp:190`

```cpp
create_service<GetInt32>(
      "get_white_balance", [this](const std::shared_ptr<GetInt32::Request> request,
                                  std::shared_ptr<GetInt32::Response> response) {
        getWhiteBalanceCallback(request, response)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ros_service.cpp:196`

```cpp
create_service<SetInt32>(
      "set_white_balance", [this](const std::shared_ptr<SetInt32::Request> request,
                                  std::shared_ptr<SetInt32::Response> response) {
        setWhiteBalanceCallback(request, response)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ros_service.cpp:201`

```cpp
create_service<GetInt32>(
      "get_auto_white_balance", [this](const std::shared_ptr<GetInt32::Request> request,
                                       std::shared_ptr<GetInt32::Response> response) {
        getAutoWhiteBalanceCallback(request, response)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ros_service.cpp:206`

```cpp
create_service<SetBool>(
      "set_auto_white_balance", [this](const std::shared_ptr<SetBool::Request> request,
                                       std::shared_ptr<SetBool::Response> response) {
        setAutoWhiteBalanceCallback(request, response)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ros_service.cpp:211`

```cpp
create_service<GetDeviceInfo>(
      "get_device_info", [this](const std::shared_ptr<GetDeviceInfo::Request> request,
                                std::shared_ptr<GetDeviceInfo::Response> response) {
        getDeviceInfoCallback(request, response)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ros_service.cpp:216`

```cpp
create_service<GetString>(
      "get_sdk_version",
      [this](const std::shared_ptr<GetString::Request> request,
             std::shared_ptr<GetString::Response> response) { getSDKVersion(request, response)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ros_service.cpp:220`

```cpp
create_service<std_srvs::srv::Empty>(
      "save_images", [this](const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                            std::shared_ptr<std_srvs::srv::Empty::Response> response) {
        saveImageCallback(request, response)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ros_service.cpp:225`

```cpp
create_service<std_srvs::srv::Empty>(
      "save_point_cloud", [this](const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                 std::shared_ptr<std_srvs::srv::Empty::Response> response) {
        savePointCloudCallback(request, response)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ros_service.cpp:230`

```cpp
create_service<SetString>(
      "switch_ir", [this](const std::shared_ptr<SetString::Request> request,
                          std::shared_ptr<SetString::Response> response) {
        switchIRCameraCallback(request, response)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ros_service.cpp:235`

```cpp
create_service<SetBool>(
      "set_ir_long_exposure", [this](const std::shared_ptr<SetBool::Request> request,
                                     std::shared_ptr<SetBool::Response> response) {
        setIRLongExposureCallback(request, response)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ros_service.cpp:240`

```cpp
create_service<GetInt32>(
      "get_lrm_measure_distance", [this](const std::shared_ptr<GetInt32::Request> request,
                                         std::shared_ptr<GetInt32::Response> response) {
        getLrmMeasureDistanceCallback(request, response)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ros_service.cpp:245`

```cpp
create_service<SetBool>(
      "set_reset_timestamp", [this](const std::shared_ptr<SetBool::Request> request,
                                    std::shared_ptr<SetBool::Response> response) {
        setRESETTimestampCallback(request, response)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ros_service.cpp:250`

```cpp
create_service<SetInt32>(
      "set_sync_interleaverlaser", [this](const std::shared_ptr<SetInt32::Request> request,
                                          std::shared_ptr<SetInt32::Response> response) {
        setSYNCInterleaveLaserCallback(request, response)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ros_service.cpp:255`

```cpp
create_service<SetBool>(
      "set_sync_hosttime", [this](const std::shared_ptr<SetBool::Request> request,
                                  std::shared_ptr<SetBool::Response> response) {
        setSYNCHostimeCallback(request, response)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ros_service.cpp:260`

```cpp
create_service<SetBool>(
      "send_software_trigger", [this](const std::shared_ptr<SetBool::Request> request,
                                      std::shared_ptr<SetBool::Response> response) {
        sendSoftwareTriggerCallback(request, response)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ros_service.cpp:266`

```cpp
create_service<SetString>(
        "write_customer_data", [this](const std::shared_ptr<SetString::Request> request,
                                      std::shared_ptr<SetString::Response> response) {
          writeCustomerDataCallback(request, response)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ros_service.cpp:271`

```cpp
create_service<GetString>(
        "read_customer_data", [this](const std::shared_ptr<GetString::Request> request,
                                     std::shared_ptr<GetString::Response> response) {
          readCustomerDataCallback(request, response)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ros_service.cpp:276`

```cpp
create_service<SetUserCalibParams>(
        "set_user_calib_params", [this](const std::shared_ptr<SetUserCalibParams::Request> request,
                                        std::shared_ptr<SetUserCalibParams::Response> response) {
          setUserCalibParamsCallback(request, response)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ros_service.cpp:281`

```cpp
create_service<GetUserCalibParams>(
        "get_user_calib_params", [this](const std::shared_ptr<GetUserCalibParams::Request> request,
                                        std::shared_ptr<GetUserCalibParams::Response> response) {
          getUserCalibParamsCallback(request, response)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ros_service.cpp:287`

```cpp
create_service<SetString>(
      "set_ae_reference_stream", [this](const std::shared_ptr<SetString::Request> request,
                                        std::shared_ptr<SetString::Response> response) {
        setAEReferenceStreamCallback(request, response)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ros_service.cpp:292`

```cpp
create_service<SetString>(
      "set_ae_strategy", [this](const std::shared_ptr<SetString::Request> request,
                                std::shared_ptr<SetString::Response> response) {
        setAEStrategyCallback(request, response)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ros_service.cpp:297`

```cpp
create_service<SetBool>(
      "set_streams_enable", [this](const std::shared_ptr<SetBool::Request> request,
                                   std::shared_ptr<SetBool::Response> response) {
        setStreamsEnableCallback(request, response)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ros_service.cpp:302`

```cpp
create_service<GetBool>(
      "get_streams_enable", [this](const std::shared_ptr<GetBool::Request> request,
                                   std::shared_ptr<GetBool::Response> response) {
        getStreamsEnableCallback(request, response)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ros_service.cpp:307`

```cpp
create_service<SetInt32>(
      "set_point_cloud_decimation", [this](const std::shared_ptr<SetInt32::Request> request,
                                           std::shared_ptr<SetInt32::Response> response) {
        setPointCloudDecimationCallback(request, response)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ros_service.cpp:312`

```cpp
create_service<GetInt32>(
      "get_point_cloud_decimation", [this](const std::shared_ptr<GetInt32::Request> request,
                                           std::shared_ptr<GetInt32::Response> response) {
        getPointCloudDecimationCallback(request, response)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ros_service.cpp:317`

```cpp
create_service<SetInt32>(
      "set_disparity_range_mode", [this](const std::shared_ptr<SetInt32::Request> request,
                                         std::shared_ptr<SetInt32::Response> response) {
        setDisparityRangeModeCallback(request, response)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ros_service.cpp:322`

```cpp
create_service<SetInt32>(
      "set_disparity_search_offset", [this](const std::shared_ptr<SetInt32::Request> request,
                                            std::shared_ptr<SetInt32::Response> response) {
        setDisparitySearchOffsetCallback(request, response)
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/examples/Gemini_435Le_example_node/camera_example_node.cpp:15`

```cpp
create_client<orbbec_camera_msgs::srv::SetUserCalibParams>(
        "/camera/set_user_calib_params")
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/examples/Gemini_435Le_example_node/camera_example_node.cpp:17`

```cpp
create_client<orbbec_camera_msgs::srv::GetUserCalibParams>(
        "/camera/get_user_calib_params")
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/examples/Gemini_435Le_example_node/camera_example_node.cpp:19`

```cpp
create_client<std_srvs::srv::SetBool>("/camera/set_streams_enable")
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/examples/Gemini_435Le_example_node/camera_example_node.cpp:20`

```cpp
create_client<orbbec_camera_msgs::srv::SetArrays>(
        "/camera/set_color_ae_roi")
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/examples/Gemini_435Le_example_node/camera_example_node.cpp:23`

```cpp
create_client<orbbec_camera_msgs::srv::GetDeviceInfo>("/camera/get_device_info")
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/examples/Gemini_435Le_example_node/camera_example_node.cpp:138`

```cpp
create_subscription<orbbec_camera_msgs::msg::DeviceStatus>(
        "/camera/device_status", 10,
        std::bind(&CameraExampleNode::deviceStatusCallback, this, std::placeholders::_1))
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/examples/multi_camera_time_sync/image_sync_example_node.cpp:59`

```cpp
subscribe(this, "/camera_01/color/image_raw", qos.get_rmw_qos_profile())
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/examples/multi_camera_time_sync/image_sync_example_node.cpp:60`

```cpp
subscribe(this, "/camera_01/depth/image_raw", qos.get_rmw_qos_profile())
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/examples/multi_camera_time_sync/image_sync_example_node.cpp:61`

```cpp
subscribe(this, "/camera_02/color/image_raw", qos.get_rmw_qos_profile())
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/examples/multi_camera_time_sync/image_sync_example_node.cpp:62`

```cpp
subscribe(this, "/camera_02/depth/image_raw", qos.get_rmw_qos_profile())
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/examples/multi_camera_time_sync/image_sync_example_node.cpp:63`

```cpp
subscribe(this, "/camera_03/color/image_raw", qos.get_rmw_qos_profile())
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/examples/multi_camera_time_sync/image_sync_example_node.cpp:64`

```cpp
subscribe(this, "/camera_03/depth/image_raw", qos.get_rmw_qos_profile())
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/examples/multi_camera_time_sync/image_sync_example_node.cpp:65`

```cpp
subscribe(this, "/camera_04/color/image_raw", qos.get_rmw_qos_profile())
```

- `orbbec_camera` / `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/examples/multi_camera_time_sync/image_sync_example_node.cpp:66`

```cpp
subscribe(this, "/camera_04/depth/image_raw", qos.get_rmw_qos_profile())
```

- `moveit_visual_tools` / `dual_arm_ws/src/moveit_visual_tools/src/moveit_visual_tools.cpp:355`

```cpp
create_publisher<moveit_msgs::msg::DisplayTrajectory>(display_planned_path_topic, 10)
```

- `moveit_visual_tools` / `dual_arm_ws/src/moveit_visual_tools/src/moveit_visual_tools.cpp:368`

```cpp
subscribe to display trajectory topic: " << display_planned_path_topic)
```

- `moveit_visual_tools` / `dual_arm_ws/src/moveit_visual_tools/src/moveit_visual_tools.cpp:383`

```cpp
create_publisher<moveit_msgs::msg::DisplayRobotState>(robot_state_topic, 1)
```

- `moveit_visual_tools` / `dual_arm_ws/src/moveit_visual_tools/src/moveit_visual_tools.cpp:396`

```cpp
subscribe to robot state topic: " << robot_state_topic)
```

- `moveit_visual_tools` / `dual_arm_ws/src/moveit_visual_tools/include/moveit_visual_tools/moveit_visual_tools.h:659`

```cpp
subscribe to the DISPLAY_ROBOT_STATE_TOPIC, above
   * \param robot_state - joint values of robot
   * \param color - how to highlight the robot (solid-ly) if desired, default keeps color as specified in URDF
   * \param highlight_links - if the |color| is not |DEFAULT|, allows selective robot links to be highlighted.
   * By default (empty) all links are highlighted.
   */
  bool publishRobotState(const moveit::core::RobotState& robot_state,
   
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/demo_panel.cpp:2118`

```cpp
create_subscription<sensor_msgs::msg::Image>(
    "/meridian_hand_vision/annotated_image", rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::Image::SharedPtr message) {
      last_hand_vision_image_ms_.store(steadyMilliseconds())
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/demo_panel.cpp:2136`

```cpp
create_subscription<geometry_msgs::msg::PointStamped>(
    "/meridian_hand_vision/pulse_point", rclcpp::SensorDataQoS(),
    [this](const geometry_msgs::msg::PointStamped::SharedPtr message) {
      {
        std::lock_guard<std::mutex> lock(pulse_point_mutex_)
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/demo_panel.cpp:2160`

```cpp
create_subscription<geometry_msgs::msg::PointStamped>(
    "/rebotarm/pulse/target", rclcpp::SensorDataQoS(),
    [this](const geometry_msgs::msg::PointStamped::SharedPtr message) {
      if (active_robot_ != QStringLiteral("rebotarm")) {return
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/demo_panel.cpp:2179`

```cpp
create_subscription<geometry_msgs::msg::PointStamped>(
    "/piperh/pulse/target", rclcpp::SensorDataQoS(),
    [this](const geometry_msgs::msg::PointStamped::SharedPtr message) {
      {
        std::lock_guard<std::mutex> lock(pulse_point_mutex_)
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/demo_panel.cpp:2391`

```cpp
create_subscription<std_msgs::msg::String>(
    "/rebotarm/handeye/auto_status",
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
    [this](const std_msgs::msg::String::SharedPtr message) {
      const auto document = QJsonDocument::fromJson(
        QByteArray::fromStdString(message->data))
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/demo_panel.cpp:2548`

```cpp
create_publisher<std_msgs::msg::Empty>(
    "/rviz/moveit/update_start_state", 1)
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/demo_panel.cpp:2550`

```cpp
create_publisher<std_msgs::msg::Empty>(
    "/rviz/moveit/update_goal_state", 1)
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/demo_panel.cpp:2552`

```cpp
create_publisher<std_msgs::msg::Empty>(
    "/rviz/moveit/stop", 1)
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/demo_panel.cpp:2606`

```cpp
create_subscription<sensor_msgs::msg::JointState>(
    endpoint(QStringLiteral("/joint_states")), rclcpp::SensorDataQoS(),
    [this, robot](const sensor_msgs::msg::JointState::SharedPtr message) {
      if (active_robot_ != robot) {return
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/demo_panel.cpp:2619`

```cpp
create_subscription<std_msgs::msg::Bool>(
    endpoint(QStringLiteral("/xbox/armed")),
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
    [this, robot](const std_msgs::msg::Bool::SharedPtr message) {
      if (active_robot_ != robot) {return
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/demo_panel.cpp:2628`

```cpp
create_subscription<std_msgs::msg::String>(
      endpoint(QStringLiteral("/teach/status")),
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      [this, robot](const std_msgs::msg::String::SharedPtr message) {
        if (active_robot_ != robot) {return
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/demo_panel.cpp:2638`

```cpp
create_subscription<moveit_msgs::msg::DisplayTrajectory>(
      endpoint(QStringLiteral("/display_planned_path")), 10,
      [this, robot](moveit_msgs::msg::DisplayTrajectory::ConstSharedPtr message) {
        if (active_robot_ != robot) {return
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/demo_panel.cpp:2646`

```cpp
create_client<GetStateValidity>(
    endpoint(QStringLiteral("/check_state_validity")))
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/demo_panel.cpp:2648`

```cpp
create_client<SetBool>(
    endpoint(QStringLiteral("/xbox/set_armed")))
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/demo_panel.cpp:2650`

```cpp
create_client<ListActionGroups>(
    endpoint(QStringLiteral("/teach/list_action_groups")))
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/demo_panel.cpp:2652`

```cpp
create_client<ConfigureShapeMarker>(
    endpoint(QStringLiteral("/teach/configure_shape_marker")))
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/demo_panel.cpp:2654`

```cpp
create_client<ConfigureTrace>(
    endpoint(QStringLiteral("/teach/configure_trace")))
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/demo_panel.cpp:2656`

```cpp
create_client<CreateShapeAction>(
    endpoint(QStringLiteral("/teach/create_shape_action")))
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/demo_panel.cpp:2658`

```cpp
create_client<CheckShapeReachability>(
    endpoint(QStringLiteral("/teach/check_shape_reachability")))
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/demo_panel.cpp:2660`

```cpp
create_client<ListActionSequences>(
    endpoint(QStringLiteral("/teach/list_action_sequences")))
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/demo_panel.cpp:2662`

```cpp
create_client<CopyActionGroup>(
    endpoint(QStringLiteral("/teach/copy_action_group")))
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/demo_panel.cpp:2664`

```cpp
create_client<RenameActionGroup>(
    endpoint(QStringLiteral("/teach/rename_action_group")))
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/demo_panel.cpp:2666`

```cpp
create_client<DeleteActionGroup>(
    endpoint(QStringLiteral("/teach/delete_action_group")))
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/demo_panel.cpp:2668`

```cpp
create_client<SelectActionGroup>(
    endpoint(QStringLiteral("/teach/select_action_group")))
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/demo_panel.cpp:2670`

```cpp
create_client<PreviewActionGroup>(
    endpoint(QStringLiteral("/teach/preview_action_group")))
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/demo_panel.cpp:2672`

```cpp
create_client<PreviewActionSequence>(
    endpoint(QStringLiteral("/teach/preview_action_sequence")))
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/demo_panel.cpp:2674`

```cpp
create_client<ReplayActionGroup>(
    endpoint(QStringLiteral("/teach/replay_action_group")))
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/demo_panel.cpp:2676`

```cpp
create_client<SaveActionSequence>(
    endpoint(QStringLiteral("/teach/save_action_sequence")))
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/demo_panel.cpp:2678`

```cpp
create_client<ReplayActionSequence>(
    endpoint(QStringLiteral("/teach/replay_action_sequence")))
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/demo_panel.cpp:2680`

```cpp
create_client<Trigger>(
    endpoint(QStringLiteral("/teach/start_recording")))
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/demo_panel.cpp:2682`

```cpp
create_client<Trigger>(
    endpoint(QStringLiteral("/teach/stop_recording")))
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/demo_panel.cpp:2684`

```cpp
create_client<Trigger>(
    endpoint(QStringLiteral("/teach/cancel")))
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/demo_panel.cpp:2686`

```cpp
create_client<Trigger>(
    endpoint(QStringLiteral("/teach/reset")))
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/demo_panel.cpp:2688`

```cpp
create_client<Trigger>(
    endpoint(QStringLiteral("/teach/pause_preview")))
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/demo_panel.cpp:2690`

```cpp
create_client<Trigger>(
    endpoint(QStringLiteral("/teach/clear_action_selection")))
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/demo_panel.cpp:2692`

```cpp
create_client<ExecuteTrajectory>(
    node_, endpoint(QStringLiteral("/execute_trajectory")))
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/demo_panel.cpp:2695`

```cpp
create_publisher<moveit_msgs::msg::DisplayTrajectory>(
    endpoint(QStringLiteral("/display_planned_path")), 1)
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/demo_panel.cpp:2709`

```cpp
create_subscription<std_msgs::msg::String>(
      (prefix + QStringLiteral("/forbidden_zone_manager/status")).toStdString(),
      forbidden_zone_qos,
      [this, robot](const std_msgs::msg::String::SharedPtr message) {
        if (forbidden_zone_status_handler_) {
          forbidden_zone_status_handler_(robot, message)
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/demo_panel.cpp:2718`

```cpp
create_client<Trigger>(
    (prefix + QStringLiteral("/forbidden_zone_manager/reload")).toStdString())
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/demo_panel.cpp:2720`

```cpp
create_client<ConfigureForbiddenZone>(
    (prefix + QStringLiteral("/forbidden_zone_manager/configure")).toStdString())
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/demo_panel.cpp:6913`

```cpp
create_subscription<sensor_msgs::msg::Image>(
    camera_image_topic_.toStdString(), rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::Image::SharedPtr message) {
      const auto now = steadyMilliseconds()
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/dual_arm_selector_panel.cpp:315`

```cpp
create_publisher<std_msgs::msg::String>(
    "/dual_arm/select_request", rclcpp::QoS(10))
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/dual_arm_selector_panel.cpp:317`

```cpp
create_subscription<std_msgs::msg::String>(
    "/dual_arm/selected", qos,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      const QString value = QString::fromStdString(msg->data)
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/dual_arm_selector_panel.cpp:323`

```cpp
create_subscription<std_msgs::msg::String>(
    "/dual_arm/status", qos,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      const QString value = QString::fromStdString(msg->data)
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/dual_arm_selector_panel.cpp:336`

```cpp
create_subscription<std_msgs::msg::Bool>(
    "/rebotarm/xbox/armed", qos, armed_callback(QStringLiteral("rebotarm")))
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/dual_arm_selector_panel.cpp:338`

```cpp
create_subscription<std_msgs::msg::Bool>(
    "/piperh/xbox/armed", qos, armed_callback(QStringLiteral("piperh")))
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/dual_arm_selector_panel.cpp:340`

```cpp
create_subscription<sensor_msgs::msg::JointState>(
    "/rebotarm/joint_states", rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::JointState::SharedPtr message) {
      QMetaObject::invokeMethod(
        this, [this, message]() {updateRebotJointState(message)
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/dual_arm_selector_panel.cpp:346`

```cpp
create_subscription<sensor_msgs::msg::JointState>(
    "/piperh/joint_states", rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::JointState::SharedPtr message) {
      QMetaObject::invokeMethod(
        this, [this, message]() {updatePiperJointState(message)
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/dual_arm_selector_panel.cpp:352`

```cpp
create_subscription<rebotarm_msgs::msg::ArmStatus>(
    "/piperh/arm_status", qos,
    [this](const rebotarm_msgs::msg::ArmStatus::SharedPtr msg) {
      const QString state = QString::fromStdString(msg->state_machine)
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/dual_arm_selector_panel.cpp:369`

```cpp
create_subscription<std_msgs::msg::String>(
    "/piperh/teach/status", qos,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      const auto document = QJsonDocument::fromJson(QByteArray::fromStdString(msg->data))
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/dual_arm_selector_panel.cpp:383`

```cpp
create_client<std_srvs::srv::SetBool>(
    "/piperh/motor/set_enabled")
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/dual_arm_selector_panel.cpp:385`

```cpp
create_client<std_srvs::srv::SetBool>(
    "/piperh/xbox/set_armed")
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/dual_arm_selector_panel.cpp:387`

```cpp
create_client<std_srvs::srv::Trigger>(
    "/piperh/teach/gravity_mode/start")
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/dual_arm_selector_panel.cpp:389`

```cpp
create_client<std_srvs::srv::Trigger>(
    "/piperh/teach/gravity_mode/stop")
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/dual_arm_selector_panel.cpp:391`

```cpp
create_client<std_srvs::srv::Trigger>(
    "/piperh/teach/cancel")
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/dual_arm_selector_panel.cpp:393`

```cpp
create_client<std_srvs::srv::Trigger>(
    "/piperh/forbidden_zone_manager/reload")
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/dual_arm_selector_panel.cpp:395`

```cpp
create_client<action_msgs::srv::CancelGoal>(
    "/piperh/move_action/_action/cancel_goal")
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/dual_arm_selector_panel.cpp:397`

```cpp
create_client<action_msgs::srv::CancelGoal>(
    "/piperh/execute_trajectory/_action/cancel_goal")
```

- `rebotarm_demo_rviz` / `dual_arm_ws/src/rebotarm_demo_rviz/src/dual_arm_selector_panel.cpp:399`

```cpp
create_client<action_msgs::srv::CancelGoal>(
    "/piperh/arm_controller/follow_joint_trajectory/_action/cancel_goal")
```

- `antbot_imu` / `src/antbot_imu/src/imu_node.cpp:92`

```cpp
create_wall_timer(
    std::chrono::milliseconds(static_cast<uint32_t>(1000.0 / publish_rate_)),
    std::bind(&ImuNode::publish_timer_callback, this))
```

- `antbot_imu` / `src/antbot_imu/src/imu_node.cpp:134`

```cpp
create_publisher<sensor_msgs::msg::Imu>(
    "imu/accel_gyro", constants::SensorDataQoS())
```

- `antbot_imu` / `src/antbot_imu/src/imu_node.cpp:137`

```cpp
create_subscription<nav_msgs::msg::Odometry>(
    "odom",
    constants::SensorDataQoS(),
    [this](nav_msgs::msg::Odometry::SharedPtr msg)
    {
      if (!odom_received_.load()) {
        odom_received_.store(true)
```

- `antbot_camera` / `src/antbot_camera/src/camera_node.cpp:47`

```cpp
create_wall_timer(period, std::bind(&CameraNode::timer_callback, this))
```

- `antbot_camera` / `src/antbot_camera/src/camera_node.cpp:254`

```cpp
create_publisher<sensor_msgs::msg::Image>(
          prefix + "image_raw", qos)
```

- `antbot_camera` / `src/antbot_camera/src/camera_node.cpp:256`

```cpp
create_publisher<sensor_msgs::msg::CameraInfo>(
          prefix + "camera_info", qos)
```

- `antbot_camera` / `src/antbot_camera/src/camera_test_node.cpp:113`

```cpp
create_subscription<sensor_msgs::msg::CameraInfo>(
      topic_prefix + "camera_info", qos,
      [this, key](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        on_camera_info(key, msg)
```

- `antbot_camera` / `src/antbot_camera/src/camera_test_node.cpp:119`

```cpp
create_subscription<sensor_msgs::msg::Image>(
      topic_prefix + "image_raw", qos,
      [this, key](const sensor_msgs::msg::Image::SharedPtr msg) {
        on_image(key, msg)
```

- `robotcar_navigation` / `src/robotcar_navigation/src/wp_nav_remote.cpp:26`

```cpp
create_client<robotcar_navigation::srv::GetNumOfWaypoints>(
    "waterplus/get_num_waypoint")
```

- `robotcar_navigation` / `src/robotcar_navigation/src/wp_nav_remote.cpp:28`

```cpp
create_client<robotcar_navigation::srv::GetWaypointByIndex>(
    "waterplus/get_waypoint_index")
```

- `robotcar_navigation` / `src/robotcar_navigation/src/wp_nav_remote.cpp:30`

```cpp
create_client<robotcar_navigation::srv::GetWaypointByName>(
    "waterplus/get_waypoint_name")
```

- `robotcar_navigation` / `src/robotcar_navigation/src/wp_nav_remote.cpp:32`

```cpp
create_publisher<std_msgs::msg::String>("wpr1/behaviors", 30)
```

- `robotcar_navigation` / `src/robotcar_navigation/src/wp_nav_remote.cpp:33`

```cpp
create_subscription<std_msgs::msg::String>(
    "wpr1/grab_result", 30,
    [node](const std_msgs::msg::String::SharedPtr msg) {
      RCLCPP_WARN(node->get_logger(), "[GrabResultCB] %s", msg->data.c_str())
```

- `robotcar_navigation` / `src/robotcar_navigation/src/wp_nav_remote.cpp:38`

```cpp
create_subscription<std_msgs::msg::String>(
    "wpr1/pass_result", 30,
    [node](const std_msgs::msg::String::SharedPtr msg) {
      RCLCPP_WARN(node->get_logger(), "[PassResultCB] %s", msg->data.c_str())
```

- `robotcar_navigation` / `src/robotcar_navigation/src/wp_nav_remote.cpp:44`

```cpp
create_client<nav2_msgs::action::NavigateToPose>(node, "navigate_to_pose")
```

- `robotcar_navigation` / `src/robotcar_navigation/src/wp_saver.cpp:67`

```cpp
create_client<robotcar_navigation::srv::SaveWaypoints>(
    "waterplus/save_waypoints")
```

- `robotcar_navigation` / `src/robotcar_navigation/src/wp_saver.cpp:69`

```cpp
create_client<std_srvs::srv::Empty>(
    "waterplus/reload_waypoints")
```

- `robotcar_navigation` / `src/robotcar_navigation/src/add_charger_tool.cpp:48`

```cpp
create_publisher<robotcar_navigation::msg::Waypoint>(
    topic_property_->getStdString(), rclcpp::QoS(1))
```

- `robotcar_navigation` / `src/robotcar_navigation/src/wp_set_pose.cpp:15`

```cpp
create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", rclcpp::QoS(1).transient_local())
```

- `robotcar_navigation` / `src/robotcar_navigation/src/wp_set_pose.cpp:17`

```cpp
create_client<robotcar_navigation::srv::GetWaypointByName>(
      "/waterplus/get_waypoint_name")
```

- `robotcar_navigation` / `src/robotcar_navigation/src/wp_set_pose.cpp:19`

```cpp
create_subscription<std_msgs::msg::String>(
      "/waterplus/set_pose", 1,
      [this](const std_msgs::msg::String::SharedPtr msg) { set_pose_from_waypoint(msg->data)
```

- `robotcar_navigation` / `src/robotcar_navigation/src/wp_nav_test.cpp:16`

```cpp
create_client<robotcar_navigation::srv::GetNumOfWaypoints>(
    "waterplus/get_num_waypoint")
```

- `robotcar_navigation` / `src/robotcar_navigation/src/wp_nav_test.cpp:18`

```cpp
create_client<robotcar_navigation::srv::GetWaypointByIndex>(
    "waterplus/get_waypoint_index")
```

- `robotcar_navigation` / `src/robotcar_navigation/src/wp_nav_test.cpp:21`

```cpp
create_client<nav2_msgs::action::NavigateToPose>(node, "navigate_to_pose")
```

- `robotcar_navigation` / `src/robotcar_navigation/src/wp_edit_node.cpp:49`

```cpp
create_publisher<visualization_msgs::msg::Marker>("text_marker", text_qos)
```

- `robotcar_navigation` / `src/robotcar_navigation/src/wp_edit_node.cpp:50`

```cpp
create_subscription<msg::Waypoint>(
      "waterplus/add_waypoint", 10,
      [this](const msg::Waypoint::SharedPtr msg) { add_waypoint(*msg)
```

- `robotcar_navigation` / `src/robotcar_navigation/src/wp_edit_node.cpp:53`

```cpp
create_subscription<msg::Waypoint>(
      "waterplus/add_charger", 10,
      [this](const msg::Waypoint::SharedPtr msg) { add_charger(*msg)
```

- `robotcar_navigation` / `src/robotcar_navigation/src/wp_edit_node.cpp:56`

```cpp
create_subscription<std_msgs::msg::String>(
      "/waypoint_reached", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) { waypoint_reached(msg->data)
```

- `robotcar_navigation` / `src/robotcar_navigation/src/wp_edit_node.cpp:82`

```cpp
create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() { on_timer()
```

- `robotcar_navigation` / `src/robotcar_navigation/src/wp_edit_node.cpp:90`

```cpp
create_service<srv::GetNumOfWaypoints>(
      "waterplus/get_num_waypoint",
      [this](const std::shared_ptr<srv::GetNumOfWaypoints::Request> request,
      std::shared_ptr<srv::GetNumOfWaypoints::Response> response) {
        (void)request
```

- `robotcar_navigation` / `src/robotcar_navigation/src/wp_edit_node.cpp:97`

```cpp
create_service<srv::GetWaypointByIndex>(
      "waterplus/get_waypoint_index",
      [this](const std::shared_ptr<srv::GetWaypointByIndex::Request> request,
      std::shared_ptr<srv::GetWaypointByIndex::Response> response) {
        fill_by_index(waypoints_, request->index, response->name, response->pose)
```

- `robotcar_navigation` / `src/robotcar_navigation/src/wp_edit_node.cpp:103`

```cpp
create_service<srv::GetWaypointByName>(
      "waterplus/get_waypoint_name",
      [this](const std::shared_ptr<srv::GetWaypointByName::Request> request,
      std::shared_ptr<srv::GetWaypointByName::Response> response) {
        fill_by_name(waypoints_, request->name, response->name, response->pose)
```

- `robotcar_navigation` / `src/robotcar_navigation/src/wp_edit_node.cpp:109`

```cpp
create_service<srv::SaveWaypoints>(
      "waterplus/save_waypoints",
      [this](const std::shared_ptr<srv::SaveWaypoints::Request> request,
      std::shared_ptr<srv::SaveWaypoints::Response> response) {
        (void)response
```

- `robotcar_navigation` / `src/robotcar_navigation/src/wp_edit_node.cpp:116`

```cpp
create_service<std_srvs::srv::Empty>(
      "waterplus/reload_waypoints",
      [this](const std::shared_ptr<std_srvs::srv::Empty::Request> request,
      std::shared_ptr<std_srvs::srv::Empty::Response> response) {
        (void)request
```

- `robotcar_navigation` / `src/robotcar_navigation/src/wp_edit_node.cpp:124`

```cpp
create_service<srv::GetWaypointNames>(
      "waterplus/get_waypoint_names",
      [this](const std::shared_ptr<srv::GetWaypointNames::Request> request,
      std::shared_ptr<srv::GetWaypointNames::Response> response) {
        (void)request
```

- `robotcar_navigation` / `src/robotcar_navigation/src/wp_edit_node.cpp:134`

```cpp
create_service<srv::RenameWaypoint>(
      "waterplus/rename_waypoint",
      [this](const std::shared_ptr<srv::RenameWaypoint::Request> request,
      std::shared_ptr<srv::RenameWaypoint::Response> response) {
        rename_waypoint(request->old_name, request->new_name, *response)
```

- `robotcar_navigation` / `src/robotcar_navigation/src/wp_edit_node.cpp:140`

```cpp
create_service<srv::WaypointFile>(
      "waterplus/save_waypoint_group",
      [this](const std::shared_ptr<srv::WaypointFile::Request> request,
      std::shared_ptr<srv::WaypointFile::Response> response) {
        if (request->filename.empty()) {
          response->success = false
```

- `robotcar_navigation` / `src/robotcar_navigation/src/wp_edit_node.cpp:159`

```cpp
create_service<std_srvs::srv::Trigger>(
      "waterplus/save_active_waypoint_group",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        (void)request
```

- `robotcar_navigation` / `src/robotcar_navigation/src/wp_edit_node.cpp:169`

```cpp
create_service<srv::WaypointFile>(
      "waterplus/load_waypoint_group",
      [this](const std::shared_ptr<srv::WaypointFile::Request> request,
      std::shared_ptr<srv::WaypointFile::Response> response) {
        std::string error
```

- `robotcar_navigation` / `src/robotcar_navigation/src/wp_edit_node.cpp:179`

```cpp
create_service<srv::GetNumOfWaypoints>(
      "waterplus/get_num_charger",
      [this](const std::shared_ptr<srv::GetNumOfWaypoints::Request> request,
      std::shared_ptr<srv::GetNumOfWaypoints::Response> response) {
        (void)request
```

- `robotcar_navigation` / `src/robotcar_navigation/src/wp_edit_node.cpp:186`

```cpp
create_service<srv::GetWaypointByIndex>(
      "waterplus/get_charger_index",
      [this](const std::shared_ptr<srv::GetWaypointByIndex::Request> request,
      std::shared_ptr<srv::GetWaypointByIndex::Response> response) {
        fill_by_index(chargers_, request->index, response->name, response->pose)
```

- `robotcar_navigation` / `src/robotcar_navigation/src/wp_edit_node.cpp:192`

```cpp
create_service<srv::GetChargerByName>(
      "waterplus/get_charger_name",
      [this](const std::shared_ptr<srv::GetChargerByName::Request> request,
      std::shared_ptr<srv::GetChargerByName::Response> response) {
        fill_by_name(chargers_, request->name, response->name, response->pose)
```

- `robotcar_navigation` / `src/robotcar_navigation/src/wp_edit_node.cpp:198`

```cpp
create_service<srv::GetWaypointNames>(
      "waterplus/get_charger_names",
      [this](const std::shared_ptr<srv::GetWaypointNames::Request> request,
      std::shared_ptr<srv::GetWaypointNames::Response> response) {
        (void)request
```

- `robotcar_navigation` / `src/robotcar_navigation/src/demo_map_tool.cpp:14`

```cpp
create_publisher<std_msgs::msg::String>("/waterplus/navi_waypoint", 10)
```

- `robotcar_navigation` / `src/robotcar_navigation/src/demo_map_tool.cpp:15`

```cpp
create_subscription<std_msgs::msg::String>(
      "/waterplus/navi_result", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_WARN(get_logger(), "[NavResultCallback] received result: %s", msg->data.c_str())
```

- `robotcar_navigation` / `src/robotcar_navigation/src/demo_map_tool.cpp:20`

```cpp
create_wall_timer(1s, [this]() {
      if (sent_) {
        return
```

- `robotcar_navigation` / `src/robotcar_navigation/src/wp_navi_server.cpp:19`

```cpp
create_publisher<std_msgs::msg::String>("waterplus/navi_result", 10)
```

- `robotcar_navigation` / `src/robotcar_navigation/src/wp_navi_server.cpp:20`

```cpp
create_publisher<std_msgs::msg::String>("/waypoint_reached", 10)
```

- `robotcar_navigation` / `src/robotcar_navigation/src/wp_navi_server.cpp:21`

```cpp
create_publisher<std_msgs::msg::String>(
    "/waterplus/charge_result", 10)
```

- `robotcar_navigation` / `src/robotcar_navigation/src/wp_navi_server.cpp:23`

```cpp
create_publisher<std_msgs::msg::String>(
    "/antbot/dock_request", 10)
```

- `robotcar_navigation` / `src/robotcar_navigation/src/wp_navi_server.cpp:25`

```cpp
create_subscription<std_msgs::msg::String>(
    "waterplus/navi_waypoint", 10,
    [&waypoint_name, &new_command, &charger_command](
      const std_msgs::msg::String::SharedPtr msg) {
      waypoint_name = msg->data
```

- `robotcar_navigation` / `src/robotcar_navigation/src/wp_navi_server.cpp:33`

```cpp
create_subscription<std_msgs::msg::String>(
    "waterplus/navi_charger", 10,
    [&waypoint_name, &new_command, &charger_command](
      const std_msgs::msg::String::SharedPtr msg) {
      waypoint_name = msg->data
```

- `robotcar_navigation` / `src/robotcar_navigation/src/wp_navi_server.cpp:41`

```cpp
create_client<robotcar_navigation::srv::GetWaypointByName>(
    "waterplus/get_waypoint_name")
```

- `robotcar_navigation` / `src/robotcar_navigation/src/wp_navi_server.cpp:43`

```cpp
create_client<robotcar_navigation::srv::GetChargerByName>(
    "waterplus/get_charger_name")
```

- `robotcar_navigation` / `src/robotcar_navigation/src/wp_navi_server.cpp:46`

```cpp
create_client<nav2_msgs::action::NavigateToPose>(node, "navigate_to_pose")
```

- `robotcar_navigation` / `src/robotcar_navigation/src/add_keepout_zone_tool.cpp:245`

```cpp
create_publisher<msg::KeepoutZone>(
      topic_property_->getStdString(), rclcpp::QoS(10))
```

- `robotcar_navigation` / `src/robotcar_navigation/src/charger_get_position.cpp:13`

```cpp
create_client<robotcar_navigation::srv::GetChargerByName>(
    "waterplus/get_charger_name")
```

- `robotcar_navigation` / `src/robotcar_navigation/src/waypoint_manager_panel.cpp:357`

```cpp
create_client<srv::GetWaypointNames>(
    "/waterplus/get_waypoint_names")
```

- `robotcar_navigation` / `src/robotcar_navigation/src/waypoint_manager_panel.cpp:359`

```cpp
create_client<srv::GetWaypointNames>(
    "/waterplus/get_charger_names")
```

- `robotcar_navigation` / `src/robotcar_navigation/src/waypoint_manager_panel.cpp:361`

```cpp
create_client<srv::RenameWaypoint>(
    "/waterplus/rename_waypoint")
```

- `robotcar_navigation` / `src/robotcar_navigation/src/waypoint_manager_panel.cpp:363`

```cpp
create_client<srv::WaypointFile>(
    "/waterplus/save_waypoint_group")
```

- `robotcar_navigation` / `src/robotcar_navigation/src/waypoint_manager_panel.cpp:365`

```cpp
create_client<srv::WaypointFile>(
    "/waterplus/load_waypoint_group")
```

- `robotcar_navigation` / `src/robotcar_navigation/src/waypoint_manager_panel.cpp:367`

```cpp
create_client<nav2_msgs::srv::LoadMap>(
    "/map_server/load_map")
```

- `robotcar_navigation` / `src/robotcar_navigation/src/waypoint_manager_panel.cpp:369`

```cpp
create_client<action_msgs::srv::CancelGoal>(
    "/navigate_to_pose/_action/cancel_goal")
```

- `robotcar_navigation` / `src/robotcar_navigation/src/waypoint_manager_panel.cpp:371`

```cpp
create_client<srv::GetWaypointNames>(
    "/waterplus/get_keepout_names")
```

- `robotcar_navigation` / `src/robotcar_navigation/src/waypoint_manager_panel.cpp:373`

```cpp
create_client<srv::RenameWaypoint>(
    "/waterplus/rename_keepout")
```

- `robotcar_navigation` / `src/robotcar_navigation/src/waypoint_manager_panel.cpp:375`

```cpp
create_client<srv::WaypointFile>(
    "/waterplus/save_keepout_group")
```

- `robotcar_navigation` / `src/robotcar_navigation/src/waypoint_manager_panel.cpp:377`

```cpp
create_client<srv::WaypointFile>(
    "/waterplus/load_keepout_group")
```

- `robotcar_navigation` / `src/robotcar_navigation/src/waypoint_manager_panel.cpp:379`

```cpp
create_client<std_srvs::srv::SetBool>(
    "/waterplus/enable_keepouts")
```

- `robotcar_navigation` / `src/robotcar_navigation/src/waypoint_manager_panel.cpp:381`

```cpp
create_client<srv::GetWaypointNames>(
    "/waterplus/get_speed_zone_names")
```

- `robotcar_navigation` / `src/robotcar_navigation/src/waypoint_manager_panel.cpp:383`

```cpp
create_client<srv::RenameWaypoint>(
    "/waterplus/rename_speed_zone")
```

- `robotcar_navigation` / `src/robotcar_navigation/src/waypoint_manager_panel.cpp:385`

```cpp
create_client<srv::WaypointFile>(
    "/waterplus/save_speed_zone_group")
```

- `robotcar_navigation` / `src/robotcar_navigation/src/waypoint_manager_panel.cpp:387`

```cpp
create_client<srv::WaypointFile>(
    "/waterplus/load_speed_zone_group")
```

- `robotcar_navigation` / `src/robotcar_navigation/src/waypoint_manager_panel.cpp:389`

```cpp
create_client<std_srvs::srv::SetBool>(
    "/waterplus/enable_speed_zones")
```

- `robotcar_navigation` / `src/robotcar_navigation/src/waypoint_manager_panel.cpp:391`

```cpp
create_publisher<std_msgs::msg::String>(
    "/waterplus/navi_waypoint", 10)
```

- `robotcar_navigation` / `src/robotcar_navigation/src/waypoint_manager_panel.cpp:393`

```cpp
create_publisher<std_msgs::msg::String>(
    "/waterplus/navi_charger", 10)
```

- `robotcar_navigation` / `src/robotcar_navigation/src/waypoint_manager_panel.cpp:395`

```cpp
create_publisher<std_msgs::msg::String>(
    "/waterplus/delete_keepout", 10)
```

- `robotcar_navigation` / `src/robotcar_navigation/src/waypoint_manager_panel.cpp:397`

```cpp
create_publisher<std_msgs::msg::String>(
    "/waterplus/delete_speed_zone", 10)
```

- `robotcar_navigation` / `src/robotcar_navigation/src/waypoint_manager_panel.cpp:399`

```cpp
create_publisher<std_msgs::msg::String>(
    "/waterplus/route_control", 10)
```

- `robotcar_navigation` / `src/robotcar_navigation/src/waypoint_manager_panel.cpp:401`

```cpp
create_subscription<std_msgs::msg::String>(
    "/waterplus/navi_result", 10,
    [this](const std_msgs::msg::String::SharedPtr message) {
      handleNavigationResult(message)
```

- `robotcar_navigation` / `src/robotcar_navigation/src/waypoint_manager_panel.cpp:406`

```cpp
create_subscription<std_msgs::msg::String>(
    "/waterplus/route_progress", 10,
    [this](const std_msgs::msg::String::SharedPtr message) {
      handleExternalRouteProgress(message)
```

- `robotcar_navigation` / `src/robotcar_navigation/src/waypoint_manager_panel.cpp:411`

```cpp
create_subscription<std_msgs::msg::String>(
    "/waterplus/charge_result", 10,
    [this](const std_msgs::msg::String::SharedPtr message) {
      handleChargeResult(message)
```

- `robotcar_navigation` / `src/robotcar_navigation/src/waypoint_manager_panel.cpp:416`

```cpp
create_subscription<std_msgs::msg::String>(
    "/waterplus/keepout_status", 10,
    [this](const std_msgs::msg::String::SharedPtr message) {
      handleKeepoutStatus(message)
```

- `robotcar_navigation` / `src/robotcar_navigation/src/waypoint_manager_panel.cpp:421`

```cpp
create_subscription<std_msgs::msg::String>(
    "/waterplus/speed_zone_status", 10,
    [this](const std_msgs::msg::String::SharedPtr message) {
      handleSpeedZoneStatus(message)
```

- `robotcar_navigation` / `src/robotcar_navigation/src/pose_navi_server.cpp:15`

```cpp
create_publisher<std_msgs::msg::String>("waterplus/navi_result", 10)
```

- `robotcar_navigation` / `src/robotcar_navigation/src/pose_navi_server.cpp:16`

```cpp
create_subscription<geometry_msgs::msg::Pose>(
    "waterplus/navi_pose", 10,
    [&goal_pose, &new_command](const geometry_msgs::msg::Pose::SharedPtr msg) {
      goal_pose = *msg
```

- `robotcar_navigation` / `src/robotcar_navigation/src/pose_navi_server.cpp:23`

```cpp
create_client<nav2_msgs::action::NavigateToPose>(node, "navigate_to_pose")
```

- `robotcar_navigation` / `src/robotcar_navigation/src/set_pose_from_waypoint.cpp:24`

```cpp
create_client<robotcar_navigation::srv::GetWaypointByName>(
    "/waterplus/get_waypoint_name")
```

- `robotcar_navigation` / `src/robotcar_navigation/src/set_pose_from_waypoint.cpp:38`

```cpp
create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose", rclcpp::QoS(1).transient_local())
```

- `robotcar_navigation` / `src/robotcar_navigation/src/vehicle_status_panel.cpp:570`

```cpp
create_client<std_srvs::srv::SetBool>(
    "/antbot/operator_enable")
```

- `robotcar_navigation` / `src/robotcar_navigation/src/vehicle_status_panel.cpp:572`

```cpp
create_client<std_srvs::srv::Trigger>(
    "/antbot/system_reset")
```

- `robotcar_navigation` / `src/robotcar_navigation/src/vehicle_status_panel.cpp:574`

```cpp
create_client<std_srvs::srv::SetBool>(
    "/antbot/teleop/use_xbox")
```

- `robotcar_navigation` / `src/robotcar_navigation/src/vehicle_status_panel.cpp:576`

```cpp
create_client<std_srvs::srv::SetBool>(
    "/antbot/mapping/set_enabled")
```

- `robotcar_navigation` / `src/robotcar_navigation/src/vehicle_status_panel.cpp:578`

```cpp
create_client<std_srvs::srv::Trigger>(
    "/antbot/mapping/save")
```

- `robotcar_navigation` / `src/robotcar_navigation/src/vehicle_status_panel.cpp:580`

```cpp
create_publisher<geometry_msgs::msg::Twist>(
    "/antbot/cmd_vel/keyboard", 10)
```

- `robotcar_navigation` / `src/robotcar_navigation/src/vehicle_status_panel.cpp:582`

```cpp
create_subscription<nav_msgs::msg::Odometry>(
    "/odometry/filtered", rclcpp::SensorDataQoS(),
    std::bind(&VehicleStatusPanel::handleOdometry, this, std::placeholders::_1))
```

- `robotcar_navigation` / `src/robotcar_navigation/src/vehicle_status_panel.cpp:587`

```cpp
create_subscription<sensor_msgs::msg::BatteryState>(
    "/battery", rclcpp::SensorDataQoS(),
    std::bind(&VehicleStatusPanel::handleBattery, this, std::placeholders::_1))
```

- `robotcar_navigation` / `src/robotcar_navigation/src/vehicle_status_panel.cpp:590`

```cpp
create_subscription<std_msgs::msg::String>(
    "/antbot/robot_intent", rclcpp::QoS(1).transient_local().reliable(),
    std::bind(&VehicleStatusPanel::handleIntent, this, std::placeholders::_1))
```

- `robotcar_navigation` / `src/robotcar_navigation/src/vehicle_status_panel.cpp:593`

```cpp
create_subscription<std_msgs::msg::String>(
    "/antbot/vehicle_status", 10,
    std::bind(&VehicleStatusPanel::handleExtendedStatus, this, std::placeholders::_1))
```

- `robotcar_navigation` / `src/robotcar_navigation/src/vehicle_status_panel.cpp:596`

```cpp
create_subscription<std_msgs::msg::String>(
    "/antbot/operator_ui_status", rclcpp::QoS(1).transient_local().reliable(),
    std::bind(&VehicleStatusPanel::handleOperatorUiStatus, this, std::placeholders::_1))
```

- `robotcar_navigation` / `src/robotcar_navigation/src/vehicle_status_panel.cpp:639`

```cpp
create_subscription<sensor_msgs::msg::Image>(
    topic.toStdString(), rclcpp::SensorDataQoS(),
    std::bind(&VehicleStatusPanel::handleImage, this, std::placeholders::_1))
```

- `robotcar_navigation` / `src/robotcar_navigation/src/floor_texture_publisher.cpp:34`

```cpp
create_publisher<visualization_msgs::msg::Marker>("~/floor_texture", marker_qos)
```

- `robotcar_navigation` / `src/robotcar_navigation/src/add_speed_zone_tool.cpp:251`

```cpp
create_publisher<msg::SpeedZone>(
      topic_property_->getStdString(), rclcpp::QoS(10))
```

- `robotcar_navigation` / `src/robotcar_navigation/src/add_waypoint_tool.cpp:48`

```cpp
create_publisher<robotcar_navigation::msg::Waypoint>(
    topic_property_->getStdString(), rclcpp::QoS(1))
```

- `robotcar_navigation` / `src/robotcar_navigation/include/robotcar_navigation/waypoint_manager_core.hpp:42`

```cpp
create_publisher<visualization_msgs::msg::Marker>("waypoints_marker", 100)
```

- `robotcar_navigation` / `src/robotcar_navigation/include/robotcar_navigation/waypoint_manager_core.hpp:43`

```cpp
create_publisher<visualization_msgs::msg::Marker>("chargers_marker", 100)
```

- `robotcar_navigation` / `src/robotcar_navigation/include/robotcar_navigation/waypoint_manager_core.hpp:45`

```cpp
create_subscription<msg::Waypoint>(
      "waterplus/add_waypoint", 10,
      [this](const msg::Waypoint::SharedPtr msg) { add_waypoint(*msg)
```

- `robotcar_navigation` / `src/robotcar_navigation/include/robotcar_navigation/waypoint_manager_core.hpp:48`

```cpp
create_subscription<msg::Waypoint>(
      "waterplus/add_charger", 10,
      [this](const msg::Waypoint::SharedPtr msg) { add_charger(*msg)
```

- `robotcar_navigation` / `src/robotcar_navigation/include/robotcar_navigation/waypoint_manager_core.hpp:52`

```cpp
create_service<srv::GetNumOfWaypoints>(
      "waterplus/get_num_waypoint",
      [this](const std::shared_ptr<srv::GetNumOfWaypoints::Request> request,
      std::shared_ptr<srv::GetNumOfWaypoints::Response> response) {
        (void)request
```

- `robotcar_navigation` / `src/robotcar_navigation/include/robotcar_navigation/waypoint_manager_core.hpp:59`

```cpp
create_service<srv::GetWaypointByIndex>(
      "waterplus/get_waypoint_index",
      [this](const std::shared_ptr<srv::GetWaypointByIndex::Request> request,
      std::shared_ptr<srv::GetWaypointByIndex::Response> response) {
        fill_by_index(waypoints_, request->index, response->name, response->pose)
```

- `robotcar_navigation` / `src/robotcar_navigation/include/robotcar_navigation/waypoint_manager_core.hpp:65`

```cpp
create_service<srv::GetWaypointByName>(
      "waterplus/get_waypoint_name",
      [this](const std::shared_ptr<srv::GetWaypointByName::Request> request,
      std::shared_ptr<srv::GetWaypointByName::Response> response) {
        fill_by_name(waypoints_, request->name, response->name, response->pose)
```

- `robotcar_navigation` / `src/robotcar_navigation/include/robotcar_navigation/waypoint_manager_core.hpp:71`

```cpp
create_service<srv::SaveWaypoints>(
      "waterplus/save_waypoints",
      [this](const std::shared_ptr<srv::SaveWaypoints::Request> request,
      std::shared_ptr<srv::SaveWaypoints::Response> response) {
        (void)response
```

- `robotcar_navigation` / `src/robotcar_navigation/include/robotcar_navigation/waypoint_manager_core.hpp:79`

```cpp
create_service<srv::GetNumOfWaypoints>(
      "waterplus/get_num_charger",
      [this](const std::shared_ptr<srv::GetNumOfWaypoints::Request> request,
      std::shared_ptr<srv::GetNumOfWaypoints::Response> response) {
        (void)request
```

- `robotcar_navigation` / `src/robotcar_navigation/include/robotcar_navigation/waypoint_manager_core.hpp:86`

```cpp
create_service<srv::GetWaypointByIndex>(
      "waterplus/get_charger_index",
      [this](const std::shared_ptr<srv::GetWaypointByIndex::Request> request,
      std::shared_ptr<srv::GetWaypointByIndex::Response> response) {
        fill_by_index(chargers_, request->index, response->name, response->pose)
```

- `robotcar_navigation` / `src/robotcar_navigation/include/robotcar_navigation/waypoint_manager_core.hpp:92`

```cpp
create_service<srv::GetChargerByName>(
      "waterplus/get_charger_name",
      [this](const std::shared_ptr<srv::GetChargerByName::Request> request,
      std::shared_ptr<srv::GetChargerByName::Response> response) {
        fill_by_name(chargers_, request->name, response->name, response->pose)
```

- `robotcar_navigation` / `src/robotcar_navigation/include/robotcar_navigation/waypoint_manager_core.hpp:99`

```cpp
create_wall_timer(
      std::chrono::milliseconds(500),
      [this]() { publish_markers()
```

- `vanjee_lidar_sdk` / `src/vanjee_lidar_sdk/src/source/source_imu_packet_ros.hpp:94`

```cpp
advertise<sensor_msgs::Imu>(ros_send_topic, 10)
```

- `vanjee_lidar_sdk` / `src/vanjee_lidar_sdk/src/source/source_pointcloud_ros.hpp:236`

```cpp
advertise<sensor_msgs::PointCloud2>(ros_send_topic, 10)
```

- `vanjee_lidar_sdk` / `src/vanjee_lidar_sdk/src/source/source_scandata_ros.hpp:91`

```cpp
advertise<sensor_msgs::LaserScan>(ros_send_topic, 10)
```

- `vanjee_lidar_sdk` / `src/vanjee_lidar_sdk/src/source/source_recv_device_ctrl_ros.hpp:58`

```cpp
subscribe device control state through ROS topic
/// '/vanjee_device_ctrl_state'
class SourceDeviceCtrlRos : public SourceDeviceCtrl {
 private:
  std::shared_ptr<ros::NodeHandle> nh_
```

- `vanjee_lidar_sdk` / `src/vanjee_lidar_sdk/src/source/source_recv_device_ctrl_ros.hpp:81`

```cpp
subscribe<vanjee_lidar_sdk::DeviceCtrl>(ros_recv_topic, 10, &SourceDeviceCtrlRos::recvDeviceCtrl, this)
```

- `vanjee_lidar_sdk` / `src/vanjee_lidar_sdk/src/source/source_recv_device_ctrl_ros.hpp:108`

```cpp
subscribe device ctrl state through ROS topic
/// '/vanjee_device_ctrl_state'
class SourceDeviceCtrlRos : virtual public SourceDeviceCtrl {
 private:
  using DeviceCtrlMsgSubPtr = std::shared_ptr<VanjeeLidarSdkSubscribeRosMsg<vanjee_lidar_msg::msg::DeviceCtrl, vanjee::lidar::DeviceCtrl>>
```

- `vanjee_lidar_sdk` / `src/vanjee_lidar_sdk/src/source/source_recv_device_ctrl_ros.hpp:121`

```cpp
subscribe device ctrl state through ROS topic '/vanjee_device_ctrl_state'
  void recvDeviceCtrl(const vanjee_lidar_msg::msg::DeviceCtrl::SharedPtr msg)
```

- `vanjee_lidar_sdk` / `src/vanjee_lidar_sdk/src/source/source_ros_msg_delegate.hpp:17`

```cpp
create_publisher<T_RosMsg>(topic_name, 10)
```

- `vanjee_lidar_sdk` / `src/vanjee_lidar_sdk/src/source/source_ros_msg_delegate.hpp:35`

```cpp
create_subscription<T_RosMsg>(topic_name_, 10, subscribe_topic_callback)
```

- `vanjee_lidar_sdk` / `src/vanjee_lidar_sdk/src/source/source_device_ctrl_ros.hpp:81`

```cpp
advertise<vanjee_lidar_sdk::DeviceCtrl>(ros_send_topic, 10)
```

## Launch Node 实例、参数覆盖、命名空间与 remap

源码声明实例，不等于实际启动数；IfCondition、OpaqueFunction 和嵌套 Include 必须联合判定。

- `dual_arm_ws/src/red_point_localizer/launch/red_point_detector.launch.py:48`

```yaml
package: '''red_point_localizer'''
executable: '''red_point_detector'''
name: '''red_point_detector'''
output: '''screen'''
parameters: '[parameters]'
```

- `dual_arm_ws/src/red_point_localizer/launch/multi_color_demo.launch.py:44`

```yaml
package: '''red_point_localizer'''
executable: '''red_point_detector'''
name: f'{color}_target_detector'
output: '''screen'''
parameters: '[parameters]'
```

- `dual_arm_ws/src/rebotarm_pulse/launch/handeye_calibrate.launch.py:121`

```yaml
package: '''image_proc'''
executable: '''rectify_node'''
name: '''handeye_rectify'''
namespace: '''rebotarm_handeye'''
output: '''screen'''
parameters: '[{''queue_size'': 1, ''qos_overrides./rebotarm_handeye/image_rect.publisher.reliability'':
  ''best_effort'', ''qos_overrides./rebotarm_handeye/image_rect.publisher.history'':
  ''keep_last'', ''qos_overrides./rebotarm_handeye/image_rect.publisher.depth'': 1}]'
remappings: '[(''image'', value(''color_topic'')), (''camera_info'', camera_info_topic),
  (''image_rect'', ''/rebotarm_handeye/image_rect'')]'
```

- `dual_arm_ws/src/rebotarm_pulse/launch/handeye_calibrate.launch.py:146`

```yaml
package: '''rebotarm_pulse'''
executable: '''charuco_board_pose'''
name: '''charuco_board_pose'''
namespace: '''rebotarm_handeye'''
output: '''screen'''
parameters: '[{''squares_x'': 5, ''squares_y'': 7, ''square_length_m'': 0.035, ''marker_length_m'':
  0.026, ''minimum_charuco_corners'': int(value(''charuco_minimum_corners'')), ''marker_frame'':
  tracking_marker}]'
remappings: '[(''image'', ''/rebotarm_handeye/image_rect''), (''camera_info'', camera_info_topic)]'
```

- `dual_arm_ws/src/rebotarm_pulse/launch/handeye_calibrate.launch.py:170`

```yaml
package: '''apriltag_ros'''
executable: '''apriltag_node'''
name: '''handeye_apriltag'''
namespace: '''rebotarm_handeye'''
output: '''screen'''
parameters: '[config, {''family'': tag_family, ''tag.ids'': detected_ids, ''tag.frames'':
  detected_frames, ''tag.sizes'': detected_sizes}]'
remappings: '[(''image_rect'', ''/rebotarm_handeye/image_rect''), (''camera_info'',
  camera_info_topic)]'
```

- `dual_arm_ws/src/rebotarm_pulse/launch/handeye_calibrate.launch.py:193`

```yaml
package: '''rebotarm_pulse'''
executable: '''apriltag_board_pose'''
name: '''apriltag_board_pose'''
namespace: '''rebotarm_handeye'''
output: '''screen'''
parameters: '[{''tag_ids'': detected_ids, ''tag_size_m'': tag_size, ''center_spacing_x_m'':
  float(value(''board_spacing_x_m'')), ''center_spacing_y_m'': float(value(''board_spacing_y_m'')),
  ''minimum_visible_tags'': int(value(''board_minimum_visible_tags'')), ''marker_frame'':
  tracking_marker}]'
remappings: '[(''detections'', ''/rebotarm_handeye/detections''), (''camera_info'',
  camera_info_topic)]'
```

- `dual_arm_ws/src/rebotarm_pulse/launch/handeye_calibrate.launch.py:233`

```yaml
package: '''rebotarm_pulse'''
executable: '''auto_handeye_sequence'''
name: '''auto_handeye_sequence'''
output: '''screen'''
condition: IfCondition(LaunchConfiguration('use_auto_sequence'))
parameters: '[{''sequence_name'': auto_sequence_name, ''action_name_prefix'': auto_action_prefix,
  ''calibration_type'': calibration_type, ''tracking_marker_frame'': tracking_marker,
  ''minimum_samples'': int(value(''auto_minimum_samples'')), ''teach_status_topic'':
  value(''teach_status_topic''), ''joint_state_topic'': value(''joint_state_topic''),
  ''teach_cancel_service'': value(''teach_cancel_service'')}]'
```

- `dual_arm_ws/src/rviz_visual_tools/launch/demo_combined.launch.py:52`

```yaml
package: '''rviz2'''
executable: '''rviz2'''
arguments: '[''--display-config'', rviz_config]'
output: '''screen'''
```

- `dual_arm_ws/src/rviz_visual_tools/launch/demo_combined.launch.py:58`

```yaml
package: '''tf2_ros'''
executable: '''static_transform_publisher'''
arguments: '[''0'', ''0'', ''0'', ''0'', ''0'', ''0'', ''/world'', ''/base'']'
output: '''screen'''
```

- `dual_arm_ws/src/rviz_visual_tools/launch/demo_combined.launch.py:64`

```yaml
package: '''rviz_visual_tools'''
executable: '''rviz_visual_tools_demo'''
output: '''screen'''
```

- `dual_arm_ws/src/rviz_visual_tools/launch/demo_combined.launch.py:69`

```yaml
package: '''rviz_visual_tools'''
executable: '''rviz_visual_tools_imarker_simple_demo'''
output: '''screen'''
```

- `dual_arm_ws/src/rviz_visual_tools/launch/demo_rviz.launch.py:43`

```yaml
package: '''rviz2'''
executable: '''rviz2'''
arguments: '[''--display-config'', rviz_config]'
output: '''screen'''
```

- `dual_arm_ws/src/rviz_visual_tools/launch/demo_rviz.launch.py:49`

```yaml
package: '''tf2_ros'''
executable: '''static_transform_publisher'''
arguments: '[''0'', ''0'', ''0'', ''0'', ''0'', ''0'', ''/world'', ''/base'']'
output: '''screen'''
```

- `dual_arm_ws/src/rebot_teach_mode/launch/teach_mode.launch.py:102`

```yaml
package: '''rebot_teach_mode'''
executable: '''teach_mode_node'''
namespace: namespace
name: '''rebot_teach_mode'''
output: '''screen'''
parameters: '[LaunchConfiguration(''config''), common_parameters]'
respawn: 'True'
respawn_delay: '2.0'
```

- `dual_arm_ws/src/rebot_teach_mode/launch/teach_mode.launch.py:112`

```yaml
package: '''rebot_teach_mode'''
executable: '''teach_xbox_bridge'''
namespace: namespace
name: '''rebot_teach_xbox_bridge'''
output: '''screen'''
condition: IfCondition(LaunchConfiguration('use_xbox'))
parameters: '[LaunchConfiguration(''config''), {''arm_namespace'': namespace, ''teach_status_topic'':
  f''/{namespace}/teach/status''}]'
```

- `dual_arm_ws/src/rebotarm_moveit_config/launch/hardware.launch.py:82`

```yaml
package: '''moveit_ros_move_group'''
executable: '''move_group'''
output: '''screen'''
parameters: '[moveit_params]'
remappings: '[(''/joint_states'', [''/'', arm_namespace, ''/joint_states''])]'
```

- `dual_arm_ws/src/rebotarm_moveit_config/launch/hardware.launch.py:97`

```yaml
package: '''rviz2'''
executable: '''rviz2'''
name: '''rviz2'''
output: '''screen'''
arguments: '[''-d'', rviz_config]'
condition: IfCondition(LaunchConfiguration('use_rviz'))
parameters: '[moveit_params]'
remappings: '[(''/joint_states'', [''/'', arm_namespace, ''/joint_states''])]'
respawn: 'True'
respawn_delay: '2.0'
```

- `dual_arm_ws/src/rebotarm_moveit_config/launch/hardware.launch.py:113`

```yaml
package: '''tf2_ros'''
executable: '''static_transform_publisher'''
name: '''static_transform_publisher'''
output: '''log'''
arguments: '[''0'', ''0'', ''0'', ''0'', ''0'', ''0'', ''world'', ''base_link'']'
```

- `dual_arm_ws/src/rebotarm_moveit_config/launch/hardware.launch.py:121`

```yaml
package: '''robot_state_publisher'''
executable: '''robot_state_publisher'''
name: '''robot_state_publisher'''
output: '''both'''
parameters: '[moveit_config.robot_description]'
remappings: '[(''/joint_states'', [''/'', arm_namespace, ''/joint_states''])]'
```

- `dual_arm_ws/src/rebotarm_moveit_config/launch/demo.launch.py:80`

```yaml
package: '''moveit_ros_move_group'''
executable: '''move_group'''
output: '''screen'''
parameters: '[moveit_params]'
```

- `dual_arm_ws/src/rebotarm_moveit_config/launch/demo.launch.py:94`

```yaml
package: '''rviz2'''
executable: '''rviz2'''
name: '''rviz2'''
output: '''screen'''
arguments: '[''-d'', rviz_config]'
condition: IfCondition(LaunchConfiguration('use_rviz'))
parameters: '[moveit_params]'
```

- `dual_arm_ws/src/rebotarm_moveit_config/launch/demo.launch.py:104`

```yaml
package: '''tf2_ros'''
executable: '''static_transform_publisher'''
name: '''static_transform_publisher'''
output: '''log'''
arguments: '[''0'', ''0'', ''0'', ''0'', ''0'', ''0'', ''world'', ''base_link'']'
```

- `dual_arm_ws/src/rebotarm_moveit_config/launch/demo.launch.py:112`

```yaml
package: '''robot_state_publisher'''
executable: '''robot_state_publisher'''
name: '''robot_state_publisher'''
output: '''both'''
parameters: '[moveit_config.robot_description]'
```

- `dual_arm_ws/src/rebotarm_moveit_config/launch/demo.launch.py:125`

```yaml
package: '''controller_manager'''
executable: '''ros2_control_node'''
parameters: '[moveit_config.robot_description, ros2_controllers_path]'
output: '''screen'''
```

- `dual_arm_ws/src/rebotarm_moveit_config/launch/demo.launch.py:132`

```yaml
package: '''controller_manager'''
executable: '''spawner'''
arguments: '[''joint_state_broadcaster'', ''--controller-manager'', ''/controller_manager'']'
```

- `dual_arm_ws/src/rebotarm_moveit_config/launch/demo.launch.py:142`

```yaml
package: '''controller_manager'''
executable: '''spawner'''
arguments: '[''rebotarm_controller'', ''--controller-manager'', ''/controller_manager'']'
```

- `dual_arm_ws/src/rebotarm_moveit_config/launch/demo.launch.py:152`

```yaml
package: '''controller_manager'''
executable: '''spawner'''
arguments: '[''gripper_controller'', ''--controller-manager'', ''/controller_manager'']'
```

- `dual_arm_ws/src/rebotarm_moveit_demos/launch/draw_square.launch.py:31`

```yaml
package: '''rebotarm_moveit_demos'''
executable: '''draw_square'''
name: '''draw_square'''
output: '''screen'''
parameters: '[config_file]'
```

- `dual_arm_ws/src/rebotarm_moveit_demos/launch/pick_place.launch.py:31`

```yaml
package: '''rebotarm_moveit_demos'''
executable: '''pick_place'''
name: '''pick_place'''
output: '''screen'''
parameters: '[config_file]'
```

- `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_description/launch/view_model.launch.py:39`

```yaml
package: '''rviz2'''
executable: '''rviz2'''
name: '''rviz2'''
output: '''screen'''
arguments: '[''-d'', rviz_config_dir]'
parameters: '[{''use_sim_time'': False}]'
```

- `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_description/launch/view_model.launch.py:47`

```yaml
name: '''model_node'''
package: '''robot_state_publisher'''
executable: '''robot_state_publisher'''
namespace: ''''''
output: '''screen'''
arguments: '[urdf]'
```

- `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/launch/lidar.launch.py:224`

```yaml
package: '''orbbec_camera'''
executable: '''orbbec_camera_node'''
name: '''ob_camera_node'''
namespace: LaunchConfiguration('camera_name')
parameters: params
output: '''log'''
```

- `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/launch/gemini305_g.launch.py:324`

```yaml
package: '''orbbec_camera'''
executable: '''orbbec_camera_node'''
name: '''ob_camera_node'''
namespace: LaunchConfiguration('camera_name')
parameters: params
output: '''log'''
```

- `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/launch/gemini210.launch.py:146`

```yaml
package: '''orbbec_camera'''
executable: '''orbbec_camera_node'''
name: '''ob_camera_node'''
namespace: LaunchConfiguration('camera_name')
parameters: parameters
output: '''log'''
```

- `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/launch/dabai_a.launch.py:235`

```yaml
package: '''orbbec_camera'''
executable: '''orbbec_camera_node'''
name: '''ob_camera_node'''
namespace: LaunchConfiguration('camera_name')
parameters: params
output: '''log'''
```

- `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/launch/gemini2L.launch.py:200`

```yaml
package: '''orbbec_camera'''
executable: '''orbbec_camera_node'''
name: '''ob_camera_node'''
namespace: LaunchConfiguration('camera_name')
parameters: params
output: '''log'''
```

- `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/launch/femto_mega.launch.py:141`

```yaml
package: '''orbbec_camera'''
executable: '''orbbec_camera_node'''
name: '''ob_camera_node'''
namespace: LaunchConfiguration('camera_name')
parameters: parameters
output: '''log'''
```

- `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/launch/gemini_330_series_low_cpu.launch.py:320`

```yaml
package: '''orbbec_camera'''
executable: '''orbbec_camera_node'''
name: '''ob_camera_node'''
namespace: LaunchConfiguration('camera_name')
parameters: params
output: '''log'''
```

- `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/launch/femto_bolt.launch.py:127`

```yaml
package: '''orbbec_camera'''
executable: '''orbbec_camera_node'''
name: '''ob_camera_node'''
namespace: LaunchConfiguration('camera_name')
parameters: parameters
output: '''log'''
```

- `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/launch/gemini305.launch.py:324`

```yaml
package: '''orbbec_camera'''
executable: '''orbbec_camera_node'''
name: '''ob_camera_node'''
namespace: LaunchConfiguration('camera_name')
parameters: params
output: '''log'''
```

- `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/launch/gemini345.launch.py:234`

```yaml
package: '''orbbec_camera'''
executable: '''orbbec_camera_node'''
name: '''ob_camera_node'''
namespace: LaunchConfiguration('camera_name')
parameters: params
output: '''log'''
```

- `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/launch/gemini_330_series.launch.py:330`

```yaml
package: '''orbbec_camera'''
executable: '''orbbec_camera_node'''
name: '''ob_camera_node'''
namespace: LaunchConfiguration('camera_name')
parameters: params
output: '''log'''
```

- `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/launch/gemini345_lg.launch.py:237`

```yaml
package: '''orbbec_camera'''
executable: '''orbbec_camera_node'''
name: '''ob_camera_node'''
namespace: LaunchConfiguration('camera_name')
parameters: params
output: '''log'''
```

- `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/launch/dabai_al.launch.py:237`

```yaml
package: '''orbbec_camera'''
executable: '''orbbec_camera_node'''
name: '''ob_camera_node'''
namespace: LaunchConfiguration('camera_name')
parameters: params
output: '''log'''
```

- `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/launch/gemini435_le.launch.py:284`

```yaml
package: '''orbbec_camera'''
executable: '''orbbec_camera_node'''
name: '''ob_camera_node'''
namespace: LaunchConfiguration('camera_name')
parameters: params
output: '''log'''
```

- `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/launch/femto.launch.py:99`

```yaml
package: '''orbbec_camera'''
executable: '''orbbec_camera_node'''
name: '''ob_camera_node'''
namespace: LaunchConfiguration('camera_name')
parameters: parameters
output: '''log'''
```

- `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/launch/orbbec_camera.launch.py:137`

```yaml
name: attach_component_container_name
package: '''rclcpp_components'''
executable: '''component_container_mt'''
output: '''screen'''
condition: UnlessCondition(attach_component_container_enable)
```

- `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/launch/orbbec_camera.launch.py:162`

```yaml
package: default_package_name
executable: '''orbbec_camera_node'''
name: '''ob_camera_node'''
namespace: camera_name
parameters: params
output: '''log'''
```

- `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/launch/dabai_dcw2.launch.py:103`

```yaml
package: '''orbbec_camera'''
executable: '''orbbec_camera_node'''
name: '''ob_camera_node'''
namespace: LaunchConfiguration('camera_name')
parameters: parameters
output: '''log'''
```

- `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/launch/gemini2.launch.py:148`

```yaml
package: '''orbbec_camera'''
executable: '''orbbec_camera_node'''
name: '''ob_camera_node'''
namespace: LaunchConfiguration('camera_name')
parameters: parameters
output: '''log'''
```

- `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/launch/astra.launch.py:119`

```yaml
package: '''orbbec_camera'''
executable: '''orbbec_camera_node'''
name: '''ob_camera_node'''
namespace: LaunchConfiguration('camera_name')
parameters: parameters
output: '''log'''
```

- `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/launch/dabai_max_pro.launch.py:95`

```yaml
package: '''orbbec_camera'''
executable: '''orbbec_camera_node'''
name: '''ob_camera_node'''
namespace: LaunchConfiguration('camera_name')
parameters: parameters
output: '''log'''
```

- `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/launch/astra2.launch.py:120`

```yaml
package: '''orbbec_camera'''
executable: '''orbbec_camera_node'''
name: '''ob_camera_node'''
namespace: LaunchConfiguration('camera_name')
parameters: parameters
output: '''log'''
```

- `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/examples/benchmark/ob_benchmark_1.launch.py:61`

```yaml
name: component_container_name_arg
package: '''rclcpp_components'''
executable: '''component_container_mt'''
output: '''log'''
condition: UnlessCondition(attach_to_shared_component_container_arg)
```

- `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/examples/benchmark/gemini_330_series_benchmark.launch.py:280`

```yaml
package: '''orbbec_camera'''
executable: '''orbbec_camera_node'''
name: '''ob_camera_node'''
namespace: LaunchConfiguration('camera_name')
parameters: params
output: '''log'''
```

- `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/examples/benchmark/gemini_330_series_benchmark.launch.py:293`

```yaml
name: component_container_name_arg
package: '''rclcpp_components'''
executable: '''component_container_mt'''
output: '''screen'''
condition: UnlessCondition(attach_to_shared_component_container_arg)
```

- `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/examples/benchmark/ob_benchmark_0.launch.py:61`

```yaml
name: component_container_name_arg
package: '''rclcpp_components'''
executable: '''component_container_mt'''
output: '''log'''
condition: UnlessCondition(attach_to_shared_component_container_arg)
```

- `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/examples/lower_cpu_usage/gemini_330_series_lower_cpu_usage.launch.py:224`

```yaml
package: '''orbbec_camera'''
executable: '''orbbec_camera_node'''
name: '''ob_camera_node'''
namespace: LaunchConfiguration('camera_name')
parameters: params
output: '''log'''
```

- `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/examples/lower_cpu_usage/gemini_330_series_lower_cpu_usage.launch.py:237`

```yaml
name: component_container_name_arg
package: '''rclcpp_components'''
executable: '''component_container_mt'''
output: '''screen'''
condition: UnlessCondition(attach_to_shared_component_container_arg)
```

- `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/examples/lower_cpu_usage/multi_camera_lower_cpu_usage.launch.py:61`

```yaml
name: component_container_name_arg
package: '''rclcpp_components'''
executable: '''component_container_mt'''
output: '''log'''
condition: UnlessCondition(attach_to_shared_component_container_arg)
```

- `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/examples/gmsl_camera/multi_gmsl_camera.launch.py:59`

```yaml
name: component_container_name_arg
package: '''rclcpp_components'''
executable: '''component_container_mt'''
output: '''log'''
condition: UnlessCondition(attach_to_shared_component_container_arg)
```

- `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/examples/gmsl_camera/multi_gmsl_camera_synced.launch.py:59`

```yaml
name: component_container_name_arg
package: '''rclcpp_components'''
executable: '''component_container_mt'''
output: '''log'''
condition: UnlessCondition(attach_to_shared_component_container_arg)
```

- `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/examples/gmsl_camera/gemini_330_gmsl.launch.py:280`

```yaml
package: '''orbbec_camera'''
executable: '''orbbec_camera_node'''
name: '''ob_camera_node'''
namespace: LaunchConfiguration('camera_name')
parameters: params
output: '''log'''
```

- `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/examples/gmsl_camera/gemini_330_gmsl.launch.py:293`

```yaml
name: component_container_name_arg
package: '''rclcpp_components'''
executable: '''component_container_mt'''
output: '''screen'''
condition: UnlessCondition(attach_to_shared_component_container_arg)
```

- `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/examples/multi_camera_synced_verification_tool/multi_camera_synced_verify.launch.py:60`

```yaml
name: component_container_name_arg
package: '''rclcpp_components'''
executable: '''component_container_mt'''
output: '''log'''
condition: UnlessCondition(attach_to_shared_component_container_arg)
```

- `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/examples/multi_camera_synced_verification_tool/gemini_synced_verify.launch.py:169`

```yaml
package: '''orbbec_camera'''
executable: '''orbbec_camera_node'''
name: '''ob_camera_node'''
namespace: LaunchConfiguration('camera_name')
parameters: params
output: '''log'''
```

- `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/examples/multi_camera_synced_verification_tool/gemini_synced_verify.launch.py:182`

```yaml
name: component_container_name_arg
package: '''rclcpp_components'''
executable: '''component_container_mt'''
output: '''screen'''
condition: UnlessCondition(attach_to_shared_component_container_arg)
```

- `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/examples/multi_camera_synced_verification_tool/gemini_330_series_synced_verify.launch.py:280`

```yaml
package: '''orbbec_camera'''
executable: '''orbbec_camera_node'''
name: '''ob_camera_node'''
namespace: LaunchConfiguration('camera_name')
parameters: params
output: '''log'''
```

- `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/examples/multi_camera_synced_verification_tool/gemini_330_series_synced_verify.launch.py:293`

```yaml
name: component_container_name_arg
package: '''rclcpp_components'''
executable: '''component_container_mt'''
output: '''screen'''
condition: UnlessCondition(attach_to_shared_component_container_arg)
```

- `dual_arm_ws/src/piper/launch/start_single_piper.launch.py:46`

```yaml
package: '''piper'''
executable: '''piper_single_ctrl'''
name: '''piper_ctrl_single_node'''
output: '''screen'''
ros_arguments: '[''--log-level'', LaunchConfiguration(''log_level'')]'
parameters: '[{''can_port'': LaunchConfiguration(''can_port''), ''auto_enable'': LaunchConfiguration(''auto_enable''),
  ''gripper_val_mutiple'': LaunchConfiguration(''gripper_val_mutiple''), ''gripper_exist'':
  LaunchConfiguration(''gripper_exist'')}]'
remappings: '[(''joint_ctrl_single'', ''/joint_states'')]'
```

- `dual_arm_ws/src/piper/launch/start_single_piper_rviz.launch.py:61`

```yaml
package: '''piper'''
executable: '''piper_single_ctrl'''
name: '''piper_ctrl_single_node'''
output: '''screen'''
parameters: '[{''can_port'': LaunchConfiguration(''can_port'')}, {''auto_enable'':
  LaunchConfiguration(''auto_enable'')}, {''gripper_val_mutiple'': LaunchConfiguration(''gripper_val_mutiple'')},
  {''gripper_exist'': LaunchConfiguration(''gripper_exist'')}]'
ros_arguments: '[''--log-level'', log_level]'
remappings: '[(''joint_ctrl_single'', ''/joint_states'')]'
```

- `dual_arm_ws/src/piper/launch/start_two_piper.launch.py:52`

```yaml
package: '''piper'''
executable: '''piper_single_ctrl'''
name: '''piper_left_ctrl_node'''
output: '''screen'''
ros_arguments: '[''--log-level'', LaunchConfiguration(''log_level'')]'
parameters: '[{''can_port'': LaunchConfiguration(''can_left_port''), ''auto_enable'':
  LaunchConfiguration(''auto_enable''), ''rviz_ctrl_flag'': LaunchConfiguration(''rviz_ctrl_flag''),
  ''girpper_exist'': LaunchConfiguration(''girpper_exist''), ''gripper_val_mutiple'':
  LaunchConfiguration(''gripper_val_mutiple'')}]'
remappings: '[(''pos_cmd'', ''/pos_cmd_left''), (''joint_ctrl_single'', ''/joint_ctrl_cmd_left''),
  (''joint_states_single'', ''/joint_states_left''), (''joint_states_feedback'', ''/joint_left''),
  (''joint_ctrl'', ''/joint_states_ctrl_left''), (''arm_status'', ''/arm_status_left''),
  (''end_pose'', ''/end_pose_left''), (''end_pose_stamped'', ''/end_pose_stamped_left'')]'
```

- `dual_arm_ws/src/piper/launch/start_two_piper.launch.py:79`

```yaml
package: '''piper'''
executable: '''piper_single_ctrl'''
name: '''piper_right_ctrl_node'''
output: '''screen'''
ros_arguments: '[''--log-level'', LaunchConfiguration(''log_level'')]'
parameters: '[{''can_port'': LaunchConfiguration(''can_right_port''), ''auto_enable'':
  LaunchConfiguration(''auto_enable''), ''rviz_ctrl_flag'': LaunchConfiguration(''rviz_ctrl_flag''),
  ''girpper_exist'': LaunchConfiguration(''girpper_exist''), ''gripper_val_mutiple'':
  LaunchConfiguration(''gripper_val_mutiple'')}]'
remappings: '[(''pos_cmd'', ''/pos_cmd_right''), (''joint_ctrl_single'', ''/joint_ctrl_cmd_right''),
  (''joint_states_single'', ''/joint_states_right''), (''joint_states_feedback'',
  ''/joint_right''), (''joint_ctrl'', ''/joint_states_ctrl_right''), (''arm_status'',
  ''/arm_status_right''), (''end_pose'', ''/end_pose_right''), (''end_pose_stamped'',
  ''/end_pose_stamped_right'')]'
```

- `dual_arm_ws/src/moveit_visual_tools/launch/demo_rviz.launch.py:61`

```yaml
package: '''moveit_visual_tools'''
executable: '''moveit_visual_tools_demo'''
output: '''screen'''
parameters: '[robot_description, robot_description_semantic]'
```

- `dual_arm_ws/src/moveit_visual_tools/launch/demo_rviz.launch.py:76`

```yaml
package: '''rviz2'''
executable: '''rviz2'''
name: '''rviz2'''
output: '''log'''
arguments: '[''-d'', rviz_config_file]'
parameters: '[robot_description, robot_description_semantic]'
```

- `dual_arm_ws/src/moveit_visual_tools/launch/demo_rviz.launch.py:89`

```yaml
package: '''tf2_ros'''
executable: '''static_transform_publisher'''
name: '''static_transform_publisher'''
output: '''log'''
arguments: '[''0.0'', ''0.0'', ''0.0'', ''0.0'', ''0.0'', ''0.0'', ''world'', ''base'']'
```

- `dual_arm_ws/src/easy_handeye2/easy_handeye2/launch/evaluate.launch.py:10`

```yaml
package: '''easy_handeye2'''
executable: '''rqt_evaluator.py'''
name: '''handeye_rqt_evaluator'''
parameters: '[{''name'': LaunchConfiguration(''name'')}]'
```

- `dual_arm_ws/src/easy_handeye2/easy_handeye2/launch/calibrate.launch.py:20`

```yaml
package: '''tf2_ros'''
executable: '''static_transform_publisher'''
name: '''dummy_publisher'''
condition: is_eye_in_hand
arguments: f'--x 0 --y 0 --z 0.1 --qx 0 --qy 0 --qz 0 --qw 1'.split(' ') + ['--frame-id',
  LaunchConfiguration('robot_effector_frame'), '--child-frame-id', LaunchConfiguration('tracking_base_frame')]
```

- `dual_arm_ws/src/easy_handeye2/easy_handeye2/launch/calibrate.launch.py:25`

```yaml
package: '''tf2_ros'''
executable: '''static_transform_publisher'''
name: '''dummy_publisher'''
condition: is_eye_on_base
arguments: f'--x 1 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1'.split(' ') + ['--frame-id',
  LaunchConfiguration('robot_base_frame'), '--child-frame-id', LaunchConfiguration('tracking_base_frame')]
```

- `dual_arm_ws/src/easy_handeye2/easy_handeye2/launch/calibrate.launch.py:35`

```yaml
package: '''easy_handeye2'''
executable: '''handeye_server'''
name: '''handeye_server'''
parameters: '[{''name'': LaunchConfiguration(''name''), ''calibration_type'': LaunchConfiguration(''calibration_type''),
  ''tracking_base_frame'': LaunchConfiguration(''tracking_base_frame''), ''tracking_marker_frame'':
  LaunchConfiguration(''tracking_marker_frame''), ''robot_base_frame'': LaunchConfiguration(''robot_base_frame''),
  ''robot_effector_frame'': LaunchConfiguration(''robot_effector_frame'')}]'
```

- `dual_arm_ws/src/easy_handeye2/easy_handeye2/launch/calibrate.launch.py:44`

```yaml
package: '''easy_handeye2'''
executable: '''rqt_calibrator.py'''
name: '''handeye_rqt_calibrator'''
parameters: '[{''name'': LaunchConfiguration(''name''), ''calibration_type'': LaunchConfiguration(''calibration_type''),
  ''tracking_base_frame'': LaunchConfiguration(''tracking_base_frame''), ''tracking_marker_frame'':
  LaunchConfiguration(''tracking_marker_frame''), ''robot_base_frame'': LaunchConfiguration(''robot_base_frame''),
  ''robot_effector_frame'': LaunchConfiguration(''robot_effector_frame'')}]'
```

- `dual_arm_ws/src/easy_handeye2/easy_handeye2/launch/publish.launch.py:10`

```yaml
package: '''easy_handeye2'''
executable: '''handeye_publisher'''
name: '''handeye_publisher'''
parameters: '[{''name'': LaunchConfiguration(''name'')}]'
```

- `dual_arm_ws/src/piper_h_description/launch/display_xacro.launch.py:26`

```yaml
package: '''robot_state_publisher'''
executable: '''robot_state_publisher'''
parameters: '[{''robot_description'': robot_description}]'
```

- `dual_arm_ws/src/piper_h_description/launch/display_xacro.launch.py:33`

```yaml
package: '''joint_state_publisher'''
executable: '''joint_state_publisher'''
condition: UnlessCondition(LaunchConfiguration('gui'))
```

- `dual_arm_ws/src/piper_h_description/launch/display_xacro.launch.py:39`

```yaml
package: '''joint_state_publisher_gui'''
executable: '''joint_state_publisher_gui'''
condition: IfCondition(LaunchConfiguration('gui'))
```

- `dual_arm_ws/src/piper_h_description/launch/display_xacro.launch.py:45`

```yaml
package: '''rviz2'''
executable: '''rviz2'''
name: '''rviz2'''
output: '''screen'''
arguments: '[''-d'', LaunchConfiguration(''rvizconfig'')]'
```

- `dual_arm_ws/src/piper_h_description/launch/display_urdf.launch.py:26`

```yaml
package: '''robot_state_publisher'''
executable: '''robot_state_publisher'''
parameters: '[{''robot_description'': robot_description}]'
```

- `dual_arm_ws/src/piper_h_description/launch/display_urdf.launch.py:33`

```yaml
package: '''joint_state_publisher'''
executable: '''joint_state_publisher'''
condition: UnlessCondition(LaunchConfiguration('gui'))
```

- `dual_arm_ws/src/piper_h_description/launch/display_urdf.launch.py:39`

```yaml
package: '''joint_state_publisher_gui'''
executable: '''joint_state_publisher_gui'''
condition: IfCondition(LaunchConfiguration('gui'))
```

- `dual_arm_ws/src/piper_h_description/launch/display_urdf.launch.py:45`

```yaml
package: '''rviz2'''
executable: '''rviz2'''
name: '''rviz2'''
output: '''screen'''
arguments: '[''-d'', LaunchConfiguration(''rvizconfig'')]'
```

- `dual_arm_ws/src/piperh_control/launch/hardware.launch.py:113`

```yaml
package: '''robot_state_publisher'''
executable: '''robot_state_publisher'''
output: '''screen'''
parameters: '[{''robot_description'': description, ''frame_prefix'': LaunchConfiguration(''tf_prefix'')}]'
```

- `dual_arm_ws/src/piperh_control/launch/hardware.launch.py:126`

```yaml
package: '''piper'''
executable: '''piper_single_ctrl'''
name: '''piper_ctrl_single_node'''
output: '''screen'''
respawn: 'True'
respawn_delay: '2.0'
parameters: '[{''can_port'': LaunchConfiguration(''can_port''), ''auto_enable'': False,
  ''gripper_exist'': False}]'
remappings: '[(''joint_ctrl_single'', ''/piperh/driver_joint_command''), (''joint_states_single'',
  ''/piperh/driver_joint_states''), (''joint_states_feedback'', ''/piperh/driver_joint_states_diagnostic''),
  (''teach_joint_states_raw'', ''/piperh/teach_joint_states_raw''), (''arm_status'',
  ''/piperh/driver_arm_status'')]'
```

- `dual_arm_ws/src/piperh_control/launch/hardware.launch.py:153`

```yaml
package: '''piperh_control'''
executable: '''hardware_adapter'''
output: '''screen'''
respawn: 'True'
respawn_delay: '2.0'
parameters: '[LaunchConfiguration(''gravity_config_file''), {''feedback_topic'': LaunchConfiguration(''joint_states_topic''),
  ''trajectory_action'': LaunchConfiguration(''trajectory_action''), ''can_port'':
  LaunchConfiguration(''can_port''), ''driver_speed_percent'': ParameterValue(LaunchConfiguration(''driver_speed_percent''),
  value_type=int), ''playback_max_velocity_rad_s'': ParameterValue(LaunchConfiguration(''playback_max_velocity_rad_s''),
  value_type=float), ''playback_max_acceleration_rad_s2'': ParameterValue(LaunchConfiguration(''playback_max_acceleration_rad_s2''),
  value_type=float), ''gravity_require_selected_robot'': LaunchConfiguration(''gravity_require_selected_robot''),
  ''gravity_mount_roll'': mount_roll, ''gravity_mount_pitch'': mount_pitch, ''gravity_mount_yaw'':
  mount_yaw}]'
```

- `dual_arm_ws/src/piperh_control/launch/hardware.launch.py:185`

```yaml
package: '''rviz2'''
executable: '''rviz2'''
output: '''log'''
arguments: '[''-d'', os.path.join(get_package_share_directory(''piperh_moveit_config''),
  ''config'', ''moveit.rviz'')]'
condition: IfCondition(use_rviz)
parameters: '[moveit.planning_pipelines, moveit.robot_description_kinematics, moveit.robot_description,
  moveit.robot_description_semantic, {''motion_preset.piper_driver_speed_percent'':
  ParameterValue(LaunchConfiguration(''driver_speed_percent''), value_type=int)}]'
```

- `dual_arm_ws/src/piperh_control/launch/hardware.launch.py:203`

```yaml
package: '''moveit_servo'''
executable: '''servo_node'''
name: '''servo_node'''
output: '''screen'''
condition: IfCondition(use_servo)
parameters: '[servo, {''update_period'': 0.01, ''planning_group_name'': ''arm''},
  moveit.robot_description, moveit.robot_description_semantic, moveit.robot_description_kinematics,
  moveit.joint_limits]'
```

- `dual_arm_ws/src/piperh_control/launch/hardware.launch.py:220`

```yaml
package: '''joy_linux'''
executable: '''joy_linux_node'''
name: '''piperh_joy'''
output: '''screen'''
condition: IfCondition(use_xbox)
parameters: '[{''dev'': LaunchConfiguration(''joy_device''), ''deadzone'': 0.05, ''autorepeat_rate'':
  20.0}]'
```

- `dual_arm_ws/src/piperh_control/launch/hardware.launch.py:230`

```yaml
package: '''rebot_xbox_servo'''
executable: '''rebot_xbox_twist'''
name: '''rebot_xbox_twist'''
output: '''screen'''
condition: IfCondition(use_xbox)
parameters: '[LaunchConfiguration(''mapping'')]'
```

- `dual_arm_ws/src/piperh_control/launch/simulation.launch.py:39`

```yaml
package: '''piperh_control'''
executable: '''joint_state_udp_bridge'''
output: '''screen'''
```

- `dual_arm_ws/src/piperh_control/launch/simulation.launch.py:44`

```yaml
package: '''moveit_servo'''
executable: '''servo_node'''
name: '''servo_node'''
output: '''screen'''
condition: IfCondition(use_xbox)
parameters: '[servo, {''update_period'': 0.01, ''planning_group_name'': ''arm''},
  moveit.robot_description, moveit.robot_description_semantic, moveit.robot_description_kinematics,
  moveit.joint_limits]'
```

- `dual_arm_ws/src/piperh_control/launch/simulation.launch.py:59`

```yaml
package: '''joy_linux'''
executable: '''joy_linux_node'''
name: '''piperh_joy'''
output: '''screen'''
condition: IfCondition(use_xbox)
parameters: '[{''dev'': LaunchConfiguration(''joy_device''), ''deadzone'': 0.05, ''autorepeat_rate'':
  20.0}]'
```

- `dual_arm_ws/src/piperh_control/launch/simulation.launch.py:67`

```yaml
package: '''rebot_xbox_servo'''
executable: '''rebot_xbox_twist'''
name: '''rebot_xbox_twist'''
output: '''screen'''
condition: IfCondition(use_xbox)
parameters: '[mapping]'
```

- `dual_arm_ws/src/rebotarm_bringup/launch/bringup.launch.py:71`

```yaml
package: '''rebotarmcontroller'''
executable: '''reBotArmController'''
name: '''reBotArmController'''
output: '''screen'''
sigterm_timeout: '''30.0'''
sigkill_timeout: '''5.0'''
on_exit: Shutdown(reason='reBotArmController exited')
parameters: '[{''hardware_config'': hardware_config, ''model'': model, ''channel'':
  channel, ''joint_state_rate'': joint_state_rate, ''cmd_arbitration'': cmd_arbitration,
  ''arm_namespace'': arm_namespace, ''frame_id'': frame_id, ''ee_frame_id'': ee_frame_id,
  ''disable_after_safe_home'': ParameterValue(disable_after_safe_home, value_type=bool),
  ''servo_joint_trajectory_topic'': servo_joint_trajectory_topic, ''servo_command_timeout'':
  ParameterValue(servo_command_timeout, value_type=float)}]'
```

- `dual_arm_ws/src/rebotarm_bringup/launch/bringup.launch.py:100`

```yaml
package: '''robot_state_publisher'''
executable: '''robot_state_publisher'''
name: '''robot_state_publisher'''
output: '''screen'''
parameters: '[{''robot_description'': robot_description}]'
remappings: '[(''/joint_states'', [''/'', arm_namespace, ''/joint_states''])]'
```

- `dual_arm_ws/src/rebotarm_bringup/launch/bringup.launch.py:108`

```yaml
package: '''rviz2'''
executable: '''rviz2'''
name: '''rviz2'''
output: '''screen'''
arguments: '[''-d'', rviz_config]'
condition: IfCondition(use_rviz)
```

- `dual_arm_ws/src/rebotarm_bringup/launch/driver.launch.py:39`

```yaml
package: '''rebotarmcontroller'''
executable: '''reBotArmController'''
name: '''reBotArmController'''
output: '''screen'''
sigterm_timeout: '''30.0'''
sigkill_timeout: '''5.0'''
parameters: '[{''hardware_config'': hardware_config, ''model'': model, ''channel'':
  channel, ''joint_state_rate'': joint_state_rate, ''cmd_arbitration'': cmd_arbitration,
  ''arm_namespace'': arm_namespace, ''disable_after_safe_home'': ParameterValue(disable_after_safe_home,
  value_type=bool), ''servo_joint_trajectory_topic'': servo_joint_trajectory_topic,
  ''servo_command_timeout'': ParameterValue(servo_command_timeout, value_type=float)}]'
```

- `dual_arm_ws/src/rebot_xbox_hardware/launch/xbox_hardware_servo.launch.py:136`

```yaml
package: '''rebot_xbox_hardware'''
executable: '''forbidden_zone_manager'''
name: '''forbidden_zone_manager'''
output: '''screen'''
condition: IfCondition(LaunchConfiguration('use_forbidden_zones'))
parameters: '[{''config_file'': LaunchConfiguration(''forbidden_zones_config''), ''active_groups'':
  LaunchConfiguration(''active_zone_groups''), ''user_config_file'': user_config,
  ''model'': model}]'
```

- `dual_arm_ws/src/rebot_xbox_hardware/launch/xbox_hardware_servo.launch.py:283`

```yaml
package: '''moveit_servo'''
executable: '''servo_node'''
name: '''servo_node'''
output: '''screen'''
parameters: '[servo_parameters, {''update_period'': 0.01, ''planning_group_name'':
  ''arm''}, moveit_config.robot_description, moveit_config.robot_description_semantic,
  moveit_config.robot_description_kinematics, moveit_config.joint_limits]'
```

- `dual_arm_ws/src/rebot_xbox_hardware/launch/xbox_hardware_servo.launch.py:297`

```yaml
package: '''joy_linux'''
executable: '''joy_linux_node'''
name: '''rebot_xbox_joy_node'''
output: '''screen'''
parameters: '[{''dev'': joy_device, ''deadzone'': 0.05, ''autorepeat_rate'': 20.0}]'
```

- `dual_arm_ws/src/rebot_xbox_hardware/launch/xbox_hardware_servo.launch.py:304`

```yaml
package: '''rebot_xbox_servo'''
executable: '''rebot_xbox_twist'''
name: '''rebot_xbox_twist'''
output: '''screen'''
parameters: '[mapping]'
```

- `dual_arm_ws/src/rebot_xbox_hardware/launch/xbox_hardware_servo.launch.py:311`

```yaml
package: '''rebot_xbox_hardware'''
executable: '''hardware_gripper'''
name: '''rebot_xbox_hardware_gripper'''
output: '''screen'''
parameters: '[{''state_topic'': f''/{namespace}/gripper/state'', ''command_topic'':
  f''/{namespace}/gripper/cmd/pos_vel'', ''open_position'': 5.0 if model == ''rs''
  else -5.0, ''closed_position'': 0.0, ''maximum_velocity'': 2.0, ''maximum_closing_torque'':
  0.8}]'
```

- `dual_arm_ws/src/rebot_xbox_hardware/launch/xbox_hardware_servo.launch.py:331`

```yaml
package: '''rebot_xbox_hardware'''
executable: '''instance_guard'''
name: '''rebotarm_single_instance_guard'''
output: '''screen'''
arguments: '[''--lock-name'', namespace]'
```

- `dual_arm_ws/src/rebot_xbox_hardware/launch/dual_arm_hardware.launch.py:149`

```yaml
package: '''rebot_xbox_servo'''
executable: '''rebot_xbox_twist'''
namespace: robot
name: '''rebot_xbox_twist'''
output: '''screen'''
parameters: '[values]'
```

- `dual_arm_ws/src/rebot_xbox_hardware/launch/dual_arm_hardware.launch.py:161`

```yaml
package: '''rebot_xbox_hardware'''
executable: '''forbidden_zone_manager'''
namespace: robot
name: '''forbidden_zone_manager'''
output: '''screen'''
condition: IfCondition(LaunchConfiguration('use_forbidden_zones'))
parameters: '[{''config_file'': os.path.join(share, ''config'', ''forbidden_zones.yaml''),
  ''user_config_file'': os.path.join(os.environ.get(''ROBOT_ZONE_ROOT'', os.path.expanduser(''~/.ros'')),
  robot, ''forbidden_zones_user.yaml''), ''model'': model, ''move_group_namespace'':
  robot, ''interactive_namespace'': f''/{robot}/forbidden_zones/interactive''}]'
```

- `dual_arm_ws/src/rebot_xbox_hardware/launch/dual_arm_hardware.launch.py:399`

```yaml
package: '''rebotarm_pulse'''
executable: '''piper_tool_geometry'''
name: '''piper_tool_geometry'''
output: '''screen'''
parameters: '[pulse_geometry_config]'
```

- `dual_arm_ws/src/rebot_xbox_hardware/launch/dual_arm_hardware.launch.py:406`

```yaml
package: '''rebotarm_pulse'''
executable: '''piper_pulse_target'''
name: '''piper_pulse_target'''
output: '''screen'''
parameters: '[pulse_target_config]'
condition: IfCondition(use_piper_pulse_observer)
```

- `dual_arm_ws/src/rebot_xbox_hardware/launch/dual_arm_hardware.launch.py:437`

```yaml
package: '''rebotarm_pulse'''
executable: '''pressure_serial_bridge'''
name: '''pressure_serial_bridge'''
output: '''screen'''
parameters: '[pulse_zero_config, {''port'': piper_pulse_serial_port}]'
```

- `dual_arm_ws/src/rebot_xbox_hardware/launch/dual_arm_hardware.launch.py:446`

```yaml
package: '''rebotarm_pulse'''
executable: '''pulse_web_gateway'''
name: '''pulse_web_gateway'''
output: '''screen'''
```

- `dual_arm_ws/src/rebot_xbox_hardware/launch/dual_arm_hardware.launch.py:455`

```yaml
package: '''robot_state_publisher'''
executable: '''robot_state_publisher'''
namespace: '''rebotarm'''
name: '''robot_state_publisher'''
output: '''both'''
parameters: '[rebot.robot_description, {''frame_prefix'': ''rebotarm/''}]'
remappings: '[(''/joint_states'', ''/rebotarm/joint_states'')]'
```

- `dual_arm_ws/src/rebot_xbox_hardware/launch/dual_arm_hardware.launch.py:464`

```yaml
package: '''tf2_ros'''
executable: '''static_transform_publisher'''
name: '''rebotarm_world_tf'''
arguments: '[LaunchConfiguration(''rebot_x''), LaunchConfiguration(''rebot_y''), LaunchConfiguration(''rebot_z''),
  ''0'', ''0'', ''0'', ''base_link'', ''rebotarm/base_link'']'
```

- `dual_arm_ws/src/rebot_xbox_hardware/launch/dual_arm_hardware.launch.py:476`

```yaml
package: '''tf2_ros'''
executable: '''static_transform_publisher'''
name: '''rebotarm_moveit_world_tf'''
arguments: '[''0'', LaunchConfiguration(''rebot_y''), ''0'', ''0'', ''0'', ''0'',
  ''world'', ''base_link'']'
```

- `dual_arm_ws/src/rebot_xbox_hardware/launch/dual_arm_hardware.launch.py:482`

```yaml
package: '''moveit_ros_move_group'''
executable: '''move_group'''
namespace: '''rebotarm'''
name: '''move_group'''
output: '''screen'''
parameters: '[rebot_moveit_params]'
remappings: '[(''/joint_states'', ''/rebotarm/joint_states''), (''/rebotarm/rebotarm/follow_joint_trajectory'',
  ''/rebotarm/follow_joint_trajectory'')]'
```

- `dual_arm_ws/src/rebot_xbox_hardware/launch/dual_arm_hardware.launch.py:494`

```yaml
package: '''moveit_servo'''
executable: '''servo_node'''
namespace: '''rebotarm'''
name: '''servo_node'''
output: '''screen'''
sigterm_timeout: '''2.0'''
sigkill_timeout: '''2.0'''
parameters: '[{''moveit_servo'': rebot_servo}, {''update_period'': 0.01, ''planning_group_name'':
  ''arm''}, rebot.robot_description, rebot.robot_description_semantic, rebot.robot_description_kinematics,
  rebot.joint_limits]'
```

- `dual_arm_ws/src/rebot_xbox_hardware/launch/dual_arm_hardware.launch.py:514`

```yaml
package: '''tf2_ros'''
executable: '''static_transform_publisher'''
name: '''piperh_world_tf'''
arguments: '[''0'', ''0'', ''0'', ''0'', ''0'', ''0'', ''base_link'', ''piperh/piperh_planning_world'']'
```

- `dual_arm_ws/src/rebot_xbox_hardware/launch/dual_arm_hardware.launch.py:526`

```yaml
package: '''tf2_ros'''
executable: '''static_transform_publisher'''
name: '''piperh_moveit_mount_tf'''
arguments: '[''0'', ''0'', ''0'', ''0'', ''0'', ''0'', ''base_link'', ''piperh_planning_world'']'
```

- `dual_arm_ws/src/rebot_xbox_hardware/launch/dual_arm_hardware.launch.py:535`

```yaml
package: '''tf2_ros'''
executable: '''static_transform_publisher'''
name: '''integrated_world_to_map_tf'''
condition: IfCondition(LaunchConfiguration('use_chassis'))
arguments: '[''0'', ''0'', ''0'', ''0'', ''0'', ''0'', ''world'', ''map'']'
```

- `dual_arm_ws/src/rebot_xbox_hardware/launch/dual_arm_hardware.launch.py:542`

```yaml
package: '''joy_linux'''
executable: '''joy_linux_node'''
name: '''dual_arm_joy'''
output: '''screen'''
parameters: '[{''dev'': LaunchConfiguration(''joy_device''), ''deadzone'': 0.05, ''autorepeat_rate'':
  20.0}]'
```

- `dual_arm_ws/src/rebot_xbox_hardware/launch/dual_arm_hardware.launch.py:557`

```yaml
package: '''rebot_xbox_hardware'''
executable: '''active_arm_manager'''
name: '''active_arm_manager'''
output: '''screen'''
parameters: '[{''initial_robot'': LaunchConfiguration(''initial_robot'')}]'
```

- `dual_arm_ws/src/rebot_xbox_hardware/launch/dual_arm_hardware.launch.py:564`

```yaml
package: '''rebot_xbox_hardware'''
executable: '''hardware_gripper'''
namespace: '''rebotarm'''
name: '''rebot_xbox_hardware_gripper'''
output: '''screen'''
parameters: '[{''open_topic'': ''/rebotarm/xbox/gripper_open'', ''close_topic'': ''/rebotarm/xbox/gripper_close'',
  ''state_topic'': ''/rebotarm/gripper/state'', ''command_topic'': ''/rebotarm/gripper/cmd/pos_vel'',
  ''open_position'': 5.0 if model == ''rs'' else -5.0, ''closed_position'': 0.0, ''maximum_velocity'':
  2.0, ''maximum_closing_torque'': 0.8}]'
```

- `dual_arm_ws/src/rebot_xbox_hardware/launch/dual_arm_hardware.launch.py:589`

```yaml
package: '''rviz2'''
executable: '''rviz2'''
name: '''dual_arm_rviz'''
output: '''screen'''
arguments: '[''-d'', rviz_config]'
condition: IfCondition(use_rviz)
additional_env: '{''LANG'': ''zh_CN.UTF-8'', ''LANGUAGE'': ''zh_CN:zh'', ''LC_ALL'':
  ''zh_CN.UTF-8'', ''LD_PRELOAD'': rviz_preload}'
parameters: '[rebot_moveit_params, {''piperh_robot_description'': piper_moveit.robot_description[''robot_description''],
  ''piperh_robot_description_semantic'': piper_moveit.robot_description_semantic[''robot_description_semantic''],
  ''motion_preset.initial_robot'': LaunchConfiguration(''initial_robot''), ''motion_preset.rebot_model'':
  model, ''motion_preset.dual_arm'': True, ''motion_preset.piper_driver_speed_percent'':
  ParameterValue(LaunchConfiguration(''piper_driver_speed_percent''), value_type=int),
  ''rebot_demo.integrate_motion_planning'': True}, piper_rviz_model_params]'
remappings: '[(''/joint_states'', ''/rebotarm/joint_states''), (''/rebot_xbox/armed'',
  ''/rebotarm/xbox/armed''), (''/rebot_xbox/set_armed'', ''/rebotarm/xbox/set_armed''),
  (''/check_state_validity'', ''/rebotarm/check_state_validity''), (''/execute_trajectory'',
  ''/rebotarm/execute_trajectory''), (''/display_planned_path'', ''/rebotarm/display_planned_path''),
  (''/forbidden_zone_manager/reload'', ''/rebotarm/forbidden_zone_manager/reload''),
  (''/forbidden_zone_manager/configure'', ''/rebotarm/forbidden_zone_manager/configure''),
  (''/forbidden_zone_manager/status'', ''/rebotarm/forbidden_zone_manager/status'')]'
respawn: 'True'
respawn_delay: '2.0'
```

- `dual_arm_ws/src/rebot_xbox_hardware/launch/dual_arm_hardware.launch.py:638`

```yaml
package: '''rebot_xbox_hardware'''
executable: '''instance_guard'''
name: '''dual_arm_single_instance_guard'''
output: '''screen'''
arguments: '[''--lock-name'', ''dual_arm'']'
```

- `dual_arm_ws/src/rebot_xbox_hardware/launch/hardware_selector.launch.py:11`

```yaml
package: '''rebot_xbox_hardware'''
executable: '''arm_selector'''
name: '''real_arm_selector'''
output: '''screen'''
```

- `dual_arm_ws/src/rebot_xbox_hardware/launch/dual_arm_offline_preview.launch.py:91`

```yaml
package: '''robot_state_publisher'''
executable: '''robot_state_publisher'''
namespace: robot
name: '''robot_state_publisher'''
output: '''both'''
parameters: '[moveit_config.robot_description, {''frame_prefix'': f''{robot}/''}]'
remappings: '[(''/joint_states'', joint_states)]'
```

- `dual_arm_ws/src/rebot_xbox_hardware/launch/dual_arm_offline_preview.launch.py:103`

```yaml
package: '''controller_manager'''
executable: '''ros2_control_node'''
namespace: robot
name: '''controller_manager'''
output: '''screen'''
parameters: '[moveit_config.robot_description, controller_config]'
remappings: '[(''/joint_states'', joint_states)]'
```

- `dual_arm_ws/src/rebot_xbox_hardware/launch/dual_arm_offline_preview.launch.py:112`

```yaml
package: '''moveit_ros_move_group'''
executable: '''move_group'''
namespace: robot
name: '''move_group'''
output: '''screen'''
parameters: '[moveit_params]'
remappings: '[(''/joint_states'', joint_states)]'
```

- `dual_arm_ws/src/rebot_xbox_hardware/launch/dual_arm_offline_preview.launch.py:121`

```yaml
package: '''controller_manager'''
executable: '''spawner'''
namespace: robot
name: '''joint_state_broadcaster_spawner'''
arguments: '[''joint_state_broadcaster'', ''--controller-manager'', f''{namespace}/controller_manager'']'
```

- `dual_arm_ws/src/rebot_xbox_hardware/launch/dual_arm_offline_preview.launch.py:135`

```yaml
package: '''controller_manager'''
executable: '''spawner'''
namespace: robot
name: f'{controller}_spawner'
arguments: '[controller, ''--controller-manager'', f''{namespace}/controller_manager'']'
```

- `dual_arm_ws/src/rebot_xbox_hardware/launch/dual_arm_offline_preview.launch.py:212`

```yaml
package: '''tf2_ros'''
executable: '''static_transform_publisher'''
name: '''rebotarm_world_tf'''
arguments: '[''0'', LaunchConfiguration(''rebot_y''), ''0'', ''0'', ''0'', ''0'',
  ''world'', ''rebotarm/base_link'']'
```

- `dual_arm_ws/src/rebot_xbox_hardware/launch/dual_arm_offline_preview.launch.py:221`

```yaml
package: '''tf2_ros'''
executable: '''static_transform_publisher'''
name: '''rebotarm_moveit_world_tf'''
arguments: '[''0'', LaunchConfiguration(''rebot_y''), ''0'', ''0'', ''0'', ''0'',
  ''world'', ''base_link'']'
```

- `dual_arm_ws/src/rebot_xbox_hardware/launch/dual_arm_offline_preview.launch.py:230`

```yaml
package: '''tf2_ros'''
executable: '''static_transform_publisher'''
name: '''piperh_world_tf'''
arguments: '[''0'', LaunchConfiguration(''piper_y''), ''0'', ''0'', ''0'', ''0'',
  ''world'', ''piperh/world'']'
```

- `dual_arm_ws/src/rebot_xbox_hardware/launch/dual_arm_offline_preview.launch.py:239`

```yaml
package: '''rebot_xbox_hardware'''
executable: '''active_arm_manager'''
name: '''active_arm_manager'''
output: '''screen'''
parameters: '[{''initial_robot'': initial_robot, ''offline_preview'': True}]'
```

- `dual_arm_ws/src/rebot_xbox_hardware/launch/dual_arm_offline_preview.launch.py:274`

```yaml
package: '''rviz2'''
executable: '''rviz2'''
name: '''dual_arm_offline_rviz'''
output: '''screen'''
arguments: '[''-d'', rviz_config]'
condition: IfCondition(LaunchConfiguration('use_rviz'))
parameters: '[rebot_moveit_params, piper_rviz_model_params, {''motion_preset.initial_robot'':
  initial_robot, ''motion_preset.rebot_model'': rebot_model, ''motion_preset.dual_arm'':
  True, ''dual_arm.offline_preview'': True, ''rebot_demo.integrate_motion_planning'':
  True}]'
remappings: '[(''/joint_states'', ''/rebotarm/joint_states''), (''/check_state_validity'',
  ''/rebotarm/check_state_validity''), (''/execute_trajectory'', ''/rebotarm/execute_trajectory''),
  (''/display_planned_path'', ''/rebotarm/display_planned_path'')]'
respawn: 'True'
respawn_delay: '2.0'
```

- `dual_arm_ws/src/rebot_xbox_hardware/launch/dual_arm_offline_preview.launch.py:304`

```yaml
package: '''rebot_xbox_hardware'''
executable: '''instance_guard'''
name: '''dual_arm_offline_single_instance_guard'''
output: '''screen'''
arguments: '[''--lock-name'', ''dual_arm'']'
```

- `dual_arm_ws/src/rebot_xbox_servo/launch/xbox_moveit_servo.launch.py:87`

```yaml
package: '''rebotarm_isaac_bridge'''
executable: '''joint_state_udp_bridge'''
name: '''rebotarm_isaac_joint_state_udp_bridge'''
output: '''screen'''
condition: IfCondition(LaunchConfiguration('start_isaac_bridge'))
parameters: '[{''joint_state_topic'': ''/joint_states'', ''host'': ''127.0.0.1'',
  ''port'': 5005, ''send_rate_hz'': 60.0, ''stale_timeout_sec'': 1.0, ''invert_joint_sign'':
  False}]'
```

- `dual_arm_ws/src/rebot_xbox_servo/launch/xbox_moveit_servo.launch.py:106`

```yaml
package: '''moveit_servo'''
executable: '''servo_node'''
name: '''servo_node'''
output: '''screen'''
condition: IfCondition(LaunchConfiguration('start_servo'))
parameters: '[servo_parameters, {''update_period'': 0.01, ''planning_group_name'':
  ''arm''}, moveit_config.robot_description, moveit_config.robot_description_semantic,
  moveit_config.robot_description_kinematics, moveit_config.joint_limits]'
```

- `dual_arm_ws/src/rebot_xbox_servo/launch/xbox_moveit_servo.launch.py:121`

```yaml
package: '''joy_linux'''
executable: '''joy_linux_node'''
name: '''rebot_xbox_joy_node'''
output: '''screen'''
condition: IfCondition(LaunchConfiguration('start_joy'))
parameters: '[{''dev'': joy_device, ''deadzone'': 0.05, ''autorepeat_rate'': 20.0,
  ''sticky_buttons'': False}]'
```

- `dual_arm_ws/src/rebot_xbox_servo/launch/xbox_moveit_servo.launch.py:138`

```yaml
package: '''rebot_xbox_servo'''
executable: '''rebot_xbox_twist'''
name: '''rebot_xbox_twist'''
output: '''screen'''
parameters: '[mapping]'
```

- `dual_arm_ws/src/rebot_xbox_servo/launch/xbox_moveit_servo.launch.py:146`

```yaml
package: '''rebot_xbox_servo'''
executable: '''arm_initializer'''
name: '''rebot_xbox_arm_initializer'''
output: '''screen'''
condition: IfCondition(LaunchConfiguration('start_initializer'))
parameters: '[mapping]'
```

- `dual_arm_ws/src/rebot_xbox_servo/launch/xbox_moveit_servo.launch.py:171`

```yaml
package: '''rebot_xbox_servo'''
executable: '''sim_gripper'''
name: '''rebot_xbox_sim_gripper'''
output: '''screen'''
condition: IfCondition(LaunchConfiguration('start_sim_gripper'))
parameters: '[mapping]'
```

- `src/antbot_dual_lidar/launch/isaac_3d_navigation.launch.py:40`

```yaml
package: '''nav2_map_server'''
executable: '''map_server'''
name: '''map_server'''
output: '''screen'''
parameters: '[params, {''yaml_filename'': map_yaml}]'
```

- `src/antbot_dual_lidar/launch/isaac_3d_navigation.launch.py:47`

```yaml
package: '''nav2_planner'''
executable: '''planner_server'''
name: '''planner_server'''
output: '''screen'''
parameters: '[params]'
```

- `src/antbot_dual_lidar/launch/isaac_3d_navigation.launch.py:54`

```yaml
package: '''nav2_controller'''
executable: '''controller_server'''
name: '''controller_server'''
output: '''screen'''
parameters: '[params]'
```

- `src/antbot_dual_lidar/launch/isaac_3d_navigation.launch.py:61`

```yaml
package: '''nav2_behaviors'''
executable: '''behavior_server'''
name: '''behavior_server'''
output: '''screen'''
parameters: '[params]'
```

- `src/antbot_dual_lidar/launch/isaac_3d_navigation.launch.py:68`

```yaml
package: '''nav2_bt_navigator'''
executable: '''bt_navigator'''
name: '''bt_navigator'''
output: '''screen'''
parameters: '[params]'
```

- `src/antbot_dual_lidar/launch/isaac_3d_navigation.launch.py:75`

```yaml
package: '''nav2_lifecycle_manager'''
executable: '''lifecycle_manager'''
name: '''lifecycle_manager_isaac_map'''
output: '''screen'''
parameters: '[params]'
```

- `src/antbot_dual_lidar/launch/isaac_3d_navigation.launch.py:82`

```yaml
package: '''nav2_lifecycle_manager'''
executable: '''lifecycle_manager'''
name: '''lifecycle_manager_navigation'''
output: '''screen'''
parameters: '[params]'
```

- `src/antbot_dual_lidar/launch/isaac_3d_navigation.launch.py:106`

```yaml
package: '''tf2_ros'''
executable: '''static_transform_publisher'''
name: '''map_to_odom_identity'''
condition: UnlessCondition(use_amcl)
arguments: '[''--x'', ''0'', ''--y'', ''0'', ''--z'', ''0'', ''--roll'', ''0'', ''--pitch'',
  ''0'', ''--yaw'', ''0'', ''--frame-id'', ''map'', ''--child-frame-id'', ''odom'']'
output: '''screen'''
```

- `src/antbot_dual_lidar/launch/isaac_3d_navigation.launch.py:119`

```yaml
package: '''nav2_amcl'''
executable: '''amcl'''
name: '''amcl'''
output: '''screen'''
condition: IfCondition(use_amcl)
parameters: '[params, {''use_sim_time'': True, ''scan_topic'': ''/scan_0''}]'
```

- `src/antbot_dual_lidar/launch/isaac_3d_navigation.launch.py:130`

```yaml
package: '''nav2_lifecycle_manager'''
executable: '''lifecycle_manager'''
name: '''lifecycle_manager_3d_amcl'''
output: '''screen'''
condition: IfCondition(use_amcl)
parameters: '[{''autostart'': True, ''bond_timeout'': 15.0, ''node_names'': [''amcl''],
  ''use_sim_time'': True}]'
```

- `src/antbot_dual_lidar/launch/isaac_3d_navigation.launch.py:143`

```yaml
package: '''antbot_dual_lidar'''
executable: '''cloud_preprocessor'''
name: '''antbot_dual_lidar_preprocessor'''
output: '''screen'''
parameters: '[lidar_params, {''use_sim_time'': True, ''lidar_profile'': ''navigation'',
  ''target_frame'': ''base_link'', ''front_left.input_topic'': ''/antbot/lidar/front_left/points_raw_native'',
  ''rear_right.input_topic'': ''/antbot/lidar/rear_right/points_raw_native''}]'
```

- `src/antbot_dual_lidar/launch/isaac_3d_navigation.launch.py:163`

```yaml
package: '''antbot_dual_lidar'''
executable: '''fixed_frame_mapper'''
name: '''antbot_fixed_frame_mapper'''
output: '''screen'''
parameters: '[{''use_sim_time'': True, ''fixed_frame'': ''odom''}]'
```

- `src/antbot_dual_lidar/launch/isaac_3d_navigation.launch.py:171`

```yaml
package: '''antbot_dual_lidar'''
executable: '''dynamic_obstacle_monitor'''
name: '''antbot_dynamic_obstacle_monitor'''
output: '''screen'''
parameters: '[{''use_sim_time'': True}]'
```

- `src/antbot_dual_lidar/launch/isaac_3d_navigation.launch.py:179`

```yaml
package: '''rviz2'''
executable: '''rviz2'''
name: '''rviz2_3d_navigation'''
arguments: '[''-d'', LaunchConfiguration(''rviz_config'')]'
parameters: '[{''use_sim_time'': True}]'
output: '''screen'''
condition: IfCondition(use_rviz)
```

- `src/antbot_dual_lidar/launch/antbot_rgbd_dataset.launch.py:49`

```yaml
package: '''antbot_rgbd_dataset'''
executable: '''rgbd_keyframe_recorder'''
name: '''rgbd_keyframe_recorder'''
output: '''screen'''
parameters: '[LaunchConfiguration(''config_file''), {''dataset_name'': LaunchConfiguration(''dataset_name''),
  ''output_root'': LaunchConfiguration(''output_root''), ''fixed_frame'': LaunchConfiguration(''fixed_frame''),
  ''camera_frame'': LaunchConfiguration(''camera_frame''), ''rgb_topic'': LaunchConfiguration(''rgb_topic''),
  ''depth_topic'': LaunchConfiguration(''depth_topic''), ''color_info_topic'': LaunchConfiguration(''color_info_topic''),
  ''depth_info_topic'': LaunchConfiguration(''depth_info_topic''), ''auto_start'':
  LaunchConfiguration(''auto_start''), ''overwrite_existing'': LaunchConfiguration(''overwrite_existing'')}]'
```

- `src/antbot_dual_lidar/launch/dual_lidar_bringup.launch.py:64`

```yaml
package: '''antbot_dual_lidar'''
executable: '''cloud_preprocessor'''
name: '''antbot_dual_lidar_preprocessor'''
output: '''screen'''
parameters: '[config_file, {''use_sim_time'': use_sim_time, ''lidar_profile'': lidar_profile,
  ''target_frame'': target_frame, ''front_left.input_topic'': front_topic, ''rear_right.input_topic'':
  rear_topic}]'
condition: IfCondition(start_preprocessing)
```

- `src/antbot_dual_lidar/launch/dual_lidar_bringup.launch.py:81`

```yaml
package: '''antbot_dual_lidar'''
executable: '''dual_lidar_diagnostics'''
name: '''antbot_dual_lidar_diagnostics'''
output: '''screen'''
parameters: '[config_file, {''use_sim_time'': use_sim_time, ''front_topic'': front_topic,
  ''rear_topic'': rear_topic}]'
condition: IfCondition(start_diagnostics)
```

- `src/antbot_dual_lidar/launch/dual_lidar_bringup.launch.py:96`

```yaml
package: '''antbot_dual_lidar'''
executable: '''dual_cloud_synchronizer'''
name: '''antbot_dual_lidar_synchronizer'''
output: '''screen'''
parameters: '[config_file, {''use_sim_time'': use_sim_time, ''front_topic'': front_topic,
  ''rear_topic'': rear_topic, ''strategy'': sync_strategy, ''publish_synchronized'':
  publish_synchronized}]'
condition: IfCondition(start_synchronizer)
```

- `src/antbot_dual_lidar/launch/dual_lidar_bringup.launch.py:113`

```yaml
package: '''antbot_dual_lidar'''
executable: '''ground_truth_deskew_validator'''
name: '''ground_truth_deskew_validator'''
output: '''screen'''
parameters: '[{''use_sim_time'': use_sim_time, ''point_time_convention'': point_time_convention}]'
condition: IfCondition(start_deskew_validator)
```

- `src/antbot_dual_lidar/launch/dual_lidar_bringup.launch.py:126`

```yaml
package: '''rviz2'''
executable: '''rviz2'''
name: '''dual_lidar_rviz'''
arguments: '[''-d'', rviz_config]'
parameters: '[{''use_sim_time'': use_sim_time}]'
output: '''screen'''
condition: IfCondition(use_rviz)
```

- `src/antbot_dual_lidar/launch/dual_lidar_replay.launch.py:33`

```yaml
package: '''antbot_dual_lidar'''
executable: '''cloud_preprocessor'''
name: '''antbot_dual_lidar_preprocessor'''
parameters: '[config, {''use_sim_time'': True}]'
output: '''screen'''
condition: IfCondition(start_preprocessing)
```

- `src/antbot_dual_lidar/launch/dual_lidar_replay.launch.py:41`

```yaml
package: '''antbot_dual_lidar'''
executable: '''dual_lidar_diagnostics'''
name: '''antbot_dual_lidar_diagnostics'''
parameters: '[config, {''use_sim_time'': True}]'
output: '''screen'''
condition: IfCondition(start_diagnostics)
```

- `src/antbot_dual_lidar/launch/dual_lidar_replay.launch.py:49`

```yaml
package: '''rviz2'''
executable: '''rviz2'''
name: '''dual_lidar_rviz'''
arguments: '[''-d'', rviz_config]'
parameters: '[{''use_sim_time'': True}]'
output: '''screen'''
condition: IfCondition(use_rviz)
```

- `src/antbot_dual_lidar/launch/isaac_mapping_stable.launch.py:27`

```yaml
package: '''slam_toolbox'''
executable: '''async_slam_toolbox_node'''
name: '''slam_toolbox'''
output: '''screen'''
parameters: '[params_file, {''use_sim_time'': True, ''scan_topic'': ''/scan_0''}]'
```

- `src/antbot_dual_lidar/launch/isaac_mapping_stable.launch.py:40`

```yaml
package: '''nav2_lifecycle_manager'''
executable: '''lifecycle_manager'''
name: '''lifecycle_manager_slam'''
output: '''screen'''
parameters: '[{''autostart'': True, ''bond_timeout'': 15.0, ''node_names'': [''slam_toolbox''],
  ''use_sim_time'': True}]'
```

- `src/antbot_dual_lidar/launch/isaac_mapping_stable.launch.py:56`

```yaml
package: '''antbot_dual_lidar'''
executable: '''cloud_preprocessor'''
name: '''antbot_dual_lidar_preprocessor'''
output: '''screen'''
condition: IfCondition(enable_3d_mapping)
parameters: '[lidar_params, {''use_sim_time'': True, ''lidar_profile'': ''mapping'',
  ''target_frame'': ''base_link'', ''front_left.input_topic'': ''/antbot/lidar/front_left/points_raw_native'',
  ''rear_right.input_topic'': ''/antbot/lidar/rear_right/points_raw_native''}]'
```

- `src/antbot_dual_lidar/launch/isaac_mapping_stable.launch.py:77`

```yaml
package: '''antbot_dual_lidar'''
executable: '''fixed_frame_mapper'''
name: '''antbot_fixed_frame_mapper'''
output: '''screen'''
condition: IfCondition(enable_3d_mapping)
parameters: '[{''use_sim_time'': True, ''fixed_frame'': ''odom''}]'
```

- `src/antbot_dual_lidar/launch/isaac_mapping_stable.launch.py:85`

```yaml
package: '''rviz2'''
executable: '''rviz2'''
name: '''rviz2_mapping'''
output: '''screen'''
condition: IfCondition(LaunchConfiguration('use_rviz'))
arguments: '[''-d'', LaunchConfiguration(''rviz_config'')]'
parameters: '[{''use_sim_time'': True}]'
```

- `src/antbot_imu/launch/imu.launch.py:45`

```yaml
package: '''antbot_imu'''
executable: '''imu_node'''
name: '''imu_node'''
parameters: '[imu_param, {''control_table_path'': control_table_path}]'
output: '''screen'''
```

- `src/antbot_camera/launch/camera_test.launch.py:35`

```yaml
package: '''antbot_camera'''
executable: '''camera_test_node'''
name: '''camera_test_node'''
namespace: ''''''
parameters: '[LaunchConfiguration(''config_file''), {''test.output_dir'': LaunchConfiguration(''output_dir''),
  ''test.save_interval_sec'': LaunchConfiguration(''save_interval_sec'')}]'
output: '''screen'''
emulate_tty: 'True'
```

- `src/antbot_camera/launch/camera.launch.py:38`

```yaml
package: '''antbot_camera'''
executable: '''antbot_camera_node'''
name: '''antbot_camera_node'''
namespace: ''''''
parameters: '[LaunchConfiguration(''config_file'')]'
output: '''screen'''
emulate_tty: 'True'
```

- `src/rebotarm_pulse/launch/handeye_calibrate.launch.py:121`

```yaml
package: '''image_proc'''
executable: '''rectify_node'''
name: '''handeye_rectify'''
namespace: '''rebotarm_handeye'''
output: '''screen'''
parameters: '[{''queue_size'': 1, ''qos_overrides./rebotarm_handeye/image_rect.publisher.reliability'':
  ''best_effort'', ''qos_overrides./rebotarm_handeye/image_rect.publisher.history'':
  ''keep_last'', ''qos_overrides./rebotarm_handeye/image_rect.publisher.depth'': 1}]'
remappings: '[(''image'', value(''color_topic'')), (''camera_info'', camera_info_topic),
  (''image_rect'', ''/rebotarm_handeye/image_rect'')]'
```

- `src/rebotarm_pulse/launch/handeye_calibrate.launch.py:146`

```yaml
package: '''rebotarm_pulse'''
executable: '''charuco_board_pose'''
name: '''charuco_board_pose'''
namespace: '''rebotarm_handeye'''
output: '''screen'''
parameters: '[{''squares_x'': 5, ''squares_y'': 7, ''square_length_m'': 0.035, ''marker_length_m'':
  0.026, ''minimum_charuco_corners'': int(value(''charuco_minimum_corners'')), ''marker_frame'':
  tracking_marker}]'
remappings: '[(''image'', ''/rebotarm_handeye/image_rect''), (''camera_info'', camera_info_topic)]'
```

- `src/rebotarm_pulse/launch/handeye_calibrate.launch.py:170`

```yaml
package: '''apriltag_ros'''
executable: '''apriltag_node'''
name: '''handeye_apriltag'''
namespace: '''rebotarm_handeye'''
output: '''screen'''
parameters: '[config, {''family'': tag_family, ''tag.ids'': detected_ids, ''tag.frames'':
  detected_frames, ''tag.sizes'': detected_sizes}]'
remappings: '[(''image_rect'', ''/rebotarm_handeye/image_rect''), (''camera_info'',
  camera_info_topic)]'
```

- `src/rebotarm_pulse/launch/handeye_calibrate.launch.py:193`

```yaml
package: '''rebotarm_pulse'''
executable: '''apriltag_board_pose'''
name: '''apriltag_board_pose'''
namespace: '''rebotarm_handeye'''
output: '''screen'''
parameters: '[{''tag_ids'': detected_ids, ''tag_size_m'': tag_size, ''center_spacing_x_m'':
  float(value(''board_spacing_x_m'')), ''center_spacing_y_m'': float(value(''board_spacing_y_m'')),
  ''minimum_visible_tags'': int(value(''board_minimum_visible_tags'')), ''marker_frame'':
  tracking_marker}]'
remappings: '[(''detections'', ''/rebotarm_handeye/detections''), (''camera_info'',
  camera_info_topic)]'
```

- `src/rebotarm_pulse/launch/handeye_calibrate.launch.py:233`

```yaml
package: '''rebotarm_pulse'''
executable: '''auto_handeye_sequence'''
name: '''auto_handeye_sequence'''
output: '''screen'''
condition: IfCondition(LaunchConfiguration('use_auto_sequence'))
parameters: '[{''sequence_name'': auto_sequence_name, ''action_name_prefix'': auto_action_prefix,
  ''calibration_type'': calibration_type, ''tracking_marker_frame'': tracking_marker,
  ''minimum_samples'': int(value(''auto_minimum_samples'')), ''teach_status_topic'':
  value(''teach_status_topic''), ''joint_state_topic'': value(''joint_state_topic''),
  ''teach_cancel_service'': value(''teach_cancel_service'')}]'
```

- `src/antbot_real_bringup/launch/real_base.launch.py:55`

```yaml
package: '''antbot_h743_bridge'''
executable: '''h743_cmd_vel_bridge'''
name: '''h743_cmd_vel_bridge'''
output: '''screen'''
parameters: '[{''port'': port, ''baud'': 115200, ''topic'': ''/cmd_vel'', ''status_topic'':
  ''/rs00/motor_status'', ''max_linear_speed'': ParameterValue(max_linear_speed, value_type=float),
  ''telemetry_period'': ParameterValue(telemetry_period, value_type=float), ''allow_disconnected'':
  ParameterValue(allow_disconnected, value_type=bool)}]'
```

- `src/antbot_real_bringup/launch/real_base.launch.py:77`

```yaml
package: '''antbot_h743_bridge'''
executable: '''antbot_operator_manager'''
name: '''antbot_operator_manager'''
output: '''screen'''
parameters: '[{''default_teleop_mode'': default_teleop_mode, ''manage_joy'': ParameterValue(start_joy,
  value_type=bool), ''joy_device'': joy_device, ''max_linear_speed'': ParameterValue(max_linear_speed,
  value_type=float), ''mapping_output_prefix'': mapping_output_prefix}]'
```

- `src/antbot_real_bringup/launch/real_base.launch.py:93`

```yaml
package: '''antbot_teleop'''
executable: '''mapping_xbox'''
name: '''antbot_real_xbox'''
output: '''screen'''
condition: IfCondition(start_xbox)
parameters: '[{''map_prefix'': ''/tmp/antbot_real_unused_map'', ''map_use_sim_time'':
  False, ''use_sim_time'': False, ''max_linear_vel'': ParameterValue(max_linear_speed,
  value_type=float), ''max_angular_vel'': 1.0, ''topics.cmd_vel'': ''/antbot/cmd_vel/xbox''}]'
```

- `src/antbot_real_bringup/launch/operator_step2.launch.py:53`

```yaml
package: '''nav2_map_server'''
executable: '''map_server'''
name: '''map_server'''
output: '''screen'''
parameters: '[{''yaml_filename'': map_yaml, ''use_sim_time'': False}]'
```

- `src/antbot_real_bringup/launch/operator_step2.launch.py:62`

```yaml
package: '''nav2_lifecycle_manager'''
executable: '''lifecycle_manager'''
name: '''lifecycle_manager_operator_map'''
output: '''screen'''
parameters: '[{''autostart'': True, ''node_names'': [''map_server''], ''use_sim_time'':
  False}]'
```

- `src/antbot_real_bringup/launch/operator_step2.launch.py:94`

```yaml
package: '''antbot_dual_lidar'''
executable: '''offline_pointcloud_publisher'''
name: '''antbot_offline_pointcloud_publisher'''
output: '''screen'''
condition: IfCondition(LaunchConfiguration('show_3d_cloud'))
parameters: '[{''pointcloud_path'': LaunchConfiguration(''pointcloud_path''), ''metadata_path'':
  LaunchConfiguration(''pointcloud_metadata''), ''publish_topic'': ''/antbot/offline_map_points'',
  ''target_frame'': ''map'', ''publish_rate'': 0.5, ''use_sim_time'': False}]'
```

- `src/antbot_real_bringup/launch/operator_step2.launch.py:109`

```yaml
package: '''antbot_rgbd_dataset'''
executable: '''offline_preview_publisher'''
name: '''antbot_rgbd_offline_cloud_publisher'''
output: '''screen'''
condition: IfCondition(LaunchConfiguration('show_rgbd_cloud'))
parameters: '[{''preview_path'': LaunchConfiguration(''rgbd_preview_path''), ''publish_topic'':
  ''/antbot/rgbd/offline_cloud'', ''publish_rate'': 0.5, ''use_sim_time'': False}]'
```

- `src/antbot_real_bringup/launch/operator_step2.launch.py:126`

```yaml
package: '''tf2_ros'''
executable: '''static_transform_publisher'''
name: '''operator_map_to_base_placeholder'''
output: '''screen'''
condition: IfCondition(LaunchConfiguration('publish_placeholder_pose'))
arguments: '[''--x'', ''0'', ''--y'', ''0'', ''--z'', ''0'', ''--roll'', ''0'', ''--pitch'',
  ''0'', ''--yaw'', ''0'', ''--frame-id'', ''map'', ''--child-frame-id'', ''base_link'']'
```

- `src/antbot_real_bringup/launch/operator_step2.launch.py:148`

```yaml
package: '''rviz2'''
executable: '''rviz2'''
name: '''rviz2_waypoints'''
output: '''screen'''
condition: IfCondition(LaunchConfiguration('use_operator_rviz'))
arguments: '[''-d'', LaunchConfiguration(''rviz_config'')]'
additional_env: '{''LANG'': ''zh_CN.UTF-8'', ''LANGUAGE'': ''zh_CN:zh'', ''LC_ALL'':
  ''zh_CN.UTF-8'', ''LD_PRELOAD'': rviz_preload}'
parameters: '[{''use_sim_time'': False}]'
```

- `src/robotcar_navigation/launch/add_waypoint.launch.py:131`

```yaml
package: '''nav2_map_server'''
executable: '''map_server'''
name: '''map_server'''
output: '''screen'''
parameters: '[{''yaml_filename'': map_file, ''use_sim_time'': use_sim_time}]'
```

- `src/robotcar_navigation/launch/add_waypoint.launch.py:144`

```yaml
package: '''nav2_lifecycle_manager'''
executable: '''lifecycle_manager'''
name: '''lifecycle_manager_map'''
output: '''screen'''
parameters: '[{''use_sim_time'': use_sim_time, ''autostart'': True, ''node_names'':
  [''map_server'']}]'
```

- `src/robotcar_navigation/launch/add_waypoint.launch.py:162`

```yaml
package: '''robotcar_navigation'''
executable: '''wp_edit_node'''
name: '''wp_edit_node'''
output: '''screen'''
parameters: '[{''load'': waypoints_file, ''use_sim_time'': use_sim_time}]'
```

- `src/robotcar_navigation/launch/add_waypoint.launch.py:179`

```yaml
package: '''robotcar_navigation'''
executable: '''wp_saver'''
name: '''wp_saver'''
output: '''screen'''
emulate_tty: 'True'
parameters: '[{''use_sim_time'': use_sim_time, ''save_file'': waypoints_file}]'
```

- `src/robotcar_navigation/launch/add_waypoint.launch.py:193`

```yaml
package: '''robotcar_navigation'''
executable: '''floor_texture_publisher'''
name: '''floor_texture_publisher'''
output: '''screen'''
condition: IfCondition(publish_floor)
parameters: '[{''use_sim_time'': use_sim_time, ''model'': floor_model, ''width'':
  ParameterValue(floor_width, value_type=float), ''height'': ParameterValue(floor_height,
  value_type=float), ''pos_x'': ParameterValue(floor_x, value_type=float), ''pos_y'':
  ParameterValue(floor_y, value_type=float), ''pos_z'': ParameterValue(floor_z, value_type=float)}]'
```

- `src/robotcar_navigation/launch/add_waypoint.launch.py:216`

```yaml
package: '''rviz2'''
executable: '''rviz2'''
name: '''rviz2'''
arguments: '[''-d'', rviz_config]'
condition: IfCondition(use_rviz)
parameters: '[{''use_sim_time'': use_sim_time}]'
output: '''screen'''
```

- `src/robotcar_navigation/launch/amcl_nav.launch.py:53`

```yaml
package: '''rviz2'''
executable: '''rviz2'''
name: '''rviz2'''
arguments: '[''-d'', LaunchConfiguration(''rviz_config'')]'
condition: IfCondition(LaunchConfiguration('use_rviz'))
parameters: '[{''use_sim_time'': use_sim_time}]'
output: '''screen'''
```

- `src/robotcar_navigation/launch/mapping.launch.py:39`

```yaml
package: '''slam_toolbox'''
executable: '''async_slam_toolbox_node'''
name: '''slam_toolbox'''
output: '''screen'''
parameters: '[LaunchConfiguration(''params_file''), {''use_sim_time'': use_sim_time}]'
```

- `src/robotcar_navigation/launch/mapping.launch.py:49`

```yaml
package: '''nav2_lifecycle_manager'''
executable: '''lifecycle_manager'''
name: '''lifecycle_manager_slam'''
output: '''screen'''
parameters: '[{''use_sim_time'': use_sim_time, ''autostart'': autostart, ''node_names'':
  [''slam_toolbox'']}]'
```

- `src/robotcar_navigation/launch/mapping.launch.py:62`

```yaml
package: '''rviz2'''
executable: '''rviz2'''
name: '''rviz2'''
arguments: '[''-d'', LaunchConfiguration(''rviz_config'')]'
condition: IfCondition(LaunchConfiguration('use_rviz'))
parameters: '[{''use_sim_time'': use_sim_time}]'
output: '''screen'''
```

- `src/robotcar_navigation/launch/go.launch.py:110`

```yaml
package: '''ros_gz_bridge'''
executable: '''parameter_bridge'''
name: '''robotcar_gz_bridge'''
output: '''screen'''
arguments: '[''/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'', ''/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry'',
  ''/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'', ''/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'',
  ''/camera/rgb/image_raw@sensor_msgs/msg/Image[gz.msgs.Image'', ''/camera/rgb/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'']'
```

- `src/robotcar_navigation/launch/go.launch.py:133`

```yaml
package: '''robotcar_navigation'''
executable: '''wp_edit_node'''
name: '''wp_edit_node'''
output: '''screen'''
parameters: '[{''load'': waypoints_file, ''use_sim_time'': use_sim_time}]'
```

- `src/robotcar_navigation/launch/go.launch.py:145`

```yaml
package: '''robotcar_navigation'''
executable: '''wp_set_pose'''
name: '''wp_set_pose'''
output: '''screen'''
parameters: '[{''use_sim_time'': use_sim_time}]'
```

- `src/robotcar_navigation/launch/go.launch.py:155`

```yaml
package: '''robotcar_navigation'''
executable: '''set_pose_from_waypoint'''
name: '''set_initial_pose_from_waypoint'''
output: '''screen'''
arguments: '[initial_pose_waypoint]'
parameters: '[{''use_sim_time'': use_sim_time}]'
```

- `src/robotcar_navigation/launch/go.launch.py:166`

```yaml
package: '''rviz2'''
executable: '''rviz2'''
name: '''rviz2'''
arguments: '[''-d'', rviz_config]'
condition: IfCondition(LaunchConfiguration('use_rviz'))
parameters: '[{''use_sim_time'': use_sim_time}]'
output: '''screen'''
```

- `src/antbot_lidar_fusion/launch/antbot_dual_lidar.launch.py:33`

```yaml
package: '''robot_state_publisher'''
executable: '''robot_state_publisher'''
name: '''antbot_robot_state_publisher'''
parameters: '[{''robot_description'': robot_description, ''use_sim_time'': use_sim_time}]'
remappings: '[(''robot_description'', ''/antbot/robot_description'')]'
condition: IfCondition(publish_description)
output: '''screen'''
```

- `src/antbot_lidar_fusion/launch/antbot_dual_lidar.launch.py:42`

```yaml
package: '''antbot_lidar_fusion'''
executable: '''lidar_fusion_node'''
parameters: '[{''use_sim_time'': use_sim_time, ''front_scan_topic'': front_topic,
  ''rear_scan_topic'': rear_topic, ''target_frame'': target_frame}]'
output: '''screen'''
```

- `src/antbot_lidar_fusion/launch/antbot_dual_lidar.launch.py:55`

```yaml
package: '''rviz2'''
executable: '''rviz2'''
arguments: '[''-d'', os.path.join(share, ''config'', ''antbot_dual_lidar.rviz'')]'
parameters: common
condition: IfCondition(use_rviz)
output: '''screen'''
```

- `src/antbot_teleop/launch/teleop_joy_sim.launch.py:23`

```yaml
package: '''joy'''
executable: '''joy_node'''
name: '''joy_node'''
output: '''screen'''
parameters: '[{''autorepeat_rate'': 20.0, ''deadzone'': 0.05}]'
```

- `src/antbot_teleop/launch/teleop_joy_sim.launch.py:33`

```yaml
package: '''antbot_teleop'''
executable: '''teleop_joystick'''
name: '''teleop_joystick'''
output: '''screen'''
```

- `src/antbot_teleop/launch/teleop_joy_sim.launch.py:39`

```yaml
package: '''antbot_teleop'''
executable: '''swerve_sim'''
name: '''swerve_sim'''
output: '''screen'''
```

- `src/antbot_teleop/launch/teleop_joy.launch.py:23`

```yaml
package: '''joy'''
executable: '''joy_node'''
name: '''joy_node'''
output: '''screen'''
parameters: '[{''autorepeat_rate'': 20.0, ''deadzone'': 0.05}]'
```

- `src/antbot_teleop/launch/teleop_joy.launch.py:33`

```yaml
package: '''antbot_teleop'''
executable: '''teleop_joystick'''
name: '''teleop_joystick'''
output: '''screen'''
```

- `src/antbot_teleop/launch/mapping_xbox.launch.py:26`

```yaml
package: '''joy_linux'''
executable: '''joy_linux_node'''
name: '''antbot_xbox_joy_node'''
output: '''screen'''
condition: IfCondition(start_joy)
parameters: '[{''dev'': joy_device, ''deadzone'': 0.05, ''autorepeat_rate'': 20.0,
  ''sticky_buttons'': False}]'
```

- `src/antbot_teleop/launch/mapping_xbox.launch.py:39`

```yaml
package: '''antbot_teleop'''
executable: '''mapping_xbox'''
name: '''antbot_mapping_xbox'''
output: '''screen'''
parameters: '[{''map_prefix'': map_prefix, ''map_use_sim_time'': use_sim_time, ''use_sim_time'':
  use_sim_time, ''max_linear_vel'': max_linear_speed, ''max_angular_vel'': max_angular_speed}]'
```

- `src/antbot_rgbd_dataset/launch/rgbd_phase4b_capture.launch.py:25`

```yaml
package: '''antbot_rgbd_dataset'''
executable: '''phase4b_capture_node'''
name: '''phase4b_capture_node'''
output: '''screen'''
parameters: '[default_config, LaunchConfiguration(''config'')]'
```

- `src/antbot_rgbd_dataset/launch/rgbd_phase4b_capture.launch.py:32`

```yaml
package: '''rviz2'''
executable: '''rviz2'''
name: '''rgbd_scan_rviz'''
output: '''screen'''
arguments: '[''-d'', LaunchConfiguration(''rviz_config'')]'
condition: IfCondition(LaunchConfiguration('use_rviz'))
```

- `src/antbot_navigation/launch/navigation.launch.py:70`

```yaml
package: '''antbot_navigation'''
executable: '''scan_fix_relay.py'''
name: f'scan_fix_relay_{index}'
output: '''screen'''
parameters: '[{''input_topic'': f''/scan_{index}'', ''output_topic'': f''/scan_{index}_fixed''}]'
```

- `src/antbot_navigation/launch/navigation.launch.py:96`

```yaml
package: '''nav2_planner'''
executable: '''planner_server'''
name: '''planner_server'''
output: '''screen'''
parameters: '[nav2_params_file, {''use_sim_time'': use_sim_time}]'
```

- `src/antbot_navigation/launch/navigation.launch.py:104`

```yaml
package: '''nav2_smoother'''
executable: '''smoother_server'''
name: '''smoother_server'''
output: '''screen'''
parameters: '[nav2_params_file, {''use_sim_time'': use_sim_time}]'
```

- `src/antbot_navigation/launch/navigation.launch.py:112`

```yaml
package: '''nav2_controller'''
executable: '''controller_server'''
name: '''controller_server'''
output: '''screen'''
parameters: '[nav2_params_file, {''use_sim_time'': use_sim_time}]'
```

- `src/antbot_navigation/launch/navigation.launch.py:120`

```yaml
package: '''nav2_behaviors'''
executable: '''behavior_server'''
name: '''behavior_server'''
output: '''screen'''
parameters: '[nav2_params_file, {''use_sim_time'': use_sim_time}]'
```

- `src/antbot_navigation/launch/navigation.launch.py:128`

```yaml
package: '''nav2_bt_navigator'''
executable: '''bt_navigator'''
name: '''bt_navigator'''
output: '''screen'''
parameters: '[nav2_params_file, {''use_sim_time'': use_sim_time}]'
```

- `src/antbot_navigation/launch/navigation.launch.py:136`

```yaml
package: '''nav2_lifecycle_manager'''
executable: '''lifecycle_manager'''
name: '''lifecycle_manager_navigation'''
output: '''screen'''
parameters: '[nav2_params_file, {''use_sim_time'': use_sim_time}]'
```

- `src/antbot_navigation/launch/waypoint_navigation.launch.py:47`

```yaml
package: '''robotcar_navigation'''
executable: '''wp_edit_node'''
name: '''wp_edit_node'''
output: '''screen'''
parameters: '[{''use_sim_time'': PythonExpression(["''", mode, "'' == ''sim''"]),
  ''load'': LaunchConfiguration(''waypoints_file'')}]'
```

- `src/antbot_navigation/launch/waypoint_navigation.launch.py:57`

```yaml
package: '''robotcar_navigation'''
executable: '''wp_navi_server'''
name: '''wp_navi_server'''
output: '''screen'''
sigterm_timeout: '''2.0'''
sigkill_timeout: '''2.0'''
parameters: '[{''use_sim_time'': PythonExpression(["''", mode, "'' == ''sim''"]),
  ''save_file'': LaunchConfiguration(''waypoints_file'')}]'
```

- `src/antbot_navigation/launch/waypoint_navigation.launch.py:71`

```yaml
package: '''robotcar_navigation'''
executable: '''wp_set_pose'''
name: '''wp_set_pose'''
output: '''screen'''
parameters: '[{''use_sim_time'': PythonExpression(["''", mode, "'' == ''sim''"])}]'
```

- `src/antbot_navigation/launch/waypoint_navigation.launch.py:80`

```yaml
package: '''robotcar_navigation'''
executable: '''wp_saver'''
name: '''wp_saver'''
output: '''screen'''
emulate_tty: 'True'
condition: IfCondition(start_waypoint_saver)
parameters: '[{''use_sim_time'': PythonExpression(["''", mode, "'' == ''sim''"])}]'
```

- `src/antbot_navigation/launch/waypoint_navigation.launch.py:91`

```yaml
package: '''rviz2'''
executable: '''rviz2'''
name: '''rviz2_waypoints'''
output: '''screen'''
condition: IfCondition(use_rviz)
arguments: '[''-d'', LaunchConfiguration(''rviz_config'')]'
parameters: '[{''use_sim_time'': PythonExpression(["''", mode, "'' == ''sim''"])}]'
```

- `src/antbot_navigation/launch/waypoint_navigation.launch.py:102`

```yaml
package: '''robotcar_navigation'''
executable: '''keepout_zone_manager.py'''
name: '''keepout_zone_manager'''
output: '''screen'''
parameters: '[{''use_sim_time'': PythonExpression(["''", mode, "'' == ''sim''"]),
  ''keepout_file'': LaunchConfiguration(''keepout_file'')}]'
```

- `src/antbot_navigation/launch/waypoint_navigation.launch.py:112`

```yaml
package: '''robotcar_navigation'''
executable: '''speed_zone_manager.py'''
name: '''speed_zone_manager'''
output: '''screen'''
parameters: '[{''use_sim_time'': PythonExpression(["''", mode, "'' == ''sim''"]),
  ''speed_zone_file'': LaunchConfiguration(''speed_zone_file'')}]'
```

- `src/antbot_navigation/launch/waypoint_navigation.launch.py:122`

```yaml
package: '''robotcar_navigation'''
executable: '''robot_intent_monitor.py'''
name: '''robot_intent_monitor'''
output: '''screen'''
parameters: '[{''use_sim_time'': PythonExpression(["''", mode, "'' == ''sim''"])}]'
```

- `src/antbot_navigation/launch/waypoint_navigation.launch.py:131`

```yaml
package: '''robotcar_navigation'''
executable: '''robot_awareness_monitor.py'''
name: '''robot_awareness_monitor'''
output: '''screen'''
parameters: '[{''use_sim_time'': PythonExpression(["''", mode, "'' == ''sim''"]),
  ''stuck_history_file'': LaunchConfiguration(''stuck_history_file'')}]'
```

- `src/antbot_navigation/launch/offline_waypoint_editor.launch.py:81`

```yaml
package: '''nav2_map_server'''
executable: '''map_server'''
name: '''map_server'''
output: '''screen'''
parameters: '[{''yaml_filename'': map_file, ''use_sim_time'': False}]'
```

- `src/antbot_navigation/launch/offline_waypoint_editor.launch.py:94`

```yaml
package: '''nav2_lifecycle_manager'''
executable: '''lifecycle_manager'''
name: '''lifecycle_manager_offline_map'''
output: '''screen'''
parameters: '[{''autostart'': True, ''node_names'': [''map_server''], ''use_sim_time'':
  False}]'
```

- `src/antbot_navigation/launch/offline_waypoint_editor.launch.py:107`

```yaml
package: '''robotcar_navigation'''
executable: '''wp_edit_node'''
name: '''wp_edit_node'''
output: '''screen'''
parameters: '[{''load'': waypoint_file, ''use_sim_time'': False}]'
```

- `src/antbot_navigation/launch/offline_waypoint_editor.launch.py:117`

```yaml
package: '''robotcar_navigation'''
executable: '''keepout_zone_manager.py'''
name: '''keepout_zone_manager'''
output: '''screen'''
parameters: '[{''keepout_file'': LaunchConfiguration(''keepout_file''), ''use_sim_time'':
  False}]'
```

- `src/antbot_navigation/launch/offline_waypoint_editor.launch.py:127`

```yaml
package: '''robotcar_navigation'''
executable: '''speed_zone_manager.py'''
name: '''speed_zone_manager'''
output: '''screen'''
parameters: '[{''speed_zone_file'': LaunchConfiguration(''speed_zone_file''), ''use_sim_time'':
  False}]'
```

- `src/antbot_navigation/launch/offline_waypoint_editor.launch.py:137`

```yaml
package: '''antbot_dual_lidar'''
executable: '''offline_pointcloud_publisher'''
name: '''antbot_offline_pointcloud_publisher'''
output: '''screen'''
condition: IfCondition(show_3d_cloud)
parameters: '[{''pointcloud_path'': LaunchConfiguration(''pointcloud_path''), ''metadata_path'':
  LaunchConfiguration(''pointcloud_metadata''), ''publish_topic'': LaunchConfiguration(''pointcloud_topic''),
  ''target_frame'': LaunchConfiguration(''pointcloud_target_frame''), ''publish_rate'':
  LaunchConfiguration(''pointcloud_publish_rate''), ''use_sim_time'': False}]'
```

- `src/antbot_navigation/launch/offline_waypoint_editor.launch.py:152`

```yaml
package: '''antbot_rgbd_dataset'''
executable: '''offline_preview_publisher'''
name: '''antbot_rgbd_offline_cloud_publisher'''
output: '''screen'''
condition: IfCondition(show_rgbd_cloud)
parameters: '[{''preview_path'': LaunchConfiguration(''rgbd_preview_path''), ''publish_topic'':
  LaunchConfiguration(''rgbd_cloud_topic''), ''publish_rate'': 0.5, ''use_sim_time'':
  False}]'
```

- `src/antbot_navigation/launch/offline_waypoint_editor.launch.py:165`

```yaml
package: '''robot_state_publisher'''
executable: '''robot_state_publisher'''
name: '''antbot_robot_state_publisher_offline_editor'''
output: '''screen'''
condition: IfCondition(show_3d_cloud)
parameters: '[{''robot_description'': robot_description, ''use_sim_time'': False}]'
remappings: '[(''robot_description'', ''/antbot/robot_description'')]'
```

- `src/antbot_navigation/launch/offline_waypoint_editor.launch.py:179`

```yaml
package: '''joint_state_publisher'''
executable: '''joint_state_publisher'''
name: '''joint_state_publisher_offline_editor'''
output: '''screen'''
condition: IfCondition(show_3d_cloud)
parameters: '[{''robot_description'': robot_description, ''use_sim_time'': False}]'
remappings: '[(''robot_description'', ''/antbot/robot_description'')]'
```

- `src/antbot_navigation/launch/offline_waypoint_editor.launch.py:193`

```yaml
package: '''tf2_ros'''
executable: '''static_transform_publisher'''
name: '''offline_map_to_robot'''
output: '''screen'''
condition: IfCondition(show_3d_cloud)
arguments: '[''--x'', ''0'', ''--y'', ''0'', ''--z'', ''0'', ''--roll'', ''0'', ''--pitch'',
  ''0'', ''--yaw'', ''0'', ''--frame-id'', ''map'', ''--child-frame-id'', ''base_link'']'
```

- `src/antbot_navigation/launch/offline_waypoint_editor.launch.py:208`

```yaml
package: '''rviz2'''
executable: '''rviz2'''
name: '''rviz2_waypoint_editor'''
output: '''screen'''
condition: IfCondition(show_3d_cloud)
arguments: '[''-d'', LaunchConfiguration(''rviz_3d_config'')]'
parameters: '[{''use_sim_time'': False}]'
```

- `src/antbot_navigation/launch/offline_waypoint_editor.launch.py:218`

```yaml
package: '''rviz2'''
executable: '''rviz2'''
name: '''rviz2_waypoint_editor'''
output: '''screen'''
condition: UnlessCondition(show_3d_cloud)
arguments: '[''-d'', LaunchConfiguration(''rviz_config'')]'
parameters: '[{''use_sim_time'': False}]'
```

- `src/antbot_navigation/launch/slam.launch.py:77`

```yaml
condition: IfCondition(PythonExpression(["'", mode, "' == 'real'"]))
package: '''antbot_navigation'''
executable: '''scan_fix_relay.py'''
name: '''scan_fix_relay'''
output: '''screen'''
parameters: '[{''input_topic'': ''/scan_0'', ''output_topic'': ''/scan_0_fixed''}]'
```

- `src/antbot_navigation/launch/slam.launch.py:95`

```yaml
package: '''slam_toolbox'''
executable: '''async_slam_toolbox_node'''
name: '''slam_toolbox'''
output: '''screen'''
parameters: '[slam_params_file, {''use_sim_time'': use_sim_time, ''scan_topic'': scan_topic}]'
```

- `src/antbot_navigation/launch/localization.launch.py:59`

```yaml
package: '''nav2_map_server'''
executable: '''map_server'''
name: '''map_server'''
output: '''screen'''
parameters: '[nav2_params_file, {''yaml_filename'': map_yaml, ''use_sim_time'': use_sim_time}]'
```

- `src/antbot_navigation/launch/localization.launch.py:72`

```yaml
package: '''nav2_amcl'''
executable: '''amcl'''
name: '''amcl'''
output: '''screen'''
parameters: '[nav2_params_file, {''use_sim_time'': use_sim_time}]'
```

- `src/antbot_navigation/launch/localization.launch.py:80`

```yaml
package: '''nav2_lifecycle_manager'''
executable: '''lifecycle_manager'''
name: '''lifecycle_manager_localization'''
output: '''screen'''
parameters: '[nav2_params_file, {''use_sim_time'': use_sim_time}]'
```

- `src/antbot_navigation/launch/isaac_mapping.launch.py:42`

```yaml
package: '''slam_toolbox'''
executable: '''async_slam_toolbox_node'''
name: '''slam_toolbox'''
output: '''screen'''
parameters: '[params_file, {''use_sim_time'': True, ''scan_topic'': ''/scan_0''}]'
```

- `src/antbot_navigation/launch/isaac_mapping.launch.py:58`

```yaml
package: '''nav2_lifecycle_manager'''
executable: '''lifecycle_manager'''
name: '''lifecycle_manager_slam'''
output: '''screen'''
parameters: '[{''autostart'': True, ''node_names'': [''slam_toolbox''], ''use_sim_time'':
  True}]'
```

- `src/antbot_navigation/launch/isaac_mapping.launch.py:71`

```yaml
package: '''antbot_dual_lidar'''
executable: '''cloud_preprocessor'''
name: '''antbot_dual_lidar_preprocessor'''
output: '''screen'''
condition: IfCondition(enable_3d_mapping)
parameters: '[lidar_params, {''use_sim_time'': True, ''lidar_profile'': ''mapping'',
  ''target_frame'': ''base_link'', ''front_left.input_topic'': ''/antbot/lidar/front_left/points_raw_native'',
  ''rear_right.input_topic'': ''/antbot/lidar/rear_right/points_raw_native''}]'
```

- `src/antbot_navigation/launch/isaac_mapping.launch.py:90`

```yaml
package: '''antbot_dual_lidar'''
executable: '''fixed_frame_mapper'''
name: '''antbot_fixed_frame_mapper'''
output: '''screen'''
condition: IfCondition(enable_3d_mapping)
parameters: '[{''use_sim_time'': True, ''fixed_frame'': ''odom''}]'
```

- `src/antbot_navigation/launch/isaac_mapping.launch.py:101`

```yaml
package: '''rviz2'''
executable: '''rviz2'''
name: '''rviz2_mapping'''
output: '''screen'''
condition: IfCondition(LaunchConfiguration('use_rviz'))
arguments: '[''-d'', LaunchConfiguration(''rviz_config'')]'
parameters: '[{''use_sim_time'': True}]'
```

- `src/antbot_description/launch/description.launch.py:67`

```yaml
package: '''robot_state_publisher'''
executable: '''robot_state_publisher'''
name: '''antbot_robot_state_publisher'''
output: '''screen'''
parameters: '[{''use_sim_time'': use_sim_time}, robot_description]'
remappings: '[(''robot_description'', ''/antbot/robot_description'')]'
```

- `src/antbot_description/launch/description.launch.py:76`

```yaml
package: '''joint_state_publisher'''
executable: '''joint_state_publisher'''
name: '''joint_state_publisher'''
output: '''screen'''
parameters: '[{''use_sim_time'': use_sim_time}, robot_description]'
condition: IfCondition(use_joint_state_publisher)
```

- `src/antbot_description/launch/description.launch.py:85`

```yaml
package: '''joint_state_publisher_gui'''
executable: '''joint_state_publisher_gui'''
name: '''joint_state_publisher_gui'''
output: '''screen'''
parameters: '[{''use_sim_time'': use_sim_time}, robot_description]'
condition: IfCondition(use_joint_state_publisher_gui)
```

- `src/antbot_description/launch/description.launch.py:95`

```yaml
package: '''rviz2'''
executable: '''rviz2'''
name: '''rviz2'''
output: '''screen'''
arguments: '[''-d'', rviz_config_path]'
parameters: '[{''use_sim_time'': use_sim_time}]'
condition: IfCondition(use_rviz)
```

## C++ Node 类清单

| Class | Source |
|---|---|
| RvizVisualToolsDemo | `dual_arm_ws/src/rviz_visual_tools/src/rviz_visual_tools_demo.cpp:54` |
| IMarkerSimpleDemo | `dual_arm_ws/src/rviz_visual_tools/src/imarker_simple_demo.cpp:43` |
| RemoteReciever | `dual_arm_ws/src/rviz_visual_tools/include/rviz_visual_tools/remote_reciever.hpp:40` |
| TopicStatistics | `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/tools/topic_statistics.hpp:17` |
| MultiCameraSubscriber | `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/tools/multi_save_rgbir.cpp:21` |
| ObBenchmark | `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/tools/ob_benchmark.cpp:14` |
| StartBenchmark | `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/tools/start_benchmark.cpp:14` |
| FrameLatencyNode | `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/tools/frame_latency.hpp:34` |
| CameraExampleNode | `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/examples/Gemini_435Le_example_node/camera_example_node.cpp:12` |
| ImageSyncNode | `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/examples/multi_camera_time_sync/image_sync_example_node.cpp:28` |
| OBCameraNodeDriver | `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/include/orbbec_camera/ob_camera_node_driver.h:37` |
| ImuNode | `src/antbot_imu/include/antbot_imu/imu_node.hpp:37` |
| CameraNode | `src/antbot_camera/src/camera_node.cpp:35` |
| CameraTestNode | `src/antbot_camera/src/camera_test_node.cpp:33` |
| WaypointSetPoseNode | `src/robotcar_navigation/src/wp_set_pose.cpp:9` |
| WaypointEditNode | `src/robotcar_navigation/src/wp_edit_node.cpp:38` |
| DemoMapTool | `src/robotcar_navigation/src/demo_map_tool.cpp:8` |
| WaypointManagerNode | `src/robotcar_navigation/include/robotcar_navigation/waypoint_manager_core.hpp:25` |
| VanjeeLidarSdkNode | `src/vanjee_lidar_sdk/src/source/source_ros_msg_delegate.hpp:44` |

## 表驱动服务补充

ArmServices同一create_service循环展开的实际当前namespace接口如下（不是一个服务）：

- `/rebotarm/enable` — `std_srvs/srv/Trigger`
- `/rebotarm/disable` — `std_srvs/srv/Trigger`
- `/rebotarm/safe_home` — `std_srvs/srv/Trigger`
- `/rebotarm/gravity_compensation/start` — `std_srvs/srv/Trigger`
- `/rebotarm/gravity_compensation/stop` — `std_srvs/srv/Trigger`
- `/rebotarm/set_zero` — `rebotarm_msgs/srv/SetZero`
- `/rebotarm/move_to_pose_ik` — `rebotarm_msgs/srv/MoveToPoseIK`
- `/rebotarm/gripper/set` — `rebotarm_msgs/srv/SetGripper`
- `/rebotarm/gripper/open` — `rebotarm_msgs/srv/GripperCommand`
- `/rebotarm/gripper/close` — `rebotarm_msgs/srv/GripperCommand`

这些服务包括safe_home、set_zero等主动硬件接口，不能作为普通只读诊断调用。

## C++ Source 对象依赖/参数/故障证据摘要

仅记录源码命中，频率表达式和动态参数未运行求值；实际设备/启动依赖及未命中故障处理均UNKNOWN / NEED_CONFIRMATION。

### `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/examples/Gemini_435Le_example_node/camera_example_node.cpp`

Package：`orbbec_camera`；Executable：见launch Node与CMake目标（插件嵌入RViz）；控制对象由上述接口调用判定。

```text
29: RCLCPP_ERROR(this->get_logger(), "SetUserCalibParams service not available");
59: RCLCPP_ERROR(this->get_logger(), "Failed to call SetUserCalibParams");
68: RCLCPP_ERROR(this->get_logger(), "GetUserCalibParams service not available");
76: RCLCPP_ERROR(this->get_logger(), "Failed to call GetUserCalibParams");
109: RCLCPP_ERROR(this->get_logger(), "GetDeviceInfo service not available");
117: RCLCPP_ERROR(this->get_logger(), "Failed to call GetDeviceInfo");
123: RCLCPP_ERROR(this->get_logger(), "GetDeviceInfo failed: %s", res->message.c_str());
150: RCLCPP_ERROR(this->get_logger(), "SetColorAERoi service not available");
160: RCLCPP_ERROR(this->get_logger(), "Failed to call SetColorAERoi");
168: RCLCPP_ERROR(this->get_logger(), "SetColorAERoi failed: %s", res->message.c_str());
175: RCLCPP_ERROR(this->get_logger(), "SetStreamsEnable service not available");
185: RCLCPP_ERROR(this->get_logger(), "Failed to call SetStreamsEnable");
```

### `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/examples/multi_camera_time_sync/image_sync_example_node.cpp`

Package：`orbbec_camera`；Executable：见launch Node与CMake目标（插件嵌入RViz）；控制对象由上述接口调用判定。

```text
172: RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
```

### `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts/service_benchmark_node.cpp`

Package：`orbbec_camera`；Executable：见launch Node与CMake目标（插件嵌入RViz）；控制对象由上述接口调用判定。

```text
102: RCLCPP_ERROR(nh_->get_logger(), "Unsupported service type: %s", service_type_.c_str());
157: RCLCPP_ERROR(nh_->get_logger(), "Service %s not available", service_name_.c_str());
194: RCLCPP_ERROR(nh_->get_logger(), "Exception calling service %s: %s", service_name_.c_str(),
317: RCLCPP_ERROR(nh_->get_logger(), "No services found in YAML config.");
371: nh->declare_parameter("yaml_file", "");
372: nh->declare_parameter("csv_file", "multi_service_results_log_cpp.csv");
373: nh->declare_parameter("service_name", "");
374: nh->declare_parameter("service_type", "");
375: nh->declare_parameter("request_data", "");
376: nh->declare_parameter("count", 10);
387: RCLCPP_ERROR(nh->get_logger(),
396: RCLCPP_ERROR(nh->get_logger(), "Failed to parse request_data: %s", e.what());
```

### `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/d2c_viewer.cpp`

Package：`orbbec_camera`；Executable：见launch Node与CMake目标（插件嵌入RViz）；控制对象由上述接口调用判定。

```text
81: RCLCPP_ERROR(logger_, "rgb and depth image size not match(%d, %d) vs (%d, %d)", rgb_msg->width,
101: RCLCPP_ERROR(logger_, "d2c_viewer publishing failed");
```

### `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/image_publisher.cpp`

Package：`orbbec_camera`；Executable：见launch Node与CMake目标（插件嵌入RViz）；控制对象由上述接口调用判定。

```text

```

### `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ob_camera_node.cpp`

Package：`orbbec_camera`；Executable：见launch Node与CMake目标（插件嵌入RViz）；控制对象由上述接口调用判定。

```text
487: RCLCPP_ERROR_STREAM(logger_, "Failed to set parameter: " << param_name << ". " << ex.what());
647: RCLCPP_ERROR_STREAM(
650: RCLCPP_ERROR_STREAM(logger_, "Failed to load device preset: " << e.what());
652: RCLCPP_ERROR_STREAM(logger_, "Failed to load device preset");
744: RCLCPP_ERROR_STREAM(logger_,
798: RCLCPP_ERROR(logger_, "ldp power level value is out of range[%d,%d], please check the value",
837: software_trigger_timer_ = node_->create_wall_timer(software_trigger_period_, [this]() {
871: RCLCPP_ERROR_STREAM(
988: RCLCPP_ERROR(logger_, "color exposure value is out of range[%d,%d], please check the value",
1000: RCLCPP_ERROR(logger_, "color gain value is out of range[%d,%d], please check the value",
1029: RCLCPP_ERROR(logger_,
1043: RCLCPP_ERROR(logger_,
1057: RCLCPP_ERROR(logger_,
1070: RCLCPP_ERROR(logger_, "color brightness value is out of range[%d,%d], please check the value",
1082: RCLCPP_ERROR(logger_,
1095: RCLCPP_ERROR(logger_, "color sharpness value is out of range[%d,%d], please check the value",
1107: RCLCPP_ERROR(logger_, "color gamm value is out of range[%d,%d], please check the value",
1119: RCLCPP_ERROR(logger_, "color saturation value is out of range[%d,%d], please check the value",
1131: RCLCPP_ERROR(logger_, "color contrast value is out of range[%d,%d], please check the value",
1143: RCLCPP_ERROR(logger_, "color hue value is out of range[%d,%d], please check the value",
1190: RCLCPP_ERROR(logger_, "depth exposure value is out of range[%d,%d], please check the value",
1202: RCLCPP_ERROR(logger_, "depth gain value is out of range[%d,%d], please check the value",
1230: RCLCPP_ERROR(logger_, "depth brightness value is out of range[%d,%d], please check the value",
1243: RCLCPP_ERROR(logger_,
1257: RCLCPP_ERROR(logger_, "IR brightness value is out of range[%d,%d], please check the value",
1269: RCLCPP_ERROR(logger_, "ir exposure value is out of range[%d,%d], please check the value",
1280: RCLCPP_ERROR(logger_, "ir gain value is out of range[%d,%d], please check the value",
1304: RCLCPP_ERROR(logger_,
1325: RCLCPP_ERROR(logger_,
1351: RCLCPP_ERROR(logger_, "disparity range mode does not support this setting");
1386: RCLCPP_ERROR(logger_, "exposure range mode does not support this setting");
1429: RCLCPP_ERROR(logger_, "intra camera sync reference does not support this setting");
1498: RCLCPP_ERROR_STREAM(logger_, "Color Decimation filter scale value is out of range "
1529: RCLCPP_ERROR_STREAM(logger_, "Left Color Decimation filter scale value is out of range "
1560: RCLCPP_ERROR_STREAM(logger_, "Right Color Decimation filter scale value is out of range "
1581: RCLCPP_ERROR_STREAM(logger_, "Color Decimation filter scale value is out of range "
1699: RCLCPP_ERROR_STREAM(logger_, "Decimation filter scale value is out of range "
1822: RCLCPP_ERROR_STREAM(logger_, "Decimation filter scale value is out of range "
1901: throw std::runtime_error("Failed to get profile " + std::to_string(i));
1905: throw std::runtime_error("Failed cast profile to VideoStreamProfile");
1946: RCLCPP_ERROR_STREAM(
1948: RCLCPP_ERROR_STREAM(
1953: RCLCPP_ERROR(logger_,
1958: RCLCPP_ERROR(logger_, "Failed to configure the requested stream profile, exiting.");
1974: RCLCPP_ERROR_STREAM(logger_, "No default profile found, disabling stream "
2109: RCLCPP_ERROR_STREAM(logger_,
2118: RCLCPP_ERROR_STREAM(logger_, "Failed to start pipeline");
2119: throw std::runtime_error("Failed to start pipeline");
2222: RCLCPP_ERROR_STREAM(
2226: RCLCPP_ERROR_STREAM(
2301: "Device or pipeline not available during stop - likely disconnected");
2304: RCLCPP_ERROR_STREAM(logger_,
2307: RCLCPP_ERROR_STREAM(logger_, "Failed to stop pipeline");
2323: RCLCPP_ERROR_STREAM(
2326: RCLCPP_ERROR_STREAM(logger_, "Failed to stop imu pipeline");
2337: RCLCPP_ERROR_STREAM(logger_, "Failed to stop "
2835: RCLCPP_ERROR_STREAM(logger_,
2837: throw std::runtime_error(orbbec_camera::formatObErrorWithStatus(e));
2839: RCLCPP_ERROR_STREAM(logger_, "Failed to setup topics: " << e.what());
2840: throw std::runtime_error(e.what());
2842: RCLCPP_ERROR(logger_, "Failed to setup topics");
2843: throw std::runtime_error("Failed to setup topics");
2852: "Device disconnected or shutting down");
2856: // Try to acquire device lock with timeout to avoid blocking during shutdown
2897: RCLCPP_ERROR_STREAM(
2902: RCLCPP_ERROR_STREAM(logger_, "Failed to TemperatureUpdate2: " << e.what());
2905: RCLCPP_ERROR(logger_, "Failed to TemperatureUpdate3: Device is deactivated/disconnected!");
2922: node_->create_wall_timer(std::chrono::seconds(int(diagnostic_period_)), [this]() {
2930: // Try to acquire device lock with timeout to avoid blocking during shutdown
2957: << " - Device may be disconnected");
2974: RCLCPP_ERROR_STREAM(logger_, "Failed to setup diagnostic updater: "
2977: RCLCPP_ERROR_STREAM(logger_, "Failed to setup diagnostic updater: " << e.what());
2979: RCLCPP_ERROR(logger_, "Failed to TemperatureUpdate");
3217: RCLCPP_ERROR_STREAM(logger_, orbbec_camera::formatObErrorWithStatus(e));
3219: RCLCPP_ERROR_STREAM(logger_, e.what());
3221: RCLCPP_ERROR_STREAM(logger_, "publishPointCloud with unknown error");
3267: RCLCPP_ERROR_STREAM(logger_, "depth frame is null");
3271: RCLCPP_ERROR_STREAM(logger_, "pipeline is null in publishDepthPointCloud");
3276: RCLCPP_ERROR_STREAM(logger_, "device is null in publishDepthPointCloud");
3281: RCLCPP_ERROR_STREAM(logger_, "device_info is null in publishDepthPointCloud");
3294: RCLCPP_ERROR_STREAM(logger_, "Failed to process depth frame");
3360: RCLCPP_ERROR_STREAM(logger_, "Failed to save point cloud: " << e.what());
3392: RCLCPP_ERROR_STREAM(logger_, "pipeline is null in publishColoredPointCloud");
3397: RCLCPP_ERROR_STREAM(logger_, "device is null in publishColoredPointCloud");
3402: RCLCPP_ERROR_STREAM(logger_, "device_info is null in publishColoredPointCloud");
3416: RCLCPP_ERROR_STREAM(logger_, "Failed to process depth frame");
3493: RCLCPP_ERROR_STREAM(logger_, "Failed to save point cloud: " << e.what());
3495: RCLCPP_ERROR(logger_, "Failed to save point cloud");
3512: RCLCPP_ERROR_STREAM(logger_, "Right Ir filter process failed");
3530: RCLCPP_ERROR_STREAM(logger_, "Left Ir filter process failed");
3550: RCLCPP_ERROR_STREAM(logger_, "Color filter process failed");
3563: RCLCPP_ERROR_STREAM(logger_, "Left color filter process failed");
3576: RCLCPP_ERROR_STREAM(logger_, "Right color filter process failed");
3802: RCLCPP_ERROR(logger_, "Failed to align depth frame to color frame");
3888: RCLCPP_ERROR_STREAM(
3891: RCLCPP_ERROR_STREAM(logger_, "onNewFrameSetCallback error: " << e.what());
3893: RCLCPP_ERROR_STREAM(logger_, "onNewFrameSetCallback error: unknown error");
3996: RCLCPP_ERROR(logger_, "Unsupported color format: %d", frame->getFormat());
4004: RCLCPP_ERROR_STREAM(logger_,
4008: RCLCPP_ERROR_STREAM(logger_, "Format convert failed: " << e.what());
4011: RCLCPP_ERROR(logger_, "Format convert failed: unknown error");
4015: RCLCPP_ERROR_SKIPFIRST_THROTTLE(logger_, *(node_->get_clock()), 1000,
4100: RCLCPP_ERROR_STREAM(logger_, "Decode frame failed");
4112: RCLCPP_ERROR_STREAM(logger_, "Failed to convert frame to video frame");
4142: RCLCPP_ERROR_STREAM(logger_,
4197: RCLCPP_ERROR(logger_, "Unsupported frame type: %d", frame->getType());
4201: RCLCPP_ERROR(logger_, "Failed to convert frame to video frame");
4209: RCLCPP_ERROR_STREAM(logger_, "device is null in onNewFrameCallback");
4214: RCLCPP_ERROR_STREAM(logger_, "device_info is null in onNewFrameCallback");
4275: RCLCPP_ERROR(logger_, "color frame is not decoded");
4279: RCLCPP_ERROR(logger_, "left color frame is not decoded");
4283: RCLCPP_ERROR(logger_, "right color frame is not decoded");
4431: RCLCPP_ERROR_STREAM(logger_, "Failed to open file: " << filename);
4445: RCLCPP_ERROR_STREAM(logger_, "Unsupported stream type: " << stream_index.first);
4459: RCLCPP_ERROR_STREAM(logger_, "stream Accel Gryo publisher not initialized");
4508: RCLCPP_ERROR_STREAM(logger_,
4542: RCLCPP_ERROR(logger_, "Unsupported IMU frame type");
4667: RCLCPP_ERROR_STREAM(logger_, "Failed to get base stream profile");
4682: RCLCPP_ERROR_STREAM(logger_, "Failed to get " << stream_name_[stream_index] << " extrinsic: "
4728: RCLCPP_ERROR_STREAM(logger_, "Failed to get " << frame_id << " extrinsic: "
4744: RCLCPP_ERROR_STREAM(logger_, "Failed to get " << frame_id << " extrinsic: "
4759: RCLCPP_ERROR_STREAM(logger_, "Failed to get " << frame_id << " extrinsic: "
4774: RCLCPP_ERROR_STREAM(logger_, "Failed to get " << frame_id << " extrinsic: "
4790: RCLCPP_ERROR_STREAM(logger_, "Failed to get " << frame_id << " extrinsic: "
4805: RCLCPP_ERROR_STREAM(logger_, "Failed to get " << frame_id << " extrinsic: "
4820: RCLCPP_ERROR_STREAM(logger_, "Failed to get " << frame_id << " extrinsic: "
5137: RCLCPP_ERROR_STREAM(logger_, "Decimation filter scale value is out of range "
5309: RCLCPP_ERROR_STREAM(logger_,
5314: RCLCPP_ERROR_STREAM(logger_, "Failed to set filter: " << e.what());
5318: RCLCPP_ERROR_STREAM(logger_, "unknown error");
```

### `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ob_camera_node_driver.cpp`

Package：`orbbec_camera`；Executable：见launch Node与CMake目标（插件嵌入RViz）；控制对象由上述接口调用判定。

```text
230: g_camera_name = declare_parameter<std::string>("camera_name", g_camera_name);
231: auto log_level_str = declare_parameter<std::string>("log_level", "none");
234: auto log_file_name = declare_parameter<std::string>("log_file_name", "");
256: force_ip_enable_ = declare_parameter<bool>("force_ip_enable", false);
257: force_ip_mac_ = declare_parameter<std::string>("force_ip_mac", "");
258: force_ip_address_ = declare_parameter<std::string>("force_ip_address", "192.168.1.10");
259: force_ip_subnet_mask_ = declare_parameter<std::string>("force_ip_subnet_mask", "255.255.255.0");
260: force_ip_gateway_ = declare_parameter<std::string>("force_ip_gateway", "192.168.1.1");
268: device_type_ = declare_parameter<std::string>("device_type", "camera");
269: connection_delay_ = static_cast<int>(declare_parameter<int>("connection_delay", 100));
270: enable_sync_host_time_ = declare_parameter<bool>("enable_sync_host_time", true);
271: double time_sync_period = declare_parameter<double>("time_sync_period", 60.0);
273: upgrade_firmware_ = declare_parameter<std::string>("upgrade_firmware", "");
274: g_time_domain = declare_parameter<std::string>("time_domain", g_time_domain);
276: declare_parameter<std::string>("preset_firmware_path", preset_firmware_path_);
279: RCLCPP_ERROR_STREAM(logger_, "Failed to open shared memory " << ORB_DEFAULT_LOCK_NAME);
284: RCLCPP_ERROR_STREAM(logger_, "Failed to truncate shared memory " << ORB_DEFAULT_LOCK_NAME);
291: RCLCPP_ERROR_STREAM(logger_, "Failed to map shared memory " << ORB_DEFAULT_LOCK_NAME);
305: serial_number_ = declare_parameter<std::string>("serial_number", "");
306: device_num_ = static_cast<int>(declare_parameter<int>("device_num", 1));
307: usb_port_ = declare_parameter<std::string>("usb_port", "");
308: net_device_ip_ = declare_parameter<std::string>("net_device_ip", "");
309: net_device_port_ = static_cast<int>(declare_parameter<int>("net_device_port", 0));
310: enumerate_net_device_ = declare_parameter<bool>("enumerate_net_device", false);
311: uvc_backend_ = declare_parameter<std::string>("uvc_backend", "libuvc");
312: device_access_mode_str_ = declare_parameter<std::string>("device_access_mode", "Default");
331: onDeviceDisconnected(removed_list);
335: this->create_wall_timer(std::chrono::milliseconds(1000), [this]() { checkConnectTimer(); });
339: this->create_wall_timer(std::chrono::milliseconds(1000 / device_status_interval_hz),
382: void OBCameraNodeDriver::onDeviceDisconnected(const std::shared_ptr<ob::DeviceList> &device_list) {
387: RCLCPP_INFO_STREAM(logger_, "onDeviceDisconnected called");
392: "onDeviceDisconnected: device connection/initialization in progress, "
393: "ignoring disconnect event");
402: RCLCPP_DEBUG_STREAM(logger_, "onDeviceDisconnected: device already disconnected");
409: RCLCPP_INFO_STREAM(logger_, "device with " << uid << " disconnected");
412: "device with " << uid << " disconnected, notify reset device thread");
505: // Use a timeout to make the wait interruptible
506: auto timeout = std::chrono::milliseconds(1000);
508: lock, timeout, [this]() { return !is_alive_ || !rclcpp::ok() || reset_device_flag_; });
533: // Mark device as disconnected immediately to prevent other threads from accessing it
622: error_msg.find("disconnected") != std::string::npos ||
626: "Device communication error in %s at line %d: %s - Device may be disconnected",
630: RCLCPP_ERROR(logger_, "Error in %s at line %d: %s", __FUNCTION__, __LINE__,
634: RCLCPP_ERROR(logger_, "Exception in %s at line %d: %s", __FUNCTION__, __LINE__, e.what());
636: RCLCPP_ERROR(logger_, "Unknown exception in %s at line %d", __FUNCTION__, __LINE__);
651: error_msg.find("disconnected") != std::string::npos ||
655: "Device communication error in %s at line %d: %s - Device may be disconnected",
659: RCLCPP_ERROR(logger_, "Error in %s at line %d: %s", __FUNCTION__, __LINE__,
663: RCLCPP_ERROR(logger_, "Exception in %s at line %d: %s", __FUNCTION__, __LINE__, e.what());
665: RCLCPP_ERROR(logger_, "Unknown exception in %s at line %d", __FUNCTION__, __LINE__);
678: error_msg.find("disconnected") != std::string::npos ||
682: "Device communication error in %s at line %d: %s - Device may be disconnected",
686: RCLCPP_ERROR(logger_, "Error in %s at line %d: %s", __FUNCTION__, __LINE__,
690: RCLCPP_ERROR(logger_, "Exception in %s at line %d: %s", __FUNCTION__, __LINE__, e.what());
692: RCLCPP_ERROR(logger_, "Unknown exception in %s at line %d", __FUNCTION__, __LINE__);
717: error_msg.find("disconnected") != std::string::npos ||
721: "Device communication error in %s at line %d: %s - Device may be disconnected",
725: RCLCPP_ERROR(logger_, "Error in %s at line %d: %s", __FUNCTION__, __LINE__,
729: RCLCPP_ERROR(logger_, "Exception in %s at line %d: %s", __FUNCTION__, __LINE__, e.what());
731: RCLCPP_ERROR(logger_, "Unknown exception in %s at line %d", __FUNCTION__, __LINE__);
762: struct timespec timeout;
763: clock_gettime(CLOCK_REALTIME, &timeout);
764: timeout.tv_sec += 15;
766: int lock_result = pthread_mutex_timedlock(orb_device_lock_, &timeout);
799: RCLCPP_ERROR_STREAM(logger_, "Failed to reboot device: " << e.what());
801: RCLCPP_ERROR_STREAM(logger_, "Failed to reboot device: unknown error");
871: RCLCPP_ERROR_STREAM_THROTTLE(
875: RCLCPP_ERROR_STREAM_THROTTLE(logger_, *get_clock(), 5000,
878: RCLCPP_ERROR_STREAM_THROTTLE(logger_, *get_clock(), 5000, "Failed to get device info");
894: RCLCPP_ERROR_STREAM_THROTTLE(logger_, *get_clock(), 5000,
896: RCLCPP_ERROR_STREAM_THROTTLE(
902: RCLCPP_ERROR_STREAM_THROTTLE(
906: RCLCPP_ERROR_STREAM_THROTTLE(logger_, *get_clock(), 5000,
909: RCLCPP_ERROR_STREAM_THROTTLE(logger_, *get_clock(), 5000, "Failed to get device info");
937: RCLCPP_ERROR_STREAM_THROTTLE(
942: RCLCPP_ERROR_STREAM_THROTTLE(logger_, *get_clock(), 5000,
946: RCLCPP_ERROR_STREAM_THROTTLE(logger_, *get_clock(), 5000, "Failed to get device info");
982: RCLCPP_ERROR_STREAM(logger_, "Failed to initialize device (Attempt "
986: RCLCPP_ERROR_STREAM(logger_, "Failed to initialize device (Attempt " << retry_count + 1
990: RCLCPP_ERROR_STREAM(logger_, "Failed to initialize device (Attempt "
997: RCLCPP_ERROR_STREAM(logger_,
999: throw std::runtime_error("Device initialization failed after " + std::to_string(max_retries) +
1013: sync_host_time_timer_ = this->create_wall_timer(time_sync_period_, [this]() {
1066: // Safely log device information - these calls can throw if device disconnects
1188: RCLCPP_ERROR(logger_, "[ForceIP] Invalid IP: %s", force_ip_address_.c_str());
1192: RCLCPP_ERROR(logger_, "[ForceIP] Invalid Mask: %s", force_ip_subnet_mask_.c_str());
1196: RCLCPP_ERROR(logger_, "[ForceIP] Invalid Gateway: %s", force_ip_gateway_.c_str());
1214: RCLCPP_ERROR(logger_, "[ForceIP] MAC address is empty");
1224: RCLCPP_ERROR(logger_, "[ForceIP] Failed to apply config (SDK returned false)");
1227: RCLCPP_ERROR(logger_, "[ForceIP] ob::Error: %s",
1230: RCLCPP_ERROR(logger_, "[ForceIP] std::exception: %s", e.what());
1232: RCLCPP_ERROR(logger_, "[ForceIP] Unknown error");
1240: RCLCPP_ERROR_STREAM(logger_, "Invalid net device ip or port");
1258: RCLCPP_ERROR_STREAM(logger_, "Failed to connect to net device " << net_device_ip);
1264: RCLCPP_ERROR_STREAM(logger_, "Failed to initialize net device " << net_device_ip);
1267: RCLCPP_ERROR_STREAM(logger_, "Exception during net device initialization: " << e.what());
1270: RCLCPP_ERROR_STREAM(logger_, "Unknown exception during net device initialization");
1318: RCLCPP_ERROR_STREAM(logger_, "Failed to lock orb_device_lock_");
1325: RCLCPP_ERROR_STREAM(logger_, "Failed to lock orb_device_lock_");
1368: RCLCPP_ERROR_STREAM(
1372: RCLCPP_ERROR_STREAM(logger_, "Failed to initialize device " << e.what());
1375: RCLCPP_ERROR_STREAM(logger_, "Failed to initialize device");
1442: RCLCPP_ERROR_STREAM(logger_, "Failed to update Preset Firmware "
1445: RCLCPP_ERROR_STREAM(logger_, "Failed to update Preset Firmware " << e.what());
1447: RCLCPP_ERROR_STREAM(logger_, "Failed to update Preset Firmware");
1537: // The resetDevice thread will handle proper cleanup when device disconnects
```

### `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ob_lidar_node.cpp`

Package：`orbbec_camera`；Executable：见launch Node与CMake目标（插件嵌入RViz）；控制对象由上述接口调用判定。

```text
59: RCLCPP_ERROR_STREAM(logger_, "Failed to set parameter: " << param_name << ". " << ex.what());
97: RCLCPP_ERROR_STREAM(logger_,
99: throw std::runtime_error(orbbec_camera::formatObErrorWithStatus(e));
101: RCLCPP_ERROR_STREAM(logger_, "Failed to setup topics: " << e.what());
102: throw std::runtime_error(e.what());
104: RCLCPP_ERROR(logger_, "Failed to setup topics");
105: throw std::runtime_error("Failed to setup topics");
227: RCLCPP_ERROR(logger_,
241: RCLCPP_ERROR(logger_, "filter level value is out of range[%d,%d], please check the value",
255: RCLCPP_ERROR(logger_, "vertical fov value is out of range[%f,%f], please check the value",
276: throw std::runtime_error("Failed to get profile " + std::to_string(i));
280: throw std::runtime_error("Failed cast profile to LiDARStreamProfile");
299: RCLCPP_ERROR_STREAM(
301: RCLCPP_ERROR_STREAM(logger_, "Stream: " << magic_enum::enum_name(elem.first)
307: RCLCPP_ERROR(logger_, "Failed to configure the requested stream profile, exiting.");
323: RCLCPP_ERROR_STREAM(
438: RCLCPP_ERROR_STREAM(logger_,
445: RCLCPP_ERROR_STREAM(logger_, "Failed to start pipeline");
446: throw std::runtime_error("Failed to start pipeline");
492: RCLCPP_ERROR_STREAM(
510: RCLCPP_ERROR_STREAM(logger_,
513: RCLCPP_ERROR_STREAM(logger_, "Failed to stop pipeline");
531: RCLCPP_ERROR_STREAM(
534: RCLCPP_ERROR_STREAM(logger_, "Failed to stop IMU pipeline");
571: RCLCPP_ERROR_STREAM(logger_, "IMU publisher not initialized");
658: RCLCPP_ERROR_STREAM(
661: RCLCPP_ERROR_STREAM(logger_, "onNewFrameSetCallback error: " << e.what());
663: RCLCPP_ERROR_STREAM(logger_, "onNewFrameSetCallback error: unknown error");
1183: RCLCPP_ERROR_STREAM(logger_, "Failed to get base stream profile");
1198: //     RCLCPP_ERROR_STREAM(logger_, "Failed to get " << stream_name_[stream_index]
1264: RCLCPP_ERROR_STREAM(
```

### `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/src/ros_service.cpp`

Package：`orbbec_camera`；Executable：见launch Node与CMake目标（插件嵌入RViz）；控制对象由上述接口调用判定。

```text
370: RCLCPP_ERROR_STREAM(logger_, response->message);
565: RCLCPP_ERROR(logger_, "%s NOT a video stream", __FUNCTION__);
576: RCLCPP_ERROR(logger_, "%s unknown error %d", __FUNCTION__, __LINE__);
603: RCLCPP_ERROR(logger_, " %s NOT a video stream", __FUNCTION__);
640: RCLCPP_ERROR(logger_, "%s NOT a video stream", __FUNCTION__);
761: RCLCPP_ERROR(logger_, "%s NOT a video stream", __FUNCTION__);
882: RCLCPP_ERROR(logger_, "%s NOT a video stream", __FUNCTION__);
1030: RCLCPP_ERROR(logger_, " %s NOT a video stream", __FUNCTION__);
1121: RCLCPP_ERROR(logger_, " %s NOT a video stream", __FUNCTION__);
1166: RCLCPP_ERROR(logger_, " %s NOT a video stream", __FUNCTION__);
1211: RCLCPP_ERROR(logger_, " %s NOT a video stream", __FUNCTION__);
1277: RCLCPP_ERROR(logger_, "PTP clock sync property is not supported or not writable");
```

### `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/tools/frame_latency.cpp`

Package：`orbbec_camera`；Executable：见launch Node与CMake目标（插件嵌入RViz）；控制对象由上述接口调用判定。

```text
55: timer_ = this->create_wall_timer(1s, [this, topic_name = topic_name]() {
96: topic_name = this->declare_parameter("topic_name", topic_name);
97: topic_type = this->declare_parameter("topic_type", topic_type);
118: RCLCPP_ERROR_STREAM(logger_, "Specified message type '" << topic_type << "' is not supported");
```

### `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/tools/multi_save_rgbir.cpp`

Package：`orbbec_camera`；Executable：见launch Node与CMake目标（插件嵌入RViz）；控制对象由上述接口调用判定。

```text
55: RCLCPP_ERROR_STREAM(get_logger(), orbbec_camera::formatObErrorWithStatus(e));
57: RCLCPP_ERROR_STREAM(get_logger(), e.what());
59: RCLCPP_ERROR_STREAM(get_logger(), "unknown error");
96: RCLCPP_ERROR(this->get_logger(), "Failed to open JSON file.");
```

### `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/tools/ob_benchmark.cpp`

Package：`orbbec_camera`；Executable：见launch Node与CMake目标（插件嵌入RViz）；控制对象由上述接口调用判定。

```text
19: function_ = this->create_wall_timer(std::chrono::seconds(test_cycle_),
21: config_ = this->create_wall_timer(std::chrono::seconds(switch_cycle_),
52: RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to open JSON file.");
171: RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to create directory: " << dir_path.c_str());
195: RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to open file for writing.");
208: RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to run command to find PID.");
231: RCLCPP_ERROR_STREAM(this->get_logger(),
248: RCLCPP_ERROR_STREAM(this->get_logger(),
```

### `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/tools/start_benchmark.cpp`

Package：`orbbec_camera`；Executable：见launch Node与CMake目标（插件嵌入RViz）；控制对象由上述接口调用判定。

```text
87: RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to open JSON file.");
```

### `dual_arm_ws/src/OrbbecSDK_ROS2/orbbec_camera/tools/topic_statistics.hpp`

Package：`orbbec_camera`；Executable：见launch Node与CMake目标（插件嵌入RViz）；控制对象由上述接口调用判定。

```text
20: this->declare_parameter("image_topic", "/camera/color/image_raw");
21: this->declare_parameter("statistics_topic", "/statistics");
64: RCLCPP_ERROR(get_logger(), "Unable to open statistics.csv for writing");
121: RCLCPP_ERROR(get_logger(), "Unable to open statistics.csv for writing");
```

### `dual_arm_ws/src/moveit_calibration/moveit_calibration_gui/handeye_calibration_rviz_plugin/src/handeye_target_widget.cpp`

Package：`moveit_calibration_gui`；Executable：见launch Node与CMake目标（插件嵌入RViz）；控制对象由上述接口调用判定。

```text
212: RCLCPP_ERROR_STREAM(node_->get_logger(),
396: RCLCPP_ERROR_STREAM(node_->get_logger(), "Image msg has empty frame_id.");
404: RCLCPP_ERROR_STREAM(node_->get_logger(), "Image msg has empty data.");
447: RCLCPP_ERROR(node_->get_logger(), "%s", error_message.c_str());
454: RCLCPP_ERROR(node_->get_logger(), "%s", error_message.c_str());
474: RCLCPP_ERROR(node_->get_logger(), "%s", error_message.c_str());
540: RCLCPP_ERROR_STREAM(node_->get_logger(), "Error OpenCV saving image.");
557: RCLCPP_ERROR_STREAM(node_->get_logger(),
```

### `dual_arm_ws/src/moveit_visual_tools/include/moveit_visual_tools/moveit_visual_tools.h`

Package：`moveit_visual_tools`；Executable：见launch Node与CMake目标（插件嵌入RViz）；控制对象由上述接口调用判定。

```text

```

### `dual_arm_ws/src/moveit_visual_tools/src/moveit_visual_tools.cpp`

Package：`moveit_visual_tools`；Executable：见launch Node与CMake目标（插件嵌入RViz）；控制对象由上述接口调用判定。

```text
143: RCLCPP_ERROR_STREAM(LOGGER, "Planning scene not configured");
265: RCLCPP_ERROR_STREAM(LOGGER, "Unable to find joint model group with address" << ee_jmg);
278: RCLCPP_ERROR_STREAM(LOGGER, "The number of joint positions given ("
313: RCLCPP_ERROR_STREAM(LOGGER,
412: RCLCPP_ERROR_STREAM(LOGGER, "Unable to publish EE marker, unable to load EE markers");
865: RCLCPP_ERROR_STREAM(LOGGER, "Unable to create mesh shape message from resource " << mesh_path);
1190: RCLCPP_ERROR_STREAM(LOGGER, "Could not find joint model group " << planning_group);
1338: RCLCPP_FATAL_STREAM(LOGGER, "arm_jmg is NULL");
1367: RCLCPP_FATAL_STREAM(LOGGER, "ee_parent_link is NULL");
1382: RCLCPP_ERROR_STREAM(LOGGER, "NAN DETECTED AT TRAJECTORY POINT i=" << i);
1402: RCLCPP_FATAL_STREAM(LOGGER, "arm_jmg is NULL");
1409: RCLCPP_ERROR_STREAM(LOGGER, "Unable to get end effector tips from jmg");
1436: RCLCPP_FATAL_STREAM(LOGGER, "arm_jmg is NULL");
1443: RCLCPP_ERROR_STREAM(LOGGER, "Unable to get end effector tips from jmg");
```

### `dual_arm_ws/src/piperh_motion_rviz/src/motion_preset_panel.cpp`

Package：`piperh_motion_rviz`；Executable：见launch Node与CMake目标（插件嵌入RViz）；控制对象由上述接口调用判定。

```text
291: node_->declare_parameter<std::string>(name, fallback);
295: node_->declare_parameter<bool>("motion_preset.dual_arm", false);
298: static_cast<int>(node_->declare_parameter<int64_t>(
```

### `dual_arm_ws/src/rebotarm_demo_rviz/src/demo_panel.cpp`

Package：`rebotarm_demo_rviz`；Executable：见launch Node与CMake目标（插件嵌入RViz）；控制对象由上述接口调用判定。

```text
1459: teach_trace_width_timer_, &QTimer::timeout,
1553: connect(readiness_timer_, &QTimer::timeout, this, &DemoPanel::updateReadiness);
2106: node_->declare_parameter<bool>("rebot_demo.integrate_motion_planning", true);
2959: "XY ≤300 mm, Z ≤200 mm, Cartesian speed ≤5 mm/s, 90 s timeout.\n"
7543: "with automatic pen lifts between disconnected strokes. A side-on 3D view compresses "
7647: "disconnected transitions; saving keeps the full list.")));
```

### `dual_arm_ws/src/rebotarm_demo_rviz/src/dual_arm_selector_panel.cpp`

Package：`rebotarm_demo_rviz`；Executable：见launch Node与CMake目标（插件嵌入RViz）；控制对象由上述接口调用判定。

```text
304: node_->declare_parameter<bool>("dual_arm.offline_preview", false);
412: connect(feedback_watchdog, &QTimer::timeout, this, [this]() {
711: // disconnected hardware never produces RViz's white error material.
```

### `dual_arm_ws/src/rviz_visual_tools/include/rviz_visual_tools/remote_reciever.hpp`

Package：`rviz_visual_tools`；Executable：见launch Node与CMake目标（插件嵌入RViz）；控制对象由上述接口调用判定。

```text

```

### `dual_arm_ws/src/rviz_visual_tools/src/remote_control.cpp`

Package：`rviz_visual_tools`；Executable：见launch Node与CMake目标（插件嵌入RViz）；控制对象由上述接口调用判定。

```text
87: RCLCPP_ERROR(logger_, "Unknown input button");
```

### `dual_arm_ws/src/rviz_visual_tools/src/rviz_visual_tools.cpp`

Package：`rviz_visual_tools`；Executable：见launch Node与CMake目标（插件嵌入RViz）；控制对象由上述接口调用判定。

```text
565: default: throw std::runtime_error("Unknown size");
587: default: throw std::runtime_error("Unknown size");
721: RCLCPP_ERROR(logger_, ss.str().c_str());
2216: RCLCPP_ERROR(logger_, ss.str().c_str());
2245: RCLCPP_ERROR(logger_, ss.str().c_str());
2816: RCLCPP_ERROR(tmp_node.get_logger(), ss.str().c_str());
2831: RCLCPP_ERROR(tmp_node.get_logger(), ss.str().c_str());
```

### `dual_arm_ws/src/rviz_visual_tools/src/tf_visual_tools.cpp`

Package：`rviz_visual_tools`；Executable：见launch Node与CMake目标（插件嵌入RViz）；控制对象由上述接口调用判定。

```text
34: #include <rclcpp/create_timer.hpp>
58: rclcpp::create_timer(node_base_interface_, timers_interface_, clock_interface_->get_clock(),
```

### `src/antbot_camera/src/camera_node.cpp`

Package：`antbot_camera`；Executable：见launch Node与CMake目标（插件嵌入RViz）；控制对象由上述接口调用判定。

```text
47: timer_ = this->create_wall_timer(period, std::bind(&CameraNode::timer_callback, this));
74: this->declare_parameter<std::vector<std::string>>(
79: this->declare_parameter<int>("camera.v4l2_driver.resolution.width", 640);
80: this->declare_parameter<int>("camera.v4l2_driver.resolution.height", 360);
81: this->declare_parameter<int>("camera.v4l2_driver.default_fps", 15);
82: this->declare_parameter<bool>("camera.v4l2_driver.use_rom_mode", false);
83: this->declare_parameter<std::string>(
85: this->declare_parameter<std::vector<std::string>>(
89: this->declare_parameter<int>("camera.usb_camera.resolution.width", 640);
90: this->declare_parameter<int>("camera.usb_camera.resolution.height", 480);
91: this->declare_parameter<int>("camera.usb_camera.default_fps", 10);
92: this->declare_parameter<std::vector<std::string>>(
125: this->declare_parameter<std::string>("camera.v4l2_driver." + pos + ".port", "");
126: this->declare_parameter<std::string>("camera.v4l2_driver." + pos + ".fourcc_type", "UYVY");
127: this->declare_parameter<std::string>(
129: this->declare_parameter<std::string>(
131: this->declare_parameter<std::string>(
133: this->declare_parameter<int>(
135: this->declare_parameter<int>(
137: this->declare_parameter<int>(
173: RCLCPP_ERROR(
189: this->declare_parameter<std::string>(
191: this->declare_parameter<std::string>(
193: this->declare_parameter<std::string>(
195: this->declare_parameter<int>(
226: RCLCPP_ERROR(
238: RCLCPP_ERROR(
```

### `src/antbot_camera/src/camera_test_node.cpp`

Package：`antbot_camera`；Executable：见launch Node与CMake目标（插件嵌入RViz）；控制对象由上述接口调用判定。

```text
63: this->declare_parameter<std::string>("test.output_dir", "/tmp/antbot_camera_test");
64: this->declare_parameter<double>("test.save_interval_sec", 1.0);
72: this->declare_parameter<std::vector<std::string>>(
78: this->declare_parameter<std::vector<std::string>>(
```

### `src/antbot_imu/src/imu_node.cpp`

Package：`antbot_imu`；Executable：见launch Node与CMake目标（插件嵌入RViz）；控制对象由上述接口调用判定。

```text
37: throw std::runtime_error("Failed to initialize IMU node");
55: this->declare_parameter("imu.frame_id", "imu_link");
56: this->declare_parameter("imu.filter_cutoff_hz", 10.0);
57: this->declare_parameter("imu.publish_rate", 100.0);
58: this->declare_parameter("imu.calibration_num", 350);
59: this->declare_parameter("imu.scale.acceleration", 0.00006103515625);
60: this->declare_parameter("imu.scale.angular_vel", 0.0609756098);
61: this->declare_parameter("imu_board.port", "");
62: this->declare_parameter("imu_board.id", 0);
63: this->declare_parameter("imu_board.baud_rate", 0);
64: this->declare_parameter("imu_board.protocol_version", 0.0);
65: this->declare_parameter("control_table_path", "");
92: publish_timer_ = this->create_wall_timer(
105: RCLCPP_ERROR(this->get_logger(), "Failed to connect to IMU board");
124: RCLCPP_ERROR_THROTTLE(
```

### `src/robotcar_navigation/include/robotcar_navigation/waypoint_manager_core.hpp`

Package：`robotcar_navigation`；Executable：见launch Node与CMake目标（插件嵌入RViz）；控制对象由上述接口调用判定。

```text
33: load_file_ = declare_parameter<std::string>("load", default_file);
99: publish_timer_ = create_wall_timer(
```

### `src/robotcar_navigation/src/add_charger_tool.cpp`

Package：`robotcar_navigation`；Executable：见launch Node与CMake目标（插件嵌入RViz）；控制对象由上述接口调用判定。

```text

```

### `src/robotcar_navigation/src/add_keepout_zone_tool.cpp`

Package：`robotcar_navigation`；Executable：见launch Node与CMake目标（插件嵌入RViz）；控制对象由上述接口调用判定。

```text

```

### `src/robotcar_navigation/src/add_speed_zone_tool.cpp`

Package：`robotcar_navigation`；Executable：见launch Node与CMake目标（插件嵌入RViz）；控制对象由上述接口调用判定。

```text

```

### `src/robotcar_navigation/src/add_waypoint_tool.cpp`

Package：`robotcar_navigation`；Executable：见launch Node与CMake目标（插件嵌入RViz）；控制对象由上述接口调用判定。

```text

```

### `src/robotcar_navigation/src/charger_get_position.cpp`

Package：`robotcar_navigation`；Executable：见launch Node与CMake目标（插件嵌入RViz）；控制对象由上述接口调用判定。

```text
26: RCLCPP_ERROR(node->get_logger(), "No charger named [%s]", request->name.c_str());
```

### `src/robotcar_navigation/src/demo_map_tool.cpp`

Package：`robotcar_navigation`；Executable：见launch Node与CMake目标（插件嵌入RViz）；控制对象由上述接口调用判定。

```text
20: timer_ = create_wall_timer(1s, [this]() {
```

### `src/robotcar_navigation/src/floor_texture_publisher.cpp`

Package：`robotcar_navigation`；Executable：见launch Node与CMake目标（插件嵌入RViz）；控制对象由上述接口调用判定。

```text
12: const auto frame_id = node->declare_parameter<std::string>("frame_id", "map");
13: auto mesh_resource = node->declare_parameter<std::string>(
15: const auto width = node->declare_parameter<double>("width", 4.19);
16: const auto height = node->declare_parameter<double>("height", 4.18);
17: const auto pos_x = node->declare_parameter<double>("pos_x", -0.004);
18: const auto pos_y = node->declare_parameter<double>("pos_y", 0.001);
19: const auto pos_z = node->declare_parameter<double>("pos_z", -0.02);
20: const auto alpha = node->declare_parameter<double>("alpha", 1.0);
23: RCLCPP_ERROR(node->get_logger(), "No floor model path provided.");
```

### `src/robotcar_navigation/src/pose_navi_server.cpp`

Package：`robotcar_navigation`；Executable：见launch Node与CMake目标（插件嵌入RViz）；控制对象由上述接口调用判定。

```text

```

### `src/robotcar_navigation/src/set_pose_from_waypoint.cpp`

Package：`robotcar_navigation`；Executable：见launch Node与CMake目标（插件嵌入RViz）；控制对象由上述接口调用判定。

```text
19: RCLCPP_ERROR(node->get_logger(), "Usage: set_pose_from_waypoint <waypoint_name>");
33: RCLCPP_ERROR(node->get_logger(), "Failed to get waypoint [%s]", request->name.c_str());
```

### `src/robotcar_navigation/src/vehicle_status_panel.cpp`

Package：`robotcar_navigation`；Executable：见launch Node与CMake目标（插件嵌入RViz）；控制对象由上述接口调用判定。

```text
136: if (flags & 0x08) {names << QObject::tr("CAN Bus-Off");}
536: connect(timer, &QTimer::timeout, this, &VehicleStatusPanel::updateDataStatus);
542: keyboard_timer, &QTimer::timeout,
```

### `src/robotcar_navigation/src/waypoint_manager_panel.cpp`

Package：`robotcar_navigation`；Executable：见launch Node与CMake目标（插件嵌入RViz）；控制对象由上述接口调用判定。

```text

```

### `src/robotcar_navigation/src/wp_edit_node.cpp`

Package：`robotcar_navigation`；Executable：见launch Node与CMake目标（插件嵌入RViz）；控制对象由上述接口调用判定。

```text
46: load_file_ = declare_parameter<std::string>("load", default_file);
82: update_timer_ = create_wall_timer(
```

### `src/robotcar_navigation/src/wp_nav_remote.cpp`

Package：`robotcar_navigation`；Executable：见launch Node与CMake目标（插件嵌入RViz）；控制对象由上述接口调用判定。

```text

```

### `src/robotcar_navigation/src/wp_nav_test.cpp`

Package：`robotcar_navigation`；Executable：见launch Node与CMake目标（插件嵌入RViz）；控制对象由上述接口调用判定。

```text

```

### `src/robotcar_navigation/src/wp_navi_server.cpp`

Package：`robotcar_navigation`；Executable：见launch Node与CMake目标（插件嵌入RViz）；控制对象由上述接口调用判定。

```text

```

### `src/robotcar_navigation/src/wp_saver.cpp`

Package：`robotcar_navigation`；Executable：见launch Node与CMake目标（插件嵌入RViz）；控制对象由上述接口调用判定。

```text
65: node->declare_parameter<std::string>("save_file", default_save_file);
```

### `src/robotcar_navigation/src/wp_set_pose.cpp`

Package：`robotcar_navigation`；Executable：见launch Node与CMake目标（插件嵌入RViz）；控制对象由上述接口调用判定。

```text
30: RCLCPP_ERROR(get_logger(), "Service /waterplus/get_waypoint_name is not available");
```

### `src/vanjee_lidar_sdk/src/source/source_device_ctrl_ros.hpp`

Package：`vanjee_lidar_sdk`；Executable：见launch Node与CMake目标（插件嵌入RViz）；控制对象由上述接口调用判定。

```text

```

### `src/vanjee_lidar_sdk/src/source/source_imu_packet_ros.hpp`

Package：`vanjee_lidar_sdk`；Executable：见launch Node与CMake目标（插件嵌入RViz）；控制对象由上述接口调用判定。

```text

```

### `src/vanjee_lidar_sdk/src/source/source_pointcloud_ros.hpp`

Package：`vanjee_lidar_sdk`；Executable：见launch Node与CMake目标（插件嵌入RViz）；控制对象由上述接口调用判定。

```text

```

### `src/vanjee_lidar_sdk/src/source/source_recv_device_ctrl_ros.hpp`

Package：`vanjee_lidar_sdk`；Executable：见launch Node与CMake目标（插件嵌入RViz）；控制对象由上述接口调用判定。

```text

```

### `src/vanjee_lidar_sdk/src/source/source_ros_msg_delegate.hpp`

Package：`vanjee_lidar_sdk`；Executable：见launch Node与CMake目标（插件嵌入RViz）；控制对象由上述接口调用判定。

```text

```

### `src/vanjee_lidar_sdk/src/source/source_scandata_ros.hpp`

Package：`vanjee_lidar_sdk`；Executable：见launch Node与CMake目标（插件嵌入RViz）；控制对象由上述接口调用判定。

```text

```

