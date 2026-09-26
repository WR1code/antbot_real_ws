# ROS 命名及 TF 迁移建议

审计日期：2026-09-15。静态源码审计；CONFIRMED 指实现/配置已发现，不代表真机在线。UNKNOWN / NEED_CONFIRMATION 不作为可执行配置。详见 [扫描范围](SOURCE_SCAN.md)。

| 当前 | 建议逻辑接口 | 注意 |
|---|---|---|
| /cmd_vel、/antbot/cmd_vel/xbox/keyboard | /robot/base/command（仅arbiter）及sources/* | bridge不可保留不受控旁路；Twist类型兼容 |
| /rs00/motor_status、/antbot/vehicle_status | /robot/base/status + hardware/rs00/* | 暂保留JSON格式，状态适配器提供统一健康 |
| /rebotarm/* | /robot/left_arm/*（待物理映射确认） | model=dm/rs显式配置；不猜左右 |
| /piperh/* | /robot/right_arm/*（右mount证据） | vendor固定绝对topic、enable服务、Action client需一起迁 |
| /dual_arm/* | /robot/system/control_selection 或 ownership | Xbox selection不等于资源所有权 |
| /sensor/camera/gemini336l/*、vision target | /robot/perception/* | driver names与CameraInfo/optical frame联合测试 |
| /piperh/pulse/* | /robot/sensors/pulse/* | 保留绑定目标臂的task配置，不把sensor绑定永久右臂 |
| /scan_0、/scan_1、/antbot/lidar/* | /robot/sensors/lidar/front/*、rear/* | real 2D和sim 3D不能混同 |
| /joint_states | 底盘状态专属topic+受控joint聚合 | 当前两臂已独立命名；joint_names也须唯一 |

证据：[`dual_arm_ws/src/rebot_xbox_hardware/launch/dual_arm_hardware.launch.py:23`](../../dual_arm_ws/src/rebot_xbox_hardware/launch/dual_arm_hardware.launch.py#L23)；[`src/antbot_camera/launch/camera.launch.py:24`](../../src/antbot_camera/launch/camera.launch.py#L24)；[`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:133`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L133)；[`src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py:30`](../../src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py#L30)。

分步用参数/remap适配新命名，不直接批量替换；短期保留旧只读state别名，命令别名必须进同一个arbiter，禁止双向relay成回环/双writer。每次只迁一个控制链并验证pub/sub/action/service全端与UI。TF frame_id不是ROS namespace，不能只PushRosNamespace就指望隔离；目前rebotarm prefixed live TF与base_link planning alias、Piper planning_world alias并存，需核实物理外参和MoveIt机器人根frame，禁止本轮改标定。导航接入时还需核查static world→map和定位动态TF是否形成竞争。
