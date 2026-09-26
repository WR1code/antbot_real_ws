# Control Ownership 审计

审计日期：2026-09-15。静态源码审计；CONFIRMED 指实现/配置已发现，不代表真机在线。UNKNOWN / NEED_CONFIRMATION 不作为可执行配置。详见 [扫描范围](SOURCE_SCAN.md)。

| Resource | 当前控制源 | 潜在冲突源 | 是否有仲裁 | 实现边界 |
|---|---|---|---|---|
| base | operator_manager Xbox/键盘择一 | 独立teleop、dry-run、Nav2、固件host/CLI | PARTIAL | 遥控输入mux + bridge门禁；/cmd_vel订阅不辨publisher；串口旁路不受mux管辖 |
| rebotarm（left映射待确认） | MoveIt/Servo/Teach/低层passthrough | demo、pulse、直接SDK、第二套driver | PARTIAL | HardwareManager state/RLock；低层reject/preempt；Xbox选臂不是任务ownership |
| piperh（右侧mount有代码证据） | MoveIt Action/Servo/Teach/Gravity | Action与Servo、vendor pos_cmd/joint command/enable、第二套Piper driver | PARTIAL | gravity/trajectory局部互斥；Servo未检查action_active；vendor旁路绕过adapter |
| pressure_serial | PressureSerialBridge | 双overlay同包、独立采集 | YES（局部） | PressureSerialLease+exclusive；parent遇已有持有者/同arm端口会跳过 |
| /joy | dual_arm_joy | standalone joy节点 | PARTIAL | 总入口共享，独立launch仍可再起 |
| whole_robot | 无统一任务控制者 | base/arm分开门禁、UI/demo/任务 | NO | 未发现BASE/LEFT_ARM/RIGHT_ARM租约和整机停稳互锁 |

## 已有保护，不能抹掉

[`src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py:186`](../../src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py#L186)按teleop_mode择源，[`src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py:219`](../../src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py#L219)0.35s超时；[`src/antbot_h743_bridge/antbot_h743_bridge/bridge.py:341`](../../src/antbot_h743_bridge/antbot_h743_bridge/bridge.py#L341)要求人工请求+新鲜ACK+MCU readiness。[`dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/active_arm_manager.py:62`](../../dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/active_arm_manager.py#L62)先发布selected=none，再请求取消多个Action，等待两臂armed=false再选择。[`dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/arm_ownership.py:29`](../../dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/arm_ownership.py#L29)是明确的Xbox handoff状态机。[`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/hardware_manager.py:417`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/hardware_manager.py#L417)、[`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/motor_passthrough.py:128`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/motor_passthrough.py#L128)、[`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:959`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L959)已有局部模式/反馈门禁；instance_guard防止同名整机重复入口。压力锁保护设备排他。

## 仍可成立的冲突条件

1. **Piper Action + Servo**：[`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:722`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L722)只检查点/范围后调用_publish_driver；[`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:699`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L699)只检查enable、feedback、gravity，无_action_active检查；_arm_goal拒绝第二Action但没有关闭Servo。因此在enable且feedback有效时两条源可交错向driver发目标。这是源码允许条件，不是已观察实机事件。
2. **Piper adapter + vendor旁路**：[`dual_arm_ws/src/piper/piper/piper_ctrl_single_node.py:361`](../../dual_arm_ws/src/piper/piper/piper_ctrl_single_node.py#L361)、[`dual_arm_ws/src/piper/piper/piper_ctrl_single_node.py:395`](../../dual_arm_ws/src/piper/piper/piper_ctrl_single_node.py#L395)直接调用EndPoseCtrl/JointCtrl，只有vendor EnableFlag门禁；厂商enable_flag可直接使能。Gravity的直接SDK writer和vendor writer之间没有全进程ownership token。
3. **切臂尚无取消完成屏障**：[`dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/active_arm_manager.py:90`](../../dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/active_arm_manager.py#L90)异步call_async的future不等待结果；armed=false不证明Action终止、关节速度为零或gravity已退出。ArmOwnership缓存的false无年龄和epoch；旧false可能参与下一次切换判断。
4. **/cmd_vel旁路**：standalone teleop默认/cmd_vel；dry-run虽然不打开H743，但若现有bridge仍在线且门禁开放，它发布的/cmd_vel仍会被接受。总入口选源mux并不能阻止外部直接写输出。
5. **跨底盘与机械臂**：选臂/手柄交互只能约束人工命令源，没有把MoveIt/把脉流程和底盘MCU停稳互锁联成原子事务。不得宣称“底盘永远与双臂互斥”。
6. **可选RS型号+Piper**：rebot model=rs 默认SDK can0，parent piper_channel也can0。实际是否接同总线 UNKNOWN；必须按USB序列号和拓扑核实，ROS namespace无法隔离物理CAN。

7. **重复driver/SDK进程**：instance_guard只限制使用同锁名的入口；独立Piper launch、reBot SDK例子或串口CLI没有接入同一个跨资源租约。实际同设备双开是否被内核/库拒绝 NEED_CONFIRMATION；CAN允许多socket存在不能作为排他保证。

## 直接硬件访问清单

PiperRosNode/C_PiperInterface是writer；HardwareAdapter direct feedback是reader；OfficialMitBackend gravity是writer。reBotArm HardwareManager/SDK/MotorBridge是writer；串口CLI/固件host也是writer。所有入口见SOURCE_SCAN和CONTROL_CHAINS；不要将CAN recv、反馈发布或JointState可视化误计为命令。

## 适合当前工程的改造建议（本轮不实现）

保留已有底盘mux与两臂adapter，在最终硬件writer前加入统一资源校验。ResourceManager负责BASE/REB0T_ARM/PIPER_ARM（物理左右确认后命名LEFT_ARM/RIGHT_ARM）的原子获取、续租、释放和故障撤销；普通任务使用Service租约，模式切换/长任务使用Action。每个writer需要current owner、epoch及lease deadline，本地monotonic超时失效。Topic mux适合Twist源选择，不能单独保护Action/SDK；仅发token但不在writer检查没有安全意义。标准JointTrajectory/FollowJointTrajectory没有token字段，可由owner专用入口+Action proxy绑定租约，最后adapter验证当前epoch；低层调试进入独占maintenance模式，生产路径收敛到一个writer。

Lifecycle仅管理配置/激活，不能代替控制仲裁。当前真机两臂轨迹server并非已部署ros2_control hardware plugin；不建议为switch_controller全量重写。本地controller switching仅在确实使用ros2_control共享接口后考虑。[ROS 2 Nav2 lifecycle说明](https://docs.ros.org/en/jazzy/p/nav2_lifecycle_manager/__README.html)；[Jazzy Controller Manager说明](https://control.ros.org/jazzy/doc/ros2_control/controller_manager/doc/userdoc.html)。这些是建议依据，不是本仓库已有实现。
