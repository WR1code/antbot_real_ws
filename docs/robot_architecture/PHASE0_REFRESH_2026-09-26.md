# Phase 0 当前源码审计刷新（2026-09-26）

本轮只做静态源码审计；没有打开 CAN、串口或真实硬件，没有启动 Nav2，也没有发送
运动命令。`CONFIRMED` 只表示当前源码中存在该路径，不代表真机状态。所有物理接口、
设备身份、固件和制动行为仍为 `HARDWARE VERIFICATION REQUIRED`。

## 审计基线

- 工作区：`/home/w/project/antbot_real_ws`
- 分支：`main`
- HEAD：`ad487d319debaebc920bf79c06ce3b6a8c0a64e2`
- 扫描时工作树：14 个 tracked 文件有修改，1691 个 untracked 文件；不得把当前文件
  内容误认为上述提交的纯净内容。
- 旧审计日期为 2026-09-15；`hardware_adapter.py` 等文件此后有修改，因此本文件按
  当前磁盘内容重新核对，不沿用旧行号作为事实。

## Piper 最终运动路径

| 路径 | 当前证据 | 当前风险状态 |
|---|---|---|
| FollowJointTrajectory Action | `piperh_control/hardware_adapter.py:624-793`；执行循环在 `:738` 调 `_publish_driver` | **仍存在**：Action 只阻止第二个 Action；最终 writer 不验证调用方 owner/token |
| Servo JointTrajectory | 同文件 `:615-622` | **仍存在**：`_stream` 直接调用 `_publish_driver`，Action ACTIVE 时仍可进入 |
| adapter ROS→vendor | 同文件 `:592-613` 发布 `driver_joint_command` | **仍存在**：只检查反馈、motor enable、gravity，没有 owner/lease/epoch |
| vendor JointState→SDK | `piper/piper_ctrl_single_node.py:395-444` 调 `JointCtrl/GripperCtrl` | **仍存在**：最终 SDK 前只检查 enable flag，没有 authoritative owner |
| vendor PosCmd→SDK | 同文件 `:361-393` 调 `EndPoseCtrl/GripperCtrl` | **仍存在**：可绕过 piperh adapter |
| vendor enable topic/service | 同文件 `:447-504` 调 `EnableArm/DisableArm` | **仍存在**：topic/service 是独立旁路 |
| gravity MIT direct SDK | `hardware_adapter.py:936-1125`，`mit_backend.py:60-118` | **仍存在但默认锁定**：真实 torque 默认关闭；仍须纳入最终 ownership |
| standalone gravity audit | `gravity_audit.py:18-86` | **维护旁路**：会直接打开 CAN；本轮禁止执行，后续须 maintenance 独占 |

当前 `piper_ctrl_single_node.py` 的 `auto_enable` 默认已经是 `False`，但旧
`start_two_piper.launch.py:16-30` 仍把左右接口都默认成 `can0`，并把 launch 参数
`auto_enable` 默认成 `true`，所以 S03 仍存在。

## reBot 生命周期

| 阶段 | 当前实现 | 风险状态 |
|---|---|---|
| connect | `hardware_manager.py:154-163`：connect 后调用 `_start_endpos_loop` 并置 `_enabled=True` | **S01 仍存在** |
| initialize/hold | `:746-754`：配置 group、hold 当前位、启动 control loop | 与 connect 耦合，尚无不运动 INITIALIZED 状态 |
| group enable | `:757-771`：`_configure_groups_for_endpos` 调两个 group 的 `enable()` | connect 可间接自动 enable |
| explicit enable/disable | `:295-306` | API 存在，但 connect 已提前进入 enabled，语义不成立 |
| safe_home | `:310-348` | 功能可保留，但必须成为显式动作 |
| shutdown | `:175-205`：默认先 `safe_home`，再 stop/disable/disconnect | **S02 仍存在**；fault/ESTOP 未分支 |
| node shutdown/signal | `rebotarm_controller.py:105-134` | signal/异常最终仍调用上述 shutdown，存在隐式运动风险 |
| watchdog/reconnect | SDK/节点有局部异常处理，但未发现统一 epoch/lease 撤销 | **仍存在** |

需要现场确认 MotorBridge transport、掉电/刹车姿态和 enable 反馈语义，标记为
`HARDWARE VERIFICATION REQUIRED`；这些未知项不阻碍离线 fail-closed 改造。

## 底盘 `/cmd_vel` 路径

当前最终 consumer 是 `antbot_h743_bridge/bridge.py:173-176`，默认订阅全局
`/cmd_vel`；在 `:354-393` 只做数值、H743 ready 和速度门禁，不验证 source owner 或
lease。`operator_manager.py:178-233` 只是局部 xbox/keyboard mux，仍向全局
`/cmd_vel` 发布，并不是 authoritative writer。

扫描到的运动 producer/入口至少包括：

- `antbot_teleop/mapping_xbox.py:114-115`（默认 `/cmd_vel`，real launch 中有 remap）；
- `mapping_keyboard.py:102`、`teleop_smooth.py:74`、`teleop_keyboard.py:63`、
  `teleop_joystick.py:110`；
- `robotcar_navigation/pure_pursuit_planner.py:45`；
- `robotcar_navigation/src/vehicle_status_panel.cpp:588`；
- `antbot_dual_lidar/phase2a_motion_experiment.py:60`；
- `operator_manager.py:178-233`。

因此 C04 **仍存在**。Phase 1 必须把真实 bridge 改为只接受带 owner/epoch/lease 的
内部 authoritative command；旧 `/cmd_vel` 只能留作 producer 输入或显式 unsafe
维护入口，不能继续直接到 UART writer。Nav2 继续关闭。

## 自动动作、退出和恢复行为

- Piper vendor 默认 `auto_enable=False`，但 legacy 双臂 launch 覆盖为 true。
- reBot connect 自动 enable/hold，shutdown 默认 safe_home。
- H743 bridge 具有 UART reconnect、fresh telemetry 和 operator gate；reconnect 不等于
  ownership 恢复，旧 command/owner 必须失效。
- Piper feedback 恢复后当前代码保持 motor disabled，这是已有的 fail-closed 行为。
- 未发现整机统一 ESTOP 状态或最终 writer epoch；Phase 3 Supervisor 不在本轮范围。

## Phase 1 离线改造边界

本轮允许：纯内存 lease/epoch gate、最终 writer 检查、显式 connect/initialize/enable、
按原因 shutdown、legacy launch fail-closed、fake transport 测试。禁止创建完整
Supervisor、启动 Nav2、修改 MCU 固件或猜测任何现场接口参数。

