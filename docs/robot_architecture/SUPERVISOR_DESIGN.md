# robot_supervisor 建议（尚未实现）

审计日期：2026-09-15。静态源码审计；CONFIRMED 指实现/配置已发现，不代表真机在线。UNKNOWN / NEED_CONFIRMATION 不作为可执行配置。详见 [扫描范围](SOURCE_SCAN.md)。

## 基于现有工程的最小方案

新增一个robot_supervisor包：Supervisor模式Action server + ResourceManager（可同进程模块）+现有驱动状态适配器。保留antbot_h743_bridge、HardwareAdapter、reBotArmController及现有Teach/MoveIt业务；先在最终writer封闭控制入口，再接Supervisor，不替换CAN/SDK协议。与Nav2 lifecycle_manager协作而不重复实现其内部lifecycle。RViz/CLI只向Supervisor发意图，不能自行绕过owner enable。

| 模块 | 职责 | 使用当前证据 | 需要补的内容 |
|---|---|---|---|
| health_adapter | 在线/初始化/可控/运动/故障及年龄 | base JSON、两臂ArmStatus、pressure sample、target diagnostics | 统一时间、feedback序号、sample heartbeat、offline和ESTOP反馈 |
| ResourceManager | 原子获取BASE/两臂、续租/释放、epoch | 现有base mux、双臂Xbox selection | task owner强制校验、writer epoch、deadline、批量资源不死锁 |
| mode_transition Action | 可取消切换、step反馈、失败原因 | CancelGoal端点、driver停止/状态 | 取消终态、停稳证明、安全姿态、回撤闭环 |
| task Action | 导航/预接触/以后完整把脉 | 现有MoveItDemoBase/PiperPulseAlign流程 | acquire/heartbeat/result/cleanup，不直接enable |
| bringup monitor | BOOTING/INITIALIZING/STANDBY能力门禁 | 当前嵌套launch/instance guard | 依赖check、deadline、blockers，不把进程存在当READY |

## 状态模型

system_state={BOOTING,INITIALIZING,STANDBY,READY,SWITCHING,FAULT,ESTOP}；mode={MANUAL,NAVIGATION,MANIPULATION,PULSE_PRECONTACT,TEACH}；active_task与resources单列。避免NAVIGATING/PULSE同时作为设备状态和任务模式，导致组合爆炸。NAVIGATION在odom验收前capability=false；完整PULSE_MEASURING直到接触压力/撤回任务验收后才开放。当前所需hardware未确认时保持STANDBY并显示blockers，不能自动使能使READY成立。

建议system_status包含boot_id/epoch、state、mode、capabilities、resource owner、faults、blockers、sample ages。READY是当前模式的依赖与安全检查通过，不是所有可选传感器都上线；ESTOP解除必须人工确认后返回STANDBY，不自动恢复旧任务。

## ownership API（建议，不属于interfaces.yaml已有接口）

`AcquireResources(requester,resources,mode,ttl)`返回lease_id/epoch/expiry；`RenewLease`；`ReleaseResources`。获取是全有或全无；已经被持有则BUSY，不隐式抢占。heartbeat失效/任务crash/driver重启使epoch无效。Supervisor的ChangeMode Action协调明确撤销旧owner，再授新owner；人工急停无需等租约协议即可本地锁输出。

BASE使用原operator mux扩展navigation/manual/task/stop输入，只有final arbiter可写bridge入口。arms使用各自Action proxy和受限stream入口；writer收到命令时校验绑定owner/epoch。标准ROS轨迹没有token字段，必须把租约绑定到代理goal/session，不能只在任务启动前acquire一次。供应商pos_cmd/enable_flag、低层passthrough和独立SDK需进入受控maintenance入口或生产配置禁用；单进程RLock不覆盖另一进程写CAN。

## 机制取舍

| 机制 | 用途 | 当前建议 |
|---|---|---|
| Topic mux | 速度流择源和零速优先 | 扩展已有operator_manager，不能独自保护arms Action |
| Service | acquire/renew/release与读取状态 | 轻量，短操作；不能负责整个长任务 |
| Action | 模式切换与任务，feedback/cancel/result | 首选；cancel必须接硬件终态屏障 |
| LifecycleNode | 初始化/激活/降级/恢复 | Supervisor可用；现有driver先加适配器，分期改造 |
| ros2_control switching | 已共享hardware command interfaces的controller仲裁 | 当前真机直接SDK路径不适用，暂不迁成plugin |
| ResourceManager+lease | 跨BASE/两臂、客户端crash撤权 | 最小必要的资源管理层，强制在writer校验 |

[Nav2 Jazzy生命周期管理](https://docs.ros.org/en/jazzy/p/nav2_lifecycle_manager/__README.html)支持有序管理lifecycle节点；[Jazzy Controller Manager](https://control.ros.org/jazzy/doc/ros2_control/controller_manager/doc/userdoc.html)管理ros2_control controller/hardware接口。它们不能覆盖现有独立SDK writer；此项适用性是结合仓库实现作出的判断。

## 当前缺口定位

[`dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/active_arm_manager.py:62`](../../dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/active_arm_manager.py#L62)需从Xbox选择扩展到事务屏障；[`src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py:186`](../../src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py#L186)需将最终BASE输出归属统一owner；[`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:722`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L722)需Action/stream互斥；[`dual_arm_ws/src/piper/piper/piper_ctrl_single_node.py:395`](../../dual_arm_ws/src/piper/piper/piper_ctrl_single_node.py#L395)需封旁路；[`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/hardware_manager.py:154`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/hardware_manager.py#L154)与[`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/hardware_manager.py:175`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/hardware_manager.py#L175)需启动/正常退出/ESTOP行为分离；[`src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py:162`](../../src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py#L162)需sample heartbeat；[`src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py:788`](../../src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py#L788)需BASE停稳互锁和task ownership。
