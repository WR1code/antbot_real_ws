# State Machines

审计日期：2026-09-15。静态源码审计；CONFIRMED 指实现/配置已发现，不代表真机在线。UNKNOWN / NEED_CONFIRMATION 不作为可执行配置。详见 [扫描范围](SOURCE_SCAN.md)。

| 范围 | 已有状态机 | 状态/互锁 | 证据 |
|---|---|---|---|
| 整机 | 未发现 | selected和UI status不是BOOTING/READY/任务状态聚合 | [`dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/active_arm_manager.py:1`](../../dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/active_arm_manager.py#L1)；[`dual_arm_ws/src/rebot_xbox_hardware/launch/dual_arm_hardware.launch.py:1`](../../dual_arm_ws/src/rebot_xbox_hardware/launch/dual_arm_hardware.launch.py#L1) |
| 底盘MCU | YES | 转向boot/stop/UID/position/mode/limits→ARMED→enable/verify→READY；FAULT；translation stop/align/run/timeout/fault | [`firmware/rs00_fk743_test/firmware/App/Inc/steering_controller.h:14`](../../firmware/rs00_fk743_test/firmware/App/Inc/steering_controller.h#L14)；[`firmware/rs00_fk743_test/firmware/App/Inc/chassis_translation_controller.h:22`](../../firmware/rs00_fk743_test/firmware/App/Inc/chassis_translation_controller.h#L22) |
| 底盘host | YES（门禁+teleop模式） | operator_requested、ACK fresh、motion_allowed、xbox/keyboard；MCU故障撤销门禁 | [`src/antbot_h743_bridge/antbot_h743_bridge/bridge.py:341`](../../src/antbot_h743_bridge/antbot_h743_bridge/bridge.py#L341)；[`src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py:94`](../../src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py#L94) |
| reBotArm | YES | IDLE、TRAJ_RUNNING、LOWLEVEL_STREAMING、GRAVITY_COMP、SAFE_HOMING；reject/preempt按入口 | [`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/hardware_manager.py:138`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/hardware_manager.py#L138)；[`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/ros_actions.py:170`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/ros_actions.py#L170) |
| Piper | YES（布尔/phase组合） | _motors_enabled、_action_active、gravity_active/transition/phase；status DISABLED/IDLE及gravity阶段；无整机ownership | [`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:629`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L629)；[`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:731`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L731) |
| 双臂Xbox handoff | YES | selected→none/pending→两armed=false→selected；未知状态阻塞；无fresh epoch | [`dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/arm_ownership.py:1`](../../dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/arm_ownership.py#L1) |
| Teach | YES | 录制/Leader/gravity/replay/FAULT及preflight feedback/status/xbox checks | [`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:959`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L959)；[`dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py:4414`](../../dual_arm_ws/src/rebot_teach_mode/rebot_teach_mode/teach_node.py#L4414) |
| Piper把脉预接触 | YES | 明确PlanningState，target暂失恢复、axis align、precontact、dummy pressure stop、ERROR | [`src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py:71`](../../src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py#L71)；[`src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py:596`](../../src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py#L596) |
| 传统PulseApproach | 过程式run | target/TF/planner等待与结果；不是整机任务状态机 | [`src/rebotarm_pulse/rebotarm_pulse/pulse_approach.py:73`](../../src/rebotarm_pulse/rebotarm_pulse/pulse_approach.py#L73) |
| Nav2 | 外部Lifecycle | configure/activate；真机operator默认关闭 | [`src/antbot_real_bringup/launch/operator_step2.launch.py:83`](../../src/antbot_real_bringup/launch/operator_step2.launch.py#L83) |

## MCU真实枚举（保留细粒度，不虚构OFF/MOVING）

```c

    CHASSIS_TRANSLATION_IDLE = 0,
    CHASSIS_TRANSLATION_STOPPING_DRIVE,
    CHASSIS_TRANSLATION_STEERING,
    CHASSIS_TRANSLATION_WAIT_ALIGNMENT,
    CHASSIS_TRANSLATION_DRIVING,
    CHASSIS_TRANSLATION_TIMEOUT_STOP,
    CHASSIS_TRANSLATION_FAULT

```

## Piper预接触真实枚举

```python
IDLE = "IDLE"
    WAITING_FOR_INPUT = "WAITING_FOR_INPUT"
    TARGET_TRACKING = "TARGET_TRACKING"
    TARGET_STABILIZING = "TARGET_STABILIZING"
    TARGET_STABLE = "TARGET_STABLE"
    TARGET_TEMPORARILY_STALE = "TARGET_TEMPORARILY_STALE"
    WAIT_TARGET_RECOVERY = "WAIT_TARGET_RECOVERY"
    ARM_AXIS_ALIGN = "ARM_AXIS_ALIGN"
    PRECONTACT_POSITIONING = "PRECONTACT_POSITIONING"
    PRECONTACT_READY = "PRECONTACT_READY"
    APPROACH_PLANNED = "APPROACH_PLANNED"
    PRECONTACT_EXECUTING = "PRECONTACT_EXECUTING"
    PRECONTACT_REACHED = "PRECONTACT_REACHED"
    DUMMY_TARGET_REACHED = "DUMMY_TARGET_REACHED"
    DUMMY_PRESSURE_STOPPED = "DUMMY_PRESSURE_STOPPED"
    ERROR = "ERROR"
```

## 互锁结论

MCU的steering/drive/translation互锁比上层整机更完整；Teach→driver状态和Xbox锁有局部关联，Piper gravity→selected/armed有局部门禁。没有找到“Nav2已取消且base停稳才允许任一臂运动”的全局状态机。ARMED=false和driver IDLE不能直接代表物理速度为零；enabled=false也不自动代表臂姿态安全/无重力下落风险。

建议整机状态用BOOTING→INITIALIZING→STANDBY→READY及FAULT/ESTOP；任务模式NAVIGATION/MANIPULATION/PULSE/TEACH/MANUAL单独字段，切换过程显式SWITCHING。PULSE当前宜标预接触/演示能力，不能把dummy pressure阶段当完整把脉。详见SUPERVISOR_DESIGN。
