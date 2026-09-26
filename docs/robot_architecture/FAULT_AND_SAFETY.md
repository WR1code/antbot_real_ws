# Fault and Safety

审计日期：2026-09-15。静态源码审计；CONFIRMED 指实现/配置已发现，不代表真机在线。UNKNOWN / NEED_CONFIRMATION 不作为可执行配置。详见 [扫描范围](SOURCE_SCAN.md)。

| 故障 | 谁发现 | 谁上报 | 谁停止 | 谁复位/恢复 | 缺口 |
|---|---|---|---|---|---|
| H743 UART/USB拔出 | bridge SerialException/OSError | JSON connected/operator_enabled | host撤销门禁；MCU300ms命令超时 | bridge重连，人工再确认 | 无整机ARM inhibit传播 |
| H743 ACK stale | motion_allowed 1.5s门禁 | JSON telemetry_online | 新命令被置零；MCU命令超时兜底 | ACK恢复+人工门禁状态检查 | 不是仅JSON判断物理停止 |
| MCU CAN bus-off | rs00_fdcan latch | debug/ACK fault bits | translation/drive emergency stop（CAN坏时送达不能保证） | guarded reset/recommission | 电气动力切断独立性待测 |
| MINI速度/电流/故障/温度/电压反馈超时/越限 | DriveController | MCU ACK/feedback | drive+translation fault/stop | MCU人工恢复 | host→整机任务取消未联通 |
| 转向UID/模式/enable/feedback故障 | SteeringController | state FAULT/debug | stop all/translation reject | ClearFaultAndRestart/系统reset门禁 | README未标定描述与当前calibration=1不一致 |
| /cmd_vel源中断 | operator watchdog0.35s / MCU300ms | host状态/ACK | host zero / MCU timeout_stop | 新命令及安全门禁 | 外部持续publisher不受mux源撤销保护 |
| Piper CAN unplug/feedback stale | vendor RX + adapter freshness | ArmStatus/control_state/log | 抑制position、轨迹abort；gravity恢复路径 | vendor/adapter respawn，人工enable | 抑制发送≠实体急停或关节停稳证明 |
| Piper trajectory误差/超时/cancel | adapter tracking | FollowJointTrajectory.Result + CSV | 取消/abort路径的局部保持 | 调用方判断Result | Servo可继续写；没有global停止屏障 |
| reBot Servo timeout | ServoTrajectoryInput 0.15s | warning/ArmStatus | hold_current_position | 下一合法输入 | 只限制该入口，不覆盖SDK所有writer |
| reBot串口/SDK motor错误 | SDK/ROS局部异常与status | ArmStatus/per_joint_status/log | 局部异常/hold，未确认统一offline watchdog | driver重启/人工处理 | enabled变量可能滞后；SDKenable CallError可被print吞掉 |
| pressure串口打不开/拔出 | PressureSerialBridge | serial_connected=false | 采集停止；task局部pressure freshness | 2s重试/设备身份变更要求重启 | 已open但无样本仍connected=true；不是sample heartbeat |
| target/camera丢失 | pulse target年龄/稳定性或vision状态 | diagnostics/PlanningState | 部分precontact执行路径局部cancel | 等恢复/ERROR | 并无所有arm/base统一传播；相机驱动不代表target有效 |
| TF timeout/标定缺失 | PulseApproach/PiperPulseAlign lookup/preflight | log/blocker/error | 拒绝规划/执行 | 修正TF/标定（本轮不改） | 物理安装/跨planning frame一致性待测 |
| UI关闭/进程SIGINT | launch / destroy/shutdown | log | base零帧；reBot尝试safe_home、disable | 重启 | shutdown可能运动，不能作为ESTOP |
| 整机实体ESTOP | 电气输入/接线 UNKNOWN | ROS整机ESTOP接口 UNKNOWN | 独立硬件动力切断 UNKNOWN | 人工复位流程 UNKNOWN | 不得宣称已实现整机急停 |

证据：[`src/antbot_h743_bridge/antbot_h743_bridge/bridge.py:244`](../../src/antbot_h743_bridge/antbot_h743_bridge/bridge.py#L244)；[`src/antbot_h743_bridge/antbot_h743_bridge/bridge.py:402`](../../src/antbot_h743_bridge/antbot_h743_bridge/bridge.py#L402)；[`src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py:219`](../../src/antbot_h743_bridge/antbot_h743_bridge/operator_manager.py#L219)；[`firmware/rs00_fk743_test/firmware/App/Src/rs00_stm32_fdcan.c:322`](../../firmware/rs00_fk743_test/firmware/App/Src/rs00_stm32_fdcan.c#L322)；[`firmware/rs00_fk743_test/firmware/App/Src/chassis_translation_controller.c:527`](../../firmware/rs00_fk743_test/firmware/App/Src/chassis_translation_controller.c#L527)；[`firmware/rs00_fk743_test/firmware/App/Inc/drive_config.h:62`](../../firmware/rs00_fk743_test/firmware/App/Inc/drive_config.h#L62)；[`dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py:757`](../../dual_arm_ws/src/piperh_control/piperh_control/hardware_adapter.py#L757)；[`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/hardware_manager.py:175`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/hardware_manager.py#L175)；[`dual_arm_ws/third_party/reBotArm_control_py/reBotArm_control_py/actuator/rebotarm.py:239`](../../dual_arm_ws/third_party/reBotArm_control_py/reBotArm_control_py/actuator/rebotarm.py#L239)；[`src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py:162`](../../src/rebotarm_pulse/rebotarm_pulse/pressure_serial_bridge.py#L162)；[`src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py:596`](../../src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py#L596)。

## “模块坏了，上层仍认为READY”风险

目前没有整机READY可被统一撤销。更具体的误判途径：instance_guard ready仅是锁；web在线但parent可能跳过压力bridge；serial_connected是打开状态而非新鲜样本；latched /dual_arm/selected 和 armed 缺fresh heartbeat；reBot enabled是软件缓存，SDK enable失败可只print；base离线发布安全零joint_states用于RViz，不能作为底盘在线状态。任何Supervisor都应使用状态年龄、硬件反馈序号、故障字段和任务依赖；不能以窗口存在/topic有publisher/service存在/TF静态存在判READY。

## 建议故障链（待实现）

本地writer watchdog先停止/抑制并撤租约→状态适配器FaultReport→Supervisor取消受影响任务并锁相关资源→验证停止或升级ESTOP→人工确认故障消除后进入STANDBY重新初始化。急停分支禁止safe_home/retract；普通退出回撤也必须显式验证路径/反馈/控制权。租约过期停止策略需按关节重力/刹车能力确认，不能统一“disable全部电机”。用monotonic计deadline，ROS采集时间另报；进程重启epoch使旧命令失效。
