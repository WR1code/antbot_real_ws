# 模式切换审计与建议

审计日期：2026-09-15。静态源码审计；CONFIRMED 指实现/配置已发现，不代表真机在线。UNKNOWN / NEED_CONFIRMATION 不作为可执行配置。详见 [扫描范围](SOURCE_SCAN.md)。

现有系统支持手柄/键盘遥控模式、Xbox选臂和Teach局部状态，不支持`robot mode navigation/pulse`的完整事务。当前无真实odom，因此**不能兑现“确认odom速度接近0”**。MCU有all_drive_stopped/反馈稳定逻辑，未来可以先包装为明确的停稳报告，但必须验证关联新stop请求和反馈有效性；收到zero frame或软件cmd=0不代表停稳。

## NAVIGATION → PULSE_PRECONTACT（推荐Action事务）

| Step | 动作 | 成功证据 | timeout/失败 |
|---|---|---|---|
| 0 | 设置SWITCHING、新epoch，冻结新导航goal和臂goal | 所有最终writer进入切换门禁 | 任一writer未知→FAULT锁定 |
| 1 | cancel Nav2 NavigateToPose/ThroughPoses与active controller任务 | cancel返回+goal最终CANCELED/终止，禁止late output | cancel超时→拒绝授臂资源 |
| 2 | BASE arbiter拥有STOP入口、连续zero并撤navigation租约 | 最终输出源为STOP，旧epoch命令不再转发 | UART/CAN未知→FAULT |
| 3 | 验证停稳与锁定 | 新鲜odom twist线/角速度阈值稳定窗口+MCU无故障；或已验收MCU停止报告 | 缺odom/有效反馈时不能放行自动流程 |
| 4 | 使camera/vision依赖健康 | Image/Depth/Info/TF/target新鲜且质量合格 | camera/TF/target超时保持BASE锁，task失败 |
| 5 | 原子acquire目标臂；确认另一臂safe | lease epoch、臂新鲜反馈、enable/模式、Xbox LOCKED | 不隐式自动enable；失败释放已取臂租约 |
| 6 | 执行预接触任务（未来完整pulse task） | task feedback/Result，pressure样本fresh、zero和阈值检查 | pressure超限/缺样本/反馈丢失→取消臂、local stop |
| 7 | 任务正常完成后有条件撤回 | 有效TF/臂反馈/碰撞检查+回撤goal成功+速度稳定 | 不能撤回→保留BASE锁，FAULT；不可贸然release |
| 8 | release臂，确认safe，再释放BASE STOP | 完成回撤、没有active Action/stream/gravity | 超时不恢复导航 |
| 9 | 人工/策略允许navigation reacquire | BASE新lease、arms safe、Nav2/odom健康 | READY或STANDBY，拒绝自动复活旧goal |

超时和停稳阈值需依据实机反馈率/制动行为验收确定，全部UNKNOWN / NEED_CONFIRMATION，不在文档写假标定数值。每一步Result带step、reason、remaining owners、faults；可取消切换按安全反向清理，但ESTOP绝不自动回撤。

## MANUAL / TEACH

MANUAL独占目标资源且底盘与臂协作策略显式确定；TEACH要求BASE锁定停稳，selected臂及driver模式匹配，另一臂safe。官方Piper Leader/follower切换需完成恢复与新反馈后才授普通trajectory；不能只等Xbox armed=false。计划可以预先生成，执行前重新检查目标年龄、joint状态和ownership，避免旧计划跨模式执行。

## 当前代码距离目标的缺项

1. final writer全入口owner检查；2. BASE停稳接口和真实odom；3. cancel结果与硬件终态屏障；4. arm safe/retract可验证动作；5. sensor heartbeat与统一fault传播；6. 长任务Action封装与失败补偿；7. 状态/模式CLI；8. ESTOP独立链及退出分支。

证据：[`dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/active_arm_manager.py:62`](../../dual_arm_ws/src/rebot_xbox_hardware/rebot_xbox_hardware/active_arm_manager.py#L62)；[`firmware/rs00_fk743_test/firmware/App/Inc/chassis_translation_controller.h:47`](../../firmware/rs00_fk743_test/firmware/App/Inc/chassis_translation_controller.h#L47)；[`firmware/rs00_fk743_test/firmware/App/Inc/drive_config.h:80`](../../firmware/rs00_fk743_test/firmware/App/Inc/drive_config.h#L80)；[`src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py:788`](../../src/rebotarm_pulse/rebotarm_pulse/piper_pulse_align.py#L788)；[`dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/hardware_manager.py:175`](../../dual_arm_ws/src/rebotarmcontroller/rebotarmcontroller/hardware_manager.py#L175)。
