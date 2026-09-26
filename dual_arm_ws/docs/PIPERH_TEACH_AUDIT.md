# Piper-H 普通反馈示教系统审计（2026-09-18）

## A. 原来的问题

旧路径在硬件适配器中切换机械臂角色，等待专用角色反馈帧并做姿态稳定检查；示教节点和
RViz 又把该专用反馈的 ready/stale 状态作为录制条件。真机不产生该流时，正常关节反馈
仍可用，但录制会被错误阻塞。

本次删除了角色切换后端、专用反馈 publisher/callback、姿态比较、稳定等待、watchdog、
UI ready 条件、测试和旧现场配置。默认示教路径不包含角色模式。

## B. 新架构

```text
Piper-H normal CAN 0x2A5/0x2A6/0x2A7
        ↓
piper driver: /piperh/teach_joint_states_raw
        ↓
hardware adapter validation: /piperh/normal_joint_feedback
        ├──── teach recorder (唯一录制入口)
        │
        └──── /piperh/joint_states ── dry-run gravity compensator
                                      ↓
                              Pinocchio RNEA (no command)
```

`teach_drag_mode` 支持 `passive_disabled`（安全默认）和 `gravity_compensation`。后者必须先
通过独立的真实力矩 gate；失败不会破坏被动录制路径。

## C. 关键文件

- `piperh_control/gravity_compensator.py`：旋转验证、`g_base`、Pinocchio RNEA、缩放和限幅。
- `piperh_control/gravity_dry_run.py`：只订阅普通 JointState；不存在命令 publisher/CAN SDK。
- `piperh_control/hardware_adapter.py`：正常反馈始终有效；转发唯一录制流；真实 MIT 三重 gate。
- `piperh_control/mit_backend.py`：只保留 Piper-H MIT 和只读固件查询，不含角色切换。
- `rebot_teach_mode/teach_node.py`：两种拖动模式，录制只读取 normal feedback。
- `rebotarm_demo_rviz/*`：UI 不再等待专用角色反馈。
- `config/gravity_compensation.yaml`：真实力矩、MIT 真机确认、安装姿态确认均默认为 false；
  力矩上限保持未配置的零值。

## D. 清理计数

对 `dual_arm_ws/src`（排除缓存和录制数据）扫描：

- `set_leader_mode`：0
- 专用角色反馈依赖：0
- `0x155/0x156/0x157`：0
- 专用角色 watchdog：0

## E. 普通反馈契约

- CAN：`0x2A5`=J1/J2，`0x2A6`=J3/J4，`0x2A7`=J5/J6。
- 顺序：严格 `joint1..joint6`，不按消息到达顺序猜测。
- 单位：raw 为 0.001 degree；使用 `0.017444 / 1000` 转成 rad；不改符号、零位或 offset。
- 时间戳：python-can/SocketCAN 接收时间；一组样本用三个 pair 时间戳中的最大值，并保留
  三个原始时间戳与组内 span。
- 去重/缓存：时间戳不前进不录制；不完整/乱序 cycle 被丢弃；完成一组后立即清空 cycle。
- publisher：驱动独占 raw 话题，适配器独占 `/piperh/normal_joint_feedback` 和
  `/piperh/joint_states`，示教节点不发布关节反馈。
- 频率：历史真机记录为 7617/38.080≈200 Hz；当前无 CAN，未重新实测。显示 JointState
  在适配器中上限 100 Hz，录制流不做降采样或重复填充。

## F. 重力补偿

- URDF：`piper_h_description/urdf/piper_h_description.urdf`，六个关节且含惯性参数。
- Pinocchio：ROS Jazzy 安装版本，使用 `pin.rnea(q, qd, zeros(6))`。
- `g_world=[0,0,-9.81]`。
- `R_world_base`：base frame 相对于 world frame 的旋转。
- `g_base=R_world_base.T @ g_world`；初始化时校验正交、det=+1 和范数 9.81。
- 侧装不修改关节零位、符号、offset 或 URDF 关节结构。
- 当前 launch 记录 roll=+90°, pitch=0°, yaw=0°，但 `gravity_base_mount_confirmed=false`；
  必须现场确认，不能把该值视为已验证重力方向。

## G. MIT 支持判断

结论：**UNKNOWN（真实力矩输出）**。

软件证据：已安装 `pyAgxArm 1.0.0`，`ArmModel.PIPER_H` 按 `S-V1.9-0` 路由到 v189，
公开 API 和 Piper-H demo 均包含 `move_mit`，输入力矩单位为 N·m。缺失证据：本机当前 CAN
入口为故障演练用 `piper_missing`，未验证本体固件响应、逐轴方向/增益、停止行为和权威
逐轴限值。因此 `gravity_real_torque_enabled=false`、`gravity_mit_support_confirmed=false`。

## H. Dry-run 状态

本次没有当前真机样本：CAN 不在线，且安装姿态未确认，因此没有伪造 Phase 4 结果。
离线管线回放旧实测 q（仅验证数学，不算当前真机 dry-run），按当前但未确认的
RPY=(+90°,0°,0°) 得到：

```text
q = [-0.0111875605, 2.9782123823, -2.7321384111, -0.2074323816,
     -0.3010343894, 0.1517389252]
qd = [0, 0, 0, 0, 0, 0]
gravity_base = [0, -9.81, -6.0069e-16]
gravity_torque = [5.7592976460, 0.0072687966, 0.0088001044,
                  0.6789277828, 0.0893506966, 0.5325406135]
```

旧审计文件使用另一安装姿态 (+90°,+90°,0°)，已经失效，不能作为当前方向证据。

## I. 当前安全状态

- PASSIVE RECORDING READY：YES（代码、构建和离线测试；需现场正常反馈/失能确认）
- GRAVITY COMPENSATION DRY-RUN READY：YES（执行前仍需填写并确认实际安装 RPY）
- REAL TORQUE OUTPUT READY：NO
