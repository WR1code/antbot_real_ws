# Piper-H 普通反馈重力补偿

本包不再包含主从臂/角色切换路径。示教与计算始终使用普通 Piper-H
`0x2A5/0x2A6/0x2A7` 反馈；当前对外话题为 `/piperh/joint_states`。驱动生成的
`/piperh/teach_joint_states_raw` 先经过硬件适配器验证，再以唯一录制入口
`/piperh/normal_joint_feedback` 提供给示教节点。

## 安全状态

默认配置同时关闭真实力矩、Piper-H MIT 真机确认和安装姿态确认，并故意不填写各轴最大
力矩。任何一项未完成时，`/piperh/gravity_compensation/start` 都会在建立 SDK 连接或发送
MIT 命令前失败。此时仍可失能机械臂并使用普通反馈录制。

## Dry-run

先由现场人员确认 `R_world_base` 的 RPY；不要根据“侧装”猜正负 90 度。确认后运行：

```bash
ros2 run piperh_control gravity_compensation_dry_run --ros-args \
  -p base_mount_confirmed:=true \
  -p base_mount_roll_deg:=<measured> \
  -p base_mount_pitch_deg:=<measured> \
  -p base_mount_yaw_deg:=<measured>
```

该进程只订阅 `/piperh/joint_states`，没有命令 publisher，也不创建 CAN SDK；它周期打印
`q`、`qd`、安装 RPY、世界/基座重力和 Pinocchio RNEA 力矩。

`R_world_base` 严格定义为 base 坐标系相对于 world 的旋转：

```text
g_world = [0, 0, -9.81]
g_base  = R_world_base.T @ g_world
```

在确认 dry-run 方向、当前固件的 MIT 真机行为和权威逐轴力矩限制以前，不得打开真实力矩
三个 gate。旧审计文件中的安装 RPY 是旧机械安装数据，不能用于当前机械臂。
