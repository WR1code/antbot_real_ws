# Phase 2A LIO 数据集

数据集使用 navigation 雷达 profile、ideal 主 IMU，并按 12 个动作分别录制。源码
仓库只保留清单和脚本，不提交 MCAP。

```bash
record_dual_lidar_bag.sh /data/antbot_lio 01_static
record_dual_lidar_bag.sh /data/antbot_lio 02_forward
```

其余合法 case 为 `03_backward`、`04_strafe_left`、`05_strafe_right`、
`06_yaw_left`、`07_yaw_right`、`08_diagonal`、`09_forward_yaw`、
`10_strafe_yaw`、`11_stop_go`、`12_reset_recovery`。每段建议 30–60 秒。

脚本记录主 IMU，不会把辅助 IMU混入主话题。若仿真已用
`--enable-lidar-imus` 启动，可设置 `ANTBOT_RECORD_AUX_IMU=1` 额外记录两路辅助
IMU；设置 `ANTBOT_RECORD_DESKEWED=1` 可记录真值去畸变输出。

每个 bag 旁会生成 capture 元数据、`ros2 bag info` 输出和 validation YAML。
