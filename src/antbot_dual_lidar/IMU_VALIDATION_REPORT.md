# IMU 验证报告

日期：2026-07-26。状态：核心静态/频率通过，运动与 GUI 矩阵 PENDING。

API 为 `isaacsim.sensors.experimental.physics.IMU.create` +
`IMUSensor.get_data(read_gravity=True)`，Prim：
`/antbot/Geometry/base_link/imu_link/imu_sensor`。这是 Isaac Sim 6.0.1-rc.7 的
非弃用 experimental physics 接口；旧 `isaacsim.sensors.physics` 示例位于
deprecated 目录，未使用。ROS 消息由 runner 直接转发 API 实测值，以避开本版本
ActionGraph/传感器组合的不稳定共享绑定。

实际 TF：

```text
base_link -> imu_link: xyz [0, 0, 0.250], rpy [0, 0, 0]
imu_link -> lidar_2d_front_scan: xyz [0.322, 0.222, 0.164], rpy [0,0,0]
imu_link -> lidar_2d_back_scan: xyz [-0.322,-0.222,0.164], rpy [0,0,pi]
```

实测结果：

| 配置 | 样本 | 平均 Hz | 最低 Hz | P95 周期 | P99 周期 | 回退/重复 |
|---|---:|---:|---:|---:|---:|---:|
| ideal 120 | 562 | 120.000 | 119.995 | 8.333445 ms | 8.333683 ms | 0/0 |
| ideal 60 | 179 | 60.000 | 59.999 | 16.666770 ms | 16.666890 ms | 0/0 |

最终默认 120 Hz。消息为 BEST_EFFORT/VOLATILE SensorDataQoS，静止重力
`linear_acceleration.z=+9.8100004 m/s²`，X/Y 约零。orientation 是 Isaac 实值，
但因没有标定误差模型标记 covariance[0]=-1，FAST-LIO 只使用角速度和线加速度。

ideal 不注入噪声/bias。realistic（固定 seed 360120）为 gyro density
0.0002 rad/s/√Hz、accel density 0.002 m/s²/√Hz、gyro bias
`[0.0005,-0.0003,0.0002] rad/s`、accel bias
`[0.01,-0.008,0.015] m/s²`。未实现 bias random walk。

可选辅助 Prim 为
`.../lidar_2d_front_scan/front_lidar_imu` 与
`.../lidar_2d_back_scan/rear_lidar_imu`，话题分别为
`/antbot/lidar/front_left/imu`、`/antbot/lidar/rear_right/imu`。realistic
短跑已确认两颗均创建（`auxiliary=2`）；它们没有进入主话题或联合融合。

左旋 z 正、右旋 z 负以及所有加减速/横移/暂停/reset 条目尚未执行，不能标记通过。
