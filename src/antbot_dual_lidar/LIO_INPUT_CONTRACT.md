# 双雷达 LIO 输入契约（Phase 2A）

机器可读权威是 `config/lio_input_contract.yaml`。第一版输入严格为两路独立点云和
一颗底盘主 IMU：

```text
/antbot/lidar/front_left/points_raw_native
/antbot/lidar/rear_right/points_raw_native
/antbot/lidar/front_left/points_lio
/antbot/lidar/rear_right/points_lio
/antbot/imu/data
```

`/antbot/imu/data_raw` 是未修改的 Isaac 读数；`data` 是选择 ideal/realistic
profile 后的主输入。两路可选雷达 IMU 只用于交叉验证，未与主 IMU 融合，也绝不
交错发布到主话题。Phase 2B 只使用 `/antbot/imu/data`。

主 IMU frame 为 `imu_link`，Prim 为
`/antbot/Geometry/base_link/imu_link/imu_sensor`，目标频率 120 Hz，
SensorDataQoS（BEST_EFFORT/VOLATILE/depth 5），时间来自 Isaac 仿真时钟。
orientation 保留实值但 covariance[0]=-1，FAST-LIO 只消费角速度和线加速度。
reset/时间倒退时所有积分、位姿缓存和 realistic profile 的随机数状态必须清零。

两路增强点云字段实测为 `x/y/z/intensity/time_offset_ns`；
`points_raw_native` 是 RTX GMO `SENSOR/NONCOMPENSATED` 的逐 ray 本地点；
`points_lio` 是独立的未来算法候选话题，不再兼作 raw 语义标签。
`time_offset_ns` 是 ROS `PointField.INT32`（datatype 5），单位 ns。没有有效
ring、channel、emitter_id，禁止按点序号合成 ring。FAST-LIO2 的 direct/raw
points 模式可不依赖特征 ring，但适配器仍需把 `time_offset_ns` 转成目标实现所
要求的时间单位。

GMO 字段层面确认 offset 相对父 timestamp，但 A/C/D 唯一时间锚点不再由本轮
几何实验下结论，标记为
`DEFERRED_TO_REAL_HARDWARE_OR_LIO_RESEARCH`。`header_plus_offset` 仅保留为
验证工具的候选运行默认值，禁止据此宣称真实硬件/LIO 时间契约已经锁定。
`points_deskew_truth` 和 `points_world_reference` 均使用真值，只供 Phase 2A，
禁止作为真实机器人 LIO 的直接输入。

外参（xyz m / rpy rad）：

| 变换 | xyz | rpy |
|---|---|---|
| base → imu | `0 0 0.250` | `0 0 0` |
| imu → front | `0.322 0.222 0.164` | `0 0 0` |
| imu → rear | `-0.322 -0.222 0.164` | `0 0 π` |

`/antbot/ground_truth/odom` 只用于时间语义与去畸变验证，不得作为产品定位输入，也
不会覆盖 `/odom`。
