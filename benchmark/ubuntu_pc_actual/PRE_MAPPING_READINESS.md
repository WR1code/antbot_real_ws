# PRE-MAPPING READINESS

双MID360：  
PASS

雷达IP：  
front: `192.168.1.116`（当前确定性测试映射，物理标签待独立确认）  
rear: `192.168.1.139`（当前确定性测试映射，物理标签待独立确认）

雷达Hz：  
PointCloud2 阶段约 9.86 Hz；FAST-LIO CustomMsg preflight 两路均 10.00 Hz

IMU：  
FAIL（单位、频率、时间戳、静止重力已通过；人工左右 yaw 轴向/符号测试未做）

Primary IMU：  
`/antbot/lidar/front_left/imu_raw_si`

IMU Hz：  
约 200.00 Hz

Per-point timestamp：  
PASS（PointCloud2 为绝对 ns；mapping 使用 CustomMsg `offset_time`，扫描跨度约 100 ms）

Extrinsics：  
PASS（已有 robot-to-LiDAR TF 未改；LiDAR-to-IMU 使用 MID360S 官方手册值）

TF ownership：  
FAIL（设计与适配器已完成，但当前 live graph 仍存在 `world -> base_link`）

LIO package：  
ROS 2 FAST-LIO2, `Ericsii/FAST_LIO_ROS2` commit `2fffc570`

LIO build：  
PASS（ROS 2 Jazzy 独立 overlay）

LIO initialization：  
PASS（`IMU Initial Done`，k-d tree initialized）

Odometry：  
PASS for `/Odometry` at 9.999 Hz; standard `/odom` and `odom -> base_link` are intentionally gated

world->base_link conflict：  
NOT RESOLVED in the currently running workbench graph; mapping-safe launch switches exist

Mapping launch：  
READY and fail-closed; it will not start LIO while a critical preflight item fails

Preflight：  
FAIL only on the current direct `world -> base_link`; sensor/time/TF/config/binary/disk checks passed

仍存在的阻塞：

1. 以 mapping-safe 参数重启工作台，关闭 `world -> base_link` 和占位 `map -> base_link`，再跑完整 preflight。
2. 用外部标签确认 `.116 -> front_left`、`.139 -> rear_right` 的物理身份。
3. 人工缓慢左右 yaw，确认主 IMU gyro Z 与 LIO yaw 符号。

STATUS:  
NOT READY TO MAP（已准备到可进行最后三项短时验收）

下一条唯一命令（完成上述人工确认且 `eno1` 同时具有两个地址后）：

```bash
ros2 launch antbot_mapping antbot_mapping.launch.py \
  front_ip:=192.168.1.116 rear_ip:=192.168.1.139 use_rviz:=false
```
