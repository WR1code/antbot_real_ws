# 双 Livox MID360S 接入 SLAM / Nav2 接口审计

审计日期：2026-09-17（Asia/Shanghai）  
工作区：`/home/w/project/antbot_real_ws`  
状态：阶段 0/1/2 完成；阶段 3 已完成单雷达 FAST-LIO 静止初始化短测，标准
`odom -> base_link` 因当前工作台 TF 冲突而由 preflight 禁止；阶段 4--5 未启动

## 1. 结论先行

当前工程具备双 MID360S 驱动、逐雷达 frame relay、保字段裁剪/TF/融合、
`slam_toolbox`、Nav2 和 `robot_localization`，但**尚不具备安全启动实机 SLAM/Nav2
闭环的完整条件**：

1. 当前底盘没有 `/odom` 发布者，也没有 `odom -> base_link`；H743 桥只发布
   `/joint_states`、电池和状态，未实现轮式里程计。
2. 已在独立 overlay 构建 ROS 2 FAST-LIO2 候选并完成 MID360S CustomMsg + IMU 静止
   初始化短测。Cartographer、LIO-SAM、Point-LIO、KISS-ICP 和 RTAB-Map 仍未安装；
   `slam_toolbox` 仍只是 2D `LaserScan` 路径。
3. 当前 Nav2 实机参数仍面向旧的 `/scan_0_fixed`、`/scan_1_fixed`，不能直接视为
   MID360 接口。
4. 当前运行的双臂工作台发布静态 `world -> base_link`；若直接增加
   `odom -> base_link`，`base_link` 会出现两个父节点。双臂 launch 已增加
   `publish_world_to_base` 开关，默认为 true 以保持旧工作流；建图模式必须显式设为
   false。操作台的 `map -> base_link` 占位 TF 也必须关闭。
5. Livox PointCloud2 保留逐点 `timestamp`。在线实测表明它是绝对纳秒值，代表帧中
   每个点的设备时间；代表帧首点与 header 时间相等，末点约晚 100 ms。
   目标 LIO 仍必须显式映射该字段，不得使用忽略它而按方位角伪造时间的
   PointCloud2 路径。
6. Livox 原始 IMU 的 `frame_id` 都是 `livox_frame`，且加速度单位是 g。已增加薄适配层，
   输出唯一 frame 和 SI 单位，并保留原始设备时戳。两个 IMU frame 到
   `base_link` 的物理外参仍未测量，适配层没有猜测 TF。
7. `.116 -> front_left`、`.139 -> rear_right` 只作为本轮确定性测试映射；物理身份
   尚未由外部标识确认，不能固化到 URDF、多份配置或永久 TF。

所以当前最小风险路线不是把 `/mid360/merged` 直接接进某个 SLAM，而是：先恢复
阶段 1 在线验证；随后选择**单颗 MID360 + 与它同机身的 IMU**建立首个 LIO 基线，
用其连续轨迹分别 deskew 两颗雷达，再做双雷达融合；Nav2 障碍层与 3D LIO 输入分开。

## 2. 本机软件与工程资产

### 2.1 已安装并可解析

| 组件 | 版本/位置 | 结论 |
|---|---|---|
| ROS 2 | Jazzy, `/opt/ros/jazzy` | 可用 |
| `livox_ros_driver2` | tag `1.2.6`，`/home/w/project/odas/.deps/mid360s_ws/install/livox_ros_driver2` | 可用，必须先 source 驱动 overlay |
| Livox-SDK2 | v1.3.1，`/home/w/project/odas/.deps/livox-sdk2-install` | 可用，运行时需加入 `LD_LIBRARY_PATH` |
| `slam_toolbox` | 2.8.5 | 可用，仅 2D `LaserScan` |
| Nav2 | 1.3.12 | 可用 |
| `robot_localization` | 3.8.3 | 可用，但当前未启动且没有可用 wheel odom 输入 |
| `fast_lio` | `Ericsii/FAST_LIO_ROS2` commit `2fffc570`, `/home/w/project/odas/.deps/fast_lio_ros2_ws` | Jazzy 构建通过；Livox CustomMsg 逐点时间路径实测可初始化 |
| `antbot_mid360_fusion` | 工作区 overlay | 可用，参数和 30 ms 同步阈值保持不变 |
| `antbot_dual_lidar` | 工作区 overlay | frame relay 可用；其多数 LIO文档/工具来自 Isaac 仿真阶段，不能冒充 MID360 实机验证 |

初始审计时以下包未找到：Cartographer、LIO-SAM、FAST-LIO/2、Point-LIO、
KISS-ICP、RTAB-Map。后续按任务要求选定 ROS 2 FAST-LIO 并放入独立依赖 overlay；
没有下载第二套 Livox driver。

### 2.2 已有 launch/config

- `src/antbot_mid360_fusion/launch/dual_mid360_fusion.launch.py`：只启动 crop/TF/
  merge，可独立关闭 RViz；默认输入为两路 relay topic。
- `src/antbot_mid360_fusion/config/dual_mid360_fusion.yaml`：`min_range=0.10 m`、
  `max_range=100 m`、`blind_width=90 deg`、`sync_tolerance=30 ms`，本次未修改。
- `src/antbot_navigation/launch/slam.launch.py`：启动 `slam_toolbox`，实机默认使用
  `/scan_0_fixed`，并假定已有 `/odom` 和 `odom -> base_link`。
- `src/antbot_navigation/launch/localization.launch.py`：启动 map server + AMCL；AMCL
  是 `map -> odom` 的唯一预期发布者。
- `src/antbot_navigation/launch/navigation.launch.py`：会一并启动 localization 和
  Nav2，实机仍创建旧 2D scan fix relay；不应在 MID360 阶段 1--3 使用。
- `src/antbot_navigation/config/real/nav2_params.yaml`：局部/全局 costmap 都使用旧
  `/scan_0_fixed`、`/scan_1_fixed`；AMCL 只用前路 scan。
- `src/antbot_navigation/config/real/slam_toolbox_params.yaml`：仍写有 COIN D4 的量程
  注释和 `/scan_0` 默认值，不是 MID360 已验证参数。
- `src/antbot_navigation/config/real/ekf.yaml`：文件存在，但当前 real launch 不启动
  EKF；不能把“有配置文件”等同于“有定位输出”。

## 3. 当前 ROS 接口审计

本节在线结果采集于 2026-09-17 18:14--18:18，采集时双臂/操作台部分节点正在运行，
雷达未在线。

| 接口 | 在线状态 | 发布责任/问题 |
|---|---|---|
| `/joint_states` | 1 个发布者 | `h743_cmd_vel_bridge`；有转向和车轮关节位置，不等于 wheel odometry |
| `/odom` | topic 不存在 | **缺失** |
| `/odometry/filtered` | 0 发布者、4 订阅者 | 占位接口，不是可用里程计 |
| `/scan_0`, `/scan_1` | 各 0 发布者 | 只被 RViz 订阅；旧接口当前无数据 |
| `/imu/accel_gyro` | 未在线 | 板载 `antbot_imu` 源码存在，配置目标 200 Hz、frame `imu_link` |
| 两路 Livox lidar/IMU | 未出现 | 本轮雷达无法通信，见阶段 1记录 |
| `base_link -> lidar_2d_front_scan` | 在线 | `[0.322, 0.222, 0.414]`, RPY `[0,0,0]`，与既有外参一致 |
| `base_link -> lidar_2d_back_scan` | 在线 | `[-0.322,-0.222,0.414]`, RPY `[0,0,pi]`，与既有外参一致 |
| `odom -> base_link` | 不存在 | **SLAM/Nav2 阻塞项** |
| `map -> odom` | 不存在 | 当前没有 SLAM/AMCL，应当不存在 |
| 静态 `world -> base_link` | 在线 | 来自 `rebotarm_moveit_world_tf`，将与未来 `odom -> base_link` 冲突 |
| 静态 `world -> map` | 在线 | `integrated_world_to_map_tf`；与导航组合时需统一全局 frame 策略 |

另外，ROS graph 报告存在同名节点（主要是 Servo/MoveIt 私有节点）。这不是本次
LiDAR 数据问题的证据，但在完整 Nav2 联调前应清理启动实例，避免把旧工作台 graph
当成导航 graph。

## 4. 点云与 IMU 输入合同

### 4.1 PointCloud2

实测 schema 与驱动 1.2.6 源码一致：

| 字段 | offset | 类型 |
|---|---:|---|
| `x`, `y`, `z`, `intensity` | 0, 4, 8, 12 | float32 |
| `tag`, `line` | 16, 17 | uint8 |
| `timestamp` | 18 | float64 |

`point_step=26`。驱动在 PointCloud2 路径直接写入
`double(pkg.points[i].offset_time)`；在 CustomMsg 路径则写入
`uint32(points[i].offset_time - pkg.base_time)`。在线数据已确认 PointCloud2 字段是绝对纳秒：

- 代表帧的首点与 header 纳秒值相等，末点晚约 99.88--100.31 ms；
- 两路 header 在线 10 s 观测中没有回退或重复；目标 LIO 还需负责将绝对 ns 转为其
  预期的相对时间；
- frame relay 和现有 fusion 会保留未知字段/原始记录，这是正确的；
- `/mid360/merged` 只是空间变换与近似配对，不是 deskew；云间 header delta 很小
  不能替代逐点运动补偿。

### 4.2 Livox IMU

驱动 multi-topic 会创建 `/livox/imu_<IP>`，消息 stamp 直接来自设备 IMU 时间，
包含角速度和线加速度；orientation/covariance 没有在驱动中填充。驱动源码将所有
IMU 消息的 frame 固定为 `livox_frame`，没有使用全局 `frame_id` 参数。

原始 IMU 实测约 200 Hz、stamp 单调，加速度单位为 g。新增 relay 输出
`/antbot/lidar/front_left/imu_raw_si` 和
`/antbot/lidar/rear_right/imu_raw_si`；加速度乘 `9.80665`，角速度保持
rad/s，orientation 标记为未知。在线抽样的 834 条规范化消息全部能找到完全相同的
原始 stamp。首个单雷达 LIO 基线暂使用 `.116` 云和它同源的规范化 IMU；
物理标识确认前不得永久化该映射。

### 4.3 板载 IMU

`antbot_imu` 源码发布 `/imu/accel_gyro`，使用 `imu_link`、目标 200 Hz，消息时间为
主机 `now()`。当前节点未运行。其校准线程在收到 `/odom` 且确认静止前不会完成，
而当前 `/odom` 缺失，所以它现在不是即插即用的 LIO 主 IMU。它更适合在修复启动
依赖、验证硬件时间和外参后作为底盘 EKF 候选；不能只因 frame 更规范就优先于
Livox 内置 IMU。

## 5. 推荐接入架构

### 5.1 3D LIO / SLAM 数据面

首个可验证基线采用单雷达，不把两路云交错伪装成一台雷达：

```text
MID360 A raw PointCloud2 + MID360 A IMU
  -> 薄适配层（独立 lidar/imu frame；只做字段/单位映射）
  -> 目标 LIO（必须支持非重复扫描、无 ring direct 模式、逐点时间、IMU deskew）
  -> 连续局部里程计/轨迹
  -> 分别 deskew A、B 两路原始云
  -> 可选双雷达空间融合
```

选定的 ROS 2 FAST-LIO 只在 Livox `CustomMsg` 路径上接入 MID360S：直接使用
`offset_time`，不使用该 fork 中会按 yaw 伪造时间的 MID360 PointCloud2 handler。
它已在 Jazzy 构建，IMU subscriber 改为 SensorDataQoS，上游 TF 广播可关闭；
AntBot 适配器是唯一预期的 `odom -> base_link` owner。

若 LIO 自己发布 `odom -> base_link`，底盘/EKF不得再发布同一 TF；若 LIO只输出
Odometry topic，则由唯一 EKF/适配节点发布 `odom -> base_link`。若 LIO发布
`map -> base_link` 的单段 TF，需要先设计拆分策略，不能同时启动 SLAM Toolbox 或
AMCL 来发布 `map -> odom`。

### 5.2 2D Nav2 障碍物数据面

Nav2 costmap 输入与 3D LIO 输入分离。推荐从**各自 deskew 后**的雷达云生成障碍
输入，而不是把未 deskew 的 `/mid360/merged` 直接送入：

- 方案 A：每路 deskew cloud 直接作为 Nav2 `ObstacleLayer`/`VoxelLayer` 的
  `PointCloud2` observation source；
- 方案 B：每路 deskew cloud 做高度切片后生成两个 `LaserScan`，继续复用二维
  costmap；
- 静态地图定位时由 AMCL 唯一发布 `map -> odom`；在线 2D 建图时由
  `slam_toolbox` 唯一发布 `map -> odom`，两者不得同时运行。

现有 `/scan_0_fixed`、`/scan_1_fixed` 是旧 COIN D4 合同。MID360 输出应使用新 topic
名和独立参数文件，避免悄悄改变旧工作流。deskew/time 语义未通过阶段 2 前，不创建
这个永久适配。

### 5.3 TF 发布责任

| TF | 唯一发布者 |
|---|---|
| `base_link -> lidar_2d_front_scan/back_scan` | `antbot_robot_state_publisher` |
| LiDAR IMU frame 的静态外参 | 校准后由同一 robot description 发布；当前不新增 |
| `odom -> base_link` | LIO 或 wheel/EKF 二选一的唯一 owner |
| `map -> odom`（建图） | `slam_toolbox` 或选定 3D SLAM 的唯一 owner |
| `map -> odom`（地图定位） | AMCL 或选定 3D localization 的唯一 owner |

启动上述 TF 前必须关闭 `world -> base_link` / `map -> base_link` 的工作台占位 TF。
`operator_step2.launch.py` 已有 `publish_placeholder_pose` 开关；双臂
`dual_arm_hardware.launch.py` 已增加 `publish_world_to_base` 开关。建图模式必须将两者都设为
false，并用 preflight 确认当前 graph 不再存在直接 `world/map -> base_link`。

## 6. 分阶段实机验证记录

### 阶段 1：通过

执行内容：

- 保留 Wi-Fi 默认路由；临时 NetworkManager profile 让 `eno1` 同时持有
  `10.42.0.1/24` 和 `192.168.1.50/24`；
- `.116`、`.139` 均 2/2 ping 成功，网卡 drop/error 没有异常；
- 使用既有 `dual_mid360_actual.json` 启动 driver 1.2.6，参数为
  `xfer_format=0`、`multi_topic=1`、10 Hz；
- 未启动 RViz、SLAM 或 Nav2。

结果：两路 raw cloud 约 9.86 Hz，两路 relay 约 9.86--10.00 Hz，frame 分别是
`lidar_2d_front_scan` / `lidar_2d_back_scan`。两路 IMU 约 200.03 Hz，header stamp 无
回退/重复。PointCloud2 schema 及 `point_step=26` 与合同一致。

额外短时 fusion 检查中，`/mid360/merged` 约 9.63 Hz，平均 23,320 点/帧，
frame 为 `base_link`；sync success 138、miss/drop 各 1，平均处理耗时 5.29 ms。
`sync_tolerance=30 ms` 和 crop 参数未改动。

阶段 1 启动器现在还可选启动 IMU 单位/frame relay，不发布任何猜测外参。
测试进程全部退出，临时 profile 已删除；`eno1` 已恢复为仅
`10.42.0.1/24`，默认路由仍走 Wi-Fi。

### 阶段 2：输入合同部分通过，LIO 合同未通过

已在不启动 SLAM 的条件下验证字段、时间语义和 IMU 单位。已通过项：

- 两路 cloud 和 IMU 各自持续发布；
- cloud/IMU frame 经 relay 后唯一且与 URDF 匹配；
- 逐点时间单位、锚点、跨度和 header 单调性有实测结论；
- IMU 原始 g 单位已转 SI，唯一 frame 和原时戳保留已在线验证；
- 单路 orphan 没有通过放大 30 ms 阈值遮盖。

已通过项：FAST-LIO Livox CustomMsg handler 使用真实 `offset_time`，单雷达 +
规范化 IMU 能完成 deskew 初始化。抽样 15 帧的约 100.08--100.39 ms 扫描区间均有
完整 IMU buffer 覆盖。尚未通过项为实体前后标识和人工左/右 yaw 符号验证。

### 阶段 3：单雷达 LIO 静止短测通过，标准 TF 输出被安全拦截

FAST-LIO 在 Jazzy 上成功构建，使用 `.116` CustomMsg 和
`/antbot/lidar/front_left/imu_raw_si` 完成 `IMU Initial Done` 及 k-d tree 初始化。
`/Odometry` 以 `odom -> mid360_front_imu` 约 9.999 Hz 输出；10 s 静止窗口端点位移
5.42 mm，最大相对位移 10.07 mm。FAST-LIO 约 7.2% 单核 CPU，RSS 约 172 MiB。

正式入口的 preflight 在所有雷达/IMU/time/TF/磁盘/LIO 检查通过后，精确检出当前
`world -> base_link` 直接父节点并拒绝启动 FAST-LIO/适配器。为避免 TF 冲突，上述
LIO 短测关闭了上游 TF，没有启动标准 `/odom` 适配器。

### 阶段 4--5：未启动

没有启动 Nav2 或 RViz，没有保存正式地图。建图入口已生成，但在当前工作台运行状态下
会 fail-closed，不宣称 PRE-MAPPING READY。

## 7. 当前可执行入口

环境加载：

```bash
source /opt/ros/jazzy/setup.bash
source /home/w/project/odas/.deps/mid360s_ws/install/setup.bash
source /home/w/project/antbot_real_ws/install/setup.bash
export LD_LIBRARY_PATH=/home/w/project/odas/.deps/livox-sdk2-install/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
```

雷达恢复在线后，且 `eno1` **同时**具有 `10.42.0.1/24` 与 `192.168.1.50/24`、
Wi-Fi 仍承担默认路由时，优先使用只读网络前置检查的阶段 1 入口：

```bash
ros2 launch antbot_mid360_fusion mid360_stage1.launch.py \
  front_ip:=192.168.1.116 rear_ip:=192.168.1.139
```

它只运行既有 driver、cloud frame relay 和可选 IMU 单位/frame relay，IP 映射仅由
本次命令参数提供；不启动
fusion、robot_state_publisher、SLAM、Nav2 或 RViz。现有 description 节点应保持
唯一发布两路 LiDAR 外参。测试结束后需移除**仅新增**的雷达测试地址，恢复原网络。
以下是与 launch 等价的 driver 命令，供独立故障排查：

```bash
ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
  -p xfer_format:=0 -p multi_topic:=1 -p data_src:=0 \
  -p publish_freq:=10.0 -p output_data_type:=0 \
  -p frame_id:=livox_frame \
  -p user_config_path:=/home/w/project/antbot_real_ws/benchmark/ubuntu_pc_actual/runtime/dual_mid360_actual.json
```

frame relay 需要显式传入本轮临时 IP 映射，避免依赖其旧 `.12/.13` 默认值：

```bash
ros2 run antbot_dual_lidar livox_frame_relay --ros-args \
  -p front_left.input_topic:=/livox/lidar_192_168_1_116 \
  -p rear_right.input_topic:=/livox/lidar_192_168_1_139
```

只有阶段 1/2 通过后才启动现有 fusion：

```bash
ros2 launch antbot_mid360_fusion dual_mid360_fusion.launch.py \
  stage:=merge use_rviz:=false
```

统一建图入口已为：

```bash
source /home/w/project/odas/.deps/fast_lio_ros2_ws/install/setup.bash
ros2 launch antbot_mapping antbot_mapping.launch.py \
  front_ip:=192.168.1.116 rear_ip:=192.168.1.139 use_rviz:=false
```

它在当前 `world -> base_link` 存在时会拒绝进入 mapping；这是验证过的安全行为。

## 8. 风险与缺失项清单

- **阻塞**：当前工作台仍在发布 `world -> base_link`；未在 mapping-safe 模式重启前，
  preflight 会禁止标准 `/odom` 和 `odom -> base_link` 输出。
- **冲突**：当前静态 `world -> base_link` / 工作台占位 TF 与未来导航 TF 树不兼容。
- **待验证**：`.116/.139` 的物理前后身份。
- **待适配**：目标 LIO 必须正确映射 PointCloud2 绝对 ns 逐点时间；不能丢弃。
- **待实物确认**：IMU-to-LiDAR 外参来自 MID360S 官方手册，并已与现有
  `base_link -> lidar` 链相乘；但 `.116/.139` 物理身份和人工 yaw 符号尚未独立验证。
- **待选择**：Livox 内置 IMU 与板载 IMU 的噪声、时钟、外参和启动可用性。
- **兼容性**：现有 real Nav2/slam_toolbox 配置仍属于旧 2D LiDAR 合同。
- **可靠性**：orphan 不能通过扩大 30 ms 阈值处理；应在 driver/DDS/relay/fusion
  边界计数，并让后续 LIO/障碍链路容忍单路短暂缺帧。
- **数据正确性**：merged cloud 保留 timestamp 字段不等于已 deskew；移动状态下不
  得把 cloud-to-cloud 同步代替逐点补偿。
- **TF 所有权**：SLAM、AMCL、LIO、EKF、底盘控制器不得重复发布
  `map/odom/base_link` 相关 TF。

## 9. 本轮软件验证

- `antbot_dual_lidar` 与 `antbot_mid360_fusion` 已重新构建成功；新增 IMU relay
  单元测试 1/1 通过，fusion C++ 单元测试 5/5 通过。
- `mid360_stage1.launch.py --show-args` 通过；front/rear IP 没有默认值，避免把
  临时物理映射误写成永久配置。
- 在当前仅有 `10.42.0.1/24` 的安全网络状态下，阶段 1 launch 以错误码 1 拒绝，
  未创建 Livox driver/relay 进程；`eno1`、Wi-Fi 默认路由和 NetworkManager 配置
  保持不变。
- 在线阶段 1 和 fusion 短测均已结束；没有启动 RViz、SLAM 或 Nav2，没有遗留
  Livox/relay/fusion 进程或临时网络 profile。
