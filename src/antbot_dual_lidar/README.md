# AntBot 双 RTX 3D LiDAR 与 Phase 2A LIO 传感器准备

本包为 Isaac Sim 双雷达提供两路 `PointCloud2` 预处理、诊断、有界体素累计、
PCD 持久化、RViz、MCAP 录制/回放及 Nav2 参数接口。它不启动 Isaac Sim，
不实现 LIO，也不启动真实 Livox 驱动。`/scan_0`、`/scan_1` 可由双 3D 雷达
水平切片生成，底盘控制链路保持不变。

## 已审计事实

原场景使用两个 RTX LiDAR，配置都是 `Example_Rotary_2D`。该配置只有一个
发射器/通道，固定 elevation 为 `-2°`，输出 `LaserScan`，所以原场景不是双
3D 雷达。

新增的派生场景是：

```text
/home/w/project/antbot/isaac/usd/antbot_dual_lidar_test.usd
```

它从 `antbot_lidar_fixed_control.usd` 复制后生成，不覆盖 raw、validated 或
control USD。两个 3D RTX LiDAR Prim 为：

```text
/antbot/Geometry/base_link/lidar_2d_front_link/lidar_2d_front_scan/rtx_lidar
/antbot/Geometry/base_link/lidar_2d_back_link/lidar_2d_back_scan/rtx_lidar
```

保留 `lidar_2d_*` 名称是为了复用已经稳定的 URDF/USD link 和 TF，不新增同位姿
别名，也不重复发布静态 TF。`rtx_lidar` 是各自 scan link 的 identity 子 Prim。

从 URDF 和生成后的 USD 读取到的 `base_link` 外参是：

| 传感器 | frame_id | xyz (m) | rpy (rad) |
|---|---|---:|---:|
| 前左光学原点 | `lidar_2d_front_scan` | `0.322 0.222 0.414` | `0 0 0` |
| 后右光学原点 | `lidar_2d_back_scan` | `-0.322 -0.222 0.414` | `0 0 π` |

其中实体雷达 link 的 z 为 `0.344 m`，scan link 再沿 z 增加 `0.070 m`。

派生 3D 场景复用 Isaac Sim 自带的通用仿真配置 `Example_Rotary`。这些是测试
参数，不是 MID-360 参数：

| 参数 | 值 |
|---|---|
| 类型 | Isaac Sim RTX rotary 3D LiDAR |
| 水平视场 | 360° |
| 垂直视场 | -15° 至 +10°（25°） |
| 通道/发射器 | 128（32 个不同 elevation，4 组方位交错） |
| 垂直角间隔 | 约 0.806° |
| 水平角采样 | 约 0.1°（36 kHz / 10 Hz） |
| 扫描频率 | 10 Hz |
| 量程 | 0.10–20.0 m（项目覆盖） |
| 最大回波 | 2 |
| 强度 | 配置支持 normalized scalar，发布为 `float32 intensity` |
| 时间 | GMO 仿真纳秒时间；同一时间源发布 `/clock` |

验证三维性不能只看消息类型：用 `ros2 topic echo --once ...` 检查 fields 中有
`x/y/z`，再在 RViz 中观察桌面、低矮障碍和立柱是否产生不同 z 的回波；启动脚本
第一次发布时也打印 `z_span_m`。如果 z 基本恒定，应检查是否误用了 2D 模式。

## 数据链路和 TF 责任

```text
Isaac Simulation Time -> /clock
front RTX GMO -> /antbot/lidar/front_left/points
rear  RTX GMO -> /antbot/lidar/rear_right/points
                   |
                   v  tf2 + range/height/voxel/body filters
             .../points_filtered  (frame_id=base_link)
```

两颗 RTX 传感器分别拥有 render product 和 GMO writer，不共享发布端点。现有
`/antbot/ROS_Control_Graph` 保持不变，其中 `SimTime -> PublishClock` 发布
`/clock`，并保留 joint-state/control 节点。rc.7 的两路点云由各自的 Replicator
GMO adapter 发布（不伪装成 ActionGraph 节点），以避开该版本 ROS helper 与 GMO
共享绑定问题。

TF 责任如下：

- `odom -> base_link`：现有 Isaac ground-truth odometry publisher；
- `base_link -> lidar_2d_front_scan`、`base_link -> lidar_2d_back_scan`：
  `robot_state_publisher` 从唯一 URDF 发布；
- `map -> odom`：本阶段不发布；只有后续 SLAM/定位节点才应发布；
- 车轮/转向 link：仍由现有 `robot_state_publisher` 和 `/joint_states` 链路负责。

不要让 Isaac 和 `robot_state_publisher` 同时发布相同的静态传感器 TF。

## 启动

终端 1 启动派生 3D 仿真（默认不会覆盖既有 validated USD）：

```bash
cd /home/w/Desktop/isaac-sim-standalone-6.0.1-linux-x86_64
./python.sh /home/w/project/antbot/isaac/scripts/run_antbot_dual_lidar.py \
  --lidar-mode 3d --lidar-profile navigation --test-layout demo
```

终端 2 启动现有控制/robot state publisher，确保只存在一个该节点。终端 3：

```bash
source /opt/ros/jazzy/setup.bash
source /home/w/project/antbot/ros2_ws/install/setup.bash
ros2 launch antbot_dual_lidar dual_lidar_bringup.launch.py \
  use_sim_time:=true use_rviz:=true lidar_profile:=navigation
```

可覆盖参数包括 `use_sim_time`、`use_rviz`、`start_preprocessing`、
`start_diagnostics`、`front_lidar_topic`、`rear_lidar_topic`、`target_frame`、
`record_bag`、`bag_root`、`rviz_config`、`config_file`、`lidar_profile`、
`start_synchronizer`、`sync_strategy` 和 `publish_synchronized`。

## 两种运行模式

| profile | scan/tick | firing | range | emitters | voxel | 实测用途 |
|---|---:|---:|---:|---:|---:|---|
| mapping | 10 Hz | 36000 Hz | 0.10–20 m | 128 | 0.03 m | 高密度采集，约 333k raw 点/帧 |
| navigation | 10 Hz | 3000 Hz | 0.15–12 m | 128 | 0.04 m | 实时避障，约 27.8k raw、9.8k filtered 点/帧 |

模式集中定义在 Isaac runner 的 `LIDAR_PROFILES`，ROS 侧用同名
`lidar_profile` 选择预处理默认值。两个新派生场景分别是
`antbot_dual_lidar_mapping.usd` 和 `antbot_dual_lidar_navigation.usd`；原阶段 1
的 `antbot_dual_lidar_test.usd` 不覆盖。navigation 保留全部垂直 emitters 和
intensity，只降低水平 firing density。相同 headless fan-out 下 navigation
四路实测 10 Hz 且无 DDS lost；mapping raw 约 6 Hz。详见
`PERFORMANCE_BASELINE.md`。

同步节点默认只诊断，不改 stamp，不发布额外话题。设
`publish_synchronized:=true` 才逐对发布两个独立 synchronized topic；支持
`exact`、`approximate`、`latest-neighbor`。最近 navigation 实测 149 对时间差
全为 0，未再出现约 100 ms 错配。

## 过滤参数

两路分别执行 NaN/Inf、距离、高度、车体 box 和可选 voxel 过滤，TF 暂不可用时
丢弃该帧但节点继续运行，输入停止会节流告警，时间戳回退会作为仿真 reset 告警。
输入时间戳原样保留，输出 frame 固定为 `base_link`。本机每帧 raw 约 33 万点，
默认 wall-time timeout 因此设为 1.5 s；仿真频率阈值仍为 5 Hz。

默认车体 box `x=[-0.43,0.43]`、`y=[-0.29,0.29]`、`z=[-0.10,0.45] m`。
依据是 URDF 三层 base collision（主箱体约 x ±0.39、y ±0.24、顶面 z 0.410 m）
并加入约 4–5 cm 安全裕量，而不是代码内写死；所有值都在
`config/dual_lidar.yaml` 中可修改。实际过滤效果必须用同一姿态的 raw/filtered
点数和 RViz 车身回波对比确认。

## RViz、录包与回放

RViz Fixed Frame 是 `base_link`，红/蓝为两路 raw，黄/青为两路 filtered，并
显示 RobotModel、TF 和 odometry。雷达装反通常表现为：在 base_link 下，同一墙面
的两路点云镜像或旋转 180°、机器人运动时回波方向不符合传感器安装方向。

录制：

```bash
ros2 launch antbot_dual_lidar dual_lidar_bringup.launch.py \
  use_rviz:=true record_bag:=true bag_root:=/home/w/project/antbot/bags
```

也可直接运行 `record_dual_lidar_bag.sh [输出根目录]`。脚本使用 MCAP、时间戳
目录，结束后自动执行 `ros2 bag info`。

回放：

```bash
ros2 launch antbot_dual_lidar dual_lidar_replay.launch.py \
  bag_path:=/absolute/path/to/bag use_rviz:=true
```

回放默认 `start_preprocessing:=false`，因为标准录包已经包含 filtered topics，
可避免 bag 和处理节点重复发布同名输出。若使用只含 raw 的包，再显式设为 `true`。

## 覆盖、遮挡和参数修改

测试 USD 含四面墙、不同高度 box、细柱、四条桌腿、悬空桌面、低门槛和中央遮挡
柱。静止后分别做前进、后退、左右横移、左右旋转、45° 斜行，并靠近墙、穿过桌腿、
接近桌面，再用障碍遮挡单颗雷达。对角安装理论上互补，但实际盲区面积和遮挡角只能
从运行点云测量，静态几何检查不能替代 RViz 验收。

修改安装位置应先改 `antbot_description/urdf/antbot.xacro` 的两个
`CalibratedTopLidar` 外参，重新展开/导入 control USD，再派生测试 USD；不要在
ROS 另发一套补偿 TF。扫描参数在
`run_antbot_dual_lidar.py::configure_lidar` 的 3D profile/attribute 中修改，
然后重新生成派生 USD。

## 仿真时间与下一阶段

```bash
ros2 topic echo --once /clock
ros2 topic echo --once /antbot/lidar/front_left/points
ros2 topic hz /antbot/lidar/front_left/points
ros2 run tf2_ros tf2_echo base_link lidar_2d_front_scan
```

诊断节点直接比较两路 header、频率和 `/clock`，不会改写时间戳。下一阶段接同步/
联合 SLAM 时，应以两路 raw 或 filtered PointCloud2 为输入，增加可配置的近似/
精确同步、运动畸变补偿、IMU/odom 质量验证、外参一致性回归和目标 LIO 的字段/
QoS 适配。3D 避障还需选择 Nav2 voxel/spatio-temporal layer、地面分割、局部
costmap 高度范围和机器人 footprint；这些不属于本阶段。

## 性能、覆盖与 LIO 准备工具

```bash
# 独立启动一轮可重复 headless 性能测试；不切换默认 RMW
ros2_ws/src/antbot_dual_lidar/scripts/run_performance_case.sh navigation 30

# 在已经运行的系统上生成 observed-return 角度桶 CSV
ros2 run antbot_dual_lidar coverage_probe --duration 30 --bin-degrees 1 \
  --output /home/w/project/antbot/isaac/reports/blind_spot_metrics.csv
```

`performance_probe` 记录四路频率、点数/字节、stamp gap、DDS lost/QoS 事件、
进程 CPU/RSS 和 NVIDIA GPU 数据。`coverage_probe` 只量化当前场景回波，不能把
无表面造成的空角桶直接称为几何盲区。

当前标准点云没有逐点时间或 ring。实验参数 `--publish-lio-fields` 实测可从 GMO
取得真实 `timeOffsetNs` 并发布 `points_lio`；rc.7 的 channel/emitter 数组为空，
因此不会伪造 ring/emitter 字段，raw 输出仍保持正常。下一阶段契约、QoS、
IMU/odom 状态和未完成的 GUI/长期矩阵
分别见本包同名报告。

## Phase 2A：主 IMU 与真值逐点去畸变

Isaac runner 使用 6.0 推荐的 experimental physics API，在既有刚性
`imu_link` 下创建真实 `IsaacImuSensor`，不从 URDF 数值合成消息：

```bash
./python.sh /home/w/project/antbot/isaac/scripts/run_antbot_dual_lidar.py \
  --headless --lidar-mode 3d --lidar-profile navigation \
  --publish-lio-fields --imu-profile ideal --imu-rate 120
```

可选 `--imu-profile realistic`；固定 seed 和噪声参数集中在
`antbot/isaac/config/imu_profiles.yaml`。`--enable-lidar-imus` 才创建两颗辅助
IMU，且不会改变 Phase 2B 唯一主输入 `/antbot/imu/data`。

启动真值逐点验证节点：

```bash
ros2 launch antbot_dual_lidar dual_lidar_bringup.launch.py \
  use_sim_time:=true start_deskew_validator:=true \
  point_time_convention:=header_plus_offset
```

该参数只是候选实验条件。节点按每点时间插值 `/antbot/ground_truth/odom`（平移+
四元数 SLERP），再将每点变换到扫描参考时刻；真值覆盖不足会丢整帧，不修改
`points_lio`。四种候选必须分别运行并用场景几何残差决定最终语义。

Phase 2A MCAP：

```bash
record_dual_lidar_bag.sh /data/antbot_lio 01_static
ros2 run antbot_dual_lidar validate_lio_dataset /data/antbot_lio/<bag>
```

脚本记录 config/USD hash、git 状态、开始/结束仿真时钟、`ros2 bag info` 和必需
话题统计。12 段数据集当前均未录制，见 `datasets/DATASET_MANIFEST.yaml`。
