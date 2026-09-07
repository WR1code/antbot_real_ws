# 阶段 1 / 1.5 测试报告

报告日期：2026-07-26；环境：Ubuntu 24.04、ROS 2 Jazzy、
Isaac Sim 6.0.1-rc.7。

## 静态检查

- 已检查 `antbot.xacro`、`sensors.xacro`、生成 URDF、raw/validated/control/
  dual-lidar USD、Isaac runner、ActionGraph 创建代码、现有 TF/launch/RViz 和
  `antbot_lidar_fusion`。
- 原两雷达确认为 RTX `Example_Rotary_2D` 单通道 2D，不伪称 3D。
- 已由 Isaac Sim 实际生成 `antbot_dual_lidar_test.usd`；生成日志确认两颗
  `Example_Rotary` 均为 128 channels/emitters、10 Hz、0.10–20 m、elevation
  -15°…+10°，render product 不同。
- 派生场景存在墙、立柱、桌腿、悬空桌面、低门槛、中央遮挡和多高度障碍。

## 自动化测试

`colcon build --symlink-install --packages-select antbot_dual_lidar` 通过。
新包 pytest 14/14 通过；`colcon test-result --verbose` 报告当前工作空间累计
85 tests、0 errors、0 failures、0 skipped。覆盖：NaN/Inf、距离、高度、车体、
voxel、空云、完整四元数变换、非法参数、时间戳回退/reset、输入 timeout、
双流独立、YAML、launch 语法和录包 topic contract。两个 launch 的
`--show-args` 均成功，Xacro 展开和 `check_urdf` 成功。

## Isaac Sim 实际运行测试

- 已执行 `--headless --create-only --lidar-mode 3d`，成功生成和保存派生 USD。
- 短时 headless 运行实际收到两路 raw 和 filtered PointCloud2。raw 单帧约
  333k/334k 点，字段为 x/y/z/intensity，z span 约 1.01 m；filtered 为
  `base_link`，约 11.3k/11.7k 点。
- TF 实测：前左 `(0.322,0.222,0.414), yaw=0`；后右
  `(-0.322,-0.222,0.414), yaw=π`。
- 探针同一时刻两路 raw header 差为 0；诊断暖机后观察频率约
  6.2–6.6 Hz（仿真时间），高于 5 Hz 下限，但低于传感器 10 Hz 配置，期间有
  一次 0.1 s 配对差告警。33 万点/帧通过 DDS/NumPy 处理存在丢帧风险。
- `/clock`、`/joint_states`、`/odom`、`/tf`、`/tf_static` 均在实际 MCAP 中
  收到；未篡改点云 header。
- 4 秒 MCAP 实录成功：63.3 MiB、632 messages，两路 raw 各 6 帧、filtered
  5/6 帧，`ros2 bag info` 成功。
- 标准 replay launch 已修正为回放包内真实 `/clock`（不使用 wall-time
  `--clock`），并增加 1 s discovery delay。实际回放探针收到 raw 1 帧、
  filtered 5 帧和 `/clock` 180 帧；frame/点数保持正确。
- 单路停止通过代码与单元状态测试验证，尚未在 Isaac GUI 中执行
  `--disable-front/--disable-rear` 的完整人工验收。

## RViz 实际观察

尚未执行 GUI 人工观察，不能记为通过。

## 已通过

- 非破坏性派生 USD 创建。
- 双 3D RTX 配置和独立 render product 静态确认。
- 两路 PointCloud2 + intensity、独立 frame、base_link 过滤输出、TF、仿真时间
  和短时 MCAP 实录。

## 未执行

- 12 项运动/墙面/桌腿/桌板/单雷达遮挡人工工况。
- RViz raw/filtered 空间重合与装反检查。
- MCAP 回放 RViz 人工观察和长时间录制（短时无 GUI 回放已通过）。

## 失败

- 首次 create-only 运行暴露缺少 `PointCloud2` import，已修复并重新运行成功。
- 首次诊断运行暴露 callback 名与 `rclpy.Node._clock` 冲突，已重命名并重跑成功。
- 文档曾指定本机不存在的 `rmw_cyclonedds_cpp`；最终验收使用 Jazzy 默认 RMW。

## 风险与下一步

- rc.7 属于 release candidate，GMO/ROS helper 行为可能与最终版不同。
- 当前通用 128-channel profile 数据量很大，实测消费频率约 6 Hz 且偶发一帧
  配对偏差；联合 SLAM 前应做 profile/传输/同步性能优化。
- 必须完成 GUI 运动和遮挡矩阵后，才能量化中央结构/横移/后退盲区。
- 下一阶段前需确认同步策略、IMU/odom、运动畸变、目标 LIO 消息字段和 QoS。

## 阶段 1.5 增量实测

- mapping 36 kHz profile：raw 约 333k 点/5.33 MiB 每帧，前后仿真频率
  6.39/6.08 Hz；DDS message-lost 28/29。
- navigation 3 kHz profile：raw 约 27.8k 点/0.445 MB、filtered 约
  9.8–9.9k 点，四路均 10.00 Hz，stamp drop、message-lost 和 incompatible QoS
  事件均为 0。
- navigation 同步诊断 149 对全部 exact，max/mean/P95/P99 为 0，未配对、
  stamp drop、>50 ms 和约 100 ms 事件均为 0；可选同步 topic 保持两路身份。
- 标准 raw 字段仍只有 x/y/z/intensity。实验增强 topic 实测可发布真实
  time_offset_ns；rc.7 的 channel/emitter 数组为空，未伪造 ring/emitter。
- GUI 运动/遮挡矩阵和 10 分钟稳定性仍未执行，详见各 PENDING 报告。

最终新包 `colcon build --symlink-install --packages-select antbot_dual_lidar`
通过，pytest 20/20 通过，累计 `colcon test-result` 为 91 tests、0 errors、
0 failures、0 skipped；两个 launch 的 `--show-args` 均通过。完整 workspace
build 已尝试，但现有 `antbot_libs` 缺少外部 `dynamixel_sdkConfig.cmake` 而失败；
完整 test 随后有 Vanjee/硬件相关 5 个包因未生成 install 文件无法启动。本阶段包
不依赖这些组件，未使用 sudo 安装或修改上游依赖。

默认 RMW 运行检查确认两路 raw publisher 均为 BEST_EFFORT/VOLATILE、各一个
publisher，实际字段只有四个 float32；TF 实测仍是阶段 1 外参。增强运行首帧
`time_offset_ns=0…99,687,670`，仅输出两条 partial schema 提示，无 assertion/
traceback，raw 保持 10 Hz。
# Phase 2A 自动化增量（2026-07-26）

- `colcon test --packages-select antbot_dual_lidar`：最终 28/28 通过。
- 覆盖：PoseBuffer reset、越界/缺样本、平移插值、SLERP、空云、逐点平移与旋转
  去畸变、四种时间公式、数据集必需话题/时间回退/共同覆盖、YAML 和 launch。
- Isaac 6.0.1-rc.7 create-only：experimental physics IMU Prim 创建并保存成功。
- 120 Hz 短跑：562 样本，sim 120.000 Hz，minimum 119.995 Hz，P95 period
  0.008333445 s，P99 0.008333683 s，回退 0，重复读取 0。
- 60 Hz 短跑：179 样本，sim 60.000 Hz，minimum 59.999 Hz，P95 period
  0.016666770 s，P99 0.016666890 s，回退 0，重复读取 0。
- 120 Hz 作为最终默认值，因为 physics callback 可产生唯一的 120 Hz 样本；
  不是重复 60 Hz render 样本。
- realistic + 两颗辅助 IMU 短跑：主 IMU 76 样本、120.000 Hz，日志确认
  `auxiliary=2`；辅助接口未与主 IMU融合。
- 真实 ROS 检查：`sensor_msgs/msg/Imu`，`frame_id=imu_link`，单发布者，
  BEST_EFFORT/VOLATILE；静止 linear_acceleration.z=9.8100004 m/s²。
- 双增强点云短跑均为 10.000 Hz、无 missing frame；两路
  `time_offset_ns=[0, 99691170]`，字段为 UINT32，无 ring/channel/emitter。
- 17 帧逐点顺序：两路内存顺序 offset 回退均为 0（非递减）；相邻重复分别为
  343,907/342,016，故不是严格递增。

未执行：运动符号矩阵、pause/resume/reset 人工矩阵、四时间假设运动残差、
墙/柱 deskew 定量、IMU 1/2/5 s 积分、12 段 MCAP、GUI 与长期压力测试。它们均
保持 PENDING，不能据当前短跑判定 Phase 2A 最终验收通过。

完整 workspace 未在本轮重建；已知 `antbot_libs` 的外部
`dynamixel_sdkConfig.cmake` 缺失不属于本包。本阶段要求的
`--packages-up-to antbot_dual_lidar` 已独立构建成功。

# Phase 2A 真实运动与数据收口（2026-07-26）

最终状态：`Phase 2A PARTIAL`。

## 实际运动与 IMU

通过现有 `/cmd_vel -> swerve_controller_node -> steering/wheel command ->
Isaac ActionGraph` 驱动车体，完成静止、前进、后退、左旋、右旋、前进左转、前进
右转。每项有 1 s 前静止、至少 2.5 s 运动、1 s 后静止；左右旋为 5.2 s。

| 动作 | truth yaw (rad) | mean IMU wz (rad/s) | mean IMU ax (m/s²) |
|---|---:|---:|---:|
| static | -0.001823 | +0.000853 | -0.000262 |
| forward | -0.001014 | -0.000393 | +0.071090 |
| backward | +0.000275 | +0.000161 | -0.072412 |
| left rotation | +1.122483 | +0.214434 | -0.000128 |
| right rotation | -1.232167 | -0.234044 | +0.000055 |
| forward left | +0.573067 | +0.226455 | +0.048786 |
| forward right | -0.400747 | -0.159121 | +0.061066 |

左右旋 IMU/truth 符号一致且相反；前后加速 X 符号相反。IMU timestamp 回退 0、
重复 0。旋转积分误差（IMU-truth）为：左 `1/2/5 s =
+0.001560/-0.000823/-0.004651 rad`；右
`+0.001878/+0.003797/+0.011835 rad`。

## 四时间假设与几何

motion-matrix MCAP：381.8 MiB、54.646 s、31,534 messages；前/后 raw cloud
439/438，IMU 5,242。扫描 Header 中位周期 `100,000,000 ns`，offset
`[0,99,691,170] ns`。

| lidar | A median/P95 RMSE | B | C | D |
|---|---:|---:|---:|---:|
| front | 0.018572/0.022212 | 0.021022/0.051511 | 0.018571/0.022213 | 0.018573/0.022211 |
| rear | 0.019670/0.022590 | 0.021345/0.052368 | 0.019668/0.022592 | 0.019669/0.022586 |

B 可排除，但 A/C/D 无稳定优势，前后模型均保持 PENDING。

C 候选下墙面中位指标：

| lidar | raw RMSE | deskew RMSE | improvement | deskew P95 | thickness |
|---|---:|---:|---:|---:|---:|
| front | 0.022338 | 0.021171 | +1.643% | 0.041952 | 0.069542 |
| rear | 0.023371 | 0.021360 | +11.528% | 0.041570 | 0.069516 |

柱体：

| lidar | raw radial RMSE | deskew RMSE | improvement | deskew P95 | thickness |
|---|---:|---:|---:|---:|---:|
| front | 0.013793 | 0.013707 | +0.070% | 0.025519 | 0.042614 |
| rear | 0.013361 | 0.013392 | +0.004% | 0.025063 | 0.041880 |

柱体左右旋改善不一致（C：左 -0.109%，右 +0.010%），不满足选择条件。证据见
`artifacts/phase2a_time_hypothesis_results.json` 与
`artifacts/phase2a_deskew_metrics.csv`。

## 前左最小 MCAP

五段均真实录制，路径 `artifacts/phase2a_bags/minimal_front`，raw 10 Hz 且
missing frame=0：

| case | duration | raw/deskew/IMU | size |
|---|---:|---:|---:|
| 01_static | 4.15 s | 39/13/462 | 23,387,777 B |
| 02_left_rotation | 4.72 s | 47/14/556 | 27,663,971 B |
| 03_right_rotation | 4.82 s | 48/14/568 | 28,114,937 B |
| 04_wall_motion | 4.85 s | 49/16/584 | 29,540,505 B |
| 05_column_motion | 5.27 s | 53/17/623 | 32,259,067 B |

每段都有 SHA256、topic 频率/最大 gap/单调性、offset、git/config hash，见
`artifacts/phase2a_minimal_dataset_manifest.yaml`。严格验证失败：录包边界的
2–3 帧缺少完整扫描前置 IMU 覆盖；在线 Python deskew 只有约 3.1 Hz。
MCAP 还观察到 `/clock` 每个物理时刻约两条相同 stamp（例如 static 为
499 messages、249 duplicates），IMU 自身重复为 0；需要进一步审计 clock 发布
端点，不能把非递减误写成严格唯一。

## 自动化与 GUI

RViz 配置已生成：
`config/rviz/dual_lidar_phase2a.rviz`。实际消息/TF已自动确认；未进行人工肉眼
查看，`GUI_VISUAL_CHECK=PENDING`。

最终重新执行构建和测试：包内 `pytest 30 passed`；`colcon test-result` 汇总
101 tests、0 errors、0 failures、0 skipped。

## 阻塞最终验收

1. A/C/D 时间锚点未定量分离，至少一个雷达的时间模型尚未锁定；
2. 柱体左右旋 deskew 没有一致改善；
3. 五段 MCAP 已录制但严格 IMU 边界覆盖未通过；
4. 在线 deskew 仅约 3 Hz，不能覆盖 10 Hz raw；
5. `/clock` 存在成对重复 stamp，需要确认是否有重复 publisher/tick；
6. GUI 目视仍 PENDING（允许独立保留，但不能写已查看）。

因此禁止写 `Phase 2A COMPLETE`，也未启动 FAST-LIO。

# Phase 2A native 语义收口增量（2026-07-26，取代上节旧阻塞统计）

- GMO 实测改为 `SENSOR/NONCOMPENSATED/SPHERICAL`；native 两路 10.000 Hz，
  `time_offset_ns=[0,99691170]`，字段改为规范要求的 `INT32`。
- 旧 WORLD adapter 已证实只用一套回调位姿反算，不能再称 raw；运行时语义门禁
  会拒绝把 WORLD endpoint 发布到 `points_raw_native`。
- 新话题分离为 `points_raw_native`、`points_deskew_truth`、
  `points_world_reference`；truth 两路只订阅 native。
- NumPy 批量 pose interpolation、SLERP、矩阵和点变换在线短跑：
  前雷达 mean/P95/P99/max `13.475/19.499/24.196/47.439 ms`，
  后雷达 `12.085/16.186/19.669/23.513 ms`；输入/输出各
  `846/845`，唯一 drop 是启动边界真值覆盖不足，不是性能过载。
- `/clock` 旧 ActionGraph 单发布者在同一 render 内重复执行两次：
  修复前 `5658 total / 2829 unique / 2829 consecutive duplicate`。断开旧
  exec 后由 physics callback 发布；复验 `584/584 unique`、重复 0、回退 0，
  单发布者 GID `01.0f.1b.9e.b5.17.c4.1d.00.00.00.00.00.00.51.03`。
- native speed-sweep MCAP 为 1.2 GiB/56.17 s。绝对已知墙面 metric 下前雷达 A
  相对 D 领先 `8.879%`，超过预设 5%，锁定 A；后雷达领先仅 `1.870%`，保持
  PENDING。B 在两路均明显恶化。
- 墙面 A 在低/中/高速、左/右旋和前/后雷达全部改善，详见
  `artifacts/phase2a_semantics_audit/raw_vs_truth_deskew_metrics.csv`。
  圆柱径向 metric 仍有方向/速度档轻微恶化，故 Phase 2A 不能写 COMPLETE。
- 五段 native MCAP 位于 `artifacts/phase2a_bags_native`，逐段内容校验均通过。
  严格有效帧 IMU 覆盖为 `51/51、53/53、49/49、53/53、54/54`；边界剔除
  `2、1、1、2、0`。各段修复后 clock 重复和回退均为 0。
- 本轮包级 pytest 为 37/37 通过；最终 colcon 结果见本报告后续 5 分钟验收段。
- `GUI_VISUAL_CHECK=PENDING`；未启动 FAST-LIO。

## 5 分钟 deskew 稳定性

持续 wall-clock 310.001 s（sim 345.483 s）。两颗 raw 各 3455 帧、
`10.000 Hz`、missing frame 0；IMU 41458 样本、120.000 Hz、stamp 回退/重复
均为 0。validator 最后周期统计：

| lidar | input/output | coverage drop | mean/P95/P99/max ms |
|---|---:|---:|---:|
| front_left | 3279/3278 | 1（启动边界） | 12.906/18.481/23.248/37.177 |
| rear_right | 3279/3278 | 1（启动边界） | 11.850/15.014/19.089/26.840 |

有效输入输出率约 9.997 Hz；无性能过载 drop、无时间戳回退、无持续队列增长。
终止测试时的 `ExternalShutdownException` 已作为正常 ROS shutdown 捕获，核心
运行期间没有 traceback。

## 1.5 m 墙 + 方柱最终 sweep

最终选择使用 `arena_linear_scale=0.5`：墙中心约 1.5 m、内表面 1.44 m，方柱
中心约 1.0 m、边长 0.4 m。修复两项预先可验证的 ROI/几何错误：墙 metric 从
cube 中心改为真实内表面；圆柱/方柱 ROI 收紧，排除距中心约 0.44 m 的墙面点。
5% 选择阈值未修改。

- 前雷达 A median/P95 `0.004716/0.012754 m`，相对 D 领先 35.424%；
- 后雷达 A median/P95 `0.005031/0.011584 m`，相对 D 领先 34.764%；
- 最后一次研究运行中，两颗雷达的 A 候选均取得最低残差；该结果只作为已保存
  的实验观测，不再升级为正常链路的唯一时间语义；
- 墙面 12 个 lidar×方向×速度组合全部改善 37.93%–65.86%；
- 去污染方柱 12 个组合全部改善 20.03%–49.71%，左右边缘误差和 P95 同步下降；
- 圆柱径向指标继续单独报告，但因对切向拖影不敏感，不用于时间模型选择。

按项目主线冻结要求，不再用几何 ROI 扩大 A/C/D 研究。最终状态为：

```text
Phase 2A PARTIAL
TIME_SEMANTICS=DEFERRED_TO_REAL_HARDWARE_OR_LIO_RESEARCH
GUI_VISUAL_CHECK=PENDING
FAST-LIO NOT STARTED
```

墙面与方柱数字保留为最后一次已执行实验的结果，不构成时间模型唯一锁定。
