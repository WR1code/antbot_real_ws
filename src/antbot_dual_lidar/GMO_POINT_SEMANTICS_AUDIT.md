# GMO / RTX 点坐标语义审计

状态：`Phase 2A PARTIAL`；`FAST-LIO NOT STARTED`

## 结论

Isaac Sim 6.0.1-rc.7 的 RTX GMO 可直接输出真正的 native scan。本项目已把
`omni:sensor:Core:outputFrameOfReference` 从 `WORLD` 改为 `SENSOR`。2026-07-26
的实际 headless 运行中，两颗雷达均自报：

```text
frame_of_reference=SENSOR
motion_compensation_state=NONCOMPENSATED
coords_type=SPHERICAL
timeOffsetNs_min=0
timeOffsetNs_max=99687670
```

因此 `/points_raw_native` 的 xyz 是每条 ray 采样时刻的 lidar 局部坐标，不再
使用 world endpoint 或 Header/current tick 位姿反算。`time_offset_ns` 直接来自
GMO `BasicElements::timeOffsetNs`，不是按点索引合成。随 Isaac Sim 安装的 GMO
1.0 头文件将其声明为 `int32_t*`，定义为相对于父 GMO `timestampNs` 的 offset；
ROS 字段也相应锁为 `INT32`：

```text
t_point_ns = Header.stamp_ns + time_offset_ns
```

旧路径配置 `WORLD`，读取 GMO world endpoint 后只调用一次
`UsdGeom.XformCache().GetLocalToWorldTransform(sensor_prim)`，以回调当下的一套
位姿执行 `(world - translation) @ rotation`。它没有按 `timeOffsetNs` 查询逐点
位姿，所以输出是统一 Header/current-pose 参考下的点，不是真正 raw。该路径
确实隐式消除了 native scan 的一部分扫描内运动形变，不能用于验证 LIO deskew。

## 数据流证据

| 阶段 | 输入坐标系 | 输出坐标系 | 使用的车辆/传感器位姿时刻 | 是否可能运动补偿 | 证据代码位置 |
|---|---|---|---|---|---|
| RTX ray tracing | 场景 world + ray 时刻 SENSOR | GMO | 每条 ray 的采样时刻由 RTX 内核处理 | 由 GMO 状态明确标记 | Isaac 随附 `gmo_lib/docs/generic_model_output.rst` 的 `MotionCompensationState`；运行输出 `SENSOR/NONCOMPENSATED` |
| GMO basic elements | `SENSOR`，球坐标 azimuth/elevation/range | 仍为 `SENSOR` | `timestampNs + timeOffsetNs[i]` | 否；实测 `NONCOMPENSATED` | `run_antbot_dual_lidar.py::configure_lidar`、`RtxGmoPointCloudPublisher.publish_raw` |
| spherical → Cartesian | 每点采样时刻 SENSOR | 每点采样时刻 SENSOR xyz | 不查询车辆或 Header 位姿 | 否 | `RtxGmoPointCloudPublisher.publish_raw` 的 SENSOR 分支 |
| ROS native/raw | 每点采样时刻 SENSOR xyz | `lidar_2d_front_scan` 或 `lidar_2d_back_scan` | 不执行跨点统一变换 | 否 | `/points_raw_native` publisher 与 `point_semantics.require_topic_route` |
| ROS LIO candidate | 与 native 相同 | 对应 lidar frame | 不使用真值；只是独立话题契约 | 否 | `/points_lio` publisher；不得与 truth deskew 混用 |
| truth world reference | native SENSOR + 连续 truth pose | `odom` world endpoint | 对 `Header + offset` 逐点插值 | 是，作为真值参考 | `ground_truth_deskew_validator.py` → `deskew_core.points_to_world` |
| truth deskew | native SENSOR + 连续 truth pose | Header 时刻的对应 lidar frame | 每点时刻与 Header 参考时刻 | 是，只用于 Phase 2A | `ground_truth_deskew_validator.py` → `deskew_core.deskew_points` |
| 离线四模型分析 | MCAP native SENSOR | Header lidar frame / truth world | A/B/C/D 各自的逐点时间 | 仅候选 truth deskew | `phase2a_bag_analysis.py` |

## 两颗雷达路径

前左和后右只在 topic、frame 和固定外参不同；两者都由
`RtxGmoPointCloudPublisher` 处理，并通过相同的 SENSOR/NONCOMPENSATED 语义
门禁。后雷达 `base_link -> lidar_2d_back_scan` 的 yaw 为 π，点坐标本身不再被
adapter 额外旋转一次。

## 三类话题

| 话题后缀 | 坐标语义 | 时间语义 | 允许用途 |
|---|---|---|---|
| `points_raw_native` | 每点采样时刻的 lidar 局部坐标 | `Header + int32 time_offset_ns` | deskew 验证、未来 LIO 原始源 |
| `points_deskew_truth` | 全部点统一到 Header 时刻的 lidar frame | 保留原 offset 仅供追踪；坐标参考时刻为 Header | Phase 2A 验证，禁止作为真实机器人 LIO 输入 |
| `points_world_reference` | 逐点 truth pose 投影后的 `odom` endpoint | 每点 `Header + offset` | 真值比较、可视化、几何指标 |

旧 `/points` 仅保留为不带逐点时间字段的兼容可视化流；旧 WORLD adapter 的
参考输出如再次启用，只能进入 `points_world_reference_gmo`，语义门禁会拒绝把
WORLD endpoint 发布到 `points_raw_native`。

## 尚待完成的实验状态

SENSOR/NONCOMPENSATED 与字段定义已经由 API 自报和静态实测确认。左/右旋、
低/中/高速的形变方向、时间模型领先幅度、五段新 MCAP 和 5 分钟吞吐仍须以
本轮新 native 话题重新执行；在这些结果完成前保持 `Phase 2A PARTIAL`。

## 冻结与收尾（2026-07-26）

五段 native MCAP、严格 IMU 覆盖、`/clock` 唯一时间戳、在线处理性能和 5 分钟
稳定性证据均已保存。最后一次已经启动的柱体 ROI 重算已完成；收尾时没有运行中
的重算进程，因此没有再次启动。

墙面 metric 曾错误使用 cube 中心距离。墙厚为 0.12 m，ray 实际命中内表面，
因此中心常量带来固定 0.06 m 偏差，并可能错误地让 raw 残差看起来更小；实现已
改为使用真实内表面。圆柱 ROI 曾使用过宽的半径容差，把距柱心约 0.44 m 的墙面
点混入柱体样本；已收紧 ROI，并将该结果只保留为诊断证据。

不再新增墙面、圆柱、方柱、速度、旋向或 MCAP 实验，也不调整 5% 阈值。时间语义
最终标记为：

```text
DEFERRED_TO_REAL_HARDWARE_OR_LIO_RESEARCH
```

`truth_deskew` 和 `world_reference` 继续严格限于实验验证，禁止进入正常定位和
导航链路。Phase 2A 保持 `PARTIAL`，`FAST-LIO NOT STARTED`。
