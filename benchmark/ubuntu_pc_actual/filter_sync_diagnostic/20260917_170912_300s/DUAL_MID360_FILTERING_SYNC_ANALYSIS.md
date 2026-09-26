# Dual MID360 Filtering & Sync Analysis

测试配置保持不变：`min_range=0.1 m`、`max_range=100 m`、`blind_width=90°`、`sync_tolerance=30 ms`、`sync_queue_size=10`。没有修改融合算法、TF、驱动或消息时间戳。

过滤统计使用每台约 1,450 帧、约 2,900 万点的样本计算比例，再乘以已确认的独立 raw 实测速率。同步统计使用额外的低开销 300.005 s 采集；每个 callback 只记录时间戳和到达时间。

## 1. Point filtering breakdown

### Front (`192.168.1.116`)

| 阶段 | pts/s | 占 raw |
|---|---:|---:|
| raw | 200,009.16 | 100.000% |
| invalid XYZ removed（NaN/Inf） | 0.00 | 0.000% |
| range `<0.1 m` removed | 34,545.31 | 17.272% |
| range `>100 m` removed | 0.00 | 0.000% |
| 90° angular sector removed | 35,306.85 | 17.653% |
| final | 130,157.00 | 65.076% |

Angular-only keep ratio（不应用 range）：**82.344%**。  
在有效量程点中，90°扇区删除 21.338%，保留 78.662%。

### Rear (`192.168.1.139`)

| 阶段 | pts/s | 占 raw |
|---|---:|---:|
| raw | 200,013.00 | 100.000% |
| invalid XYZ removed（NaN/Inf） | 0.00 | 0.000% |
| range `<0.1 m` removed | 48,649.54 | 24.323% |
| range `>100 m` removed | 0.00 | 0.000% |
| 90° angular sector removed | 42,057.89 | 21.028% |
| final | 109,305.57 | 54.649% |

Angular-only keep ratio（不应用 range）：**78.972%**。  
在有效量程点中，90°扇区删除 27.786%，保留 72.214%。

### Combined

| 阶段 | pts/s | 占 raw |
|---|---:|---:|
| raw | 400,022.16 | 100.000% |
| invalid removed | 0.00 | 0.000% |
| range `<0.1 m` removed | 83,194.85 | 20.794% |
| range `>100 m` removed | 0.00 | 0.000% |
| angular removed | 77,364.74 | 19.338% |
| final | 239,462.57 | 59.868% |

本次诊断的 239,463 pts/s 与原阶段 C 的 239,026 pts/s 相差约 0.18%，属于场景/采样窗口变化。双雷达 angular-only 保留率为 **80.660%**；如果先排除 range 无效点，则 angular 条件保留率为 **75.585%**。

短时复核显示，Front 的 280,776 个 `<0.1 m` 点中 280,724 个（99.981%）是严格 `(0,0,0)`；Rear 为 398,438/398,438（100%）。它们不是 NaN/Inf，因而属于 range removal。

## 2. Blind sector verification

### Front

- TF translation：`[0.322, 0.222, 0.414] m`
- TF quaternion xyzw：`[0, 0, 0, 1]`
- TF RPY：`[0°, 0°, 0°]`
- 自动 blind center：`-145.416°`
- blind width：`90.000°`
- 未归一化范围：`[-190.416°, -100.416°]`
- 实际跨界逻辑：`[169.584°, 180°) ∪ [-180°, -100.416°]`

### Rear

- TF translation：`[-0.322, -0.222, 0.414] m`
- TF quaternion xyzw：`[0, 0, 1, 0]`
- TF RPY：`[0°, 0°, 180°]`
- 自动 blind center：`-145.416°`
- blind width：`90.000°`
- 未归一化范围：`[-190.416°, -100.416°]`
- 实际跨界逻辑：`[169.584°, 180°) ∪ [-180°, -100.416°]`

两台雷达在各自局部坐标系中的 inward direction 相同，这是后雷达 180° yaw 的预期结果。代码使用 `abs(atan2(sin(angle-center), cos(angle-center))) <= width/2`，跨 `-π/+π` 正确；现有单元测试也覆盖了 `179°/-179°` 跨界情况。

## 3. Why only about 59.75% remains

实测原因可精确拆为：

```text
range pass        = 79.206%
angular pass      = 75.585%（条件于有效量程点）
combined final    = 79.206% × 75.585% = 59.868%
```

因此 270°/360° 的 75% 只对应 angular 这一级，不能直接作为完整流水线的最终比例。额外约 20.8% raw 点是 `<0.1 m` 的近零点，几乎全为 `(0,0,0)`。

原始角度分布不是均匀分布：`(0,0,0)` 会由 `atan2(0,0)` 落入 0° bin，使该 bin 非常高；排除 range 无效点后，Front 各 5° bin 的平均点数约 124–279，Rear 约 48–272，仍受 MID360 扫描几何、遮挡和场景回波影响。Front 和 Rear 单独对 blind sector 的占比不同，但合计在有效量程点中删除 24.415%，很接近几何理论 25%。

诊断未发现裁剪逻辑 bug：

- blind width 实际为 90°，没有变宽；
- `atan2(y,x)`、坐标系和自动中心计算一致；
- `±π` 跨界正确；
- invalid、range、angular 按顺序且互斥计数，同一点不会重复过滤；
- range removal 没有混入 angular counter。

## 4. Sync timing

有效轻量采集：300.005 s；Front 2,981 帧，Rear 2,983 帧；成功候选配对 2,967。

### Header timestamp absolute delta

| 指标 | ms |
|---|---:|
| average | 0.2774 |
| min | 0.0049 |
| P50 | 0.1864 |
| P90 | 0.6923 |
| P95 | 0.9625 |
| P99 | 1.2026 |
| max | 1.4079 |

Signed delta 定义为 `front_stamp - rear_stamp`：mean **+0.0110 ms**，median **+0.0280 ms**。Front timestamp 领先 1,373 对，Rear 领先 1,594 对，方向基本均衡，没有固定的 20–30 ms 偏移。

| 阈值 | 比例 |
|---|---:|
| `<10 ms` | 100.000% |
| `<20 ms` | 100.000% |
| `<30 ms` | 100.000% |
| `<40 ms` | 100.000% |
| `<50 ms` | 100.000% |

### Callback arrival delta

成功配对的绝对 callback 到达差：average 0.5448 ms、P50 0.5320 ms、P90 0.5881 ms、P95 0.6877 ms、P99 0.7522 ms、max 1.0369 ms。

Signed arrival delta `front - rear` 的 mean 为 −0.5448 ms、median −0.5320 ms，即 front callback 稳定先执行约 0.53 ms。这与 driver 发布顺序/单线程 callback 调度一致，不是 header 时钟偏移。`arrival_system - header` 平均值为 Front 100.788 ms、Rear 101.344 ms，二者差约 0.556 ms；主要包含驱动约 100 ms 的成帧周期，不能解释 30 ms miss。

## 5. The 47 sync misses

原阶段 C 的 CSV 只记录成功配对后的累计计数，没有保存被丢帧的 stamp 和 arrival time，因此无法对历史 47 次逐条事后分类。为避免猜测，本次做了同配置复现实测：

| 数据源 | success | miss/drop | 说明 |
|---|---:|---:|---|
| 轻量订阅器离线复现 | 2,967 | 30 | 直接订阅 driver raw topics |
| 同窗口真实 fusion | 2,952 | 55 | driver → Python relay → fusion |
| 原阶段 C | 2,949 | 47 | 历史正式窗口 |

轻量复现的 30 次分类：

- `best delta >30 ms`：30 次；
- 对端 100 ms 周期帧未出现在该订阅器：30 次（因果分类）；
- queue depth overflow：0 次；
- other：0 次。

注意，“delta >30 ms”不是说两台雷达对应帧真的相差超过 30 ms。对应帧缺失后，队列只能看到相邻周期帧，最近 delta 约 100 ms；成功配对帧的最大 delta 只有 1.408 ms。

Front 存在 17 个约 200 ms header 间隔，Rear 为 16 个；其中若两路在同一周期都缺失便不会产生孤帧，最终形成 30 次同步丢弃。两路频率分别为 9.9433 Hz 与 9.9467 Hz，差仅 0.0033 Hz，不是持续频率漂移。

30 个 orphan header 在时间轴上彼此不连续，但由于当前同步器会优先匹配整个队列中的最佳新帧，旧 orphan 可在队列中停留较久，之后集中清理。本次出现 6 个多次清理批次，最大一次连续清理 7 个；没有任何 `queue_size > 10` overflow。

真实 fusion 比直接订阅复现多 25 次 miss，且总接收量只略少。这说明独立的 best-effort DDS 订阅和 Python frame relay 会让两路缺失位置不完全相关，从而产生更多单边 orphan。网卡计数仍为 0 drop/error；无法仅凭 ROS 层数据区分是 driver publisher、DDS best-effort 还是 relay subscriber 丢失，但可以排除 30 ms 阈值偏紧、固定时钟偏移、CPU、物理链路带宽和 queue depth overflow。

原 47 次与本次真实 fusion 的 55 次量级一致，最符合相同的“单边消息未到达同步器”机制，而非 timestamp delta 本身。

## 6. Recommendation

### A. 当前约 60% 点保留率是否合理？

合理。约 20.794% raw 是近零点；剩余有效量程点再保留约 75.585%，最终约 59.868%。

### B. 90° angular crop 本身到底保留多少？

- 对全部有限 raw 点：Front 82.344%，Rear 78.972%，合计 80.660%。这里包含位于 0° 的全零点。
- 对有效量程点：Front 78.662%，Rear 72.214%，合计 75.585%。

### C. 是否存在裁剪逻辑 bug？

没有发现。计数守恒、扇区宽度、跨界判断、自动中心和过滤顺序均正确。

### D. 30 ms sync tolerance 是否偏紧？

不偏紧。成功配对 P99 仅 1.203 ms，最大 1.408 ms。增大到 40/50 ms 不会匹配缺失的对应帧，只可能误配相邻周期，因此不建议以增大阈值解决这些 miss。

### E. 后续用于 SLAM 的同步策略

建议保留小容差 approximate-time 配对，并显式处理单路缺帧：记录每路 sequence/stamp、及时淘汰孤帧、不要让旧 orphan 长期滞留；同时检查 relay 是否可改为同进程/组件内传递或可靠 QoS 的专项试验。这里仅提出建议，未实施参数或代码修改。

### F. 是否依赖严格 message 同步，还是保留 per-point timestamp 做 deskew？

移动平台上的 SLAM 更应保留 Livox per-point `timestamp` 并进行每雷达 deskew，再按各自 cloud stamp 变换到公共坐标系。严格 cloud-to-cloud 同步不能替代 deskew。融合层可使用近邻时间关联，并允许某一周期只有单雷达更新，而不应因一帧缺失阻塞整个感知链路。

## Evidence files

- `filtering_breakdown.csv`：逐级过滤汇总。
- `front_angle_histogram.csv`、`rear_angle_histogram.csv`：原始点 5° bin。
- `filter_validation_30s/*_angle_histogram_in_range.csv`：有效量程点 5° bin。
- `sync_only_events.csv`：轻量 300 s header/arrival 原始记录。
- `sync_only_pairs.csv`、`sync_only_drops.csv`：配对和逐次 drop。
- `sync_delta_histogram.csv`：有效 header delta 直方图。
- `sync_only_summary.json`：统计摘要。
- `fusion_reference_metrics.csv`：同窗口真实 fusion 对照。

`filter_collector_*_not_for_sync.csv` 是首轮重型点解析订阅器的中间结果；由于该订阅器自身消费不及 10 Hz，已明确标记为不可用于 sync 结论，但其约 2,900 万点过滤比例仍有效。
