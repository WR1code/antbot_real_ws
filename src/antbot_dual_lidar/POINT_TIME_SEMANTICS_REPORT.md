# PointCloud2 逐点时间语义报告

状态：真实运动实验已执行；Header 锚点和公式仍 PENDING。

两路 navigation 实际消息字段均为
`x,y,z,intensity,time_offset_ns`。`time_offset_ns` 是 PointField UINT32
（datatype 6），本次短跑两路范围均为 `[0, 99,691,170] ns`，10 Hz 且无 missing
frame；channel/emitter 数组长度为 0，所以没有创建 ring。

17 帧附加顺序检查中，两路帧内 offset 回退均为 0，因此内存顺序是非递减；但前
雷达相邻重复 offset 343,907 次、后雷达 342,016 次，明确不是严格递增。重复符合
同一 firing 时刻多回波/多 emitter 交错的表现，但因为身份数组缺失，不能进一步
声称已识别具体 vertical emitter。

约 100 ms 范围不能证明 Header 是开始或结束。验证节点明确支持：

1. `header + offset`
2. `header - offset`
3. `header + offset - scan_period`
4. Header 为中点：`header + offset - scan_period/2`

必须在恒定前进、横移、左右旋、组合运动中分别运行，以真值连续位姿 deskew，并
比较墙厚、平面 RMSE、细柱扩散、点面 P50/P95/P99、帧间和双雷达重叠残差。最终
只选择跨运动一致的最低残差候选。当前 launch 默认第一项仅为了可执行，不是结论。

内存顺序单调性、回绕、reset 恢复、mapping/navigation 一致性以及几何残差仍是
PENDING，因此契约保留 `pending_ground_truth_experiment`。

## 2026-07-26 真实运动结果

从 381.8 MiB、54.65 s 的 motion-matrix MCAP 中，以 Header 为统一 deskew
reference，分别计算 A/B/C/D。前雷达 66 条几何记录的 median deskew RMSE：

```text
A 0.01857243 m
B 0.02102196 m
C 0.01857142 m
D 0.01857285 m
```

后雷达分别为 `0.01967009 / 0.02134476 / 0.01966830 / 0.01966915 m`。B 明显较差，
但 A/C/D 的差异只有微米量级，没有相对第二名的稳定优势，因此按预先规则禁止选择。

墙面在 C 下左右旋均改善，但柱体不一致：左旋中位改善 `-0.109%`、右旋
`+0.010%`。进一步审计发现当前 GMO adapter 先恢复 world endpoint，再用 Header
时刻传感器位姿转换到发布 frame；该过程可能已经生成“Header-frame
motion-compensated cloud”，令 A/C/D 的常数锚点不可观。必须先修正或证明 adapter
的原始坐标语义，当前配置改为 `pending_world_endpoint_adapter_audit`。

## Native 复核后的最终语义（取代以上 PENDING 结论）

GMO 规范及运行时元数据共同确认：

```text
frameOfReference = SENSOR
motionCompensationState = NONCOMPENSATED
BasicElements::timeOffsetNs = int32
t_point_ns = timestampNs + timeOffsetNs
```

ROS `Header.stamp` 直接复制 GMO `timestampNs`，offset 直接复制 GMO 数组，未按
点索引合成；PointField datatype 已改为 `INT32(5)`。坐标是每点采样时刻 SENSOR
局部坐标，所以点坐标与 offset 属于同一逐点时间参考。

新的三档左右旋 native MCAP 用预先声明的 5% 领先阈值重算。相对场景已知墙面
法向/位置的绝对残差可观察统一时间锚点，而旧自由平面拟合对此不敏感：

| lidar | A median/P95 | B | C | D | 选择 |
|---|---:|---:|---:|---:|---|
| front | 0.004716/0.012754 | 0.016474/0.039650 | 0.011912/0.031390 | 0.007304/0.017088 | A；领先 D 35.424% |
| rear | 0.005031/0.011584 | 0.016808/0.042942 | 0.012489/0.032297 | 0.007712/0.019046 | A；领先 D 34.764% |

最后一次研究运行中，两颗雷达的 A 候选均最低，但这依赖修正后的场景常量与 ROI，
且圆柱指标仍表现出几何敏感性。按 Phase 2A 冻结决定，该结果不再作为正常链路的
唯一时间语义。最终标记为：

```text
DEFERRED_TO_REAL_HARDWARE_OR_LIO_RESEARCH
```

约 1.5 m 墙的真实内表面和去除墙面污染的方柱结果仅作为已保存实验观测。A/C/D
过去微米级不可分的原因是自由拟合几何对整帧刚体偏转不敏感，不是 offset 没有
运动信息；不再新增实验尝试分离这些候选。
