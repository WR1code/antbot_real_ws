# LIO 输入兼容性

实际 raw `PointCloud2` 仅有 `x/y/z/intensity`，均为 float32；frame stamp 来自
GMO `timestampNs` 和 `/clock`。当前配置 `accumulate_outputs=true`，10 Hz scan
和 10 Hz publish，因此消息代表一圈积累结果，而不是单一瞬时射线。

标准 raw 没有逐点时间、ring/channel、emitter ID、azimuth、elevation、
object ID 或 scan ID。Isaac GMO Python API在 FULL auxiliary 模式定义了
`timeOffsetNs/channelId/emitterId` 等底层数组。实际运行中 `timeOffsetNs` 与
return count 等长，首帧范围 `0…99,687,670 ns`，与约 100 ms 一圈一致；
`channelId/emitterId` 为空且报告
`auxType is NONE`，所以当前不能恢复 ring。

`--publish-lio-fields` 是实验开关：当前发布独立的 `points_lio`，字段为
`x,y,z,intensity,time_offset_ns`；只有底层身份数组完整时才追加
`ring,emitter_id`。任何数组均不补零伪造。默认仍使用 raw 标准话题，契约把
per-point timing 标为 available-when-enabled，把 ring 标为 unavailable。

一般 LIO 的去畸变需要高频 IMU、每点相对采样时间和明确的雷达模型/扫描顺序；
具体 FAST-LIO 分支还可能要求其自定义点类型。默认 raw 的帧级 stamp 只能做帧
配对；增强 topic 提供相对点时间，但当前没有仿真 IMU，仍不能完成运动补偿。
下一阶段需验证 time offset 的方向/单位与扫描模型，修通 channel/emitter buffer，
并做目标算法字段映射验证。
