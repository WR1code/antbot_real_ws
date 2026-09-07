# QoS 设计

| 链路 | 发布与订阅 | reliability / durability / history / depth | 行为 |
|---|---|---|---|
| raw 点云 | Isaac publisher、预处理、诊断、同步、探针 | best_effort / volatile / keep_last / 5 | 优先最新传感器帧，允许过载时丢旧帧 |
| filtered 点云 | 预处理 publisher、RViz/探针 subscriber | best_effort / volatile / keep_last / 5 | 不积压旧障碍 |
| 可选 synchronized | 同步节点 publisher | best_effort / volatile / keep_last / 5 | 保留各自消息和原始 stamp |
| `/clock` | Isaac 与消费者 | SensorDataQoS | reset 后继续 |
| 文本诊断 | ROS logger | 不适用 | 当前没有伪造一个可靠诊断 topic |

以上使用 `qos_profile_sensor_data`。实际 navigation 压测四路
`incompatible_qos_events=0`、`middleware_message_lost=0`；mapping 因消费过载
raw message-lost 为 28/29，这符合低延迟丢旧帧设计，并非改 reliable 就能消除的
吞吐瓶颈。后续 LIO 订阅必须使用兼容的 best-effort SensorDataQoS 和小队列，并
自行检测 stamp gap、掉线与 reset。IMU 若新增也应使用 SensorDataQoS；低频状态/
测试结论若形成消息则可使用 reliable。

本阶段使用 Jazzy 默认 RMW，未安装、未选择、未测试 CycloneDDS。后续只有安装后
才能用同一脚本显式设置 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` 做 A/B 测试。
