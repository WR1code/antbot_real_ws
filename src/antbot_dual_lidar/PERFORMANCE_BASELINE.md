# 双 3D LiDAR 性能基线

日期 2026-07-26。测试为 headless、demo 场景、Jazzy 默认 RMW、robot state
publisher、NumPy 预处理、诊断和四路性能探针；未开启 RViz 或 rosbag。原始 JSON
位于 `isaac/reports/dual_lidar_performance_{mapping,navigation}.json`。

## 实测结果

| 指标 | mapping | navigation |
|---|---:|---:|
| firing rate / scan rate | 36000 Hz / 10 Hz | 3000 Hz / 10 Hz |
| front raw 点/帧，字节/帧 | 334106，5345699 | 27835，445354 |
| rear raw 点/帧，字节/帧 | 332963，5327401 | 27737，443788 |
| front/rear raw 仿真频率 | 6.39 / 6.08 Hz | 10.00 / 10.00 Hz |
| front/rear filtered 点/帧 | 23154 / 22752 | 9915 / 9785 |
| front/rear filtered 仿真频率 | 5.53 / 5.77 Hz | 10.00 / 10.00 Hz |
| raw stamp 推断丢帧 | 26 / 29 | 0 / 0 |
| raw DDS message-lost | 28 / 29 | 0 / 0 |
| 周期 P95/P99（两路范围） | 0.3–0.4 / 0.6–0.7 s | 0.1 / 0.1 s |
| Isaac 主进程 CPU / RSS | 784% / 6662 MiB | 450% / 6526 MiB |
| Python 预处理 CPU / RSS | 72.5% / 146 MiB | 29.9% / 82.9 MiB |
| GPU 利用率 / 显存 | 28.4% / 4309 MiB | 44.3% / 4071 MiB |

CPU 百分比可超过 100%，表示多核使用。探针枚举到 Isaac 的 shell/wrapper 和主
进程；表中 CPU 是进程组求和，RSS 使用最大常驻进程，避免重复计算 wrapper。
Isaac World 配置为 physics 120 Hz、rendering 60 Hz。另一次 5 s、navigation、
empty-layout、增强字段 sanity run 正常退出，实测 runner render loop 为
56.39 wall FPS、4.717 s sim / 5.019 s wall，点云 10.00 sim Hz；physics 由 World
按 120 Hz 配置在每个 render step 内子步进，但未单独插桩计数，不能把 120 写成
独立性能实测。基线点云 header 和 wall 消费频率均在 JSON 中保留。

按 10 Hz 名义采集计算，mapping raw 为前 53.46 MB/s、后 53.27 MB/s，双路
106.73 MB/s（不含 DDS/RTPS 开销）；navigation 双路为 8.89 MB/s，降低 91.7%。
高密度点数来自 128 emitters（32 个 elevation 重复四组方位）、36 kHz firing、
10 Hz 完整圈积累；两传感器各只有一个 render product 和一个 GMO writer，没有
重复发布节点。

## 瓶颈结论

mapping 在 RTX 生成、GMO 到 NumPy、约 10.7 MB/双帧的序列化/复制与多个订阅者
共同压力下发生队列丢旧帧；GPU并未满载，CPU和内存带宽/ROS 数据搬运是主要边界。
Python无逐点循环，使用结构化数组、布尔 mask 和 voxel 向量化；它增加负载但不是
navigation 的限制因素。navigation 在相同探针负载下四路均达到 10 Hz，因此当前
没有证据支持新增 C++ 实现。mapping 用于低可视化负载下的数据质量优先任务。

本轮没有执行单雷达、RViz raw/filtered 开关和三种 rosbag 组合对照；这些不能写为
已测。可重复基线：

```bash
ros2_ws/src/antbot_dual_lidar/scripts/run_performance_case.sh navigation 30
ros2_ws/src/antbot_dual_lidar/scripts/run_performance_case.sh mapping 30
```

脚本不设置 `RMW_IMPLEMENTATION`。探针的队列深度为 SensorDataQoS depth 5；rclpy
不暴露队列占用量，积压通过 stamp gap、message-lost 事件以及 wall/sim 频率差
推断。
