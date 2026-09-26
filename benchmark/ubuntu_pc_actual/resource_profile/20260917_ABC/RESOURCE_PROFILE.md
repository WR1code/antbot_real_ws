# Dual MID360 Resource Profile

## 简洁总结

双 MID360 输入：

- **400,022 pts/s**
- **10.401 MB/s PointCloud2 payload**

Driver：

- CPU **4.710%**，即 **0.047 logical CPU**
- RSS 平均 **56.30 MiB**，峰值 **56.85 MiB**

Fusion：

- CPU **5.613%**，即 **0.056 logical CPU**
- RSS 平均 **38.13 MiB**，峰值 **39.98 MiB**

整套系统：

- C−A 整机 CPU 增量 **0.352 个百分点**；24 逻辑 CPU 上约 **0.085 logical CPU**
- C−A RAM 增量 **186.4 MiB**

`eno1`：

- RX **5.967 MB/s / 47.74 Mbit/s**
- RX packets **约 4,571 packets/s**
- RX/TX error、drop：**0**

Merged：

- **9.830 Hz**
- **234,927 pts/s**
- **6.108 MB/s PointCloud2 payload**

**RESOURCE_PRESSURE = NO**

当前没有证据表明 orphan 是由整体资源耗尽导致。CPU、RAM、1 Gbit/s 网络和 callback 处理时间都有明显余量；DDS、subscriber、Python frame relay、executor 调度以及 best-effort 消息交付仍是后续诊断方向。

## 1. 测试来源和定义

本报告复用同一台机器、同一驱动与同一 fusion 配置下已经完成的连续采样：

- A：Idle baseline，59 个约 1 s 样本（约 60 s）。
- B：双 MID360 driver-only，300 个 1 s 样本（300 s）。
- C：driver + frame relay + TF + crop + fusion，300 个 1 s 样本（300 s）。
- callback arrival：独立的低开销 300 s 时间采集，仅记录 stamp 和 monotonic callback-enter 时间。

整机 CPU 来自 `mpstat`/`/proc/stat`，是 24 个逻辑 CPU 的平均忙碌百分比。进程 CPU 来自 `pidstat`，其中 **100% 等于占满一个逻辑 CPU**。因此 fusion 5.613% 等于约 0.056 logical CPU。

历史 A/B/C 采样没有记录 `sar -q` load average、UDP 内核 counter 起止值和 NIC `rx_missed_errors` 起止值。本报告不使用测试结束后的当前累计值冒充历史窗口数据。接口 `ip -s` 起止 counter、`sar -n DEV,EDEV` error/drop/rxfifo 则已完整记录。

## 2. CPU

### 整机 CPU

| Stage | mean | median | P95 | P99 | max |
|---|---:|---:|---:|---:|---:|
| A idle | 2.156% | 1.820% | 4.523% | 6.908% | 7.070% |
| B driver | 2.164% | 1.630% | 5.881% | 9.767% | 13.600% |
| C full | 2.509% | 2.130% | 5.753% | 8.381% | 8.660% |

| 增量 | CPU percentage points | 等效 logical CPU |
|---|---:|---:|
| B−A | +0.008 pp | +0.002 |
| C−B | +0.345 pp | +0.083 |
| C−A | +0.352 pp | +0.085 |

整机差分受后台负载噪声影响，特别是 B−A；独立 PID CPU 更适合描述 driver/fusion 自身成本。

### 进程 CPU

| Stage/process | mean | median | P95 | P99 | max | logical CPU |
|---|---:|---:|---:|---:|---:|---:|
| B driver | 4.710% | 5.000% | 6.000% | 6.000% | 6.000% | 0.047 |
| C driver | 4.873% | 5.000% | 6.000% | 7.000% | 7.000% | 0.049 |
| C fusion | 5.613% | 6.000% | 7.000% | 7.000% | 8.000% | 0.056 |
| C frame relay | 1.110% | 1.000% | 2.000% | 2.000% | 3.000% | 0.011 |
| C robot_state_publisher | 0.020% | 0.000% | 0.000% | 1.000% | 2.000% | <0.001 |

Cyclone/Fast DDS 工作线程位于上述 ROS 进程内部，没有独立的“DDS 进程”可单独归账。relay 与 robot_state_publisher 是可见的额外 ROS 进程。最高单核的 C 阶段平均仅 9.30%、P95 19.19%，不存在持续单核压力。

## 3. Memory

### 整机

RAM 定义为 `MemTotal − MemAvailable`。

| Stage | used mean | used P95 | used max | available mean |
|---|---:|---:|---:|---:|
| A | 19.027 GiB | 19.193 GiB | 19.236 GiB | 43.575 GiB |
| B | 18.898 GiB | 19.343 GiB | 19.504 GiB | 43.706 GiB |
| C | 19.209 GiB | 19.416 GiB | 19.578 GiB | 43.394 GiB |

- B−A：−132.3 MiB，属于页缓存/后台负载波动，不代表 driver 释放内存。
- C−B：+318.8 MiB。
- C−A：+186.4 MiB。

### 进程

| Process | RSS mean | RSS P95 | RSS max | VSZ mean | RSS linear slope |
|---|---:|---:|---:|---:|---:|
| B driver | 56.30 MiB | 56.79 MiB | 56.85 MiB | 1363.45 MiB | +0.161 MiB/min |
| C driver | 57.52 MiB | 57.86 MiB | 58.10 MiB | 1364.82 MiB | +0.006 MiB/min |
| C fusion | 38.13 MiB | 38.70 MiB | 39.98 MiB | 938.76 MiB | −0.155 MiB/min |
| C frame relay | 75.98 MiB | 75.98 MiB | 75.98 MiB | 931.54 MiB | ~0 |

B driver 的拟合增长在 5 分钟内总计不到 1 MiB；C driver 基本稳定，fusion 为负斜率。没有 `POSSIBLE_MEMORY_GROWTH` 证据。

## 4. Network

### 实际 Ethernet ingress

| Stage | RX mean | RX P95 | RX max | RX packets/s | TX mean |
|---|---:|---:|---:|---:|---:|
| A | 0.00012 MB/s | 0.00012 | 0.00012 | 未采集 | 0.00042 MB/s |
| B | 5.9669 MB/s | 5.9688 | 5.9703 | 4,570.9 | 0.00026 MB/s |
| C | 5.9669 MB/s | 5.9688 | 5.9717 | 4,570.7 | 0.00085 MB/s |

300 秒接口 counter：

- B RX：1,790,180,316 bytes，1,371,280 packets。
- C RX：1,790,134,608 bytes，1,371,246 packets。
- B/C RX errors、RX drops、TX errors、TX drops：全部 0。
- `sar EDEV` 的 `rxfifo/s` 全部为 0。

实际物理 ingress 约 47.74 Mbit/s，占 1 Gbit/s 链路约 4.77%。PointCloud2 payload 为 10.401 MB/s，明显高于 Ethernet ingress，因为 Livox wire packet 与 ROS PointCloud2 的点表示、字段和封装不同；二者不是同一层的字节数。

## 5. ROS topic data rate

PointCloud2 payload 使用 `row_step × height`，当前 layout 为 26 bytes/point。

| Topic | messages | Hz | points/msg | points/s | payload MB/s |
|---|---:|---:|---:|---:|---:|
| `/livox/lidar_192_168_1_116` | 约 100/10 s | 10.0003 | 20,000.32 | 200,009.16 | 5.20024 |
| `/livox/lidar_192_168_1_139` | 约 100/10 s | 10.0003 | 20,000.64 | 200,013.00 | 5.20034 |
| `/mid360/merged` | 2,949/300.011 s | 9.8297 | 23,899.83 | 234,927.04 | 6.10810 |

## 6. Callback and processing timing

正式 fusion 节点已有逐帧内部 timing，因此没有修改正式节点。

| Metric | mean | median | P95 | P99 | max |
|---|---:|---:|---:|---:|---:|
| Front crop work proxy | 1.742 ms | 1.711 | 1.817 | 2.538 | 2.692 |
| Rear crop work proxy | 1.630 ms | 1.602 | 1.700 | 2.375 | 3.320 |
| Front TF | 0.960 ms | 0.950 | 0.994 | 1.342 | 1.543 |
| Rear TF | 0.800 ms | 0.792 | 0.821 | 1.120 | 1.289 |
| Merge | 0.089 ms | 0.051 | 0.244 | 0.258 | 0.383 |
| Total pair processing | 5.311 ms | 5.210 | 5.595 | 7.608 | 8.002 |

Crop 是 callback 中主要的单流工作，但不是完整 callback wall time；在不修改正式节点的前提下无法把 rclcpp callback 框架开销单独拆出。

独立轻量 callback-enter 测量：

- Front/Rear callback-enter 绝对差：mean 0.545 ms、P95 0.688 ms、P99 0.752 ms、max 1.037 ms。
- Front inter-arrival：mean 100.570 ms、P95 100.519 ms、P99 101.015 ms；max 200.277 ms。
- Rear inter-arrival：mean 100.536 ms、P95 100.554 ms、P99 101.061 ms；max 200.922 ms。

约 200 ms 的最大 inter-arrival 对应某一 100 ms 周期消息未进入该订阅器，而不是长时间 callback 堵塞。该测量是 callback 入口的 monotonic 时间，不是内核级 executor scheduling latency；它给出了调度/到达抖动的上界数量级。

## 7. Orphan correlation

C 阶段 47 次 miss 在累计计数中表现为 25 个事件窗口，其中两次各集中清理 10 个旧 orphan。每个事件使用下一次成功配对时间作为近似定位，并统计 ±2 秒资源窗口。

| Orphan 窗口指标 | mean | observed max |
|---|---:|---:|
| system CPU mean | 2.335% | 5.537% |
| system CPU window max | 3.052% | 8.660% |
| driver CPU mean | 4.880% | 5.500% |
| fusion CPU mean | 5.410% | 6.000% |
| RAM used mean | 19.185 GiB | 19.525 GiB |
| eno1 RX mean | 5.9670 MB/s | 5.9685 MB/s |
| eno1 RX window max | 5.9686 MB/s | 5.9702 MB/s |
| network drop rate max | 0 | 0 |
| next successful pair total time | 5.273 ms | 6.452 ms |
| callback total window max | 6.085 ms | 7.903 ms |

这些窗口与整体 C 稳态相比没有 CPU、RAM、RX 或 callback 异常升高：

- orphan 窗口 system CPU mean 2.335%，整体 C mean 2.509%；
- orphan 窗口 fusion CPU mean 5.410%，整体 fusion mean 5.613%；
- orphan 后下一成功帧 5.273 ms，整体 total mean 5.311 ms；
- RX 保持约 5.967 MB/s，drop/error 为 0。

因此没有观察到 orphan 与整体 CPU spike、callback spike、RAM 压力或物理网卡丢包的正相关。此结论不证明 DDS/subscriber/executor 一定是根因，但保留它们作为更合理的后续方向。

## 8. Final assessment

**RESOURCE_PRESSURE = NO**

依据：

- fusion 与 driver 各自只使用约 0.05 个逻辑 CPU；
- 最忙单核也远未持续接近饱和；
- fusion RSS 稳定且没有单调增长；
- 系统仍有约 43.4 GiB available RAM；
- `eno1` 只使用约 4.77% 的 1 Gbit/s 链路；
- 网络 drop/error 为 0；
- fusion total P99 7.61 ms，远小于约 100 ms 的帧周期；
- orphan 窗口没有资源或 callback 峰值。

当前没有证据表明 orphan 是由整体资源耗尽导致。后续若继续诊断，应优先在 driver publisher、DDS reader、frame relay 输入/输出和 fusion subscription 四个边界分别增加轻量 sequence/stamp counter，再判断丢失发生在哪一跳；本阶段没有实施这些修改。
