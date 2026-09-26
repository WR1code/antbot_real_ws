# 双 Livox MID360S 实机增量性能报告

测试日期：2026-09-17  
平台：Ubuntu 24.04 x86_64、ROS 2 Jazzy、Intel i7-13700KF（24 逻辑 CPU）、62.60 GiB RAM

## 环境排查结论

- 复用了项目 `/home/w/project/odas/avtwin_linux` 中已有的固定版本部署方案。
- `livox_ros_driver2`：1.2.6；`Livox-SDK2`：v1.3.1。
- 驱动 overlay：`/home/w/project/odas/.deps/mid360s_ws/install/setup.bash`。
- `ros2 pkg prefix livox_ros_driver2` 返回 `/home/w/project/odas/.deps/mid360s_ws/install/livox_ros_driver2`。
- 唯一实际有线雷达口为 `eno1`（Realtek RTL8125，1 Gbit/s full duplex）。
- 实际雷达为 `192.168.1.116`（SN `ARMCP5F0037716`，MAC `9c:5a:8a:5b:76:13`）和 `192.168.1.139`（SN `ARMCP680032239`，MAC `9c:5a:8a:d8:18:0f`）。旧候选 `.12/.13` 不可达。
- 测试期间仅临时使用 `192.168.1.50/24`；测试结束后 `eno1` 已恢复原 NetworkManager profile 与 `10.42.0.1/24`。
- 运行时配置保存在 `runtime/dual_mid360_actual.json`，没有修改融合算法、URDF 或 TF 外参。
- 物理摆位无法由当前对称外参和静态点云唯一判别，因此测试采用确定性逻辑映射 `.116 -> front_left`、`.139 -> rear_right`。此映射不影响资源、裁剪总量或延迟结论。

## A/B/C 对比

| 指标 | A 空载 | B 双驱动 | C 完整链路 | B−A | C−B | C−A |
|---|---:|---:|---:|---:|---:|---:|
| 整机 CPU avg | 2.156% | 2.164% | 2.509% | +0.008 pp | +0.345 pp | +0.352 pp |
| RAM avg | 19.027 GiB | 18.898 GiB | 19.209 GiB | −132.3 MiB | +318.8 MiB | +186.4 MiB |

RAM 使用 `MemTotal−MemAvailable`。B−A 的小幅负值来自页缓存/后台负载波动，不代表驱动释放内存；driver 独立 RSS 为 56.30 MiB。C−B 包含项目为 fusion 所需的 frame relay、robot_state_publisher 和 fusion；新增三进程 RSS 合计约 146.2 MiB。

完整链路相对空载的整机 CPU 增量为 0.352 个百分点；在 24 逻辑 CPU 上约等于 0.085 个逻辑 CPU。fusion 单进程平均 CPU 为 5.613%，即 0.056 个逻辑 CPU，平均 RSS 为 38.13 MiB。

## 最终关键结果

1. 两台 MID360 实际总输入：**400,022 pts/s**。
2. 原始 PointCloud2 payload：**10.401 MB/s**；物理网卡 RX 为 **5.967 MB/s / 47.735 Mbit/s**。
3. 270° 裁剪后：**约 239,026 pts/s**；同步合并实际输出 **234,927 pts/s**。
4. fusion 平均 CPU：**5.613%**。
5. fusion 相当于：**0.056 个逻辑 CPU 线程**。
6. fusion RSS：**平均 38.13 MiB，峰值 39.98 MiB**。
7. 完整系统额外 CPU（C−A）：**+0.352 个百分点**，约 **0.085 个逻辑 CPU**。
8. 完整系统额外 RAM（C−A）：**+186.4 MiB**；C−B 为 **+318.8 MiB**。
9. 平均总处理延迟：**5.311 ms**。
10. 总处理延迟 P95：**5.595 ms**；P99 **7.608 ms**，最大 **8.002 ms**。
11. 有 fusion 同步队列丢帧：正式 300 s 内 **47 clouds**；网卡层 **0 drop / 0 error**。
12. 有 sync miss：**47 次**，约占同步尝试的 **1.569%**。
13. 无持续单核瓶颈：最忙核心平均 **9.30%**、P95 **19.19%**；100% 仅为单个 1 s 瞬态。
14. 无网络瓶颈：1 Gbit/s 链路利用率约 **4.77%**，且没有接口错误或丢包。

## 原始证据

- A：`20260917_142845_A/`
- B：`20260917_150255_B/`，含 mpstat、pidstat、sar、网卡起止计数和阶段分析。
- C：`20260917_151026_C/`，含正式窗口时间戳、内部逐帧样本中的正式窗口裁片、进程/系统/网络原始采样、ROS graph 及阶段分析。

注：C 的 `fusion_metrics.csv` 覆盖预热、正式测试及短暂收尾；报告只使用 `formal_start_ns.txt` 到 `formal_end_ns.txt` 之间的 2,949 行。
