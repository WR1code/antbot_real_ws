# Phase 2A 收尾

日期：2026-07-26

```text
Phase 2A PARTIAL
TIME_SEMANTICS=DEFERRED_TO_REAL_HARDWARE_OR_LIO_RESEARCH
GUI_VISUAL_CHECK=PENDING
FAST-LIO NOT STARTED
```

## 已保存证据

- 五段 native MCAP 位于 `artifacts/phase2a_bags_native/`。
- 严格覆盖清单位于
  `artifacts/phase2a_minimal_dataset_manifest_native.yaml`；所有有效 lidar
  帧均有完整 IMU 区间覆盖，`/clock` 无重复与回退。
- 5 分钟稳定性结果：两路 raw 均为 3455 帧、10.000 Hz、missing 0；IMU 为
  41458 样本、120.000 Hz；truth validator 有效输出约 9.997 Hz，无持续队列
  增长或性能过载丢帧。
- 最后一次已经启动的柱体 ROI 重算结果保存在
  `artifacts/phase2a_semantics_audit/`；收尾检查时无运行中重算，因此未重启。

## 已知 metric/ROI 错误

墙面 metric 原先以 cube 中心作为命中真值。墙厚 0.12 m，ray 命中的是内表面，
所以该常量产生固定 0.06 m 误差，甚至可能让 raw 看起来优于 deskew。实现已改用
真实内表面。

圆柱 ROI 原先半径容差过宽，曾把距柱心约 0.44 m 的墙面点混入柱体样本；ROI 已
收紧。圆柱径向指标对切向拖影也不敏感，因此不再用它扩展时间模型选择研究。

## 冻结边界

最后一次 sweep 的候选排名保留为实验观测，但不构成唯一时间语义。A/C/D 的最终
状态是 `DEFERRED_TO_REAL_HARDWARE_OR_LIO_RESEARCH`。不再新增几何、速度、
旋向、录包或 truth deskew 优化。`truth_deskew` 与 `world_reference` 只能用于
验证，禁止进入正常定位和导航链路。

## 收尾构建与测试

在 `/home/w/project/antbot/ros2_ws`、ROS 2 Jazzy 环境中执行：

```text
python3 -m pytest -q src/antbot_dual_lidar/test
37 passed in 0.23s

colcon build --symlink-install --packages-up-to antbot_dual_lidar
Summary: 1 package finished [0.92s]

colcon test --packages-select antbot_dual_lidar --event-handlers console_direct+
37 passed in 0.23s
Summary: 1 package finished [0.64s]

colcon test-result --verbose
Summary: 108 tests, 0 errors, 0 failures, 0 skipped
```

`colcon test-result` 的 108 是当前工作区测试结果目录中的累计用例数；本次
`antbot_dual_lidar` 控制台明确收集并通过 37 项。
