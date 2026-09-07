# 长期稳定性与压力测试

状态：完成约 10–12 秒有效采样的 mapping/navigation headless 性能测试；**没有
执行 10 分钟静止、10 分钟运动、RViz/rosbag 压力矩阵**，因此不能判为长期终验
通过。短测详见 `PERFORMANCE_BASELINE.md`：navigation 四路 10 Hz、P95/P99
0.1 s、无 stamp drop/message-lost；mapping 在当前 fan-out 下约 5.5–6.4 Hz 且
发生丢旧帧。

待执行边界矩阵：

| profile | 运动 | RViz | 录包 | 时长 | 状态 |
|---|---|---|---|---:|---|
| navigation | 静止 | off | off | 600 s | PENDING |
| navigation | 连续组合 | off | off | 600 s | PENDING |
| navigation | 静止 | filtered only | off | 600 s | PENDING |
| navigation | 静止 | raw + filtered | off | 600 s | PENDING |
| navigation | 静止 | off | filtered | 600 s | PENDING |
| navigation | 静止 | off | raw + filtered | 600 s | PENDING |
| mapping | 静止 | off | off | 600 s | PENDING |
| mapping | 静止 | off | raw + filtered | 600 s | PENDING |

基础静止命令为
`run_performance_case.sh navigation 600 <output.json>`；运动、RViz 和录包项应在
同样的 `performance_probe --duration 600` 期间只改变一个变量。每轮保存平均/
最低频率（探针当前保存完整 interval 可进一步求最小）、P95/P99、stamp gap、
message-lost、同步差、CPU/GPU/RSS 和消息大小。录包分别只列 filtered，以及
raw+filtered；RViz分别隐藏 raw 和显示全部。因本轮无人可可靠操作 GUI/连续控制，
未自动执行这些工况。
