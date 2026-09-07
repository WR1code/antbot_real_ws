# GUI 运动矩阵验收

状态：**未执行，不能判定通过**。当前自动化会话只能可靠执行 headless ROS/Isaac
测试，不能实际观察 Isaac GUI 与 RViz。下表是必须由操作者填写的终验记录，不用
headless 结果代替。

| # | 动作 | front Hz | rear Hz | filtered Hz | Δt max/P95 | TF | 方向/拖影/旧帧/异常栈 | 结果 |
|---:|---|---:|---:|---:|---:|---|---|---|
| 1 | 静止 | — | — | — | — | — | 未执行 | PENDING |
| 2 | 前进 | — | — | — | — | — | 未执行 | PENDING |
| 3 | 后退 | — | — | — | — | — | 未执行 | PENDING |
| 4 | 左横移 | — | — | — | — | — | 未执行 | PENDING |
| 5 | 右横移 | — | — | — | — | — | 未执行 | PENDING |
| 6 | 左旋转 | — | — | — | — | — | 未执行 | PENDING |
| 7 | 右旋转 | — | — | — | — | — | 未执行 | PENDING |
| 8 | 左前 45° | — | — | — | — | — | 未执行 | PENDING |
| 9 | 右前 45° | — | — | — | — | — | 未执行 | PENDING |
| 10 | 左后 45° | — | — | — | — | — | 未执行 | PENDING |
| 11 | 右后 45° | — | — | — | — | — | 未执行 | PENDING |
| 12 | 复合运动 | — | — | — | — | — | 未执行 | PENDING |
| 13 | 停止 | — | — | — | — | — | 未执行 | PENDING |
| 14 | 仿真暂停 | — | — | — | — | — | 未执行 | PENDING |
| 15 | 仿真恢复 | — | — | — | — | — | 未执行 | PENDING |
| 16 | 仿真重置 | — | — | — | — | — | 未执行 | PENDING |

执行时用 navigation profile 启动 GUI 与
`dual_lidar_bringup.launch.py use_rviz:=true start_synchronizer:=true`，并持续观察
四个 `ros2 topic hz`、`SYNC_STATS`、两条 `tf2_echo` 和 Isaac/RViz console。
每项至少稳定 20 秒；reset 后要求同步器/预处理不崩溃、stamp 回退有明确告警且新
数据恢复。RViz 固定帧为 `base_link`，必须观察 raw 与 filtered 的方向、车体回波、
墙面重合及运动拖影。
