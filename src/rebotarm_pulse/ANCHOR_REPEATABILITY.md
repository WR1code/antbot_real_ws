# Piper-H Anchor Repeatability

该节点在正式手眼标定前，自动测量回 Anchor 后真实反馈相对指定目标的关节角误差。它不改动控制器、不发布关节控制 topic、不生成仿真反馈，不需要相机或标定板。

## 已有接口及复用路径

- `rebotarm_demo_rviz/src/demo_panel.cpp` 的关节目标移动启动代码调用 `rebotarm_pulse/joint_target_move`，将 MoveIt 规划服务 remap 到 `/piperh/plan_kinematic_path`，执行 action remap 到 `/piperh/execute_trajectory`，Xbox 状态 remap 到 `/piperh/xbox/armed`。
- `rebotarm_pulse/joint_target_move.py` 的 `JointTargetMove.run()` 校验关节范围和奇异位形保护、要求 Xbox LOCKED、请求碰撞检查规划、预览并执行。
- `rebotarm_pulse/piperh_auto_handeye.py` 的 `_return_anchor()` 调用 `_execute(..., goal_tolerance=0.0001)`，通过 `/piperh/plan_kinematic_path` 规划，再使用 `/piperh/arm_controller/follow_joint_trajectory` 返回 Anchor。测试沿用 `0.0001 rad` 的规划容差，并复用 demo 的完整 MoveIt 执行链路。
- `piperh_control/launch/hardware.launch.py` 将真实驱动的 `joint_states_single` remap 到 `/piperh/driver_joint_states`。测量及规划起点只使用这个 topic。

新增的 `AnchorRepeatability` 继承 `JointTargetMove`，每次移动调用原有 `run()`；只在测试节点中增加真实反馈校验、轨迹终点校验、执行中安全检查和超时/中断取消。既有控制器容差不变。

## 构建和运行

先启动工程已有的 Piper-H **真机**驱动及 MoveIt，保持 Xbox LOCKED；不要同时运行其他自动运动或标定程序。以下构建/测试命令不会驱动机械臂；最后一条 launch 命令会立即开始真机运动。

```bash
cd /home/w/project/antbot_real_ws/dual_arm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --base-paths src/rebotarm_pulse --packages-select rebotarm_pulse --symlink-install
source install/setup.bash
ros2 launch rebotarm_pulse anchor_repeatability.launch.py \
  iterations:=20 settle_sec:=2.0 output_dir:=$HOME/.ros/anchor_repeatability
```

也可以直接运行，程序自带与面板相同的 Piper-H 接口 remap：

```bash
ros2 run rebotarm_pulse anchor_repeatability --ros-args \
  -p iterations:=20 -p settle_sec:=2.0 -p output_dir:=/tmp/anchor_repeatability
```

`dual_arm_ws/COLCON_IGNORE` 会阻止默认目录发现，因此构建时须显式指定上面的 `--base-paths`。

项目根目录 `src/rebotarm_pulse` 中同步提供相同节点和 launch，可在根工作空间构建同名包；不要同时构建两份同名包。

## 行为和参数

固定 Anchor（rad）：`[-0.15216, 1.50210, -1.55853, -0.26415, 1.00740, -1.93523]`。

先安全移动到 Anchor，然后循环：离开、等待稳定以确认实际离开、返回、等待稳定、记录。默认只对 J1 交替施加 ±8°，J2–J6 的离开目标保持 Anchor 值。所有目标通过原有范围/奇异保护和 MoveIt 碰撞检查，不绕过规划失败。

| 参数 | 默认值 | 含义 |
|---|---|---|
| `iterations` | 20 | 循环次数 |
| `settle_sec` | 2.0 | 至少连续稳定的时间 |
| `settle_timeout_sec` | 15.0 | 等待稳定超时，需大于稳定时间 |
| `stationary_range_deg` | 0.02 | 稳定窗口内任意关节的最大角度范围 |
| `moveit_feedback_topic` | `/piperh/joint_states` | MoveIt 执行校验所用的状态流，仅用于预检，测量仍来自原始驱动 topic |
| `feedback_timeout_sec` | 0.75 | 真实反馈最大允许年龄 |
| `departure_offset_rad` | `[0.13962634015954636, 0.0, 0.0, 0.0, 0.0, 0.0]` | 六关节离开偏移，奇偶轮取正负 |
| `velocity_scaling` / `acceleration_scaling` | 0.05 | 沿用面板安全移动默认值 |
| `joint_tolerance` | 0.0001 | MoveIt 目标和规划终点容差（rad） |
| `result_timeout` | 60.0 | 执行 action 超时 |
| `output_dir` | 当前目录 | CSV 保存位置 |

节点持续处理反馈，以名字匹配六关节，允许附带 gripper；拒绝缺失、重复、非有限值、过期、零时间戳及重复/倒退时间戳。禁止 `use_sim_time=true`。稳定判断只使用返回之后的新反馈窗口；记录窗口末尾的真实关节值，不以目标值或窗口平均值代替。Xbox 解锁、反馈失联或动作失败即停止测试；活动轨迹请求取消，不能宣告 PASS。

## 输出

逐轮输出六个有符号角度误差 `(current - anchor) * 180/pi`，CSV 每轮刷新，文件名 `anchor_repeatability_YYYYMMDD_HHMM.csv`。日期按 Asia/Shanghai，CSV timestamp 使用该次驱动反馈的时间戳（ISO 8601，+08:00）。同一分钟已有同名文件时拒绝覆盖。

完成全部轮次后输出各关节 mean、总体标准差 std（分母 N）、max_absolute、min、max，单位 deg；只有全部关节的 max_absolute **严格小于 0.1°** 才输出 PASS，否则输出 FAIL 并指出最大误差关节。退出码 PASS=0，FAIL/中止=1。中止时 CSV 保留已完成轮次，不能作为完整测试结果。

这些统计是相对给定 Anchor 目标的误差：mean 包含定位偏差，std 表示重复回位的离散程度。本程序未在开发验证中驱动真机；实际 PASS/FAIL 必须通过现场真实反馈得出。

## 执行前被中止时的排查

`ExecuteTrajectory status=6, CONTROL_FAILED (-4)` 只说明 MoveIt 执行被中止，具体原因见 `/piperh/move_group` 日志，不能据此判断重复定位精度 FAIL。

本次现场日志显示 `Failed to validate trajectory: couldn't receive full current joint state within 1s`：`/piperh/driver_joint_states` 持续更新，但 `/piperh/joint_states` 无新消息。只读 `/piperh/control_state/dump` 返回原始驱动反馈新鲜、适配器直接 CAN 反馈过期、`enabled=false`。现有硬件状态链路和电机使能尚不满足执行条件。

新版本在初次规划前及执行过程中检查 `/piperh/joint_states` 的完整性和新鲜度，异常输出 `MOVEIT_STATE_STALE`，不会通过伪造状态或放宽 MoveIt 校验继续执行。已终止的 action 不再重复取消。

可使用下面的只读命令检查现有链路：

```bash
ros2 topic hz /piperh/driver_joint_states
ros2 topic hz /piperh/joint_states
ros2 service call /piperh/control_state/dump std_srvs/srv/Trigger '{}'
```

通过工程现有硬件启动和面板流程恢复适配器反馈，确认 `/piperh/joint_states` 持续更新，并按既有流程确认电机使能和 Xbox LOCKED，然后重新运行测试。测试节点不负责重启控制器或改变电机使能。
