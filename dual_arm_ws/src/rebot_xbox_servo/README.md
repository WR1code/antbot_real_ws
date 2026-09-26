# rebot_xbox_servo

用于 reBotArm MoveIt Servo 的 Xbox 输入共享与安全锁定功能。

该功能包中的 `xbox_moveit_servo.launch.py` **仅用于仿真环境**：它负责驱动模拟夹爪，并通过 UDP 将 `/joint_states` 转发到 Isaac Sim。

真实硬件使用独立的 `rebot_xbox_hardware` 功能包，且绝不会启动这些仿真适配节点。

仓库中提交的手柄映射参数，是根据当前连接的 `Generic X-Box pad` 实际测量得到的。如果更换了手柄型号或 Linux 驱动程序，请使用 `inspect_joy` 重新检查映射：

```bash
cd /home/w/project/rebotarm/simulation_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run rebot_xbox_servo inspect_joy --ros-args \
  --params-file src/rebot_xbox_servo/config/xbox_mapping.yaml
```

实测的水平方向摇杆符号保持“向右为正”。

在当前要求的机械臂垂直安装姿态下：

* 左摇杆向上推动时，产生负方向的线速度 X。
* 右摇杆向上推动时，产生负方向的线速度 Z。
* 十字方向键控制俯仰角时，保持手柄实测得到的方向。

要启动完整的 Isaac Sim 控制链路，需要打开两个终端。

终端 1：

```bash
cd /home/w/project/rebotarm/simulation_ws
./scripts/start_isaac_receiver.sh
```

终端 2：

```bash
cd /home/w/project/rebotarm/simulation_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch rebot_xbox_servo xbox_moveit_servo.launch.py
```

该启动文件会启动以下组件：

* MoveIt 模拟硬件；
* MoveIt Servo；
* Xbox 安全控制节点；
* 仿真夹爪适配节点；
* 将 `/joint_states` 通过 UDP 发送到 Isaac Sim 的桥接节点，目标地址为 `127.0.0.1:5005`。

不要同时运行 `moveit_isaac_demo.launch.py`。

RS 模拟机械臂现在会从真实机械臂的休眠姿态启动，六个机械臂关节的初始值均为零。

随后，`arm_initializer` 会发送一条持续 6 秒的关节轨迹，将机械臂运动到：

```text
[0.0, 1.75, 0.7, -0.7, 0.0, 0.0]
```

只有当该轨迹到达目标位置后，MoveIt Servo 和 Xbox 指令节点才会启动。因此，在初始化期间，手柄运动控制始终处于锁定状态。

仅在调试时设置：

```text
start_initializer:=false
```

初始化节点会明确等待 `rebotarm_controller` 控制器报告 `active` 状态。

控制器发现过程拥有独立的 30 秒超时时间，因此不会占用用于等待轨迹完成的 12 秒时间窗口。

Xbox 节点启动时，终端会输出当前实际生效的测量方向：

```text
[XBOX SERVO] DIRECTIONS: left up=-linear.x, left right=+linear.y, right up=-linear.z, right right=+angular.z
```

对应关系为：

* 左摇杆向上：`-linear.x`
* 左摇杆向右：`+linear.y`
* 右摇杆向上：`-linear.z`
* 右摇杆向右：`+angular.z`

## 夹爪按键映射

* LT 左扳机键会将归一化后的 `0.0..1.0` 行程发布到：

  ```text
  /rebot_xbox/gripper_open
  ```

  并打开两个仿真夹爪手指。

* RT 右扳机键会将归一化后的 `0.0..1.0` 行程发布到：

  ```text
  /rebot_xbox/gripper_close
  ```

  并关闭两个仿真夹爪手指。

* 扳机按下行程会按比例缩放夹爪运动速度。

* 松开扳机后，节点会发布零速度，并保持当前实测到的夹爪手指位置。

* 扳机完全按下时，最大夹爪速度为：

  ```text
  0.045 m/s
  ```

* 5% 的死区用于抑制扳机输入噪声。

* B 键和 X 键当前没有分配功能。

* 当 Xbox 控制器处于 `LOCKED` 状态时，所有夹爪控制事件都会被阻止。

* 夹爪闭合采用位置控制，不具备接触检测或力限制功能。请将其视为仿真控制指令，而不是防夹保护功能。

## 控制器所有权

控制器所有权已经被明确管理，因此 RViz MotionPlanning 和 Xbox Servo 可以安全地共享 `rebotarm_controller`。

### `LOCKED`

* MoveIt Servo 处于暂停状态。
* 可以拖动 RViz 中的目标交互标记。
* 可以使用 **Plan & Execute** 控制仿真机械臂。

### `ARMED`

* MoveIt Servo 处于激活状态。
* Xbox 手柄拥有笛卡尔空间点动控制权。
* 在通过 RViz 执行下一次规划之前，应先重新切换到锁定状态。

除手柄 A 键外，综合 RViz 面板通过以下幂等服务切换控制权：

```text
/rebot_xbox/set_armed  std_srvs/srv/SetBool
```

请求 `data=true` 时会强制检查手柄在线、输入未超时、控制目标为机械臂和全部摇杆回中；
有夹爪的机械臂还会检查两个扳机已经初始化并松开。无夹爪机型可通过
`safety.require_released_triggers_to_arm=false` 跳过不存在的扳机映射。请求 `data=false`
始终锁定并立即发布零速度。

RViz 中的交互标记表示的是规划目标，而不是机械臂 TCP 的实时位置指示器。

要观察 MoveIt Servo 控制下的实际运动，请查看：

* RViz 中的机械臂模型；
* `base_link -> gripper_tcp` 的 TF 变换。

预设位姿服务目前被有意保留，不会生成任何 Twist 速度指令：

```text
/rebot_xbox/go_to_preset  std_srvs/srv/Trigger
```
