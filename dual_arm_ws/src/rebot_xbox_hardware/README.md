# rebot_xbox_hardware

Unified real-arm integration for the shared Xbox mapping. The launch-time
`robot` interface selects reBotArm or Piper-H together with its hardware driver,
URDF/SRDF, joint limits, MoveIt configuration, RViz layout, topics and safety
adapters. Use only after checking the physical emergency stop and clearing the
robot workspace.

两台机械臂分别接入独立 USB-CAN（按当前上位机配置：reBotArm DM 桥=`/dev/ttyACM2`、
Piper-H=`can0`）时，
日常只执行这一条固定命令：

```bash
cd /home/w/project/rebotarm/real_robot_ws
source ./activate.sh
ros2 launch rebot_xbox_hardware dual_arm_hardware.launch.py
```

Piper-H J6 三路把脉探头可通过同一个入口接入原始串口数据：

```bash
ros2 launch rebot_xbox_hardware dual_arm_hardware.launch.py \
  piper_pulse_serial_port:=/dev/serial/by-id/<ESP32-S3的实际设备名>
```

默认会启动 Piper-H 腕点坐标观测，并在 RViz 显示照片估计的约 110 mm 支架；该灰色模型
标记为未标定，不发布测压口 TCP，也不开放人体接触运动。网页或桌面采集工具与 ROS 桥
不能同时占用 ESP32-S3 串口。

这会让 ANTBot H743 底盘、两套机械臂驱动、各自的 URDF/SRDF、MoveIt、Servo、关节反馈和安全禁区同时常驻。
只启动一个 RViz：左侧主页为“机械臂选择 / 机械臂控制 / 底盘控制”，右侧同时显示
ANTBot STEP 整机底盘和两台机械臂。“机械臂控制”内的自由 MoveIt 规划仍在原位；
“底盘控制”直接嵌入原车辆控制中心和航点巡航插件，没有重写或裁剪其功能。
RViz 左侧 **双臂统一工作台** 可在 `reBotArm` 与 `Piper-H` 间切换共享 Xbox 控制权，
不需要重启程序。切换时系统先广播 `none`、请求取消两边尚未结束的 MoveIt/控制器动作，
确认两套 Xbox 节点都为 `LOCKED` 后才交接；
新机械臂仍需摇杆回中并按 A 解锁。原生 MoveIt 面板的命名空间和 Robot Description 也随
选择同步切换；三维视图把底盘和两台机械臂作为一个整机常驻显示，控制权切换只改变命令归属。
底盘运行网格已去掉 CAD 内固化的 Piper-H，实时模型默认安装在原法兰位置，
按已改装的右侧装设置 `piper_roll=+90°`、`piper_pitch=0°`、`piper_yaw=0°`。
现场复测后可用 `piper_x`、`piper_y`、`piper_z` 和
`piper_roll`、`piper_pitch`、`piper_yaw`（米、弧度）覆盖默认安装姿态，无需再修改模型文件。
Piper-H 的 MoveIt 自由规划模型与真机 TF 共用这组安装参数和同一个底盘安装框架，因此橙色
目标/轨迹预览会直接覆盖在蓝色实时机械臂上，不再单独出现在全局原点。
每次启动还会先执行双重冲突检查：进程锁阻止另一套工作台并发启动，进程与 ROS 图扫描会
检查上一轮残留的驱动、MoveIt、Servo、TF、RViz、Xbox 和控制节点。发现残留时不会打开硬件，
终端会列出冲突 PID 和命令；清理所列进程后重新执行同一启动命令即可。
若某台机械臂驱动尚无完整关节反馈，它的模型会保持隐藏并在
控制页提示等待反馈，避免 RViz 把缺少 TF 的模型渲染成白色错误轮廓。两台机械臂的预设动作分别保存、分别
校验型号和关节限位，不能跨机械臂误加载；选择 Piper-H 后可在 **机械臂控制 -> 演示与动作**
使用同一套六关节动作点、动作组和顺序回放。Piper-H 的 **真机与安全** 页把电机使能与
Xbox 实时控制分成两个按钮，另提供停止全部运动和禁区重载，未连接手柄不会影响动作编排或
MoveIt。

不连接任何真实机械臂时，用同一套选择界面启动双臂离线预览：

```bash
source /home/w/project/rebotarm/real_robot_ws/activate.sh
ros2 launch rebot_xbox_hardware dual_arm_offline_preview.launch.py rebot_model:=dm
```

此入口为 reBotArm 和 Piper-H 各启动一套 MoveIt 假硬件。“机械臂选择”仍然可用，但切换
只改变 RViz 模型、规划命名空间和动作库；串口/CAN 驱动、Xbox 和所有真机输出均不启动。

若设备名不同，仍是同一个启动指令，只需传入现场接口名。例如 reBotArm RS 和
Piper-H 都使用 SocketCAN 时：

```bash
ros2 launch rebot_xbox_hardware dual_arm_hardware.launch.py \
  rebot_model:=rs rebot_channel:=can_rebot piper_channel:=can_piper
```

RS 与 Piper 的两个参数不能指向同一个 CAN 接口。建议用 udev/systemd 固定命名，避免
USB 拔插后接口对调。Piper 驱动仍保持 `auto_enable=false`，对应使能服务为
`/piperh/enable_srv`。

旧的 `hardware_selector.launch.py` 仍保留为“每次只启动一台”的兼容入口，适合 DM 串口版
或不需要双机械臂常驻的场景。

以下带 `robot:=` 的命令是单机械臂底层接口，供无界面调试或自动化使用。

本机 DM 串口版使用：

```bash
ros2 launch rebot_xbox_hardware xbox_hardware_servo.launch.py \
  robot:=rebotarm model:=dm channel:=/dev/ttyACM2
```

Piper-H CAN 版使用同一个入口：

```bash
ros2 launch rebot_xbox_hardware xbox_hardware_servo.launch.py \
  robot:=piperh channel:=can0
```

`robot:=rebotarm` 是默认值，因此原来的 DM/RS 命令继续有效。这个参数是安全的
**启动时选择**，不支持机械臂已使能时热切换；切换前应先锁定 Xbox、停止轨迹、失能当前
机械臂并完整退出 launch，再用另一种 `robot` 重启。

两种分支复用同一套 A 键解锁、回中检查、Start 急停、手柄断联锁定、三档速度、基座/末端
坐标切换、MoveIt Servo、碰撞规划、禁区和单实例保护。选择 Piper-H 时自动加载六轴无夹爪
模型、`Link6` 末端和 Piper 关节限位；选择 reBotArm 时加载 DM/RS 对应模型、
`gripper_tcp` 和夹爪适配器。两者都使用“机械臂控制”内原有的预设动作和自由运动规划。
Piper-H 项目模型本身没有夹爪；拖动示教只读取普通关节反馈。默认模式为电机失能后的
`passive_disabled` 录制。Pinocchio + MIT 重力补偿代码仅作为显式验证后的可选模式，默认
配置关闭真实力矩且不填写逐轴力矩上限。当前启动文件记录的 MoveIt/RViz 安装角为
`(+90°,0°,0°)`，但它不能代替现场确认重力坐标；确认前只能运行只读 dry-run。
Piper 官方驱动仍然 `auto_enable=false`，启动后必须完成现场
检查后通过界面的电机使能按钮（标准接口 `/piperh/motor/set_enabled`）使能；底层仍由
`/piperh/enable_srv` 调用厂商驱动。

reBotArm 分支会启动驱动、MoveIt、RViz、Servo 安全仲裁、夹爪适配器以及型号匹配的拖动
示教/动作回放节点。RViz 配置中的“机械臂控制”由 `rebotarm_demo_rviz` 原面板完整嵌入。

DM 启动后保持 Xbox `LOCKED`：按住 X 拖动录制，松开 X 保存并在 RViz2 循环预览；按 B
回放当前动作；Start 取消。刚录完、选择/重载动作和回放时都会向
`/display_planned_path` 发布轨迹。可用 `use_teach:=false` 关闭整个示教模块，或用
`teach_use_xbox:=false` 仅保留 ROS 服务。

该入口带单实例锁。同一 `arm_namespace` 已经运行时，第二次启动会在打开串口和创建
MoveIt/RViz 节点之前退出，并显示 `REFUSING DUPLICATE LAUNCH`。不要同时运行
MotorBridge Studio Gateway、`rebotarm_bringup` 或另一个 `hardware.launch.py`；它们会
争用同一串口或控制器。

升级前启动、尚未持有锁的旧进程也会被 ROS 图预检识别；发现已有
`reBotArmController`、`move_group`、Servo、RViz 或 Xbox 核心节点时，新入口同样拒绝
启动，避免升级切换期间形成两套 Action 服务。

MoveIt Servo 会先成功选择 TWIST 命令类型，再开始发布零帧或手柄指令，因此启动阶段
不应再出现 `Command type has not been set, cannot accept input`。RViz 若异常崩溃会在
2 秒后由同一个 launch 自动重启，不需要再次启动整套组合入口。

真机轨迹动作只有在最终关节误差不超过 0.005 rad 且速度已经停止后才报告成功；路径
跟踪误差超过 0.03 rad 会立即中止并保持当前位置。MoveIt 的执行起点容差为 0.01 rad。

## 三维禁入区域

RViz 的 **机械臂控制 -> 真机与安全 -> 三维禁止通行区域** 提供现场禁区编辑器：可选择
长方体、球体、圆柱体或圆锥体，填写尺寸、基座坐标系 XYZ 和 RPY 后直接加入 MoveIt；也可
选择本地 `.stl`（二进制/ASCII）或 `.obj` 网格，并设置统一缩放。毫米建模文件通常使用
`0.001`，但必须根据模型实际单位核对。导入文件限制为 50 MiB，并限制顶点和三角面数量，
无效或非有限几何会被拒绝。面板只能删除由面板创建的用户禁区，不会误删基础安全配置。

面板创建的区域保存在 `~/.ros/rebotarm/forbidden_zones_user.yaml`，上传的网格会复制到同目录
下由程序管理的 `forbidden_zone_meshes/`，因此移动或删除原始上传文件不会破坏禁区。启动时
用户区域会与下述基础配置合并，所以重启后仍然生效。导入、删除和重新加载都只更新 MoveIt
Planning Scene，不会发送真机运动命令。

所有已加载的禁区会以红色显示；用户禁区还带红色六自由度拖动模型。拖动箭头可沿 X/Y/Z 平移，
拖动圆环可绕对应轴旋转；拖动过程中 MoveIt 碰撞对象实时跟随，松开鼠标后新位姿会原子写回
用户配置。基础 YAML 中的固定安全区不会生成拖动手柄，避免现场安全边界被误改。

面板可从多个用户禁区创建、替换和删除命名禁区组，组名及成员同样持久化到用户 YAML；删除组
不会删除其中的禁区。RViz 仅显示真正已加入 MoveIt 碰撞场景的禁区，不叠加几何可达边界球。

编辑 `config/forbidden_zones.yaml` 可以增加命名的禁入区域。支持 `box`（长方体）、
`sphere`（球体）、`cylinder`（圆柱）和 `cone`（圆锥）。每个区域包含名称、尺寸、相对
`frame_id` 的位置和 RPY 姿态；`groups` 可将多个区域组合成命名区域组。只有
`enabled: true` 的区域才会加载，默认加载所有 `enabled: true` 的组。未加入任何组的
已启用区域会始终加载。

也可以在启动时只选择指定的组（多个名称用英文逗号分隔）：

```bash
ros2 launch rebot_xbox_hardware xbox_hardware_servo.launch.py \
  model:=dm channel:=/dev/ttyACM2 active_zone_groups:=workcell_safety
```

区域会以碰撞物体进入 MoveIt Planning Scene，RViz 规划和 Xbox Servo 均禁止机械臂
连杆触碰。修改 YAML 后无需重启整套程序，可执行：

```bash
ros2 service call /forbidden_zone_manager/reload std_srvs/srv/Trigger '{}'
ros2 topic echo --once /forbidden_zone_manager/status
```

若不需要该功能，可加 `use_forbidden_zones:=false`。首次启用现场区域前，应保持急停可用，
先在 RViz 核对形状和坐标，再低速试运行；这里的软件碰撞区不能替代实体围栏和急停。

Start locked. Center all controls and press A to arm. Press A again before RViz
Plan & Execute. LT opens and RT closes proportionally; release holds position.
