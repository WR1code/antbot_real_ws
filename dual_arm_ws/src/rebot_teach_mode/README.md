# reBotArm 拖动示教

这个节点用于：用手拖动机械臂录制一个动作，保存为命名动作组，然后再复现。

当前支持 reBotArm DM 和 RS，记录六个机械臂关节，不记录夹爪。两个型号使用各自的
关节限位、最近轨迹文件和动作组库，不能跨型号回放。

## 一键启动

第一次使用需要先构建一次：

```bash
cd /home/w/project/rebotarm/real_robot_ws
./build.sh
```

以后直接运行下面的脚本，不需要手动 `source`：

```bash
/home/w/project/rebotarm/packages/rebot_teach_mode/scripts/start_teach_mode.sh
```

这是默认的**锁定模式**，可以查询动作组，但绝不会录制或回放。按 `Ctrl+C` 退出。

### 不连接真机时预览动作

不接串口/CAN 时，统一使用包含机械臂选择的综合工作台入口：

```bash
source /home/w/project/rebotarm/real_robot_ws/activate.sh
ros2 launch rebot_xbox_hardware dual_arm_offline_preview.launch.py rebot_model:=dm
```

该入口不启动串口/CAN 真机驱动，切换机械臂只会切换 RViz 模型、MoveIt 假硬件和对应动作
编辑环境。RViz 只显示一个综合工作台入口，机械臂选择、综合控制和运动规划都位于其中。

该入口仅使用 MoveIt 假硬件，不启动 `rebotarm_bringup` 或任何串口/CAN 驱动，并在示教后端
强制设置 `allow_hardware=false`。在“演示与动作 → 动作组编排”选择动作组后，点击
“播放勾选动作的 RViz 动画”即可看到模型连续运动；“真机回放”会显示为离线禁用。
不要与真机组合 launch 同时运行，以免 ROS 节点和话题重名。

## 真机示教怎么启动

真机驱动必须已经在另一个终端正常运行，实体急停有效，并且机械臂周围无人。示教
脚本本身不会启动 CAN 驱动，也不会自动使能机械臂。

DM 真机已经集成到组合入口，只需运行：

```bash
source /home/w/project/rebotarm/real_robot_ws/activate.sh
ros2 launch rebot_xbox_hardware xbox_hardware_servo.launch.py \
  model:=dm channel:=/dev/ttyACM2
```

该命令默认同时启动驱动、MoveIt、RViz2、Xbox、DM 示教节点和示教按键桥。无需再开第二个
示教 launch。要临时关闭示教功能可追加 `use_teach:=false`；只保留 ROS 服务、不使用
X/B 示教按键可追加 `teach_use_xbox:=false`。

RS 或需要独立启动示教节点时，真机驱动必须已经在另一个终端正常运行。先在终端 1 运行：

```bash
source /home/w/project/rebotarm/real_robot_ws/activate.sh
ros2 launch rebot_xbox_hardware xbox_hardware_servo.launch.py \
  model:=rs channel:=can0
```

然后保留终端 1，在终端 2 选择下面一种示教启动方式。独立启动 DM 时需给
`teach_mode.launch.py` 传入 `model:=dm`。

### 方式一：用 ROS 服务操作

```bash
/home/w/project/rebotarm/packages/rebot_teach_mode/scripts/start_teach_mode.sh --hardware
```

这个模式不监听手柄，适合第一次现场联调。

### 方式二：用 Xbox 操作

```bash
/home/w/project/rebotarm/packages/rebot_teach_mode/scripts/start_teach_mode.sh --xbox
```

手柄按键：

| 按键 | 功能 |
|---|---|
| 按住 X | 开始拖动录制 |
| 松开 X | 停止并保存为一个新动作组 |
| B | 复现当前动作组 |
| Start | 取消当前录制或回放 |

使用示教模式时，A 键必须保持 `LOCKED`，不要同时操作摇杆或夹爪扳机。每次按住并
松开 X 都会生成独立文件，例如 `示教动作_20260825_153000_123456`，不会覆盖上一次。

## 指定动作组名称

启动 `--hardware` 模式后，另开一个终端并激活工作空间：

```bash
source /home/w/project/rebotarm/real_robot_ws/activate.sh
```

开始录制名为 `greeting_wave` 的动作：

```bash
ros2 service call /rebotarm/teach/start_action_group \
  rebot_teach_msgs/srv/StartActionGroup \
  "{name: greeting_wave, description: 观众问候挥手, overwrite: false}"
```

用手拖动完成动作后，停止并保存：

```bash
ros2 service call /rebotarm/teach/stop_recording std_srvs/srv/Trigger "{}"
```

查看已经保存的动作组：

```bash
ros2 service call /rebotarm/teach/list_action_groups \
  rebot_teach_msgs/srv/ListActionGroups "{}"
```

按名称复现：

```bash
ros2 service call /rebotarm/teach/replay_action_group \
  rebot_teach_msgs/srv/ReplayActionGroup "{name: greeting_wave}"
```

复现前，机械臂必须接近该动作录制时的起点或终点。若在终点，节点会先沿原路径
慢速返回起点，再正向复现；处于其他未知姿态时会拒绝执行。

在组合入口的 RViz 面板中，也可以先按“开始拖动录制/停止并保存”，再选中自动生成的动作，
点击“重命名”改成自己的名称。

## 绘图形状动作包与 TCP 轨迹

RViz 面板的“绘图形状动作包”可生成矩形、三角形、圆形、五角星和心形，也可选择
“英文/数字/符号”或“图片转简笔画”。字符模式提供 A-Z、a-z、0-9、空格及
`.,:;!?+-*/=_#@%&()[]<>`，最多 24 个字符，并使用轻量单线字体转换为少量笔画。
图片模式支持 PNG/JPEG/BMP/WebP，本地完成灰度化、Canny 边缘提取、短轮廓过滤、折线简化
和点数限制；默认最多保留 24 条主要轮廓和 480 个路径点，不上传图片，也不需要联网模型。

字符和图片都可能包含多条不相连笔画。面板中的 `L` 是笔从实体夹爪前端继续向外露出的
长度（对应 `shape_pen_length_m`），而不是从夹爪内部的 `gripper_tcp` 开始量。型号配置会
自动加入 TCP 到夹爪前端的安装偏移：DM 为 106.4 mm，RS 为 157.2 mm。绘图位置始终表示
纸面上的笔尖位置，而不是夹爪位置。
实体笔从 `gripper_tcp` 的 +X 方向向外伸出，对应绘图平面的局部 -Z；局部 +Z 是朝机械臂
一侧的抬笔方向，笔始终垂直于绘图平面。节点会先在第一笔上方悬停再落笔；每条笔画之间沿 +Z 抬高
`shape_pen_lift_m`（默认
12 mm），移动到下一笔起点后再落笔；最后一笔结束后也先抬笔再退场，因此不会把进场、
空白区域或退场路线画到纸上。
紫色圆柱会同时附着到真实灰色机械臂和青色预览机械臂，青色球表示夹爪前端，黄色球表示
笔尖，红色箭头表示沿夹爪轴返回腕部的抬笔方向。生成前应先检查 RViz 中的青色目标笔画和
三维虚拟笔，必要时调整 W/H、位姿、笔长以及
图片简化参数。目标笔画在平面正反两侧各绘制一层，前后观察都可见；总运动待执行部分为
橙色，控制器确认已经经过的部分为绿色。生成时要求
MoveIt、最新关节反馈和 `LOCKED` 状态就绪，但不会驱动真机；节点会从当前姿态规划到形状、
沿形状连续运动并安全返回当前姿态，然后保存为 `形状包_*` 动作。若同名动作已存在，界面
会在覆盖前确认。

在生成前点击 **“预检当前笔尖轨迹（不动真机）”**，节点会按 `shape_cartesian_step_m`
加密笔尖路径（默认最多 60 个诊断点），并对每一点执行带虚拟笔碰撞体的 MoveIt IK：

- 绿色：存在无碰撞 IK；
- 黄色：存在无碰撞 IK，但任一关节距限位不超过 0.15 rad；
- 紫色：关闭碰撞检查后有 IK，说明带笔模型或机械臂在该点发生碰撞；
- 红色：即使关闭碰撞检查也没有 IK；
- 断续线段：抬笔、空白转移或退笔段。

预检失败后会在绘图平面的局部 XYZ 六个方向按 20 mm 步长搜索附近候选位置，最多搜索
60 mm。找到候选位置时面板只询问是否更新 RViz 画布，绝不会移动真机；应用候选后必须再次
预检。修改位置、方向、大小、内容、笔长或抬笔量都会使旧预检结果失效。形状规划使用的
RobotState 同样附带虚拟笔碰撞体，因此预检和最终规划对笔的几何处理一致。

每次切换内容、大小或位姿时，还会在 `/rebotarm/teach/drawing_path` 发布
`rebot_teach_msgs/msg/DrawingPath`。其中 `points` 是基座坐标系中的实体笔尖简化笛卡尔点，
`stroke_start_indices` 标出每条不相连笔画的起点，便于其他节点直接消费；MoveIt 生成的密集
关节轨迹仍按原方式发布到 `/display_planned_path`。消息中的 `pen_length_m` 和
`pen_lift_m` 记录生成该路径时采用的工具参数。

也可以直接调用服务。`source` 在字符模式中是文本，在图片模式中是节点可访问的绝对路径：

```bash
ros2 service call /rebotarm/teach/create_shape_action \
  rebot_teach_msgs/srv/CreateShapeAction \
  "{shape: text, source: 'ROS2!', overwrite: false}"

ros2 service call /rebotarm/teach/create_shape_action \
  rebot_teach_msgs/srv/CreateShapeAction \
  "{shape: image, source: '/home/w/Pictures/logo.png', overwrite: false}"

ros2 service call /rebotarm/teach/check_shape_reachability \
  rebot_teach_msgs/srv/CheckShapeReachability '{}'
```

RViz 中当前形状使用半透明填充面、双面青色粗轮廓和中心点高亮，可直接点住本体拖动；面板上的
X/Y/Z（米）及 Roll/Pitch/Yaw（度）输入框可做精确调整，W/H（米）会改变后续规划实际使用的
形状宽高。填充面和青色轮廓均双面可见，并附红色抬笔方向箭头、三维笔模型和尺寸标签。
三色箭头平移形状，三个旋转环同时
改变绘制平面与 TCP 朝向，黄色 W/H 三维手柄直接调整实际宽高；切换形状不重置位姿。面板可隐藏标记或恢复 `shape_center` 和单位四元数定义的
默认位姿。拖动过程只更新预览，生成动作时位姿被冻结，MoveIt 规划结束前不接受新的拖动。
面板还会显示当前形状的二维缩略图。若选择“不添加图形（自由拖动录制）”，节点会清除预设
形状标记，用户直接通过“开始拖动录制/停止并保存”记录手动拖动轨迹，不经过形状生成器。

动作预览和真机回放会在 `/rebotarm/teach/tcp_trajectory` 发布夹爪 TCP 的 MarkerArray。
青色为目标笔画，橙色为总运动待执行段，绿色为已完成段；真机运行中的颜色分界随控制器
轨迹时间反馈推进。

## 命名动作序列

RViz 面板下方的“命名动作组（按列表顺序执行）”支持：添加当前动作、移除、上移、下移、
命名保存、按顺序播放 RViz 动画和依次真机回放。列表勾选框只控制本次播放范围，RViz 与真机
都只执行勾选动作且仍保持原列表顺序；保存时仍保存完整动作组。动作组页提供独立的真机倍速、
当前动作进度、总进度和 RViz 时间轴。顺序动画可暂停、继续、取消，并支持总进度拖动；开始播放
动作组时会清除单动作轨迹并隐藏其形状标记。真机动作组可随时取消，包括轨迹目标刚
发送但尚未正式执行的阶段；取消后按面板既有安全规则回 Home。真机执行与单动作一样不提供
中途暂停续播。以上 RViz 预览只发布显示消息，不驱动真机。动作可以重复加入。重命名单个动作时，已保存序列里的同名引用
会自动更新。相邻边界误差不超过 `0.01 rad` 时只做限速微调；超过该值时，真机开始运动前，
节点会使用 MoveIt/OMPL 预先规划带碰撞检查的过渡路径。过渡成功后到达下一动作起点并正向执行，不再通过倒放下一动作返回
起点。任一边界规划失败时整组不会启动，并会报告失败的动作边界。RViz 循环回放也使用同一套
过渡规划并连续显示“动作 A→过渡→动作 B”；规划失败时拒绝播放，不再直接切换姿态。真机
回放时发布的橙色执行轨迹包含实际过渡路径。静态叠加查看仍只叠加所选动作本身。

序列保存在型号动作目录下的 `sequences/*.sequence.json`。ROS 服务方式示例：

```bash
ros2 service call /rebotarm/teach/save_action_sequence \
  rebot_teach_msgs/srv/SaveActionSequence \
  "{name: 搬运一轮, action_names: [拿起, 移动, 放下], overwrite: false}"

ros2 service call /rebotarm/teach/replay_action_sequence \
  rebot_teach_msgs/srv/ReplayActionSequence \
  "{name: 搬运一轮, speed_scale: 1.0, action_names: []}"
```

## 动作保存在哪里

每个命名动作组保存为一个文件。DM 默认目录为：

```text
/home/w/project/rebotarm/packages/rebot_teach_mode/action_groups/dm/*.teach.json
```

RS 为兼容已有动作仍使用 `action_groups/*.teach.json`。

仓库初始状态只有该目录的说明文件；没有录制过时，动作组列表为空。动作组是一条带时间戳的
密集六轴关节轨迹，作用类似机械臂航点序列，但不是经过 MoveIt 重新规划的笛卡尔航点。
刚录制完成、选择动作组、重新加载最近动作或开始复现时，节点都会把关节轨迹发布到
`/display_planned_path`。RViz2 的 `MotionPlanning -> Planned Path` 会循环播放动画；
其中“选择动作组”只做预览，不会驱动真机。

组合入口的 RViz2 右侧面板还提供动作下拉框、`0.1×–2.0×` 真机回放倍速、暂停/继续和
0%–100% 预览进度滑动条。倍速同时用于单动作和命名动作序列，但不会越过关节安全速度
上限。示教
动画机械臂使用青蓝色，区别于橙色规划目标和灰色真机反馈。RViz 使用录制的
动作顺序和运动时间，但会把只有编码器微小波动的超长静止段压缩到 0.2 秒。示教节点按
50 Hz 向 `/display_robot_state` 主动发布插值姿态，画面和进度条使用同一个预览时钟，不再
依赖 MotionPlanning 内部播放器；拖动松手后从对应时刻继续循环预览，不发送真机控制命令。
静止段压缩和插值只作用于 RViz，动作文件及真机回放仍使用原始时间戳。

面板还显示两条真机进度：当前动作进度和整个命名动作序列的总进度。它们读取控制器
`FollowJointTrajectory` 反馈；序列执行时同时显示当前动作名称与第几个/共几个动作。

最近一次录制还会保存恢复副本（DM）：

```text
/home/w/project/rebotarm/real_robot_ws/log/teach/latest_dm.motion.json
```

RS 的恢复副本仍为 `latest.motion.json`。
节点启动时只加载这份恢复副本并在 RViz 放置静态姿态，不会自动播放动画或推进预览进度条；
点击面板“播放 RViz 动画”后才会开始循环预览。

Piper-H 的动作/序列 JSON 与这里的 reBotArm `*.teach.json` 和 `sequences/*.sequence.json`
格式不同，不能互相导入。双臂演示只共用动作名称，不共用关节坐标。

## 启动失败时先看这里

| 提示 | 原因或处理方式 |
|---|---|
| `allow_hardware=false` | 当前是默认锁定模式；真机操作需用 `--hardware` 或 `--xbox` |
| `joint feedback is missing or stale` | 真机驱动未启动，或 `/rebotarm/joint_states` 已中断 |
| `arm status is missing or stale` | `/rebotarm/arm_status` 心跳未到达；确认使用最新构建并重启整套组合 launch |
| `arm is not enabled` | 驱动已启动但机械臂未处于使能状态 |
| `Xbox lock state is unknown or ARMED` | Xbox 节点未报告状态，或 Servo 仍是 ARMED；按 A 切回 LOCKED |
| `current pose is not near...` | 当前姿态不在录制起点/终点附近，节点为安全起见拒绝回放 |
| `action group ... already exists` | 名称重复；换名称，或明确设置 `overwrite: true` |

## 安全限制

- 默认启动永远是锁定模式；
- 录制和回放都要求新鲜的关节反馈与驱动状态；
- 回放默认按录制时间 `1.0x`，最大关节速度仍限制为 `0.30 rad/s`；若录制动作超过该上限，
  只会安全放慢，不会加速；
- 反馈中断、驱动报错、Xbox 重新 ARMED 或连续跟踪误差都会停止/取消；
- 录制轨迹不经过 MoveIt 碰撞规划，桌面、工装或障碍物变化后必须重新验证；
- Start 和 ROS `cancel` 都只是软件取消，不能替代实体急停；
- 未经仿真和低速真机验证的动作文件不能直接用于正式演示。

## 完整服务列表

| 服务 | 作用 |
|---|---|
| `/<arm>/teach/gravity_mode/start` | 独立开启重力补偿，不录制；与录制状态互锁 |
| `/<arm>/teach/gravity_mode/stop` | 独立退出重力补偿；录制过程中拒绝 |
| `/rebotarm/teach/start_recording` | 自动命名并开始录制 |
| `/rebotarm/teach/start_action_group` | 指定名称并开始录制 |
| `/rebotarm/teach/stop_recording` | 停止、校验并保存 |
| `/rebotarm/teach/list_action_groups` | 查询全部动作组 |
| `/rebotarm/teach/rename_action_group` | 重命名动作，并同步更新序列引用 |
| `/rebotarm/teach/copy_action_group` | 复制动作包，并保留轨迹、说明和可编辑形状参数 |
| `/rebotarm/teach/delete_action_group` | 删除未被动作序列引用的动作包 |
| `/rebotarm/teach/clear_action_selection` | 进入增加动作模式并清除旧动作及 RViz 轨迹显示 |
| `/rebotarm/teach/select_action_group` | 选择动作组，不驱动硬件 |
| `/rebotarm/teach/preview_action_group` | 播放完整预览或显示滑动条指定姿态，不驱动硬件 |
| `/rebotarm/teach/replay_action_group` | 按名称复现 |
| `/rebotarm/teach/list_action_sequences` | 查询命名动作序列及顺序 |
| `/rebotarm/teach/save_action_sequence` | 命名保存一个有序动作序列 |
| `/rebotarm/teach/replay_action_sequence` | 按顺序真机执行命名动作序列 |
| `/rebotarm/teach/preview_action_sequence` | 按顺序预览整个命名动作序列，不动真机 |
| `/rebotarm/teach/configure_shape_marker` | 切换形状，以及显示、隐藏或复位六自由度形状标记 |
| `/rebotarm/teach/check_shape_reachability` | 对当前带笔尖轨迹逐点检查 IK、碰撞与关节余量；不动真机 |
| `/rebotarm/teach/create_shape_action` | 将内置形状、字符或图片简笔画规划并保存为示教动作；不立即驱动真机 |
| `/rebotarm/teach/save_action_group` | 将当前轨迹另存为动作组 |
| `/rebotarm/teach/replay` | 复现当前选中的动作组 |
| `/rebotarm/teach/cancel` | 取消录制或回放 |
| `/rebotarm/teach/pause_preview` | 暂停/继续 RViz 示教动画，不动真机 |
| `/rebotarm/teach/reload` | 重新加载最近轨迹 |
| `/rebotarm/teach/reset` | 在安全反馈正常后退出 `FAULT` |
| `/rebotarm/teach/status` | JSON 状态 Topic |

RViz 的 `Loop Animation` 保持开启以便反复检查动作。播放动画后可点击“暂停/继续”；点击
面板“取消”会发布一帧
当前关节姿态来替换循环轨迹并立即停止动画；这只更新 `/display_planned_path`，不会向真机
发送运动命令。拖动预览进度条会立即从所选时刻继续循环；面板按轨迹真实时长同步推进。
RViz 同时启用 `Interrupt Display`，因此新定位的轨迹会立即替换此前的循环动画。

如果示教节点或 RViz 单独重启，而驱动仍处于 `GRAVITY_COMP`，下一次“开始拖动录制”会
接管现有的重力补偿并开始采样，无需先重启机械臂；真机回放仍严格要求驱动处于
`IDLE`。

DM 的重力补偿模式切换会经过串口反馈、六轴模式配置和重新使能。等待驱动服务时示教节点
会释放自身状态锁，让关节反馈、状态消息和服务响应继续被执行器处理，避免驱动已完成却因
执行线程饥饿而误报 `service timed out`。
模式切换期间排队的关节反馈会在取得示教状态锁后再记录单调时间，避免旧时间戳晚到造成
`sample time moved backwards`。
重力补偿已经安全停止后，如果录制太短、采样点不足或文件保存失败，节点会回到
`IDLE/READY` 允许重新录制，而不会停留在 `FAULT`；真正的硬件停止失败仍进入
`FAULT`，并保留首个错误供诊断。

DM 真机回放默认按原始示教时间 `1.0x` 生成轨迹，同时仍强制执行 `0.30 rad/s` 单关节
速度上限、路径误差阈值和驱动自适应减速。发送 FollowJointTrajectory 目标时也会释放示教
状态锁，避免 Action 响应被反馈回调阻塞。
