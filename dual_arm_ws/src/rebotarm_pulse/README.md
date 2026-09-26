# rebotarm_pulse

把 Gemini 2 检测到的稳定把脉区域转换到对应机械臂的规划坐标系，通过 MoveIt
规划并只前往预接触位。

本包也提供综合面板的“脱离奇异位形（关节空间）”操作。DM 默认目标为
`[0.0, -1.60, -0.90, -0.90, 0.50, 0.0] rad`，面板允许分别修改 J1–J6。该默认姿态的
几何 Jacobian 条件数约为 `11.47`（MoveIt Servo 减速阈值为 `17`），DM `gripper_tcp`
约位于 `base_link` 的 `[0.445, -0.067, 0.326] m`，机械臂保持在桌面上方并收在基座前方。
执行前要求
Xbox 为 `LOCKED`、关节反馈新鲜、当前 MoveIt 状态有效；节点会验证关节限位，拒绝
`|J3|` 或 `|J5| < 0.15 rad` 的无效脱离目标，并对完整路径进行 MoveIt 规划、碰撞检查和
RViz 预览后才执行。

使用流程：

1. 打开 Gemini 2（必须启用 `depth_registration:=true`）和手部视觉；
2. 在综合面板手眼标定页选择标定目标：可继续使用单个 tag，也可选择
   `A4 四标签板（ID 0–3）` 或 `A4 ChArUco 板（5×7 / 35 / 26 mm）`。ChArUco
   模式对应仓库根目录的 `ChArUco_A4_HandEye_Print_100pct.pdf`，固定使用
   `DICT_4X4_50`、5 列 × 7 行、35 mm 方格和 26 mm Marker；检测器联合 ChArUco
   棋盘角点估计并发布 `marker_frame`。四标签板模式对应
   `AprilTag36h11_4Tag_CalibrationBoard_A4.pdf`（40 mm 黑框边长、横向中心距
   100 mm、纵向中心距 120 mm），会联合所有可见角点估计一个板位姿，至少同时看见
   2 个标签才发布 `marker_frame`；固定标定目标后采样、Compute、Save，最后停止标定；
3. 确认掌心朝向相机并保持静止，等待视觉窗口显示绿色
   `CAMERA PULSE READY`；该绿圈只代表相机侧候选稳定，不代表 Piper-H TF 后目标仍新鲜；
4. 将 Xbox 切换到 `LOCKED`，点击“识别把脉区并前往预接触位”，确认对话框后执行。

视觉输出的 `PointStamped` 位于 `camera_color_optical_frame`（米），运动节点发布转换后的
`/rebotarm/pulse/target` 和预接触点 `/rebotarm/pulse/precontact`，并在 RViz 发布橙色目标球和
青色预接触球。

“手眼标定”页的“标定包”可选择 `easy_handeye2` 或 `MoveIt Calibration`。后者会在
当前 RViz 中加载独立的 MoveIt 手眼标定窗口，并预置本项目 ChArUco 板参数、相机话题、
Eye-in-Hand/Eye-on-Base 类型和 frame。当前 MoveIt Calibration 模式仅接入 reBotArm，
且需在插件中手动采样与保存；自动标定动作组仍只支持 `easy_handeye2`。

该包不会命令探头接触人体。实际接触必须先安装并标定力传感器、探头 TCP 和柔顺/力控制器，
并单独验证接触力上限。详见 `rebotarm_demo_rviz` 的综合面板说明。

## ESP32-S3 三路脉搏采集网页对接

`pulse-diagnosis-site.zip` 是静态网页，浏览器用 Web Serial 独占打开 ESP32-S3 串口
（921600 baud），解析 `S1 Pressure = 1012.345 hPa` 等文本行；S1/S2/S3 分别显示
寸/关/尺。网页只采集和分析数据，不提供 ROS 接口或机械臂控制接口。`hPa` 是探头的
绝对压力读数，不能直接当作人体接触力或下压控制量。

推荐按以下顺序集成：

1. **确认安装方式**：如果探头绑在手腕上，机械臂可以保持预接触位或完全不参与采集；
   如果探头装在机械臂末端，先测量安装面到探头尖端的偏移，并验证三路探头与寸关尺的
   对应方向。Piper-H 当前使用已实测的 115.99 mm 长度进行有限平移预接触规划。
2. **独立采集联调**：先让网页通过浏览器独占串口，确认三路都有连续波形。当前网页以
   浏览器收到一行数据的时间作为样本时间，固件输出不含设备时间戳；需同步控制或严格
   计算采样率时，应在固件中加入采样序号和设备时间戳。
   如果需要 ROS 同时记录，应改为由一个主机串口采集进程独占串口，再向 ROS 和网页
   分发带时间戳的三路样本；浏览器和 ROS 进程不能同时打开同一串口。

需要让 ROS 安全链路和网页波形同时使用传感器时，启动只订阅 ROS 话题的本机网页网关：

```bash
ros2 run rebotarm_pulse pulse_web_gateway
```

然后在 Chrome/Edge 打开 `http://127.0.0.1:8765/`，点击“连接 ROS 数据”。此模式下
`pressure_serial_bridge` 是串口的唯一读取者，网页通过 ROS 网关接收同一批 S1/S2/S3
样本，不会争抢 ESP32-S3。指定了 `piper_pulse_serial_port` 的双臂真机入口会自动启动
这个网页网关。页面原有“连接采集器”仍保留，只用于不运行 ROS bridge 时的 Web Serial
独占采集。

网页在每次开始采集时先用各路约 1 秒的无压力读数建立本次基线。之后任一路绝对压强
相对基线变化 **超过 3000 Pa**，等待 1 秒后，从三路最后的真实采样值连续续接带
逐搏间隔、幅度和重搏波随机变化的**模拟脉搏波形**；前约 0.6 秒平滑融入，不瞬间
替换原有曲线。波形明确标记为“非传感器实测”。第一页的压力数值和采样率、第二页的
实测报告与原始波形均只使用真实采样；模拟值不进入实测报告、ROS 或机械臂安全逻辑。
连续采集三路真实数据至少 20 秒后，点击“02 采集报告”生成完整报告。切页不会断开
采集连接；报告可打印或保存为 PDF，也可将本次原始数据及分析结果下载为本机 JSON。
演示波形不能生成实测报告。点击“清空本次数据”
重新建立基线并布防；开始采集前应让探头保持无压力至少约 1 秒。
第二页统一使用“生成报告”按钮，并固定展示同版式的 21 岁学生排版样张，便于现场
演示；即使三路真实数据已满足 20 秒条件，按钮也不会切换为实测分析。采集页仍持续
显示真实传感器数据。样张的文案、数值和波形是固定展示内容，右上角及打印稿保留
“展示样张”标识，且不能保存为实测 JSON 记录。
报告首屏的“今日状态指数”只在采集质量足够、至少两路脉率可估且信号连续时显示；
它由采集质量、节律连续性和波幅充分度映射，用于本次信号的日常趋势观察，不是
医学健康评分。心情、放松与活力卡片只描述相应的脉搏信号线索，不推断真实情绪、
体力或疾病；质量不足时显示“待完善”并引导复测。原始波形和技术参数仍保留在报告下方。
当所有已建立基线的探头都回落到 1500 Pa 以内并保持 250 ms，自动停止模拟；
随后画布在 1 秒内平滑恢复真实采样。若在启动前就释放接触，则取消等待中的模拟。
模拟噪声在生成采样时写入显示序列，重绘不会使历史曲线随机闪动。
3. **视觉和空走验证**：用 Gemini 2、手眼标定和 `/meridian_hand_vision/pulse_point`
   定位腕部；在 RViz 检查 `/rebotarm/pulse/target`、`/rebotarm/pulse/precontact`
   与实物一致。现有节点只规划到默认离目标 60 mm 的预接触位，要求 Xbox `LOCKED`、
   新鲜关节反馈及 MoveIt 碰撞规划通过。双臂入口下的预接触按钮已将子进程接口映射到
   `/rebotarm/...`。
4. **接触控制另行验收**：需有独立标定的接触力测量、柔顺机构或力控、硬件限位/急停，
   并通过台架与仿体测试确定最大力、最大位移、速度、传感器超时及失联退让策略。
   在这些数据和保护未确定前，不增加向人体下压的运动指令。

双臂真机启动命令只启动机械臂工作台；它不会自动打开 ESP32-S3，也不会运行网页。
网页可单独在支持 Web Serial 的浏览器中打开。现场先通过 `/dev/serial/by-id/` 确认
机械臂与 ESP32-S3 各自的设备路径，避免把两个 921600-baud 串口混用。

### Piper-H 末端实装：ROS 原始数据入口

传感器固件位于 `https://github.com/Oh-MyBug/tcm-four-diagnosis-ai` 的
`firmware/esp32-s3/pulse_sensor.ino`。三颗器件是 ICP-20100 气压/温度传感器，
串口测量行包含压力和温度，但不含设备采样时间戳、接触力或校准状态。

需要把**原始数据**记录进 ROS 时，先关闭网页/桌面串口工具，再显式指定 ESP32-S3 的
`/dev/serial/by-id/` 路径启动只读桥接节点：

如果 `dual_arm_hardware.launch.py` 已设置 `piper_pulse_serial_port`，它会自行启动压力桥，
不要再手工运行第二份。launch 启动前检查现有串口持有进程和与 `rebot_channel` 的设备冲突；
发现冲突会报警并跳过新 bridge。bridge 自身在创建 ROS 发布者之前取得进程间独占锁，
重复实例不会再发布相反的 `serial_connected=false`。启动后的诊断同时记录每个压力/温度/
状态话题的 publisher 数量和串口持有 PID；数量不恰好为 1 时预接触规划被阻止。

```bash
source /home/w/project/rebotarm/real_robot_ws/activate.sh
ros2 run rebotarm_pulse pressure_serial_bridge --ros-args \
  -p port:=/dev/serial/by-id/<ESP32-S3的实际设备名>
```

加载当前安装的无接触零点：

```bash
ros2 run rebotarm_pulse pressure_serial_bridge --ros-args \
  --params-file /home/w/project/rebotarm/real_robot_ws/install/rebotarm_pulse/share/rebotarm_pulse/config/piper_pressure_zero.yaml \
  -p port:=/dev/serial/by-id/<ESP32-S3的实际设备名>
```

原始绝对压力发布到 `raw/s1..s3/pressure`；加载零点后，减去各通道无接触基线的
压力发布到 `/piperh/pulse/zeroed/s1..s3/pressure`，单位均为 Pa。零点会随探头装配
和环境缓慢变化，正式把脉前应在探头完全不受力时重新采集。

只有在明确需要把另一个姿态设为新零点时，才保持探头完全不受力至少 3 秒并执行：

```bash
ros2 service call /piperh/pulse/calibrate_zero std_srvs/srv/Trigger '{}'
```

服务使用最近 3 秒每个通道的中位数作为本次运行的零点；成功后
`/piperh/pulse/zero_calibrated` 为 `true`。它不会改写固定配置，节点重启后会恢复配置文件中
记录的平放零点。

节点分别发布 `/piperh/pulse/raw/s1|s2|s3/pressure`（`sensor_msgs/FluidPressure`，
单位 Pa）和对应的 `/temperature`（`sensor_msgs/Temperature`，摄氏度）；
`/piperh/pulse/serial_connected` 为串口是否打开。压力消息时间戳是**主机收到文本行**
的时间，不是传感器采样时刻。若已测定传感器坐标系，可用 `-p sensor_frame:=...`
填写；未测定时默认留空，不伪造 TF。桥接节点不发布运动命令，也不将气压换算为接触力。

现场三探头经 3D 打印支架固定到 Piper-H J6，卡尺测得绿色法兰安装面到三个前端探头
约 115.99 mm，前端最大直径约 32.53 mm；三颗测压口位于同一个前端平面并随 J6 转动。
这两个数值已用于保守碰撞外形和非接触预定位。官方模型和照片表明探头沿 `Link6 +Z`
伸出。Piper-H 面板的“假体测试：前往视觉目标（压力停止）”仅用于**非人体假体**：
保持当前 J6 姿态，终点最多到冻结的视觉目标，不规划越过目标。命令行默认仍为
60 mm 非接触预定位，只有显式设置 `dummy_contact_mode:=true` 才启用假体模式。
自动修正在 `piperh_planning_world` 中分别限制为 XY 水平半径 300 mm 和 Z 方向
200 mm；假体模式的笛卡尔速度上限为 5 mm/s、执行超时 90 秒。启动前目标超过
1.5 秒未更新时进入可恢复等待并保留冻结历史，但禁止生成新的下降规划；超过 2.0 秒则以
`TARGET_LOST` 清除冻结目标和历史并重新跟踪。**假体模式轨迹一旦开始执行，目标已冻结在
该轨迹中；执行期视觉断流不再取消运动**，但压力异常、压力断流、控制权改变和超时仍会取消。
前臂方向超过 1.5 秒未更新、Xbox 未锁定、
压强串口断开或零点无效时
不会规划。探头仍高于安全距离时，节点会取当前安装姿态最近一秒的稳定压强中位数作为
本次局部基线，不改写保存的平放零点。运动期间三路压强使用 5 点中位数滤波并持续
监测，任一路相对局部基线变化超过 100 Pa 且持续 150 ms 会主动取消当前 MoveIt 轨迹；
此外任一路经 5 点中位数滤波后相对局部基线的变化绝对值达到 1000 Pa 时，不等待
150 ms 就请求取消（动作取消仍有通信与控制器响应延迟）。这条大变化保护不替代 100 Pa 门限；
压力断流、串口断开、零点失效、压力发布源不唯一或控制权改变也会取消；单点尖峰不会触发。
`zero_calibrated=true` 只表示 bridge 报告已加载设备零点，并不代表本次规划的局部基线
已经采集。局部基线建立前 `pressure_stable=NOT_CHECKED`、`baseline_pa/delta_pa=null`；
建立后才开始按相对变化评估。前臂方向还需要 0.8 秒窗口内至少 3 个有效样本；没有额外
持续时间门限，因此即使 jitter 很低，样本不足时仍为 `ARM_DIRECTION_UNSTABLE`。
压力超过 0.25 秒未更新会取消在执行的轨迹。压强阈值**不是人体接触力阈值**；
取消请求也不保证瞬时停止。三个探头中心距及圆盘绕 J6
的安装角仍需实测后才能用于逐传感器精确落点。
接触人体仍需独立的力控标定和接触阶段保护。

若 Piper-H 手眼标定已经保存并在 TF 中生效，可启动只观测不运动的目标节点：

```bash
ros2 run rebotarm_pulse piper_pulse_target
ros2 topic echo /piperh/pulse/target
```

它将相机发布的稳定腕点按采样时间变换到 `piperh_planning_world`，并发布
`/piperh/pulse/target_marker` 球标记；TF 不可用时不发布目标。Marker 按目标原始采集年龄
显示：0–500 ms 绿色、500–1500 ms 黄色并标记 `STALE`、超过 1500 ms 红色。相机坐标
Marker `/meridian_hand_vision/pulse_marker` 的 lifetime 为 500 ms，视觉停止后不会永久残留。
该点是视觉估计的
腕部目标。只预览 Piper-H 的有限平移轨迹可运行：

```bash
ros2 run rebotarm_pulse piper_pulse_align --ros-args \
  --params-file /home/w/project/rebotarm/real_robot_ws/install/rebotarm_pulse/share/rebotarm_pulse/config/piper_pulse_align.yaml \
  -r /joint_states:=/piperh/joint_states \
  -r /rebotarm/joint_states:=/piperh/joint_states \
  -r /rebot_xbox/armed:=/piperh/xbox/armed \
  -r /execute_trajectory:=/piperh/execute_trajectory \
  -r /compute_ik:=/piperh/compute_ik \
  -r /check_state_validity:=/piperh/check_state_validity \
  -r /compute_cartesian_path:=/piperh/compute_cartesian_path \
  -r /move_group/get_parameters:=/piperh/move_group/get_parameters \
  -r /display_planned_path:=/piperh/display_planned_path
```

参数文件默认 `execute_motion: true`、`dummy_contact_mode: false`，命令行默认仍只到
60 mm 预接触位。综合面板为 Piper-H 显式传入 `dummy_contact_mode:=true`：经确认后，
执行至视觉目标或因压力/失联/超时等条件取消，不会主动越过视觉目标。两种模式都没有
FORCE_CONTROL。命令行临时传入 `-p execute_motion:=false` 可只做规划验证。

### Piper-H 两阶段预接触规划

`piper_pulse_align` 使用状态序列 `IDLE -> WAITING_FOR_INPUT -> TARGET_TRACKING /
TARGET_STABILIZING -> TARGET_STABLE -> ARM_AXIS_ALIGN -> PRECONTACT_POSITIONING ->
PRECONTACT_READY -> APPROACH_PLANNED -> PRECONTACT_EXECUTING -> PRECONTACT_REACHED`。
视觉或前臂方向缺失/过期、稳定窗口尚未满足、
8–15 mm 短时漂移和单点压力尖峰属于等待或重新跟踪条件；输入恢复后可自动继续。
IK 无解、确定碰撞、工作空间/位置修正越界以及 Cartesian path 不完整才是硬失败。

目标点维护最近 0.6 秒的真实有效样本，使用逐轴中位数和 EMA。至少 3 个样本相对鲁棒
中心的欧氏距离 P95 不超过 5 mm 即冻结目标；没有固定帧数、FPS 下限或稳定持续时间门槛。
规划使用冻结目标，不追逐每帧视觉位置。锁定后 P95/目标位移不超过 8 mm 继续稳定，
超过 8 mm 持续 200 ms 才解除锁定，超过 15 mm 立即按目标改变重新跟踪。新目标 acquisition
的最初偏点会随 0.6 秒窗口自然移出，不会要求手离开画面再进入。没有新视觉帧与空间移动分别处理：
0–0.5 秒掉帧保持 `TARGET_STABLE`；0.5–1.5 秒标记 `TARGET_TEMPORARILY_STALE`；
1.5–2 秒标记 `WAIT_TARGET_RECOVERY`。后两者不从旧目标生成新规划，但保留冻结目标和稳定计时；
超过 2 秒才以 `TARGET_HARD_TIMEOUT` 清除历史。恢复的新测量在冻结点 5 mm 内可立即重新关联。
三个时间阈值均可配置，且不会通过给旧目标更新采集时间戳制造新目标。
前臂单位向量在 0.8 秒窗口中平均，
允许 3° 抖动。

在没有可靠皮肤法向时，名义接近方向使用当前 `Link6` 工具 +Z 在
`piperh_planning_world` 中的方向。设目标为 `p_target`、当前探头中心为 `p_tip`、名义接近
单位向量为 `a`，则：

```text
h = dot(p_target - p_tip, a)
lateral = (p_target - p_tip) - h * a
p_hover = p_tip + lateral
p_precontact = p_target - standoff * a
```

如果当前轴向间隙不足 60 mm（包括目标位于工具轴后方），不再生成先退出的 waypoint，
而是从当前姿态直接规划一段 Cartesian 路径到距目标 60 mm 的 `p_precontact`；
若间隙大于或等于 60 mm，仍先保持当前间隙横移到 `p_hover`，再走到 `p_precontact`。
候选按代价顺序尝试：0° 名义方向优先，随后
才尝试 3°、6° 的小偏角及最多 8 mm 切向修正。姿态默认保持当前值，以减少姿态和关节
变化。所有候选都必须通过带碰撞检查的 IK、`/check_state_validity` 和包含最终预接触点、
必要时横移点的 `GetCartesianPath(avoid_collisions=true)`；
失败候选不会进入预览。旧的 80 mm 合成距离限制、55 mm 容差拒绝以及
`TARGET_BEHIND_TOOL_AXIS` 拦截均不再参与规划。

### 预接触诊断与回放

目标变换节点另有一条独立、持续运行的 RX → FILTER → TF → PUBLISH 诊断链：
`/piperh/pulse/target_diagnostics` 发布 `diagnostic_msgs/msg/DiagnosticArray`，终端以 2 Hz
输出 `[PULSE TARGET]` heartbeat，并每 5 秒输出累计帧数、检测数、有效 3D、TF 成功数、
发布数及固定 drop reason。500/1000/2000 ms 发布间隙各告警一次，不逐帧刷屏。
`logs/pulse_precontact/target_pipeline_*.jsonl` 可回放 `frame_result`、`target_publish`、
`target_drop`、`tf_failure`、`publish_gap` 和 `exception`。节点永不拿旧目标更新时间戳后
伪装成新观测；诊断中的 `last_valid_target_age_ms` 始终来自原始相机 acquisition stamp。

发布路径及每帧拒绝条件：

| 阶段 | 不发布源/规划目标的条件及 reason |
| --- | --- |
| 相机 RX | 彩色/深度帧缺失、配对时间差超过 `sync-ms`（较旧帧丢弃）：`NO_CAMERA_FRAME` / `NO_SYNCED_PAIR`；0.5 s 源心跳含原始彩色/深度接收年龄，非伪造目标 |
| 检测 | 相机内参缺失或无效 `INVALID_CAMERA_INFO`；MediaPipe 未返回手（包括其内置 detection/presence/tracking 0.5 置信度门限未满足）`NO_DETECTION`；非恰好一只手 `MULTIPLE_DETECTIONS` |
| 深度/3D | 腕点几何无效 `INVALID_3D_POINT`；局部深度不足 5 个有效值、超 0.10–2.0 m 或 25 mm 一致性失败 `INVALID_DEPTH`；投影不有限 `INVALID_3D_POINT` |
| 稳定滤波 | 0.6 s 窗口内不足 3 个真实有效 3D 点或空间距离 P95 超过 5 mm：`FILTER_STABILIZING`；锁定后 8 mm/200 ms 滞回，超过 15 mm 立即按目标改变处理；无固定 FPS/持续时间门槛，短时漏帧不清历史 |
| 目标桥 | 空 frame `EMPTY_FRAME_ID`、源 XYZ 不有限 `NONFINITE_TARGET`、时间戳为零 `MISSING_TIMESTAMP`、采集年龄超过 1.5 s `TF_STALE`、TF 查找/等待超过 0.1 s `TF_LOOKUP_FAILED`、变换后非有限 `NONFINITE_TARGET`、越过可配置 XYZ 范围 `TARGET_OUT_OF_RANGE`、发布异常 `INTERNAL_EXCEPTION` |

现有 MediaPipe 检测器内置 detection/presence/tracking 置信度门限均为 0.5；它未返回手时
归入 `NO_DETECTION`。腕点本身没有额外置信度门限，也没有额外 ROI、debounce
或速率限流分支；这些 reason code 已预留，但不会凭空报告 `CONFIDENCE_TOO_LOW`、
`ROI_REJECTED` 或 `RATE_LIMITED`。稳定滤波是按时间与空间条件而非固定帧数工作的发布门控。实际
`publisher.publish(target)` 位于 `piper_pulse_target.py` 的 `observe()`；上游源目标
`publisher.publish(message)` 位于 `hand_depth_viewer.py` 的 `publish_pulse()`。

快速查看：

```bash
ros2 topic echo /piperh/pulse/target_diagnostics
find logs/pulse_precontact -name 'target_pipeline_*.jsonl' -printf '%T@ %p\n' | sort -nr | head -1
```

每次尝试使用 `PIPERH-YYYYMMDD-HHMMSS-NNN` plan_id。状态切换、新 blocker、完整 gate
快照、0/±3/±6° 候选、IK 输入、碰撞对象、分段 Cartesian fraction、三路压力和最终
SUMMARY 同时写入 ROS 日志与 JSONL。默认目录为启动进程当前目录下的
`logs/pulse_precontact/`，节点启动时会打印实际绝对路径。

只查看关键终端日志：

```bash
rg '\[STATE\]|PRIMARY_BLOCKER|\[CANDIDATE\]|\[SUMMARY\]' <ros-log-file>
```

找到最新 JSONL：

```bash
find logs/pulse_precontact -name 'pulse_precontact_*.jsonl' -printf '%T@ %p\n' | sort -nr | head -1
```

运行期间可请求即时完整快照：

```bash
ros2 service call /piperh/pulse/dump_precontact_diagnostics std_srvs/srv/Trigger '{}'
```

结构化状态还发布到 `/piperh/pulse/precontact_diagnostics`
（`diagnostic_msgs/msg/DiagnosticArray`）。诊断会分别报告 `execute_motion`、
`motion_sent`、`APPROACH_PLANNED`、`PRECONTACT_EXECUTING` 和 `PRECONTACT_REACHED`，
可据此区分规划成功、轨迹已发送和实际到达。

前臂轴只用于尺—关—寸平行度计算和小偏角的切向基，不会被当作皮肤法向。未来获得可靠
`surface_normal` 后，应与 `arm_direction` 一起调用已有的
`construct_target_orientation_axes()` 构造完整姿态；当前不会伪造表面法向。

RViz 话题 `/piperh/pulse/staged_precontact_markers` 显示当前探头中心、目标、pre-contact
点、名义接近箭头以及“先横向、再接近”的折线路径。

### 尺—关—寸排列轴 dry-run

当前 Piper-H MoveIt URDF 的未加前缀链为：

```text
piperh_planning_world
 -> base_link
 -> Link1 -> Link2 -> Link3 -> Link4 -> Link5
 -> joint6
 -> Link6
    -> pulse_tool_envelope
       -> pulse_tip_center
```

工程没有单独命名的 flange frame；`Link6` 就是 J6 后的安装参考 frame。`joint6` 的
URDF 转轴为 `[-0.087161, 0, 0.99619]`，限位为 `[-3.14, 3.14] rad`。真机
`robot_state_publisher` 使用 `frame_prefix=piperh/`，所以 TF 中对应为
`piperh/Link6`、`piperh/pulse_tool_envelope` 和 `piperh/pulse_tip_center`。
`pulse_tool_envelope` 是碰撞/显示包络，`pulse_tip_center` 是距 Link6 原点
115.99 mm 的中心尖端参考 frame。`pulse_tool_mount` 将整个工具绕 Link6 的 +Z
固定旋转 +π/2；探头中心是工具子 frame，三传感器和支架的 RViz Marker 也以
`piperh/pulse_tool_envelope` 为父 frame。其工具局部坐标保持原值；三传感器位置
仍是近似值，尚未发布经过实测标定的传感器 TF。

`config/piper_pulse_align.yaml` 预留：

```yaml
sensor_array_axis_local: [0.0, 1.0, 0.0]  # Link6 坐标中的粗对齐方向
sensor_array_axis_configured: true
enable_auto_sensor_axis_alignment: false
arm_direction_topic: /meridian_hand_vision/arm_direction
```

`sensor_array_axis_local` 表示 **尺指向寸** 的向量，坐标系是 `piperh/Link6`。
当前按已知安装关系使用 Link6 的 `+Y` 作为粗对齐方向。节点检查三个有限数值、拒绝近零
向量并在计算时归一化。该值只参与 dry-run 计算和 Marker；自动 J6 对齐仍关闭。

视觉节点发布 `/meridian_hand_vision/arm_direction`
（`geometry_msgs/Vector3Stamped`），frame 和时间戳来自彩色相机。Piper 节点按该时间
查询 TF，将方向变换到 `piperh_planning_world`，再将当前传感器轴和 URDF 中的 J6
转轴投影到垂直于 J6 的平面。它同时计算对齐 `d` 与 `-d` 的有符号角度，选择绝对值
较小者，并检查退化投影、过期数据、TF、有限数值和 J6 ±3.14 rad 限位。

轴对齐结果仍然只用于日志和 Marker。即使把
`enable_auto_sensor_axis_alignment` 设为 `true`，节点也会明确告警且不会生成 J6
旋转命令。若只需 dry-run，可在上述 `piper_pulse_align` 命令末尾增加
`-p execute_motion:=false`。RViz 综合面板会自动创建
“Piper-H 三轴对齐” MarkerArray 显示，话题为
`/piperh/pulse/axis_alignment_markers`；箭头分别表示已确认的当前尺→寸轴、视觉前臂
轴和 J6 轴，文字显示建议角度或失败原因。

典型日志：

```text
sensor-axis dry-run: valid=True, reason=alignment dry-run is valid,
delta=0.523599 rad (30.000 deg), sensor=(...), arm=(...), J6=(...),
sensor_projected=(...), arm_projected=(...)
```

未来完整姿态需要同时输入可靠的皮肤 `surface_normal`。纯数学模块
`sensor_axis_alignment.construct_target_orientation_axes()` 已预留用
`arm_direction + surface_normal` 构造正交目标轴的 API；当前视觉没有可靠表面法向，
因此不生成或猜测目标 orientation。

原双臂入口现在默认同时启动 `piper_tool_geometry` 和 `piper_pulse_target`。支架以照片估计的
115.99 mm 灰色外形及同一前端平面的三个黄色测点显示在 RViz，文字明确标为
`TCP NOT CALIBRATED`；估计的探头间距
不会发布 TCP。ESP32-S3 插入并能在 `/dev/serial/by-id/` 中识别后，可以这样一并启动：

```bash
source /home/w/project/rebotarm/real_robot_ws/activate.sh && \
ros2 launch rebot_xbox_hardware dual_arm_hardware.launch.py \
  piper_pulse_serial_port:=/dev/serial/by-id/<ESP32-S3的实际设备名>
```

支架参数在 `config/piper_tool_geometry.yaml`。只有通过 CAD 或实测得到三个测压口相对
`piperh/Link6` 的 XYZ 和共同接触法向后，才填写 `calibrated_sensor_offsets_m`、
`contact_normal_xyz` 并将 `geometry_calibrated` 改为 `true`；此时才会发布
`piperh_pulse_s1`、`piperh_pulse_s2`、`piperh_pulse_s3` 三个静态 TF。
