# meridian_hand_vision

ROS 2 可视化包，用于同时显示：

- Orbbec Gemini 2 彩色图；
- MediaPipe 手部 21 个关键点；
- 对齐深度伪彩色图；
- 腕点附近的深度值；
- 根据腕点方向、深度连续性和肤色辅助估计的手臂轮廓。
- 面向演示的四种稳定手势事件：`open_palm`、`victory`、`thumbs_up`、`fist`。

同时发布 RGB 标注图像 `/meridian_hand_vision/annotated_image`，供 RViz 综合控制面板的
“手部视觉”标签页直接显示；该图像只包含原三联画面的最左侧 RGB/关键点/轮廓区域。

本包不使用 `cv_bridge`，不会修改 Orbbec 驱动、ROS 2 或 MediaPipe Conda 环境。

## 启动

推荐从 reBotArm 真机综合入口启动，DM 串口版只需这一条命令：

```bash
source /home/w/project/rebotarm/real_robot_ws/activate.sh && ros2 launch rebot_xbox_hardware xbox_hardware_servo.launch.py model:=dm channel:=/dev/ttyACM2
```

进入 RViz 的 **reBot 综合控制** 后，从面板依次打开摄像头、启动手部视觉、执行手眼标定
和把脉区预接触；面板启动视觉时默认使用无窗口模式，图像只显示在当前标签页，不会再弹出
独立 OpenCV 窗口，也不需要执行下面的独立调试命令。

以下双终端方式仅用于单独调试视觉：

终端 1：

```bash
cd /home/w/project/rebotarm/real_robot_ws
./scripts/start_gemini2_rgbd.sh
```

终端 2：

```bash
/home/w/project/rebotarm/packages/meridian_hand_vision/scripts/run_hand_depth_viewer.sh
```

默认订阅：

```text
/camera/color/image_raw
/camera/depth/image_raw
```

相机启动时必须启用 `depth_registration:=true`，否则深度与彩色像素不能直接对应。

## 常用参数

```bash
/home/w/project/rebotarm/packages/meridian_hand_vision/scripts/run_hand_depth_viewer.sh \
  --depth-tolerance 0.18 \
  --max-depth 3.0 \
  --display-scale 0.55
```

- `--depth-tolerance`：腕部深度连续范围，单位米；
- `--max-depth`：深度伪彩色显示上限，单位米；
- `--display-scale`：三联画面显示缩放；
- `--no-window`：只发布 RViz 标注画面，不打开独立窗口；
- 默认显示摄像头原始方向；需要自拍镜像时传 `--mirror`（旧参数 `--no-mirror` 仍兼容但不再需要）；
- `--num-hands 1`：只检测一只手。
- `--gesture-topic /demo/gesture`：稳定手势事件输出；默认需要连续 12 帧一致；
- `--gesture-hold-frames 12`：修改连续确认帧数。
- `--target-filter-window-sec 0.6`：腕部目标仅保留最近 0.6 秒的真实有效 3D 点；
- `--target-stable-duration-sec 0.0`：兼容旧命令的保留参数，不再参与稳定门控；
- `--target-stable-min-samples 3`：时间窗口中至少 3 个有效点，**不是连续 3 帧确认**；
- `--target-stable-radius-m 0.005`：XYZ 相对逐轴中位数的距离 P95 不超过 5 mm即可进入稳定；
- `--target-stable-hold-radius-m 0.008`：锁定后 P95/目标位移不超过 8 mm 保持稳定；
- `--target-unstable-hold-sec 0.2`：超过 8 mm 持续 200 ms 才解除稳定锁定；
- `--vision-dropout-grace-sec 0.5`：短时 RGB-D 掉帧保持已锁定稳定状态，不产生新测量或重写旧时间戳；
- `--target-soft-timeout-sec 1.5`：超过 0.5 秒暂时过期，超过 1.5 秒等待恢复，均保留冻结目标；
- `--target-hard-timeout-sec 2.0`：超过 2 秒没有有效目标才清空跟踪历史；
- `--target-hard-jump-m 0.015`：P95、鲁棒中心或当前真实点变化超过 15 mm 时立即按新目标跟踪；
- `--arm-direction-stable-frames 12`：仅用于独立的前臂方向滤波，不决定腕点目标是否稳定。

## 把脉区域输出

视觉节点在检测到恰好一只手、深度有效且腕点连续稳定后，会按“腕部近端 + 拇指侧”
输出一个近似桡侧把脉区域。它不是医学定位结果，要求被测者掌心朝向相机、手腕伸直并
保持静止。输出包括：

- `/meridian_hand_vision/pulse_point`（`geometry_msgs/PointStamped`，相机光学坐标系，米）；
- `/meridian_hand_vision/pulse_marker`（`visualization_msgs/Marker`，相机坐标区域球标记，
  lifetime 为 500 ms，输入停止后自动消失）；
- `/meridian_hand_vision/pulse_status`（每个同步 RGB-D 处理周期一条 JSON，包含 `frame_seq`、
  帧间隔、检测/3D 有效性、原始像素/深度/相机 XYZ、是否发布源目标和固定 reason code，
  以及 `vision_sample_rate_hz`、`vision_valid_samples`、`spatial_jitter_mm`、
  `spatial_drift_mm`、`stable_duration_ms`、`stable_required_ms`、
  `stable_reset_count`、`last_stable_reset_reason`）；
- `/meridian_hand_vision/arm_direction`（`geometry_msgs/Vector3Stamped`，相机光学
  坐标系中的近似前臂无向轴，使用图像时间戳）。

深度必须与彩色图注册，且必须有 `/camera/color/camera_info`。`pulse_point` 只负责观测，
不会驱动机械臂；综合 RViz 面板的“识别把脉区并前往预接触位”会在手眼 TF、MoveIt 碰撞
检查和 Xbox LOCKED 都满足后，将探头停在皮肤外的安全距离处。

腕部稳定门控不再使用固定帧数、FPS 下限或稳定持续时间：0.6 秒滚动窗口内至少 3 个
真实有效 3D 点，空间距离 P95 不超过 5 mm 即可进入稳定。锁定后使用 8 mm/200 ms
退出滞回，超过 15 mm 立即按目标改变处理。单次或少量 RGB-D 漏帧不清空历史，也不
重新发布旧目标。新目标 acquisition 的早期偏点只留在滚动窗口内，过期后自动淘汰，
不要求手离开画面再进入。已稳定目标在掉帧后恢复且距冻结点不超过 5 mm 时直接恢复。
超过 2 秒无有效点或高置信度左右手标签变化才因数据/身份原因重建历史。MediaPipe 的左右手标签只是可用的身份代理，
不是跨帧唯一 hand ID。状态 JSON 同时记录目标年龄、grace、连续漏检数、稳定锁定状态、
重新关联距离/结果和重置原因。

`arm_direction` 由 MediaPipe 掌心中心到腕点的两处注册深度估计。MediaPipe 没有前臂
关键点，因此它是供后续姿态 dry-run 使用的近似方向，`d` 与 `-d` 表示同一条轴线；
它不会直接产生机械臂命令。

手势输出只用于请求演示动作。它不会使能机械臂，`fist` 也不能替代实体急停。
只有画面中恰好一只手时才参与稳定判定；多手、遮挡或无法确定时输出逻辑会回到
`unknown`，不会触发动作。

轮廓是实时估计结果，不是医学或毫米级边界。衣袖、遮挡、反光和无效深度都会影响结果。
