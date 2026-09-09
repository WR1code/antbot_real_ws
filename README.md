# AntBot Orin 真机工作区

这是以 `antbot_real_ws` 为唯一运行源的 AntBot 真机工作区。它可以整体复制到
NVIDIA Jetson Orin Nano，模型、RViz2 配置、地图、操作界面和 H743 底层均从本仓库
加载，不依赖 `/home/w/project/antbot`，也不会启动 Isaac Sim、Gazebo、
`fake_hardware` 或原厂底盘 `ros2_control`。

## 当前可用范围

- Orin 通过 3.3 V USB-TTL 与 H743 USART1 通信；
- ROS 2 `/cmd_vel` 的 `linear.x/linear.y` 发送给 H743，桥接层丢弃并告警任何
  非零 `angular.z`；
- H743 状态发布到 `/rs00/motor_status`；
- H743 桥发布四个转向关节和四个车轮关节的 `/joint_states`；未连接底盘时发布
  安全零位，使 RViz 中整个轮组 TF 仍保持连接；收到反馈后切换为实车角度；
- 支持只读状态查询、系统健康、UID 和 MINI 反馈查询；
- 可选启动 Generic Xbox，`angular.z` 强制为 0；
- 保留 AntBot 机器人模型、传感器、双雷达和导航源码，供硬件确认后继续接入。

当前尚不能宣称完整 Nav2 实机闭环：H743 还不支持 `angular.z`，也没有向 ROS 发布
轮式里程计 `/odom` 和 `odom -> base_link`。在解决这两项前只允许底盘纯平移、状态
监控和传感器联调，不启动自动导航。

## 目录

```text
antbot_real_ws/
├── src/
│   ├── antbot_h743_bridge/    H743 ROS 2 串口桥和CLI
│   ├── antbot_real_bringup/   不含仿真的真机启动入口
│   ├── antbot_description/    GK4XC STEP 实车模型及 ROS 描述
│   ├── antbot_*               真机相关ROS源码
│   ├── vanjee_lidar_*         2D雷达源码
│   └── robotcar_navigation/   导航/RViz插件实际副本，不是软链接
├── firmware/
│   ├── rs00_fk743_test/       H743完整源码、文档和主机测试
│   └── images/                已验证的Debug/Release HEX
├── repos/                     可选传感器外部依赖清单
├── artifacts/maps/home_01/    默认二维地图、航点及离线点云
├── maps/                      其他真机地图预留目录（不含仿真地图）
├── scripts/                   构建、迁移、预检和启动脚本
└── docs/                      Orin迁移与硬件确认文档
```

复制来源、基线提交和工作树状态见
[源码快照说明](docs/SOURCE_SNAPSHOT.md)。

特意没有复制 `antbot_gazebo`、旧 `antbot_hw_interface`、旧
`antbot_swerve_controller` 和旧 `antbot_bringup`，防止真机上同时启动两个底盘
控制实现。

## 本机验证

```bash
cd /home/w/project/antbot_real_ws
./scripts/install_dependencies.sh core
./scripts/build.sh
./scripts/test.sh
source ./scripts/setup_env.sh
./scripts/check_hardware.sh
```

首次连接只做查询：

```bash
export RS00_UART_PORT=/dev/serial/by-id/你的H743_USB-TTL
./scripts/query_chassis.sh
```

在连接 H743 之前，可先插入 Xbox/兼容手柄并单独验证遥控链路：

```bash
./scripts/start_xbox_dry_run.sh
```

该入口只启动手柄驱动和 `/joy -> /cmd_vel` 映射，不启动 H743 串口桥。默认将
线速度限制为 `0.10 m/s`，并将 `angular.z` 强制为 0。脚本会忽略触摸屏等错误生成
的 `/dev/input/js*` 设备；如果连接了多个手柄，请设置 `ANTBOT_JOY_DEVICE`。

启动本仓库自带的完整航点控制界面、GK4XC 实车模型和 H743 底层：

```bash
./scripts/start_antbot_operator.sh
```

它默认打开 `home_01`，保留原 Step2 的二维地图、航点、禁行区、限速区、历史卡点、
离线三维点云和 RGB-D 预览界面，但不会启动 Isaac Sim、Gazebo、旧
`antbot_hw_interface` 或旧 `antbot_swerve_controller`。当前 H743 尚无里程计且不
支持 `angular.z`，因此 Nav2 自动导航明确保持关闭，RViz 中的航点可查看和编辑，
但不能据此下发自动导航。左侧操作区采用分页，
航点面板分为“地图与航点 / 区域规则 / 巡航任务”，各页仍可独立滚动。

车辆控制中心现在分为“控制与安全 / 底盘信息 / 建图与遥控 / 相机”四页：

- “底盘信息”集中显示 MINI 24 V 母线电压估算电量，以及 H743 当前可查询的
  转向、行走驱动、故障、反馈年龄、CAN 计数等状态；百分比是可配置的
  `18–30 V` 线性估算，不是库仑计读数。
- “建图与遥控”可常驻切换 Xbox 与 RViz 键盘控制。键盘控制框支持
  `Q/W/E/A/D/Z/X/C`，也可按住屏幕方向按钮，松键或失焦即发送零速。
  界面只显示当前控制方式：选键盘时显示键盘控制器，选 Xbox 时显示
  实测按键和摇杆说明。
- 同一页可开始、停止和保存 SLAM Toolbox 建图；需要 `/scan_0` 已有发布者，
  实时地图独立显示在 `/antbot/mapping/map`，不会覆盖正式 `/map`。默认保存到
  `artifacts/maps/<地图名>/mapping_runs/<时间>/map.yaml`。

即使 H743 和手柄都没有连接，上位机也会正常打开并在“真机安全门禁”面板显示
离线。连接 H743 后，必须在 RViz 中点击“确认安全并启用”并通过二次确认；H743
报告 ready 后，Xbox 模式仍需按手柄 `A` 键，运动指令才可能通过。
“停止并锁定”、连接中断、硬件故障或关闭 RViz 都会撤销运动权限并发送零速帧。
键盘模式不需要 Xbox `A` 键，但仍必须先通过同一 H743 安全门禁。
可用 `ANTBOT_TELEOP_MODE=keyboard` 设置默认控制方式，用
`ANTBOT_MAPPING_OUTPUT_PREFIX=/绝对路径/map` 设置建图保存位置。
仅检查环境、不访问硬件时可运行
`./scripts/start_antbot_operator.sh --check-only`。

小车外观来自
`src/antbot_description/cad/GK4XC-001-003 4轮转向小车总装.STEP`。原始 STEP
由 Git LFS 保存；RViz2 实际加载其约 8.2 MB 的轻量化 STL，因此运行和部署不需要
实时解析 295 MB CAD。首次克隆后如需取得原始 STEP，执行 `git lfs pull`；只运行
机器人则普通克隆即可，因为 STL 随 Git 正常下载。

只启动机器人模型和H743串口桥，不启动手柄：

```bash
./scripts/start_base.sh
```

标定、急停和人员安全全部确认后才可以运行：

```bash
./scripts/start_xbox.sh
```

## 迁移到Orin

开发机执行：

```bash
./scripts/deploy_to_orin.sh orin@192.168.1.50 /home/orin/antbot_real_ws
```

详细过程见 [Orin迁移说明](docs/ORIN_MIGRATION.md)，接线、标定和放行条件见
[硬件确认清单](docs/HARDWARE_CHECKLIST.md)。需要让小车主机上的 Codex/终端助手
直接从 GitHub 安全拉取时，可复制使用
[小车主机拉取更新提示词](docs/ORIN_PULL_PROMPT.md)。

## 安全边界

- `real_base.launch.py` 默认 `start_xbox:=false`，不会自动使能电机；
- `start_xbox.sh` 要求人工输入 `ENABLE`，并等待 H743 报告 `ready=yes`；
- H743 未标定配置仍会拒绝整车运动；
- 实体急停必须直接切断驱动器使能或电机动力，不能依赖 Orin、ROS、UART 或 CAN；
- 状态面板、CLI 和 ROS 串口桥不能同时打开同一串口。
