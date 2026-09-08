# AntBot Orin 真机工作区

这是从现有 AntBot、RobotCar 和 STM32H743 工程整理出的独立真机工作区。它可以
整体复制到 NVIDIA Jetson Orin Nano，不依赖原来的绝对路径，也不会启动 Isaac Sim、
Gazebo、`fake_hardware` 或原厂底盘 `ros2_control`。

## 当前可用范围

- Orin 通过 3.3 V USB-TTL 与 H743 USART1 通信；
- ROS 2 `/cmd_vel` 的 `linear.x/linear.y` 发送给 H743，桥接层丢弃并告警任何
  非零 `angular.z`；
- H743 状态发布到 `/rs00/motor_status`；
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
│   ├── antbot_*               复制的真机相关ROS源码
│   ├── vanjee_lidar_*         2D雷达源码
│   └── robotcar_navigation/   导航/RViz插件实际副本，不是软链接
├── firmware/
│   ├── rs00_fk743_test/       H743完整源码、文档和主机测试
│   └── images/                已验证的Debug/Release HEX
├── repos/                     可选传感器外部依赖清单
├── maps/                      真机地图预留目录（不含仿真地图）
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
[硬件确认清单](docs/HARDWARE_CHECKLIST.md)。

## 安全边界

- `real_base.launch.py` 默认 `start_xbox:=false`，不会自动使能电机；
- `start_xbox.sh` 要求人工输入 `ENABLE`，并等待 H743 报告 `ready=yes`；
- H743 未标定配置仍会拒绝整车运动；
- 实体急停必须直接切断驱动器使能或电机动力，不能依赖 Orin、ROS、UART 或 CAN；
- 状态面板、CLI 和 ROS 串口桥不能同时打开同一串口。
