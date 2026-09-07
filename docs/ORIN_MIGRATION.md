# 迁移到 NVIDIA Jetson Orin Nano

## 1. 系统选择

先在 Orin 上执行：

```bash
uname -m
cat /etc/os-release
cat /etc/nv_tegra_release
apt list --installed 2>/dev/null | grep nvidia-jetpack
```

应为 `aarch64`。截至本工作区整理日（2026-09-05），NVIDIA 的 Orin Nano 官方指南
以 JetPack 7.2.1 / L4T r39.2.1 为当前版本；已经安装好的机器也可能仍在 JetPack 6.x。
不要只根据“Orin Nano”猜操作系统，以上面命令的实机结果为准：Ubuntu 24.04选择
ROS 2 Jazzy，Ubuntu 22.04选择ROS 2 Humble。

本工作区核心链路当前在 Ubuntu 24.04/Jazzy 上完成构建和测试。H743桥只用标准
`rclpy`、`geometry_msgs`、`std_msgs` 和 `pyserial`，设计上兼容 Humble，但尚未在
Humble/aarch64 实机验收。若使用 JetPack 6/Ubuntu 22.04，先只构建 `core`；相机、
雷达和GPU组件再按厂商针对该JetPack的分支逐项验证。

版本依据：[NVIDIA Orin Nano 快速开始](https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/quick_start.html)、
[ROS 2 Jazzy Ubuntu 支持](https://docs.ros.org/en/jazzy/Installation/Alternatives/Ubuntu-Install-Binary.html)。

## 2. Orin硬件基础

- 使用稳定的Orin供电和主动散热；
- 建议使用NVMe保存地图、日志和点云；
- H743通过独立3.3 V USB-TTL连接，使用稳定的 `/dev/serial/by-id`；
- 雷达尽量走独立千兆以太网交换机，不与大流量上位机链路混用；
- USB相机和多个串口建议使用有源USB 3.x Hub，不能依靠Orin单口给全部设备供电。

## 3. 从开发机复制

```bash
cd /home/w/project/antbot_real_ws
./scripts/deploy_to_orin.sh orin@192.168.1.50 /home/orin/antbot_real_ws
```

脚本不复制 `build/install/log`，也不会删除远端已有文件。如果不用SSH，也可以把整个
目录复制到移动硬盘，再放到Orin任意位置；工程脚本全部按自身位置计算工作区根目录。

## 4. 安装与构建

Orin终端：

```bash
cd /home/orin/antbot_real_ws
sudo usermod -aG dialout "$USER"
```

重新登录后：

```bash
./scripts/install_dependencies.sh core
./scripts/build.sh
./scripts/test.sh
source ./scripts/setup_env.sh
```

默认只构建机器人模型、遥控、H743桥和真机bringup。需要相机、IMU、雷达和导航源码
时再执行：

```bash
./scripts/import_full_dependencies.sh
./scripts/install_dependencies.sh full
ANTBOT_BUILD_FULL=1 ./scripts/build.sh
```

外部依赖来自 [依赖清单](../repos/antbot_dependencies.repos)。不同JetPack/ROS版本的
相机和雷达SDK可能需要厂商对应分支，不能盲目沿用桌面机二进制。

## 5. 固定设备名称

连接USB-TTL后：

```bash
ls -l /dev/serial/by-id/
export RS00_UART_PORT=/dev/serial/by-id/实际设备名
```

建议把变量写入Orin用户自己的环境文件。程序不会自动猜测任意 `ttyACM`/`ttyUSB`，
避免向错误设备发送底盘命令。

## 6. 首次只读联调

保持轮子架空，实体急停可用，不启动任何旧底盘节点：

```bash
source ./scripts/setup_env.sh
./scripts/check_hardware.sh
./scripts/query_chassis.sh
```

期望：

- 串口可读写；
- 固件身份和配置指纹可读取；
- 四台RS00 UID各不相同且轮位正确；
- 四台MINI故障码为0，电压/温度/电流合理，反馈年龄小于400 ms；
- 未完成标定时明确显示整车运动锁定。

## 7. 启动真实底盘

只启动模型与H743桥：

```bash
./scripts/start_base.sh
```

另一个终端可查看：

```bash
ros2 topic echo /rs00/motor_status
ros2 topic hz /rs00/motor_status
```

完成硬件清单后，停止桥接进程，再运行：

```bash
./scripts/start_xbox.sh
```

该脚本会查询状态、显式请求转向使能、等待 `ready=yes`，之后才启动手柄节点。速度
默认限制为 `0.10 m/s`，旋转固定为0。

## 8. 暂时不要启动的功能

- 原 AntBot `controller.launch.py` 和 `antbot_hw_interface`；
- Isaac Sim、Gazebo及任何fake hardware；
- Nav2、SLAM Toolbox自动移动；
- 会发布非零 `angular.z` 的遥控或规划器；
- 任何第二个 `/cmd_vel` 到串口的桥接节点。

Nav2放行前必须先补齐真实 `/odom`、`odom -> base_link` 和H743旋转控制，并完成紧急
停车、串口掉线、两条CAN掉线及电源跌落实测。
