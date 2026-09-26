# 把脉代码目录与启动

## 目录

把脉代码已从 `rebotarm/packages/rebotarm_pulse` 实体迁入
`antbot_real_ws/src/rebotarm_pulse`。原位置仅保留兼容软链接。

- `src/rebotarm_pulse/web/`：网页 HTML、CSS、波形逻辑与报告展示数据。
- `src/rebotarm_pulse/rebotarm_pulse/`：ROS 串口采集、网页网关、标定与机械臂预接触代码。
- `src/rebotarm_pulse/config/`：压力零点、几何与预接触配置。
- `src/rebotarm_pulse/launch/`：把脉相关 ROS 启动文件。
- `src/rebotarm_pulse/test/`：把脉模块测试。
- `artifacts/pulse/pulse-diagnosis-site/`：网页展示副本及原站点配置。
- `artifacts/pulse/pulse-diagnosis-site.zip`：原有历史压缩包，未重新打包。

修改网页请优先编辑 `src/rebotarm_pulse/web/`，如需分发展示副本，
同步到 `artifacts/pulse/pulse-diagnosis-site/dist/`。

## 独立启动网页与压力采集

这两个入口直接使用本工作空间源码，不依赖旧 rebotarm 的 build/install，
不启动机械臂或底盘运动。保持两个终端开启，并确保每项服务只启动一份。
已经由机器人 launch 启动压力桥时，不要重复启动。

终端一：

```bash
cd /home/w/project/antbot_real_ws
./scripts/start_pulse_web.sh
```

终端二（设备路径以实际连接为准）：

```bash
cd /home/w/project/antbot_real_ws
./scripts/start_pulse_pressure.sh /dev/robot_serial
```

打开 `http://127.0.0.1:8765/#report`。
本机压力板已通过 `/etc/udev/rules.d/99-robot-usb.rules` 绑定 `/dev/robot_serial`。
总启动默认使用此别名。规则副本位于 `config/udev/99-robot-usb.rules`；
迁移到其他机器需安装规则并 reload/trigger，同一块板换 USB 口不依赖 ACM 编号。
默认订阅压力话题前缀仍为 `/piperh/pulse`，没有改变与机械臂安全系统的接口。
原有 `rebotarm/real_robot_ws/activate.sh` 与 `ros2 run` 指令仍可兼容使用。

## 构建与依赖边界

双臂整机总入口：在工作区根目录运行 `bash start_dual_arm.sh`。
它统一启动双臂、底盘控制和把脉服务；内部底盘 include 禁用重复把脉服务。
不要再同时运行 `scripts/start_antbot_operator.sh` 或独立采集/网页脚本。
机械臂串口 `rebot_channel` 必须与压力串口不同；若默认 `/dev/ttyACM2`
对应压力板，应先确认机械臂设备，再通过 `rebot_channel:=实际机械臂设备`
传入，不能随意猜测设备地址。

真机一键启动（先连接压力传感器，并结束原独立网页/采集终端）：

```bash
cd /home/w/project/antbot_real_ws
./scripts/start_antbot_operator.sh
```

默认同时启动底盘控制界面、把脉网页和压力采集。关闭 RViz 或 Ctrl+C
会停止本次启动的全部进程；不会自动启用底盘运动权限。
压力串口可用 `ANTBOT_PULSE_PORT` 指定，网页端口可用 `ANTBOT_PULSE_WEB_PORT` 指定。
仅启动底盘：`ANTBOT_START_PULSE=false ./scripts/start_antbot_operator.sh`。
`start_base.sh` 等其他低层入口保持原样。

把脉包保留原 ROS 包名 `rebotarm_pulse`，默认构建无需新增机械臂依赖；
一键启动通过源码脚本运行网页和采集服务，由 ROS launch 管理生命周期。
可独立构建：

```bash
./scripts/build.sh --packages-select rebotarm_pulse
```

网页与压力采集需要 ROS 2、rclpy、sensor_msgs、std_msgs、ament_index_python、
Python serial 与 YAML。其余机械臂预接触/标定功能还需要 MoveIt、
rebotarm_moveit_demos、easy_handeye2、AprilTag 和相关机械臂包；
这些独立依赖没有随本次迁移复制，不能认为 AntBot 全量机械臂功能已可独立运行。

原包 README 中的 rebotarm 机械臂路径属于原系统使用说明，
运行旧机械臂功能仍按其依赖条件执行。
