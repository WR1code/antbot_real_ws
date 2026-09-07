# Linux 实时状态面板

`host/chassis_dashboard.py` 是只读 Tkinter 图形界面，通过 USB-TTL/USART1 轮询
STM32，不依赖 ROS，也不会发送使能、速度、清故障或复位命令。

## 当前显示内容

- 串口连接、ACK 状态、协议 CRC/重同步/超时计数；
- MCU 运行时间、底盘状态、拒绝原因和故障位；
- CAN 最近发送/接收状态、Bus-Off、Error Passive 和 TX/RX 计数；
- 四台 RS00 的机械角度、反馈年龄、模式、初始化、使能、在线和故障状态；
- 四台 MINI 的 erpm、电流、位置、温度、电压、反馈年龄、单一故障码、软件安全标志和反馈有效位；
- 转向初始化状态、转向错误和平移控制器状态。
- 启动/看门狗/CPU 异常累计、RCC 复位标志和最近一条持久事件；
- 主循环本次/平均/最大耗时、最小空闲栈、PVD 2.85 V 供电状态；
- 固件版本、Git 哈希、配置指纹、已实际启用的可靠性能力；
- 上次 CPU 异常的 PC 和 CFSR。

值显示为 `--` 表示尚未收到对应分页数据或该电机反馈无效，不等于数值 0。
所有计数均来自现有 ACK v3，其中 UART/CAN 计数是 16 位截断值，会自然回绕。

## 启动

依赖为 Python 3、Tkinter 和 pyserial。当前开发机已经具备这些组件：

```bash
cd /home/w/project/rs00_fk743_test
python3 host/chassis_dashboard.py
```

指定串口时，全局参数放在程序名后：

```bash
python3 host/chassis_dashboard.py \
  --port /dev/ttyUSB0 \
  --baud 115200
```

默认每 `0.1 s` 请求一个分页，一轮十八页约 `1.8 s`。可使用
`--request-period 0.2` 降低串口轮询频率。GUI 自动连接并在串口暂时消失时重试；
使用 `--no-autoconnect` 可以先修改界面中的串口再手动连接。

同一串口只能由一个进程占用。启动面板前，需要先退出
`start_xbox_chassis.sh`、`ros2_cmd_vel_uart.py`、`control_tool.py`、串口助手等程序。
如果希望在 Xbox/ROS 控制期间监控，应查看既有 `/rs00/motor_status` ROS 话题，
不能再让该 GUI 同时打开 USART1。

长期运行功能、Backup SRAM 保留范围、尚待接线的 ADC/外部看门狗和安全升级边界见
[STM32H743 长期运行可靠性](LONG_RUNNING_RELIABILITY.md)。
首次接线和标定必须先按
[安全接线、标定与长期运行验收手册](SAFETY_COMMISSIONING_AND_WIRING.md)执行。
