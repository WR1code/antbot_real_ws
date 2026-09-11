# ROS 2 `cmd_vel` 到 STM32 的 USART1 控制链路

## 数据流

```text
/cmd_vel (geometry_msgs/Twist)
 -> ros2_cmd_vel_uart.py
 -> USB-TTL / USART1, 115200 8N1
 -> ChassisTranslation_CommandTwist(vx, vy, wz)
 -> FDCAN1 / 1 Mbit/s -> 四台 RS00（ID 1/2/3/4）
 -> FDCAN2 / 500 kbit/s -> 四台 MINI（ID 5/6/7/8）
```

两条总线均使用同一物理顺序：左上/FL、右上/FR、左下/RL、右下/RR；对应
RS00 ID `1/2/3/4`、MINI ID `5/6/7/8`。

当前底盘控制器支持平移和原地旋转：

- `linear.x`：前后速度，单位 `m/s`
- `linear.y`：左右速度，单位 `m/s`
- `angular.z`：正值逆时针、负值顺时针，范围 `-1.0～1.0 rad/s`

四个转向轴中心位于 `330 mm × 330 mm` 正方形顶点，旋转半径为
`330/sqrt(2) = 233.35 mm`。原地旋转时逻辑转向角为
FL/RR=`135°`、FR/RL=`45°`，后轮行走方向与前轮相反。为避免误操作，非零
`angular.z` 优先于手柄平移；STM32 对其他来源的平移与旋转混合帧会拒绝并停车。

ROS `base_link` 的 `x` 向前、`y` 向左，与本工程约定一致。

当前实机零位约定为：逻辑转向 0°配合 MINI 正转时朝车体前方；四轮零偏已在
轮子对准车头时采集，安装方向仍需用架空小步动作最终确认。桥接程序在控制期间复用同一串口
轮询电机反馈，每秒在终端输出四个 RS00 角度以及四个 MINI 的转速、电流和故障
摘要，并把包含位置、温度、电压、反馈年龄、安全标志、有效位和 CAN 计数器的
完整 JSON 发布到 `/rs00/motor_status`：

```bash
ros2 topic echo /rs00/motor_status
```

## 使能前提

为了避免上电即运动，当前
`STEERING_AUTO_ENABLE=0`、`CHASSIS_AUTO_ENABLE_AFTER_STARTUP=0`。因此收到
串口命令不会绕过 RS00 的显式使能门槛。除此以外，
`STEERING_CALIBRATION_CONFIRMED=0` 时任何非零整车运动都被拒绝；四台 MINI 的
故障、速度、电压、电流和温度未全部建立新鲜反馈时也会拒绝运动。完成四轮架空
标定和方向核对后，可由
应用显式调用 `SteeringController_RequestEnable()`；确认需要上电自动使能时，
再把 `CHASSIS_AUTO_ENABLE_AFTER_STARTUP` 改为 `1`。首次联调不要开启自动使能。

## 接线

USART1 使用 3.3 V TTL 电平：

| STM32H743 | USB-TTL |
|---|---|
| `PA10 / USART1_RX` | `TX` |
| `PA9 / USART1_TX` | `RX`，当前控制可不接 |
| `GND` | `GND` |

不能把 RS-232 电平直接接到 STM32。串口参数为 `115200, 8N1`。

## 二进制帧

每帧固定 12 字节，所有多字节整数均为小端：

| 偏移 | 长度 | 内容 |
|---:|---:|---|
| 0 | 2 | 帧头 `AA 55` |
| 2 | 1 | 协议版本 `01` |
| 3 | 1 | 循环序号 |
| 4 | 2 | `vx`，`int16`，单位 `mm/s` |
| 6 | 2 | `vy`，`int16`，单位 `mm/s` |
| 8 | 2 | `wz`，`int16`，单位 `mrad/s` |
| 10 | 2 | CRC-16/CCITT-FALSE，小端 |

CRC参数：初值 `0xFFFF`、多项式 `0x1021`，覆盖字节 0～9。

示例：`vx=0.05 m/s, vy=0, wz=0` 在编码前转换为
`vx_mm_s=50, vy_mm_s=0, wz_mrad_s=0`。

## 上位机运行

安装依赖：

```bash
python3 -m pip install pyserial
```

在已经安装并 source ROS 2 的终端运行：

```bash
python3 host/ros2_cmd_vel_uart.py \
  --ros-args \
  -p port:=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5CE6063665-if00 \
  -p baud:=115200 \
  -p max_linear_speed:=0.05
```

另一个终端以 20～50 Hz 发布测试命令：

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

停止发布后，STM32 的 300 ms 命令超时会把四台 MINI 的速度目标降为零。

## 复用 AntBot Xbox 遥控

AntBot 的 `mapping_xbox` 节点发布标准 `/cmd_vel`，可以直接复用本工程的串口桥：

```bash
cd /home/w/project/rs00_fk743_test

# 交互确认安全后，自动完成清故障、初始化、显式使能和 Xbox/串口桥启动
./start_xbox_chassis.sh
```

启动器只有确认 `state=IDLE` 后才进入遥控；初始化或使能失败会停止并打印诊断
状态。非交互运行必须显式设置 `RS00_ASSUME_SAFE=1`，普通实机操作不要设置它。

默认最大速度为 `0.25 m/s`，可用 `RS00_MAX_LINEAR_SPEED` 调整；串口和手柄设备
可分别用 `RS00_UART_PORT`、`RS00_JOY_DEVICE` 覆盖。该入口不会启动 AntBot 原有的
`controller.launch.py`/`antbot_hw_interface`，避免两个底盘执行端同时消费
`/cmd_vel`。Xbox 的 LT 触发器控制逆时针原地旋转，RT 控制顺时针原地旋转；
触发器有输入时旋转优先并把平移置零。左摇杆、A 键安全锁和手柄掉线锁定仍沿用
AntBot 逻辑。

同一个 UART 不能同时被该入口、`control_tool.py`、`uart_debug_tool.py` 或串口助手
打开。需要查询或重新使能时，先退出 Xbox 控制入口。

工程登记的沁恒 USB Single Serial 稳定名称为
`/dev/serial/by-id/usb-1a86_USB_Single_Serial_5CE6063665-if00`。不显式传入端口时，
脚本会读取 `RS00_UART_PORT`，或安全识别唯一的
`usb-1a86_USB_Single_Serial_*-if00`；不会猜测任意 `ttyUSB`/`ttyACM`。

## 固件行为与调试

USART1中断只把字节放进环形缓冲区；解析和底盘命令调用在主循环完成。合法的
重复命令只刷新安全时间戳，不会重新启动停车和转向流程。

可以在调试器中调用 `HostCmdVelUart_GetDebugSnapshot()` 查看：

- 已接受命令数
- 无效或混合角速度拒绝数
- 超范围运动命令拒绝数
- UART溢出/错误数
- CRC错误数及最后序号

首次旋转测试必须架空四轮，从轻压 LT/RT 开始，并确认 LT 为逆时针、RT 为顺时针；
首次平移测试仍应把最大线速度限制在 `0.03～0.05 m/s`。

完整解锁条件和逐项验收见
[安全接线、标定与长期运行验收手册](SAFETY_COMMISSIONING_AND_WIRING.md)。
