# UART + ST-Link + USB-CAN 分层实机调试

本调试系统是现有 STM32H743 底盘固件的一部分。USB-TTL 承载控制帧和二进制
ACK，ST-Link 观察全局快照，USB-CAN 旁路监听当前选中的一条总线：FDCAN1
监听 RS00，或改接 FDCAN2 监听 MINI，绝不能把两条总线并接。拔掉 ST-Link 或
USB-CAN 后固件仍能独立运行。文本日志默认关闭，不在中断中 `printf`。

## 已核对的工程入口

| 层级 | 当前实现 |
|---|---|
| USART1 初始化 | `firmware/RS00FK743/Core/Src/usart.c`，PA9/PA10，115200 8N1 |
| 接收方式 | `host_cmd_vel_uart.c`，逐字节 `HAL_UART_Receive_IT` + 64 字节环形队列 |
| 控制协议/CRC | `host_cmd_vel_protocol.h/.c`，固定 12 字节，CRC-16/CCITT-FALSE |
| ROS 转发 | `host/ros2_cmd_vel_uart.py` |
| 底盘状态机 | `chassis_translation_controller.h/.c` |
| 转向使能入口 | `SteeringController_RequestEnable()`，默认不自动调用 |
| FDCAN1 初始化 | `firmware/RS00FK743/Core/Src/fdcan.c`，PB8/PB9，RS00，1 Mbit/s |
| FDCAN2 初始化 | `firmware/RS00FK743/Core/Src/fdcan.c`，PB12/PB13，MINI，500 kbit/s |
| CAN 发送入口 | `rs00_fdcan_send_detailed()` / `rs00_fdcan_send_standard()` |
| CAN 接收回调 | `HAL_FDCAN_RxFifo0Callback()` → `rs00_fdcan_on_rx_fifo0()` |
| RS00 ID | `steering_config.h` 的 `STEERING_MOTOR_IDS`，扩展帧 |
| MINI ID | `drive_config.h` 的 `DRIVE_CAN_ID_*`，标准帧 |
| 测试框架 | 根目录 CMake + CTest；Python 使用 `unittest` 并接入 CTest |

所有四轮字段固定按左上/FL、右上/FR、左下/RL、右下/RR 排列；RS00 ID 为
`1/2/3/4`，MINI ID 为 `5/6/7/8`。

仅实际构建的 `firmware/RS00FK743/RS00FK743.ioc` 配置了 USART1。没有可靠依据
启用 USART2/3/UART4，因此本实现没有占用第二串口。

## 接线

USB-TTL（必须是 STM32 兼容 TTL 电平，不是 RS-232）：

```text
USB-TTL TX  -> STM32 PA10 / USART1_RX
USB-TTL RX  -> STM32 PA9  / USART1_TX
USB-TTL GND -> STM32 GND
```

不要把 RS-232 电平或 USB-TTL 的 5 V 接到 STM32 信号脚。

ST-Link：

```text
SWDIO       -> SWDIO / PA13
SWCLK       -> SWCLK / PA14
GND         -> GND
VTref/3.3 V -> 目标板电压参考
NRST        -> NRST（建议）
```

开发板和底盘使用自身稳定电源；不要默认由 ST-Link 给整个底盘供电。

USB-CAN 作为只监听节点并联：

```text
USB-CAN CAN_H -> 总线 CAN_H
USB-CAN CAN_L -> 总线 CAN_L
USB-CAN GND   -> 系统信号 GND
```

总线断电后测 CAN_H 与 CAN_L 应约为 60 Ω。总线两端只保留两个 120 Ω 终端；
旁路 USB-CAN 不得再形成第三个终端。监听 RS00 时设为 Classic CAN、1 Mbit/s；
监听 MINI 时改接 FDCAN2 总线并设为 Classic CAN、500 kbit/s。默认使用
listen-only，不发送电机控制帧，也不要把两条 CAN 总线并接。

同一个 USB 串口同时只能由 ROS 转发脚本、普通串口助手、VSCode Serial
Monitor、`uart_debug_tool.py` 之一打开。ST-Link 和 USB-CAN 可同时工作。

## UART 控制帧与 ACK

既有 12 字节 `cmd_vel` 控制帧 `AA 55 01 ... CRC16` 保持兼容；显式控制命令
使用 16 字节 `AA 55 02 ... CRC16`。ACK v3 是固定 52 字节，追加四个 RS00
机械位置和分页查询结果；所有多字节字段均为小端，CRC 参数与控制帧相同：

| 偏移 | 长度 | 字段 |
|---:|---:|---|
| 0 | 1 | `A5` |
| 1 | 1 | `5A` |
| 2 | 1 | ACK 协议版本 `03` |
| 3 | 1 | 帧长 `34`（52） |
| 4 | 1 | 对应命令序号 |
| 5 | 1 | ACK 状态 |
| 6 | 1 | 调试底盘状态 |
| 7 | 1 | 命令拒绝原因 |
| 8 | 2 | 故障位 |
| 10 | 1 | 转向位：bit0 enabled、bit1 homed、bit2 ready、bit3 fault |
| 11 | 1 | CAN 位：bit0 最近TX提交成功、bit1 最近500 ms有有效反馈、bit2 Bus-Off、bit3 error-passive |
| 12 | 2 | UART 有效帧计数低 16 位 |
| 14 | 2 | CAN 提交成功计数低 16 位 |
| 16 | 2 | CAN 接收计数低 16 位 |
| 18 | 4 | STM32 `HAL_GetTick()` |
| 22 | 2 | FL 转向位置，`int16`，单位 mrad；`-32768` 表示尚未初始化 |
| 24 | 2 | FR 转向位置，格式同上 |
| 26 | 2 | RL 转向位置，格式同上 |
| 28 | 2 | RR 转向位置，格式同上 |
| 30 | 1 | 对应显式控制命令 ID；旧 `cmd_vel` 为 0 |
| 31 | 1 | 分页详情类型 |
| 32 | 16 | 四个 `int32` 详情值，顺序 FL/FR/RL/RR |
| 48 | 2 | 详情有效位掩码 |
| 50 | 2 | 字节 0～49 的 CRC16，小端 |

ACK 状态值：

```text
0 OK                         7 REJECTED_STEERING_FAULT
1 BAD_HEADER                 8 REJECTED_ANGULAR_Z
2 BAD_LENGTH                 9 ACCEPTED_WAIT_STEERING
3 BAD_CRC                   10 ACCEPTED_DRIVING
4 UNSUPPORTED_COMMAND       11 TIMEOUT_STOP
5 REJECTED_DISABLED         12 CAN_TX_ERROR
6 REJECTED_NOT_HOMED        13 CAN_BUS_OFF
                             14 ACCEPTED_RESET
```

固定 12 字节接收器没有可变长度字段，`BAD_LENGTH` 为协议保留值。ACK 使用深度
4 的固定队列和 `HAL_UART_Transmit_IT()`；队列满只增加
`uart_ack_drop_count`。ACK 帧头是 `A5 5A`，不会被 `AA 55` 控制解析器接受。

## 状态、拒绝原因和故障

调试状态：`BOOT=0`、`DISABLED=1`、`WAIT_ENABLE=2`、
`WAIT_HOMING=3`、`IDLE=4`、`STEERING=5`、`DRIVE=6`、
`TIMEOUT_STOP=7`、`FAULT=8`、`EMERGENCY_STOP=9`。

拒绝原因：`NONE=0`、`DISABLED=1`、`NOT_HOMED=2`、
`STEERING_FAULT=3`、`NONZERO_ANGULAR_Z=4`、`COMM_TIMEOUT=5`、
`INVALID_COMMAND=6`、`CAN_FAULT=7`。

故障位：bit0 转向、bit1 行走、bit2 CAN、bit3 CAN Bus-Off、bit4 UART。
这些是对既有详细状态机的调试映射，不改变电机方向、零点、轮径、减速比或 ID。

## ST-Link Watch 与断点

主快照是 `volatile ChassisDebugSnapshot g_chassis_debug`；最近原始帧还可直接查看
`volatile uint8_t uart_last_raw_frame[12]`。建议 Watch：

```text
g_chassis_debug.uart_rx_byte_count
g_chassis_debug.uart_valid_frame_count
g_chassis_debug.uart_crc_error_count
g_chassis_debug.command_vx_mps
g_chassis_debug.command_vy_mps
g_chassis_debug.chassis_state
g_chassis_debug.chassis_reject_reason
g_chassis_debug.steering_enabled
g_chassis_debug.steering_ready
g_chassis_debug.can_tx_submit_ok_count
g_chassis_debug.can_tx_submit_error_count
g_chassis_debug.can_rx_frame_count
g_chassis_debug.fdcan_bus_off
g_chassis_debug.fdcan_error_passive
g_chassis_debug.fdcan_last_error_code
g_chassis_debug.fdcan_tx_error_count
g_chassis_debug.fdcan_rx_error_count
g_chassis_debug.last_can_tx_id
g_chassis_debug.last_can_rx_id
g_chassis_debug.last_can_tx_data
g_chassis_debug.last_can_rx_data
```

推荐断点：

```text
DebugHook_UartRx
DebugHook_UartFrameValid
DebugHook_UartFrameRejected
DebugHook_ChassisStateChanged
DebugHook_CanTxBeforeSubmit
DebugHook_CanTxSubmitted
DebugHook_CanTxFailed
DebugHook_CanRxReceived
DebugHook_Fault
DebugHook_TimeoutStop
```

断点暂停会自然触发 300 ms 通信超时，这是预期安全行为。真实电机运行时不要
长时间停在断点；优先用 Watch 和计数器。

FDCAN 状态每 20 ms 读取 `HAL_FDCAN_GetProtocolStatus()` 和
`HAL_FDCAN_GetErrorCounters()`。Bus-Off 一旦观察到便锁存，所有后续发送被
拒绝，底盘进入故障且不会恢复旧命令。断电、排除总线问题、重启并重新完成显式
使能后才能继续。

## 上位机工具

无需 ROS，只生成与 ROS 脚本完全一致的 HEX：

```bash
cd /home/w/project/rs00_fk743_test
python3 host/uart_debug_tool.py \
  --vx 0.03 --vy 0 --wz 0 --print-frame-only
```

输出应为：

```text
AA 55 01 00 1E 00 00 00 00 00 DE 0E
```

以 5 Hz 发送 2 秒并解析 ACK：

```bash
python3 host/uart_debug_tool.py \
  --port /dev/serial/by-id/usb-1a86_USB_Single_Serial_5CE6063665-if00 --baud 115200 \
  --vx 0.03 --vy 0 --wz 0 --rate 5 --duration 2 \
  --print-hex --wait-ack
```

单帧：

```bash
python3 host/uart_debug_tool.py \
  --vx 0.03 --send-once --wait-ack --ack-timeout-ms 200 --print-hex
```

监听或停车：

```bash
python3 host/uart_debug_tool.py --listen-only
python3 host/uart_debug_tool.py --stop --send-once --wait-ack
```

Ctrl+C 时工具会发送零速度停车帧，并在 `finally` 中关闭串口。

## 首次实机步骤

1. 四轮架空，人员远离机构，急停和切断电机动力的手段可用。
2. 先只接一个 RS00 和一个 MINI；断电检查总线约 60 Ω。
3. 先不给执行器动力，连接 UART、ST-Link、只监听 USB-CAN，检查 UART/ACK。
4. 上电等待转向初始化。启动阶段预期 `REJECTED_NOT_HOMED`；到 ARMED 后预期
   `REJECTED_DISABLED`，不得静默运动。
5. 核对 UID、方向、零偏和连续旋转线束余量后，在调试器表达式窗口显式执行
   `SteeringController_RequestEnable()`；确认返回 `true`。
6. 观察 `steering_ready=1`，再运行上述 `0.03 m/s`、5 Hz、2 秒命令。
7. 预期先 `STEERING`，对准后 `DRIVE`；停止有效帧后 300 ms 内变为
   `TIMEOUT_STOP` 并停车。
8. 完成单组验证后才逐个增加设备；首次速度保持 `0.03～0.05 m/s`。

## 分层验收与故障定位

| 层 | 预期 | 异常重点检查 |
|---|---|---|
| USB-TTL 发送 | 工具 TX 计数、12 字节 HEX 正确 | 串口号/占用、参数 |
| STM32 UART | `uart_rx_byte_count` 增加 | TX/RX 交叉、GND、USART 中断 |
| 协议 | `uart_valid_frame_count` 增加，CRC错误为0 | 帧头、长度、CRC、大小端、错位 |
| 状态机 | WAIT_HOMING/WAIT_ENABLE/STEERING/DRIVE | 未使能、未标定、转向故障 |
| CAN 提交 | `can_tx_submit_ok_count` 增加、提交错误为0 | FIFO、FDCAN 启动、HAL 错误 |
| USB-CAN | 能看到 STM32 发出的标准/扩展帧 | 波特率、接线、收发器 |
| 电机反馈 | RS00 或 MINI RX 计数增加 | ID、协议、使能、过滤器 |
| 执行 | 先转向后低速行走，300 ms 超时停车 | 对准、方向、反馈、故障 |

CAN 已提交但 `TxErrorCnt` 持续增加，通常表示没有 ACK、接线或波特率错误、
设备未供电或收发器异常。USB-CAN 能看到发送但无反馈，应核对代码中的实际
ID、协议、设备使能和过滤器。插入设备后全总线失效，重点排查 H/L 反接、重复
终端或该设备收发器。出现 Bus-Off 必须停止实机测试。
