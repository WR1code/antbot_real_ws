# RS00 四轮转向安全启动说明

## 1. 适用范围和资料性质

本文只说明四台 RS00 转向电机的安全启动子系统。完整固件还包含独立 FDCAN2
上的四台欧艾迪 MINI 行走电机、纯平移控制、USART1 二进制上位机协议和 ROS 2
`cmd_vel` 串口桥；相关入口见文末链接。

- **手册规定**：RS00 私有 CAN 的 29 位扩展 ID、类型 0/2/3/4/17/18/21、
  参数 index、字节序和反馈映射。
- **工程安全策略**：默认不使能、逐台初始化、100 ms 应答超时、最多重试
  3 次、任一电机异常时停止全部四台。
- **待实机确认**：四台 UID、安装方向、零位偏置、机械角度范围、CAN 收发器
  与板端引脚的实际连线。

首次通电必须架空四个转向轮，人员和线束远离机构，并准备可立即切断电机动力
电源。不要仅依赖软件急停。

## 2. 文件结构

```text
firmware/App/Inc/steering_config.h       四轮 ID、参数和安全开关
firmware/App/Inc/steering_controller.h   状态、对象和控制 API
firmware/App/Src/steering_controller.c   非阻塞初始化/控制状态机
firmware/App/Inc/rs00_protocol.h         RS00 帧和参数定义
firmware/App/Src/rs00_protocol.c         编码、字节序和反馈解码
firmware/App/Src/rs00_stm32_fdcan.c      STM32 HAL FDCAN 适配
firmware/RS00FK743/Core/Src/main.c       CubeMX USER CODE 接入
```

## 3. CAN 身份和工程 FDCAN 配置

| 位置 | 名称 | 电机 ID |
|---|---|---:|
| 左上 | FL | `0x01` |
| 右上 | FR | `0x02` |
| 左下 | RL | `0x03` |
| 右下 | RR | `0x04` |

物理位置和数组顺序始终为左上/FL、右上/FR、左下/RL、右下/RR；不得按接线
顺序或电机旋向重新排列。MINI 行走电机按同一位置顺序使用 ID `5/6/7/8`，
并位于独立的 FDCAN2（500 kbit/s）。

主机 ID 为 `0xFD`。实际固件使用 `FDCAN1`：

| 项目 | 当前工程值 |
|---|---|
| 模式 | Classic CAN，Normal，自动重发 |
| 标称波特率 | 1 Mbit/s |
| FDCAN 内核时钟 | 25 MHz HSE |
| 位时序 | Prescaler=1, Seg1=19, Seg2=5, SJW=5 |
| 帧 | 29 位扩展数据帧，DLC=8，BRS off |
| 接收 | 扩展帧进入 FIFO0；标准帧和远程帧拒绝 |
| FIFO/队列 | RX FIFO0 16 项，TX FIFO 8 项 |
| CubeMX 引脚 | PB8=FDCAN1_RX，PB9=FDCAN1_TX |

PB8/PB9 是现有 `.ioc` 中的配置，不是协议层猜测。烧录前仍须结合 FK743M2-IIT6
核心板原理图确认这两个 MCU 引脚是否接到了所用 CAN 收发器。MCU FDCAN 引脚
不能直接连接 CAN_H/CAN_L。

CAN 总线两端各放一个 120 Ω 终端电阻；断电测量 CAN_H 与 CAN_L 之间应约为
60 Ω。所有节点共地，并核对 CAN_H/CAN_L 没有接反。

## 4. 上电状态机

`SteeringController_Init(&hfdcan1)` 配置过滤器、启动 FDCAN 和 FIFO0 新消息
中断。主循环持续调用 `SteeringController_Task()`，没有长时间
`HAL_Delay()`。

状态机依次执行：

1. 非阻塞等待电机启动 500 ms。
2. 逐台停止，收到各自类型 2 反馈后继续。
3. 逐台查询 UID，验证不是全 `00` 或全 `FF`。
4. 逐台读取 `mechPos (0x7019)`；保存 float 和每台自己的原始 4 字节。
5. 检查机械位置有限；本车连续旋转机构不执行 0～π 机械范围检查。
6. 写 `run_mode=5`，然后读回 `0x7005`。
7. 写并逐项读回 `limit_spd=1.0 rad/s`、`limit_cur=2.0 A`、
   `canTimeout=10000`（约 500 ms）。
8. 把每台自己的 `mechPos` 写入本机 `loc_ref (0x7016)` 并读回。
9. 默认进入 `ARMED`，不发送使能帧。
10. 显式请求后逐台使能；每台必须返回无故障、运行模式、低速且新鲜的类型 2
   反馈。
11. 四台验证成功后进入 `READY`，每 20 ms 保持发送四台当前 `loc_ref`。

常规请求 ID 为：

```text
((type & 0x1F) << 24) | (0xFD << 8) | motor_id
```

关键命令数据：

| 步骤 | ID（xx=电机 ID） | Data |
|---|---|---|
| 查询 | `0000FDxx` | `00 00 00 00 00 00 00 00` |
| 停止 | `0400FDxx` | `00 00 00 00 00 00 00 00` |
| 读 `mechPos` | `1100FDxx` | `19 70 00 00 00 00 00 00` |
| CSP 模式 | `1200FDxx` | `05 70 00 00 05 00 00 00` |
| CSP 限速 | `1200FDxx` | `17 70 00 00 00 00 80 3F` |
| 限流 | `1200FDxx` | `18 70 00 00 00 00 00 40` |
| CAN 超时 | `1200FDxx` | `28 70 00 00 10 27 00 00` |
| 预装位置 | `1200FDxx` | `16 70 00 00 pp pp pp pp` |
| 使能 | `0300FDxx` | `00 00 00 00 00 00 00 00` |

`pp pp pp pp` 是同一台电机 `mechPos` 的 float 小端编码。

## 5. 预期返回和调试观察

- 查询返回：类型 0，ID 的 bit15..8 是来源电机 ID，bit7..0 为 `0xFE`，
  Data 是 8 字节 UID。
- 参数读取：类型 17，bit23..16 为 0 表示成功，bit15..8 是来源 ID，
  bit7..0 是 `0xFD`；Data Byte0..1 必须匹配等待的 index。
- 状态反馈：类型 2，ID 中包含模式、6 位 fault、来源 ID 和主机 ID；
  Data 按大端映射位置、速度、力矩和温度。
- 故障反馈：类型 21 的 8 字节原样保存在 `fault_raw`；任何非零 fault
  触发四轮停止。

调试器中查看：

```text
g_steering_debug_state
g_steering_debug_error
g_steering_debug_fault_motor_id
g_steering_can_trace
SteeringController_GetMotorSnapshot(STEERING_MOTOR_FL ... RR, &snapshot)
```

多字段状态应使用 `GetMotorSnapshot()` 的短临界区一致性副本。四个实际 UID
在各 `SteeringMotor.received_uid`。把它们填入
`steering_config.h` 的 `STEERING_EXPECTED_UIDS`，再把
`STEERING_ENFORCE_UID_CHECK` 改为 `1`。不要在未记录 UID 时开启强制检查。

USART1（PA9/PA10，115200 8N1）已用于 12 字节 `cmd_vel` 帧、16 字节显式控制帧
和二进制 ACK，不能再由串口助手或另一上位机进程同时占用。文本日志默认关闭；
若实现 `SteeringController_Log(level, message)`，应使用不会与二进制协议混流的
RTT/ITM，或先明确停用 USART1 二进制链路。中断只更新缓存，不格式化或阻塞输出。

## 6. 使能、角度命令和急停

安全默认值：

```c
#define STEERING_AUTO_ENABLE 0
```

初始化完成并确认四台 UID、位置和参数后，显式调用：

```c
if (SteeringController_GetState() == STEERING_STATE_ARMED) {
    (void)SteeringController_RequestEnable();
}
```

只有进入 `READY` 后才接受角度。目标会经过每台的方向符号和零偏；连续旋转
机构选择离当前保持目标最近的等效圈数。`STEERING_CALIBRATION_CONFIRMED=0` 时整车运动被锁定，直接转向
命令还必须相对上一目标不超过 `0.01 rad`，用于架空标定：

```c
(void)SteeringController_SetAllAngles(0.01f, 0.01f, 0.01f, 0.01f);
```

这四个参数是底盘角度，经 `direction_sign * chassis_angle + zero_offset`
转换。若未完成零偏标定，函数可能因相对当前保持位置步长过大而安全拒绝。

软件急停：

```c
SteeringController_EmergencyStop();
```

它立即尽力发送 `0400FD01` 到 `0400FD04`，清除 READY 并进入 FAULT。
清故障只允许显式调用 `SteeringController_ClearFaultAndRestart()`；其停止帧
Data Byte0 为 `01`，之后重新走完整初始化，且默认仍停在 ARMED。

## 7. 安装方向与零偏标定

`STEERING_DIRECTION_SIGNS` 和 `STEERING_ZERO_OFFSETS_RAD` 初始均为 `+1` 和
`0`，只用于代码初始化，不能根据 CAN ID 推断方向。

1. 架空全部转向轮，只连接一台待测电机或确保其余电机不会运动。
2. 完成 UID 验证并使系统进入 ARMED。
3. 显式使能，采用不超过 `0.01 rad` 的小步长。
4. 记录底盘正方向命令对应的实际轮向；反向者把 sign 改为 `-1`。
5. 机械对中后记录电机 `mechPos`，求出使底盘角度 0 对应的 zero offset。
6. 本车转向机构已解除 360°机械限制，保持
   `STEERING_ENABLE_MECHANICAL_LIMIT_CHECK=0`。固件会选择离当前保持目标最近的
   等效圈数，避免编码器跨圈后无故多转一整圈。

完成全部实机验证前不要把 `STEERING_AUTO_ENABLE` 改为 `1`。
完成 UID、方向和零偏后，同时填写 `STEERING_EXPECTED_UIDS`、设置
`STEERING_ENFORCE_UID_CHECK=1` 和 `STEERING_CALIBRATION_CONFIRMED=1`；编译器会
拒绝“已标定但未强制 UID”的组合。

## 8. 故障排查

1. 先切断电机动力，保存 `g_steering_debug_*`、四台 `last_error`、
   `fault_raw` 和最后 RX/TX 帧。
2. 若 UID 超时：检查 ID、1 Mbit/s、扩展帧、收发器供电/待机脚、终端电阻和
   CAN_H/CAN_L。
3. 若参数失败：核对返回 result、index，以及电机固件是否支持当前 CSP 参数
   `0x7005/0x7017/0x7018/0x7028`；本状态机不写 PP 参数 `0x7024/0x7025`。
4. 若模式或反馈超时：确认写参数后存在类型 2 应答，且主机目标刷新周期没有
   被其他阻塞任务拖延。
5. 若 fault 非零：保留原始 8 字节并按对应电机固件手册解释；不要在循环中
   自动清故障。
6. 查明原因后，才显式调用 ClearFaultAndRestart；它不会直接恢复 READY。

## 9. 功能边界与相关文档

本启动流程不会修改 CAN ID、波特率、通信协议、机械零位，也不会保存 Flash
参数或执行电机初始化/齿槽标定。测试写入在电机掉电后丢失。完整 CAN 指令、
行走与纯平移、ROS 2 串口桥和显式上位机命令分别见：

- [四轮转向与行走电机 CAN 控制指令](STEERING_AND_DRIVE_CAN_COMMANDS.md)
- [CSP 与纯平移控制](RS00_CSP_180_DEG_TRANSLATION.md)
- [ROS 2 cmd_vel 串口接入](HOST_CMD_VEL_UART.md)
- [UART 显式控制与查询命令](UART_CONTROL_COMMANDS.md)
