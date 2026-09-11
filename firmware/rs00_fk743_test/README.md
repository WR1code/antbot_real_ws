# FK743M2-IIT6 + RS00 转向电机测试工作区

本目录用于反客 `FK743M2-IIT6`（STM32H743IIT6）通过私有 CAN 协议测试
四台 RS00 转向电机。电机 ID 依次为 FL=`0x01`、FR=`0x02`、RL=`0x03`、
RR=`0x04`，主机 ID 为 `0xFD`，CAN 波特率为 `1 Mbit/s`。

转向和行走电机采用同一物理位置编号：左上/FL=`1`、右上/FR=`2`、
左下/RL=`3`、右下/RR=`4`。RS00 使用扩展帧，MINI 使用标准帧，因此相同的
数字 ID 不会混淆。两种电机的完整运动指令见
[四轮转向与行走电机 CAN 控制指令](docs/STEERING_AND_DRIVE_CAN_COMMANDS.md)。

## 安全设计

- 上电逐台停止、查询 UID、校验位置数据、设置并读回 CSP 安全参数。
- 每台自己的 `mechPos` 原始值预装为 `loc_ref`，不会突然给零位目标。
- `STEERING_AUTO_ENABLE=0`，初始化完成后停在 ARMED。
- 只有显式调用 `SteeringController_RequestEnable()` 才会逐台使能。
- CSP 使用 `run_mode=5`、`limit_spd=1.0 rad/s`、`limit_cur=2.0 A`。
- READY 后每 `20 ms` 刷新位置目标；任一反馈超时或故障会停止全部转向和行走。
- 行走电机使用欧艾迪 MINI 标准 CAN 协议：周期心跳、速度环加/减速度、
  速度目标、主动安全反馈轮询和急停撤流均已接入；真实转速连续低于阈值后才允许
  开始转向。
- RS00 UID、方向和零偏未全部确认时锁定整车运动，只保留架空小步标定；本车
  转向机构允许连续旋转，不启用虚假的 0～π 机械限位。
- 行走协议的 CAN ID、轮径、减速比和电机极对数集中在
  `firmware/App/Inc/drive_config.h`；烧录前必须按实物核对。
- 本工程不会自动保存参数，测试写入在电机掉电后丢失。

即使采用以上限制，RS00 仍是高扭矩执行器。首次运动必须把电机/转向机构架空，
远离机械限位和人员，并准备切断电机动力电源。

## 硬件配置

| 项目           | 配置                                |
| -------------- | ----------------------------------- |
| MCU            | STM32H743IIT6                       |
| 外部晶振       | 25 MHz HSE（板载）                  |
| CPU 时钟       | 480 MHz                             |
| FDCAN 内核时钟 | 25 MHz（HSE）                       |
| RS00 CAN 位率  | FDCAN1，1 Mbit/s                    |
| MINI CAN 位率  | FDCAN2，500 kbit/s                  |
| 位时序         | Prescaler=1, Seg1=19, Seg2=5, SJW=5 |
| CAN RX         | PB8 / FDCAN1_RX                     |
| CAN TX         | PB9 / FDCAN1_TX                     |
| 调试串口       | USART1，PA9/PA10，115200 8N1        |
| 调试接口       | SWD，PA13/PA14                      |

选择 PB8/PB9 是为了避开板载 Type-C 直接连接的 PA11/PA12。详细接线见
[docs/WIRING.md](docs/WIRING.md)。

## 目录

```text
firmware/
  App/                  协议、FDCAN 适配和四轮安全状态机
  RS00FK743/             实际编译和下载的 STM32H743 工程
    RS00FK743.ioc        该工程对应的 CubeMX 配置
    CMakeLists.txt       GNU Arm 固件构建入口
  cubemx_generate.sh    CubeMX 代码生成脚本
tests/
  test_rs00_protocol.c  不连接硬件即可运行的协议测试
  test_mini_drive_protocol.c  欧艾迪标准 CAN 报文字节测试
  test_drive_controller.c  心跳、加减速、速度刷新和急停顺序测试
  test_chassis_direction.c  0～360°纯平移方向映射测试
  test_chassis_translation_controller.c  停轮、对准、恢复和超时测试
docs/
  WIRING.md             接线与电气要求
  RS00_STEERING_STARTUP.md  四轮首次上电、标定和故障排查
  RS00_CSP_180_DEG_TRANSLATION.md  CSP 与纯平移控制说明
  STEERING_AND_DRIVE_CAN_COMMANDS.md  两种电机的 ID 对应与完整 CAN 指令
  SAFETY_COMMISSIONING_AND_WIRING.md  安全接线、标定和长期运行验收
```

## 先运行协议测试

```bash
cd /home/w/project/rs00_fk743_test
cmake -S . -B build/host
cmake --build build/host --parallel
ctest --test-dir build/host --output-on-failure
```

测试会同时核对 RS00 协议，以及欧艾迪文档中的 `0x00`、`0x01`、`0x02`、
`0x0A`、`0x10` 标准 CAN 报文。

## 编译 STM32 固件

实际固件工程是
[firmware/RS00FK743](firmware/RS00FK743)，其中已经接入 `firmware/App`，
无需再手工添加源文件或修改 `main.c`。使用 STM32Cube 安装的 GNU Arm 和 Ninja：

```bash
cd /home/w/project/rs00_fk743_test/firmware/RS00FK743
export PATH=/home/w/.local/share/stm32cube/bundles/gnu-tools-for-stm32/14.3.1+st.2/bin:/home/w/.local/share/stm32cube/bundles/ninja/1.13.2+st.1/bin:$PATH
cmake --preset Debug
cmake --build --preset Debug
```

成功后待下载文件为：

```text
firmware/RS00FK743/build/Debug/RS00FK743.elf
```

只有需要重新配置引脚或时钟时，才打开
[firmware/RS00FK743/RS00FK743.ioc](firmware/RS00FK743/RS00FK743.ioc)。
不要把同目录外的其他 `.ioc` 当作此固件的生成入口。

## 调试入口

在 Watch 窗口查看 `g_steering_debug_state`、`g_steering_debug_error`、
`g_steering_debug_fault_motor_id` 和 `g_steering_can_trace`。四台详细状态由
`SteeringController_GetMotor()` 返回。完整首次上电流程见
[docs/RS00_STEERING_STARTUP.md](docs/RS00_STEERING_STARTUP.md)。

行走调试可查看 `DriveController_IsConfigured()` 和
`DriveController_GetTxErrorCount()`。调用
`ChassisTranslation_CommandDirection(direction_deg, speed_mps)` 后，必须持续
刷新命令（间隔小于 `300 ms`），否则安全超时会把四轮速度降为零。首次架空
测试建议从 `0.03~0.05 m/s` 开始。

USART1 已接入 ROS 2 风格的 `cmd_vel` 二进制命令，支持 `linear.x/linear.y`
平移以及 `angular.z` 原地旋转（正值逆时针、负值顺时针）。协议、接线和上位机脚本
见 [ROS 2 cmd_vel 串口接入](docs/HOST_CMD_VEL_UART.md)。

完整的 UART 二进制 ACK、ST-Link `g_chassis_debug` 快照、断点位置和 USB-CAN
旁路监听步骤见
[UART + ST-Link + USB-CAN 分层实机调试](docs/HARDWARE_DEBUG_UART_SWD_CAN.md)。
调试工具可独立于 ROS 运行：

```bash
python3 host/uart_debug_tool.py \
  --port /dev/ttyUSB0 --vx 0.03 --rate 5 --duration 2 \
  --print-hex --wait-ack
```

不传 `--port` 时，主机工具会优先使用 `RS00_UART_PORT`，否则只在
`/dev/serial/by-id` 中识别唯一的 `USB_Single_Serial` 适配器。它不会猜测任意
`ttyUSB`/`ttyACM`，避免把底盘命令发给其他串口设备；多适配器场景必须显式指定。

编译开关为 `CHASSIS_DEBUG_ENABLE`、`CHASSIS_UART_ACK_ENABLE` 和
`CHASSIS_DEBUG_TEXT_ENABLE`。Debug 构建启用快照，Release 构建关闭快照更新；
ACK 保持可用，文本日志默认关闭。

## 自动故障恢复与远程复位

固件启用约 2 秒超时的 IWDG1 独立看门狗，且只在主循环完成全部任务后喂狗。
主循环、异常处理或系统时钟之后的初始化流程卡死时，MCU 会自动复位；SWD
调试器真正暂停内核时，Debug 构建会冻结看门狗，允许正常使用断点。LSI 有频差，
因此 2 秒是标称值而非精密定时。

Linux 可以通过现有 USART1 链路请求 MCU 级复位：

```bash
python3 host/control_tool.py system-reset
```

固件校验 CRC 和固定 `RST!` 载荷后，先急停并发送 `ACK_ACCEPTED_RESET`，UART
发送完成后调用系统复位。完整命令格式见
[UART 显式控制与查询命令](docs/UART_CONTROL_COMMANDS.md)。

## Linux 实时状态面板

不使用 ROS 时，可以启动只读 Tkinter 面板显示底盘、四台 RS00、四台 MINI、
CAN、UART 和协议健康状态：

```bash
python3 host/chassis_dashboard.py
```

面板不会发送运动或使能命令。它与 Xbox/ROS 串口桥不能同时占用同一串口；
完整字段、串口参数和限制见 [Linux 实时状态面板](docs/HOST_DASHBOARD.md)。

## 长期运行健康与黑匣子

固件现已通过 Backup SRAM 保存启动/看门狗/CPU 异常计数、RCC 复位原因、最后一次
Fault 寄存器和 32 条事件，并上报主循环耗时、主栈水位、PVD 欠压、固件版本、Git
与安全配置指纹。Linux 面板会直接显示这些字段，也可命令行查询：

```bash
python3 host/control_tool.py system-health
python3 host/control_tool.py event-log 8
```

外部看门狗、物理急停、外部电压/温度采样和 A/B 安全升级涉及尚未确定的引脚、
电路和 Flash 分区，工程已保留接口但默认不占用硬件。当前实际启用范围、接线原则
和升级设计见 [STM32H743 长期运行可靠性](docs/LONG_RUNNING_RELIABILITY.md)。

实际接线和首次放行不要只参考功能简介，必须按
[RS00 + MINI 安全接线、标定与长期运行验收手册](docs/SAFETY_COMMISSIONING_AND_WIRING.md)
逐项完成。当前固件保持未标定运动锁定，不需要为这些安全功能新增串口。

## 资料依据

- `/home/w/project/tools/RS00使用说明书260713 (1).pdf`
- `/home/w/project/tools/RS00_私有CAN协议指令详解_DMTool版_每种类型例子解释版.md`
- `/home/w/project/tools/欧艾迪_MINI驱动器_CAN指令详解_DMTool版_V1.0.md`
- `/home/w/project/tools/欧艾迪MINI驱动器产品手册V4.0 - 本(3).pdf`
- 反客开发板资料中的原理图和 `0.cubeMX配置参考.zip`
