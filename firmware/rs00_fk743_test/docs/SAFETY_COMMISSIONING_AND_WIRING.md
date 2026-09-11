# RS00 + MINI 安全接线、标定与长期运行验收手册

本文是当前固件的实机操作顺序。不要跳过“架空、断动力、只做查询”阶段。
RS00 是高扭矩转向执行器，MINI 手册中的电子停车也不能替代实体急停。

## 1. 先看结论

- 不需要新增串口。Linux 查询、控制、看门狗状态和远程 MCU 复位都复用现有
  `USART1`（PA9/PA10，115200 8N1）。同一时间只能有一个主机程序打开它。
- RS00 和 MINI 必须使用两条独立 CAN：RS00 为 FDCAN1/1 Mbit/s，MINI 为
  FDCAN2/500 kbit/s，两对 CAN_H/CAN_L 绝不能并接。
- 当前固件故意保持“未标定锁定”：可查询和小步标定 RS00，但不能让整车行走。
  填完 UID、方向、零偏和机械安全范围后才能显式解锁。
- MINI 由 STM32 主动查询故障、速度、电压、电流和温度；即使驱动器没有配置
  周期主动上报，也能完成安全监测。
- 软件急停、串口急停和看门狗都不是实体急停。实体急停必须直接切断驱动器使能
  或电机动力，并使用安全继电器/接触器；STM32 只能读取辅助触点用于记录。

## 2. 当前接线和接口

| 用途 | STM32 引脚 | 参数 | 外部器件 |
|---|---|---|---|
| RS00 CAN | PB8 RX、PB9 TX | FDCAN1，1 Mbit/s | 独立 3.3 V CAN 收发器 |
| MINI CAN | PB12 RX、PB13 TX | FDCAN2，500 kbit/s | 独立 3.3 V CAN 收发器 |
| Linux 主机 | PA10 RX、PA9 TX | USART1，115200 8N1，3.3 V TTL | USB-TTL |
| 下载调试 | PA13 SWDIO、PA14 SWCLK | SWD | ST-Link |

排针版模块编号为 PB8=`43`、PB9=`46`、PB12=`82`、PB13=`79`、PA9=`64`、
PA10=`62`、PA13=`63`、PA14=`61`。编号来自 FK743M2-IIT6 V1.1 原理图；接线前
仍需用正反面丝印确认模块方向。核心板推荐由排针 `5/6` 中任一处输入稳定 5 V，
禁止连接电机动力电压。

USB-TTL 交叉接线：

```text
USB-TTL TX  -> PA10 / USART1_RX
USB-TTL RX  -> PA9  / USART1_TX
USB-TTL GND -> STM32 GND
```

不要把 USB-TTL 的 5 V 接到 STM32 3.3 V，也不要接 RS-232 的正负电压接口。
只有 RX 不足以查看 ACK，查询和远程复位必须把 TX/RX/GND 都接好。

每条 CAN 都采用总线拓扑而不是星形拓扑：在线缆物理两端各放一个 120 Ω，其他
节点关闭终端。完全断电后测该总线 CAN_H 与 CAN_L 应约为 60 Ω。STM32、收发器和
驱动器需要参考地；电机动力线与 CAN/串口线分开走线。

## 3. 首次上电前的实体安全条件

1. 四个轮子全部架空，转向机构远离机械限位和人员。
2. 准备一个无需 STM32 参与、可直接切断驱动器使能或动力的实体急停。
3. 先只给控制电源，能分开供电时最后才接电机动力；保险丝、接触器和线径按
   电源与驱动器额定值设计。
4. 不使用 MINI 的 `0x09` 手刹命令。原厂文档明确提示应用不得使用，误用有损坏
   电机风险。
5. 不把 MINI 的 `0x08` 回馈制动当作急停。本项目不发送它，因为回馈能量可能
   抬高 24 V 母线电压。正常停车使用目标速度降零；急停则降零并发送零电流撤流。
   只有四轮实际转速连续低于阈值后，状态机才允许转向。
6. 第一次测试只允许一人操作，另一人站在动力断开装置旁；不要依靠键盘窗口获得
   焦点后才能停车。

## 4. 用 MINI 上位机写入的参数

四台驱动器选择普通 CAN（不是 CANopen），波特率统一为 500 kbit/s：

| 位置 | 接收 ID | 发送 ID |
|---|---:|---:|
| FL 左前 | 5 | 5 |
| FR 右前 | 6 | 6 |
| RL 左后 | 7 | 7 |
| RR 右后 | 8 | 8 |

再设置：

- 心跳超时：1000 ms；STM32 正常情况下每 100 ms 以内发送一次 `00`。
- 驱动器自带终端：只有它确实位于线缆物理端点时才打开，否则关闭。
- 异步反馈周期：全部设为 `-1 ms`（关闭），避免四台驱动器高频回传与同 ID
  控制帧争用；固件每 5 ms 轮询一项，约
  100 ms 能刷新四台的完整安全反馈。
- 电机参数：24 V、150 W、10 极对、编码器配置值 4096、直驱 1:1；必须与实物
  铭牌和上位机读回一致。

MINI 故障值是一个 `0..19` 的故障码，不是可组合位掩码。固件保护阈值见第 9 节。

## 5. 保持第一次固件处于锁定状态

首次烧录时确认下列值不要修改：

```c
// firmware/App/Inc/steering_config.h
#define STEERING_AUTO_ENABLE                   0
#define STEERING_ENFORCE_UID_CHECK             0
#define STEERING_CALIBRATION_CONFIRMED          0

// firmware/App/Inc/drive_config.h
#define DRIVE_ALLOW_RAW_WHEEL_COMMANDS          0
```

此状态允许查询 RS00 UID 和位置、显式使能后以每条不超过 `0.01 rad`（约 0.57°）
的小步命令标定，但整车 `cmd_vel` 和非零原始轮速都会被拒绝。面板应显示
“未标定：禁止整车运动”。

## 6. 编译、烧录和主机连通检查

先运行不接硬件的测试：

```bash
cd /home/w/project/rs00_fk743_test
cmake -S . -B build/host
cmake --build build/host --parallel
ctest --test-dir build/host --output-on-failure
```

构建 STM32 固件：

```bash
cd /home/w/project/rs00_fk743_test/firmware/RS00FK743
export PATH=/home/w/.local/share/stm32cube/bundles/gnu-tools-for-stm32/14.3.1+st.2/bin:/home/w/.local/share/stm32cube/bundles/ninja/1.13.2+st.1/bin:$PATH
cmake --preset Debug
cmake --build --preset Debug
```

烧录 `firmware/RS00FK743/build/Debug/RS00FK743.elf`。接上 USB-TTL 后先只查询：

```bash
cd /home/w/project/rs00_fk743_test
python3 host/control_tool.py status
python3 host/control_tool.py system-health
python3 host/control_tool.py drive-feedback all
python3 host/control_tool.py rs00-query-uid all
python3 host/control_tool.py rs00-read-position
```

若串口不是默认设备，把全局参数放在子命令前：

```bash
python3 host/control_tool.py --port /dev/ttyUSB0 status
```

也可以打开只读面板：

```bash
python3 host/chassis_dashboard.py --port /dev/ttyUSB0
```

面板、`control_tool.py`、ROS 串口桥和串口助手不能同时打开同一串口。

## 7. RS00 四轮标定

### 7.1 记录身份

执行 `rs00-query-uid all`，按 FL/FR/RL/RR 抄下四个 8 字节 UID。把它们填到
`firmware/App/Inc/steering_config.h` 的 `STEERING_EXPECTED_UIDS`。不要仅凭 CAN ID
猜 UID，也不要互换轮位。

### 7.2 小步确认方向和零位

保持架空并准备切断动力：

```bash
python3 host/control_tool.py steering-enable
python3 host/control_tool.py status
```

必须确认转向状态进入 `READY`。未标定固件只允许目标相对上一目标每次变化不超过
0.01 rad，建议实际每步使用 0.5°。先用 `rs00-read-position` 记录当前四轮角度；
然后用 `steering-set-each FL FR RL RR`，每次只改变一个轮，其他三个保持上一数值：

```bash
# 仅为命令格式示例；数字必须从当前读数开始，每步最多约 0.5°
python3 host/control_tool.py steering-set-each 30.5 40.0 50.0 60.0
python3 host/control_tool.py steering-set-each 31.0 40.0 50.0 60.0
```

对每个轮记录：

- 命令增加时，底盘定义角度是增加还是减少，据此得到 `direction_sign`；
- 轮子朝车体前方的位置定义为逻辑 0°，据此求 `zero_offset`；
- 本车转向机构已解除 360°机械限制，因此保持
  `STEERING_ENABLE_MECHANICAL_LIMIT_CHECK=0`，不再寻找不存在的机械端点；
- 全程监控电流、温度和机构干涉，任何异常立即实体断动力。

把结果按 FL/FR/RL/RR 顺序填入：

```c
#define STEERING_DIRECTION_SIGNS      { /* 四个 +1.0f 或 -1.0f */ }
#define STEERING_ZERO_OFFSETS_RAD      { /* 四个弧度 */ }
#define STEERING_ENABLE_MECHANICAL_LIMIT_CHECK 0
```

完成后先重新构建、架空复测四轮正方向和零位。确认无误才同时改为：

```c
#define STEERING_ENFORCE_UID_CHECK      1
#define STEERING_CALIBRATION_CONFIRMED  1
```

固件有编译期检查：已确认标定但未开启 UID 强制检查时会拒绝编译。

## 8. MINI 反馈和安装方向验收

先运行：

```bash
python3 host/control_tool.py drive-feedback all
```

四台都应满足：故障码 `0`、软件保护 `0`、总反馈年龄通常小于 100 ms、电压接近
实际 48 V、静止转速接近 0、电流和温度合理。安全必需反馈的有效位为故障 bit0、速度
bit1、电压 bit4、电流 bit5、温度 bit7，合计至少 `0xB3`；额外收到位置 bit8 是
正常的。

整车控制会自动处理转向后再驱动，不建议开放逐轮原始速度。如果确实需要核对四个
MINI 的安装正反方向，必须已经完成 RS00 标定、保持四轮架空，并临时改为：

```c
#define DRIVE_ALLOW_RAW_WHEEL_COMMANDS 1
```

重建烧录后，每次只给一个轮 `0.03 m/s`，其他轮为零，并立即停止：

```bash
python3 host/control_tool.py drive-set-all 0.03 0 0 0
python3 host/control_tool.py drive-stop
```

依次核对 FL/FR/RL/RR，记录正命令对应的车轮方向。完成后必须把
`DRIVE_ALLOW_RAW_WHEEL_COMMANDS` 恢复为 `0`、重新构建和烧录。正式运行只使用
`cmd_vel`/平移控制器，不能用原始逐轮命令绕过“先停轮、再转向、再行走”的顺序。

## 9. 当前软件安全阈值

| 检查 | 当前值 | 动作 |
|---|---:|---|
| 主机运动命令超时 | 300 ms | 行走目标降为零 |
| RS00 反馈超时 | 500 ms，再确认 100 ms | 全局故障、停止转向和行走 |
| RS00 温度 | 85 °C | 全局故障 |
| MINI 速度/电流反馈 | 250 ms | 建立过反馈后锁存故障 |
| MINI 故障反馈 | 500 ms | 建立过反馈后锁存故障 |
| MINI 温度反馈 | 750 ms | 建立过反馈后锁存故障 |
| MINI 电压反馈 | 2 s | 建立过反馈后锁存故障 |
| MINI 连续发送失败 | 500 ms | 锁存故障、撤流；仍早于驱动器 1000 ms 心跳超时 |
| MINI 电流绝对值 | 8 A | 锁存故障、撤流 |
| MINI 温度 | 85 °C | 锁存故障、撤流 |
| MINI 电压 | 36～60 V（48 V 标称母线） | 锁存故障、撤流 |
| MINI 反馈速度 | 3000 erpm | 锁存故障、撤流 |
| 停轮确认 | 四轮 ≤30 erpm 持续 100 ms | 才允许转向 |
| MCU IWDG | 标称约 2 s | 主循环卡死自动 MCU 复位 |

这些是保守初值，不是产品最终安全认证值。改阈值前要结合实测供电波动、电机热试验、
最大负载和制动能量验证。主机下发的 RS00 限速/限流只能降低当前编译安全上限
1 rad/s 和 2 A，不能从串口提高。

## 10. 必做故障注入验收

全部故障测试仍要架空，并准备实体断动力：

1. 建立正常 MINI 完整反馈后拔掉 MINI CAN：CAN bus-off 应立即锁存；若物理层未
   进入 bus-off，速度/电流反馈最迟约 250 ms 触发过期并撤流。
2. 正常低速运行时停止 Linux 发布命令：约 300 ms 后应停车。
3. 拔掉一台 RS00 CAN 支路：反馈超时后整车应进入故障并停止。
4. 发送 `emergency-stop`：应立即停止转向并撤去 MINI 电流；恢复必须显式执行
   `clear-fault-restart` 并重新走初始化，不能自行恢复运动。
5. 发送 `system-reset`：应先收到已接受复位 ACK，电机先停止，然后 MCU 重启；
   `system-health` 的启动/复位记录应更新。
6. 用调试构建临时制造主循环不喂狗，验证约 2 s 后 IWDG 复位；不要在有载运动时
   做这项试验。
7. 断开 Linux 串口、短时断开每条 CAN、电源从正常范围缓慢接近边界，分别确认
   不会自动重新起动。恢复线路后必须人工清故障。

查询黑匣子：

```bash
python3 host/control_tool.py system-health
python3 host/control_tool.py event-log 8
```

Backup SRAM 要在板上 `VBT/VBAT` 接约 3 V 后备电源才可跨彻底掉电保留；未接时
仅能保留软复位/看门狗复位期间的数据。

## 11. 实体急停和外部看门狗的预留接线

当前没有可靠的空闲 GPIO 和主板接线资料，因此工程已保留接口但默认不占用引脚：

```c
// firmware/App/Inc/board_reliability_config.h
#define BOARD_EXTERNAL_WATCHDOG_ENABLE 0
#define BOARD_PHYSICAL_ESTOP_ENABLE    0
```

实体急停推荐结构：急停常闭回路直接控制安全继电器/接触器，切断驱动器 ENABLE 或
电机动力；其一组隔离辅助触点再接 STM32 GPIO（建议常闭、断线也判急停）。STM32
输入只用于更快发软件停止和记录事件，不能成为断动力的必要条件。

外部看门狗推荐使用独立 3.3 V 监控芯片，WDI 接一个确认空闲的 STM32 GPIO，复位
输出以开漏方式接 NRST，超时约 3 s；它应独立于 MCU 时钟。确定引脚后需要定义：

```c
#define BOARD_EXTERNAL_WATCHDOG_GPIO_PORT ...
#define BOARD_EXTERNAL_WATCHDOG_GPIO_PIN ...
#define BOARD_EXTERNAL_WATCHDOG_GPIO_CLOCK_ENABLE() ...

#define BOARD_PHYSICAL_ESTOP_GPIO_PORT ...
#define BOARD_PHYSICAL_ESTOP_GPIO_PIN ...
#define BOARD_PHYSICAL_ESTOP_GPIO_CLOCK_ENABLE() ...
```

然后才把对应 `ENABLE` 改为 1。请提供底板/载板原理图、已占用引脚表、急停模块和
外部看门狗芯片型号，我才能给出不会冲突的精确引脚与阻容接线。

## 12. 是否需要第二串口

目前不需要。增加第二串口不能让软件链路变成实体安全链路，反而会增加占用引脚、
连接器和多主机冲突。建议继续使用：

- USART1：唯一的 Linux 二进制控制/遥测/远程复位通道；
- SWD + RTT/ITM：开发日志，不与 USART1 二进制数据混流；
- 独立 GPIO + 安全继电器：实体急停状态；
- 独立 GPIO + 外部监控芯片：外部看门狗。

只有以后增加独立安全 MCU 或隔离维护口时再开第二串口；即使如此，安全 MCU 仍应
直接控制安全继电器，串口只交换带 CRC、序号和超时的状态，不能作为唯一急停路径。

## 13. 正式运行放行清单

- [ ] 两条 CAN 完全分开，各自断电测得约 60 Ω，波特率正确。
- [ ] MINI ID 为 5/6/7/8、发送 ID 等于接收 ID、心跳超时 1000 ms。
- [ ] 四台 RS00 UID 已记录且强制校验，方向、零偏、软限位已实测。
- [ ] `STEERING_CALIBRATION_CONFIRMED=1`，面板显示“整车运动锁：已解锁”。
- [ ] `DRIVE_ALLOW_RAW_WHEEL_COMMANDS=0`，所有自动使能仍保持关闭。
- [ ] MINI 四台故障码为 0、安全标志为 0、总反馈年龄通常小于 100 ms。
- [ ] 主机掉线、两条 CAN 掉线、急停、看门狗和远程复位测试均通过。
- [ ] 实体急停无需 STM32 或 Linux 正常运行即可断动力。
- [ ] 温升、满载、电源跌落和制动母线电压已经在最终机械系统上测试。
- [ ] 重新上电后默认静止，必须人工显式使能才可能运动。
