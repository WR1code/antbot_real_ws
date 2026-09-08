# UART 显式控制与查询命令

## 帧格式

显式控制帧固定 16 字节，与既有 12 字节 `cmd_vel` 共用 USART1（115200 8N1），
但通过版本字段区分：

| 偏移 | 长度 | 内容                                  |
| ---: | ---: | ------------------------------------- |
|    0 |    2 | `AA 55`                             |
|    2 |    1 | 版本`02`                            |
|    3 |    1 | 命令 ID                               |
|    4 |    1 | 循环序号                              |
|    5 |    1 | 有效载荷长度 0～8                     |
|    6 |    8 | 载荷，不足部分补 0                    |
|   14 |    2 | 字节 0～13 的 CRC16/CCITT-FALSE，小端 |

所有多字节载荷均为小端。角度使用 `int16 mrad`，轮速使用 `int16 mm/s`。
涉及四轮的载荷与轮索引始终按 `0=左上/FL`、`1=右上/FR`、`2=左下/RL`、
`3=右下/RR` 排列；对应 RS00 ID `1/2/3/4`、MINI ID `5/6/7/8`。

## 命令表

|     ID | 命令                                          | 载荷                                       |
| -----: | --------------------------------------------- | ------------------------------------------ |
| `01` | `QUERY_STATUS`                              | 无                                         |
| `02` | `QUERY_SYSTEM_BOOT`                         | 无；启动/复位/看门狗/异常计数              |
| `03` | `QUERY_SYSTEM_RUNTIME`                      | 无；循环耗时和主栈水位                      |
| `04` | `QUERY_FIRMWARE`                            | 无；版本/Git/配置指纹/能力位                |
| `05` | `QUERY_CRASH_REGISTERS`                     | 页 0/1；上次异常入栈寄存器                  |
| `06` | `QUERY_CRASH_FAULTS`                        | 页 0/1；SCB 故障寄存器                      |
| `07` | `QUERY_EVENT_LOG`                           | 0=最新，最大 31；持久事件                   |
| `08` | `QUERY_POWER`                               | 无；PVD 及可选 ADC/温度                     |
| `10` | `STEERING_ENABLE`                           | 无                                         |
| `11` | `STEERING_SET_ALL`                          | 一个`int16` 角度                         |
| `12` | `STEERING_SET_EACH`                         | FL/FR/RL/RR 四个`int16` 角度             |
| `13` | `STEERING_DISABLE` / RS00 停止              | 无                                         |
| `14` | `RS00_CLEAR_FAULT` 并重新初始化             | 无                                         |
| `15` | `RS00_QUERY_UID`                            | 轮索引 0～3；返回对应 8 字节 UID           |
| `16` | `RS00_READ_POSITION`                        | 无；位置随 ACK 返回                        |
| `17` | `RS00_SET_CSP` 并重新初始化、写入、读回校验 | 无                                         |
| `18` | `RS00_SET_LIMITS`                           | `uint16 mrad/s` + `uint16 mA`          |
| `20` | `DRIVE_SET_ACCELERATION`                    | `int32 erpm/s²`                         |
| `21` | `DRIVE_SET_DECELERATION`                    | `int32 erpm/s²`                         |
| `22` | `DRIVE_SET_ALL`                             | FL/FR/RL/RR 四个`int16 mm/s`             |
| `23` | `DRIVE_STOP`                                | 无                                         |
| `24` | `DRIVE_WITHDRAW_CURRENT`                    | 无                                         |
| `25` | `DRIVE_QUERY_FEEDBACK`                      | 0有效位、1故障码、2速度、3电流、4位置、5温度、6年龄、7安全标志、8电压 |
| `30` | `EMERGENCY_STOP`                            | 无；转向停止且行走撤流                     |
| `31` | `CLEAR_FAULT_RESTART`                       | 无；应用状态机重新初始化                   |
| `32` | `SYSTEM_RESET`                             | 固定 ASCII `RST!`；急停、回 ACK 后复位 MCU |

Bus-Off 是 FDCAN 控制器锁存故障，普通清故障命令不能替代 MCU 复位。

## 上位机用法

统一工具为 `host/control_tool.py`：

```bash
# 只查询，不改变目标
python3 host/control_tool.py status

# 转向使能并设置角度
python3 host/control_tool.py steering-enable
python3 host/control_tool.py steering-set-all 90
python3 host/control_tool.py steering-set-each 90 90 90 90
python3 host/control_tool.py steering-disable

# RS00 参数和 UID
python3 host/control_tool.py rs00-query-uid all
python3 host/control_tool.py rs00-read-position
python3 host/control_tool.py rs00-set-csp
python3 host/control_tool.py rs00-set-limits 1.0 2.0

# MINI 参数、速度和全部反馈
python3 host/control_tool.py drive-set-acceleration 364
python3 host/control_tool.py drive-set-deceleration 546
python3 host/control_tool.py drive-set-all 0.03 0.03 0.03 0.03
python3 host/control_tool.py drive-stop
python3 host/control_tool.py drive-withdraw-current
python3 host/control_tool.py drive-feedback all

# 全系统安全命令
python3 host/control_tool.py emergency-stop
python3 host/control_tool.py clear-fault-restart
python3 host/control_tool.py system-reset
python3 host/control_tool.py system-health
python3 host/control_tool.py event-log 8
```

首次动作必须架空四轮。`STEERING_SET_*` 只有在显式使能并进入 READY 后才接受；
未确认标定时，每条直接转向命令相对上一目标不得超过 0.01 rad，整车运动保持
锁定。`DRIVE_SET_ALL` 的非零原始逐轮命令在正式构建中默认禁用，零值停车始终
允许；正常运动必须使用 `cmd_vel`，由平移控制器执行先停轮、转向、再行走。
底盘逻辑转向角使用 0～180°并通过行走轮正反转覆盖 360°方向；RS00 机械位置
允许连续旋转，并选择离当前目标最近的等效圈数。配置加减速度或 RS00 CSP 参数时，固件先停止相关
执行器，再按状态机下发参数；主机只能降低 1 rad/s、2 A 的编译安全上限。

`system-reset` 与 `clear-fault-restart` 不同：前者调用 Cortex-M7 系统复位，等价于
MCU 级软件复位；后者只重新初始化应用状态机。复位命令要求合法 CRC 和固定的
`RST!` 载荷，固件会先向两类电机发送急停、返回 `ACK_ACCEPTED_RESET`，随后复位。

系统健康数据复用 ACK v3 的四个 `int32 detail_values`，由 `detail_type=10..19`
区分页。不存在的异常现场或未接线的 ADC 字段会清除对应 `detail_valid_mask` 位，
主机必须显示为不可用，不能解释为数值零。

接线、RS00 标定、MINI 参数和故障注入的完整顺序见
[安全接线、标定与长期运行验收手册](SAFETY_COMMISSIONING_AND_WIRING.md)。
