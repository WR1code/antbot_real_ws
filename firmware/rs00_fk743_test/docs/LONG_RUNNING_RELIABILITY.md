# STM32H743 长期运行可靠性

## 当前已启用

| 功能 | 实现 | 主机可见性 |
| --- | --- | --- |
| 程序卡死自动复位 | IWDG1，标称 2 秒；仅完整主循环结束后喂狗 | 启动次数、IWDG 复位累计 |
| Linux 主动复位 | USART1 `SYSTEM_RESET` + CRC + `RST!` 密钥；先急停、回 ACK、再复位 | `control_tool.py system-reset` |
| 复位黑匣子 | 4 KB Backup SRAM 保存 RCC 复位标志、启动/看门狗/异常计数 | 仪表盘系统状态区 |
| CPU 异常现场 | HardFault、MemManage、BusFault、UsageFault 保存堆栈寄存器与 SCB 故障寄存器后复位 | PC、CFSR；CLI 可查看两页详情 |
| 运行健康 | DWT 统计主循环本次/平均/最大耗时；4 KB 主栈水位 | 仪表盘显示 µs 和最小空闲字节 |
| 固件身份 | 版本 `1.0.0`、Git 8 位哈希、转向/行走配置 SHA-256 前 32 位、能力位 | 仪表盘和 `system-health` |
| 故障历史 | Backup SRAM 32 条循环事件，含启动、看门狗、异常、远程复位、欠压和 Error Handler | `event-log 1..32`，仪表盘显示最新一条 |
| 供电早期告警 | STM32 PVD 2.85 V 阈值，记录下降/恢复事件 | 仪表盘“正常/欠压” |
| 主机报警 | 遥测过期为橙色，固件故障为红色，正常为绿色 | Tkinter 面板 |

Backup SRAM 在 MCU 普通复位后保留。要在整板完全断电后仍保留历史，需要按核心板
资料给 `VBT/VBAT` 提供约 3 V 后备电源；没有后备电源时，断电丢失属于正常现象。
写入 Backup SRAM 不消耗内部 Flash 擦写寿命。

查询命令：

```bash
python3 host/control_tool.py system-health
python3 host/control_tool.py event-log 8
python3 host/chassis_dashboard.py --port /dev/ttyUSB0
```

## 已预留但默认关闭的硬件接口

`firmware/App/Inc/board_reliability_config.h` 已提供外部看门狗 WDI 和物理急停输入
的编译配置，`board_reliability_io.c` 已实现健康主循环边沿喂狗、急停状态变化记录及
急停动作。由于当前不知道空闲 GPIO、外部芯片电路和急停触点电气形式，两项默认
为 `0`，因此不会误占引脚。选定接线后只需配置宏并重新编译，不需要改控制逻辑。

物理急停不能只依赖 MCU GPIO。正式设备应让急停触点/安全继电器直接切断电机使能
或动力接触器，GPIO 仅作为状态反馈；这样即使 MCU、软件或供电故障，人员保护仍然
有效。外部看门狗应使用 3.3 V 兼容 WDI、独立时基、复位输出接 `NRST` 的器件，
超时建议约 3 秒，略长于内部 IWDG。

## 仍需实物参数才能安全启用

以下字段在协议中存在，但有效位为 0，GUI 不会把未知值显示成 0：

- 外部 5 V/电池电压 ADC：需要供电正常值、最大可能电压、分压电阻和空闲 ADC 引脚；
- MCU 内部温度：需要在 CubeMX 中加入 ADC3 并在实板校准，建议 85 °C 告警、
  95 °C 只做安全停机，避免“过温复位—再次启动”的循环；
- 外部看门狗和急停：需要确定 GPIO、电路原理图和有效电平；
- 掉电永久日志：板载 W25Q64 当前未被本固件占用，但必须先确认没有被显示屏、
  字库或其他应用使用，再划分日志区。

## 安全升级边界

当前固件仍从 `0x08000000` 单镜像启动，**尚未宣称具备自动回滚能力**，能力位中的
`SAFE_UPDATER` 保持关闭。不能在不知道升级传输方式和 Flash 使用情况时直接改
链接地址或 Option Bytes，否则一次错误配置就可能让板子无法正常启动。

推荐的下一阶段方案是不修改 Option Bytes：保留首个 128 KB sector 为不可远程
覆盖的最小 bootloader，内部 Flash 放 A/B 两个应用槽；USART1 接收带版本、长度、
SHA-256 和签名的镜像，写入非活动槽，校验后试启动。新应用必须在限定时间写入
“启动成功”；连续看门狗复位或未确认时 bootloader 回滚。实施前只需最终确认：

1. USART1 是否作为升级通道；
2. 板载 W25Q64 是否完全空闲；
3. 是否要求防止未授权固件（建议要求签名）；
4. 生产时能否通过 SWD 锁定 bootloader sector。

这些确认只影响升级和外接硬件，不影响本次已经启用的自动复位与健康监控。
