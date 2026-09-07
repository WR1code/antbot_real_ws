# RS00 四轮转向协议测试向量

以下均为 STM32 FDCAN1 原始 29 位扩展数据帧、Classic CAN、1 Mbit/s、DLC=8。
四轮物理顺序固定为左上/FL=`0x01`、右上/FR=`0x02`、左下/RL=`0x03`、
右下/RR=`0x04`，主机 ID 为 `0xFD`。Data
中不包含 USB-CAN 串口适配器的 `41 54` 或 `0D 0A`。

| 操作 | 扩展 ID | Data（十六进制） |
|---|---:|---|
| 查询电机 1 | `0000FD01` | `00 00 00 00 00 00 00 00` |
| 停止电机 4 | `0400FD04` | `00 00 00 00 00 00 00 00` |
| 使能电机 2 | `0300FD02` | `00 00 00 00 00 00 00 00` |
| 电机 3 写 `run_mode=5` | `1200FD03` | `05 70 00 00 05 00 00 00` |
| 电机 1 读 `mechPos` | `1100FD01` | `19 70 00 00 00 00 00 00` |
| 写 `limit_spd=1.0f` | `1200FDxx` | `17 70 00 00 00 00 80 3F` |
| 写 `limit_cur=2.0f` | `1200FDxx` | `18 70 00 00 00 00 00 40` |
| 写 `canTimeout=10000` | `1200FDxx` | `28 70 00 00 10 27 00 00` |
| 原样预装 `loc_ref=0.2f` | `1200FDxx` | `16 70 00 00 CD CC 4C 3E` |

参数读取成功返回：

```text
ID   110001FD
Data 19 70 00 00 CD CC 4C 3E
```

应解析为类型 `0x11`、result `0`、来源电机 `0x01`、主机 `0xFD`、
index `0x7019`、值约 `0.2f`。若 ID 的 bit23..16 为 `1`，读取失败。

状态反馈示例：

```text
ID   028001FD
```

应解析为类型 `0x02`、运行态 `2`、fault bits `0`、来源电机 `0x01`、
主机 `0xFD`。

上述向量由 `tests/test_rs00_protocol.c` 自动检查：

```bash
cmake -S . -B build/host
cmake --build build/host --parallel
ctest --test-dir build/host --output-on-failure
```

`steering_controller` 测试还会用模拟 FDCAN 完整走过四轮初始化，确认默认
停在 ARMED、显式使能后才 READY，以及越界触发四台停止。全部测试均只
处理内存中的模拟帧，不会发送真实 CAN 指令。
