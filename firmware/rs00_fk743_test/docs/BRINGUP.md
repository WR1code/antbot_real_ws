# 首次上电说明（已迁移）

早期单电机 `0x7F` 速度测试已经退出实际固件构建。当前四台转向电机的完整
上电、使能、标定和故障排查流程见
[RS00_STEERING_STARTUP.md](RS00_STEERING_STARTUP.md)。

当前固定映射为左上/FL=`RS00 1, MINI 5`、右上/FR=`2,6`、
左下/RL=`3,7`、右下/RR=`4,8`；RS00 使用 FDCAN1 1 Mbit/s，MINI 使用独立
FDCAN2 500 kbit/s。

不要继续使用旧的 `g_rs00_command` 调试命令。
