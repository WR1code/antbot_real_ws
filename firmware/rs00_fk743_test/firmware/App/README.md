# App 层接入说明

当前固件使用 `steering_controller.c` 控制四台 RS00 转向电机。把 `App/Inc`
加入头文件路径，并编译：

```text
App/Src/rs00_protocol.c
App/Src/rs00_stm32_fdcan.c
App/Src/steering_controller.c
App/Src/mini_drive_protocol.c
App/Src/drive_controller.c
App/Src/chassis_direction.c
App/Src/chassis_translation_controller.c
App/Src/host_cmd_vel_protocol.c
App/Src/host_cmd_vel_uart.c
```

在 CubeMX 外设初始化完成后：

```c
SteeringController_Init(&hfdcan1);
ChassisTranslation_Init();
mini_fdcan_start(&hfdcan2, DriveController_OnCanFrame);
HostCmdVelUart_Init(&huart1);
```

主循环：

```c
while (1) {
    SteeringController_Task();
    HostCmdVelUart_Task();
    ChassisTranslation_Task();
}
```

FDCAN1 以 1 Mbit/s 承载 RS00 扩展帧，FDCAN2 以 500 kbit/s 承载 MINI 标准
帧；FIFO0 回调只把合法 RS00 扩展数据帧交给转向回调，MINI 标准回复当前保留
给后续反馈解析：

```c
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t flags)
{
    if ((flags & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U) {
        rs00_fdcan_on_rx_fifo0(hfdcan);
    }
}
```

所有电机 ID、安全参数和校准占位值位于 `steering_config.h`。默认不会自动
使能。详细流程见 `docs/RS00_STEERING_STARTUP.md`。

MINI 行走参数位于 `drive_config.h`。行走控制会发送标准帧心跳、速度环
加减速度、速度目标，急停发送零电流。CAN ID、轮径、减速比和极对数必须按
实机修改，不能直接把文档示例值当成产品固定值。

`rs00_app.c/.h` 是早期单电机速度测试的保留源码，不再加入实际固件构建。
