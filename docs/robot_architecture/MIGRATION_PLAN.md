# 渐进迁移路线

原审计日期：2026-09-15；Phase 0 当前源码复核日期：2026-09-26。静态源码审计；CONFIRMED 指实现/配置已发现，不代表真机在线。UNKNOWN / NEED_CONFIRMATION 不作为可执行配置。Phase 0 的改造前证据见 [Phase 0 刷新](PHASE0_REFRESH_2026-09-26.md)，本轮实施及离线验收后的状态见 [Phase 1 离线报告](PHASE1_OFFLINE_REPORT_2026-09-26.md)。

| Phase | 目标 | 需要修改的文件/实现 | 风险 | 是否影响现有功能 | 验收方法 |
|---|---|---|---|---|---|
| Phase 0 | 记录证据、现状及未知项 | 本目录所有文档；SOURCE_SCAN/source_inventory/interfaces.yaml | 低风险，仅新增文档 | 不影响当前功能 | 源码链接/接口统计可复查；清楚区分真机/候选/仿真 |
| Phase 1 | 先封安全与命令入口 | hardware_adapter.py _stream/_publish_driver；PiperRosNode命令入口；HardwareManager.connect/shutdown；旧Piper launch；operator_manager/bridge最终cmd入口 | 改变使能/停机语义，须台架分次验收 | 可能影响旧demo/手拖/自动回家，需保留明确受控兼容入口 | 无CAN真机运动的fake transport验证互斥/ESTOP分支，再按人工安全清单实测 |
| Phase 2 | 设备身份/环境与健康标准化 | rebot SDK _make_controller/hardware_config；activate；pressure_serial_bridge；ros_publishers；bridge状态适配器；设备配置新文件 | 错误transport/设备映射会使启动失败 | 默认行为经确认后分步迁移 | 拔插/错误设备/无样本/部分电机失效都报blocker；不自动enable |
| Phase 3 | 跨资源ownership与writer强制租约 | 新增robot_supervisor ResourceManager/Action proxy；active_arm_manager/arm_ownership；operator_manager；两arm最终writer | 竞争时序/租约失效处理 | 旧客户端经代理接入；手动入口maintenance独占 | 并发acquire仅一个成功；client crash/旧epoch/driver重启拒绝late命令；stream与Action互斥 |
| Phase 4 | Supervisor能力READY/模式切换基础 | 新增health_adapter/ChangeMode Action；base/arm状态适配器；safe/stop proof | 误判READY风险 | 默认STANDBY，继续保留人工enable | 断开任一必需模块撤READY/租约；取消未终止/停止未知不授新owner |
| Phase 5 | 整机Bringup与部署生命周期 | antbot_real_bringup robot.launch.py包装现有launch；dual_arm_hardware拆可选UI组合；必要OS CAN/网络服务与设备配置 | 重复启动/overlay/respawn联动 | 保留start_dual_arm兼容入口转新包装 | 一命令离线可进STANDBY显示blockers；重启无重复串口/joy/web；安全有序退出 |
| Phase 6 | 真实odom/导航及预接触任务联动 | 新增轮式odom路径；navigation/config/real；operator_step2/waypoint_navigation；PiperPulseAlign/PulseApproach task Action | 最高实机运动风险；里程计精度、TF、刹车 | 导航直到验收前保持关闭 | 测平移/旋转odom及TF、导航取消停稳、BASE锁后臂任务、回撤成功才恢复导航 |
| Phase 7 | 故障恢复与维护命名收敛 | Supervisor恢复策略；硬件状态/ESTOP反馈；ROS命名逐链迁移；两份pulse归一计划 | 自动恢复可能产生意外运动 | 只恢复到STANDBY，不重放旧任务 | UART/CAN bus-off/单电机掉线/camera/TF/pressure timeout/ESTOP人工恢复全场景验收 |

优先完成Phase1中的Piper final adapter Action/Servo互斥与旁路清点，并单独复核reBot自动使能/退出回家。先让“谁能发命令”可验证，再做上层任务调度；不要先接通Nav2。设备身份修正前核实MotorBridge协议/USB分配。文档Phase不授权执行任何硬件测试或修改标定。

Phase6的odom可以先在隔离台架并行开发，但自动navigation/pulse切换必须等待ownership/stop proof验收，不能用虚拟TF补齐导航。
