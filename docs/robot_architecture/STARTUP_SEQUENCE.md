# Startup Sequence

审计日期：2026-09-15。静态源码审计；CONFIRMED 指实现/配置已发现，不代表真机在线。UNKNOWN / NEED_CONFIRMATION 不作为可执行配置。详见 [扫描范围](SOURCE_SCAN.md)。

## 实际入口

`bash start_dual_arm.sh → source activate.sh → /opt/ros/jazzy/setup.bash → 当前仓库 third_party/install/local_setup.bash → root install/local_setup.bash → dual_arm_ws/install/local_setup.bash → ros2 launch rebot_xbox_hardware dual_arm_hardware.launch.py → instance_guard → stdout ready触发stack`。

证据：[`start_dual_arm.sh:1`](../../start_dual_arm.sh#L1)；[`activate.sh:1`](../../activate.sh#L1)；[`dual_arm_ws/src/rebot_xbox_hardware/launch/dual_arm_hardware.launch.py:646`](../../dual_arm_ws/src/rebot_xbox_hardware/launch/dual_arm_hardware.launch.py#L646)。guard ready只证明进程锁成功，不是硬件ready。

stack包括rebot driver/RSP/MoveIt/Servo、Piper hardware/RSP/MoveIt/Servo、静态TF、共享joy、选臂、gripper、禁区、双Teach、Piper工具/target observer、底盘operator、条件压力/web和RViz。parent `start_pulse=false`避免底盘子launch重复压力/web。参数use_teach/use_chassis/use_piper_pulse_observer等决定实例，详见接口矩阵。

## 真实依赖/门禁

| 层/节点 | 启动必要条件 | 执行任务前额外条件 | 当前如何等待 |
|---|---|---|---|
| ROS环境 | 两overlay已build、系统ROS Jazzy | Python3.12 ABI/外部SDK可用 | activate检查安装文件，不检查硬件 |
| Piper vendor/adapter | can接口online、权限、SDK | fresh feedback+人工使能；gravity另有模式/firmware检查 | 并行起/respawn2s；没有CAN健康管理服务 |
| reBot driver | 正确channel/model、MotorBridge库 | 反馈和模式、任务局部门禁 | connect即组enable/保持；没有统一初始化屏障 |
| H743 bridge | 对应串口；allow_disconnected可离线 | 人工确认+fresh ACK+MCU ready/fault-free | 1s重连；状态门禁，不按launch顺序判断 |
| MoveIt/Servo | URDF/SRDF/kinematics、joint_states、服务 | 新鲜状态、控制权 | 外部组件等待状态/任务service等待；无整机deadline |
| vision | camera Image/Depth/Info、模型、手眼标定 | fresh target/TF/质量判据 | 总入口不默认启动完整检测相机链 |
| pulse | pressure串口不同于arm、topic/zero、TF/规划/臂 | BASE停稳锁定、臂ownership、压力fresh | 压力冲突可跳过；局部task检查，缺base互锁 |
| navigation | scan/map、真实odom及TF | arms安全、BASE ownership | operator明确start_nav2=false；当前缺odom |
| 整机READY | 所需子系统在线/初始化/停稳/安全证据 | 按任务能力进一步收敛 | 未发现统一READY聚合 |

## OS/部署准备边界

[`install_dual_arm_dependencies.sh:1`](../../install_dual_arm_dependencies.sh#L1)创建.hardware-venv/.venv；activate将hardware site-packages加入PYTHONPATH，MERIDIAN_VISION_PYTHON单独选择视觉Python，MOTOR_DM_DEVICE_LIB按uname架构选择共享库。不能复制旧venv直接当可移植环境。build_dual_arm先root packages-up-to antbot_real_bringup，再dual全overlay；activate source local_setup避免绝对旧链。

Orbbec有99-obsensor-libusb.rules及安装脚本，但**本机已安装udev/systemd UNKNOWN**。仓库未发现生产整机.service，也未发现总入口负责Linux CAN ip-link初始化或网络初始化；设备can0准备可能在仓库外。未发现项目Docker/conda生产启动入口。README/check_hardware文档是检查说明，不是READY执行器。完整shell/launch/rules文件清单见source_inventory files。

## 分项入口及退出

scripts/start_base.sh、start_xbox.sh、start_xbox_dry_run.sh、start_antbot_operator.sh、start_pulse_web.sh、start_pulse_pressure.sh与总启动有重叠。start_xbox有人工ENABLE与最多多次sleep1+状态字符串轮询；不是无条件sleep宣布ready。总launch guard退出会触发Shutdown；real_base桥退出会触发Shutdown；vendor/adapter和RViz respawn。bridge.destroy_node发5次零帧；reBot driver.shutdown尝试safe_home后disable/disconnect。不存在已确认整机依赖逆序shutdown及ESTOP分支，退出不能作为急停操作。

## 推荐 Bringup

沿用antbot_real_bringup和现有硬件launch，新增robot.launch.py作为包装总入口：组合hardware/sensors（设备配置检查，不自动运动）、base（原real_base）、arms（原双臂驱动，不含重复UI）、perception（按需camera+vision）、navigation（odom验收后默认inactive）、supervisor。left/right_arm独立文件只有在物理映射和维护需要确认后才拆；不机械创建十个空文件。RViz/web作为operator可选层，退出UI不应隐式触发故障回家。初始化进程存在→硬件反馈健康→任务能力READY分开报告；故障时保持锁定且给出blockers。
