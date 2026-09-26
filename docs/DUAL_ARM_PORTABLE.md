# 独立双臂启动工作空间

本目录包含原 `rebotarm/real_robot_ws` 启动所需的全部工作空间源码副本，保留原目录不动。
双臂包位于 `dual_arm_ws/src`，不依赖原工作空间的源码软链接，也不会覆盖底盘 `src/rebotarm_pulse`。
双臂 overlay 最后加载，使用的是这里保存的最新双臂版本。
Livox ROS Driver 2、Livox SDK2 和 FAST-LIO 也保存在根目录 `third_party`，
运行时不再读取其他项目工作区。

## 当前机器启动

```bash
cd /home/w/project/antbot_real_ws
bash start_dual_arm.sh
```

等价命令：

```bash
source activate.sh
ros2 launch rebot_xbox_hardware dual_arm_hardware.launch.py \
  piper_pulse_serial_port:=/dev/robot_serial
```

启动脚本接受其他 ROS launch 参数。它保留原 launch 的执行授权与安全配置；打包工作没有启动硬件或发送运动命令。

## 换路径或换机器

源码、模型、标定和动作文件随整个目录复制即可；**并非无系统依赖的二进制便携应用**。
目标系统需 Ubuntu 24.04 / ROS 2 Jazzy、colcon、rosdep、python3-venv、编译工具和硬件驱动（包括 `ros-jazzy-pinocchio`）。
相机 SDK 中的预编译库需要匹配 CPU 架构。串口权限、CAN 接口和设备 ID 仍需按目标机器配置。

1. 复制整个目录，不必带 `build`、`install`、`log`、`dual_arm_ws/build`、`dual_arm_ws/install`、`dual_arm_ws/log` 和两个 `.venv` 环境。
   已复制这些生成目录时，首次构建前将它们移至备份目录；CMake 缓存和 Python venv 不能跨路径复用。
2. 在新目录执行 `bash install_dual_arm_dependencies.sh`，需网络、rosdep 初始化及系统包安装权限。
3. 执行 `bash build_dual_arm.sh`。
4. 执行 `bash start_dual_arm.sh`。

只查看 launch 参数、不启动节点：`bash start_dual_arm.sh --show-args`。

## 随目录保存的数据

- `dual_arm_ws/resources/models/hand_landmarker.task`：现有手部模型原样复制。
- `dual_arm_ws/resources/calibrations/piperh_camera_eye_in_hand.calib`：当前有效标定原样复制，不含已拒绝标定。
- `dual_arm_ws/resources/motions`：当前示教动作及动作组。
- `dual_arm_ws/resources/zones`：当前用户禁入区。
- `dual_arm_ws/third_party/reBotArm_control_py`：控制器实际加载的第三方 SDK 源码及模型。
- `third_party/livox_ros_driver2`、`third_party/livox-sdk2`、
  `third_party/FAST_LIO_ROS2`：MID360/FAST-LIO 的本地固定版本；由
  `scripts/build_local_mapping_dependencies.sh` 构建到 `third_party/install`。
- `dual_arm_ws/resources/dm_device/x86_64/libdm_device.so`：当前 DM 驱动运行库；其他架构由依赖安装脚本下载匹配库到本目录内的 cache。
- `artifacts/maps/home_01`：底盘默认地图，路径由当前目录推导。
- 新运行日志位于 `dual_arm_ws/log`，部分节点仍使用原有用户日志目录。

标定只适用于原机械安装；移动相机、工具或改变安装后需重新标定，不能把这份标定直接当成新机器的有效标定。
模型/相机厂商文件的再分发许可需遵守上游条款。
