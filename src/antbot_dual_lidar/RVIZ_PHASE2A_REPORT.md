# Phase 2A RViz2 人工验收

状态：`GUI_VISUAL_CHECK=PENDING`。

自动确认：

- `/antbot/lidar/front_left/points_lio`：
  `sensor_msgs/msg/PointCloud2`，frame `lidar_2d_front_scan`；
- `/antbot/lidar/rear_right/points_lio`：
  `sensor_msgs/msg/PointCloud2`，frame `lidar_2d_back_scan`；
- 两路 Header 仿真频率 10.000 Hz，BEST_EFFORT/VOLATILE；
- `base_link` 到两 frame 的 TF 存在，后雷达 yaw 为 π。

启动：

```bash
source /opt/ros/jazzy/setup.bash
source /home/w/project/antbot/ros2_ws/install/setup.bash
rviz2 -d /home/w/project/antbot/ros2_ws/src/antbot_dual_lidar/config/rviz/dual_lidar_phase2a.rviz
```

配置含 RobotModel、TF、红色前雷达、蓝色后雷达；Fixed Frame 为 `base_link`，
两路 PointCloud2 使用 Best Effort/Volatile，Decay Time 0.3 s。

人工通过标准：分别关闭一路时剩余颜色与对应安装方向一致；同时开启时同一墙面
重合且后雷达不镜像；车体运动时点云不固定在传感器错误侧。任一 frame 缺失、后云
反向、颜色话题对调或点云无限累积均判失败。本轮未肉眼操作 GUI，不能写通过。

## Native 分层配置更新

RViz 配置现已明确分离：

- 红色前左、蓝色后右：`points_raw_native`，默认开启；
- 黄/青：`points_deskew_truth`，默认关闭；
- 紫/浅青：`points_world_reference`，默认关闭。

自动检查确认 native 两路 frame 正确，后雷达外参 yaw=π；truth world 的 frame
为 `odom`，不会与 native 同名。由于本轮在 headless 环境执行，墙面变薄、左右旋
形变方向、reset 后恢复和后雷达无镜像仍未由人眼确认：
`GUI_VISUAL_CHECK=PENDING`。
