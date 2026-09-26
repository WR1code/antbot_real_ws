# red_point_localizer

ROS 2 Jazzy 彩色标记 RGBD 定位包。节点检测最大的 `red`、`green` 或 `cyan` 区域，保留二维像素和调试图发布，并从与彩色图对齐的深度及彩色 `CameraInfo` 计算光学坐标系下的米制三维点。

## 前提与编译

Gemini 2 必须以 `depth_registration:=true enable_depth:=true enable_color:=true` 启动。节点不会缩放不同尺寸的深度图，也不会把未对齐的深度当作对齐深度。

```bash
sudo apt update && sudo apt install -y ros-jazzy-cv-bridge ros-jazzy-message-filters ros-jazzy-rqt-image-view python3-opencv python3-numpy
```

```bash
cd /home/w/project/rebotarm/real_robot_ws && source /opt/ros/jazzy/setup.bash && colcon build --packages-select red_point_localizer --symlink-install
```

## 运行

先用 `ros2 topic list -t` 实时确认彩色图、对齐深度图和彩色 `CameraInfo` 话题，再将真实名称传给 launch：

```bash
source /opt/ros/jazzy/setup.bash && source /home/w/project/rebotarm/real_robot_ws/install/setup.bash && ros2 launch red_point_localizer red_point_detector.launch.py image_topic:=/camera/color/image_raw depth_topic:=/camera/depth/image_raw camera_info_topic:=/camera/color/camera_info color_mode:=green min_area:=500.0 depth_window_size:=7
```

主要参数包括 `image_topic`、`depth_topic`、`camera_info_topic`、`debug_topic`、`pixel_topic`、`camera_point_topic`、`color_mode`、`min_area`、`depth_window_size`、`min_valid_depth_count`、`min_depth_m`、`max_depth_m`、`sync_slop_sec` 和 `depth_scale_16u`，都可从 launch 覆盖。

## 输出和深度单位

- `/red_point/pixel`：`PointStamped`，`x=u`、`y=v`、`z=0`，单位是像素。
- `/red_point/camera_point`：`PointStamped`，`x/y/z` 是彩色相机光学坐标系下的米制坐标，`+X` 向右、`+Y` 向下、`+Z` 向前。
- `/red_point/debug_image`：检测轮廓、像素、面积、有效深度数、XYZ 和彩色/深度时间差。

## 实验室三色分拣准备

三色演示可以同时启动三个相互隔离的检测实例：

```bash
source /home/w/project/rebotarm/real_robot_ws/activate.sh
ros2 launch red_point_localizer multi_color_demo.launch.py
```

它分别发布 `/demo/color/{red,green,cyan}/camera_point`、`pixel` 和
`debug_image`。三个节点只负责观测，不会生成机械臂运动；编排器还必须检查目标
连续帧稳定性、消息时效、工作区范围和手眼标定状态。第一版固定工装演示只用颜色
选择预设料箱，不直接把相机坐标作为抓取坐标。

对于 `16UC1` 或 `mono16`，原始值乘 `depth_scale_16u`（默认 `0.001`）转换为米；对于 `32FC1`，数值直接按米读取。其他编码不会猜测单位，也不会发布三维点。节点在窗口中排除 0、NaN、Inf 和范围外深度，再对有效值取中位数。二维检测有效但深度无效时仍发布 `/red_point/pixel`，调试图显示 `DEPTH INVALID`，不发布 `/red_point/camera_point`。

## 距离验证

将平整绿色标记放在镜头前约 0.3--0.8 m，运行绿色模式并采集至少 30 条 `camera_point`。比较 Z 平均值与卷尺测得的相机前表面到标记距离；小固定偏差可能来自深度坐标原点不在外壳前表面。如果误差达到数厘米，检查深度单位、对齐状态、窗口是否落在标记表面，以及相机坐标原点。

```bash
source /opt/ros/jazzy/setup.bash && source /home/w/project/rebotarm/real_robot_ws/install/setup.bash && ros2 topic echo /red_point/pixel
```

```bash
source /opt/ros/jazzy/setup.bash && source /home/w/project/rebotarm/real_robot_ws/install/setup.bash && ros2 topic echo /red_point/camera_point
```

```bash
source /opt/ros/jazzy/setup.bash && source /home/w/project/rebotarm/real_robot_ws/install/setup.bash && ros2 run rqt_image_view rqt_image_view /red_point/debug_image
```
