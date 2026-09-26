# 红色标记距离标定操作手册

本文用于验证 Orbbec Gemini 2 对红色平面标记的对齐深度读取和相机光学坐标系三维计算。这里的“标定”是距离测量验证，不是机械臂手眼标定。

## 快速启动：三个终端

按顺序打开三个终端。每条命令都只在对应终端执行一次。

### 终端 1：启动 RGBD 相机

先用第 6 节的方法确认相机是否已经运行。如果已经存在 `/camera/camera`，跳过此命令，不要启动第二个相机；如果相机尚未运行，执行：

```bash
cd /home/w/project/rebotarm/real_robot_ws && ./scripts/start_gemini2_rgbd.sh
```

保持终端 1 运行，等待彩色图、对齐深度图和 CameraInfo 话题出现。

### 终端 2：启动红色三维检测

```bash
source /opt/ros/jazzy/setup.bash && source /home/w/project/rebotarm/real_robot_ws/install/setup.bash && ros2 launch red_point_localizer red_point_detector.launch.py image_topic:=/camera/color/image_raw depth_topic:=/camera/depth/image_raw camera_info_topic:=/camera/color/camera_info color_mode:=red min_area:=500.0 depth_window_size:=7 min_valid_depth_count:=5 min_depth_m:=0.10 max_depth_m:=2.50 sync_slop_sec:=0.08 depth_scale_16u:=0.001 sample_csv_path:=/home/w/project/rebotarm/real_robot_ws/log/red_point_localizer/camera_point_samples.csv debug_image_path:=/home/w/project/rebotarm/real_robot_ws/log/red_point_localizer/rgbd_debug_latest.png
```

保持终端 2 运行。真正执行红色检测、深度读取和 XYZ 计算的是这个程序。

### 终端 3：打开调试图

```bash
source /opt/ros/jazzy/setup.bash && source /home/w/project/rebotarm/real_robot_ws/install/setup.bash && ros2 run rqt_image_view rqt_image_view /red_point/debug_image
```

终端 3 只负责显示检测结果，可以单独关闭，不影响相机和检测计算。首次使用前仍应继续阅读本文的安全规则、实时话题检查和卷尺验证步骤。

## 1. 范围

本流程验证：

```text
红色标记中心像素 (u,v)
+ 对齐到彩色图的深度
+ 彩色 CameraInfo
→ camera_color_optical_frame 下的 (X,Y,Z)，单位米
```

本流程不进行 TF、`base_link` 坐标转换、MoveIt 控制或机械臂运动，不修改 Orbbec 驱动、驱动 launch、udev 规则或 `/home/w/rebotarm_ros2`。

## 2. 安全规则

- 相机、检测器和查看器各只启动一个实例。
- 已有相机正常运行时，不要再次执行 `start_gemini2_view.sh` 或启动第二个 `gemini2.launch.py`。
- 不使用 `pkill ros2`、`killall` 或模糊匹配停止进程。
- 优先在启动进程的终端按 `Ctrl+C`，只停止该终端启动的程序。
- 相机可在检测测试结束后继续运行；只关闭本次检测节点。
- 不删除整个 `build`、`install` 或 `log` 目录。

## 3. 红色标记要求

推荐使用 A4 白纸中央的实心红色图案：

- 实心圆直径约 10 cm，或实心方块约 10 cm × 10 cm；
- 推荐颜色为 `RGB(255,0,0)` 或 `#FF0000`；
- 使用哑光纸，或将打印纸平整贴在硬纸板上；
- 图案中心不能留白，因为程序会读取中心周围的 7×7 深度窗口；
- 避免亮面反光、褶皱、弯曲、暗红、粉红、橙红、空心圆环和细线图案。

现场打印颜色可能偏洋红。当前节点的红色 HSV 高色相段已覆盖 160–179，可检测本次现场打印块；仍应通过调试图确认检测框只覆盖目标。

## 4. 三个终端的职责

| 终端 | 程序 | 作用 | 必需 |
|---|---|---|---|
| 1 | Gemini 2 RGBD 相机 | 发布彩色图、对齐深度和 CameraInfo | 是 |
| 2 | `red_point_detector` | 检测红色并计算二维、三维坐标 | 是 |
| 3 | `rqt_image_view` | 显示 `/red_point/debug_image` | 建议 |

调试图查看器只负责显示；真正检测和计算距离的是 `red_point_detector`。

## 5. 加载环境

每个新终端都先执行：

```bash
source /opt/ros/jazzy/setup.bash && source /home/w/project/rebotarm/real_robot_ws/install/setup.bash
```

## 6. 检查相机是否已经运行

```bash
source /opt/ros/jazzy/setup.bash && source /home/w/project/rebotarm/real_robot_ws/install/setup.bash && ros2 node list --no-daemon
```

如果存在 `/camera/camera`，先检查参数，不要重复启动相机：

```bash
source /opt/ros/jazzy/setup.bash && source /home/w/project/rebotarm/real_robot_ws/install/setup.bash && ros2 param get /camera/camera depth_registration --no-daemon
```

```bash
source /opt/ros/jazzy/setup.bash && source /home/w/project/rebotarm/real_robot_ws/install/setup.bash && ros2 param get /camera/camera enable_depth --no-daemon
```

```bash
source /opt/ros/jazzy/setup.bash && source /home/w/project/rebotarm/real_robot_ws/install/setup.bash && ros2 param get /camera/camera enable_color --no-daemon
```

三项应均为 `True`。如果没有相机节点，在终端 1 前台启动安全 RGBD 脚本：

```bash
cd /home/w/project/rebotarm/real_robot_ws && ./scripts/start_gemini2_rgbd.sh
```

该脚本启动参数为 `depth_registration:=true enable_depth:=true enable_color:=true`，并将 PID、PGID 和日志保存到 `/home/w/project/rebotarm/real_robot_ws/log/gemini2_rgbd/`。此终端保持运行。

## 7. 实时确认话题和元数据

列出在线话题，使用实时结果，不凭名称猜测：

```bash
source /opt/ros/jazzy/setup.bash && source /home/w/project/rebotarm/real_robot_ws/install/setup.bash && ros2 topic list -t --no-daemon
```

当前 Gemini 2 的预期实时发现结果为：

```text
/camera/color/image_raw       sensor_msgs/msg/Image
/camera/depth/image_raw       sensor_msgs/msg/Image
/camera/color/camera_info     sensor_msgs/msg/CameraInfo
```

检查端点和 QoS：

```bash
source /opt/ros/jazzy/setup.bash && source /home/w/project/rebotarm/real_robot_ws/install/setup.bash && ros2 topic info /camera/color/image_raw --verbose --no-daemon
```

```bash
source /opt/ros/jazzy/setup.bash && source /home/w/project/rebotarm/real_robot_ws/install/setup.bash && ros2 topic info /camera/depth/image_raw --verbose --no-daemon
```

```bash
source /opt/ros/jazzy/setup.bash && source /home/w/project/rebotarm/real_robot_ws/install/setup.bash && ros2 topic info /camera/color/camera_info --verbose --no-daemon
```

只读取元数据字段，不把深度 `data` 数组输出到终端：

```bash
source /opt/ros/jazzy/setup.bash && source /home/w/project/rebotarm/real_robot_ws/install/setup.bash && timeout 5s ros2 topic echo /camera/depth/image_raw --once --field encoding
```

```bash
source /opt/ros/jazzy/setup.bash && source /home/w/project/rebotarm/real_robot_ws/install/setup.bash && timeout 5s ros2 topic echo /camera/depth/image_raw --once --field width
```

```bash
source /opt/ros/jazzy/setup.bash && source /home/w/project/rebotarm/real_robot_ws/install/setup.bash && timeout 5s ros2 topic echo /camera/depth/image_raw --once --field height
```

本机已经验证的配置是：彩色和深度均为 1280×720，深度编码为 `16UC1`，三者 frame_id 均为 `camera_color_optical_frame`。如果现场结果改变，应以新的实时结果为准。

只有同时满足以下条件才继续：

- `depth_registration=true`；
- 彩色和深度图宽高相同；
- CameraInfo 宽高和彩色图相同；
- 彩色和深度持续发布；
- 同步时间差不超过 `sync_slop_sec`。

## 8. 启动红色三维检测

如果输出文件已经存在，先保留备份：

```bash
test ! -e /home/w/project/rebotarm/real_robot_ws/log/red_point_localizer/camera_point_samples.csv || cp -a /home/w/project/rebotarm/real_robot_ws/log/red_point_localizer/camera_point_samples.csv /home/w/project/rebotarm/real_robot_ws/log/red_point_localizer/camera_point_samples.csv.$(date +%Y%m%d_%H%M%S).bak
```

在终端 2 启动红色检测：

```bash
source /opt/ros/jazzy/setup.bash && source /home/w/project/rebotarm/real_robot_ws/install/setup.bash && ros2 launch red_point_localizer red_point_detector.launch.py image_topic:=/camera/color/image_raw depth_topic:=/camera/depth/image_raw camera_info_topic:=/camera/color/camera_info color_mode:=red min_area:=500.0 depth_window_size:=7 min_valid_depth_count:=5 min_depth_m:=0.10 max_depth_m:=2.50 sync_slop_sec:=0.08 depth_scale_16u:=0.001 sample_csv_path:=/home/w/project/rebotarm/real_robot_ws/log/red_point_localizer/camera_point_samples.csv debug_image_path:=/home/w/project/rebotarm/real_robot_ws/log/red_point_localizer/rgbd_debug_latest.png
```

看到 `Depth metadata` 日志后，核对：

- `encoding=16UC1` 或 `mono16` 时，原始深度乘 `0.001` 转换为米；
- `encoding=32FC1` 时，原值直接按米使用；
- 其他编码不应发布三维点；
- 目标在几十厘米处时，转换结果也应为零点几米，而不是几十米或几毫米。

## 9. 打开调试图

在终端 3 执行：

```bash
source /opt/ros/jazzy/setup.bash && source /home/w/project/rebotarm/real_robot_ws/install/setup.bash && ros2 run rqt_image_view rqt_image_view /red_point/debug_image
```

有效检测时，调试图应显示：

```text
color_mode=red
u、v、area
valid_depth
Z
X、Y、Z
delta
```

如果显示 `NO RED POINT`，检查打印颜色、光照和检测面积。如果显示 `DEPTH INVALID`，二维像素仍可发布，但不能把该帧用于距离标定。如果显示 `WAITING CAMERA INFO`，等待有效 CameraInfo，不能伪造 frame_id。

## 10. 查看二维和三维结果

查看二维像素：

```bash
source /opt/ros/jazzy/setup.bash && source /home/w/project/rebotarm/real_robot_ws/install/setup.bash && ros2 topic echo /red_point/pixel
```

查看三维相机坐标：

```bash
source /opt/ros/jazzy/setup.bash && source /home/w/project/rebotarm/real_robot_ws/install/setup.bash && ros2 topic echo /red_point/camera_point
```

坐标约定为：

```text
+X：图像右侧
+Y：图像下方
+Z：从相机向前
```

## 11. 卷尺距离验证

1. 将红色标记放在镜头正前方约 30–80 cm。
2. 通过调试图把红块中心尽量移动到图像中心附近；当前 1280×720 图像中心约为 `(640,360)`。
3. 保持纸张平整，并尽量使标记平面正对相机。
4. 用卷尺测量“相机外壳前表面到红色标记平面”的距离。
5. 保持标记不动，等待 CSV 新增至少 30 条稳定样本。

检查样本数：

```bash
wc -l /home/w/project/rebotarm/real_robot_ws/log/red_point_localizer/camera_point_samples.csv
```

CSV 第一行是表头，因此行数至少为 31 才表示已有 30 条样本。统计全部有效样本的 X、Y、Z 均值、总体标准差和范围：

```bash
awk -F, 'NR>1{n++;sx+=$4;sy+=$5;sz+=$6;sx2+=$4*$4;sy2+=$5*$5;sz2+=$6*$6;if(n==1||$4<minx)minx=$4;if(n==1||$4>maxx)maxx=$4;if(n==1||$5<miny)miny=$5;if(n==1||$5>maxy)maxy=$5;if(n==1||$6<minz)minz=$6;if(n==1||$6>maxz)maxz=$6}END{if(n<1){print "NO DATA";exit}printf "N=%d\nX mean=%.6f std=%.6f range=[%.6f,%.6f]\nY mean=%.6f std=%.6f range=[%.6f,%.6f]\nZ mean=%.6f std=%.6f range=[%.6f,%.6f]\n",n,sx/n,sqrt(sx2/n-(sx/n)^2),minx,maxx,sy/n,sqrt(sy2/n-(sy/n)^2),miny,maxy,sz/n,sqrt(sz2/n-(sz/n)^2),minz,maxz}' /home/w/project/rebotarm/real_robot_ws/log/red_point_localizer/camera_point_samples.csv
```

误差计算：

```text
卷尺距离米值 = 卷尺厘米值 ÷ 100
绝对误差 = |Z 平均值 - 卷尺距离米值|
```

相机深度坐标原点通常不在外壳前表面，因此允许少量固定偏差。如果误差达到数厘米以上，依次检查深度单位、深度对齐、深度窗口是否落在红色标记表面、纸张是否倾斜，以及相机坐标原点位置。

## 12. 方向合理性检查

每次移动后保持约 2 秒，并观察 `/red_point/camera_point`：

- 标记向画面右侧移动，X 应增大；
- 标记向画面下方移动，Y 应增大；
- 标记远离相机，Z 应增大；
- 标记靠近相机，Z 应减小。

该检查只移动红色纸张，不移动机械臂。

## 13. 保存材料

检测节点启用本文命令中的输出参数后，会保存：

```text
/home/w/project/rebotarm/real_robot_ws/log/red_point_localizer/rgbd_debug_latest.png
/home/w/project/rebotarm/real_robot_ws/log/red_point_localizer/camera_point_samples.csv
```

测试报告保存为：

```text
/home/w/project/rebotarm/real_robot_ws/log/red_point_localizer/rgbd_test_report.txt
```

报告至少记录卷尺距离、话题、frame_id、分辨率、深度编码、深度比例、相机内参、30 条样本统计、时间差和绝对误差。

## 14. 结束测试

在终端 2 按 `Ctrl+C`，只停止该终端启动的 `red_point_detector`。终端 3 的查看器可单独按 `Ctrl+C`。相机可以保持运行。

确认检测节点已经退出：

```bash
source /opt/ros/jazzy/setup.bash && source /home/w/project/rebotarm/real_robot_ws/install/setup.bash && ros2 node list --no-daemon | grep -x /red_point_detector || echo 'red_point_detector 已退出'
```

不要为了结束测试而执行 `pkill ros2`、`killall`，也不要关闭不属于本次测试的 ROS 2 节点。
