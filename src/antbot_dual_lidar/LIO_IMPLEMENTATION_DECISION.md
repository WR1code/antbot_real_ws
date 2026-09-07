# Phase 2B/2C/3 LIO 实现路线

调查日期：2026-07-26。

| 实现 | ROS/构建 | 输入与时间 | 双雷达 | 许可证/风险 | 判断 |
|---|---|---|---|---|---|
| hku-mars FAST_LIO/FAST-LIO2 | ROS 1 catkin | Livox CustomMsg 或标准 PCL；direct 模式可无特征 ring | 单输入 | GPL-2.0；需完整 ROS 2/Jazzy 移植 | 算法参考，不作为首个集成 |
| Ericsii FAST_LIO_ROS2 | ROS 2（推荐 Humble）ament | 仍耦合 livox_ros_driver2；支持标准回调需适配字段/时间 | 单输入 | GPL-2.0；Jazzy/reset/QoS 工作量中高 | 备选参考 |
| MIT-SPARK spark-fast-lio | ROS 2 colcon | PointCloud2 + Imu，配置 lidar_type、scan_line、timestamp_unit；外参是 lidar→IMU | 单输入 | 继承 FAST-LIO GPL；接口最接近本项目 | Phase 2B 首选基线 |
| FAST_LIO_MULTI | ROS 1 catkin + livox_ros_driver | 多路 CustomMsg，bundle/async/adaptive | 原生多输入 | GPL-2.0；ROS 1/旧 driver/reset/QoS 移植量大 | 仅 Phase 3 设计参考 |

推荐顺序：

1. Phase 2B：spark-fast-lio 的前左单雷达 + 唯一主 IMU
   `/antbot/imu/data`，编写薄 PointCloud2 时间单位适配，先验证无 ring direct 模式。
2. Phase 2C：只换后右雷达和对应外参，做对称回归。
3. Phase 3：在两个单雷达基线都通过后，评估 FAST_LIO_MULTI 的 async/bundle
   设计并原生移植 ROS 2，不把两云或三 IMU交错伪装成单传感器。

开始 Phase 2B 前必须先锁定 Header/offset 公式并完成运动数据集；否则任何算法
结果都无法区分时间错误与估计器问题。

上游：

- https://github.com/hku-mars/FAST_LIO
- https://github.com/Ericsii/FAST_LIO_ROS2
- https://github.com/MIT-SPARK/spark-fast-lio
- https://github.com/engcang/FAST_LIO_MULTI
