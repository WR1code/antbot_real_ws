# 假设与未确认项

- `Example_Rotary` 是 Isaac Sim 自带通用测试雷达，不代表 Livox MID-360 或任何
  真实硬件；量程覆盖为项目测试值。
- USD/URDF 外参已静态核对，前左/后右对角关系明确；尚未通过完整运动序列在 RViz
  中测量点云重合误差。
- 默认车体过滤 box 由 URDF collision 外包围加裕量推导。轮胎最外缘、动态附件或
  后续机械臂姿态可能超出 box，最终值需要用 raw 点云调整。
- Isaac Sim rc.7 的 GMO `scalar` 已作为 `float32 intensity` 字段通过 ROS 运行
  样本确认；但其 normalized 数值分布和与真实反射率的物理对应仍未标定。
- 现有稳定 frame 名包含 `lidar_2d_`。本阶段复用该名称以避免双 TF；语义不够理想，
  但 frame 对应的光学原点和 3D sensor 位姿一致。
- GUI 中的暂停、恢复、reset 和全部 12 项运动/遮挡工况仍需人工操作验证；未执行
  的项目不会记为通过。
- navigation 的 10 Hz 结论来自同负载短时 headless 采样，不外推为 GUI、录包或
  10 分钟稳定性结论。
- rc.7 本机 GMO 的 `timeOffsetNs` 可用，但 channel/emitter 返回 NONE；不从
  点序号猜测或伪造 ring。
- `imu_link` 的存在不等于仿真 IMU 已存在；在检测到有效 Prim/topic 前契约明确
  标记 IMU unavailable。
- 极坐标回波桶反映场景可观察回波，不等同于射线真值盲区。
# Phase 2A 补充（2026-07-26）

- `imu_link` 的现有外参 `base_link -> imu_link = [0, 0, 0.250], rpy=[0,0,0]`
  合理且刚性，不修改原始 URDF。
- Isaac experimental physics IMU 的 `read_gravity=True` 为原始话题保留重力；
  本机静止实测约 `+9.81 m/s²` 沿 imu +Z。
- realistic profile 只实现固定 bias 与白噪声；当前接口没有被项目实现为 bias
  random walk，因此不声称具备该模型。
- GMO 头文件和运行消息确认 `time_offset_ns` 是 INT32 且相对 GMO timestamp；
  A/C/D 的唯一锚点语义不进入正常导航契约，统一标记为
  `DEFERRED_TO_REAL_HARDWARE_OR_LIO_RESEARCH`。最后一次几何 sweep 只保留为
  实验观测，不再扩大研究。
- 两颗辅助雷达 IMU 是可选观测接口，不代表三 IMU 融合。
