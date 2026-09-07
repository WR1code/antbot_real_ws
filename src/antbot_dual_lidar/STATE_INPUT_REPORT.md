# IMU 与里程计输入报告

项目 URDF 有 `imu_link`，相对 `base_link` 为 xyz `[0,0,0.250]`、rpy
`[0,0,0]`，但当前派生 Isaac 场景没有已验证的 IMU sensor Prim，也没有
`/antbot/imu/data` 仿真 publisher。仓库中的 `antbot_imu` 面向真实硬件，不在
仿真中启动，不能冒充仿真 IMU。故当前无 orientation、angular velocity、
linear acceleration、covariance、频率或与 `/clock` 同步的可用 IMU 数据。

实际 runner 的 `/odom` 为仿真时间，`frame_id=odom`、
`child_frame_id=base_link`，包含 ground-truth pose、计算得到的 linear/angular
twist；pose covariance 对角为 `1e-5`、twist 为 `1e-4`，并发布
`odom -> base_link` TF。它可用于回归、初值或对比，但不应在评估 SLAM 精度时
当作未知真值输入。

结论：现有 odom 接口可用；现有 IMU 不可用，因而当前数据不支持严谨的旋转/平移
去畸变。下一阶段需在新的派生 USD 中添加、标定和实测一个以 `imu_link` 为 frame、
使用 `/clock`、频率显著高于 10 Hz 的 IMU；TF 仍由唯一 URDF/
robot_state_publisher 权威发布。
