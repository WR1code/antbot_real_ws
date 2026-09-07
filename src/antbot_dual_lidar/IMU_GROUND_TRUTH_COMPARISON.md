# IMU 与 Ground Truth 短时对比

状态：真实七动作矩阵已执行。

静止短跑确认 IMU 和 `/antbot/ground_truth/odom` 使用仿真时间，重力沿 IMU +Z，
无重复或倒退。1 s、2 s、5 s 的角速度积分、重力补偿、ideal/realistic 角度误差
仍需要可控运动数据；本文件不以静止样本替代运动对比。

运行后将结果写入 `imu_ground_truth_metrics.csv`，至少包含窗口、profile、运动、
角度误差、重力补偿误差和 bias 影响。

左原地旋转：truth yaw `+1.12248 rad`，IMU mean wz `+0.21443 rad/s`；右原地
旋转：truth yaw `-1.23217 rad`，IMU mean wz `-0.23404 rad/s`。符号相反且与
truth 一致。前进 mean ax `+0.07109 m/s²`，后退 `-0.07241 m/s²`。

旋转积分误差：

| 动作 | 1 s | 2 s | 5 s |
|---|---:|---:|---:|
| 左转 | +0.001560 rad | -0.000823 rad | -0.004651 rad |
| 右转 | +0.001878 rad | +0.003797 rad | +0.011835 rad |

共采集 120 Hz IMU，时间戳回退 0、重复 0。完整原始结果：
`/home/w/project/antbot/artifacts/phase2a_imu_motion_results.yaml`。
