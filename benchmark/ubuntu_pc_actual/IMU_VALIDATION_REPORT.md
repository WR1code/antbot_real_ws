# MID360S IMU validation (short live tests)

Date: 2026-09-17 20:36 and 20:49 CST  
Mode: real sensors, stage-1 driver + frame relay only; no SLAM/Nav2/RViz.  
Network: temporary NetworkManager profile carried both `10.42.0.1/24` and
`192.168.1.50/24`; Wi-Fi remained the default route. The temporary profile was
deleted and `eno1` returned to the original `shared` profile after the test.

## Observed interfaces

| Sensor | Topic | Count / 10 s | Rate | `frame_id` | Result |
|---|---|---:|---:|---|---|
| front cloud | `/livox/lidar_192_168_1_116` | 73 | 9.862 Hz | `livox_frame` | PASS |
| rear cloud | `/livox/lidar_192_168_1_139` | 73 | 9.863 Hz | `livox_frame` | PASS |
| front relay | `/antbot/lidar/front_left/points_raw_native` | 71 | 9.858 Hz | `lidar_2d_front_scan` | PASS |
| rear relay | `/antbot/lidar/rear_right/points_raw_native` | 71 | 10.000 Hz | `lidar_2d_back_scan` | PASS |
| front IMU | `/livox/imu_192_168_1_116` | 1000 probe cap | 200.034 Hz | `livox_frame` | PASS rate; frame needs normalization |
| rear IMU | `/livox/imu_192_168_1_139` | 1000 probe cap | 200.038 Hz | `livox_frame` | PASS rate; frame needs normalization |

## PointCloud2 contract

Both raw and relayed clouds carried `point_step=26` and these fields:

```text
x:0:FLOAT32, y:4:FLOAT32, z:8:FLOAT32, intensity:12:FLOAT32,
tag:16:UINT8, line:17:UINT8, timestamp:18:FLOAT64
```

No timestamp regression or duplicate header timestamp was observed in the 10 s
probe. A representative scan had header/first-point equality and a last-point
offset of 99.88--100.31 ms, so the per-point field is present and spans roughly
one 10 Hz scan. The field must still be mapped explicitly to the selected LIO's
expected unit; this report does not claim LIO compatibility.

## IMU values and protocol units

Representative raw stationary samples had acceleration norms of `0.979` and
`1.002` and gyro magnitudes near zero. The Livox protocol defines acceleration
in g and angular velocity in rad/s; driver 1.2.6 copies those values directly
into `sensor_msgs/Imu`. The raw ROS message is therefore not SI-compliant for
linear acceleration and does not provide a measured orientation.

An `antbot_dual_lidar/livox_imu_relay` adapter was added. It preserves the
device timestamp and angular velocity, multiplies acceleration by standard
gravity (`9.80665`), assigns a unique device frame, and sets
`orientation_covariance[0] = -1`. A second live probe observed:

| Output | Samples | Rate | Mean acceleration norm | Frame | Stamp result |
|---|---:|---:|---:|---|---|
| `/antbot/lidar/front_left/imu_raw_si` | 400 | 200.071 Hz | 9.7338 m/s² | `mid360_front_imu` | monotonic; 834/834 captured output stamps matched raw |
| `/antbot/lidar/rear_right/imu_raw_si` | 400 | 199.933 Hz | 9.7254 m/s² | `mid360_rear_imu` | monotonic; 834/834 captured output stamps matched raw |

The relay is enabled by default in `mid360_stage1.launch.py` and can be disabled
with `normalize_imu:=false`. No motion/sign test was performed, so axis/sign
validation remains pending.

## Primary IMU decision

Use `/antbot/lidar/front_left/imu_raw_si` as the temporary primary stream for
the `.116 -> front_left` test mapping. Do not feed both IMUs into a single LIO.
The mapping is not a permanent physical identity; confirm the external labels
before recording permanent calibration.

## Remaining blockers

- Normalized topic frames are unique. Mapping mode derives their TF from the
  unchanged robot-to-LiDAR mounting TF plus the official MID360S internal
  geometry (`LiDAR -> IMU = [0.011, 0.02329, -0.04412] m`, axes aligned).
- Axis/sign test (static gravity, slow left/right yaw) was not performed.
- FAST-LIO initialized with the front normalized IMU and produced stable
  short-test odometry. Standard `odom -> base_link` remains preflight-gated
  until the live `world -> base_link` workbench conflict is disabled.
