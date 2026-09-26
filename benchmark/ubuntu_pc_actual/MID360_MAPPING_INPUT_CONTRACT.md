# MID360 mapping input contract

This contract records the live stage-1 observation from 2026-09-17. It is not
a claim that a mapping algorithm has been validated.

| Stream | Input | Type | Frame now | Expected mapping frame |
|---|---|---|---|---|
| front raw | `/livox/lidar_192_168_1_116` | `sensor_msgs/PointCloud2` | `livox_frame` | `lidar_2d_front_scan` via existing relay |
| rear raw | `/livox/lidar_192_168_1_139` | `sensor_msgs/PointCloud2` | `livox_frame` | `lidar_2d_back_scan` via existing relay |
| front raw IMU | `/livox/imu_192_168_1_116` | `sensor_msgs/Imu` | `livox_frame` | source only; acceleration is in g |
| rear raw IMU | `/livox/imu_192_168_1_139` | `sensor_msgs/Imu` | `livox_frame` | source only; acceleration is in g |
| front normalized IMU | `/antbot/lidar/front_left/imu_raw_si` | `sensor_msgs/Imu` | `mid360_front_imu` | temporary primary LIO input candidate |
| rear normalized IMU | `/antbot/lidar/rear_right/imu_raw_si` | `sensor_msgs/Imu` | `mid360_rear_imu` | secondary stream; not first-LIO input |

Cloud schema is `x/y/z/intensity/tag/line/timestamp`, point step 26. The field
`timestamp` is a float64 absolute nanosecond value: the first point matched the
header timestamp in representative frames, and the last point was about
99.88--100.31 ms later. The relay changes only `header.frame_id`; it preserves
all fields and bytes. The existing fusion output is suitable for visualization
and obstacle preprocessing, not automatically for motion-compensated LIO.

The IMU relay converts Livox acceleration from g to m/s², leaves gyro in rad/s,
preserves the device stamp, marks orientation unknown, and assigns unique
frames. It does not publish an unmeasured IMU extrinsic.

The first LIO baseline shall use one cloud and one IMU only. The rear cloud may
be integrated only after the primary LIO provides a continuous pose and a
per-point deskew path is implemented. No merged cloud is accepted as a fake
single-LiDAR LIO input.
