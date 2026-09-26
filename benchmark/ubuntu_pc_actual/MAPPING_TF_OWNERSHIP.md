# Mapping TF ownership

This table is the required mapping-mode contract. Each child has exactly one
authority. The current live workbench does not yet satisfy the table because it
still publishes `world -> base_link`.

| TF | Publisher | Mode | Frequency | Purpose |
|---|---|---|---:|---|
| `odom -> base_link` | `antbot_mapping/lio_odom_adapter` | dynamic | about 10 Hz | Convert FAST-LIO's IMU-body pose to robot base pose |
| `base_link -> lidar_2d_front_scan` | `antbot_robot_state_publisher` | static | transient-local | Existing measured front LiDAR installation |
| `base_link -> lidar_2d_back_scan` | `antbot_robot_state_publisher` | static | transient-local | Existing measured rear LiDAR installation |
| `lidar_2d_front_scan -> mid360_front_imu` | mapping launch static publisher | static | transient-local | MID360S manual: `[0.011, 0.02329, -0.04412] m`, axes aligned |
| `lidar_2d_back_scan -> mid360_rear_imu` | mapping launch static publisher | static | transient-local | Same internal MID360S factory geometry |
| `map -> odom` | future mapping/localization back end | dynamic | back-end dependent | Not published by the current FAST-LIO odometry baseline |

FAST-LIO publishes `/Odometry` with frames `odom -> mid360_front_imu`, but its
TF broadcaster is disabled. The adapter is the only intended broadcaster for
`odom -> base_link`. AMCL, slam_toolbox, another EKF, and the legacy
`world/map -> base_link` publishers must remain off in mapping mode.
