# PRE-MAPPING TF audit

Audit basis: live ROS graph before/after the short stage-1 sensor test and
repository launch/source inspection. No SLAM or Nav2 was started.

| Parent | Child | Current publisher | Static/dynamic | Mapping status |
|---|---|---|---|---|
| `base_link` | `lidar_2d_front_scan` | `antbot_robot_state_publisher` from URDF | static | unique, PASS |
| `base_link` | `lidar_2d_back_scan` | `antbot_robot_state_publisher` from URDF | static | unique, PASS |
| `base_link` | `mid360_front_imu` / `mid360_rear_imu` | mapping launch, chained through existing LiDAR TF | static | official internal MID360S geometry; physical IP identity still provisional |
| `odom` | `base_link` | `antbot_mapping/lio_odom_adapter` when preflight passes | dynamic, about 10 Hz | prepared; correctly withheld in current conflicting graph |
| `map` | `odom` | none (SLAM/AMCL stopped) | — | expected absent before mapping |
| `world` | `base_link` | `rebotarm_moveit_world_tf` in dual-arm hardware launch | static | conflict in mapping mode unless disabled |
| `world` | `map` | `integrated_world_to_map_tf` when chassis workbench is active | static | do not use in mapping TF tree |

The dual-arm launch now has `publish_world_to_base:=true` by default for
backward compatibility. A mapping-mode invocation must pass
`publish_world_to_base:=false`; this suppresses only the legacy MoveIt
`world -> base_link` transform and does not remove arm functionality. The
operator UI already has `publish_placeholder_pose:=false` in the dual-arm
parent path. Mapping must still run with a single robot-state publisher and
must not start AMCL, slam_toolbox, or an EKF that duplicates the selected LIO
TF owner.

The formal mapping entry was tested against the current workbench graph. All
sensor/time/dependency checks passed, then preflight detected direct
`world -> base_link` and terminated the launch before FAST-LIO or the odometry
adapter started. This is the intended fail-closed result, not a sensor failure.

Required final tree:

```text
map -> odom       (one mapping/SLAM owner)
odom -> base_link (one LIO or wheel/EKF owner)
base_link -> sensors (robot_state_publisher / normalized static TF)
```
