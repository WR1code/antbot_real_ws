# AntBot MID360S mapping baseline

This package provides a fail-closed, single-primary-LiDAR FAST-LIO baseline. It
does not launch Nav2 and does not use the spatially merged cloud as an LIO
input. The primary input is Livox `CustomMsg`, whose per-point `offset_time` is
consumed by FAST-LIO for IMU deskew.

The validated mapping dependencies are vendored inside this workspace:

```text
third_party/livox_ros_driver2
third_party/livox-sdk2
third_party/FAST_LIO_ROS2
```

It is `Ericsii/FAST_LIO_ROS2`, branch `ros2`, pinned at commit
`2fffc570a25d0df172720bac034fbdb6a13d2162`. Local Jazzy integration changes
remove an unused `pcl_ros` dependency, declare the actually used `tf2_ros`, use
`SensorDataQoS` for IMU, and add parameters for output frame names and disabling
the upstream TF broadcaster. The Livox driver remains version 1.2.6. Build
both packages into the local `third_party/install` overlay with
`scripts/build_local_mapping_dependencies.sh`.

Source order:

```bash
source /opt/ros/jazzy/setup.bash
source /home/w/project/antbot_real_ws/third_party/install/setup.bash
source /home/w/project/antbot_real_ws/install/setup.bash
```

The launch requires `eno1` to retain `10.42.0.1/24` while also carrying
`192.168.1.50/24`; it never changes NetworkManager. It starts sensor processes,
runs preflight, and starts FAST-LIO plus the odometry adapter only on a complete
preflight pass. RViz is off by default.

```bash
ros2 launch antbot_mapping antbot_mapping.launch.py \
  front_ip:=192.168.1.116 rear_ip:=192.168.1.139 use_rviz:=false
```

The IP-to-position mapping remains provisional. Do not use this entry for a
formal map until external labels, yaw sign, and the workbench TF shutdown path
have been verified.

## Integrated dual-arm RViz workflow

`start_dual_arm.sh` enables the mapping-safe TF mode by default, but it does
not start the Livox driver or FAST-LIO. In the unified RViz window, use
`建图与遥控 -> 开始建图` to start this launch as a supervised child. The same
button path performs preflight and displays `/cloud_registered`; `保存当前地图`
calls FAST-LIO's `/map_save` service and writes
`artifacts/maps/home_01/mapping_runs/current/map.pcd`.

Use `选择保存路径…` before starting mapping to choose another `.pcd` file.
The selector is locked while mapping runs because FAST-LIO receives its output
path at process start. The chosen parent directory is created when mapping
starts.

For this integrated path only, the operator manager applies the temporary
active-device address set `10.42.0.1/24,192.168.1.50/24` with
`nmcli device modify`. It does not edit the persistent NetworkManager profile,
refuses to act if `eno1` owns the default route or has lost `10.42.0.1`, and
reapplies the unchanged persistent profile when mapping stops or the workbench
exits.

During the pre-mapping idle state, the manager publishes an identity
`odom -> base_link` placeholder so the unified RViz remains usable. It stops
that placeholder before spawning FAST-LIO; the LIO odometry adapter is then the
sole `odom -> base_link` publisher. Legacy `world -> base_link` and
`world -> map` publishers are disabled in this mode.

The waypoint/navigation panel remains available in the same RViz, but real
Nav2 startup is intentionally still disabled. The current real Nav2 contract
requires `/scan_0_fixed`, `/scan_1_fixed`, a saved 2D occupancy map, and AMCL;
FAST-LIO's PCD is not silently substituted for those inputs. Add and validate
the MID360-to-2D costmap/localization contract before enabling autonomous
motion.
