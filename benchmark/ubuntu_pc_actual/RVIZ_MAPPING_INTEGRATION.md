# RViz-controlled MID360 mapping integration

Date: 2026-09-17

## Result

The dual-arm/chassis one-click workbench now starts in mapping-compatible TF
mode by default, while the Livox driver and FAST-LIO remain stopped until the
operator presses `开始建图` in the existing RViz Vehicle Status panel.

The RViz action now supervises this chain:

```text
/antbot/mapping/set_enabled
  -> temporary eno1 active-device addresses
     10.42.0.1/24 + 192.168.1.50/24
  -> antbot_mapping.launch.py preflight
  -> Livox CustomMsg + normalized front IMU
  -> FAST-LIO deskew/LIO
  -> /Odometry
  -> antbot_lio_odom_adapter
  -> /odom and odom -> base_link
  -> /cloud_registered displayed by unified RViz
```

`保存当前地图` calls FAST-LIO `/map_save` and targets:

```text
artifacts/maps/home_01/mapping_runs/current/map.pcd
```

This is a working PCD, not a formal Nav2 occupancy map.

The same panel provides `选择保存路径…`. It publishes the selected path to
`/antbot/mapping/output_prefix`; the operator manager normalizes the `.pcd` or
`.yaml` suffix and reports the effective path back in
`/antbot/operator_ui_status`. Path changes are disabled and rejected while a
mapping child is running.

## TF ownership

- Before mapping starts, the operator manager publishes an identity dynamic
  `odom -> base_link` placeholder solely to keep the unified RViz usable.
- As soon as the mapping child is started, the placeholder stops before the
  preflight delay elapses.
- During LIO, `antbot_lio_odom_adapter` is the sole `odom -> base_link`
  publisher.
- In mapping mode the legacy `world -> base_link` and `world -> map` static
  publishers are disabled.
- FAST-LIO's own TF broadcaster remains disabled.

## Network ownership

The operator manager changes only the active NetworkManager device state. It
does not edit the persistent `Wired connection 1` profile. It refuses the
operation unless `eno1` still has `10.42.0.1` and the default route is not on
`eno1`. On stop/exit it reapplies the unchanged persistent profile.

The integration test ended with:

- `eno1`: only `10.42.0.1/24`
- default route: Wi-Fi `wlp0s20f3`
- no Livox/FAST-LIO test processes left running

## 2026-09-17 short test

The RViz-equivalent SetBool start and stop calls succeeded in an isolated ROS
domain. The driver, IMU relay, static sensor TF nodes and preflight were
started, and stopping restored the network state.

The preflight correctly denied FAST-LIO because neither configured MID360
produced packets during this test. The isolated test domain also intentionally
did not contain the main workbench robot-state publisher, so the three
base/sensor TF checks failed there. Consequently `/Odometry` and
`/cloud_registered` were not expected and Nav2 was not started.

## Navigation gate

The existing waypoint/navigation panel remains in the unified RViz, but real
Nav2 startup remains disabled. Current real Nav2 configuration still requires:

- `/scan_0_fixed` and `/scan_1_fixed` LaserScan observations;
- a saved 2D occupancy-grid YAML/image;
- AMCL as the sole `map -> odom` publisher.

FAST-LIO's PCD and `/cloud_registered` are not substituted for those contracts.
Before enabling autonomous navigation, add and validate a dedicated MID360 2D
projection/costmap path and a localization strategy. The LIO adapter may then
remain the sole `odom -> base_link` provider.

## Start command

```bash
cd /home/w/project/antbot_real_ws
./start_dual_arm.sh
```

Then use the unified RViz `建图与遥控` panel. To retain the legacy workbench
TF mode and old 2D mapping backend for a diagnostic session only:

```bash
ANTBOT_MAPPING_CONTROLS=false ./start_dual_arm.sh
```

The physical mapping `192.168.1.116 -> front_left` remains provisional and
must be independently confirmed before creating a formal map.
