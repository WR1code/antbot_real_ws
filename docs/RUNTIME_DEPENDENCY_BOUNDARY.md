# Runtime dependency boundary

Audit date: 2026-09-18

## Project-local boundary

The one-click dual-arm/chassis/MID360 program now resolves all project-owned
runtime inputs below this workspace root:

```text
/home/w/project/antbot_real_ws
```

In particular, mapping no longer sources or links against
`/home/w/project/odas/.deps`.

| Component | Local source/runtime location |
|---|---|
| Livox ROS Driver 2 1.2.6 | `third_party/livox_ros_driver2` |
| Livox SDK2 1.3.1 | `third_party/livox-sdk2` |
| FAST-LIO ROS 2 | `third_party/FAST_LIO_ROS2` |
| Generated vendor overlay | `third_party/install` |
| Main AntBot packages | `src`, `install` |
| Dual-arm packages | `dual_arm_ws/src`, `dual_arm_ws/install` |
| Models/calibrations/motions/zones | `dual_arm_ws/resources` |
| MID360 driver JSON | `src/antbot_mapping/config/dual_mid360_actual.json` |
| Mapping output | `artifacts/maps` unless changed in RViz |

The MID360 runtime JSON has one canonical installed source in
`antbot_mapping`. The stage-1 fusion launch resolves that same package resource
instead of keeping another hard-coded copy.

## Build and activation

`build_dual_arm.sh` first calls
`scripts/build_local_mapping_dependencies.sh`, then builds the main and
dual-arm overlays. `activate.sh` sources overlays in this order:

1. `/opt/ros/jazzy`
2. `third_party/install`
3. `install`
4. `dual_arm_ws/install`

The Livox SDK runtime library path is
`third_party/livox-sdk2/lib`.

Run the repeatable audit with:

```bash
./scripts/audit_runtime_boundaries.sh
```

The audit fails if a key ROS package resolves outside this workspace, a
mapping binary has an unresolved/non-system external library, or the active
environment contains one of the former external project prefixes.

## Verified result

The following packages resolve inside this workspace:

- `livox_ros_driver2`
- `fast_lio`
- `antbot_mapping`
- `antbot_mid360_fusion`
- `antbot_real_bringup`
- `robotcar_navigation`
- `rebot_xbox_hardware`

The driver shared library resolves `liblivox_lidar_sdk_shared.so` from
`third_party/livox-sdk2/lib`. A file-access trace of
`start_dual_arm.sh --show-args` found no access to the former `odas`,
`rebotarm`, `piperh`, or `antbot` project workspaces.

## Allowed external system boundary

This workspace is self-contained for project files, not a containerized copy
of Ubuntu. These system-owned paths remain intentionally external:

- `/opt/ros/jazzy`: ROS 2 Jazzy and Nav2/MoveIt/RViz packages;
- `/usr`, `/lib`, `/lib64`: compiler/runtime, Qt, PCL, Python and system shared
  libraries;
- `/dev`, `/sys`, `/proc`: robot hardware and Linux process/device state;
- `/etc`: NetworkManager, udev and other system configuration;
- `/tmp`: transient ROS launch parameter files only.

Historical benchmark reports and upstream README/example files may contain old
absolute paths as recorded evidence or examples. They are not launch/config
inputs and are not read by the one-click runtime.
