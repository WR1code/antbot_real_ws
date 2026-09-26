# Dual Livox MID360 crop / TF / fusion benchmark

This ROS 2 Jazzy C++ package removes the robot-facing horizontal sector in each
LiDAR's own frame, transforms the retained records to `base_link`, approximately
synchronizes the two streams, and concatenates them without discarding Livox
point fields.

## What was and was not verified in this workspace

At implementation time, `ros2 pkg prefix livox_ros_driver2` returned "Package
not found", and the live graph contained no LiDAR topic. Therefore the actual
on-robot PointCloud2 schema and TF tree are **not yet verified**.

Repository evidence in another AntBot bringup tree configures one
`livox_ros_driver2_node` with `xfer_format=0`, `multi_topic=1`, two devices at
`192.168.1.12` and `192.168.1.13`, and expected driver topics
`/livox/lidar_192_168_1_12` and `/livox/lidar_192_168_1_13`. The existing relay
in this workspace expects to republish those as:

- `/antbot/lidar/front_left/points_raw_native`, frame `lidar_2d_front_scan`
- `/antbot/lidar/rear_right/points_raw_native`, frame `lidar_2d_back_scan`

Those names are configuration evidence, not a live measurement. The driver
config also contains zero extrinsics, so it is not a valid source for the
physical installation transform.

Before a real benchmark, source the driver workspace, start the driver and
robot description, then capture the actual interfaces:

```bash
ros2 pkg prefix livox_ros_driver2
ros2 pkg executables livox_ros_driver2
ros2 topic list -t
ros2 topic info -v /livox/lidar_192_168_1_12
ros2 topic echo --once /livox/lidar_192_168_1_12 | sed -n '/header:/,/data:/p'
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo base_link lidar_2d_front_scan
ros2 run tf2_ros tf2_echo base_link lidar_2d_back_scan
```

The node logs the complete field name/offset/type/count list and `frame_id` on
the first message of each stream. Do not proceed if the two physical devices
do not have distinct source frames with valid transforms to `base_link`.

## Processing contract

For every input cloud, the node queries `target_frame <- message.frame_id`.
The LiDAR translation defines the inward vector from the sensor to
`robot_center` (default origin); the inverse TF rotation expresses that vector
in the local LiDAR frame. A wrap-safe angular comparison rejects
`blind_center +/- blind_width/2` before any point transformation.

The cropped records are byte-for-byte copies of each complete input point.
During TF only the existing `x`, `y`, and `z` fields are changed. Every other
field—including fields unknown to this package—is retained. Fusion requires
identical field layouts and endianness on both inputs and fails visibly rather
than corrupting a mismatched schema.

Outputs:

- `/mid360/front/filtered`, `/mid360/rear/filtered` (local sensor frames)
- `/mid360/front/rejected`, `/mid360/rear/rejected` (optional, local frames)
- `/mid360/merged` (`target_frame`)

`processing_stage` supports the incremental test cases: `crop`, `transform`,
and `merge`. No voxel filter, ground segmentation, SLAM, Nav2, or GPU path is
part of this package.

## Build and run

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select antbot_mid360_fusion --symlink-install
source install/setup.bash
ros2 launch antbot_mid360_fusion dual_mid360_fusion.launch.py use_rviz:=true
```

Override topics at launch if the live inspection differs:

```bash
ros2 launch antbot_mid360_fusion dual_mid360_fusion.launch.py \
  front_topic:=/actual/front/topic rear_topic:=/actual/rear/topic
```

For manual blind-sector debugging, set `auto_blind_direction: false` and edit
the two `blind_center_deg` values. In normal operation leave automatic mode on.
If rejected points appear outside the chassis in RViz, stop and correct the TF;
do not compensate by hard-coding an algorithmic yaw.

## Benchmark

Drivers and TF publishers must already be running for stages B-E. Run each
increment independently (60 seconds shown; use 300 for stability):

```bash
ros2 run antbot_mid360_fusion benchmark_dual_mid360.sh --stage A --duration 60
ros2 run antbot_mid360_fusion benchmark_dual_mid360.sh --stage B --duration 60
ros2 run antbot_mid360_fusion benchmark_dual_mid360.sh --stage C --duration 60
ros2 run antbot_mid360_fusion benchmark_dual_mid360.sh --stage D --duration 60
ros2 run antbot_mid360_fusion benchmark_dual_mid360.sh --stage E --duration 300
```

Each run creates `benchmark/YYYYMMDD_HHMMSS_STAGE/` with `system.csv`,
`process.csv`, `topics.csv`, `network.csv`, `latency.csv`, `tegrastats.log`,
`fusion.log` where applicable, and `summary.txt`. The benchmark intentionally
reports measurements only; it does not infer available compute headroom.
