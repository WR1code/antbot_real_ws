# antbot_description

URDF/xacro description for the GK4XC-001-003 4-wheel independent steering
chassis. The package name, robot name, sensor/control TF frames and eight
actuated joint names remain compatible with the original project.

The supplied SolidWorks STEP assembly was converted to a centred, ground-aligned
binary STL for RViz. Its visual envelope is approximately
`0.551 x 0.819 x 1.286 m` (ROS X x Y x Z). The detailed CAD mesh is visual-only; collision uses
a conservative box so runtime planning does not process the high-detail mesh.

## Robot Model

### Joints

| Joint | Type | Axis | Limits |
|-------|------|------|--------|
| `steering_{corner}_joint` | continuous | Z | — |
| `wheel_{corner}_joint` | continuous | Y | — |

where `{corner}` = `front_left`, `front_right`, `rear_left`, `rear_right`

The four nominal wheel/steering axes are at `(±0.165, ±0.165) m`, forming the
measured 330 mm square. The detailed one-piece CAD mesh is the visual source of
truth; the steering and wheel child links remain available for TF, collision,
control and `/joint_states` without overlaying legacy concept meshes. Wheel
collision radius is `0.0525 m`, matching the 105 mm drive-wheel diameter in the
H743 configuration.

### Sensor Frames

The model defines TF frames for the following sensors:

- **Cameras** — stereo depth (front), 4x mono (left/front/right/back)
- **IMU** — 6-axis inertial measurement unit
- **GNSS** — u-blox GPS receiver
- **Magnetometer**
- **LiDAR** — two diagonal 2D lidars (`lidar_2d_front_link` at front-left and
  `lidar_2d_back_link` at rear-right), plus the top 3D lidar frame
- **Charging coil** — wireless charging contact

## Xacro Structure

| File | Description |
|------|-------------|
| `urdf/antbot.xacro` | Top-level robot definition, sensor mounting, and parameters |
| `urdf/base.xacro` | Base link using the GK4XC production CAD mesh |
| `urdf/wheel.xacro` | Swerve wheel module macro (steering + drive) |
| `urdf/ros2_control.xacro` | ros2_control hardware interface definition |
| `urdf/sensors.xacro` | Sensor frame mounting macros |

## Visualization

```bash
# View in RViz with interactive joint sliders
ros2 launch antbot_description description.launch.py

# Launch arguments
ros2 launch antbot_description description.launch.py use_rviz:=true use_joint_state_publisher_gui:=true
```

| Argument | Default | Description |
|----------|---------|-------------|
| `use_sim_time` | `false` | Use simulation clock |
| `use_joint_state_publisher` | `false` | Enable joint_state_publisher |
| `use_joint_state_publisher_gui` | `true` | Enable GUI joint sliders |
| `use_rviz` | `true` | Launch RViz |

## Meshes

| File | Description |
|------|-------------|
| `meshes/gk4xc_001_003_full.stl` | Decimated binary mesh generated from the supplied STEP assembly |
| `cad/GK4XC-001-003 4轮转向小车总装.STEP` | Original SolidWorks assembly, stored with Git LFS |

## Dependencies

| Dependency | Description |
|-----------|-------------|
| `urdf` | URDF parser |
| `xacro` | Xacro macro processor |
| `robot_state_publisher` | URDF → TF broadcast |
| `joint_state_publisher` | Joint state publishing |
| `joint_state_publisher_gui` | Interactive joint visualization |

## Build

```bash
colcon build --symlink-install --packages-select antbot_description
```

## License

Apache License 2.0 — Copyright 2026 ROBOTIS AI CO., LTD.
