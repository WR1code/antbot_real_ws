# antbot_description

URDF/xacro description for the concept-v2 ANTBot 4-wheel independent
swerve-drive robot. The package name, robot name and eight actuated joint names
remain compatible with the original project.

The body envelope is approximately `0.830 x 0.540 x 0.410 m`; wheel radius is
`0.080 m`, wheelbase is `0.530 m`, and wheel-center track is `0.480 m`.

## Robot Model

### Joints

| Joint | Type | Axis | Limits |
|-------|------|------|--------|
| `steering_{corner}_joint` | revolute | Z | -90° ~ +90° |
| `wheel_{corner}_joint` | continuous | Y | — |

where `{corner}` = `front_left`, `front_right`, `rear_left`, `rear_right`

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
| `urdf/base.xacro` | Base link with mesh and inertia |
| `urdf/wheel.xacro` | Swerve wheel module macro (steering + drive) |
| `urdf/payload_tower.xacro` | RViz mast, RGB-D housing, and fixed dual Piper-X visual model |
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
| `meshes/umr_*.stl` | Concept-v2 lower body, shell, fascia and top parts |
| `meshes/steering_link_r.stl` | Steering module |
| `meshes/wheel_link_r.stl` | Wheel |

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
