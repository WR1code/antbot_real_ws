# antbot_teleop

Teleoperation package for the ANTBot swerve-drive robot. Provides keyboard,
DualSense, and guarded Xbox control nodes. All motion nodes publish
`geometry_msgs/msg/Twist` on `/cmd_vel`.

## Nodes

### mapping_xbox（Isaac 扫描建图，默认）

`step1_mapping.sh` 默认启动已实测的 `Generic X-Box pad`：

| 操作 | 功能 |
|---|---|
| A | 底盘安全锁切换；解锁前左摇杆必须回中 |
| 左摇杆 | 全向平移；松开立即发布零速 |
| LT / RT | 逆时针 / 顺时针原地旋转 |
| 左 / 右摇杆按下 | 降低 / 提高速度档位（10% / 25% / 50% / 75% / 100%） |
| B | 急停、锁定并保存地图 |
| X | 急停、锁定、保存地图并退出第一阶段 |
| Xbox / Mode | 在底盘和机械臂之间切换 |

节点启动、手柄断联超过 0.3 秒或切换控制对象时都会锁定并持续发布零速。
切换对象后需将控制器回中，再按 A 解锁当前对象。若机械臂已经提供 `/joy`，
第一阶段会自动复用；若先启动底盘、再启动机械臂，机械臂 launch 需传入
`start_joy:=false`，避免两个驱动读取同一设备。

```bash
cd /home/w/project/antbot/isaac
./scripts/step1_mapping.sh

# 临时退回原键盘控制
ANTBOT_TELEOP=keyboard ./scripts/step1_mapping.sh
```

### mapping_keyboard（Isaac 扫描建图）

四舵轮全向键位：`q/w/e/d/c/x/z/a` 依次对应八个平移方向，`r/t`
对应顺时针/逆时针原地旋转，空格急停、`1~9` 调速、`m` 保存 `/map`。斜向
指令会归一化，合速度与直向相同。该节点由
`/home/w/project/antbot/isaac/scripts/step1_mapping.sh` 直接启动，一般无需
单独运行。

### teleop_smooth（推荐）

从 robotcar 平滑控制逻辑适配的 ANTBot 键盘节点。它保留 20 Hz 连续发布和加减速，
同时增加 ANTBot 的横移控制；只发布标准 `/cmd_vel`，所以仿真和实车使用同一命令。

```bash
ros2 run antbot_teleop teleop_smooth

# 实车首次测试建议限速
ros2 run antbot_teleop teleop_smooth --ros-args \
  -p max_linear_vel:=0.3 -p max_angular_vel:=0.6
```

按键为 `w/x` 前后、`a/d` 横移、`q/e` 旋转、`s/空格` 急停、`1~9` 调速。
节点必须直接在带 TTY 的终端运行。

### teleop_keyboard

Terminal-based keyboard control. The robot moves only while a key is held and stops on release.

```
   q    w    e
   a         d
        x

w/x       : forward / backward     (linear.x)
a/d       : strafe left / right    (linear.y)
q/e       : rotate CCW / CW        (angular.z)
1~9       : speed level (1=slow, 9=max)
ESC/Ctrl+C: quit
```

> **Note:** Requires a terminal (TTY) for keyboard input. Always run directly in a terminal, not via a launch file.

#### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `max_linear_vel` | `1.0` | Maximum linear velocity [m/s] |
| `max_angular_vel` | `1.0` | Maximum angular velocity [rad/s] |
| `speed_level` | `3` | Initial speed level (1~9) |
| `publish_rate` | `10.0` | Publish rate [Hz] |

### teleop_joystick

Joystick control with automatic controller detection (DualSense) and geometry-based angular velocity limiting. Prevents steering angles from exceeding hardware limits by computing maximum angular velocity from robot geometry.

**Supported controllers:** PS5 DualSense (tested), PS4 DualShock 4 (untested) via USB connection. The node auto-detects the controller type via USB product ID, with axis-based fallback detection. Hot-plug is supported — swapping controllers mid-session is automatically handled.

```
[Left Stick]                        [Triggers]
  Y-axis : forward / backward        L2 : in-place rotate CCW
  X-axis : curve turning              R2 : in-place rotate CW
           (while moving)

[Buttons]
  Triangle : speed level UP (+1)     L1 : headlight toggle (ON/OFF)
  Cross    : speed level DOWN (-1)   R1 : wiper toggle (REPEAT/OFF)
  Square   : cargo lock              Circle : cargo unlock
```

**Two driving modes** (mutually exclusive):
- **Curve driving** (`|vx| >= 0.05 m/s`): Left stick X controls angular velocity, constrained by `w_max = min(|vx| / R_min, W_ABS_MAX)`. Steering inverts automatically when reversing.
- **In-place rotation** (`|vx| < 0.05 m/s`): L2/R2 triggers control spin. `wz = max_spin_vel * speed_ratio * (L2 - R2)`.

**Service calls:** Square/Circle buttons call `cargo/command` (`CargoCommand`). L1 toggles `headlight/operation` (`SetBool`). R1 toggles `wiper/operation` (`WiperOperation`).

#### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `max_linear_vel` | `1.0` | Maximum linear velocity [m/s] |
| `max_spin_vel` | `1.0` | Maximum in-place rotation speed [rad/s] |
| `speed_level` | `3` | Initial speed level (1~9) |
| `deadzone` | `0.1` | Stick/trigger deadzone threshold |
| `module_x` | `0.265` | Swerve module X-offset from base center [m] |
| `module_y` | `0.256` | Swerve module Y-offset from base center [m] |
| `steering_limit_deg` | `60.0` | Hardware steering limit [deg] |
| `safety_factor` | `0.95` | Use 95% of steering limit (3 deg margin) |
| `w_abs_max` | `2.0` | Absolute max angular velocity [rad/s] |

## Usage

```bash
# Keyboard teleop (run in terminal)
ros2 run antbot_teleop teleop_keyboard

# Joystick teleop (DualSense via USB)
ros2 launch antbot_teleop teleop_joy.launch.py
```

Override parameters via command line:
```bash
ros2 run antbot_teleop teleop_keyboard --ros-args -p max_linear_vel:=0.5
ros2 run antbot_teleop teleop_joystick --ros-args -p max_linear_vel:=0.5 -p deadzone:=0.15
```

## Dependencies

| Dependency | Description |
|-----------|-------------|
| `rclpy` | ROS 2 Python client library |
| `geometry_msgs` | Twist message type |
| `sensor_msgs` | Joy message type (joystick) |
| `antbot_interfaces` | CargoCommand, WiperOperation service types |
| `std_srvs` | SetBool service type (headlight) |
| `joy` | Joystick driver node (runtime) |

## Build

```bash
colcon build --symlink-install --packages-select antbot_teleop
```

## License

Apache License 2.0 — Copyright 2026 ROBOTIS AI CO., LTD.
