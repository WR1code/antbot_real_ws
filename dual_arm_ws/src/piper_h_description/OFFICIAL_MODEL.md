# Piper-H official model provenance

The deployed six-axis URDF is derived from AgileX Robotics' `piper_h` model:

- Repository: <https://github.com/agilexrobotics/agx_arm_urdf>
- Pinned commit: `f6642ce0d7872c686f29c99e9e10cd23d1d49313`
- Source file: `piper_h/urdf/piper_h_description.urdf`
- License: MIT; see `LICENSE.agx_arm_urdf`

Compatibility changes are intentionally limited to removing the upstream
`world` link (the MoveIt wrapper provides the selectable mounting frame),
renaming `link1` through `link6` to the workspace's established `Link1`
through `Link6`, and rewriting mesh package paths. Joint names, transforms,
axes, inertias and physical limits follow the pinned upstream model.

MoveIt receives a separate 0.1 rad start-state tolerance in its planning-only
joint-limit configuration. The hardware adapter retains the physical limits;
it can clamp only an initial, in-tolerance trajectory prefix while the robot
enters the physical range. Raw CAN feedback and calibration transforms are
never clamped.

The older `*_d435*.urdf`, `*_dark.urdf`, and
`piper_h_description.xacro` files contain a historical camera/gripper model.
They are not used by the default display launch, MoveIt, or gravity
compensation because their joint frames do not match the pinned official
six-axis model.
