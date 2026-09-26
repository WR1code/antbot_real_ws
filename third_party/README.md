# Local mapping dependencies

These files are the project-local runtime/build copies used by
`start_dual_arm.sh`. No mapping component is sourced from another project
workspace.

| Directory | Version | Origin |
|---|---|---|
| `livox_ros_driver2` | 1.2.6, commit `13eb05e4e6dd7a765b934d0c5fd6236676a57b49` | Livox official driver, existing validated checkout |
| `livox-sdk2` | 1.3.1 | Existing validated SDK install payload |
| `FAST_LIO_ROS2` | commit `2fffc570a25d0df172720bac034fbdb6a13d2162` | Ericsii ROS 2 port plus the existing Jazzy integration changes |

The FAST-LIO copy includes the checked-out `ikd-Tree` sources and has no
external Git submodule pointer. Git histories, generated logs, PCD output, and
the large paper/document directory were intentionally not copied because they
are not build or runtime inputs.

Build the local overlay with:

```bash
./scripts/build_local_mapping_dependencies.sh
```

Generated `third_party/build`, `third_party/install`, and `third_party/log`
directories are local to this workspace and ignored by Git.
