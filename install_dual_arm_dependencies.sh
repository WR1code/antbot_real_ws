#!/usr/bin/env bash
set -eo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
export PYTHONNOUSERSITE=1
rosdep install --from-paths "$task_root/src" "$task_root/dual_arm_ws/src" --ignore-src --rosdistro jazzy -r -y
# The reBotArm SDK uses ROS Pinocchio (NumPy 1.x), not a pip NumPy 2 overlay.
if ! /usr/bin/python3 -c 'import pinocchio' >/dev/null 2>&1; then
  echo 'Missing ROS Pinocchio: install ros-jazzy-pinocchio before continuing.' >&2
  exit 1
fi
# Ubuntu python3-venv is required. Recreate these environments after relocation.
/usr/bin/python3 -m venv --system-site-packages "$task_root/dual_arm_ws/.hardware-venv"
"$task_root/dual_arm_ws/.hardware-venv/bin/python" -m pip install -r "$task_root/dual_arm_ws/requirements-hardware.txt"
task_dm_lib="$task_root/dual_arm_ws/resources/dm_device/$(uname -m)/libdm_device.so"
if [[ ! -f "$task_dm_lib" ]]; then
  MOTOR_DM_DEVICE_CACHE_DIR="$task_root/dual_arm_ws/resources/dm_device/cache" \
    "$task_root/dual_arm_ws/.hardware-venv/bin/python" -m motorbridge.dm_device_runtime --download
fi
/usr/bin/python3 -m venv --system-site-packages "$task_root/dual_arm_ws/.venv"
"$task_root/dual_arm_ws/.venv/bin/python" -m pip install -r "$task_root/dual_arm_ws/requirements-vision.txt"
