#!/usr/bin/env bash
# Source this file; paths are derived from its current location.
_antbot_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash || return 1
for _antbot_overlay in \
  "$_antbot_root/third_party/install" \
  "$_antbot_root/install" \
  "$_antbot_root/dual_arm_ws/install"; do
  if [[ ! -f "$_antbot_overlay/local_setup.bash" ]]; then
    echo "Missing build: $_antbot_overlay. Run $_antbot_root/build_dual_arm.sh" >&2
    return 1
  fi
  source "$_antbot_overlay/local_setup.bash" || return 1
done
_antbot_livox_sdk="$_antbot_root/third_party/livox-sdk2/lib"
if [[ -d "$_antbot_livox_sdk" ]]; then
  export LD_LIBRARY_PATH="$_antbot_livox_sdk${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
export ANTBOT_REAL_WS="$_antbot_root"
export REBOTARM_WORKSPACE="$_antbot_root/dual_arm_ws"
export REBOTARM_PROJECT=real_robot
export PYTHONNOUSERSITE=1
export EASY_HANDEYE2_CALIBRATIONS_DIRECTORY="$_antbot_root/dual_arm_ws/resources/calibrations"
export MERIDIAN_HAND_MODEL="$_antbot_root/dual_arm_ws/resources/models/hand_landmarker.task"
export MERIDIAN_VISION_PYTHON="$_antbot_root/dual_arm_ws/.venv/bin/python"
if [[ -d "$_antbot_root/dual_arm_ws/.hardware-venv/lib/python3.12/site-packages" ]]; then
  export PYTHONPATH="$_antbot_root/dual_arm_ws/.hardware-venv/lib/python3.12/site-packages:${PYTHONPATH:-}"
fi
export ROBOT_MOTION_ROOT="$_antbot_root/dual_arm_ws/resources/motions"
export ROBOT_ZONE_ROOT="$_antbot_root/dual_arm_ws/resources/zones"
export MOTOR_DM_DEVICE_CACHE_DIR="$_antbot_root/dual_arm_ws/resources/dm_device/cache"
_antbot_dm_lib="$_antbot_root/dual_arm_ws/resources/dm_device/$(uname -m)/libdm_device.so"
if [[ -f "$_antbot_dm_lib" ]]; then
  export MOTOR_DM_DEVICE_LIB="$_antbot_dm_lib"
fi
export ROS_LOG_DIR="$_antbot_root/dual_arm_ws/log/ros"
mkdir -p "$ROS_LOG_DIR"
unset _antbot_root _antbot_overlay _antbot_dm_lib _antbot_livox_sdk
