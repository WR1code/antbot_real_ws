#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck disable=SC1091
source "${script_dir}/setup_env.sh"

joy_device="$("${script_dir}/find_gamepad.sh")"
joy_name="$(tr -d '\n' <"/sys/class/input/${joy_device##*/}/device/name")"

echo "Controller: ${joy_device} (${joy_name})"
echo "Dry run only: publishing /cmd_vel; the H743 bridge is not started."
echo "In another terminal, run:"
echo "  source ${script_dir}/setup_env.sh && ros2 topic echo /cmd_vel"

exec ros2 launch antbot_teleop mapping_xbox.launch.py \
  start_joy:=true \
  joy_device:="${joy_device}" \
  use_sim_time:=false \
  max_linear_speed:=1.50 \
  max_angular_speed:=1.0 \
  "$@"
