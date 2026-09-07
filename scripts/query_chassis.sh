#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck disable=SC1091
source "${script_dir}/setup_env.sh"

ros2 run antbot_h743_bridge h743_control status
ros2 run antbot_h743_bridge h743_control system-health
ros2 run antbot_h743_bridge h743_control rs00-query-uid all
ros2 run antbot_h743_bridge h743_control drive-feedback all
