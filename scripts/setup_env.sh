#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace="$(cd -- "${script_dir}/.." && pwd)"

ros_distro="${ANTBOT_ROS_DISTRO:-${ROS_DISTRO:-}}"
if [[ -z "$ros_distro" ]]; then
  if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    ros_distro=jazzy
  elif [[ -f /opt/ros/humble/setup.bash ]]; then
    ros_distro=humble
  else
    echo "错误：没有找到 ROS 2 Jazzy 或 Humble。" >&2
    return 1 2>/dev/null || exit 1
  fi
fi

ros_setup="/opt/ros/${ros_distro}/setup.bash"
if [[ ! -f "$ros_setup" ]]; then
  echo "错误：找不到 ${ros_setup}。" >&2
  return 1 2>/dev/null || exit 1
fi

set +u
# shellcheck disable=SC1090
source "$ros_setup"
if [[ "${ANTBOT_SKIP_OVERLAY:-0}" != "1" \
  && -f "${workspace}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${workspace}/install/setup.bash"
fi
set -u

export ANTBOT_REAL_WS="$workspace"
export H743_FIRMWARE_ROOT="${workspace}/firmware/rs00_fk743_test"
export ANTBOT_REAL_MAP_DIR="${ANTBOT_REAL_MAP_DIR:-${workspace}/maps/real}"

echo "AntBot real workspace: ${workspace}"
echo "ROS 2 distro: ${ros_distro}"
