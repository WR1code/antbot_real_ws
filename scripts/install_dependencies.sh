#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace="$(cd -- "${script_dir}/.." && pwd)"
mode="${1:-core}"

if [[ "$mode" != "core" && "$mode" != "full" ]]; then
  echo "用法：$0 [core|full]" >&2
  exit 2
fi

# shellcheck disable=SC1091
export ANTBOT_SKIP_OVERLAY=1
source "${script_dir}/setup_env.sh"
unset ANTBOT_SKIP_OVERLAY

if ! command -v rosdep >/dev/null 2>&1; then
  echo "错误：请先安装 python3-rosdep。" >&2
  exit 1
fi

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  echo "rosdep 尚未初始化，需要执行：sudo rosdep init" >&2
  exit 1
fi

rosdep update

if [[ "$mode" == "core" ]]; then
  paths=(
    "${workspace}/src/antbot_interfaces"
    "${workspace}/src/antbot_description"
    "${workspace}/src/antbot_teleop"
    "${workspace}/src/antbot_h743_bridge"
    "${workspace}/src/antbot_real_bringup"
  )
else
  paths=("${workspace}/src")
fi

rosdep install --from-paths "${paths[@]}" --ignore-src \
  --skip-keys "ament_python ament_pytest" \
  --rosdistro "$ROS_DISTRO" -r -y
