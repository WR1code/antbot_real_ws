#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace="$(cd -- "${script_dir}/.." && pwd)"

# shellcheck disable=SC1091
export ANTBOT_SKIP_OVERLAY=1
source "${script_dir}/setup_env.sh"
unset ANTBOT_SKIP_OVERLAY

cd "$workspace"
if (($# > 0)); then
  colcon build --symlink-install "$@"
elif [[ "${ANTBOT_BUILD_FULL:-0}" == "1" ]]; then
  colcon build --symlink-install
else
  colcon build --symlink-install --packages-up-to antbot_real_bringup
fi
