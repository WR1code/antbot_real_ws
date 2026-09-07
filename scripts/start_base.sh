#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck disable=SC1091
source "${script_dir}/setup_env.sh"

exec ros2 launch antbot_real_bringup real_base.launch.py \
  start_xbox:=false "$@"
