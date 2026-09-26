#!/usr/bin/env bash
# Only serve the dashboard and subscribe to pressure topics; never move the robot.
set -Eeuo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace="$(cd -- "${script_dir}/.." && pwd)"
if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  echo "用法：$0 [ROS 参数，例如 -p port:=8766]"
  exit 0
fi
source "${script_dir}/setup_env.sh"
export PYTHONPATH="${workspace}/src/rebotarm_pulse${PYTHONPATH:+:${PYTHONPATH}}"
exec python3 -c 'from rebotarm_pulse.pulse_web_gateway import main; main()' \
  --ros-args -p "site_directory:=${workspace}/src/rebotarm_pulse/web" "$@"
