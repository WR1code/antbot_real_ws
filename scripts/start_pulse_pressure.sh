#!/usr/bin/env bash
# The serial bridge owns the pressure sensor port; do not run a second bridge.
set -Eeuo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace="$(cd -- "${script_dir}/.." && pwd)"
if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  echo "用法：$0 /dev/serial/by-id/传感器设备名 [ROS 参数]"
  exit 0
fi
if (($# < 1)); then
  echo "用法：$0 /dev/serial/by-id/传感器设备名 [ROS 参数]" >&2
  exit 2
fi
pulse_port="$1"
shift
if [[ ! -e "$pulse_port" ]]; then
  echo "压力传感器串口不存在：${pulse_port}" >&2
  exit 1
fi
source "${script_dir}/setup_env.sh"
export PYTHONPATH="${workspace}/src/rebotarm_pulse${PYTHONPATH:+:${PYTHONPATH}}"
exec python3 -c 'from rebotarm_pulse.pressure_serial_bridge import main; main()' \
  --ros-args --params-file "${workspace}/src/rebotarm_pulse/config/piper_pressure_zero.yaml" \
  -p "port:=${pulse_port}" "$@"
