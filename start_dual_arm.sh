#!/usr/bin/env bash
set -eo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$task_root/activate.sh"
export ANTBOT_CAMERA_SERIAL="${ANTBOT_CAMERA_SERIAL:-AY6R46300KN}"
# An absent adapter must not silently select another arm's can0.
if piper_device="$(python3 "$task_root/scripts/find_piper_can.py")"; then
  echo "Piper-H USB-CAN 已按序列号识别：$piper_device"
else
  piper_device=piper_missing
  echo "Piper-H 专用 USB-CAN 未找到；不会回退到其他 CAN 设备。" >&2
fi
echo "总启动：双臂 + 底盘控制 + 把脉网页 + 压力采集（串口无冲突时）"
mapping_arg=()
mapping_arg_present=false
for arg in "$@"; do
  if [[ "$arg" == use_mapping:=* ]]; then
    mapping_arg_present=true
    break
  fi
done
if [[ "$mapping_arg_present" == false ]]; then
  mapping_arg=("use_mapping:=${ANTBOT_MAPPING_CONTROLS:-true}")
fi
echo "MID360 建图：由 RViz 按钮按需启停（默认启用建图兼容 TF 模式）"
echo "请先停止独立底盘/网页/采集流程；按 Ctrl+C 停止本次总启动。"
exec ros2 launch rebot_xbox_hardware dual_arm_hardware.launch.py \
  piper_channel:="$piper_device" \
  piper_pulse_serial_port:="${ANTBOT_PULSE_PORT:-/dev/robot_serial}" \
  "${mapping_arg[@]}" "$@"
