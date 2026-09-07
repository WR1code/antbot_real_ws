#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck disable=SC1091
source "${script_dir}/setup_env.sh"

if [[ ! -e /dev/input/js0 ]]; then
  echo "错误：没有找到 /dev/input/js0。" >&2
  exit 1
fi

if [[ "${ANTBOT_REAL_MOTION_CONFIRMED:-}" != "YES" ]]; then
  if [[ ! -t 0 ]]; then
    echo "错误：非交互运行必须设置 ANTBOT_REAL_MOTION_CONFIRMED=YES。" >&2
    exit 1
  fi
  echo "请确认：标定已完成、实体急停可用、轮旁无人、速度限制合适。"
  read -r -p "确认后输入 ENABLE：" answer
  [[ "$answer" == "ENABLE" ]] || { echo "已取消。"; exit 1; }
fi

ros2 run antbot_h743_bridge h743_control status
ros2 run antbot_h743_bridge h743_control steering-enable

ready=false
for _ in {1..10}; do
  sleep 1
  status="$(ros2 run antbot_h743_bridge h743_control status 2>&1 || true)"
  echo "$status"
  if [[ "$status" == *"ready=yes fault=no"* ]]; then
    ready=true
    break
  fi
done

if [[ "$ready" != true ]]; then
  echo "错误：H743 未进入 ready=yes，未启动手柄控制。" >&2
  exit 1
fi

exec ros2 launch antbot_real_bringup real_base.launch.py \
  start_xbox:=true start_joy:=true "$@"
