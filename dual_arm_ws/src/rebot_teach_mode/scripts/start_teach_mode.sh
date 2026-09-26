#!/usr/bin/env bash

set -Eeo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly REBOTARM_ROOT="$(cd -- "${SCRIPT_DIR}/../../.." && pwd)"
readonly REAL_WORKSPACE="${REBOTARM_ROOT}/real_robot_ws"

mode=locked
mode_was_set=false

usage() {
  printf '%s\n' \
    '用法：./scripts/start_teach_mode.sh [模式]' \
    '' \
    '模式：' \
    '  --locked    只读安全模式，不允许录制和回放（默认）' \
    '  --hardware  开放服务录制/回放，不监听手柄' \
    '  --xbox      开放录制/回放，并启用 X/B/Start 按键' \
    '  -h, --help  显示帮助' \
    '' \
    '真机模式只启动示教节点，不会启动 CAN 驱动或自动使能机械臂。'
}

select_mode() {
  local requested=$1
  if [[ "${mode_was_set}" == true && "${mode}" != "${requested}" ]]; then
    printf '一次只能选择一个启动模式。\n' >&2
    exit 2
  fi
  mode="${requested}"
  mode_was_set=true
}

while (($#)); do
  case "$1" in
    --locked)
      select_mode locked
      ;;
    --hardware)
      select_mode hardware
      ;;
    --xbox)
      select_mode xbox
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      printf '未知参数：%s\n' "$1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  printf '找不到 ROS 2 Jazzy：/opt/ros/jazzy/setup.bash\n' >&2
  exit 1
fi
if [[ ! -f "${REAL_WORKSPACE}/install/setup.bash" ]]; then
  printf '真机工作空间尚未构建，请先执行：\n  cd %s && ./build.sh\n' \
    "${REAL_WORKSPACE}" >&2
  exit 1
fi

source /opt/ros/jazzy/setup.bash
source "${REAL_WORKSPACE}/install/setup.bash"
export ROS_LOG_DIR="${REAL_WORKSPACE}/log/ros"
mkdir -p "${ROS_LOG_DIR}"

case "${mode}" in
  locked)
    printf '启动示教节点：锁定模式（不会录制或驱动机械臂）。\n'
    exec ros2 launch rebot_teach_mode teach_mode.launch.py \
      allow_hardware:=false use_xbox:=false
    ;;
  hardware)
    printf '%s\n' \
      '启动示教节点：真机服务模式。' \
      '前提：驱动已启动、实体急停有效、Xbox/Servo 为 LOCKED。'
    exec ros2 launch rebot_teach_mode teach_mode.launch.py \
      allow_hardware:=true use_xbox:=false
    ;;
  xbox)
    printf '%s\n' \
      '启动示教节点：真机 Xbox 模式。' \
      '前提：驱动已启动、实体急停有效、手柄 A 键状态为 LOCKED。'
    exec ros2 launch rebot_teach_mode teach_mode.launch.py \
      allow_hardware:=true use_xbox:=true
    ;;
esac
