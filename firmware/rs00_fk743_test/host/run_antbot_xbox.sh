#!/usr/bin/env bash
set -Eeuo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly RS00_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
readonly ANTBOT_ROOT="${ANTBOT_ROOT:-/home/w/project/antbot}"
readonly UART_PORT="$(PYTHONPATH="$SCRIPT_DIR" python3 -c \
  'from serial_port import resolve_uart_port; print(resolve_uart_port())')"
readonly JOY_DEVICE="${RS00_JOY_DEVICE:-/dev/input/js0}"
readonly MAX_LINEAR_SPEED="${RS00_MAX_LINEAR_SPEED:-0.25}"
readonly PREPARE_TIMEOUT_SEC="${RS00_PREPARE_TIMEOUT_SEC:-30}"

declare -a CHILD_PIDS=()

cleanup()
{
  local pid
  trap - EXIT INT TERM HUP
  for pid in "${CHILD_PIDS[@]}"; do
    if kill -0 -- "-$pid" 2>/dev/null; then
      kill -INT -- "-$pid" 2>/dev/null || true
    fi
  done
  for pid in "${CHILD_PIDS[@]}"; do
    wait "$pid" 2>/dev/null || true
  done
}

handle_signal()
{
  cleanup
  exit 130
}

trap cleanup EXIT
trap handle_signal INT TERM HUP

query_status()
{
  python3 "$RS00_ROOT/host/control_tool.py" \
    --port "$UART_PORT" status
}

wait_for_chassis_state()
{
  local wanted_state=$1
  local deadline=$((SECONDS + PREPARE_TIMEOUT_SEC))
  local status_output

  while ((SECONDS < deadline)); do
    if ! status_output="$(query_status 2>&1)"; then
      echo "$status_output" >&2
      sleep 1
      continue
    fi
    echo "$status_output"
    if [[ "$wanted_state" == "WAIT_ENABLE" \
      && "$status_output" == *"homed=yes ready=no fault=no"* ]]; then
      return 0
    fi
    if [[ "$wanted_state" == "IDLE" \
      && "$status_output" == *"ready=yes fault=no"* ]]; then
      return 0
    fi
    if [[ "$status_output" == *"state=FAULT"* \
      || "$status_output" == *"fault=yes"* ]]; then
      return 2
    fi
    sleep 1
  done
  return 1
}

confirm_safe_start()
{
  local answer
  if [[ "${RS00_ASSUME_SAFE:-0}" == "1" ]]; then
    return 0
  fi
  if [[ ! -t 0 ]]; then
    echo "错误：非交互运行必须显式设置 RS00_ASSUME_SAFE=1。" >&2
    return 1
  fi
  echo
  echo "即将清故障、初始化并使能四台 RS00。"
  echo "请确认：四轮已架空、轮旁无人、物理急停可用。"
  read -r -p "确认后输入 ENABLE：" answer
  if [[ "$answer" != "ENABLE" ]]; then
    echo "已取消，未使能底盘。"
    return 1
  fi
}

prepare_chassis()
{
  local status_output

  status_output="$(query_status)"
  echo "$status_output"

  if [[ "$status_output" != *"steering_debug "* ]]; then
    echo "错误：STM32 正在运行旧版固件，缺少可靠的 RS00 状态诊断。" >&2
    echo "请先执行：$RS00_ROOT/flash_firmware.sh" >&2
    echo "刷写完成并按 RST 后，再重新运行本启动程序。" >&2
    return 1
  fi

  confirm_safe_start

  if [[ "$status_output" == *"state=FAULT"* \
    || "$status_output" == *"state=EMERGENCY_STOP"* \
    || "$status_output" == *"fault=yes"* ]]; then
    echo "检测到锁存故障，正在停止执行器并重新初始化……"
    python3 "$RS00_ROOT/host/control_tool.py" \
      --port "$UART_PORT" clear-fault-restart
    status_output=""
  fi

  if [[ "$status_output" != *"ready=yes fault=no"* \
    && "$status_output" != *"homed=yes ready=no fault=no"* ]]; then
    echo "等待转向初始化完成并进入 WAIT_ENABLE……"
    if ! wait_for_chassis_state WAIT_ENABLE; then
      echo "错误：转向初始化未在 ${PREPARE_TIMEOUT_SEC}s 内进入 WAIT_ENABLE。" >&2
      echo "请检查 g_steering_debug_state/error/fault_motor_id。" >&2
      return 1
    fi
    status_output="state=WAIT_ENABLE"
  fi

  if [[ "$status_output" == *"homed=yes ready=no fault=no"* \
    || "$status_output" == *"state=WAIT_ENABLE"* ]]; then
    echo "正在显式使能四台 RS00……"
    python3 "$RS00_ROOT/host/control_tool.py" \
      --port "$UART_PORT" steering-enable
    if ! wait_for_chassis_state IDLE; then
      echo "错误：转向使能未在 ${PREPARE_TIMEOUT_SEC}s 内进入 IDLE。" >&2
      python3 "$RS00_ROOT/host/control_tool.py" \
        --port "$UART_PORT" emergency-stop || true
      return 1
    fi
  fi

  echo "底盘已就绪：state=IDLE。"
}

if [[ ! -r "$ANTBOT_ROOT/scripts/setup_env.sh" ]]; then
  echo "错误：找不到 AntBot 环境脚本：$ANTBOT_ROOT/scripts/setup_env.sh" >&2
  exit 1
fi
if [[ ! -e "$UART_PORT" ]]; then
  echo "错误：串口不存在：$UART_PORT" >&2
  exit 1
fi
if ! python3 - "$MAX_LINEAR_SPEED" "$PREPARE_TIMEOUT_SEC" <<'PY'
import math
import sys

try:
    value = float(sys.argv[1])
    timeout = int(sys.argv[2])
except ValueError:
    raise SystemExit(1)
valid = math.isfinite(value) and 0.0 < value <= 0.5 and timeout > 0
raise SystemExit(0 if valid else 1)
PY
then
  echo "错误：速度必须在 (0, 0.5] m/s，准备超时必须是正整数。" >&2
  exit 2
fi
# shellcheck disable=SC1091
source "$ANTBOT_ROOT/scripts/setup_env.sh"

for package in antbot_teleop joy_linux; do
  if ! ros2 pkg prefix "$package" >/dev/null 2>&1; then
    echo "错误：当前 ROS 2 环境缺少 $package，请先构建 AntBot。" >&2
    exit 1
  fi
done

if ros2 node list --no-daemon 2>/dev/null |
  grep -Eq '^/(antbot_mapping_xbox|cmd_vel_uart_bridge)$'
then
  echo "错误：检测到旧的 Xbox 或串口桥节点，请先结束旧流程。" >&2
  exit 1
fi

echo "============================================================"
echo "RS00 实机 Xbox 控制（纯平移）"
echo "串口：$UART_PORT"
echo "手柄：$JOY_DEVICE"
echo "最大平移速度：$MAX_LINEAR_SPEED m/s"
echo "LT/RT 旋转：禁用（当前 STM32 固件尚不支持 angular.z）"
echo "============================================================"

prepare_chassis

echo
echo "Xbox 控制即将启动：按 A 解锁；再次按 A 立即锁定。"
echo "默认是 25% 档，按右摇杆可升到 $MAX_LINEAR_SPEED m/s。"
echo "本终端每秒显示电机摘要；完整 JSON 状态发布在 /rs00/motor_status。"

if ros2 topic info /joy 2>/dev/null |
  grep -Eq 'Publisher count: [1-9][0-9]*'
then
  echo "检测到现有 /joy 驱动，将复用它。"
else
  if [[ ! -e "$JOY_DEVICE" ]]; then
    echo "错误：Xbox 设备不存在：$JOY_DEVICE" >&2
    exit 1
  fi
  setsid ros2 run joy_linux joy_linux_node --ros-args \
    -r __node:=rs00_xbox_joy_node \
    -p dev:="$JOY_DEVICE" \
    -p deadzone:=0.05 \
    -p autorepeat_rate:=20.0 \
    -p sticky_buttons:=false &
  CHILD_PIDS+=("$!")
fi

setsid python3 "$RS00_ROOT/host/ros2_cmd_vel_uart.py" \
  --ros-args \
  -p port:="$UART_PORT" \
  -p baud:=115200 \
  -p max_linear_speed:="$MAX_LINEAR_SPEED" &
CHILD_PIDS+=("$!")

setsid ros2 run antbot_teleop mapping_xbox --ros-args \
  -p map_prefix:=/tmp/rs00_xbox_unused_map \
  -p map_use_sim_time:=false \
  -p use_sim_time:=false \
  -p max_linear_vel:="$MAX_LINEAR_SPEED" \
  -p max_angular_vel:=0.0 &
CHILD_PIDS+=("$!")

wait -n "${CHILD_PIDS[@]}"
