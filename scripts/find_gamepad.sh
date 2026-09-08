#!/usr/bin/env bash
set -Eeuo pipefail

describe_device() {
  local device="$1"
  local sys_name="/sys/class/input/${device##*/}/device/name"
  if [[ -r "$sys_name" ]]; then
    tr -d '\n' <"$sys_name"
  else
    basename "$device"
  fi
}

is_gamepad() {
  local device="$1"
  local name properties
  name="$(describe_device "$device")"
  properties="$(udevadm info -q property -n "$device" 2>/dev/null || true)"

  if [[ "$name" =~ ([Tt]ouch|[Mm]ouse|ILITEK) ]]; then
    return 1
  fi
  if grep -Fxq 'ID_INPUT_JOYSTICK=1' <<<"$properties"; then
    return 0
  fi
  [[ "$name" =~ ([Xx]-?[Bb]ox|[Gg]amepad|[Jj]oystick|[Cc]ontroller|DualShock|DualSense) ]]
}

if [[ -n "${ANTBOT_JOY_DEVICE:-}" ]]; then
  if [[ ! -r "$ANTBOT_JOY_DEVICE" ]]; then
    echo "Configured controller is not readable: $ANTBOT_JOY_DEVICE" >&2
    exit 1
  fi
  if ! is_gamepad "$ANTBOT_JOY_DEVICE"; then
    echo "Configured input is not recognized as a gamepad: $ANTBOT_JOY_DEVICE" >&2
    exit 1
  fi
  printf '%s\n' "$ANTBOT_JOY_DEVICE"
  exit 0
fi

shopt -s nullglob
gamepads=()
for device in /dev/input/js*; do
  if [[ -r "$device" ]] && is_gamepad "$device"; then
    gamepads+=("$device")
  fi
done
shopt -u nullglob

if ((${#gamepads[@]} == 0)); then
  echo "No real gamepad found. Touchscreen joystick devices are ignored." >&2
  exit 1
fi
if ((${#gamepads[@]} > 1)); then
  echo "Multiple gamepads found: ${gamepads[*]}. Set ANTBOT_JOY_DEVICE." >&2
  exit 1
fi

printf '%s\n' "${gamepads[0]}"
