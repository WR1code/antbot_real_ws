#!/usr/bin/env bash
set -Eeuo pipefail

readonly PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly PROGRAMMER_DEFAULT="/home/w/.local/share/stm32cube/bundles/programmer/2.23.0/bin/STM32_Programmer_CLI"
readonly PROGRAMMER="${STM32_PROGRAMMER_CLI:-$PROGRAMMER_DEFAULT}"
readonly IMAGE="$PROJECT_DIR/firmware/RS00FK743/build/Debug/RS00FK743.hex"
readonly BIN_IMAGE="$PROJECT_DIR/firmware/RS00FK743/build/Debug/RS00FK743.bin"
readonly DFU_UTIL_DEFAULT="/home/w/.cache/antbot-arm-toolchain/root/usr/bin/dfu-util"
readonly DFU_UTIL="${DFU_UTIL:-$DFU_UTIL_DEFAULT}"

fail()
{
  echo "错误：$*" >&2
  exit 1
}

if [[ -x "$PROGRAMMER" ]]; then
  flash_backend=programmer
  [[ -s "$IMAGE" ]] || fail "找不到已编译固件：$IMAGE"
  listing="$($PROGRAMMER -l 2>&1)"
  dfu_port="$(printf '%s\n' "$listing" | sed -n 's/.*Device Index[[:space:]]*:[[:space:]]*\(USB[0-9][0-9]*\).*/\1/p' | head -n 1)"
else
  flash_backend=dfu-util
  [[ -x "$DFU_UTIL" ]] || fail "找不到 STM32CubeProgrammer 或 dfu-util"
  [[ -s "$BIN_IMAGE" ]] || fail "找不到已编译固件：$BIN_IMAGE"
  listing="$($DFU_UTIL -l 2>&1)"
  dfu_port="$(printf '%s\n' "$listing" | sed -n 's/.*0483:df11.*/0483:df11/p' | head -n 1)"
  if [[ "$listing" == *"LIBUSB_ERROR_ACCESS"* ]]; then
    fail "已发现 STM32 DFU，但当前用户没有 USB 写权限"
  fi
fi

if [[ -z "$dfu_port" ]]; then
  cat >&2 <<'EOF'
没有发现处于 DFU 模式的 STM32。

请完成下面的物理操作后，再运行本脚本：
  1. 停止底盘并架空四轮。
  2. 用核心板自身的 Type-C USB 口直连电脑（不是 CH340 串口）。
  3. 按住核心板 BOOT 键至少 1 秒后松开。
  4. 再执行 ./flash_firmware.sh
EOF
  exit 2
fi

echo "即将刷写：$([[ "$flash_backend" == programmer ]] && printf '%s' "$IMAGE" || printf '%s' "$BIN_IMAGE")"
echo "烧录工具：$flash_backend"
echo "DFU 端口：$dfu_port"
echo "请保持底盘架空，刷写期间不要断电或拔线。"

if [[ "${RS00_ASSUME_FLASH_SAFE:-0}" != "1" ]]; then
  [[ -t 0 ]] || fail "非交互刷写必须设置 RS00_ASSUME_FLASH_SAFE=1"
  read -r -p "确认后输入 FLASH：" answer
  [[ "$answer" == "FLASH" ]] || fail "已取消，未刷写固件。"
fi

if [[ "$flash_backend" == programmer ]]; then
  "$PROGRAMMER" -c "port=$dfu_port" -w "$IMAGE" -v
else
  "$DFU_UTIL" -d 0483:df11 -a 0 -s 0x08000000:leave -D "$BIN_IMAGE"
fi

cat <<'EOF'

固件写入并校验完成。
请按一下核心板 RST 键（或重新上电），等待串口重新出现，然后运行：
  ./start_xbox_chassis.sh
EOF
