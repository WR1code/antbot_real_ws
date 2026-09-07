#!/usr/bin/env bash
set -Eeuo pipefail

readonly PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly PROGRAMMER_DEFAULT="/home/w/.local/share/stm32cube/bundles/programmer/2.23.0/bin/STM32_Programmer_CLI"
readonly PROGRAMMER="${STM32_PROGRAMMER_CLI:-$PROGRAMMER_DEFAULT}"
readonly IMAGE="$PROJECT_DIR/firmware/RS00FK743/build/Debug/RS00FK743.hex"

fail()
{
  echo "错误：$*" >&2
  exit 1
}

[[ -x "$PROGRAMMER" ]] || fail "找不到 STM32CubeProgrammer：$PROGRAMMER"
[[ -s "$IMAGE" ]] || fail "找不到已编译固件：$IMAGE"

listing="$($PROGRAMMER -l 2>&1)"
if [[ "$listing" == *"No STM32 device in DFU mode connected"* ]]; then
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

dfu_port="$(printf '%s\n' "$listing" | sed -n 's/.*Device Index[[:space:]]*:[[:space:]]*\(USB[0-9][0-9]*\).*/\1/p' | head -n 1)"
[[ -n "$dfu_port" ]] || fail "发现了设备，但无法解析 DFU 端口；请把 STM32_Programmer_CLI -l 的输出发给我。"

echo "即将刷写：$IMAGE"
echo "DFU 端口：$dfu_port"
echo "请保持底盘架空，刷写期间不要断电或拔线。"

if [[ "${RS00_ASSUME_FLASH_SAFE:-0}" != "1" ]]; then
  [[ -t 0 ]] || fail "非交互刷写必须设置 RS00_ASSUME_FLASH_SAFE=1"
  read -r -p "确认后输入 FLASH：" answer
  [[ "$answer" == "FLASH" ]] || fail "已取消，未刷写固件。"
fi

"$PROGRAMMER" -c "port=$dfu_port" -w "$IMAGE" -v

cat <<'EOF'

固件写入并校验完成。
请按一下核心板 RST 键（或重新上电），等待串口重新出现，然后运行：
  ./start_xbox_chassis.sh
EOF
