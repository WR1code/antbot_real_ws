#!/usr/bin/env bash
set -u

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace="$(cd -- "${script_dir}/.." && pwd)"
failures=0
warnings=0

pass() { echo "[通过] $*"; }
warn() { echo "[提醒] $*"; warnings=$((warnings + 1)); }
fail() { echo "[缺失] $*"; failures=$((failures + 1)); }

echo "AntBot / Orin 真机硬件预检"
echo "工作区：${workspace}"
echo

arch="$(uname -m)"
if [[ "$arch" == "aarch64" ]]; then
  pass "CPU 架构为 aarch64（Orin 原生架构）"
else
  warn "当前架构为 ${arch}；可做开发验证，但不是 Orin aarch64"
fi

if [[ -r /etc/os-release ]]; then
  # shellcheck disable=SC1091
  source /etc/os-release
  echo "[信息] 系统：${PRETTY_NAME:-unknown}"
fi

if [[ -r /etc/nv_tegra_release ]]; then
  pass "检测到 Jetson Linux：$(head -n 1 /etc/nv_tegra_release)"
  if command -v dpkg-query >/dev/null 2>&1; then
    jetpack_version="$(dpkg-query -W -f='${Version}' nvidia-jetpack 2>/dev/null || true)"
    [[ -n "$jetpack_version" ]] \
      && pass "nvidia-jetpack=${jetpack_version}" \
      || warn "未发现 nvidia-jetpack 元包；请核对已安装的 JetPack 组件"
  fi
elif [[ "$arch" == "aarch64" ]]; then
  warn "aarch64 系统没有 /etc/nv_tegra_release；请确认是否真是 Jetson BSP"
fi

if [[ -n "${ROS_DISTRO:-}" ]]; then
  pass "ROS_DISTRO=${ROS_DISTRO}"
elif [[ -f /opt/ros/jazzy/setup.bash || -f /opt/ros/humble/setup.bash ]]; then
  warn "ROS 已安装但当前终端尚未 source 环境"
else
  fail "没有找到 ROS 2 Jazzy/Humble"
fi

uart_port="${RS00_UART_PORT:-}"
if [[ -z "$uart_port" ]]; then
  shopt -s nullglob
  uart_candidates=(/dev/serial/by-id/usb-1a86_USB_Single_Serial_*-if00)
  shopt -u nullglob
  if ((${#uart_candidates[@]} == 1)); then
    uart_port="${uart_candidates[0]}"
  elif ((${#uart_candidates[@]} > 1)); then
    fail "发现多个 H743 USB-TTL 候选，请设置 RS00_UART_PORT"
  fi
fi

if [[ -n "$uart_port" && -e "$uart_port" ]]; then
  pass "H743 串口：${uart_port}"
  [[ -r "$uart_port" && -w "$uart_port" ]] \
    && pass "当前用户拥有串口读写权限" \
    || fail "当前用户没有串口读写权限；检查 dialout 组"
else
  fail "未发现 H743 专用 USB-TTL；不要用任意 ttyACM 代替"
fi

if id -nG | tr ' ' '\n' | grep -Fxq dialout; then
  pass "当前用户属于 dialout 组"
else
  warn "当前用户不属于 dialout 组"
fi

[[ -e /dev/input/js0 ]] \
  && pass "Xbox/手柄设备：/dev/input/js0" \
  || warn "没有 /dev/input/js0；不影响只启动底盘桥"

video_count="$(find /dev -maxdepth 1 -name 'video*' 2>/dev/null | wc -l)"
((video_count > 0)) \
  && pass "发现 ${video_count} 个 V4L2 视频设备" \
  || warn "没有发现 /dev/video* 相机设备"

if command -v ip >/dev/null 2>&1; then
  echo
  echo "[信息] 网络接口："
  ip -brief address | sed 's/^/  /'
fi

for item in \
  "前雷达:${ANTBOT_FRONT_LIDAR_IP:-192.168.1.12}" \
  "后雷达:${ANTBOT_REAR_LIDAR_IP:-192.168.1.13}"
do
  label="${item%%:*}"
  address="${item#*:}"
  if command -v ping >/dev/null 2>&1 && ping -c 1 -W 1 "$address" >/dev/null 2>&1; then
    pass "${label}可达：${address}"
  else
    warn "${label}不可达或尚未连接：${address}"
  fi
done

config="${workspace}/firmware/rs00_fk743_test/firmware/App/Inc/steering_config.h"
if grep -Eq '^#define[[:space:]]+STEERING_CALIBRATION_CONFIRMED[[:space:]]+1' "$config"; then
  pass "复制的 H743 配置标记为已完成转向标定"
else
  fail "H743 仍是未标定锁定配置；禁止整车运动"
fi

if [[ -e /sys/class/net/can0 ]]; then
  warn "检测到可选 USB-CAN can0；它不是 Orin-H743 正常控制链路的必需设备"
fi

echo
echo "结果：缺失 ${failures} 项，提醒 ${warnings} 项。"
if ((failures > 0)); then
  exit 1
fi
