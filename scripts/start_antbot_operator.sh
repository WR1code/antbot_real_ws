#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
real_workspace="$(cd -- "${script_dir}/.." && pwd)"
map_name="${ANTBOT_MAP_NAME:-home_01}"
map_project_dir="${ANTBOT_MAP_PROJECT_DIR:-${real_workspace}/artifacts/maps/${map_name}}"
rviz_config="${ANTBOT_RVIZ_CONFIG:-${real_workspace}/src/antbot_navigation/rviz/waypoint_navigation.rviz}"
max_linear_speed="${ANTBOT_MAX_LINEAR_SPEED:-1.50}"
default_teleop_mode="${ANTBOT_TELEOP_MODE:-xbox}"
mapping_run_id="${ANTBOT_MAPPING_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
mapping_output_prefix="${ANTBOT_MAPPING_OUTPUT_PREFIX:-${map_project_dir}/mapping_runs/${mapping_run_id}/map}"
uart_port="${RS00_UART_PORT:-/dev/serial/by-id/usb-1a86_USB_Single_Serial_5CE6063665-if00}"
check_only=false

usage()
{
  cat <<'EOF'
用法：
  ./scripts/start_antbot_operator.sh
  ./scripts/start_antbot_operator.sh --check-only

默认组合：
  工作区：当前 antbot_real_ws（模型、界面、导航工具和 H743 底层）
  地图：  <antbot_real_ws>/artifacts/maps/home_01

不会启动 Isaac Sim、Gazebo 或 Nav2 自动导航。没有连接 H743/手柄时界面也会打开，
并显示底盘离线。连接真机后须在 RViz 的“真机安全门禁”中人工确认。

环境变量：
  ANTBOT_MAP_NAME            地图项目名，默认 home_01
  ANTBOT_MAP_PROJECT_DIR     完整地图项目目录（优先于默认目录）
  RS00_UART_PORT             H743 的 /dev/serial/by-id/... 路径
  ANTBOT_JOY_DEVICE          Xbox/手柄设备，例如 /dev/input/js0
  ANTBOT_MAX_LINEAR_SPEED    线速度上限，默认 1.50 m/s（五档 10/25/50/75/100%）
  ANTBOT_TELEOP_MODE         默认控制方式：xbox 或 keyboard
  ANTBOT_MAPPING_OUTPUT_PREFIX  RViz 建图页面保存前缀
  ANTBOT_RVIZ_CONFIG         完整 Step2 RViz 配置文件
EOF
}

if [[ "$default_teleop_mode" != xbox && "$default_teleop_mode" != keyboard ]]; then
  echo "错误：ANTBOT_TELEOP_MODE 只能是 xbox 或 keyboard。" >&2
  exit 2
fi

if (($# > 1)); then
  usage >&2
  exit 2
fi
if (($# == 1)); then
  case "$1" in
    --check-only)
      check_only=true
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "错误：未知参数 $1" >&2
      usage >&2
      exit 2
      ;;
  esac
fi

require_file()
{
  local label=$1
  local path=$2
  if [[ ! -f "$path" ]]; then
    echo "错误：找不到${label}：$path" >&2
    exit 1
  fi
}

require_file "真机工作区环境" "${real_workspace}/install/setup.bash"
require_file "二维地图" "${map_project_dir}/map.yaml"
require_file "航点文件" "${map_project_dir}/waypoints.xml"
require_file "Step2 RViz 配置" "$rviz_config"

# shellcheck disable=SC1091
source "${script_dir}/setup_env.sh"

require_prefix()
{
  local package=$1
  local expected_root=$2
  local actual
  actual="$(ros2 pkg prefix "$package" 2>/dev/null || true)"
  if [[ -z "$actual" ]]; then
    echo "错误：当前 ROS 环境缺少包 $package。" >&2
    exit 1
  fi
  if [[ "$actual" != "$expected_root"/* ]]; then
    echo "错误：$package 不是来自 antbot_real_ws。" >&2
    echo "  实际：$actual" >&2
    echo "  期望位于：$expected_root" >&2
    exit 1
  fi
  printf '  %-26s %s\n' "$package" "$actual"
}

echo "AntBot real workspace: ${real_workspace}"
echo "Step2 map project:     ${map_project_dir}"
echo
echo "单一真机工作区检查："
for package in antbot_h743_bridge antbot_real_bringup antbot_description \
  antbot_teleop antbot_navigation antbot_dual_lidar antbot_rgbd_dataset \
  robotcar_navigation; do
  require_prefix "$package" "$real_workspace"
done
for package in rviz2 nav2_map_server nav2_lifecycle_manager slam_toolbox tf2_ros; do
  if ! ros2 pkg prefix "$package" >/dev/null 2>&1; then
    echo "错误：当前 ROS 环境缺少包 $package。" >&2
    exit 1
  fi
done

show_3d_cloud=false
show_rgbd_cloud=false
if [[ -f "${map_project_dir}/cloud.pcd" && -f "${map_project_dir}/cloud_metadata.yaml" ]]; then
  show_3d_cloud=true
fi
if [[ -f "${map_project_dir}/rgbd_preview.npz" ]]; then
  show_rgbd_cloud=true
fi

if [[ "$check_only" == true ]]; then
  echo
  echo "检查通过：地图、完整 Step2 界面和 H743 底层均可解析。"
  echo "未打开串口、未启动底盘、未启动 RViz 或仿真。"
  exit 0
fi

for node in /h743_cmd_vel_bridge /antbot_operator_manager /antbot_operator_slam \
  /antbot_real_xbox /rviz2_waypoints; do
  if ros2 node list --no-daemon 2>/dev/null | rg -Fxq "$node"; then
    echo "错误：检测到旧节点 $node，请先结束旧流程。" >&2
    exit 1
  fi
done

start_joy=true
joy_device="${ANTBOT_JOY_DEVICE:-auto}"
if detected_joy="$("${script_dir}/find_gamepad.sh" 2>/dev/null)"; then
  joy_device="$detected_joy"
  echo "手柄已连接：$joy_device"
else
  echo "手柄暂未连接：后台会持续检测并自动接入，也可在 RViz 切换为键盘控制。"
fi

echo
echo "正在从 antbot_real_ws 打开完整航点控制界面……"
echo "H743 可离线并自动重连；界面内点击“确认安全并启用”才会请求真机权限。"
echo "“建图与遥控”页可选 Xbox/键盘并启停建图；建图需要 /scan_0。"
echo "地图保存前缀：${mapping_output_prefix}"
echo "自动导航保持关闭（当前底层没有 /odom；Xbox LT/RT 原地旋转可用）。"
echo "关闭 RViz 或按 Ctrl+C 会同时停止整个真机控制流程。"

exec ros2 launch antbot_real_bringup operator_step2.launch.py \
  port:="$uart_port" \
  start_joy:="$start_joy" \
  joy_device:="$joy_device" \
  max_linear_speed:="$max_linear_speed" \
  default_teleop_mode:="$default_teleop_mode" \
  mapping_output_prefix:="$mapping_output_prefix" \
  map:="${map_project_dir}/map.yaml" \
  waypoints_file:="${map_project_dir}/waypoints.xml" \
  keepout_file:="${map_project_dir}/keepouts.keepout.json" \
  speed_zone_file:="${map_project_dir}/speeds.speed.json" \
  stuck_history_file:="${map_project_dir}/stuck_history.json" \
  pointcloud_path:="${map_project_dir}/cloud.pcd" \
  pointcloud_metadata:="${map_project_dir}/cloud_metadata.yaml" \
  rgbd_preview_path:="${map_project_dir}/rgbd_preview.npz" \
  show_3d_cloud:="$show_3d_cloud" \
  show_rgbd_cloud:="$show_rgbd_cloud" \
  rviz_config:="$rviz_config"
