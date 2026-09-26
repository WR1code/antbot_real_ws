#!/usr/bin/env bash
set -eo pipefail

task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$task_root/activate.sh"

failed=false
packages=(
  livox_ros_driver2
  fast_lio
  antbot_mapping
  antbot_mid360_fusion
  antbot_real_bringup
  robotcar_navigation
  rebot_xbox_hardware
)

echo "Runtime package prefixes:"
for package in "${packages[@]}"; do
  prefix="$(ros2 pkg prefix "$package")"
  printf '  %-28s %s\n' "$package" "$prefix"
  if [[ "$prefix" != "$task_root"/* ]]; then
    echo "ERROR: $package resolves outside this workspace" >&2
    failed=true
  fi
done

echo "Runtime binary linkage:"
binaries=(
  "$task_root/third_party/install/livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node"
  "$task_root/third_party/install/livox_ros_driver2/lib/liblivox_ros_driver2.so"
  "$task_root/third_party/install/fast_lio/lib/fast_lio/fastlio_mapping"
)
for binary in "${binaries[@]}"; do
  echo "  $binary"
  while IFS= read -r dependency; do
    case "$dependency" in
      *"not found"*)
        echo "ERROR: unresolved library: $dependency" >&2
        failed=true
        ;;
      *"=> /"*)
        path="${dependency#*=> }"
        path="${path%% *}"
        case "$path" in
          "$task_root"/*|/opt/ros/*|/usr/*|/lib/*|/lib64/*) ;;
          *)
            echo "ERROR: non-system library outside workspace: $path" >&2
            failed=true
            ;;
        esac
        ;;
    esac
  done < <(ldd "$binary")
done

echo "External project paths in active environment:"
if env | grep -E '/home/w/project/(odas|rebotarm|piperh|antbot)(/|:|$)' ; then
  echo "ERROR: an external project path remains active" >&2
  failed=true
else
  echo "  none"
fi

if [[ "$failed" == true ]]; then
  exit 1
fi
echo "Runtime boundary audit: PASS"
