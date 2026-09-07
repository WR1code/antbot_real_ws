#!/usr/bin/env bash
set -Eeuo pipefail

PROFILE="${1:-navigation}"
DURATION="${2:-30}"
PROJECT_ROOT="${ANTBOT_SIM_ROOT:-}"
ISAAC_ROOT="${ISAAC_SIM_PATH:-}"
if [[ -z "$PROJECT_ROOT" || -z "$ISAAC_ROOT" ]]; then
  echo "This optional simulation benchmark requires ANTBOT_SIM_ROOT and ISAAC_SIM_PATH." >&2
  exit 2
fi
OUTPUT="${3:-$PROJECT_ROOT/isaac/reports/dual_lidar_performance_${PROFILE}.json}"
LOG_ROOT="${TMPDIR:-/tmp}/antbot_dual_lidar_perf_${PROFILE}_$$"

if [[ "$PROFILE" != mapping && "$PROFILE" != navigation ]]; then
  echo "profile must be mapping or navigation" >&2
  exit 2
fi
if [[ ! -x "$ISAAC_ROOT/python.sh" ]]; then
  echo "Isaac Sim python.sh not found; set ISAAC_SIM_PATH" >&2
  exit 2
fi
mkdir -p "$LOG_ROOT"

pids=()
cleanup() {
  trap - EXIT INT TERM
  for pid in "${pids[@]}"; do
    kill -INT "$pid" 2>/dev/null || true
  done
  wait "${pids[@]}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

set +u
source /opt/ros/jazzy/setup.bash
source "$PROJECT_ROOT/ros2_ws/install/setup.bash"
set -u

"$ISAAC_ROOT/python.sh" "$PROJECT_ROOT/isaac/scripts/run_antbot_dual_lidar.py" \
  --headless --lidar-mode 3d --lidar-profile "$PROFILE" --test-layout demo \
  >"$LOG_ROOT/isaac.log" 2>&1 &
pids+=("$!")

deadline=$((SECONDS + 180))
until ros2 topic list 2>/dev/null | grep -qx /clock; do
  if (( SECONDS >= deadline )); then
    echo "timed out waiting for /clock; see $LOG_ROOT/isaac.log" >&2
    exit 1
  fi
  sleep 1
done

ros2 launch antbot_description description.launch.py use_sim_time:=true \
  use_rviz:=false use_joint_state_publisher:=false \
  use_joint_state_publisher_gui:=false \
  >"$LOG_ROOT/rsp.log" 2>&1 &
pids+=("$!")
ros2 launch antbot_dual_lidar dual_lidar_bringup.launch.py \
  lidar_profile:="$PROFILE" use_rviz:=false start_synchronizer:=false \
  >"$LOG_ROOT/bringup.log" 2>&1 &
pids+=("$!")

ros2 run antbot_dual_lidar performance_probe \
  --duration "$DURATION" --expected-frequency 10 --output "$OUTPUT"
echo "report=$OUTPUT logs=$LOG_ROOT"
