#!/usr/bin/env bash
set -eo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# Rebuild both overlays after relocation; do not source recorded setup chains.
source /opt/ros/jazzy/setup.bash
"$task_root/scripts/build_local_mapping_dependencies.sh"
source "$task_root/third_party/install/local_setup.bash"
cd "$task_root"
colcon build --base-paths src \
  --packages-up-to antbot_real_bringup antbot_mapping antbot_mid360_fusion \
  --event-handlers console_cohesion+
source "$task_root/install/local_setup.bash"
cd "$task_root/dual_arm_ws"
colcon build --base-paths src --event-handlers console_cohesion+ "$@"
