#!/usr/bin/env bash
set -eo pipefail

VISION_PYTHON="${MERIDIAN_VISION_PYTHON:?Source antbot_real_ws/activate.sh first}"
if [[ ! -x "$VISION_PYTHON" ]]; then
  echo "Missing vision environment: $VISION_PYTHON; run install_dual_arm_dependencies.sh" >&2
  exit 1
fi
export MPLCONFIGDIR="/tmp/meridian-matplotlib"
mkdir -p "${MPLCONFIGDIR}"
set -u

# cv_bridge is intentionally not used because the Jazzy binary targets NumPy 1.x.
exec "$VISION_PYTHON" -m meridian_hand_vision.hand_depth_viewer "$@"
