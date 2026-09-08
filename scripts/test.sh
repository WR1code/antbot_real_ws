#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace="$(cd -- "${script_dir}/.." && pwd)"

# shellcheck disable=SC1091
source "${script_dir}/setup_env.sh"

cd "$workspace"
colcon test --packages-select antbot_h743_bridge antbot_teleop \
  antbot_real_bringup \
  --event-handlers console_direct+
colcon test-result --verbose

cmake -S firmware/rs00_fk743_test -B build/rs00_protocol_tests \
  -DCMAKE_BUILD_TYPE=Release
cmake --build build/rs00_protocol_tests -j"$(nproc)"
ctest --test-dir build/rs00_protocol_tests --output-on-failure
