#!/usr/bin/env bash
set -eo pipefail

task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
vendor_root="$task_root/third_party"

source /opt/ros/jazzy/setup.bash
export LD_LIBRARY_PATH="$vendor_root/livox-sdk2/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

colcon --log-base "$vendor_root/log" build \
  --base-paths "$vendor_root/livox_ros_driver2" "$vendor_root/FAST_LIO_ROS2" \
  --build-base "$vendor_root/build" \
  --install-base "$vendor_root/install" \
  --symlink-install \
  --packages-up-to fast_lio \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DROS_EDITION=ROS2 \
    -DDISTRO_ROS="$ROS_DISTRO"
