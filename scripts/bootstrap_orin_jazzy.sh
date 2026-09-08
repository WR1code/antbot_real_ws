#!/usr/bin/env bash
set -Eeuo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "Please run this script with sudo." >&2
  exit 1
fi

antbot_target_user="${SUDO_USER:-w}"
if ! id "${antbot_target_user}" >/dev/null 2>&1; then
  echo "Target user does not exist: ${antbot_target_user}" >&2
  exit 1
fi

export DEBIAN_FRONTEND=noninteractive

apt-get update
apt-get install -y curl software-properties-common
add-apt-repository -y universe

antbot_ros_apt_version="$({
  curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest |
    sed -n 's/.*"tag_name":[[:space:]]*"\([^"]*\)".*/\1/p'
} | head -n 1)"

if [[ -z "${antbot_ros_apt_version}" ]]; then
  echo "Could not determine the latest ros-apt-source release." >&2
  exit 1
fi

. /etc/os-release
antbot_ubuntu_codename="${UBUNTU_CODENAME:-${VERSION_CODENAME:-}}"
antbot_ros_source_deb="/tmp/ros2-apt-source.deb"
curl -fL -o "${antbot_ros_source_deb}" \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${antbot_ros_apt_version}/ros2-apt-source_${antbot_ros_apt_version}.${antbot_ubuntu_codename}_all.deb"
dpkg -i "${antbot_ros_source_deb}"

apt-get update
apt-get install -y \
  ros-jazzy-ros-base \
  ros-dev-tools \
  python3-serial \
  python3-tk \
  cmake \
  build-essential

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  rosdep init
fi

usermod -aG dialout "${antbot_target_user}"

echo
echo "ROS 2 Jazzy prerequisites installed for AntBot."
echo "Log out and back in later for the dialout group change to take effect."
