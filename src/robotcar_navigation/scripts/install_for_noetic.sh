#!/bin/bash
set -e

# Legacy filename kept for compatibility. This installs the ROS 2 Jazzy
# navigation packages used by robotcar_navigation.
sudo apt-get update
sudo apt-get install -y \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-rosbridge-server \
  ros-jazzy-tf-transformations
