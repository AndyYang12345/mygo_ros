#!/bin/bash
set -euo pipefail

export ROS_LOG_DIR=/home/harekasa/mygo_ws/log/startup
mkdir -p "${ROS_LOG_DIR}"

source /opt/ros/jazzy/setup.bash
source /home/harekasa/mygo_ws/install/setup.bash

exec ros2 launch mygo_bringup bringup_on_raspi.launch.xml
