#!/bin/zsh
set -e

# Some ROS/colcon setup scripts may read this variable in tracing paths.
export AMENT_TRACE_SETUP_FILES=""

export ROS_LOG_DIR=/home/harekasa/mygo_ws/log/startup
mkdir -p "${ROS_LOG_DIR}"

source /opt/ros/jazzy/setup.zsh
source /home/harekasa/mygo_ws/install/setup.zsh

exec ros2 launch mygo_bringup bringup_on_raspi.launch.xml
