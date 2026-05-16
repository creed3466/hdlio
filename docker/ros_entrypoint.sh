#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

export MAP_CONTAINER=/root/ros2_ws/data/map
export RAW_MAP_CONTAINER=/root/ros2_ws/data/map
export DATASET_DIR="${DATASET_DIR:-/root/ros2_ws/data/nrx}"
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-171}"

echo "============== TofSLAM ROS2 Docker Env Ready ================"

cd /root/ros2_ws
if [ -f /root/ros2_ws/install/setup.bash ]; then
  source /root/ros2_ws/install/setup.bash
fi
exec "$@"
