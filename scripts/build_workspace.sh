#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
cd /home/uas/ros2_ws
colcon build --symlink-install --packages-select uas_earthen_inspection
source install/setup.bash
echo "ROS workspace built: uas_earthen_inspection"
