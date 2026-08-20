#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
export PYTHONPATH=$PYTHONPATH:/home/uas/scripts

echo "[Nav] Ensuring the canonical camera bridge is running..."
if ! ros2 node list 2>/dev/null | grep -qx '/ros_gz_bridge'; then
  bash /home/uas/scripts/simulation/camera_bridge_native.sh &
fi

echo "[Nav] Launching Navigation Stack..."
ros2 launch /home/uas/scripts/navigation/navigation_launch.py
