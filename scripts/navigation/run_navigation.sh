#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
export PYTHONPATH=$PYTHONPATH:/home/uas/scripts

echo "[Nav] Ensuring Camera Bridge is running..."
/home/uas/scripts/simulation/bridge_camera_topics.sh &

echo "[Nav] Launching Navigation Stack..."
ros2 launch /home/uas/scripts/navigation/navigation_launch.py
