#!/usr/bin/env bash
set -euo pipefail

CONTAINER="${UAS_SIM_CONTAINER:-uas_sim}"

inner='
set -euo pipefail
source /opt/ros/humble/setup.bash
echo "[bridge_camera_topics] Starting native C++ Gazebo -> ROS 2 camera bridge..."
exec ros2 run ros_gz_bridge parameter_bridge \
  /camera@sensor_msgs/msg/Image@gz.msgs.Image \
  /depth_camera@sensor_msgs/msg/Image@gz.msgs.Image \
  /camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo
'

if docker inspect -f '{{.State.Running}}' "$CONTAINER" >/dev/null 2>&1; then
  if [[ -t 0 ]]; then
    exec docker exec -it "$CONTAINER" bash -lc "$inner"
  fi
  exec docker exec "$CONTAINER" bash -lc "$inner"
fi

exec bash -lc "$inner"
