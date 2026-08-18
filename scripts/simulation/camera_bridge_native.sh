#!/usr/bin/env bash
# camera_bridge_native.sh — Persistent C++ ROS-Gazebo Parameter Bridge
# Bridges Gazebo camera topics (/camera, /camera_info, /depth_camera) to ROS 2.
set -eo pipefail

source /opt/ros/humble/setup.bash

if [ -f /opt/ros_gz_harmonic/local_setup.bash ]; then
  # Image built from this repo: overlay compiled against Gazebo Harmonic,
  # which speaks the gz.msgs.* wire family.
  source /opt/ros_gz_harmonic/local_setup.bash
  export GZ_VERSION=harmonic
  TYPE_PREFIX='gz'
else
  # Stale image without the Harmonic overlay: the apt binary was compiled
  # for the Fortress wire family (ignition.msgs.*), so match that instead.
  TYPE_PREFIX='ignition'
fi

echo "[camera_bridge] Starting Native C++ ROS-Gazebo Parameter Bridge (${TYPE_PREFIX}.msgs)..."

while true; do
  ros2 run ros_gz_bridge parameter_bridge \
    /camera@sensor_msgs/msg/Image@${TYPE_PREFIX}.msgs.Image \
    /camera_info@sensor_msgs/msg/CameraInfo@${TYPE_PREFIX}.msgs.CameraInfo \
    /depth_camera@sensor_msgs/msg/Image@${TYPE_PREFIX}.msgs.Image \
    --ros-args \
    -r /camera:=/camera/color/image_raw \
    -r /camera_info:=/camera/color/camera_info \
    -r /depth_camera:=/camera/depth/image_raw || true

  echo "[camera_bridge] Bridge process exited. Re-attempting connection in 2 seconds..."
  sleep 2
done
