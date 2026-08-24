#!/usr/bin/env bash
# NOTE: no `set -u` — /opt/ros/humble/setup.bash is not nounset-safe
set -Eeo pipefail

source /opt/ros/humble/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/uas/fastdds_udp.xml

BAG_ROOT="${FLY_PATTERN_BAG_DIR:-/home/uas/rosbags}"
BAG_NAME="${UAS_BAG_NAME:-nav_run_$(date +%Y%m%d_%H%M%S)}"
BAG_PATH="$BAG_ROOT/$BAG_NAME"

mkdir -p "$BAG_ROOT"
cd "$BAG_ROOT"
echo "[recorder] recording curated topics to $BAG_PATH"

exec ros2 bag record \
  --storage sqlite3 \
  -o "$BAG_PATH" \
  /clock \
  /camera/color/image_raw \
  /camera/color/camera_info \
  /camera/depth/image_raw \
  /points \
  /points_clean \
  /octomap_point_cloud_centers \
  /uas1/state \
  /uas1/extended_state \
  /uas1/battery \
  /uas1/local_position/pose \
  /uas1/local_position/odom \
  /uas1/local_position/velocity_local \
  /uas1/local_position/velocity_body \
  /uas1/imu/data \
  /uas1/imu/data_raw \
  /uas1/global_position/global \
  /uas1/global_position/raw/fix \
  /uas1/setpoint_position/local \
  /navigation/goal \
  /planned_path \
  /planner/coverage_path \
  /tf \
  /tf_static
