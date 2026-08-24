#!/usr/bin/env bash
set -e
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/uas/fastdds_udp.xml
source /opt/ros/humble/setup.bash
export MAVROS_NS="${MAVROS_NS:-/uas1}"
export FLY_PATTERN_SKIP_PARAM="${FLY_PATTERN_SKIP_PARAM:-1}"
REQUIRE_DEPTH="${FLY_PATTERN_REQUIRE_DEPTH:-0}"

echo "[fly] Waiting for /uas1/cmd/arming service..."
for i in $(seq 1 120); do
  if ros2 service list 2>/dev/null | grep -qF "/uas1/cmd/arming"; then
    echo "[fly] /uas1/cmd/arming is up"
    break
  fi
  sleep 1
done

echo "[fly] Waiting for camera topics to register in ROS graph..."
for i in $(seq 1 30); do
  if ros2 topic list 2>/dev/null | grep -qx "/camera/color/image_raw"; then
    echo "[fly] RGB camera topic detected at /camera/color/image_raw"
    break
  fi
  sleep 1
done

if [ "$REQUIRE_DEPTH" = "1" ]; then
  for i in $(seq 1 30); do
    if ros2 topic list 2>/dev/null | grep -qx "/camera/depth/image_raw"; then
      echo "[fly] Depth camera topic detected at /camera/depth/image_raw"
      break
    fi
    sleep 1
  done
fi

echo "[fly] Launching flight pattern controller..."
python3 /home/uas/scripts/simulation/fly_pattern.py
echo "[fly] Flight pattern complete. Exporting telemetry CSV and camera MP4 video..."
BAG_SELECTOR=latest
if [[ -s /home/uas/rosbags/.active_bag ]]; then
  # `read` exits nonzero on a final line without a trailing newline; never let
  # that kill the post-flight export under set -e (fall back to 'latest')
  read -r BAG_SELECTOR < /home/uas/rosbags/.active_bag || BAG_SELECTOR=latest
fi
python3 /home/uas/scripts/analysis/analyze_rosbags.py \
  --bag "$BAG_SELECTOR" \
  --export-csv \
  --export-video \
  --video-topic /camera/color/image_raw
