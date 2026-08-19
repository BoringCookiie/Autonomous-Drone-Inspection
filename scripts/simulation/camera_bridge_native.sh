#!/usr/bin/env bash
# camera_bridge_native.sh — Persistent GZ-to-ROS camera bridge.
#
# PX4's camera models do not all use the same Gazebo topic names.  The mono
# camera publishes /camera, while the RGB-D model normally publishes topics
# below /rgbd_camera.  Discover the topics instead of manufacturing empty ROS
# topics for a model that is not running.
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
set -u

RGB_ROS_TOPIC="${ROS_CAMERA_TOPIC:-/camera/color/image_raw}"
RGB_INFO_ROS_TOPIC="${ROS_CAMERA_INFO_TOPIC:-/camera/color/camera_info}"
DEPTH_ROS_TOPIC="${ROS_DEPTH_TOPIC:-/camera/depth/image_raw}"
POINTS_ROS_TOPIC="${ROS_POINTS_TOPIC:-/points}"

wait_for_gz_topics() {
  local timeout="${BRIDGE_CAMERA_WAIT_SEC:-120}"
  local deadline=$((SECONDS + timeout))
  local topics

  while true; do
    topics="$(gz topic -l 2>/dev/null || true)"
    if [[ -n "$topics" ]]; then
      printf '%s\n' "$topics"
      return 0
    fi
    if (( SECONDS >= deadline )); then
      echo "[camera_bridge] Gazebo has no topics after ${timeout}s" >&2
      return 1
    fi
    echo "[camera_bridge] Waiting for Gazebo camera topics..." >&2
    sleep 2
  done
}

select_topic() {
  local kind="$1"
  local topics="$2"
  local override=""
  if [[ "$kind" == rgb ]]; then
    override="${GZ_CAMERA_TOPIC:-}"
  elif [[ "$kind" == depth ]]; then
    override="${GZ_DEPTH_TOPIC:-}"
  elif [[ "$kind" == points ]]; then
    override="${GZ_POINTS_TOPIC:-}"
  fi

  if [[ -n "$override" ]]; then
    if printf '%s\n' "$topics" | grep -Fxq "$override"; then
      printf '%s' "$override"
      return 0
    fi
    echo "[camera_bridge] Requested ${kind} topic is absent: ${override}" >&2
    return 1
  fi

  while IFS= read -r candidate; do
    if [[ "$kind" == rgb ]]; then
      case "$candidate" in
        /camera|*/image|*/image_raw)
          [[ "$candidate" == *depth* || "$candidate" == *infra* || "$candidate" == *thermal* ]] && continue
          printf '%s' "$candidate"
          return 0
          ;;
      esac
    elif [[ "$kind" == depth ]]; then
      case "$candidate" in
        *depth_camera*|*depth_image|*/depth|*/depth_raw)
          printf '%s' "$candidate"
          return 0
          ;;
      esac
    elif [[ "$kind" == points ]]; then
      case "$candidate" in
        */points|*/point_cloud|*/pointcloud)
          printf '%s' "$candidate"
          return 0
          ;;
      esac
    fi
  done <<< "$topics"
  return 1
}

bridge_topic() {
  local gz_topic="$1"
  local ros_topic="$2"
  local ros_type="$3"
  local gz_type="$4"

  # '[' is the ros_gz_bridge GZ_TO_ROS direction.  The old '@' form is
  # bidirectional and creates an unnecessary ROS->GZ publisher for images.
  printf '[camera_bridge] %s -> %s (%s)\n' "$gz_topic" "$ros_topic" "$gz_type"
  BRIDGE_ARGS+=("${gz_topic}@${ros_type}[${gz_type}")
  BRIDGE_REMAPS+=(-r "${gz_topic}:=${ros_topic}")
}

echo "[camera_bridge] Starting discovered GZ-to-ROS bridge (${TYPE_PREFIX}.msgs)..."

while true; do
  gz_topics="$(wait_for_gz_topics)" || exit 1
  rgb_topic="$(select_topic rgb "$gz_topics" || true)"
  rgb_info_topic="${GZ_CAMERA_INFO_TOPIC:-}"
  depth_topic="$(select_topic depth "$gz_topics" || true)"
  points_topic="$(select_topic points "$gz_topics" || true)"

  if [[ -z "$rgb_topic" ]]; then
    echo "[camera_bridge] No RGB image topic found. Current Gazebo topics:" >&2
    printf '%s\n' "$gz_topics" | grep -Ei 'camera|image|depth' >&2 || true
    sleep 2
    continue
  fi

  # CameraInfo usually shares the camera prefix.  Prefer an explicit override,
  # otherwise select the first matching info topic.
  if [[ -z "$rgb_info_topic" ]]; then
    rgb_prefix="${rgb_topic%/*}"
    while IFS= read -r candidate; do
      if [[ "$candidate" == "${rgb_prefix}/camera_info" ]]; then
        rgb_info_topic="$candidate"
        break
      fi
    done <<< "$gz_topics"
    if [[ -z "$rgb_info_topic" ]]; then
      while IFS= read -r candidate; do
        if [[ "$candidate" == */camera_info ]]; then
          rgb_info_topic="$candidate"
          break
        fi
      done <<< "$gz_topics"
    fi
  fi

  BRIDGE_ARGS=()
  BRIDGE_REMAPS=()
  bridge_topic "$rgb_topic" "$RGB_ROS_TOPIC" sensor_msgs/msg/Image "${TYPE_PREFIX}.msgs.Image"
  if [[ -n "$rgb_info_topic" ]]; then
    bridge_topic "$rgb_info_topic" "$RGB_INFO_ROS_TOPIC" sensor_msgs/msg/CameraInfo "${TYPE_PREFIX}.msgs.CameraInfo"
  else
    echo "[camera_bridge] No CameraInfo topic found; continuing with RGB only" >&2
  fi
  if [[ -n "$depth_topic" ]]; then
    bridge_topic "$depth_topic" "$DEPTH_ROS_TOPIC" sensor_msgs/msg/Image "${TYPE_PREFIX}.msgs.Image"
  else
    echo "[camera_bridge] No depth topic found; depth remains unavailable for this model"
  fi
  # Phase 2: PointCloud2 bridge for OctoMap (gz_x500_depth model only).
  # The gz_x500_mono_cam model does not publish a /points topic; this is silently skipped.
  if [[ -n "$points_topic" ]]; then
    bridge_topic "$points_topic" "$POINTS_ROS_TOPIC" sensor_msgs/msg/PointCloud2 "${TYPE_PREFIX}.msgs.PointCloudPacked"
  else
    echo "[camera_bridge] No pointcloud topic found; /points will remain unpublished (mono-cam model)"
  fi

  if ! ros2 run ros_gz_bridge parameter_bridge "${BRIDGE_ARGS[@]}" --ros-args "${BRIDGE_REMAPS[@]}"; then
    echo "[camera_bridge] Bridge process exited; reconnecting after 2 seconds..." >&2
  fi

  sleep 2
done
