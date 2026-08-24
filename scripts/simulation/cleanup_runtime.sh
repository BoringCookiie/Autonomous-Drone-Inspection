#!/usr/bin/env bash
set -u

# Stop only processes belonging to this repository's simulation runtime.
# This script intentionally lives in the container and is executed as a file:
# matching its own command text is therefore impossible.

SELF=$$
PARENT=$(ps -o ppid= -p "$SELF" 2>/dev/null | tr -d ' ')

matches_runtime() {
  case "$1" in
    *"/home/uas/scripts/simulation/run_obstacle_flight.sh"*|\
    *"/home/uas/scripts/simulation/run_mavros.sh"*|\
    *"/home/uas/scripts/simulation/camera_bridge_native.sh"*|\
    *"/home/uas/scripts/simulation/run_fly_pattern.sh"*|\
    *"/home/uas/scripts/simulation/fly_pattern.py"*|\
    *"/home/uas/scripts/navigation/navigation_launch.py"*|\
    *"/home/uas/scripts/navigation/planner_3d.py"*|\
    *"/home/uas/scripts/navigation/path_follower.py"*|\
    *"/home/uas/scripts/navigation/coverage_planner.py"*|\
    *"/home/uas/scripts/navigation/tf_bridge_node.py"*|\
    *"/home/uas/scripts/navigation/depth_cloud_sanitizer.py"*|\
    *"/home/uas/scripts/navigation/truth_watchdog.py"*|\
    *"ros_gz_bridge parameter_bridge"*|\
    *"/opt/ros_gz_harmonic/lib/ros_gz_bridge/parameter_bridge"*|\
    *"rosbag2_transport/record"*|\
    *"/opt/ros/humble/bin/ros2 bag record"*|\
    *"/opt/ros/humble/bin/ros2 launch"*|\
    *"/opt/ros/humble/bin/ros2 topic"*|\
    *"/opt/ros/humble/bin/ros2 service"*|\
    *"/opt/ros/humble/lib/mavros/mavros_node"*|\
    *"/opt/ros/humble/lib/octomap_server/octomap_server_node"*|\
    *"/opt/ros/humble/lib/rtabmap_slam/rtabmap"*|\
    *"/home/uas/PX4-Autopilot/build/px4_sitl_default/bin/px4"*|\
    *"bin/px4 -d"*|\
    *"/home/uas/PX4-Autopilot/build/px4_sitl_default/bin/px4-commander"*|\
    *"px4-commander arm"*|\
    *"gz sim"*|\
    *"rviz2"*) return 0 ;;
  esac
  return 1
}

collect_pids() {
  ps -eo pid=,pgid=,args= | while read -r pid pgid command; do
    [ "$pid" = "$SELF" ] && continue
    [ -n "${PARENT:-}" ] && [ "$pid" = "$PARENT" ] && continue
    [ "$pid" = "1" ] && continue
    matches_runtime "$command" || continue
    printf '%s\n' "$pid"
  done
}

signal_pids() {
  local signal="$1"
  local pid
  for pid in $(collect_pids); do
    kill "$signal" "$pid" 2>/dev/null || true
  done
}

collect_recording_pids() {
  local pid pgid command
  ps -eo pid=,pgid=,args= | while read -r pid pgid command; do
    [ "$pid" = "$SELF" ] && continue
    [ -n "${PARENT:-}" ] && [ "$pid" = "$PARENT" ] && continue
    [ "$pid" = "1" ] && continue
    case "$command" in
      *"ros2 bag record"*|*"rosbag2_transport/record"*)
        printf '%s\n' "$pid"
        ;;
    esac
  done
}

signal_recorders() {
  local pid
  for pid in $(collect_recording_pids); do
    kill -INT "$pid" 2>/dev/null || true
  done
}

if [ "${1:-}" = "--recorders-only" ]; then
  signal_recorders
  for _ in 1 2 3 4 5 6; do
    sleep 2
    if ! collect_recording_pids | grep -q .; then
      echo "recorders clean"
      exit 0
    fi
  done
  echo "recorders did not stop gracefully" >&2
  exit 1
fi

# rosbag2 must receive SIGINT/SIGTERM and finish its writer before anything
# else is removed, otherwise metadata.yaml may never be written.
signal_pids -INT
sleep 5
signal_pids -TERM
sleep 3
signal_pids -KILL
sleep 2

remaining=0
for pid in $(collect_pids); do
  echo "runtime process still present: $pid"
  remaining=1
done

if [ "$remaining" -eq 0 ]; then
  echo "simulation runtime clean"
else
  echo "simulation runtime cleanup incomplete" >&2
  exit 1
fi
