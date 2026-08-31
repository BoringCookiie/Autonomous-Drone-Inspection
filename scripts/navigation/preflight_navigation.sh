#!/usr/bin/env bash
# Static preflight. This runs before Gazebo/PX4 is started. The runtime
# communication_preflight.py performs the live ROS/T.F./sensor checks after
# the simulator is up and before the vehicle is allowed to arm.
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CONTAINER="${UAS_SIM_CONTAINER:-uas_sim}"

failures=()
require_file() {
  if [[ ! -f "$1" ]]; then failures+=("missing file: $1"); fi
}
require_executable() {
  if [[ ! -x "$1" ]]; then failures+=("not executable: $1"); fi
}

require_file "$ROOT/gazebo_simulation/worlds/obstacle_maze.sdf"
require_file "$ROOT/gazebo_simulation/models/x500_depth/model.sdf"
require_file "$ROOT/scripts/navigation/navigation_launch.py"
require_file "$ROOT/scripts/navigation/communication_preflight.py"
require_file "$ROOT/scripts/navigation/planner_3d.py"
require_file "$ROOT/scripts/navigation/path_follower.py"
require_file "$ROOT/scripts/navigation/tf_bridge_node.py"
require_executable "$ROOT/scripts/simulation/run_obstacle_flight.sh"
require_executable "$ROOT/scripts/simulation/camera_bridge_native.sh"

if ! grep -q 'sensor name="rgbd_camera"' "$ROOT/gazebo_simulation/models/x500_depth/model.sdf"; then
  failures+=("x500_depth model has no rgbd_camera sensor")
fi
if ! grep -q 'world name="obstacle_maze"' "$ROOT/gazebo_simulation/worlds/obstacle_maze.sdf"; then
  failures+=("obstacle_maze.sdf is not the expected world")
fi

if ! docker info >/dev/null 2>&1; then
  failures+=("Docker daemon unavailable")
elif ! docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null | grep -qx true; then
  failures+=("container $CONTAINER is not running")
else
  for command in ros2 gz python3; do
    if ! docker exec "$CONTAINER" bash -c "source /opt/ros/humble/setup.bash && command -v $command" >/dev/null 2>&1; then
      failures+=("container command unavailable: $command")
    fi
  done
  if ! docker exec "$CONTAINER" bash -c "source /opt/ros/humble/setup.bash && python3 -c 'import rclpy, sensor_msgs_py, tf2_ros'" >/dev/null 2>&1; then
    failures+=("required ROS Python modules unavailable")
  fi
  if docker exec "$CONTAINER" bash -c "ps -eo args" | grep -Eq 'bin/px4|gz sim|mavros_node|navigation_launch.py|planner_3d.py|path_follower.py'; then
    failures+=("project runtime already active; stop it before launching a new simulation")
  fi
fi

if ((${#failures[@]})); then
  printf '[preflight] FAIL: %s\n' "${failures[@]}" >&2
  exit 1
fi

echo '[preflight] Static checks passed. Runtime gate will be required before arming.'
