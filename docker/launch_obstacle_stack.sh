#!/usr/bin/env bash
# tmux: PX4+Gazebo first, then MAVROS, camera bridge, then fly_pattern (waits for MAVROS + FCU).
# Linux + tmux; Windows: use docs/setup.md (three steps / headless / noVNC).
#
# Why this order: when it "worked", PX4 was up before MAVROS and the script.
# Old order (MAVROS → PX4 → fly) raced: fly could start before pxh> / param plugins.
set -euo pipefail

CONTAINER="${UAS_SIM_CONTAINER:-uas_sim}"
SESSION="${UAS_OBSTACLE_TMUX_SESSION:-uas_obstacle}"
PX4_GZ_MODEL_TARGET="${PX4_GZ_MODEL_TARGET:-gz_x500_depth}"
PX4_GZ_WORLD="${PX4_GZ_WORLD:-earthen_heritage_wall}"
HEADLESS="${HEADLESS:-1}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ATTACH=1
NO_FLY=0
for arg in "$@"; do
  case "$arg" in
    --no-attach) ATTACH=0 ;;
    --no-fly) NO_FLY=1 ;;
    --gui) HEADLESS=0 ;;
    --headless) HEADLESS=1 ;;
  esac
done

if ! docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null | grep -qx true; then
  echo "Container '$CONTAINER' is not running. Start it first: cd '$ROOT' && docker compose --project-directory . -f docker/docker-compose.yml --profile sim up -d --force-recreate sim_stack"
  exit 1
fi

if ! command -v tmux >/dev/null 2>&1; then
  echo "tmux required. Install it, or launch components manually (see docs/setup.md)."
  exit 1
fi

# Clean any leftover processes inside the container before launching new stack
docker exec "$CONTAINER" bash -c "pkill -9 -f 'mavros|px4|gz|parameter_bridge|gz_camera_bridge|fly_pattern' || true; rm -rf /tmp/gz* /tmp/ign* /tmp/px4*" 2>/dev/null || true

if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "Session '$SESSION' already exists. Attach: tmux attach -t $SESSION"
  echo "Or replace: tmux kill-session -t $SESSION && $0"
  exit 1
fi

# MAVROS: fixed ROS namespace /uas1 so services are always /uas1/... (matches fly_pattern default).
# Short delay so PX4's onboard MAVLink port is listening before connect.
mavros_inner="export FASTRTPS_DEFAULT_PROFILES_FILE=/home/uas/fastdds_udp.xml && source /opt/ros/humble/setup.bash && sleep 5 && until ros2 run mavros mavros_node --ros-args -r __ns:=/uas1 -p fcu_url:=udp://:14540@127.0.0.1:14580; do echo '[mavros] MAVROS exited, restarting in 2s...'; sleep 2; done"

# Camera bridge: Gazebo Harmonic -> ROS 2 image and depth bridge.
camera_bridge_inner="export FASTRTPS_DEFAULT_PROFILES_FILE=/home/uas/fastdds_udp.xml && source /opt/ros/humble/setup.bash && until python3 /home/uas/scripts/simulation/gz_camera_ros_bridge.py; do echo '[camera_bridge] Bridge exited, restarting in 2s...'; sleep 2; done"

# Wait only for /uas1 (do not treat a stray /mavros as "ready"), then camera topic, then FCU link.
fly_inner="export FASTRTPS_DEFAULT_PROFILES_FILE=/home/uas/fastdds_udp.xml && source /opt/ros/humble/setup.bash && export MAVROS_NS=\"\${MAVROS_NS:-/uas1}\" && bash -c '
set -e
for i in \$(seq 1 180); do
  if ros2 service list 2>/dev/null | grep -qF \"/uas1/cmd/arming\"; then
    echo \"[fly] /uas1/cmd/arming is up\"
    break
  fi
  sleep 1
done
for i in \$(seq 1 60); do
  if ros2 topic list 2>/dev/null | grep -qE \"/camera|/depth_camera\"; then
    echo \"[fly] Camera topics are up\"
    break
  fi
  sleep 1
done
echo \"[fly] Waiting for FCU link (heartbeat)...\"
sleep 10
export MAVROS_NS=\"\${MAVROS_NS:-/uas1}\"
python3 /home/uas/scripts/simulation/fly_pattern.py
echo \"[fly] Flight pattern complete. Exporting telemetry CSV and camera MP4 video...\"
python3 /home/uas/scripts/analysis/analyze_rosbags.py --bag latest --export-csv --export-video
'"

# Window 0: SITL + Gazebo (must be first)
tmux new-session -d -s "$SESSION" -n px4_gz "cd $(printf %q "$ROOT") && PX4_GZ_MODEL_TARGET=$(printf %q "$PX4_GZ_MODEL_TARGET") PX4_GZ_WORLD=$(printf %q "$PX4_GZ_WORLD") HEADLESS=$HEADLESS ./scripts/simulation/run_obstacle_flight.sh"

# Window 1: MAVROS
tmux new-window -t "$SESSION" -n mavros "docker exec -i \"$CONTAINER\" bash -c $(printf %q "$mavros_inner")"

# Window 2: Gazebo camera/depth topics -> ROS 2 bridge
tmux new-window -t "$SESSION" -n camera_bridge "docker exec -i \"$CONTAINER\" bash -c $(printf %q "$camera_bridge_inner")"

# Window 3: flight script (after waits)
if [[ "$NO_FLY" -eq 1 ]]; then
  bag_inner="source /opt/ros/humble/setup.bash && mkdir -p /home/uas/rosbags && cd /home/uas/rosbags && ros2 bag record -a"
  tmux new-window -t "$SESSION" -n recorder "docker exec -i \"$CONTAINER\" bash -c $(printf %q "$bag_inner")"
else
  tmux new-window -t "$SESSION" -n fly "docker exec -i \"$CONTAINER\" bash -c $(printf %q "$fly_inner")"
fi

tmux select-window -t "$SESSION:0"

echo "tmux session '$SESSION' — windows: 0=px4_gz ($PX4_GZ_MODEL_TARGET)  1=mavros  2=camera_bridge  3=fly."
echo "After pxh> appears: you can still set params manually if the script skips them."
echo "Switch: Ctrl-b 0|1|2   detach: Ctrl-b d   attach: tmux attach -t $SESSION"
if [[ "$ATTACH" -eq 1 ]]; then
  exec tmux attach-session -t "$SESSION"
fi
