#!/usr/bin/env bash
# tmux: PX4+Gazebo first, then MAVROS, camera bridge, then fly_pattern (waits for MAVROS + FCU).
# Linux + tmux; Windows: use docs/setup.md (three steps / headless / noVNC).
#
# Why this order: when it "worked", PX4 was up before MAVROS and the script.
# Old order (MAVROS → PX4 → fly) raced: fly could start before pxh> / param plugins.
set -euo pipefail

CONTAINER="${UAS_SIM_CONTAINER:-uas_sim}"
SESSION="${UAS_OBSTACLE_TMUX_SESSION:-uas_obstacle}"
PX4_GZ_MODEL_TARGET="${PX4_GZ_MODEL_TARGET:-gz_x500_mono_cam}"
PX4_GZ_WORLD="${PX4_GZ_WORLD:-earthen_heritage_wall}"
HEADLESS="${HEADLESS:-1}"
if [[ -n "${FLY_PATTERN_REQUIRE_DEPTH:-}" ]]; then
  REQUIRE_DEPTH="$FLY_PATTERN_REQUIRE_DEPTH"
elif [[ "$PX4_GZ_MODEL_TARGET" == *depth* ]]; then
  REQUIRE_DEPTH=1
else
  REQUIRE_DEPTH=0
fi
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

# Window 0: SITL + Gazebo (must be first)
tmux new-session -d -s "$SESSION" -n px4_gz "cd \"$ROOT\" && PX4_GZ_MODEL_TARGET=\"$PX4_GZ_MODEL_TARGET\" PX4_GZ_WORLD=\"$PX4_GZ_WORLD\" HEADLESS=$HEADLESS ./scripts/simulation/run_obstacle_flight.sh"

# Window 1: MAVROS
tmux new-window -t "$SESSION" -n mavros "docker exec -i \"$CONTAINER\" bash /home/uas/scripts/simulation/run_mavros.sh"

# Window 2: Gazebo camera/depth topics -> ROS 2 bridge
tmux new-window -t "$SESSION" -n camera_bridge "docker exec -i \"$CONTAINER\" bash /home/uas/scripts/simulation/camera_bridge_native.sh"

# Window 3: flight script (after waits)
if [[ "$NO_FLY" -eq 1 ]]; then
  tmux new-window -t "$SESSION" -n recorder "docker exec -i \"$CONTAINER\" bash -c 'source /opt/ros/humble/setup.bash && mkdir -p /home/uas/rosbags && cd /home/uas/rosbags && ros2 bag record -a'"
else
  tmux new-window -t "$SESSION" -n fly "docker exec -i \"$CONTAINER\" env FLY_PATTERN_REQUIRE_DEPTH=\"$REQUIRE_DEPTH\" bash /home/uas/scripts/simulation/run_fly_pattern.sh"
fi

tmux select-window -t "$SESSION:0"

echo "tmux session '$SESSION' — windows: 0=px4_gz ($PX4_GZ_MODEL_TARGET)  1=mavros  2=camera_bridge  3=fly."
echo "After pxh> appears: you can still set params manually if the script skips them."
echo "Switch: Ctrl-b 0|1|2   detach: Ctrl-b d   attach: tmux attach -t $SESSION"
if [[ "$ATTACH" -eq 1 ]]; then
  exec tmux attach-session -t "$SESSION"
fi
