#!/bin/bash
# run_maze_mission_v2.sh
# Launches full autonomous maze navigation stack.
#
# Env overrides: GOAL_X GOAL_Y GOAL_Z TIMEOUT HEADLESS
#                UAS_SIM_CONTAINER UAS_MAZE_TMUX_SESSION
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CONTAINER="${UAS_SIM_CONTAINER:-uas_sim}"
SESSION="${UAS_MAZE_TMUX_SESSION:-uas_maze}"
GOAL_X="${GOAL_X:-4.0}"
GOAL_Y="${GOAL_Y:-7.0}"
GOAL_Z="${GOAL_Z:-1.5}"
TIMEOUT="${TIMEOUT:-400}"
HEADLESS="${HEADLESS:-1}"
TS="$(date +%Y%m%d_%H%M%S)"
RESULTS_DIR="$ROOT/results/maze_$TS"
CONTAINER_RESULTS="/home/uas/maze_results/mission_$TS"
CONTAINER_CSV="/home/uas/maze_results/trajectory_$TS.csv"

echo "============================================================"
echo "  MAZE NAVIGATION MISSION v2"
echo "============================================================"
echo "  Goal    : ($GOAL_X, $GOAL_Y, $GOAL_Z)"
echo "  Timeout : ${TIMEOUT}s"
echo "  Results : $RESULTS_DIR"
echo "============================================================"

# ── Cleanup on exit ────────────────────────────────────────────────────────
cleanup() {
    echo "[Cleanup] Stopping simulation..."
    docker exec "$CONTAINER" bash -c "pkill -SIGINT -f 'ros2 bag record' 2>/dev/null || true"
    sleep 1
    docker exec "$CONTAINER" bash /home/uas/scripts/simulation/cleanup_runtime.sh 2>/dev/null || true
    tmux kill-session -t "$SESSION" 2>/dev/null || true
}
trap cleanup EXIT
trap 'cleanup; exit 130' INT TERM

# ── Step 1: Kill everything and start fresh ────────────────────────────────
echo "[1/8] Cleaning previous runtime..."
tmux kill-session -t "$SESSION" 2>/dev/null || true
# Use bracket trick to avoid pkill matching its own command line
docker exec "$CONTAINER" bash -c "pkill -9 -f '[m]aze_navigator' 2>/dev/null || true; pkill -9 -f '[m]aze_navigation_test' 2>/dev/null || true"
docker exec "$CONTAINER" bash -c "pkill -SIGINT -f 'ros2 bag record' 2>/dev/null || true"
docker exec "$CONTAINER" bash /home/uas/scripts/simulation/cleanup_runtime.sh 2>/dev/null || true
# Defense in depth: ensure no duplicate setpoint publishers survive
docker exec "$CONTAINER" bash -c "pkill -9 -f '[m]aze_navigator' 2>/dev/null || true; sleep 1; pgrep -a python3 2>/dev/null | grep -E '[m]aze_navigator|[m]aze_navigation_test' || echo '[OK] No stale navigator'"
sleep 3   # let OS fully release ports and processes

# ── Step 2: Start PX4 + Gazebo ────────────────────────────────────────────
echo "[2/8] Starting PX4 + Gazebo (obstacle_maze, gz_x500_depth)..."
tmux new-session -d -s "$SESSION" -n px4_gz \
    "cd '$ROOT' && PX4_GZ_MODEL_TARGET=gz_x500_depth PX4_GZ_WORLD=obstacle_maze PX4_GZ_MODEL_POSE='0,0,0.15,0,0,0' HEADLESS=$HEADLESS ./scripts/simulation/run_obstacle_flight.sh"

# ── Step 3: Wait for Gazebo + PX4 EKF2 to initialise ─────────────────────
# 30s: PX4 boots (~10s) + EKF2 initial convergence (~10-20s)
# The navigator also waits for 3s of position stability before arming.
echo "[3/8] Waiting for Gazebo + PX4 EKF2 init (30s)..."
sleep 30

# ── Step 4: Start MAVROS ──────────────────────────────────────────────────
echo "[4/8] Starting MAVROS..."
tmux new-window -t "$SESSION" -n mavros \
    "docker exec -i '$CONTAINER' bash /home/uas/scripts/simulation/run_mavros.sh"

echo "  Waiting for MAVROS /uas1/state topic..."
for i in $(seq 1 60); do
    if docker exec "$CONTAINER" bash -c \
        "source /opt/ros/humble/setup.bash && ros2 topic list 2>/dev/null" \
        | grep -q "/uas1/state"; then
        echo "  MAVROS ready (${i}s)"
        break
    fi
    sleep 1
done

# ── Step 5: Start camera bridge ───────────────────────────────────────────
echo "[5/8] Starting camera bridge..."
tmux new-window -t "$SESSION" -n camera \
    "docker exec -i '$CONTAINER' bash /home/uas/scripts/simulation/camera_bridge_native.sh"
sleep 5

# ── Step 6: Start navigation components ──────────────────────────────────
echo "[6/8] Starting navigation components..."
tmux new-window -t "$SESSION" -n nav \
    "docker exec -i '$CONTAINER' bash -c '
source /opt/ros/humble/setup.bash
export PYTHONPATH=\${PYTHONPATH:-}:/home/uas/scripts
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/uas/fastdds_udp.xml

echo \"[nav] tf_bridge...\"
python3 /home/uas/scripts/navigation/tf_bridge_node.py \
    --ros-args -p use_sim_time:=true &
TF_PID=\$!
sleep 3

echo \"[nav] depth_cloud_sanitizer...\"
python3 /home/uas/scripts/navigation/depth_cloud_sanitizer.py \
    --ros-args -p use_sim_time:=true &
sleep 2

echo \"[nav] octomap_server...\"
ros2 run octomap_server octomap_server_node \
    --ros-args -p use_sim_time:=true \
              -p frame_id:=odom \
              -p resolution:=0.2 \
              -p base_frame_id:=base_link \
              -p sensor_model.max_range:=5.0 \
    -r cloud_in:=/points_clean &
sleep 3

echo \"[nav] planner_3d (inflation=2, margin=15m, 800k nodes)...\"
python3 /home/uas/scripts/navigation/planner_3d.py \
    --ros-args -p use_sim_time:=true \
              -p inflation_radius:=1 \
              -p search_margin_m:=15.0 \
              -p max_search_nodes:=800000 &
echo \"[nav] All nav components started\"
wait
'"

echo "  Waiting 15s for nav nodes to settle..."
sleep 15

# Verify nodes started
for node in /planner_3d /tf_bridge /octomap_server; do
    if docker exec "$CONTAINER" bash -c \
        "source /opt/ros/humble/setup.bash && ros2 node list 2>/dev/null" \
        | grep -q "^${node}$"; then
        echo "  [OK] $node"
    else
        echo "  [WARN] $node not found — nav may be degraded"
    fi
done

# ── Step 7: Rosbag (curated topics: no raw camera → prevents 80GB bags) ─────
echo "[7/8] Starting rosbag recorder (curated sensor topics)..."
# Use docker exec -d (detached) instead of tmux to avoid quoting hell and ensure bag starts
docker exec -d "$CONTAINER" bash -c '
source /opt/ros/humble/setup.bash
mkdir -p /home/uas/maze_results/bags
BAG=/home/uas/maze_results/bags/maze_$(date +%Y%m%d_%H%M%S)
echo "$BAG" > /home/uas/maze_results/bags/.maze_bag_path
echo "Rosbag: $BAG" | tee /tmp/rosbag_path.log
ros2 bag record -o "$BAG" \
    /clock \
    /uas1/state /uas1/extended_state \
    /uas1/local_position/pose /uas1/local_position/odom \
    /uas1/local_position/velocity_local \
    /uas1/imu/data /uas1/imu/data_raw \
    /uas1/global_position/global /uas1/global_position/rel_alt \
    /uas1/battery /uas1/altitude \
    /uas1/setpoint_position/local \
    /points_clean \
    /octomap_point_cloud_centers \
    /planned_path \
    /navigation/goal \
    /tf /tf_static
' 2>&1
sleep 3
# Verify rosbag started
if docker exec "$CONTAINER" bash -c "pgrep -f 'ros2 bag record' >/dev/null"; then
    echo "  [OK] Rosbag recording"
else
    echo "  [WARN] Rosbag not found, retrying..."
    docker exec -d "$CONTAINER" bash -c '
    source /opt/ros/humble/setup.bash
    BAG=$(cat /home/uas/maze_results/bags/.maze_bag_path 2>/dev/null || echo "/home/uas/maze_results/bags/maze_$(date +%Y%m%d_%H%M%S)")
    ros2 bag record -o "$BAG" /clock /uas1/state /uas1/local_position/pose /uas1/imu/data /uas1/global_position/global /uas1/battery /tf &
    '
    sleep 2
fi
# Create a dummy tmux window for consistency (so cleanup can find it)
tmux new-window -t "$SESSION" -n rosbag "echo 'rosbag via docker exec -d'; sleep 400" 2>/dev/null || true
# NOTE: raw camera topics excluded intentionally (1-2 GB/min each)
sleep 2

# ── Step 8: Run maze navigator ────────────────────────────────────────────
echo "[8/8] Running maze navigator..."
echo ""
echo "============================================================"
echo "  MISSION IN PROGRESS  goal=($GOAL_X,$GOAL_Y,$GOAL_Z)"
echo "============================================================"
echo ""

MISSION_EXIT=1
docker exec "$CONTAINER" bash -c "
source /opt/ros/humble/setup.bash
export PYTHONPATH=\${PYTHONPATH:-}:/home/uas/scripts
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/uas/fastdds_udp.xml
python3 /home/uas/scripts/navigation/maze_navigator.py \
    --ros-args -p use_sim_time:=true \
    -- \
    --goal-x $GOAL_X --goal-y $GOAL_Y --goal-z $GOAL_Z \
    --timeout $TIMEOUT \
    --csv $CONTAINER_CSV
" && MISSION_EXIT=0 || MISSION_EXIT=$?

echo ""
echo "============================================================"
case "$MISSION_EXIT" in
    0) echo "  RESULT: SUCCESS — Drone reached ($GOAL_X,$GOAL_Y,$GOAL_Z)!" ;;
    2) echo "  RESULT: FAULT  — Envelope violation / crash" ;;
    *) echo "  RESULT: FAILED — exit $MISSION_EXIT" ;;
esac
echo "============================================================"

# ── Step 9: Export ─────────────────────────────────────────────────────────
echo ""
echo "[Analysis] Stopping rosbag..."
docker exec "$CONTAINER" bash -c "pkill -SIGINT -f 'ros2 bag record' 2>/dev/null || true"
sleep 3

BAG_PATH=$(docker exec "$CONTAINER" bash -c \
    "cat /home/uas/maze_results/bags/.maze_bag_path 2>/dev/null || ls -dt /home/uas/maze_results/bags/maze_* 2>/dev/null | head -1" | tr -d '\r\n')
BAG_PATH=$(echo "$BAG_PATH" | xargs)

if [ -n "$BAG_PATH" ]; then
    mkdir -p "$RESULTS_DIR"
    docker exec "$CONTAINER" bash -c "
    source /opt/ros/humble/setup.bash
    export PYTHONPATH=\${PYTHONPATH:-}:/home/uas/scripts
    echo \"[Analysis] Bag: $BAG_PATH\"
    ls -lh \"$BAG_PATH\" 2>&1 | head -5
    python3 /home/uas/scripts/analysis/analyze_rosbags.py \
        --bag '$BAG_PATH' --export-csv \
        --export-dir '$CONTAINER_RESULTS/analysis' 2>&1 || true
    echo \"[Analysis] CSVs done, plotting trajectory...\"
    python3 /home/uas/scripts/analysis/plot_trajectory.py \
        '$CONTAINER_RESULTS/analysis/local_position.csv' \
        '$CONTAINER_RESULTS/analysis/trajectory.png' 2>&1 || true
    # also export depth video if points available (optional, not required for GPS proof)
    # ensure nav trajectory is copied even if bag analysis failed
    cp '$CONTAINER_CSV' '$CONTAINER_RESULTS/analysis/nav_trajectory.csv' 2>/dev/null || true
    ls -lh '$CONTAINER_RESULTS/analysis/' 2>&1 | head -20
    "
    echo "[Host] Copying results..."
    docker cp "$CONTAINER:$CONTAINER_RESULTS" "$RESULTS_DIR/.." 2>/dev/null || true
    # also copy bag path for debugging
    echo "$BAG_PATH" > "$RESULTS_DIR/bag_path.txt" 2>/dev/null || true
fi

echo ""
echo "Results: $RESULTS_DIR/"
ls -lh "$RESULTS_DIR/analysis/" 2>/dev/null || echo "  (no analysis dir)"
echo "Done. exit=$MISSION_EXIT"
