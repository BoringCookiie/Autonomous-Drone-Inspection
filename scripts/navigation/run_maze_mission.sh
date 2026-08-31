#!/bin/bash
# run_maze_mission.sh — End-to-end maze navigation mission.
#
# Mission: Drone starts at (0,0,0) inside obstacle_maze.sdf.
#          Goal: (4.0, 7.0, 1.5) — outside the maze through the north gap.
#          A* planner finds path, path_follower executes it.
#
# Pipeline:
#   1. Start PX4 SITL + Gazebo (obstacle_maze world, depth camera model)
#   2. Start MAVROS (/uas1 namespace)
#   3. Start camera bridge (Gazebo → ROS topics)
#   4. Start rosbag recorder (curated topics)
#   5. Launch navigation stack (tf_bridge, A* planner, path_follower, OctoMap, RTAB-Map)
#   6. Run test harness (publish goal, monitor trajectory, log to CSV)
#   7. Export all analysis (CSVs, videos, trajectory plot, OctoMap PLY, world cloud)
#
# Usage:
#   ./scripts/navigation/run_maze_mission.sh
#
# Or with custom goal:
#   GOAL_X=5.0 GOAL_Y=8.0 GOAL_Z=2.0 ./scripts/navigation/run_maze_mission.sh
#
# Or from inside the container:
#   bash /home/uas/scripts/navigation/run_maze_mission.sh

set -euo pipefail

# ── Configuration ────────────────────────────────────────────────────────────
GOAL_X="${GOAL_X:-4.0}"
GOAL_Y="${GOAL_Y:-7.0}"
GOAL_Z="${GOAL_Z:-1.5}"
TIMEOUT="${TIMEOUT:-300}"
CONTAINER="${UAS_SIM_CONTAINER:-uas_sim}"
SESSION="${UAS_MAZE_TMUX_SESSION:-uas_maze}"
RUN_ID="maze_mission_$(date +%Y%m%d_%H%M%S)"
BAG_DIR="/home/uas/maze_results/bags"
CONTAINER_RESULTS_DIR="/home/uas/maze_results/$RUN_ID"
RESULTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../results" && pwd)/$RUN_ID"
PX4_GZ_WORLD="${PX4_GZ_WORLD:-obstacle_maze}"
PX4_GZ_MODEL_TARGET="${PX4_GZ_MODEL_TARGET:-gz_x500_depth}"
HEADLESS="${HEADLESS:-1}"
ENABLE_SLAM="${ENABLE_SLAM:-false}"

echo "============================================================"
echo "  MAZE NAVIGATION MISSION"
echo "============================================================"
echo "  World:     $PX4_GZ_WORLD"
echo "  Model:     $PX4_GZ_MODEL_TARGET"
echo "  Goal:      ($GOAL_X, $GOAL_Y, $GOAL_Z)"
echo "  Timeout:   ${TIMEOUT}s"
echo "  Results:   $RESULTS_DIR"
echo "============================================================"
echo ""

# Detect if we're inside the container or on the host
inside_container() {
    [ -f /home/uas/docker/bootstrap_px4.sh ] || [ -d /opt/ros/humble ]
}

if inside_container; then
    echo "[Mode] Running INSIDE container"
    source /opt/ros/humble/setup.bash
    export PYTHONPATH="${PYTHONPATH:-}:/home/uas/scripts"
    RUN_PREFIX=""
else
    echo "[Mode] Running from HOST via docker exec"
    RUN_PREFIX="docker exec $CONTAINER"
fi

cleanup_on_exit() {
    $RUN_PREFIX bash /home/uas/scripts/simulation/cleanup_runtime.sh 2>/dev/null || true
    tmux kill-session -t "$SESSION" 2>/dev/null || true
}
trap cleanup_on_exit EXIT INT TERM

# ── Step 1: Verify environment ───────────────────────────────────────────────
echo "[Step 1] Verifying environment..."

if ! inside_container; then
    if ! docker info >/dev/null 2>&1; then
        echo "[Error] Docker is not running."
        exit 1
    fi
    if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
        echo "[Info] Container not running. Starting via docker compose..."
        docker compose --project-directory . -f docker/docker-compose.yml --profile sim up -d --force-recreate sim_stack
        echo "[Info] Waiting for container..."
        sleep 15
    fi
fi

# Source ROS
$RUN_PREFIX bash -c "source /opt/ros/humble/setup.bash && echo '[OK] ROS environment ready'"

# ── Step 2: Clean previous runtime ───────────────────────────────────────────
echo "[Step 2] Cleaning previous runtime..."
$RUN_PREFIX bash /home/uas/scripts/simulation/cleanup_runtime.sh 2>/dev/null || true
$RUN_PREFIX bash -c "rm -rf /tmp/gz* /tmp/ign* /tmp/px4*" 2>/dev/null || true

# Kill old tmux session if exists
tmux kill-session -t "$SESSION" 2>/dev/null || true
echo "[OK] Cleaned"

# Static checks run after cleanup but before any PX4/Gazebo process is
# created. Runtime checks are then enforced by communication_preflight.py.
if ! inside_container; then
    scripts/navigation/preflight_navigation.sh
else
    /home/uas/scripts/navigation/preflight_navigation.sh
fi

# ── Step 3: Start simulation stack (PX4 + Gazebo) ────────────────────────────
echo "[Step 3] Starting PX4 SITL + Gazebo ($PX4_GZ_WORLD, $PX4_GZ_MODEL_TARGET)..."

tmux new-session -d -s "$SESSION" -n px4_gz \
    "cd '$(pwd)' && PX4_GZ_MODEL_TARGET='$PX4_GZ_MODEL_TARGET' PX4_GZ_WORLD='$PX4_GZ_WORLD' HEADLESS=$HEADLESS ./scripts/simulation/run_obstacle_flight.sh"

echo "[OK] PX4+Gazebo started in tmux window 0"

# ── Step 4: Start MAVROS ─────────────────────────────────────────────────────
echo "[Step 4] Starting MAVROS..."
tmux new-window -t "$SESSION" -n mavros \
    "docker exec -i '$CONTAINER' bash /home/uas/scripts/simulation/run_mavros.sh"
echo "[OK] MAVROS started in tmux window 1"

# ── Step 5: Wait for MAVROS telemetry ────────────────────────────────────────
echo "[Step 5] Waiting for MAVROS telemetry (/uas1/state)..."
READY=0
for i in $(seq 1 300); do
    if $RUN_PREFIX bash -c "source /opt/ros/humble/setup.bash && ros2 topic list 2>/dev/null" 2>/dev/null | grep -q "/uas1/state"; then
        echo "[OK] MAVROS ready (${i}s)"
        READY=1
        break
    fi
    if [ "$((i % 10))" -eq 0 ]; then echo -n "."; fi
    sleep 1
done

if [ "$READY" -eq 0 ]; then
    echo "[Error] MAVROS did not start. Check tmux window 1 (mavros)."
    echo "  tmux attach -t $SESSION"
    exit 1
fi

# ── Step 6: Start camera bridge ──────────────────────────────────────────────
echo "[Step 6] Starting camera bridge..."
tmux new-window -t "$SESSION" -n camera_bridge \
    "docker exec -i '$CONTAINER' bash /home/uas/scripts/simulation/camera_bridge_native.sh"
echo "[OK] Camera bridge started in tmux window 2"

# Wait for camera topics
echo "  Waiting for /camera/color/image_raw..."
CAM_READY=0
for i in $(seq 1 60); do
    if $RUN_PREFIX bash -c "source /opt/ros/humble/setup.bash && ros2 topic list 2>/dev/null" 2>/dev/null | grep -q "/camera/color/image_raw"; then
        echo "  [OK] Camera topic ready"
        CAM_READY=1
        break
    fi
    sleep 1
done

# ── Step 7: Start rosbag recorder ────────────────────────────────────────────
echo "[Step 7] Starting rosbag recorder..."
$RUN_PREFIX bash -c "
    source /opt/ros/humble/setup.bash
    export PYTHONPATH=\${PYTHONPATH:-}:/home/uas/scripts
    mkdir -p $BAG_DIR
    BAG_STAMP=\$(date +%Y%m%d_%H%M%S)
    BAG_PATH=\"$BAG_DIR/fly_pattern_\${BAG_STAMP}\"
    echo \"\$BAG_PATH\" > $BAG_DIR/.maze_bag_path

    ros2 bag record -o \"\$BAG_PATH\" \\
        /clock \\
        /uas1/state /uas1/extended_state \\
        /uas1/local_position/pose /uas1/local_position/odom \\
        /uas1/local_position/velocity_local \\
        /uas1/imu/data /uas1/imu/data_raw \\
        /uas1/global_position/global /uas1/global_position/rel_alt \\
        /uas1/battery /uas1/altitude \\
        /uas1/setpoint_position/local \\
        /camera/color/image_raw /camera/color/camera_info \\
        /camera/depth/image_raw \\
        /points /points_clean \\
        /octomap_point_cloud_centers \\
        /planned_path /planner/coverage_path \\
        /path_follower/status \\
        /navigation/goal \\
        /tf /tf_static \\
        >/tmp/maze_rosbag.log 2>&1 &
    echo \"[OK] Recording to \$BAG_PATH\"
"
echo "[OK] Rosbag recorder started"

# ── Step 8: Launch navigation stack ──────────────────────────────────────────
echo "[Step 8] Launching navigation stack..."
$RUN_PREFIX bash -c "
    source /opt/ros/humble/setup.bash
    export PYTHONPATH=\${PYTHONPATH:-}:/home/uas/scripts
    export FASTRTPS_DEFAULT_PROFILES_FILE=/home/uas/fastdds_udp.xml
    ros2 launch /home/uas/scripts/navigation/navigation_launch.py \\
        has_depth:=true \\
        enable_coverage_planner:=false \\
        enable_slam:=$ENABLE_SLAM \\
        takeoff_x:=0.0 takeoff_y:=0.0 takeoff_z:=2.0 \\
        queue_planned_paths:=false &
    echo \$! > /tmp/maze_nav_pid
"
echo "[OK] Navigation stack launched"

# Wait for nav nodes
echo "  Waiting for nav nodes to initialize..."
sleep 10

NAV_NODES=("/planner_3d" "/path_follower" "/tf_bridge" "/octomap_server")
ALL_OK=1
for node in "${NAV_NODES[@]}"; do
    if $RUN_PREFIX bash -c "source /opt/ros/humble/setup.bash && ros2 node list 2>/dev/null" 2>/dev/null | grep -q "^${node}$"; then
        echo "  [OK] $node"
    else
        echo "  [WARN] $node not found yet (may still be starting)"
        ALL_OK=0
    fi
done

if [ "$ALL_OK" -eq 0 ]; then
    echo "  Waiting additional 10s for remaining nodes..."
    sleep 10
fi

# ── Step 9: Run test harness (publish goal + monitor) ────────────────────────
echo ""
echo "============================================================"
echo "  Step 9: RUNNING MAZE NAVIGATION TEST"
echo "  Goal: ($GOAL_X, $GOAL_Y, $GOAL_Z)"
echo "  Timeout: ${TIMEOUT}s"
echo "============================================================"
echo ""

TEST_EXIT=1
$RUN_PREFIX bash -c "
    source /opt/ros/humble/setup.bash
    export PYTHONPATH=\${PYTHONPATH:-}:/home/uas/scripts

    python3 /home/uas/scripts/navigation/maze_navigation_test.py \\
        --goal-x $GOAL_X --goal-y $GOAL_Y --goal-z $GOAL_Z \\
        --timeout $TIMEOUT \\
        --trajectory-csv '$BAG_DIR/maze_trajectory.csv'
" && TEST_EXIT=0 || TEST_EXIT=$?

echo ""
echo "============================================================"
if [ "$TEST_EXIT" -eq 0 ]; then
    echo "  MISSION RESULT: SUCCESS — Drone reached the goal!"
else
    echo "  MISSION RESULT: Drone did not reach goal within timeout"
    echo "  (check trajectory CSV for closest approach)"
fi
echo "============================================================"
echo ""

# ── Step 10: Export analysis ─────────────────────────────────────────────────
echo "[Step 10] Running analysis pipeline..."

# Stop rosbag recorder cleanly. Do not kill unrelated ROS processes.
$RUN_PREFIX bash /home/uas/scripts/simulation/cleanup_runtime.sh --recorders-only 2>/dev/null || true

# Get the bag path
BAG_PATH=$($RUN_PREFIX bash -c "cat $BAG_DIR/.maze_bag_path 2>/dev/null || echo ''")
BAG_PATH=$(echo "$BAG_PATH" | tr -d '\r\n' | xargs)

if [ -z "$BAG_PATH" ] || [ "$BAG_PATH" = "" ]; then
    echo "[WARN] Could not determine bag path, using latest"
    BAG_PATH="latest"
fi

echo "[Info] Analyzing bag: $BAG_PATH"

# Run the analysis
OUTPUT_DIR="$CONTAINER_RESULTS_DIR/analysis"
$RUN_PREFIX bash -c "
    source /opt/ros/humble/setup.bash
    export PYTHONPATH=\${PYTHONPATH:-}:/home/uas/scripts
    mkdir -p '$OUTPUT_DIR'

    # Step 1: CSVs + RGB video
    echo '[Analysis] Exporting CSVs and RGB video...'
    python3 /home/uas/scripts/analysis/analyze_rosbags.py \\
        --bag '$BAG_PATH' \\
        --export-csv --export-video \\
        --video-topic '/camera/color/image_raw' \\
        --export-dir '$OUTPUT_DIR' 2>&1 || true

    # Step 2: Depth video
    echo '[Analysis] Exporting depth video...'
    python3 /home/uas/scripts/analysis/export_depth_video.py \\
        '$BAG_PATH' --every 3 \\
        -o '$OUTPUT_DIR/depth_sensor_view.mp4' 2>&1 || true

    # Step 3: Trajectory plot
    if [ -f '$OUTPUT_DIR/local_position.csv' ]; then
        echo '[Analysis] Generating trajectory plot...'
        python3 /home/uas/scripts/analysis/plot_trajectory.py \\
            '$OUTPUT_DIR/local_position.csv' \\
            '$OUTPUT_DIR/trajectory.png' 2>&1 || true
    fi

    echo '[Analysis] Exports complete.'
"

# Step 4: Try OctoMap capture (if sim still running)
echo "[Analysis] Capturing OctoMap..."
$RUN_PREFIX bash -c "
    source /opt/ros/humble/setup.bash
    timeout 20 python3 /home/uas/scripts/analysis/octomap_to_ply.py \
        '$OUTPUT_DIR/octomap_world.ply' 2>&1 || true
"

# Step 5: World cloud fusion
echo "[Analysis] Fusing RGB+Depth world cloud..."
$RUN_PREFIX bash -c "
    source /opt/ros/humble/setup.bash
    export PYTHONPATH=\${PYTHONPATH:-}:/home/uas/scripts
    timeout 300 python3 /home/uas/scripts/analysis/build_world_cloud.py \
        '$BAG_PATH' '$OUTPUT_DIR/world_cloud.ply' \
        --stride 8 2>&1 || true
"

# Copy trajectory CSV into the same container-side export directory.
$RUN_PREFIX bash -c "cp '$BAG_DIR/maze_trajectory.csv' '$OUTPUT_DIR/maze_trajectory_live.csv' 2>/dev/null || true"

# The repository is mounted read-only in the sim container. Copy the completed
# export back to the host only after all writers have closed their files.
if ! inside_container; then
    mkdir -p "$RESULTS_DIR"
    docker cp "$CONTAINER:$CONTAINER_RESULTS_DIR" "$RESULTS_DIR/.."
fi

# ── Final Summary ────────────────────────────────────────────────────────────
echo ""
echo "============================================================"
echo "  MISSION COMPLETE"
echo "============================================================"
echo ""

if inside_container; then
    RESULTS_LIST=$(ls -lh "$OUTPUT_DIR/" 2>/dev/null || echo '  (empty)')
else
    RESULTS_LIST=$(ls -lh "$RESULTS_DIR/analysis/" 2>/dev/null || echo '  (empty)')
fi
echo "Artifacts in: $RESULTS_DIR/analysis/"
echo "$RESULTS_LIST"

# Print trajectory stats if available
TRAJ_CSV="$OUTPUT_DIR/local_position.csv"
if $RUN_PREFIX bash -c "test -f '$TRAJ_CSV'" 2>/dev/null; then
    echo ""
    echo "--- Flight Statistics ---"
    $RUN_PREFIX bash -c "
        tail -n +2 '$TRAJ_CSV' | awk -F, '
        BEGIN{min_x=999;max_x=-999;min_y=999;max_y=-999;min_z=999;max_z=-999;max_dist=0;count=0}
        {
            x=\$2;y=\$3;z=\$4;
            if(x<min_x)min_x=x; if(x>max_x)max_x=x;
            if(y<min_y)min_y=y; if(y>max_y)max_y=y;
            if(z<min_z)min_z=z; if(z>max_z)max_z=z;
            count++
        }
        END{
            printf \"  Duration:     %.1f s\n\", \$1
            printf \"  Samples:      %d\n\", count
            printf \"  X range:      %.2f to %.2f m\n\", min_x, max_x
            printf \"  Y range:      %.2f to %.2f m (maze exit at y=4.75)\n\", min_y, max_y
            printf \"  Z range:      %.2f to %.2f m\n\", min_z, max_z
            if(max_y > 4.75)
                printf \"  *** DRONE EXITED THE MAZE ***\n\"
            else if(max_y > 3.5)
                printf \"  *** DRONE APPROACHED EXIT (y=%.2f) ***\n\", max_y
            else
                printf \"  *** DRONE DID NOT REACH EXIT ***\n\"
        }'
    " 2>/dev/null || true
fi

echo ""
echo "To view tmux session: tmux attach -t $SESSION"
echo "To stop simulation:  bash scripts/simulation/stop_simulation.sh"
echo ""

# Leave the host in a clean state after exports. All bag writers and map
# readers have completed by this point.
$RUN_PREFIX bash /home/uas/scripts/simulation/cleanup_runtime.sh 2>/dev/null || true
tmux kill-session -t "$SESSION" 2>/dev/null || true
