#!/bin/bash
# run_maze_analysis.sh — Post-flight analysis for the maze navigation test.
#
# Exports:
#   1. CSVs (position, velocity, IMU, state, battery) from rosbag
#   2. RGB camera video (MP4)
#   3. Depth sensor video (Turbo colormap, H.264)
#   4. Flight trajectory plot (top-down, altitude, X/Y vs time)
#   5. OctoMap point cloud (PLY)
#   6. Fused RGB+Depth world cloud (PLY)
#
# Usage (inside container or host with docker exec):
#   bash /home/uas/scripts/analysis/run_maze_analysis.sh [bag_name] [output_dir]
#
# If bag_name is omitted, uses the latest bag under /home/uas/rosbags.
# If output_dir is omitted, uses /home/uas/rosbags/<bag_name>/analysis.

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAG_DIR="/home/uas/rosbags"
CONTAINER="${UAS_SIM_CONTAINER:-uas_sim}"

# Determine if we're inside the container or on the host
inside_container() {
    [ -f /home/uas/docker/bootstrap_px4.sh ] || [ -d /opt/ros/humble ]
}

BAG_NAME="${1:-latest}"
OUTPUT_DIR_OVERRIDE="${2:-}"

echo "============================================="
echo "  MAZE NAVIGATION ANALYSIS PIPELINE"
echo "============================================="
echo ""

# Source ROS if inside container
if inside_container; then
    source /opt/ros/humble/setup.bash
    export PYTHONPATH="${PYTHONPATH:-}:/home/uas/scripts"
    RUN_CMD=""
    echo "[Info] Running inside container"
else
    RUN_CMD="docker exec $CONTAINER bash -c"
    echo "[Info] Running on host via docker exec"
fi

# Resolve bag path
if [ "$BAG_NAME" = "latest" ]; then
    echo "[Step 0] Finding latest rosbag..."
    BAG_PATH=$($RUN_CMD "find $BAG_DIR -maxdepth 1 -name 'fly_pattern_*' -type d | sort | tail -n 1")
    if [ -z "$BAG_PATH" ]; then
        BAG_PATH=$($RUN_CMD "find $BAG_DIR -maxdepth 2 -name 'metadata.yaml' -type f | sort | tail -n 1 | xargs dirname")
    fi
    if [ -z "$BAG_PATH" ]; then
        echo "[Error] No rosbag found under $BAG_DIR"
        exit 1
    fi
    echo "[Info] Using bag: $BAG_PATH"
else
    BAG_PATH="$BAG_DIR/$BAG_NAME"
fi

# Resolve output directory
if [ -n "$OUTPUT_DIR_OVERRIDE" ]; then
    ANALYSIS_DIR="$OUTPUT_DIR_OVERRIDE"
else
    ANALYSIS_DIR="${BAG_PATH}/analysis"
fi

echo "[Info] Output directory: $ANALYSIS_DIR"
echo ""

# ============================================================
# Step 1: Rosbag analysis — CSVs + RGB video
# ============================================================
echo "[Step 1/6] Exporting CSVs and RGB video from rosbag..."
$RUN_CMD "source /opt/ros/humble/setup.bash && export PYTHONPATH=\${PYTHONPATH:-}:/home/uas/scripts && python3 /home/uas/scripts/analysis/analyze_rosbags.py --bag '$BAG_PATH' --export-csv --export-video --video-topic '/camera/color/image_raw' --export-dir '$ANALYSIS_DIR'" || true
echo ""

# ============================================================
# Step 2: Depth video (fast SQLite direct export)
# ============================================================
echo "[Step 2/6] Exporting depth sensor video..."
$RUN_CMD "source /opt/ros/humble/setup.bash && python3 /home/uas/scripts/analysis/export_depth_video.py '$BAG_PATH' --every 3 -o '$ANALYSIS_DIR/depth_sensor_view.mp4'" || true
echo ""

# ============================================================
# Step 3: Flight trajectory plot
# ============================================================
echo "[Step 3/6] Generating trajectory plots..."
if $RUN_CMD "test -f '$ANALYSIS_DIR/local_position.csv'"; then
    $RUN_CMD "source /opt/ros/humble/setup.bash && python3 /home/uas/scripts/analysis/plot_trajectory.py '$ANALYSIS_DIR/local_position.csv' '$ANALYSIS_DIR/trajectory.png'" || true
else
    echo "[Warning] local_position.csv not found, skipping trajectory plot"
fi
echo ""

# ============================================================
# Step 4: OctoMap point cloud export (while sim is still running)
# ============================================================
echo "[Step 4/6] Capturing OctoMap point cloud..."
$RUN_CMD "source /opt/ros/humble/setup.bash && timeout 30 python3 /home/uas/scripts/analysis/octomap_to_ply.py '$ANALYSIS_DIR/octomap_world.ply'" || true
echo ""

# ============================================================
# Step 5: Fused RGB+Depth world cloud
# ============================================================
echo "[Step 5/6] Fusing RGB+Depth+Pose into colored world cloud..."
$RUN_CMD "source /opt/ros/humble/setup.bash && export PYTHONPATH=\${PYTHONPATH:-}:/home/uas/scripts && python3 /home/uas/scripts/analysis/build_world_cloud.py '$BAG_PATH' '$ANALYSIS_DIR/world_cloud.ply' --stride 8" || true
echo ""

# ============================================================
# Step 6: Summary report
# ============================================================
echo "[Step 6/6] Generating summary..."
$RUN_CMD "cat > '$ANALYSIS_DIR/summary.txt' << 'SUMMARY_EOF'
=== MAZE NAVIGATION TEST SUMMARY ===
Date: $(date)
Bag: $BAG_PATH

Exported artifacts:
SUMMARY_EOF
for f in '$ANALYSIS_DIR'/*; do
    if [ -f \"\$f\" ]; then
        sz=\$(du -h \"\$f\" | cut -f1)
        echo \"  \$(basename \$f)  (\$sz)\" >> '$ANALYSIS_DIR/summary.txt'
    fi
done
" || true

echo ""
echo "============================================="
echo "  ANALYSIS COMPLETE"
echo "============================================="
echo ""
echo "Artifacts in: $ANALYSIS_DIR"
echo ""

# List exported files
$RUN_CMD "ls -lh '$ANALYSIS_DIR' 2>/dev/null || echo '(directory listing failed)'" || true
echo ""

# If we have trajectory data, print some stats
if $RUN_CMD "test -f '$ANALYSIS_DIR/local_position.csv'" 2>/dev/null; then
    echo "--- Trajectory Stats ---"
    $RUN_CMD "tail -n +2 '$ANALYSIS_DIR/local_position.csv' | awk -F, 'BEGIN{min_x=999;max_x=-999;min_y=999;max_y=-999;min_z=999;max_z=-999} {x=\$2;y=\$3;z=\$4; if(x<min_x)min_x=x; if(x>max_x)max_x=x; if(y<min_y)min_y=y; if(y>max_y)max_y=y; if(z<min_z)min_z=z; if(z>max_z)max_z=z} END{print \"X range: \" min_x \" to \" max_x \" m\"; print \"Y range: \" min_y \" to \" max_y \" m\"; print \"Z range: \" min_z \" to \" max_z \" m\"; print \"Y span (maze exit=7m): \" max_y \" m\"; if(max_y>4.75) print \"*** DRONE EXITED THE MAZE (y > 4.75) ***\"; else print \"*** DRONE DID NOT EXIT (y <= 4.75) ***\"}}'" || true
fi
