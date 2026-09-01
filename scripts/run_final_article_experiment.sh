#!/bin/bash
# run_final_article_experiment.sh
# Master script to launch the full pipeline for the final article.
# This script orchestrates the Gazebo simulation (with the custom GLB map),
# the PX4/MAVROS navigation stack, and the AI inspection pipeline (YOLOv11s)
# leveraging the NVIDIA GPU for both rendering and inference.
set -e

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# ==============================================================================
# CONFIGURATION INSTRUCTIONS
# ==============================================================================
echo "============================================================"
echo "  FINAL ARTICLE EXPERIMENT PIPELINE (YOLOv11s + Gazebo)"
echo "============================================================"
echo ""

# 1. YOLO Weights check
YOLO_WEIGHTS="$ROOT_DIR/models/yolo/yolo_earthen_v11.pt"
if [ ! -f "$YOLO_WEIGHTS" ]; then
    echo "[ERROR] YOLO weights not found!"
    echo "-> Please place the trained YOLOv11s weights file at:"
    echo "   $YOLO_WEIGHTS"
    exit 1
fi

# 2. Custom GLB Map check
CUSTOM_MAP="$ROOT_DIR/gazebo_simulation/worlds/custom_map.glb"
if [ ! -f "$CUSTOM_MAP" ]; then
    echo "[ERROR] Custom GLB map not found!"
    echo "-> Please place your custom GLB map containing the houses at:"
    echo "   $CUSTOM_MAP"
    exit 1
fi

echo "[OK] Prerequisites found."
echo "[INFO] Running on NVIDIA GPU configuration."
echo ""

# ==============================================================================
# 1. LAUNCH SIMULATION & AI CONTAINERS
# ==============================================================================
echo "[1/4] Starting Gazebo Simulation and AI containers (NVIDIA GPU enabled)..."
# Set environment for Gazebo to load our custom SDF
export PX4_GZ_WORLD=custom_inspection
export PX4_GZ_MODEL_TARGET=gz_x500_depth
export CONTAINER="uas_sim_headless"
export AI_CONTAINER="uas_ai_gpu"

# Start the headless simulation and the GPU AI stack
docker compose --project-directory "$ROOT_DIR" -f "$ROOT_DIR/docker/docker-compose.yml" \
    --profile sim-headless --profile ai-gpu up -d

echo "Waiting 15 seconds for containers to initialize..."
sleep 15

# ==============================================================================
# 2. BOOTSTRAP PX4 & NAVIGATION STACK
# ==============================================================================
echo "[2/4] Bootstrapping PX4 and Navigation Stack..."
# The launch_obstacle_stack.sh script correctly starts MAVROS, octomap_server,
# planner_3d, and the camera bridge. We will use a customized headless launch.
docker exec -i "$CONTAINER" bash -c '
    source /opt/ros/humble/setup.bash
    bash /home/uas/scripts/navigation/run_maze_mission_v2.sh &
'
echo "Waiting 10 seconds for navigation nodes to settle..."
sleep 10

# ==============================================================================
# 3. LAUNCH AI INSPECTION PIPELINE (YOLOv11s)
# ==============================================================================
echo "[3/4] Starting the Inspection Pipeline (YOLOv11s) on NVIDIA GPU..."
docker exec -d "$AI_CONTAINER" bash -c '
    source /opt/ros/humble/setup.bash
    source /home/uas/ros2_ws/install/setup.bash
    ros2 launch uas_earthen_inspection inspection_pipeline.launch.py \
        detector_backend:=yolo \
        flight_strategy:=revisit \
        > /home/uas/rosbags/inspection_pipeline.log 2>&1
'
echo "Waiting 5 seconds for YOLOv11s to load weights..."
sleep 5

# ==============================================================================
# 4. EXECUTE FLIGHT MISSION
# ==============================================================================
echo "[4/4] Sending initial coverage waypoints to the drone..."
# The coverage_planner.py or a series of send_goal.py commands would go here.
# For demonstration, we send a sequence of waypoints to cover the houses.
docker exec -i "$CONTAINER" bash -c '
    source /opt/ros/humble/setup.bash
    echo "Sending Goal 1..."
    python3 /home/uas/scripts/navigation/send_goal.py 10.0 0.0 2.5
    echo "Sending Goal 2..."
    python3 /home/uas/scripts/navigation/send_goal.py 10.0 10.0 2.5
    echo "Sending Goal 3..."
    python3 /home/uas/scripts/navigation/send_goal.py 0.0 10.0 2.5
    echo "Sending Return to Launch (Goal 4)..."
    python3 /home/uas/scripts/navigation/send_goal.py 0.0 0.0 2.5
'

# ==============================================================================
# 5. GENERATE FINAL ARTICLE METRICS & GRAPHS
# ==============================================================================
echo "============================================================"
echo "  MISSION COMPLETE"
echo "============================================================"
echo "Generating metrics and graphs for the final article..."

# Stop the simulation and rosbag recording cleanly
bash "$ROOT_DIR/scripts/simulation/stop_simulation.sh"

# Find the latest rosbag directory
BAG_DIR=$(ls -td "$ROOT_DIR"/rosbags/*/ | head -n 1)
echo "Analyzing Rosbag: $BAG_DIR"

if [ -n "$BAG_DIR" ]; then
    # Run the comprehensive analysis script (which outputs trajectory plots, 
    # CSV metrics, and bounding box visualizations for YOLO)
    docker compose --project-directory "$ROOT_DIR" -f "$ROOT_DIR/docker/docker-compose.yml" \
        run --rm ai_stack_gpu bash -c "
        source /opt/ros/humble/setup.bash && \
        python3 /home/uas/scripts/analysis/analyze_rosbags.py \
            --bag /home/uas/rosbags/$(basename "$BAG_DIR") \
            --export-csv --export-video \
            --video-topic /camera/color/image_raw && \
        python3 /home/uas/scripts/analysis/plot_trajectory.py \
            /home/uas/rosbags/$(basename "$BAG_DIR")/analysis/local_position.csv \
            /home/uas/rosbags/$(basename "$BAG_DIR")/analysis/trajectory.png
    "
    echo ""
    echo "All graphs, CSV metrics, and video outputs have been successfully exported to:"
    echo "-> $BAG_DIR/analysis/"
else
    echo "[ERROR] No rosbag found to analyze."
fi

echo "Done."
