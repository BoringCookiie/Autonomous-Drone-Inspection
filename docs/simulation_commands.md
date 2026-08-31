# Simulation Command Reference

All commands below are run from the `Autonomous-Drone-Inspection/` repository root.

## Host Preparation

```bash
cd Autonomous-Drone-Inspection
xhost +local:
```

Use native Docker Engine with `/dev/dri` available for the GUI profile. Use the headless or noVNC profiles when host X11 is unavailable.

## Build And Start

```bash
docker compose --project-directory . -f docker/docker-compose.yml --profile sim build sim_stack
docker compose --project-directory . -f docker/docker-compose.yml --profile sim up -d --force-recreate sim_stack
```

Bootstrap PX4 once per PX4 volume:

```bash
docker exec -it uas_sim bash -lc '/home/uas/docker/bootstrap_px4.sh'
```

Build the ROS2 inspection package:

```bash
docker exec uas_sim bash -lc '/home/uas/scripts/build_workspace.sh'
```

## Launch The Validated Flight

The default is the validated moving RGB camera model:

```bash
./docker/launch_obstacle_stack.sh --gui
```

Run detached from tmux:

```bash
./docker/launch_obstacle_stack.sh --gui --no-attach
```

Use the explicit RGB-D model:

```bash
PX4_GZ_MODEL_TARGET=gz_x500_depth ./docker/launch_obstacle_stack.sh --gui
```

Use another world:

```bash
PX4_GZ_WORLD=obstacle_maze \
PX4_GZ_MODEL_TARGET=gz_x500_mono_cam \
./docker/launch_obstacle_stack.sh --gui
```

Record without running the fixed flight script:

```bash
./docker/launch_obstacle_stack.sh --gui --no-fly
```

Inspect tmux:

```bash
tmux list-windows -t uas_obstacle
```

Inside tmux, windows are `0=px4_gz`, `1=mavros`, `2=camera_bridge`, and `3=fly` when the flight process is active. Detach with `Ctrl-b d`.

## Camera Diagnostics

ROS camera topics:

```bash
docker exec uas_sim bash -lc 'source /opt/ros/humble/setup.bash; ros2 topic hz /camera/color/image_raw'
docker exec uas_sim bash -lc 'source /opt/ros/humble/setup.bash; ros2 topic info -v /camera/color/image_raw'
```

Gazebo camera topics:

```bash
docker exec uas_sim bash -lc 'source /opt/ros/humble/setup.bash; gz topic -l | grep -Ei "camera|image|depth|points"'
```

Canonical topics are:

```text
/camera/color/image_raw
/camera/color/camera_info
/camera/depth/image_raw       # depth model only
```

The bridge discovers model-specific Gazebo names. Do not record or build AI nodes around raw `/camera`, `/camera_info`, or `/rgbd_camera/...` names.

## Rosbag Analysis And Video Export

List bags:

```bash
docker exec uas_sim bash -lc 'source /opt/ros/humble/setup.bash; python3 /home/uas/scripts/analysis/analyze_rosbags.py --bag list'
```

Export the latest flight:

```bash
docker exec uas_sim bash -lc 'source /opt/ros/humble/setup.bash; \
python3 /home/uas/scripts/analysis/analyze_rosbags.py \
  --bag latest \
  --export-csv \
  --export-video \
  --video-topic /camera/color/image_raw'
```

The output is written to:

```text
rosbags/<bag-name>/analysis/camera_color_image_raw.mp4
rosbags/<bag-name>/analysis/local_position.csv
rosbags/<bag-name>/analysis/velocity_local.csv
rosbags/<bag-name>/analysis/imu.csv
rosbags/<bag-name>/analysis/gps.csv
rosbags/<bag-name>/analysis/state.csv
```

The exporter fails when a moving flight has no decodable RGB frames or every decoded frame is identical.

### Depth Sensor-View Video (fast path)

Exports the depth stream as a Turbo-colormapped H.264 video. This reads the
bag's SQLite database directly and parses the Image CDR wire format with
numpy — roughly 100x faster than the generic analyzer for this topic, and it
decodes its own output before declaring success:

```bash
docker exec uas_sim bash -lc 'source /opt/ros/humble/setup.bash; \
python3 /home/uas/scripts/analysis/export_depth_video.py \
  /home/uas/rosbags/<bag-name> --every 3'
```

Output: `rosbags/<bag-name>/analysis/depth_sensor_view.mp4`.
Options: `--fps 15`, `--every 3` (frame decimation), `--colormap jet`,
`-o custom.mp4`.

Requires `ffmpeg` inside the container (installed via the Dockerfile; on
older containers run `sudo apt-get install -y ffmpeg` once).

### 3D World Point Cloud (OctoMap export)

While the simulation and the navigation stack are running, OctoMap fuses the
depth stream into a filtered voxel map. Export that finished map as a colored
binary PLY readable by CloudCompare, MeshLab, Open3D, or Blender:

```bash
docker exec uas_sim bash -lc 'source /opt/ros/humble/setup.bash; \
python3 /home/uas/scripts/analysis/octomap_to_ply.py /home/uas/rosbags/octomap_world.ply'
```

Optional: `--timeout 30` (seconds to wait for a message), `--topic T`.

### Flight Trajectory Plot

After `--export-csv`, plot top-down track, altitude profile, and X/Y vs time:

```bash
docker exec uas_sim bash -lc 'source /opt/ros/humble/setup.bash; \
python3 /home/uas/scripts/analysis/plot_trajectory.py \
  /home/uas/rosbags/<bag-name>/analysis/local_position.csv \
  /home/uas/rosbags/<bag-name>/analysis/trajectory.png'
```

### Offline Depth+RGB+Pose Fusion (advanced, slow)

`build_world_cloud.py` fuses raw depth, RGB and poses from any bag into an
RGB-colored PLY. It streams with bounded memory (~100 MB), but prefer the
OctoMap export above for large bags — it is faster and produces a cleaner map:

```bash
docker exec uas_sim bash -lc 'source /opt/ros/humble/setup.bash; \
python3 /home/uas/scripts/analysis/build_world_cloud.py \
  /home/uas/rosbags/<bag-name> world_cloud.ply --stride 8'
```

Inspect a specific bag:

```bash
docker exec uas_sim bash -lc 'source /opt/ros/humble/setup.bash; \
python3 /home/uas/scripts/analysis/analyze_rosbags.py \
  --bag /home/uas/rosbags/fly_pattern_YYYYMMDD_HHMMSS \
  --export-csv --export-video \
  --video-topic /camera/color/image_raw'
```

## ROS2 Inspection Pipeline

Build the workspace first:

```bash
docker exec uas_sim bash -lc '/home/uas/scripts/build_workspace.sh'
```

Launch YOLO mode:

```bash
docker exec uas_sim bash -lc 'source /opt/ros/humble/setup.bash; \
source /home/uas/ros2_ws/install/setup.bash; \
ros2 launch uas_earthen_inspection inspection_pipeline.launch.py \
detector_backend:=yolo flight_strategy:=single_pass'
```

Launch revisit mode:

```bash
docker exec uas_sim bash -lc 'source /opt/ros/humble/setup.bash; \
source /home/uas/ros2_ws/install/setup.bash; \
ros2 launch uas_earthen_inspection inspection_pipeline.launch.py \
detector_backend:=yolo flight_strategy:=revisit'
```

The raw and RAG VLM backend names are accepted by the interface, but the current `uas_sim` image does not contain Transformers, BitsAndBytes, or Accelerate. Install and validate the dedicated AI/VLM environment before treating those backends as real inference.

Inspect AI topics:

```bash
docker exec uas_sim bash -lc 'source /opt/ros/humble/setup.bash; ros2 topic echo /inspection/detections'
docker exec uas_sim bash -lc 'source /opt/ros/humble/setup.bash; ros2 topic echo /inspection/captured_frame --once'
docker exec uas_sim bash -lc 'source /opt/ros/humble/setup.bash; ros2 topic echo /uav/waypoint_reached'
```

## YOLOv11 Training

The data YAML must exist and use class IDs consistent with `knowledge_base/defect_ontology.json`.

```bash
python3 scripts/train_yolov11.py \
  --data-yaml data/earthen_defects.yaml \
  --epochs 50 \
  --batch-size 16 \
  --output models/yolo/yolo_earthen_v11.pt
```

Training artifacts are written under `models/yolo/train_run/`. The detector runtime expects `models/yolo/yolo_earthen_v11.pt`.

## CLIP Knowledge Base Embeddings

The current checked-in ontology contains placeholders for reference crops. Populate the crop files before generating research embeddings. The current builder requires the AI/VLM dependencies and model download.

```bash
python3 scripts/build_clip_embeddings.py \
  --ontology knowledge_base/defect_ontology.json \
  --output models/embeddings/clip_kb_embeddings.pt
```

Do not use random fallback embeddings for reported experiments.

## Navigation Stack

Before starting a simulation, run the static preflight. It verifies the
world/model assets, required ROS dependencies, executable scripts, and that
no previous project runtime is still alive:

```bash
scripts/navigation/preflight_navigation.sh
```

At runtime, `navigation_launch.py` starts a communication gate. It checks
types, endpoint QoS, live rates, timestamps, TF, and OctoMap bounds. The
follower will not arm until `/navigation/preflight_ok` is true. Inspect it
with:

```bash
docker exec uas_sim bash -lc 'source /opt/ros/humble/setup.bash; ros2 topic echo /navigation/preflight_ok --once'
```

The complete orchestration is available through:

```bash
./run_autonomous_navigation.sh 8.0 0.5 1.5
```

The obstacle-maze mission uses a robust A* planner and MAVROS controller for navigating dynamic environments. To run the optimized v2 stack (with improved EKF stabilization, takeoff tracking, path smoothing, and zero-bloat curated rosbags):

```bash
bash scripts/navigation/run_maze_mission_v2.sh
```

This launches the depth simulation, MAVROS, recording, OctoMap, A* planner, and the maze_navigator before sending the goal `(4.0, 7.0, 1.5)`. You can view the full navigation proof, plotted trajectory, and extracted CSV metrics in the `results/` directory generated after each mission.

Manual navigation commands:

```bash
docker exec uas_sim bash -lc 'source /opt/ros/humble/setup.bash; export PYTHONPATH=/home/uas/scripts:$PYTHONPATH; bash /home/uas/scripts/navigation/run_navigation.sh'
docker exec uas_sim bash -lc 'source /opt/ros/humble/setup.bash; python3 /home/uas/scripts/navigation/send_goal.py 5.0 2.0 1.5'
```

## Health And Shutdown

```bash
./health_check.sh
```

Stop a run cleanly (kills tmux session, stops rosbag2 with SIGINT so
metadata.yaml is finalized, then removes every project-owned process —
PX4, Gazebo, MAVROS, bridges, nav nodes — without touching unrelated ROS
commands):

```bash
scripts/simulation/stop_simulation.sh            # runtime only
scripts/simulation/stop_simulation.sh --down     # also `compose down`
```

`launch_obstacle_stack.sh` runs the same cleanup automatically before
starting, so stale orphans from previous runs cannot accumulate. The
curated recorder (`--no-fly` window) records an explicit topic list
including `/clock`; never use bare `ros2 bag record -a` here — raw
MAVLink streams alone produced a 105 GB bag.

Stop only the recorders (e.g. before analyzing while leaving the sim up):

```bash
docker exec uas_sim bash /home/uas/scripts/simulation/cleanup_runtime.sh --recorders-only
```

If stale PX4/Gazebo processes remain:

```bash
docker restart uas_sim
```

## Important Environment Variables

```text
UAS_SIM_CONTAINER=uas_sim
PX4_GZ_WORLD=earthen_heritage_wall
PX4_GZ_MODEL_TARGET=gz_x500_mono_cam
HEADLESS=1
MAVROS_NS=/uas1
FLY_PATTERN_RECORD_BAG=1
FLY_PATTERN_BAG_DIR=/home/uas/rosbags
FLY_PATTERN_REQUIRE_DEPTH=1
FLY_PATTERN_RECORD_DEPTH=1
FLY_PATTERN_SKIP_PARAM=1
```

The camera readiness check must remain enabled. A flight without a live canonical RGB stream must not be used for AI or video experiments.
