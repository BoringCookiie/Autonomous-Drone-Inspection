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

The complete orchestration is available through:

```bash
./run_autonomous_navigation.sh 8.0 0.5 1.5
```

This launches the depth simulation, MAVROS, recording, RTAB-Map, OctoMap, planner, follower, and inspection package before sending a goal. The depth simulation currently requires separate control/telemetry validation; use the mono-camera launcher for the stable camera-flight baseline.

Manual navigation commands:

```bash
docker exec uas_sim bash -lc 'source /opt/ros/humble/setup.bash; export PYTHONPATH=/home/uas/scripts:$PYTHONPATH; bash /home/uas/scripts/navigation/run_navigation.sh'
docker exec uas_sim bash -lc 'source /opt/ros/humble/setup.bash; python3 /home/uas/scripts/navigation/send_goal.py 5.0 2.0 1.5'
```

## Health And Shutdown

```bash
./health_check.sh
```

Stop the tmux processes and container:

```bash
tmux kill-session -t uas_obstacle 2>/dev/null || true
docker compose --project-directory . -f docker/docker-compose.yml --profile sim down
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
