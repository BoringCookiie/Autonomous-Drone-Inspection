# Setup Instructions

The repository uses the migrated `droneit` simulation stack as its runtime foundation. Run Docker Compose from the repository root with the explicit compose file below.

## Prerequisites

- Ubuntu 22.04 or a Linux host capable of running Docker host networking.
- Docker Engine and Compose plugin.
- `tmux` on the host for the default launcher.
- X11 access for the graphical simulation, or the headless/noVNC Compose variants.

## Build And Start

```bash
cd Autonomous-Drone-Inspection
docker compose --project-directory . -f docker/docker-compose.yml --profile sim build sim_stack
docker compose --project-directory . -f docker/docker-compose.yml --profile sim up -d --force-recreate sim_stack
docker exec -it uas_sim bash -lc '/home/uas/docker/bootstrap_px4.sh'
```

The bootstrap clones PX4 `v1.15.2`, initializes its submodules, copies root-level `.sdf` worlds from `gazebo_simulation/worlds`, and builds `gz_x500`.

For a GUI session, allow local X11 access before starting the container:

```bash
xhost +local:
```

## Flight Stack

Start the migrated PX4, MAVROS, camera bridge, and fixed OFFBOARD flight pattern:

```bash
./docker/launch_obstacle_stack.sh
```

The default model is `gz_x500_mono_cam`. Use the depth-equipped model for inspection:

```bash
PX4_GZ_WORLD=earthen_heritage_wall \
PX4_GZ_MODEL_TARGET=gz_x500_depth \
./docker/launch_obstacle_stack.sh
```

The launcher creates tmux windows for PX4/Gazebo, MAVROS, camera bridging, and flight. Use `--no-attach` to leave the session detached or `--no-fly` to record without running the fixed flight script.

## ROS Package

The source is mounted into `/home/uas/ros2_ws/src` inside the simulation container:

```bash
```

Launch the inspection nodes after the workspace is built:

```bash
  'source /opt/ros/humble/setup.bash && \
   source /home/uas/ros2_ws/install/setup.bash && \
   ros2 launch uas_earthen_inspection inspection_pipeline.launch.py \
   detector_backend:=yolo flight_strategy:=revisit'
```

The pipeline includes a MAVROS setpoint/pose adapter that publishes `/uav/waypoint_reached`, the per-waypoint capture node, the detector, and the revisit generator. The migrated A* planner subscribes to `/planner/revisit_waypoints`.

## End-To-End Orchestration

```bash
./run_autonomous_navigation.sh 8.0 0.5 1.5
```

This starts the depth simulation, MAVROS, recording, RTAB-Map/OctoMap/navigation, builds the inspection package, launches the inspection pipeline, and sends a navigation goal.

## Data And Models

Place datasets in `data/`, trained weights in `models/yolo/`, and cached CLIP embeddings in `models/embeddings/`. The repository currently contains manifests and placeholders only. Do not interpret the committed example results as experimental measurements.

## Diagnostics

```bash
./health_check.sh
python3 scripts/analyze_rosbags.py --bag list
python3 scripts/analyze_rosbags.py --bag latest --export-csv --export-video
```

The original Docker/Gazebo troubleshooting notes are retained in `docs/legacy/`.
