# Autonomous UAV Inspection of Earthen Heritage Architecture

[![ROS2](https://img.shields.io/badge/ROS2-Humble%2F%20Jazzy-blue.svg)](https://docs.ros.org/)
[![PX4](https://img.shields.io/badge/PX4-Autopilot-red.svg)](https://px4.io/)
[![PyTorch](https://img.shields.io/badge/PyTorch-2.0%2B-orange.svg)](https://pytorch.org/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

This repository is the unified successor to the original `droneit` PX4/Gazebo prototype. It preserves the validated flight, camera-bridge, rosbag, and navigation tools while adding the earthen-wall inspection package on top.

The operational path is:

```text
PX4 SITL/Gazebo -> MAVROS -> camera bridge -> waypoint trigger
    -> RGB capture -> detector -> ambiguous detection
    -> revisit PoseArray -> migrated A* planner
```

The VLM and trained-model backends are selectable, but require the corresponding model/data assets. Without those assets the package runs its explicit structural fallback detectors; fallback output must not be used as research results.

---

## 🏛️ Project Architecture Overview

```
                        +---------------------------------------+
                        |  1. Decoupled Waypoint Capture Node   |
                        |   (Captures RGB & Depth @ Waypoint)   |
                        +-------------------+-------------------+
                                            |
                                            v
                        +-------------------+-------------------+
                        |       2. AI Detection Node            |
                        |  [Raw VLM | RAG VLM | YOLOv11]        |
                        |  Outputs: (BBox, Class, Confidence C) |
                        +-------------------+-------------------+
                                            |
                                            v
                   +------------------------+------------------------+
                   |  Confidence Evaluation: C in [0.4, 0.7]?        |
                   +------------------------+------------------------+
                             /                            \
                      YES   /                              \   NO
                           v                                v
         +----------------------------------+    +--------------------------+
         | 3. Revisit Waypoint Generator    |    | Continue Standard        |
         |  • 3D BBox Projection            |    | Coverage Waypoints       |
         |  • Dynamic Standoff Calculation  |    +--------------------------+
         |  • Revisit Waypoint Queue to A*  |
         +----------------------------------+
```

---

## 👥 3-Person Team Roles & Structure

- **Person 1 (AI / VLM Lead)**: Zero-shot VLM integration (`raw_vlm`, `rag_vlm`), CLIP knowledge base embedding generation, prompt optimization, VLM benchmark script (`scripts/eval_vlm_benchmark.py`).
- **Person 2 (YOLO / Data Lead)**: Dataset curation (`SDNET2018`, `MBDD2025`, earthen-augmented sets), YOLOv11 model training (`scripts/train_yolov11.py`), metrics logging.
- **Person 3 (UAV / ROS2 & Sim Lead)**: Gazebo simulation environment setup (`gazebo_simulation/`), confidence-triggered revisit generator (`revisit_waypoint_generator.py`), A* path planner integration, PX4 SITL orchestration.

---

## Repository Directory Structure

```
Autonomous-Drone-Inspection/
├── docker/                 # Unified PX4/Gazebo/ROS2 image and Compose stack
├── docs/                   # Current documentation and migrated legacy notes
├── data/                   # Dataset locations and evaluation assets
├── gazebo_simulation/      # PX4 worlds, earthen facade, and legacy obstacle worlds
├── knowledge_base/         # Defect taxonomy JSON, prompts YAML & reference crops
├── models/                 # Cached CLIP embeddings & trained YOLO weights
├── results/                # 6-condition sweep logs, Table 4 summaries & Figure 6 plots
├── rosbags/                # Flight bags, including migrated legacy evidence
├── scripts/
│   ├── simulation/         # PX4 launch, flight, camera bridge, curated recorder, cleanup tools
│   ├── navigation/         # RTAB-Map, OctoMap, A*, follower, coverage planner, TF bridge
│   └── analysis/           # Bag analyzer, depth/PLY/trajectory exporters
└── src/
    └── uas_earthen_inspection/   # Main ROS2 Python package
        ├── config/               # Parameters configuration (ambiguity thresholds, standoff math)
        ├── launch/               # Parameterized launch files (3x2 matrix evaluation)
        └── uas_earthen_inspection/ # Python ROS2 node sources
```

---

## 🚀 Final Article Experiment Guide (YOLOv11s on Custom GLB)

**This section is the absolute single source of truth for executing the final article experiment combining the custom earthen housing GLB map, the fully trained YOLOv11s model, and the A* confidence-revisit flight strategy on an NVIDIA GPU.**

### 1. Prerequisite: NVIDIA GPU & Container Toolkit
The experiment is orchestrated using Docker Compose to isolate the physics simulation from the AI inference stack. However, since the YOLOv11s model requires sub-10ms inference times (and Gazebo requires heavy 3D acceleration), the host machine *must* have an NVIDIA GPU.

Ensure the host Ubuntu machine has the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) installed. If you can run `nvidia-smi` inside a docker container, you are ready.

### 2. Required Assets Placement
Before running the master script, you must place the trained weights and the custom map exactly in the following directories (the script will refuse to run without them):

*   **YOLOv11s Weights:**
    Place the `.pt` file containing the weights trained by the Data Lead here:
    `models/yolo/yolo_earthen_v11.pt`
    *(The YOLO detection node will automatically lazy-load this file into the AI container's VRAM).*

*   **Custom Houses GLB Map:**
    Place the `.glb` 3D map containing the earthen houses and cracks here:
    `gazebo_simulation/worlds/custom_map.glb`
    *(The simulation uses a custom `custom_inspection.sdf` Gazebo world which is hardcoded to dynamically import this specific `.glb` file at origin `(0,0,0)`.)*

### 3. Execution Orchestration
Once the assets are in place, the entire pipeline is automated. Run this master script from the repository root:

```bash
bash scripts/run_final_article_experiment.sh
```

**What this script does autonomously:**
1.  **Container Provisioning:** Launches the `uas_sim_headless` container (for Gazebo/PX4) and the `uas_ai_gpu` container (with `runtime: nvidia` for PyTorch/YOLO inference).
2.  **Simulation & Navigation:** Boots PX4, OctoMap, and the A* planner targeting the `custom_inspection` world.
3.  **AI Pipeline:** Starts the ROS2 inspection pipeline (`inspection_pipeline.launch.py`) configured strictly with `detector_backend:=yolo` and `flight_strategy:=revisit`.
4.  **Coverage Flight:** Dispatch a sequence of waypoints covering the map. The revisit-generator will automatically interrupt the planner to zoom in on any ambiguous cracks detected by YOLO.
5.  **Metrics Export:** When the drone completes the mission, the script safely halts recording, kills the simulation, and runs the analyzer.

### 4. Retrieving Your Data
At the end of the script execution, look inside the `rosbags/` directory. The script will export all the necessary metrics and figures for the final article directly into the analysis subfolder of the latest run, including:
*   `trajectory.png` (Top-down visual plot of the flight path and revisits)
*   `local_position.csv` & `velocity_local.csv`
*   Extracted annotated RGB video highlighting YOLO's bounding boxes.

---

## Quick Start Guide

### 1. Build ROS2 Workspace
```bash
cd Autonomous-Drone-Inspection
docker compose --project-directory . -f docker/docker-compose.yml --profile sim build sim_stack
docker compose --project-directory . -f docker/docker-compose.yml --profile sim up -d --force-recreate sim_stack
docker exec -it uas_sim bash -lc '/home/uas/docker/bootstrap_px4.sh'
```

### 2. Build the ROS workspace
```bash
docker exec -it uas_sim bash -lc '/home/uas/scripts/build_workspace.sh'
```

### 3. Run the migrated flight and inspection stack
```bash
./docker/launch_obstacle_stack.sh --no-attach
```

For the complete navigation orchestration, use:
```bash
./run_autonomous_navigation.sh 8.0 0.5 1.5
```

### 4. Pre-compute CLIP knowledge-base embeddings
```bash
python scripts/build_clip_embeddings.py \
    --ontology knowledge_base/defect_ontology.json \
    --output models/embeddings/clip_kb_embeddings.pt
```

This command requires the AI/VLM dependencies and a real CLIP model. Do not use the generated fallback tensors for reported experiments. See [`docs/simulation_commands.md`](docs/simulation_commands.md) for the current environment limitations.

### 5. Launch the inspection pipeline independently
```bash
# Example: launching the currently runnable YOLO interface with revisit mode
docker exec uas_sim bash -lc 'source /opt/ros/humble/setup.bash && \
source /home/uas/ros2_ws/install/setup.bash && \
ros2 launch uas_earthen_inspection inspection_pipeline.launch.py \
detector_backend:=yolo flight_strategy:=revisit'
```

The `raw_vlm` and `rag_vlm` interface names are present, but real Qwen/CLIP inference requires the dedicated AI/VLM environment described in the integration guide.

### 6. Run the evaluation tooling
```bash
python scripts/run_6condition_sweep.py --mock --output-dir results/sweeps
python scripts/generate_table4_figure6.py --input-dir results/sweeps --output-dir results/
```

`--mock` is only a smoke test. The real six-condition scorer still needs the hand-labeled evaluation set and detector inference implementation.

---

## 🎬 Flight Artifacts & Resource Lifecycle

Every run records an explicit curated topic list (`/clock`, camera RGB/depth,
MAVROS telemetry, planner topics) — never `ros2 bag record -a`. After a
flight, export the deliverables:

```bash
# RGB first-person view (validated; fails loudly if frames are missing/static)
docker exec uas_sim bash -lc 'source /opt/ros/humble/setup.bash; \
python3 /home/uas/scripts/analysis/analyze_rosbags.py --bag latest \
  --export-csv --export-video --video-topic /camera/color/image_raw'

# Depth sensor-view video (fast SQLite+CDR path, self-verifying)
docker exec uas_sim bash -lc 'source /opt/ros/humble/setup.bash; \
python3 /home/uas/scripts/analysis/export_depth_video.py /home/uas/rosbags/<bag> --every 3'

# 3D world cloud from OctoMap -> PLY (CloudCompare/MeshLab/Open3D)
docker exec uas_sim bash -lc 'source /opt/ros/humble/setup.bash; \
python3 /home/uas/scripts/analysis/octomap_to_ply.py /home/uas/rosbags/octomap_world.ply'

# Trajectory plot from exported CSVs
docker exec uas_sim bash -lc 'source /opt/ros/humble/setup.bash; \
python3 /home/uas/scripts/analysis/plot_trajectory.py \
  /home/uas/rosbags/<bag>/analysis/local_position.csv \
  /home/uas/rosbags/<bag>/analysis/trajectory.png'
```

Stop a run cleanly between sessions (kills tmux, finalizes rosbag metadata,
removes every project-owned process — PX4/Gazebo/MAVROS/bridges/nav nodes —
without touching unrelated ROS commands):

```bash
scripts/simulation/stop_simulation.sh            # runtime only
scripts/simulation/stop_simulation.sh --down     # also `compose down`
```

`launch_obstacle_stack.sh` runs the same cleanup automatically before
starting. See [`docs/simulation_commands.md`](docs/simulation_commands.md)
for the full reference.

---

## Current Simulation Command Reference

The complete command reference for the validated PX4/Gazebo/MAVROS/camera setup is in [`docs/simulation_commands.md`](docs/simulation_commands.md). It covers:

- Container build, PX4 bootstrap, and ROS2 workspace build.
- GUI, headless, mono-camera, and RGB-D launches.
- Gazebo and canonical ROS camera diagnostics.
- Rosbag listing, CSV export, authoritative RGB MP4, depth sensor-view video,
  OctoMap-to-PLY world cloud, and trajectory plotting.
- Inspection pipeline launch for YOLO single-pass and revisit modes.
- YOLOv11 training and model placement.
- CLIP knowledge-base embedding generation.
- Navigation, health checks, clean shutdown, and orphan-process cleanup.

The teammate-facing architecture and integration contract for the Person 1 RAG-VLM work and the Person 2 YOLO work is documented in [`docs/person1_ai_vlm_yolo_integration.tex`](docs/person1_ai_vlm_yolo_integration.tex). The LaTeX file is an integration guide; model-dependent VLM commands must not be treated as runnable in the simulation image until the dedicated AI dependencies are installed.

---

## Important Status

The migrated PX4/MAVROS flight path and canonical RGB camera export are validated in the mono-camera facade flight. Depth delivery is validated with `gz_x500_depth`; defect inference, dataset preparation, and six-condition metrics still require real model/data assets and research validation. Existing result files are generated examples, not measured experiments.

## License

Developed for Autonomous Earthen Heritage Inspection research. MIT License.
